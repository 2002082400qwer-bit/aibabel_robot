#include "flash.h"

#define FlashCalibrationFlashAddress 0x0803F800 // FT32F103VC最后一个2KB页，链接区已预留。
#define FlashCalibrationWordCount 9 // 校准记录的32位字数量，共36字节。
#define FlashCalibrationMagic 0x4D43414C // 记录成功标记，最后写入以防掉电误用。
#define FlashCalibrationVersion 0x00010000 // 记录版本1，低三位保存启用电机组合。
#define FlashPositionFlashAddress 0x0803F000 // 停稳位置日志独占的2KB页。
#define FlashPositionSlotCount 102 // 每条20字节，页尾8字节不用。
#define FlashPositionWordCount 5 // 标记、版本组合、两路坐标和CRC。
#define FlashPositionMagic 0x4D504F53 // 可信停稳位置成功标记，最后提交。
#define FlashPositionVersion 0x00010000 // 位置日志格式版本1。
#define FlashCalibrationCrcPolynomial 0xEDB88320 // CRC32的反射多项式。

/** 函数作用：初始化Flash存储访问状态并锁定写保护，不擦除已有记录；无传入参数。 */
void FlashInit(void)
{
    FLASH_Lock();
}

/**
 * 函数作用：从预留Flash页读取校准记录，不解锁或擦写Flash。
 * record：接收9个32位字的输出缓冲区。
 */
static void FlashCalibrationReadRecord(u32 *record)
{
    u8 word; // 当前读取的32位字序号。

    for( word = 0; word < FlashCalibrationWordCount; word++ ) {
        record[word] = *(__IO u32 *)(FlashCalibrationFlashAddress + (u32)word * 4);
    }
}

/**
 * 函数作用：计算校准数据的CRC32，覆盖版本、行程和换算系数。
 * record：含9个32位字的记录，首字标记和末字CRC不参与计算。
 * 返回值：数据校验值。
 */
static u32 FlashCalibrationChecksum(const u32 *record)
{
    u32 crc = 0xFFFFFFFF; // CRC计算的初始值。
    u8 word; // 当前数据字序号。
    u8 bit; // 当前数据字的位序号。

    for( word = 1; word < FlashCalibrationWordCount - 1; word++ ) {
        crc ^= record[word];
        for( bit = 0; bit < 32; bit++ ) {
            if( crc & 1 ) crc = (crc >> 1) ^ FlashCalibrationCrcPolynomial;
            else crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}

/**
 * 函数作用：核对持久化记录的成功标记、版本、电机组合和CRC。
 * record：待校验的9字记录；motorMask：本次启动启用的电机组合。
 * 返回值：1表示记录完整有效，0表示未完成、损坏或不兼容。
 */
static u8 FlashCalibrationValidate(const u32 *record, u8 motorMask)
{
    if( (motorMask == 0) || (motorMask > 7) ) return 0;
    if( record[0] != FlashCalibrationMagic ) return 0;
    if( record[1] != (FlashCalibrationVersion + (u32)motorMask) ) return 0;
    if( record[8] != FlashCalibrationChecksum(record) ) return 0;
    return 1;
}

/**
 * 函数作用：读取并校验持久化记录，成功后返回三路校准数据。
 * data：输出数据指针，失败时内容不变；motorMask：启用电机组合，范围1至7。
 * 返回值：1表示读取成功，0表示记录无效或参数错误。
 */
u8 FlashCalibrationRead(FlashCalibrationData *data, u8 motorMask)
{
    u32 record[FlashCalibrationWordCount]; // 从预留页读出的校准记录。
    u8 axis; // 当前输出的电机序号。

    if( (data == 0) || (motorMask == 0) || (motorMask > 7) ) return 0;
    FlashCalibrationReadRecord(record);
    if( FlashCalibrationValidate(record, motorMask) == 0 ) return 0;
    for( axis = 0; axis < FlashCalibrationMotorCount; axis++ ) {
        data->MaxPulseNum[axis] = record[2 + axis * 2];
        data->PulseScale[axis] = record[3 + axis * 2];
    }
    return 1;
}

/**
 * 函数作用：写入校准记录，先写并读回数据，最后提交成功标记。
 * data：调用方已校验范围的校准数据；motorMask：启用电机组合，范围1至7。
 * 返回值：1表示保存和读回校验成功，0表示参数或擦写校验失败。
 * 所有电机停止后才调用；使用厂家驱动返回状态，任何擦写失败都会重新锁定Flash。
 */
u8 FlashCalibrationWrite(const FlashCalibrationData *data, u8 motorMask)
{
    u32 record[FlashCalibrationWordCount]; // 本次待保存的校准记录。
    u32 verify[FlashCalibrationWordCount]; // 写入后的读回校验记录。
    u8 word; // 当前初始化、写入或比较的字序号。
    u8 axis; // 当前打包校准数据的电机序号。

    if( (data == 0) || (motorMask == 0) || (motorMask > 7) ) return 0;
    for( word = 0; word < FlashCalibrationWordCount; word++ ) record[word] = 0;
    record[0] = FlashCalibrationMagic;
    record[1] = FlashCalibrationVersion + (u32)motorMask;
    for( axis = 0; axis < FlashCalibrationMotorCount; axis++ ) {
        record[2 + axis * 2] = data->MaxPulseNum[axis];
        record[3 + axis * 2] = data->PulseScale[axis];
    }
    record[8] = FlashCalibrationChecksum(record);

    if( FlashMotorPositionInvalidate() == 0 ) return 0;
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    if( FLASH_ErasePage(FlashCalibrationFlashAddress) != FLASH_COMPLETE ) {
        FLASH_Lock();
        return 0;
    }
    for( word = 1; word < FlashCalibrationWordCount; word++ ) {
        if( FLASH_ProgramWord(FlashCalibrationFlashAddress + (u32)word * 4, record[word]) != FLASH_COMPLETE ) {
            FLASH_Lock();
            return 0;
        }
    }
    FlashCalibrationReadRecord(verify);
    for( word = 1; word < FlashCalibrationWordCount; word++ ) {
        if( verify[word] != record[word] ) {
            FLASH_Lock();
            return 0;
        }
    }
    if( FLASH_ProgramWord(FlashCalibrationFlashAddress, FlashCalibrationMagic) != FLASH_COMPLETE ) {
        FLASH_Lock();
        return 0;
    }
    FLASH_Lock();
    FlashCalibrationReadRecord(verify);
    return FlashCalibrationValidate(verify, motorMask);
}

/**
 * 函数作用：读出指定槽位的原始位置记录，不擦写Flash。
 * slot：0至101的日志槽序号；record：接收5个字的缓冲区。
 */
static void FlashPositionReadRecord(u16 slot, u32 *record)
{
    u8 word; // 当前读取字的序号。
    for( word = 0; word < FlashPositionWordCount; word++ ) {
        record[word] = *(__IO u32 *)(FlashPositionFlashAddress + (u32)slot * 20 + (u32)word * 4);
    }
}

/**
 * 函数作用：扫描所有槽，返回最新占用槽的下一序号，0代表整页空白。
 * 无传入参数；未提交或已失效记录仍算占用，禁止回退旧记录。
 */
static u16 FlashPositionUsedSlots(void)
{
    u32 record[FlashPositionWordCount]; // 当前槽的内容。
    u16 slot; // 扫描槽序号。
    u16 used = 0; // 最新占用槽的下一序号。
    u8 word; // 当前检查字序号。
    for( slot = 0; slot < FlashPositionSlotCount; slot++ ) {
        FlashPositionReadRecord(slot, record);
        for( word = 0; word < FlashPositionWordCount; word++ ) {
            if( record[word] != 0xFFFFFFFF ) used = slot + 1;
        }
    }
    return used;
}

/**
 * 函数作用：计算位置记录CRC32，覆盖版本组合与两路脉冲坐标。
 * record：5字位置记录；返回32位CRC。
 */
static u32 FlashPositionChecksum(const u32 *record)
{
    u32 crc = 0xFFFFFFFF; // 初始校验值。
    u8 word; // 数据字序号。
    u8 bit; // 当前字位序号。
    for( word = 1; word < FlashPositionWordCount - 1; word++ ) {
        crc ^= record[word];
        for( bit = 0; bit < 32; bit++ ) {
            if( crc & 1 ) crc = (crc >> 1) ^ FlashCalibrationCrcPolynomial;
            else crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}

/**
 * 函数作用：校验位置记录标记、版本、启用组合和CRC。
 * record：5字记录；motorMask：启用组合1至7；返回1代表有效。
 */
static u8 FlashPositionValidate(const u32 *record, u8 motorMask)
{
    if( (motorMask == 0) || (motorMask > 7) ) return 0;
    if( record[0] != FlashPositionMagic ) return 0;
    if( record[1] != (FlashPositionVersion + (u32)motorMask) ) return 0;
    if( record[4] != FlashPositionChecksum(record) ) return 0;
    return 1;
}

/**
 * 函数作用：只读取最新占用槽的可信位置，最新无效时不回退旧槽。
 * data：输出坐标，失败保持不变；motorMask：启用组合1至7。
 * 返回1代表成功，0代表记录或参数无效。
 */
u8 FlashMotorPositionRead(FlashMotorPositionData *data, u8 motorMask)
{
    u32 record[FlashPositionWordCount]; // 最新原始记录。
    u16 used; // 最新占用槽的下一序号。
    if( (data == 0) || (motorMask == 0) || (motorMask > 7) ) return 0;
    used = FlashPositionUsedSlots();
    if( used == 0 ) return 0;
    FlashPositionReadRecord(used - 1, record);
    if( FlashPositionValidate(record, motorMask) == 0 ) return 0;
    data->Motor1Pulse = record[2];
    data->Motor3Pulse = record[3];
    return 1;
}

/**
 * 函数作用：运动前将最新可信标记写为0并读回，写失败不得启动电机。
 * 无传入参数；返回1代表无可信旧记录或失效成功，0代表失效失败。
 * 只清除Flash位而不擦页；调用方必须在第一路电机启动前调用。
 */
u8 FlashMotorPositionInvalidate(void)
{
    u32 record[FlashPositionWordCount]; // 最新记录内容。
    u32 address; // 待清除标记的地址。
    u16 used; // 最新占用槽的下一序号。
    used = FlashPositionUsedSlots();
    if( used == 0 ) return 1;
    FlashPositionReadRecord(used - 1, record);
    if( record[0] != FlashPositionMagic ) return 1;
    address = FlashPositionFlashAddress + (u32)(used - 1) * 20;
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    if( FLASH_ProgramWord(address, 0) != FLASH_COMPLETE ) {
        FLASH_Lock();
        return 0;
    }
    FLASH_Lock();
    FlashPositionReadRecord(used - 1, record);
    return record[0] == 0;
}

/**
 * 函数作用：全部电机停稳后追加位置，数据读回正确后最后提交标记。
 * data：停稳坐标；motorMask：启用组合1至7；返回1表示提交成功。
 * 页满才擦位置页，校准页不擦；旧记录先失效，任何中断均拒绝旧坐标。
 */
u8 FlashMotorPositionWrite(const FlashMotorPositionData *data, u8 motorMask)
{
    u32 record[FlashPositionWordCount]; // 待写入的位置记录。
    u32 verify[FlashPositionWordCount]; // 读回校验内容。
    u32 address; // 新记录起始地址。
    u16 used; // 已占用槽数。
    u8 word; // 当前写入或比较字序号。
    if( (data == 0) || (motorMask == 0) || (motorMask > 7) ) return 0;
    if( FlashMotorPositionInvalidate() == 0 ) return 0;
    used = FlashPositionUsedSlots();
    record[0] = FlashPositionMagic;
    record[1] = FlashPositionVersion + (u32)motorMask;
    record[2] = data->Motor1Pulse;
    record[3] = data->Motor3Pulse;
    record[4] = FlashPositionChecksum(record);
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    if( used >= FlashPositionSlotCount ) {
        if( FLASH_ErasePage(FlashPositionFlashAddress) != FLASH_COMPLETE ) {
            FLASH_Lock();
            return 0;
        }
        used = 0;
    }
    address = FlashPositionFlashAddress + (u32)used * 20;
    for( word = 1; word < FlashPositionWordCount; word++ ) {
        if( FLASH_ProgramWord(address + (u32)word * 4, record[word]) != FLASH_COMPLETE ) {
            FLASH_Lock();
            return 0;
        }
    }
    FlashPositionReadRecord(used, verify);
    for( word = 1; word < FlashPositionWordCount; word++ ) {
        if( verify[word] != record[word] ) {
            FLASH_Lock();
            return 0;
        }
    }
    if( FLASH_ProgramWord(address, FlashPositionMagic) != FLASH_COMPLETE ) {
        FLASH_Lock();
        return 0;
    }
    FLASH_Lock();
    FlashPositionReadRecord(used, verify);
    return FlashPositionValidate(verify, motorMask);
}
