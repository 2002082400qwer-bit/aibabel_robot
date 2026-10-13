#include "flash.h"
#include "iap.h"

#define FlashCalibrationFlashAddress 0x0803F800 // FT32F103VC最后一个2KB页，链接区已预留。
#define FlashCalibrationWordCount 9 // 校准记录的32位字数量，共36字节。
#define FlashCalibrationMagic 0x4D43414C // 记录成功标记，最后写入以防掉电误用。
#define FlashCalibrationVersion 0x00010000 // 记录版本1，低三位保存启用电机组合。
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
    Flash_Read(FlashCalibrationFlashAddress, (u8 *)record, sizeof(u32) * FlashCalibrationWordCount);
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
    if( (motorMask == 0) || (motorMask > 7) )                       return 0;
    if( record[0] != FlashCalibrationMagic )                        return 0;
    if( record[1] != (FlashCalibrationVersion + (u32)motorMask) )   return 0;
    if( record[8] != FlashCalibrationChecksum(record) )             return 0;

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
 * 所有电机停止后才调用；复用IAP擦写接口，通过硬件状态及读回检查确认结果。
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

    /* IAP接口不返回操作结果，先清除历史错误，再检查状态及记录区是否擦空。 */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_Erase_OnePage(FlashCalibrationFlashAddress);

    if( FLASH_GetStatus() != FLASH_COMPLETE ) return 0;

    FlashCalibrationReadRecord(verify);

    for( word = 0; word < FlashCalibrationWordCount; word++ ) {
        if( verify[word] != 0xFFFFFFFF ) return 0;
    }

    /* 每次只写4字节并检查状态，满足IAP写入长度要求，失败时停止后续写入。 */
    for( word = 1; word < FlashCalibrationWordCount; word++ ) {
        FLASH_WriteWord(FlashCalibrationFlashAddress + (u32)word * 4, (u8 *)&record[word], 4);
        if( FLASH_GetStatus() != FLASH_COMPLETE ) return 0;
    }

    FlashCalibrationReadRecord(verify);

    for( word = 1; word < FlashCalibrationWordCount; word++ ) {
        if( verify[word] != record[word] ) {
            FLASH_Lock();
            return 0;
        }
    }

    /* 数据读回完全一致后，最后通过IAP接口写入成功标记。 */
    FLASH_WriteWord(FlashCalibrationFlashAddress, (u8 *)&record[0], 4);

    if( FLASH_GetStatus() != FLASH_COMPLETE ) return 0;

    FlashCalibrationReadRecord(verify);

    return FlashCalibrationValidate(verify, motorMask);
}
