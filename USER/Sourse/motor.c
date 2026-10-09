#include "motor.h"

#define MotorCalibrationFlashAddress 0x0803F800 // FT32F103VC最后一个2KB页，链接区已预留。
#define MotorCalibrationWordCount 9 // 校准记录的32位字数量，共36字节。
#define MotorCalibrationMagic 0x4D43414C // 记录成功标记，最后写入以防掉电误用。
#define MotorCalibrationMask (Motor1Enable + Motor2Enable * 2 + Motor3Enable * 4) // 当前启用的电机组合。
#define MotorCalibrationFormat (0x00010000 + MotorCalibrationMask) // 记录版本1及电机组合。
#define MotorCalibrationCrcPolynomial 0xEDB88320 // CRC32的反射多项式。

uint8_t MotorEn; // 电机服务初始化完成标志。
u8 MotorCalibrationLoaded; // 本次启动是否使用历史标定数据。
static u8 MotorCalibrationSaveAttempted; // 本次启动是否已尝试保存，避免失败后反复擦写。

/**
 * 函数作用：计算校准数据的CRC32，覆盖版本、行程和换算系数。
 * record：含9个32位字的记录，首字标记和末字CRC不参与计算。
 * 返回值：数据校验值。
 */
static u32 MotorCalibrationChecksum(const u32 *record)
{
    u32 crc = 0xFFFFFFFF; // CRC计算的初始值。
    u8 word; // 当前数据字序号。
    u8 bit; // 当前数据字的位序号。

    for( word = 1; word < MotorCalibrationWordCount - 1; word++ ) {
        crc ^= record[word];
        for( bit = 0; bit < 32; bit++ ) {
            if( crc & 1 ) crc = (crc >> 1) ^ MotorCalibrationCrcPolynomial;
            else crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}

/**
 * 函数作用：核对记录版本、CRC和每路标定范围，拒绝未完成或损坏的记录。
 * record：待校验的9字记录；返回1表示有效，0表示应执行完整自检。
 */
static u8 MotorCalibrationValidate(const u32 *record)
{
    if( record[0] != MotorCalibrationMagic ) return 0;
    if( record[1] != MotorCalibrationFormat ) return 0;
    if( record[8] != MotorCalibrationChecksum(record) ) return 0;
#if Motor1Enable == 1
    if( (record[2] < C_VerifyMinimumPulseNum) || (record[2] > 65535) ||
        (record[3] == 0) || (record[3] != record[2] * 10 / Mot1MaxAngle) ) return 0;
#endif
#if Motor2Enable == 1
    if( (record[4] < C_VerifyMinimumPulseNum) || (record[4] > 0x0FFFFFFF) ||
        (record[5] == 0) || (record[5] != record[4] * 10 / Motor2CalibrationHeight) ) return 0;
#endif
#if Motor3Enable == 1
    if( (record[6] < C_VerifyMinimumPulseNum) || (record[6] > 65535) ||
        (record[7] == 0) || (record[7] != record[6] * 10 / Mot3MaxAngle) ) return 0;
#endif
    return 1;
}

/**
 * 函数作用：初始化后读取历史标定，保留Reset状态让电机先找基准再归位。
 * 无传入参数；空白、版本不兼容或CRC错误时保留默认值和完整自检流程。
 */
static void MotorCalibrationLoad(void)
{
    u32 record[MotorCalibrationWordCount]; // 从Flash读出的校准记录。

    MotorCalibrationLoaded = 0;
    Flash_Read(MotorCalibrationFlashAddress, (u8 *)record, sizeof(record));
    if( MotorCalibrationValidate(record) == 0 ) return;
#if Motor1Enable == 1
    Mot1.MaxPulseNum = (u16)record[2];
    Mot1.PulseNumPerAngle = (u16)record[3];
#endif
#if Motor2Enable == 1
    Mot2.MaxPulseNum = record[4];
    Mot2.PulseNumPerMm = record[5];
#endif
#if Motor3Enable == 1
    Mot3.MaxPulseNum = (u16)record[6];
    Mot3.PulseNumPerAngle = (u16)record[7];
#endif
    MotorCalibrationLoaded = 1;
    printf("motor calibration loaded\n");
}

/**
 * 函数作用：将首次成功校准写入预留Flash页，先写并读回数据，最后提交成功标记。
 * 无传入参数；返回1表示保存和读回校验成功，0表示失败。
 * 所有电机停止后才调用；使用厂家驱动返回状态，任何失败都会重新锁定Flash。
 */
static u8 MotorCalibrationSave(void)
{
    u32 record[MotorCalibrationWordCount]; // 本次待保存的校准记录。
    u32 verify[MotorCalibrationWordCount]; // 写入后的读回校验记录。
    u8 word; // 当前初始化、写入或比较的字序号。

    for( word = 0; word < MotorCalibrationWordCount; word++ ) record[word] = 0;
    record[0] = MotorCalibrationMagic;
    record[1] = MotorCalibrationFormat;
#if Motor1Enable == 1
    record[2] = Mot1.MaxPulseNum;
    record[3] = Mot1.PulseNumPerAngle;
#endif
#if Motor2Enable == 1
    record[4] = Mot2.MaxPulseNum;
    record[5] = Mot2.PulseNumPerMm;
#endif
#if Motor3Enable == 1
    record[6] = Mot3.MaxPulseNum;
    record[7] = Mot3.PulseNumPerAngle;
#endif
    record[8] = MotorCalibrationChecksum(record);
    if( MotorCalibrationValidate(record) == 0 ) return 0;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    if( FLASH_ErasePage(MotorCalibrationFlashAddress) != FLASH_COMPLETE ) {
        FLASH_Lock();
        return 0;
    }
    for( word = 1; word < MotorCalibrationWordCount; word++ ) {
        if( FLASH_ProgramWord(MotorCalibrationFlashAddress + (u32)word * 4, record[word]) != FLASH_COMPLETE ) {
            FLASH_Lock();
            return 0;
        }
    }
    Flash_Read(MotorCalibrationFlashAddress, (u8 *)verify, sizeof(verify));
    for( word = 1; word < MotorCalibrationWordCount; word++ ) {
        if( verify[word] != record[word] ) {
            FLASH_Lock();
            return 0;
        }
    }
    if( FLASH_ProgramWord(MotorCalibrationFlashAddress, MotorCalibrationMagic) != FLASH_COMPLETE ) {
        FLASH_Lock();
        return 0;
    }
    FLASH_Lock();
    Flash_Read(MotorCalibrationFlashAddress, (u8 *)verify, sizeof(verify));
    return MotorCalibrationValidate(verify);
}

/**
 * 函数作用：等待全部启用电机自检和归位成功后保存一次，历史标定启动不重复擦写。
 * 无传入参数；任一路未停止、未完成或出现故障时不保存。
 */
static void MotorCalibrationService(void)
{
    if( (MotorCalibrationLoaded != 0) || (MotorCalibrationSaveAttempted != 0) ) return;
#if Motor1Enable == 1
    if( (Mot1.Reset != 0) || (Mot1.En == 0) || (Mot1.Action != motor_stop) || (Mot1.Faultflag != NoFault) ) return;
#endif
#if Motor2Enable == 1
    if( (Mot2.Reset != 0) || (Mot2.En == 0) || (Mot2.Action != motor_stop) || (Mot2.Faultflag != NoFault) ) return;
#endif
#if Motor3Enable == 1
    if( (Mot3.Reset != 0) || (Mot3.En == 0) || (Mot3.Action != motor_stop) || (Mot3.Faultflag != NoFault) ) return;
#endif
    if( MotorCalibrationMask == 0 ) return;
    MotorCalibrationSaveAttempted = 1;
    if( MotorCalibrationSave() != 0 ) printf("motor calibration saved\n");
    else printf("motor calibration save failed\n");
}

/** 函数作用：初始化启用的电机并读取历史校准数据；无传入参数。 */
void MotorInit(void)
{
    MotorCalibrationLoaded = 0;
    MotorCalibrationSaveAttempted = 0;
#if Motor1Enable == 1
    Mot1Init();
#endif
#if Motor2Enable == 1
    Mot2Init();
#endif
#if Motor3Enable == 1
    Mot3Init();
#endif
    MotorCalibrationLoad();
    MotorEn = 1;
}

/** 函数作用：依次执行三路电机服务，并在首次成功后保存标定；无传入参数。 */
void MotorService(void)
{
#if Motor1Enable == 1
    Mot1MoveService();
#endif
#if Motor2Enable == 1
    Mot2MoveService();
#endif
#if Motor3Enable == 1
    Mot3MoveService();
#endif
    MotorCalibrationService();
}
