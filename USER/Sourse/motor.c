#include "motor.h"
#include "flash.h"

#define MotorCalibrationMask (Motor1Enable + Motor2Enable * 2 + Motor3Enable * 4) // 当前启用的电机组合。

uint8_t MotorEn; // 电机服务初始化完成标志。
u8 MotorCalibrationLoaded; // 本次启动是否使用历史标定数据。
static u8 MotorCalibrationSaveAttempted; // 本次启动是否已尝试保存，避免失败后反复擦写。

/**
 * 函数作用：检查每路电机标定范围和换算关系，拒绝不适用于运动控制的数据。
 * calibration：待检查的三路校准数据；返回1表示有效，0表示应执行完整自检。
 */
static u8 MotorCalibrationValidate(const FlashCalibrationData *calibration)
{
    if( calibration == 0 ) return 0;
#if Motor1Enable == 1
    if( (calibration->MaxPulseNum[0] < C_VerifyMinimumPulseNum) || (calibration->MaxPulseNum[0] > 65535) ||
        (calibration->PulseScale[0] == 0) || (calibration->PulseScale[0] != calibration->MaxPulseNum[0] * 10 / Mot1MaxAngle) ) return 0;
#endif
#if Motor2Enable == 1
    if( (calibration->MaxPulseNum[1] < C_VerifyMinimumPulseNum) || (calibration->MaxPulseNum[1] > 0x0FFFFFFF) ||
        (calibration->PulseScale[1] == 0) || (calibration->PulseScale[1] != calibration->MaxPulseNum[1] * 10 / Motor2CalibrationHeight) ) return 0;
#endif
#if Motor3Enable == 1
    if( (calibration->MaxPulseNum[2] < C_VerifyMinimumPulseNum) || (calibration->MaxPulseNum[2] > 65535) ||
        (calibration->PulseScale[2] == 0) || (calibration->PulseScale[2] != calibration->MaxPulseNum[2] * 10 / Mot3MaxAngle) ) return 0;
#endif
    return 1;
}

/**
 * 函数作用：通过Flash接口读取历史标定，保留Reset状态让电机先找基准再归位。
 * 无传入参数；空白、损坏、不兼容或超出电机范围时保留默认值和完整自检流程。
 */
static void MotorCalibrationLoad(void)
{
    FlashCalibrationData calibration; // Flash接口返回的三路校准数据。

    MotorCalibrationLoaded = 0;
    if( FlashCalibrationRead(&calibration, MotorCalibrationMask) == 0 ) return;
    if( MotorCalibrationValidate(&calibration) == 0 ) return;
#if Motor1Enable == 1
    Mot1.MaxPulseNum = (u16)calibration.MaxPulseNum[0];
    Mot1.PulseNumPerAngle = (u16)calibration.PulseScale[0];
#endif
#if Motor2Enable == 1
    Mot2.MaxPulseNum = calibration.MaxPulseNum[1];
    Mot2.PulseNumPerMm = calibration.PulseScale[1];
#endif
#if Motor3Enable == 1
    Mot3.MaxPulseNum = (u16)calibration.MaxPulseNum[2];
    Mot3.PulseNumPerAngle = (u16)calibration.PulseScale[2];
#endif
    MotorCalibrationLoaded = 1;
    printf("motor calibration loaded\n");
}

/**
 * 函数作用：整理并校验电机实测标定，再调用Flash模块保存。
 * 无传入参数；返回1表示保存成功，0表示标定参数无效或Flash保存失败。
 */
static u8 MotorCalibrationSave(void)
{
    FlashCalibrationData calibration; // 提交给Flash模块的三路校准数据。
    u8 axis; // 当前清零校准数据的电机序号。

    for( axis = 0; axis < FlashCalibrationMotorCount; axis++ ) {
        calibration.MaxPulseNum[axis] = 0;
        calibration.PulseScale[axis] = 0;
    }
#if Motor1Enable == 1
    calibration.MaxPulseNum[0] = Mot1.MaxPulseNum;
    calibration.PulseScale[0] = Mot1.PulseNumPerAngle;
#endif
#if Motor2Enable == 1
    calibration.MaxPulseNum[1] = Mot2.MaxPulseNum;
    calibration.PulseScale[1] = Mot2.PulseNumPerMm;
#endif
#if Motor3Enable == 1
    calibration.MaxPulseNum[2] = Mot3.MaxPulseNum;
    calibration.PulseScale[2] = Mot3.PulseNumPerAngle;
#endif
    if( MotorCalibrationValidate(&calibration) == 0 ) return 0;
    return FlashCalibrationWrite(&calibration, MotorCalibrationMask);
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

/** 函数作用：初始化启用的电机、Flash存储访问并读取历史校准数据；无传入参数。 */
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
    FlashInit();
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
