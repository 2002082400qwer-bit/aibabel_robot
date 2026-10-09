#include "motor.h"
#include "flash.h"

#define MotorCalibrationMask (Motor1Enable + Motor2Enable * 2 + Motor3Enable * 4) // 当前启用的电机组合。

uint8_t MotorEn; // 电机服务初始化完成标志。
u8 MotorCalibrationLoaded; // 本次启动是否使用历史标定数据。
u8 MotorPositionLoaded; // 本次启动是否恢复可信坐标，运动后仍保留启动路径选择。
static u8 MotorCalibrationStored; // 校准已在Flash中有效保存。
static u8 MotorPositionDirty; // 旧位置已持久化失效，允许本轮多轴运动。
static u8 MotorPositionSaveAttempted; // 本轮停稳是否已尝试保存，避免失败重试擦写。
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
 * 函数作用：通过Flash接口读取历史标定，保留Reset状态等待位置恢复及归位。
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
    MotorCalibrationStored = 1;
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
    if( MotorCalibrationSave() != 0 ) {
        MotorCalibrationStored = 1;
        printf("motor calibration saved\n");
    }
    else printf("motor calibration save failed\n");
}


/**
 * 函数作用：有效校准加载后恢复可信停稳坐标，仍保留Reset进行直接回中。
 * 无传入参数；坐标无效或越界时保留寻限位流程。
 */
static void MotorPositionLoad(void)
{
    FlashMotorPositionData position; // Flash中的两路停稳坐标。
    MotorPositionLoaded = 0;
    if( MotorCalibrationLoaded == 0 ) return;
    if( FlashMotorPositionRead(&position, MotorCalibrationMask) == 0 ) return;
#if Motor1Enable == 1
    if( position.Motor1Pulse > Mot1.MaxPulseNum ) return;
#endif
#if Motor3Enable == 1
    if( position.Motor3Pulse > Mot3.MaxPulseNum ) return;
#endif
#if Motor1Enable == 1
    Mot1.CurPosition = (u16)position.Motor1Pulse;
    Mot1.CurAngle = (u16)((u32)Mot1.CurPosition * 100 / Mot1.PulseNumPerAngle);
#endif
#if Motor3Enable == 1
    Mot3.CurPosition = (u16)position.Motor3Pulse;
    Mot3.CurAngle = (u16)((u32)Mot3.CurPosition * 100 / Mot3.PulseNumPerAngle);
#endif
    MotorPositionLoaded = 1;
}

/**
 * 函数作用：判断所有启用轴是否完成归位、无故障、停稳且无待执行任务。
 * 无传入参数；返回1表示允许保存停稳坐标。
 */
static u8 MotorPositionStopped(void)
{
#if Motor1Enable == 1
    if( (Mot1.Reset != 0) || (Mot1.En == 0) || (Mot1.Action != motor_stop) ||
        (Mot1.Faultflag != NoFault) || (Move1.En != 0) ) return 0;
#endif
#if Motor2Enable == 1
    if( (Mot2.Reset != 0) || (Mot2.En == 0) || (Mot2.Action != motor_stop) ||
        (Mot2.Faultflag != NoFault) || (Move2.En != 0) ) return 0;
#endif
#if Motor3Enable == 1
    if( (Mot3.Reset != 0) || (Mot3.En == 0) || (Mot3.Action != motor_stop) ||
        (Mot3.Faultflag != NoFault) || (Move3.En != 0) ) return 0;
#endif
    return MotorCalibrationMask != 0;
}

/**
 * 函数作用：第一路电机启动前持久化失效旧坐标，多轴同轮运动只失效一次。
 * 无传入参数；返回1允许启动，0必须报故障并拒绝使能PWM。
 */
u8 MotorPositionPrepareMove(void)
{
    if( MotorPositionDirty != 0 ) {
        // 上轮保存失败也允许下一次真实运动开启新的停稳保存周期。
        MotorPositionSaveAttempted = 0;
        return 1;
    }
    if( FlashMotorPositionInvalidate() == 0 ) return 0;
    MotorPositionDirty = 1;
    MotorPositionSaveAttempted = 0;
    return 1;
}

/**
 * 函数作用：校准有效且全部电机停稳后追加保存一次位置。
 * 无传入参数；失败本轮不重试，下一次运动才开启新的保存周期。
 */
static void MotorPositionService(void)
{
    FlashMotorPositionData position; // 本次停稳位置快照。
    u8 saved; // Flash提交结果，失败也可能已完成标记写入。
    if( (MotorCalibrationStored == 0) || (MotorPositionSaveAttempted != 0) ) return;
    if( MotorPositionStopped() == 0 ) return;
    MotorPositionSaveAttempted = 1;
    position.Motor1Pulse = 0;
    position.Motor3Pulse = 0;
#if Motor1Enable == 1
    if( Mot1.CurPosition > Mot1.MaxPulseNum ) return;
    position.Motor1Pulse = Mot1.CurPosition;
#endif
#if Motor3Enable == 1
    if( Mot3.CurPosition > Mot3.MaxPulseNum ) return;
    position.Motor3Pulse = Mot3.CurPosition;
#endif
    saved = FlashMotorPositionWrite(&position, MotorCalibrationMask);
    // 失败也可能已经提交标记，下一次启动必须重新确认并失效。
    MotorPositionDirty = 0;
    if( saved != 0 ) {
        printf("motor position saved\n");
    }
    else printf("motor position save failed\n");
}

/** 函数作用：初始化启用的电机、Flash存储访问并读取历史校准数据；无传入参数。 */
void MotorInit(void)
{
    MotorCalibrationLoaded = 0;
    MotorCalibrationSaveAttempted = 0;
    MotorCalibrationStored = 0;
    MotorPositionLoaded = 0;
    MotorPositionDirty = 0;
    MotorPositionSaveAttempted = 0;
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
    MotorPositionLoad();
    MotorEn = 1;
}

/** 函数作用：依次执行三路电机服务，首次保存标定并在停稳后保存位置；无传入参数。 */
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
    MotorPositionService();
}
