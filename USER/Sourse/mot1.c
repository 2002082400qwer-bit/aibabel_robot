/*******************************************************************************

	include file
*******************************************************************************/
#include "mot1.h"

/*******************************************************************************
	constant define
*******************************************************************************/
#define c_MaxAngle 	180									//最大角度180度
#define c_PulseScaleFactor 10							//10倍
#define c_AngleScaleFactor 10							//10倍
#define c_RadianScaleFactor 10							//10倍
#define c_DefaultTotalPulseNum	3600					//总的脉冲数
#define c_DefaultPulseNumPerAngle 20*c_PulseScaleFactor	//每度对应的脉冲数
#define c_MoveTimeOver 20000							//20s,马达不转无脉冲输出时间
#define Mot1TriggerAdc 2800
#define Mot1StopDebTicks 30000 // 编码器连续300毫秒无电平变化才确认电机1停稳，SysTick周期为10微秒。
//-----------------------------------------------------------------
//	ram 
//-----------------------------------------------------------------

Motor1St Mot1;
Move1St Move1;

void Mot1Init(void)
{
	Mot1.Reset = 1;
	Mot1.Action = motor_stop;
	Mot1.Step = 0;
	Mot1.PidTimer = 0;
	Mot1.PidTimerB = 0;
	Mot1.PidEn = 0;
	Mot1.PulseSta = GPIO_ReadInputDataBit( Mot1PulseIO, Mot1PulsePin );
	Mot1.PulseDeb = 0;
	Mot1.PulseDebB = 0;
	Mot1.PulseNum = 0;
	Mot1.PulseNumB = 0;
	Mot1.BrakeType = 0;
	Mot1.ErrPulseNum = 0;
	Mot1.BrakeType = 0;
	Mot1.TarPulseNum = 0;
	Mot1.MaxPulseNum = c_DefaultTotalPulseNum;
	Mot1.PulseNumPerAngle = c_DefaultPulseNumPerAngle;
	Mot1.TarVelocity = 0;
	Mot1.CurVelocity = 0;
	Mot1.TarPosition = 0;
	Mot1.CurPosition = 0;	
	Mot1.CurAngle = 0;
	Mot1.TarAngle = 0;
	Mot1.Hall1Sta = 0;
	Mot1.Hall1Deb = 0;
	Mot1.Hall2Sta = 0;
	Mot1.Hall2Deb = 0;
	Mot1.Faultflag = 0;
	Mot1.En = 0;
	
	Move1.En = 0;
	Move1.Step = 0;
}

void Mot1LeftHallScan(void)
{
	if( Mot1.Hall1Sta == 0 ) {
		if( Adc.Mot1.Hall1 >= Mot1TriggerAdc ) {
			if( Mot1.Hall1Deb >= 20 )	{	
				//printf("tigger1\n");
				Mot1.Hall1Sta = 1;
			}	
		}	
		else {
			Mot1.Hall1Deb = 0;
		}	
  }
  else {
		if( Adc.Mot1.Hall1 < Mot1TriggerAdc ) {
			if( Mot1.Hall1Deb >= 20 )	{	
				Mot1.Hall1Sta = 0;
			}	
		}	
		else {
			Mot1.Hall1Deb = 0;
		}	
  }
}

u8 Mot1ReadLeftHallState(void)
{
	return Mot1.Hall1Sta;
}

void Mot1RightHallScan(void)
{
	if( Mot1.Hall2Sta == 0 ) {
		if( Adc.Mot1.Hall3 >= Mot1TriggerAdc ) {
			if( Mot1.Hall2Deb >= 20 )	{	
				Mot1.Hall2Sta = 1;
				//printf("tigger2\n");
			}	
		}	
		else {
			Mot1.Hall2Deb = 0;
		}	
  }
  else {
		if( Adc.Mot1.Hall3 < Mot1TriggerAdc ) {
			if( Mot1.Hall2Deb >= 20 )	{	
				Mot1.Hall2Sta = 0;
			}	
		}	
		else {
			Mot1.Hall2Deb = 0;
		}	
  }
}	

u8 Mot1ReadRightHallState(void)
{
	return Mot1.Hall2Sta;
}

/* 增量式速度PID，增益按百分之一缩放，Ki按每个50ms控制周期累加。 */
#define Mot1PidScale          100
#define Mot1VelocityKp        250 // 速度误差变化时的比例响应系数。
#define Mot1VelocityKi        4 // 上位机运行时持续速度误差的积分系数。
#define Mot1VelocityResetKi   2 // 复位时持续速度误差的积分系数。
#define Mot1VelocityKd        50
#define Mot1PidStartDuty      1500
#define Mot1PidCruiseDuty     2100
// 上位机控制状态下的最低PWM占空比，同时作为启动占空比。
#define Mot1PidHostMinimumPwm 3000 // 正常运行与复位回中前段的最低PWM，保证通过高负载区。
#define Mot1PidApproachPulseNum 300 // 长行程结束前提前降速的脉冲窗口。
#define Mot1PidApproachMinimumVelocity 50 // 末段速度指令的下限。
#define Mot1PidApproachMinimumPwm 1800 // 末段PWM下限，避免完全失去驱动力。
#define Mot1PidApproachMaximumPwm 2200 // 到目标前允许的PWM上限。
/* 末端回拉补偿：停稳后按残差再低速往回走一小段，用来抵消制动过冲留下的固定偏差。 */
#define Mot1PosCorrEnable        1    // 1：启用末端回拉补偿；0：关闭，方便做对比试验。
#define Mot1PosCorrDeadbandAngle 10   // 回拉死区，单位0.1度(即1.0度)；残差小于这个值就不再回拉。
#define Mot1PosCorrMaxAngle      50   // 单次回拉允许的最大残差，单位0.1度；超过就判为异常不回拉。
#define Mot1PosCorrVelocity      50   // 回拉运动的目标速度，取小值让末端走得更慢。
#define Mot1PosCorrMaxTimes      2    // 一次目标最多回拉几次，防止来回找位置。
#define Mot1PosCorrTimeOver      2000 // 单次回拉最长持续时间，单位毫秒；超时按普通制动结束。
#define Mot1PosCorrRatioPercent  50   // 每次只回拉残差的百分比，留出停机制动的过冲余量。
#define Mot1PosCorrMinimumPwm    1500 // 回拉运动专用PWM下限，比正常运动小很多。
#define Mot1PosCorrMaximumPwm    2000 // 回拉运动专用PWM上限，限制回拉力量避免再冲过目标。
#define Mot1PidResetApproachMinimumPwm 1500 // 复位回中末段的PWM下限，参考旧版末段驱动。
#define Mot1PidResetApproachMaximumPwm 1800 // 复位回中末段的PWM上限，限制冲过中点。

static int32_t Mot1PidPreviousError = 0;
static int32_t Mot1PidPreviousPreviousError = 0;
static int32_t Mot1PidOutputScaled = 0;
/* 末端回拉补偿的运行状态和函数声明 */
static u8 Mot1PosCorrTimes = 0;  // 本次目标已经回拉的次数。
static u8 Mot1PosCorrBusy = 0;   // 1：当前这次运动就是回拉补偿运动。
static u16 Mot1AngleToPulse(u16 Angle);         // 把0.1度角度偏差换算成脉冲数。
static void Mot1PositionCorrectionHandle(void); // 停稳后按残差判断是否还要回拉。
void Mot1MoveBrake(void);                       // 回拉超时需要制动，函数定义在后面先声明。

/**
 * 函数作用：清除电机1速度PID的历史误差，并设置本次启动占空比。
 * InitialPwm：本次运动开始时的PWM占空比，范围为0到最大PWM。
 */
static void Mot1PidReset(u16 InitialPwm)
{
	Mot1PidPreviousError = 0;
	Mot1PidPreviousPreviousError = 0;
	if( InitialPwm > c_MaxPwmDuty ) InitialPwm = c_MaxPwmDuty;
	Mot1PidOutputScaled = (int32_t)InitialPwm * Mot1PidScale;
}

/**
 * 函数作用：根据目标速度和当前速度执行电机1增量式PID并输出限幅PWM。
 * TargetVelocity：本周期目标速度，单位与Mot1.CurVelocity相同。
 * CurrentVelocity：由电机1编码脉冲计算出的当前速度。
 * 返回值：范围为0到最大PWM的占空比。
 */
static u16 Mot1VelocityFeedbackControl(u16 TargetVelocity, u16 CurrentVelocity)
{
	int32_t Error;
	int32_t DeltaError;
	int32_t SecondDifference;
	int32_t DeltaOutputScaled;
	int32_t OutputLimitScaled;

	if( TargetVelocity == 0 ) {
		Mot1PidReset(0);
		return 0;
	}

	Error = (int32_t)TargetVelocity - (int32_t)CurrentVelocity;
	DeltaError = Error - Mot1PidPreviousError;
	SecondDifference = Error - 2 * Mot1PidPreviousError + Mot1PidPreviousPreviousError;
	/* 复位沿用较小积分系数，正常运行使用试调积分系数。 */
	DeltaOutputScaled = Mot1VelocityKp * DeltaError +
		((Mot1.Reset != 0) ? Mot1VelocityResetKi : Mot1VelocityKi) * Error +
		Mot1VelocityKd * SecondDifference;
	OutputLimitScaled = (int32_t)c_MaxPwmDuty * Mot1PidScale;
	Mot1PidOutputScaled += DeltaOutputScaled;
	if( Mot1PidOutputScaled < 0 ) Mot1PidOutputScaled = 0;
	if( Mot1PidOutputScaled > OutputLimitScaled ) Mot1PidOutputScaled = OutputLimitScaled;

	Mot1PidPreviousPreviousError = Mot1PidPreviousError;
	Mot1PidPreviousError = Error;
	return (u16)(Mot1PidOutputScaled / Mot1PidScale);
}

/**
 * 函数作用：检测电机1编码脉冲和停稳时间；正常运行且电机未停稳时，在限位霍尔区域也累计脉冲。
 */
void Mot1Encode(void)
{
	u8 state; // 本次读取的编码器电平。
	state = GPIO_ReadInputDataBit( Mot1PulseIO, Mot1PulsePin ) ;
	if( state != Mot1.PulseSta ) {
		Mot1.PulseDebB = 0;
		Mot1.PulseDeb++;
		if( Mot1.PulseDeb >= 10 ) {
				
			Mot1.PulseSta = state;
      if( state != 0 ) {
					 if( ((Mot1.Reset == 0) && (Mot1.Action != motor_stop)) ||
					((Mot1.Hall1Sta == 0) && (Mot1.Hall2Sta == 0)) ) {
							Mot1.PulseNum++;
							Mot1.PulseNumB++;
					 } 
        }    
     }
  }
  else
  {
		Mot1.PulseDeb = 0;
		if( Mot1.PulseDebB < Mot1StopDebTicks ) Mot1.PulseDebB++;
  }
}
u16 Mot1ReadEncode(void)
{
	u16 PulseNum;
	__disable_irq();
	PulseNum = Mot1.PulseNumB;
	Mot1.PulseNumB = 0;
	__enable_irq();
	return PulseNum; 
}	
void Mot1ClearEncode(void)
{
	Mot1.PulseNum = 0;
	Mot1.PulseNumB = 0;
	Mot1.BrakeType = 0;
	Mot1.PidTimer = 0;
	Mot1.PidTimerB = 0;
}	
/**
 * 函数作用：启动电机1速度闭环前清零测速脉冲并重置PID状态。
 */
void Mot1EnablePidControl(void)
{
	Mot1ClearEncode();
	Mot1.CurVelocity = 0;
	if( Mot1.Reset != 0 ) {
		if( Mot1.TarPulseNum >= 50 ) Mot1PidReset(Mot1PidCruiseDuty);
		else Mot1PidReset(Mot1PidStartDuty);
	}
	else Mot1PidReset(Mot1PidHostMinimumPwm);
	Mot1.PidEn = Enable;
}
/**
 * 函数作用：关闭电机1速度PID、清零PWM和历史控制状态，并重新开始停稳计时。
 */
void Mot1DisablePidControl(void)
{
	TIM1_SetDuty_CH3(0);
	Mot1.PulseDebB = 0;
	Mot1.PidEn = Disable;
	Mot1PidReset(0);
}
void Mot1SetCurrentPosition( u16 num )
{
	Mot1.CurPosition = num;
}	
void Mot1SetTargetPosition( u16 num )
{
	Mot1.TarPosition = num;
}	
/**
 * 函数作用：每50毫秒更新电机1位置、速度与PWM；复位回中时按行程长度
 *          设置末段降速窗口，并打印右转霍尔原始值供限位分析。
 */
void Mot1VelocityControl(void)
{
    u16 Pwm; // 本周期写入定时器的PWM占空比。
    u16 PidTargetVelocity; // 经过末段降速处理的目标速度。
    u16 MinimumPwm; // 本周期运动所需的最低PWM。
    u16 MaximumPwm; // 末段避免高速撞击的最高PWM。
    u32 sum; // 根据本次编码脉冲换算的位置中间量。
    u32 encode_count; // 本控制周期测得的编码脉冲换算量。
    u32 remainingPulseNum; // 距本次目标剩余的编码脉冲数。
    u32 approachPulseNum; // 本次运动末段提前降速的脉冲窗口。
    u16 approachMinimumPwm; // 本次运动末段PWM下限。
    u16 approachMaximumPwm; // 本次运动末段PWM上限。

    if( Mot1.PidEn == 0 ) return;
    /* 回拉补偿超时保护：机构卡住或走不动时按普通制动收尾，避免一直停在半路。 */
    if( (Mot1PosCorrBusy != 0) && (Mot1.PidTimerB > Mot1PosCorrTimeOver) ) {
        Mot1PosCorrTimes = Mot1PosCorrMaxTimes;
        Mot1MoveBrake();
        return;
    }
    if( Mot1.PidTimer < C_PID_RATE ) return;

    Mot1.PidTimer = Mot1.PidTimer - C_PID_RATE;
    sum = 0;
    if( Mot1.Action == motor_forward ) {
        sum = Mot1.CurPosition + Mot1.PulseNum;
        if( sum > Mot1.MaxPulseNum ) sum = Mot1.MaxPulseNum;
    }
    else if( Mot1.Action == motor_backward ) {
        if( Mot1.CurPosition > Mot1.PulseNum ) sum = Mot1.CurPosition - Mot1.PulseNum;
    }
    if( (sum != 0) && (Mot1.MaxPulseNum != 0) ) {
        // 按完整标定行程换算，避免每度脉冲整数截断造成角度上报偏大。
        Mot1.CurAngle = (sum * c_MaxAngle * 10) / Mot1.MaxPulseNum;
    }
    else Mot1.CurAngle = 0;

    encode_count = Mot1ReadEncode() * (1000 / C_PID_RATE);
    Mot1.PulseNumPerMin = encode_count * 60;
    encode_count = (u32)(encode_count * c_RadianScaleFactor * c_PulseScaleFactor);
    if( encode_count != 0 ) Mot1.CurVelocity = (u32)(encode_count / Mot1.PulseNumPerAngle);
    else Mot1.CurVelocity = 0;

    // 复位回中先保持驱动力穿过约三分之一行程的重载区，再在余下三分之一回中行程降速。
    approachPulseNum = Mot1PidApproachPulseNum;
    approachMinimumPwm = Mot1PidApproachMinimumPwm;
    approachMaximumPwm = Mot1PidApproachMaximumPwm;
    if( (Mot1.Reset != 0) && (Mot1.Step == 3) ) {
        approachPulseNum = Mot1.TarPulseNum / 3;
        approachMinimumPwm = Mot1PidResetApproachMinimumPwm;
        approachMaximumPwm = Mot1PidResetApproachMaximumPwm;
    }

    if( Mot1.TarPulseNum > Mot1.PulseNum ) {
        remainingPulseNum = Mot1.TarPulseNum - Mot1.PulseNum;
        PidTargetVelocity = Mot1.TarVelocity;
        if( ((Mot1.Reset == 0) || (Mot1.Step == 3)) &&
            (approachPulseNum != 0) &&
            (Mot1.TarPulseNum >= 2 * approachPulseNum) &&
            (remainingPulseNum < approachPulseNum) &&
            (PidTargetVelocity > Mot1PidApproachMinimumVelocity) ) {
            PidTargetVelocity = (u16)(((u32)Mot1.TarVelocity * remainingPulseNum) / approachPulseNum);
            if( PidTargetVelocity < Mot1PidApproachMinimumVelocity )
                PidTargetVelocity = Mot1PidApproachMinimumVelocity;
        }
        else if( (remainingPulseNum < 50) && (PidTargetVelocity > 50) ) {
            PidTargetVelocity = 50;
        }
        Pwm = Mot1VelocityFeedbackControl(PidTargetVelocity, Mot1.CurVelocity);
        if( (PidTargetVelocity != 0) && ((Mot1.Reset == 0) || (Mot1.Step == 3)) ) {
            MinimumPwm = Mot1PidHostMinimumPwm;
            if( (approachPulseNum != 0) &&
                (Mot1.TarPulseNum >= 2 * approachPulseNum) &&
                (remainingPulseNum < approachPulseNum) ) {
                MinimumPwm = approachMinimumPwm +
                    (Mot1PidHostMinimumPwm - approachMinimumPwm) *
                    remainingPulseNum / approachPulseNum;
                MaximumPwm = approachMaximumPwm +
                    (c_MaxPwmDuty - approachMaximumPwm) *
                    remainingPulseNum / approachPulseNum;
                if( Pwm > MaximumPwm ) Pwm = MaximumPwm;
            }
            if( Pwm < MinimumPwm ) Pwm = MinimumPwm;
            /* 回拉补偿单独用小力窗口，力量更小，避免回拉时又冲过目标。 */
            if( Mot1PosCorrBusy != 0 ) {
                if( Pwm > Mot1PosCorrMaximumPwm ) Pwm = Mot1PosCorrMaximumPwm;
                if( Pwm < Mot1PosCorrMinimumPwm ) Pwm = Mot1PosCorrMinimumPwm;
            }
        }
    }
    else {
        Pwm = Mot1VelocityFeedbackControl(0, Mot1.CurVelocity);
    }
    TIM1_SetDuty_CH3(Pwm);
#if 1
    printf("M1.TarVel =%d\n", Mot1.TarVelocity);
    printf("M1.CurVel =%d\n", Mot1.CurVelocity);
    printf("M1.PulseNum =%d\n", Mot1.PulseNum);
    printf("M1.CurAngle=%d\n", Mot1.CurAngle);
    printf("M1.PulseNumPerMin=%d\n", Mot1.PulseNumPerMin);
    printf("pwm =%d\n", Pwm);
    if( Mot1.Action == motor_backward )
        printf("M1.Reset=%d Step=%d Hall1Adc=%d\n", Mot1.Reset, Mot1.Step, Adc.Mot1.Hall1);
#endif
}

u8 Mot1ActionState(void)
{
		return Mot1.Action;
}	
u16 Mot1ExcuteTime(void)
{
		return Mot1.PidTimerB;
}	
void Mot1MoveStop(void)
{
		TIM1_SetDuty_CH3(0);
		Mot1.Action = motor_stop;
		Mot1DisablePidControl();
}
void Mot1MoveBrake(void)
{
		if( Mot1.Action == motor_forward ) {
			Mot1.BrakeType = pulse_forward_brake;
			Mot1DisablePidControl();
			Mot1.Action = motor_brake;
			printf("m1pulsebrake\n");
		}	
		else if( Mot1.Action == motor_backward ) {
			Mot1.BrakeType = pulse_backward_brake;
			Mot1DisablePidControl();
			Mot1.Action = motor_brake;
			printf("m1pulsebrake\n");
		}	
}	
void Mot1MoveForward( u16 TarPosition, u16 Velocity )
{
		printf("m1 forward\n");
	/* 普通运动开始时清空回拉计数；回拉运动本身保留计数，保证次数上限有效。 */
	if( Mot1PosCorrBusy == 0 ) Mot1PosCorrTimes = 0;
		//Mot1DirPin = NegativeMove;
			GPIO_SetBits(Mot1DirIO, Mot1DirPin);	
	
		Mot1.TarPosition = TarPosition;
	  Mot1.TarVelocity = Velocity;
		
		printf("M1.CurPos=%d\n", Mot1.CurPosition);
		printf("M1.TarPos=%d\n", Mot1.TarPosition);
	
		if(  Mot1.TarPosition >  Mot1.CurPosition )	Mot1.TarPulseNum =  Mot1.TarPosition -  Mot1.CurPosition;
	  else Mot1.TarPulseNum = 0;
		
		printf("M1.TarPulseNum=%d\n", Mot1.TarPulseNum);
		if( Mot1.TarPulseNum != 0 ) {
		
			Mot1EnablePidControl();
			Mot1.Action = motor_forward;
		
		}
			Mot1.PidTimerB = 0;
}

void Mot1MoveBackward( u16 TarPosition, u16 Velocity)
{
		printf("m1 backward\n");
	/* 普通运动开始时清空回拉计数；回拉运动本身保留计数，保证次数上限有效。 */
	if( Mot1PosCorrBusy == 0 ) Mot1PosCorrTimes = 0;
		//Mot1DirPin = PositiveMove;
	  GPIO_ResetBits(Mot1DirIO, Mot1DirPin);
	
		Mot1.TarPosition = TarPosition;
	  Mot1.TarVelocity = Velocity;
		
		printf("M1.CurPos=%d\n", Mot1.CurPosition);
		printf("M1.TarPos=%d\n", Mot1.TarPosition);
	
		if(  Mot1.CurPosition >  Mot1.TarPosition )	Mot1.TarPulseNum =  Mot1.CurPosition -  Mot1.TarPosition;
	  else Mot1.TarPulseNum = 0;
	
		printf("M1.TarPulseNum=%d\n", Mot1.TarPulseNum);
		
		if( Mot1.TarPulseNum != 0 ) {
			
			Mot1EnablePidControl();
			Mot1.Action = motor_backward;
		}
		Mot1.PidTimerB = 0;
}

/**
 * 函数作用：处理电机1运动、限位刹车，并在编码器连续300毫秒无变化后更新位置和角度。
 */
void Mot1ActionHandle(void)
{
	if( Mot1.Action == motor_forward ) {
		
		Mot1VelocityControl();
		
		if( Mot1.PulseNum >= Mot1.TarPulseNum )	{
			Mot1.BrakeType = pulse_forward_brake;
			Mot1DisablePidControl();
			Mot1.Action = motor_brake;
			printf("m1pulsebrake\n");
		}	
		else if( Mot1ReadRightHallState() != 0 ){

			Mot1.BrakeType = hall_forward_brake;
			Mot1DisablePidControl();
			Mot1.Action = motor_brake;
			printf("m1hallbrake\n");
		}		
	}		
	else if( Mot1.Action == motor_backward ) {
		
		Mot1VelocityControl();

		if( Mot1.PulseNum >= Mot1.TarPulseNum )	{
			
			Mot1.BrakeType = pulse_backward_brake;
			Mot1DisablePidControl();
			Mot1.Action = motor_brake;
			printf("m1pulsebrake\n");
		}	
		else if( Mot1ReadLeftHallState() != 0 ){
			
			Mot1.BrakeType = hall_backward_brake;
			Mot1DisablePidControl();
			Mot1.Action = motor_brake;
			printf("m1hallbrake\n");
		}		
	}		
	else if( Mot1.Action == motor_brake )  {
		
			if( Mot1.PulseDebB >= Mot1StopDebTicks )	{
				
				printf("m1 stop\n");
				printf("M1.PulseNum =%d\n ",Mot1.PulseNum );
				printf("M1.TarPulseNum =%d\n ",Mot1.TarPulseNum );
				//Mot1.ErrPulseNum = Mot1.PulseNum - Mot1.TarPulseNum;
				//printf("Mot1.ErrPulseNum = %d\n",Mot1.ErrPulseNum);

				if( Mot1.BrakeType == pulse_forward_brake ) {
					Mot1.CurPosition += Mot1.PulseNum;
					if( Mot1.CurPosition > Mot1.MaxPulseNum ) Mot1.CurPosition = Mot1.MaxPulseNum;
				}
				else if( Mot1.BrakeType == hall_forward_brake ) {
					
					Mot1.CurPosition = Mot1.MaxPulseNum;
					
				}
				else if( Mot1.BrakeType == pulse_backward_brake ) {
					
						if( Mot1.CurPosition > Mot1.PulseNum )	Mot1.CurPosition -= Mot1.PulseNum;
					 else Mot1.CurPosition = 0;
				}	
				else if( Mot1.BrakeType == hall_backward_brake ) {
					
					Mot1.CurPosition = 0;					
					
				}	
				printf("M1.CurPos =%d\n ",Mot1.CurPosition );
				printf("M1.CurAngle=%d\n", Mot1.CurAngle);
				if( Mot1.MaxPulseNum != 0 )
					Mot1.CurAngle = (u32)Mot1.CurPosition*(c_MaxAngle*10)/Mot1.MaxPulseNum;
				else Mot1.CurAngle = 0;
				Mot1.CurVelocity = 0;
				Mot1.Action = motor_stop;				
				/* 停稳后按“目标位置-当前记录位置”的残差做一次低速回拉补偿。 */
				Mot1PositionCorrectionHandle();
		}		
	}		
}	

/**
 * 函数作用：将电机1目标角度换算为脉冲位置，并按当前位置选择运动方向。
 * Angle：目标角度，单位为0.1度。
 * speed：目标运动速度。
 */
void Mot1SetAngleSpeed( u16 Angle, u16 speed )
{
		u32 sum; // 目标角度乘标定总脉冲数的中间量。
	
	 if( Angle >= (c_MaxAngle*10) ) Angle=c_MaxAngle*10;
	
		Mot1.TarVelocity = speed;
	
		printf("M1.PulseNumPerAngle =%d\n", Mot1.PulseNumPerAngle );
	
		sum = (u32)Angle*Mot1.MaxPulseNum;
	
		//printf("sum =%d\n", sum );
		if( Mot1.MaxPulseNum != 0 ) Mot1.TarPosition = sum/(c_MaxAngle*10);
		else Mot1.TarPosition = 0;
		printf("M1.TarPos =%d\n", Mot1.TarPosition );
	
	if( Mot1.TarPosition > Mot1.CurPosition ) {
			
			Mot1MoveForward( Mot1.TarPosition, speed ); 
		}	
		else if( Mot1.TarPosition < Mot1.CurPosition ) {
			
			Mot1MoveBackward( Mot1.TarPosition, speed ); 
		}	
}	
void Move1SetTask( u16 Angle, u16 Speed )
{
	if( Mot1.Reset != 0 ) return ;
	if( ( Angle == Move1.Angle )&&( Speed == Move1.Speed ) )  return ;
	Move1.Angle = Angle;
	Move1.Speed = Speed;
	printf("m1 task angle =%d\n",Angle );
	printf("m1 task speed =%d\n",Speed );
	Move1.Step = 0;
	Move1.En = Enable;
}	
void Mot1MoveTask(void)
{
		if( Move1.En == 0 ) return ;
		switch( Move1.Step )
		{
				case 0:
							if( Mot1ActionState() != motor_stop ) {
								Mot1MoveBrake();
							}	
							Move1.Step++;
							break;
				case 1:
							if( Mot1ActionState() == motor_stop ) {
								Mot1SetAngleSpeed( Move1.Angle, Move1.Speed );
								Move1.En = 0;
							}	
							break;
		}		
}	

void Mot1MoveService(void)
{
		u32 total;
		
		Mot1LeftHallScan();
	
		Mot1RightHallScan();
	
		Mot1ActionHandle();
	
		if( Mot1.Reset ) {
			
			switch( Mot1.Step )
			{
				case 0:
							Mot1SetCurrentPosition(20000);
							Mot1MoveBackward( 0, 100 );
							Mot1.Step++;
							break;
				case 1:
							if( Mot1ActionState() == motor_stop ){
								Mot1SetCurrentPosition(0);
								Mot1MoveForward( 20000, 100 ); 
								Mot1.Step++;
							}		
							else if( Mot1ExcuteTime() > c_MoveTimeOver ){
									//printf("mot1 move timer over \n");
									Mot1MoveStop();
									Mot1.Faultflag = Hall1Fault;
									Mot1.Step = 4;	
							}			
							break;
				case 2:
							if( Mot1ActionState() == motor_stop ){
								
								if( Mot1.PulseNum < C_VerifyMinimumPulseNum ) {
									Mot1.Faultflag = MotorFault;
									Mot1.Step = 4;
								}
								else {	
									Mot1.MaxPulseNum = Mot1.PulseNum;
									printf("M1.MaxPulseNum =%d\n", Mot1.MaxPulseNum );
									Mot1.CurPosition =  Mot1.MaxPulseNum; 
									total = (u32)Mot1.MaxPulseNum*c_PulseScaleFactor;
									Mot1.PulseNumPerAngle = total / c_MaxAngle;
									printf("M1.PulseNumPerAngle =%d\n", Mot1.PulseNumPerAngle );
									
									delay_ms(500);
									Mot1MoveBackward( Mot1.MaxPulseNum/2, 300 ); 
									Mot1.Step = 3;
								}	
							}		
							else if( Mot1ExcuteTime() > c_MoveTimeOver )	{
								//	printf("mot1 move timer over\n");
									Mot1MoveStop();
									Mot1.Faultflag = Hall2Fault;
									Mot1.Step = 4;
							}										
							break;
				case 3:
							if( Mot1ActionState() == motor_stop ){
									Mot1.En = 1;
									Mot1.Reset = 0;
							}	
							break;
				case 4:
							printf("m1 fault =%d\n", Mot1.Faultflag );
							Mot1.Step++;
							break;
			}		
		}		
		else {
			
			if( Mot1.En ) {
				
				Mot1MoveTask();
			
			}	
				
		}	
}         

/**
 * 函数作用：把0.1度单位的角度偏差换算成脉冲数，供回拉死区判断使用。
 * Angle：角度偏差，单位0.1度。
 * 返回值：换算出的脉冲数；行程还没标定时返回0。
 */
static u16 Mot1AngleToPulse(u16 Angle)
{
	u32 sum; // 角度乘标定行程换算的中间量

	if( Mot1.MaxPulseNum == 0 ) return 0;
	sum = (u32)Angle * Mot1.MaxPulseNum;
	return (u16)(sum / (c_MaxAngle * 10));
}

/**
 * 函数作用：一次运动停稳后，用“目标位置-当前记录位置”的残差判断要不要低速反向回拉，
 *           只对脉冲制动生效；霍尔限位由开关定位，不做回拉。
 */
static void Mot1PositionCorrectionHandle(void)
{
	u8 brakeType;  // 本次停稳前的制动类型，用来区分脉冲制动和霍尔制动
	u16 deadband;  // 允许的残余脉冲死区，小于它就认为已经到位
	u16 maxCorr;   // 单次回拉允许的最大脉冲数，超过就判为异常
	u16 residual;  // 目标位置与当前记录位置之间差的脉冲数
	u16 stepPulse; // 本次实际回拉的脉冲数，按残差比例缩小。
	u16 targetPos; // 回拉前保存的真正目标位置，避免被中间目标覆盖。

	brakeType = Mot1.BrakeType;
	Mot1PosCorrBusy = 0;

	if( Mot1PosCorrEnable == 0 ) return;
	if( (brakeType != pulse_forward_brake) && (brakeType != pulse_backward_brake) ) return;
	/* 复位找行程阶段不参与回拉，只处理正常运行和复位回中(Step3)。 */
	if( (Mot1.Reset != 0) && (Mot1.Step != 3) ) return;
	if( Mot1.MaxPulseNum == 0 ) return;
	if( Mot1PosCorrTimes >= Mot1PosCorrMaxTimes ) return;

	deadband = Mot1AngleToPulse(Mot1PosCorrDeadbandAngle);
	if( deadband < 2 ) deadband = 2; /* 行程标定值小时不要按1个脉冲判断，避免来回抖。 */
	maxCorr = Mot1AngleToPulse(Mot1PosCorrMaxAngle);
	if( maxCorr < deadband ) maxCorr = deadband;

	if( Mot1.CurPosition > Mot1.TarPosition ) residual = Mot1.CurPosition - Mot1.TarPosition;
	else residual = Mot1.TarPosition - Mot1.CurPosition;

	if( residual < deadband ) return; /* 已经落在允许偏差内，不再动作。 */
	if( residual > maxCorr ) return;  /* 偏差过大不像是停机制动过冲，不做自动回拉。 */

	targetPos = Mot1.TarPosition;
	/* 只回拉残差的一部分：停机制动还会继续冲一点，按比例回拉才不会再冲过目标。 */
	stepPulse = (u16)(((u32)residual * Mot1PosCorrRatioPercent) / 100);
	if( stepPulse == 0 ) stepPulse = 1;
	Mot1PosCorrTimes++;
	Mot1PosCorrBusy = 1;
	printf("m1 poscorr residual=%d step=%d\n", residual, stepPulse);
	if( Mot1.CurPosition > targetPos ) {
		/* 记录位置比目标大，朝位置减小的方向(Backward)小幅回拉。 */
		Mot1MoveBackward( Mot1.CurPosition - stepPulse, Mot1PosCorrVelocity );
	}
	else {
		/* 记录位置比目标小，朝位置增大的方向(Forward)小幅回拉。 */
		Mot1MoveForward( Mot1.CurPosition + stepPulse, Mot1PosCorrVelocity );
	}
	/* Move函数会把TarPosition改成中间目标，这里改回真正的目标位置，供下一次残差判断。 */
	Mot1.TarPosition = targetPos;
}
