/*******************************************************************************

	include file
*******************************************************************************/
#include "sys.h"

/*******************************************************************************
	constant define
*******************************************************************************/
#define c_MaxAngle 	65								//最大角65度
#define c_PulseScaleFactor 10				//10倍
#define c_AngleScaleFactor 10					//10倍
#define c_RadianScaleFactor 10				//10倍
#define c_SpeedScaleFactor  20				//10倍
#define c_DefaultTotalPulseNum	3600	//总的脉冲数
#define c_DefaultPulseNumPerAngle 20*c_PulseScaleFactor	//每度对应的脉冲数
#define c_MoveTimeOver 30000					//30s,马达不转无脉冲输出时间
#define Mot3TriggerAdc 2200
//-----------------------------------------------------------------
//	ram 
//-----------------------------------------------------------------

Motor3St Mot3;
Move3St Move3;

void Mot3Init(void)
{
	Mot3.Reset = 1;
	Mot3.Action = motor_stop;
	Mot3.Step = 0;
	Mot3.PidTimer = 0;
	Mot3.PidTimerB = 0;
	Mot3.PidEn = 0;
	Mot3.PulseSta = GPIO_ReadInputDataBit( Mot3PulseIO, Mot3PulsePin );
	Mot3.PulseDeb = 0;
	Mot3.PulseDebB = 0;
	Mot3.PulseNum = 0;
	Mot3.PulseNumB = 0;
	Mot3.ErrPulseNum = 0;
	Mot3.BrakeType = 0;
	Mot3.TarPulseNum = 0;
	Mot3.MaxPulseNum = c_DefaultTotalPulseNum;
	Mot3.PulseNumPerAngle = c_DefaultPulseNumPerAngle;
	Mot3.TarVelocity = 0;
	Mot3.CurVelocity = 0;
	Mot3.TarPosition = 0;
	Mot3.CurPosition = 0;	
	Mot3.CurAngle = 0;
	Mot3.TarAngle = 0;
	Mot3.Hall1Sta = 0;
	Mot3.Hall1Deb = 0;
	Mot3.Hall2Sta = 0;
	Mot3.Hall2Deb = 0;
	Mot3.Faultflag = 0;
	Mot3.En = 0;
	
	Move3.En = 0;
	Move3.Step = 0;
}

void Mot3UpHallScan(void)
{
	if( Mot3.Hall1Sta == 0 ) {
		if( Adc.Mot3.Hall1 >= ( Mot3TriggerAdc+100 ) ) {
			if( Mot3.Hall1Deb >= 20 )	{	
				Mot3.Hall1Sta = 1;
			}	
		}	
		else {
			Mot3.Hall1Deb = 0;
		}	
  }
  else {
		if( Adc.Mot3.Hall1 < ( Mot3TriggerAdc+100 ) ) {
			if( Mot3.Hall1Deb >= 20 )	{	
				Mot3.Hall1Sta = 0;
			}	
		}	
		else {
			Mot3.Hall1Deb = 0;
		}	
  }
}

u8 Mot3ReadUpHallState(void)
{
	return Mot3.Hall1Sta;
}

void Mot3DownHallScan(void)
{
	if( Mot3.Hall2Sta == 0 ) {
		if( Adc.Mot3.Hall2 >= ( Mot3TriggerAdc+100 ) ) {
			if( Mot3.Hall2Deb >= 20 )	{	
				Mot3.Hall2Sta = 1;
			}	
		}	
		else {
			Mot3.Hall2Deb = 0;
		}	
  }
  else {
		if( Adc.Mot3.Hall2 < ( Mot3TriggerAdc+100) ) {
			if( Mot3.Hall2Deb >= 20 )	{	
				Mot3.Hall2Sta = 0;
			}	
		}	
		else {
			Mot3.Hall2Deb = 0;
		}	
  }
}	

u8 Mot3ReadDownHallState(void)
{
	return Mot3.Hall2Sta;
}

/* 增量式速度PID，增益按百分之一缩放，Ki按每个50ms控制周期累加。 */
// PID定点缩放系数，输出除以此值换算为PWM占空比。
#define Mot3PidScale          100
#define Mot3VelocityKp        250 // 速度误差变化时的比例响应系数。
#define Mot3VelocityKi        4 // 上位机运行时持续速度误差的积分系数。
#define Mot3VelocityResetKi   2 // 复位时持续速度误差的积分系数。
// 增量式PID微分系数，作用于误差二阶差分。
#define Mot3VelocityKd        50
// 复位状态下PID输出的初始PWM占空比。
#define Mot3PidStartDuty      1500
// 非复位状态下PID输出的初始PWM占空比。
#define Mot3PidCruiseDuty     2500
// 复位状态下目标速度非零时PID输出的最低PWM占空比。
#define Mot3PidMinimumPwm     1500
#define Mot3PidHostMinimumPwm 2500 // 上位机控制时的最低PWM占空比。
// 剩余脉冲小于此值时进入减速阶段。
#define Mot3PidSlowDownPulseNum 50
// 减速阶段目标速度下限，单位同TarVelocity和CurVelocity。
#define Mot3PidMinimumTargetVelocity 50
/* 末端回拉补偿：停稳后按残差再低速往回走一小段，用来抵消制动过冲留下的固定偏差。 */
#define Mot3PosCorrEnable        1    // 1：启用末端回拉补偿；0：关闭，方便做对比试验。
#define Mot3PosCorrDeadbandAngle 10   // 回拉死区，单位0.1度(即1.0度)；残差小于这个值就不再回拉。
#define Mot3PosCorrMaxAngle      250  // 单次回拉允许的最大残差，单位0.1度(即25.0度)；超过判为异常不回拉。
#define Mot3PosCorrVelocity      50   // 回拉运动的目标速度，取小值让末端走得更慢。
#define Mot3PosCorrMaxTimes      4    // 一次目标最多回拉几次，残差大时逐步逼近，不再一次拉过。
#define Mot3PosCorrTimeOver      2000 // 单次回拉最长持续时间，单位毫秒；超时按普通制动结束。
#define Mot3PosCorrRatioPercent  50   // 每次只回拉残差的百分比，留出停机制动的过冲余量。
#define Mot3PosCorrMinimumPwm    1300 // 回拉运动专用PWM下限，比正常运动小很多。
#define Mot3PosCorrMaximumPwm    1600 // 回拉运动专用PWM上限，限制回拉力量避免再冲过目标。

static int32_t Mot3PidPreviousError = 0;
static int32_t Mot3PidPreviousPreviousError = 0;
static int32_t Mot3PidOutputScaled = 0;
/* 末端回拉补偿的运行状态和函数声明 */
static u8 Mot3PosCorrTimes = 0;  // 本次目标已经回拉的次数。
static u8 Mot3PosCorrBusy = 0;   // 1：当前这次运动就是回拉补偿运动。
static u16 Mot3PosCorrLastResidual = 0; // 上一次回拉后的残差，用来判断这次回拉有没有效果。
static u16 Mot3AngleToPulse(u16 Angle);         // 把0.1度角度偏差换算成脉冲数。
static void Mot3PositionCorrectionHandle(void); // 停稳后按残差判断是否还要回拉。
void Mot3MoveBrake(void);                       // 回拉超时需要制动，函数定义在后面先声明。

/**
 * 函数作用：清除电机3速度PID的历史误差，并设置本次启动占空比。
 * InitialPwm：本次运动开始时的PWM占空比，范围为0到最大PWM。
 */
static void Mot3PidReset(u16 InitialPwm)
{
	Mot3PidPreviousError = 0;
	Mot3PidPreviousPreviousError = 0;
	if( InitialPwm > c_MaxPwmDuty ) InitialPwm = c_MaxPwmDuty;
	Mot3PidOutputScaled = (int32_t)InitialPwm * Mot3PidScale;
}

/**
 * 函数作用：根据目标速度和当前速度执行电机3增量式PID并输出限幅PWM。
 * TargetVelocity：本周期目标速度，单位与Mot3.CurVelocity相同。
 * CurrentVelocity：由电机3编码脉冲计算出的当前速度。
 * 返回值：范围为0到最大PWM的占空比。
 */
static u16 Mot3VelocityFeedbackControl(u16 TargetVelocity, u16 CurrentVelocity)
{
	int32_t Error;
	int32_t DeltaError;
	int32_t SecondDifference;
	int32_t DeltaOutputScaled;
	int32_t OutputLimitScaled;

	if( TargetVelocity == 0 ) {
		Mot3PidReset(0);
		return 0;
	}

	Error = (int32_t)TargetVelocity - (int32_t)CurrentVelocity;
	DeltaError = Error - Mot3PidPreviousError;
	SecondDifference = Error - 2 * Mot3PidPreviousError + Mot3PidPreviousPreviousError;
	/* 复位沿用较小积分系数，正常运行使用试调积分系数。 */
	DeltaOutputScaled = Mot3VelocityKp * DeltaError +
		((Mot3.Reset != 0) ? Mot3VelocityResetKi : Mot3VelocityKi) * Error +
		Mot3VelocityKd * SecondDifference;
	OutputLimitScaled = (int32_t)c_MaxPwmDuty * Mot3PidScale;
	Mot3PidOutputScaled += DeltaOutputScaled;
	if( Mot3PidOutputScaled < 0 ) Mot3PidOutputScaled = 0;
	if( Mot3PidOutputScaled > OutputLimitScaled ) Mot3PidOutputScaled = OutputLimitScaled;

	Mot3PidPreviousPreviousError = Mot3PidPreviousError;
	Mot3PidPreviousError = Error;
	return (u16)(Mot3PidOutputScaled / Mot3PidScale);
}

/**
 * 函数作用：检测电机3编码脉冲；仅在两个限位霍尔均未触发时累计脉冲，与复位标定口径一致。
 */
void Mot3Encode(void)
{
	u8 state; // 本次读取的编码器电平。
	state = GPIO_ReadInputDataBit( Mot3PulseIO, Mot3PulsePin) ;
	if( state != Mot3.PulseSta ) {
		Mot3.PulseDebB = 0;
		Mot3.PulseDeb++;
		if( Mot3.PulseDeb >= 10 ) {
				
			Mot3.PulseSta = state;
      if( state != 0 ) {
					 if( (Mot3.Hall1Sta == 0) && (Mot3.Hall2Sta == 0) ) {
							Mot3.PulseNum++;
							Mot3.PulseNumB++;
					 } 
        }    
     }
  }
  else
  {
		Mot3.PulseDeb = 0;
		if( Mot3.PulseDebB < C_MotorStopDeb ) Mot3.PulseDebB++;
  }
}
u16 Mot3ReadEncode(void)
{
	u16 PulseNum;
	__disable_irq();
	PulseNum = Mot3.PulseNumB;
	Mot3.PulseNumB = 0;
	__enable_irq();
	return PulseNum; 
}	
void Mot3ClearEncode(void)
{
	Mot3.PulseNum = 0;
	Mot3.PulseNumB = 0;
	Mot3.BrakeType = 0;
	Mot3.PidTimer = 0;
	Mot3.PidTimerB = 0;
}	
/**
 * 函数作用：启动电机3速度闭环前清零测速脉冲并重置PID状态。
 */
void Mot3EnablePidControl(void)
{
	Mot3ClearEncode();
	Mot3.CurVelocity = 0;
	if( Mot3.Reset == 1 ) Mot3PidReset(Mot3PidStartDuty);
	else Mot3PidReset(Mot3PidCruiseDuty);
	Mot3.PidEn = Enable;
}
/**
 * 函数作用：关闭电机3速度PID并清零PWM和历史控制状态。
 */
void Mot3DisablePidControl(void)
{
	TIM1_SetDuty_CH2(0);
	Mot3.PidEn = Disable;
	Mot3PidReset(0);
}
void Mot3SetCurrentPosition( u16 num )
{
	Mot3.CurPosition = num;
}	
void Mot3SetTargetPosition( u16 num )
{
	Mot3.TarPosition = num;
}	
/**
 * 函数作用：按固定周期更新电机3位置、编码器速度和PID PWM输出。
 */
void Mot3VelocityControl(void)
{
	u16 Pwm, PidTargetVelocity;
	u32 sum, encode_count, remainingPulseNum;

	if( Mot3.PidEn == 0 ) return;
	/* 回拉补偿超时保护：机构卡住或走不动时按普通制动收尾，避免一直停在半路。 */
	if( (Mot3PosCorrBusy != 0) && (Mot3.PidTimerB > Mot3PosCorrTimeOver) ) {
		Mot3PosCorrTimes = Mot3PosCorrMaxTimes;
		Mot3MoveBrake();
		return;
	}
	if( Mot3.PidTimer < C_PID_RATE ) return;

	Mot3.PidTimer = Mot3.PidTimer - C_PID_RATE;
	sum = 0;
	if( Mot3.Action == motor_forward ) {
		sum = Mot3.CurPosition + Mot3.PulseNum;
		if( sum > Mot3.MaxPulseNum ) sum = Mot3.MaxPulseNum;
	}
	else if( Mot3.Action == motor_backward ) {
		if( Mot3.CurPosition > Mot3.PulseNum ) sum = Mot3.CurPosition - Mot3.PulseNum;
	}
	if( (sum != 0) && (Mot3.MaxPulseNum != 0) ) {
		// 按完整标定行程换算，避免每度脉冲整数截断造成角度上报偏大。
		Mot3.CurAngle = (sum * c_MaxAngle * 10) / Mot3.MaxPulseNum;
	}
	else Mot3.CurAngle = 0;

	encode_count = Mot3ReadEncode() * (1000 / C_PID_RATE);
	Mot3.PulseNumPerMin = encode_count * 60;
	encode_count = (u32)(encode_count * c_RadianScaleFactor * c_PulseScaleFactor);
	if( encode_count != 0 ) Mot3.CurVelocity = (u32)(encode_count / Mot3.PulseNumPerAngle);
	else Mot3.CurVelocity = 0;

	if( Mot3.TarPulseNum > Mot3.PulseNum ) {
		remainingPulseNum = Mot3.TarPulseNum - Mot3.PulseNum;
		PidTargetVelocity = Mot3.TarVelocity;
		if( (remainingPulseNum < Mot3PidSlowDownPulseNum) && (PidTargetVelocity > Mot3PidMinimumTargetVelocity) ) {
			PidTargetVelocity = (u16)(((u32)Mot3.TarVelocity * remainingPulseNum) / Mot3PidSlowDownPulseNum);
			if( PidTargetVelocity < Mot3PidMinimumTargetVelocity ) PidTargetVelocity = Mot3PidMinimumTargetVelocity;
		}
		Pwm = Mot3VelocityFeedbackControl(PidTargetVelocity, Mot3.CurVelocity);
		/* 目标速度非零时，复位使用原下限，上位机运动全程使用运行下限。 */
		if( (Mot3PosCorrBusy != 0) && (PidTargetVelocity != 0) ) {
			/* 回拉补偿单独用小力窗口，力量更小，避免回拉时又冲过目标。 */
			if( Pwm > Mot3PosCorrMaximumPwm ) Pwm = Mot3PosCorrMaximumPwm;
			if( Pwm < Mot3PosCorrMinimumPwm ) Pwm = Mot3PosCorrMinimumPwm;
		}
		else if( (Mot3.Reset != 0) && (PidTargetVelocity != 0) &&
			(Pwm < Mot3PidMinimumPwm) ) {
			Pwm = Mot3PidMinimumPwm;
		}
		else if( (Mot3.Reset == 0) && (PidTargetVelocity != 0) &&
			(Pwm < Mot3PidHostMinimumPwm) ) {
			Pwm = Mot3PidHostMinimumPwm;
		}
	}
	else {
		Pwm = Mot3VelocityFeedbackControl(0, Mot3.CurVelocity);
	}
	TIM1_SetDuty_CH2(Pwm);
#if 1
	printf("M3.TarVel =%d\n", Mot3.TarVelocity);
	printf("M3.CurVel =%d\n", Mot3.CurVelocity);
	printf("M3.PulseNum =%d\n", Mot3.PulseNum);
	printf("M3.CurAngle=%d\n", Mot3.CurAngle);
	printf("M3.PulseNumPerMin=%d\n", Mot3.PulseNumPerMin);
	printf("pwm =%d\n", Pwm);
#endif
}		


u8 Mot3ActionState(void)
{
		return Mot3.Action;
}	
u16 Mot3ExcuteTime(void)
{
		return Mot3.PidTimerB;
}	
void Mot3MoveStop(void)
{
		TIM1_SetDuty_CH2(0);
		Mot3.Action = motor_stop;
		Mot3DisablePidControl();
}
void Mot3MoveBrake(void)
{
		if( Mot3.Action == motor_forward ) {
			Mot3.BrakeType = pulse_forward_brake;
			Mot3DisablePidControl();
			Mot3.Action = motor_brake;
			printf("M3pulsebrake\n");
		}	
		else if( Mot3.Action == motor_backward ) {
			Mot3.BrakeType = pulse_backward_brake;
			Mot3DisablePidControl();
			Mot3.Action = motor_brake;
			printf("M3pulsebrake\n");
		}	
}	
void Mot3MoveForward( u16 TarPosition, u16 Velocity )
{
		printf("M3 forward\n");
	/* 普通运动开始时清空回拉计数；回拉运动本身保留计数，保证次数上限有效。 */
	if( Mot3PosCorrBusy == 0 ) Mot3PosCorrTimes = 0;
		//Mot3DirPin = PositiveMove;
	  GPIO_SetBits( Mot3DirIO, Mot3DirPin );
		 
		Mot3.TarPosition = TarPosition;
	  Mot3.TarVelocity = Velocity;
		
		printf("M3.CurPos=%d\n", Mot3.CurPosition);
		printf("M3.TarPos=%d\n", Mot3.TarPosition);
	
		if(  Mot3.TarPosition >  Mot3.CurPosition )	Mot3.TarPulseNum =  Mot3.TarPosition -  Mot3.CurPosition;
	  else Mot3.TarPulseNum = 0;
		
		printf("M3.TarPulseNum=%d\n", Mot3.TarPulseNum);
		if( Mot3.TarPulseNum != 0 ) {
		
			Mot3EnablePidControl();
			Mot3.Action = motor_forward;
		
		}
			
}

void Mot3MoveBackward( u16 TarPosition, u16 Velocity)
{
		printf("M3 backward\n");
		/* 普通运动开始时清空回拉计数；回拉运动本身保留计数，保证次数上限有效。 */
		if( Mot3PosCorrBusy == 0 ) Mot3PosCorrTimes = 0;
		//Mot3DirPin =  NegativeMove;
		
		GPIO_ResetBits( Mot3DirIO, Mot3DirPin );
	
		Mot3.TarPosition = TarPosition;
	  Mot3.TarVelocity = Velocity;
		
		printf("M3.CurPos=%d\n", Mot3.CurPosition);
		printf("M3.TarPos=%d\n", Mot3.TarPosition);
	
		if(  Mot3.CurPosition >  Mot3.TarPosition )	Mot3.TarPulseNum =  Mot3.CurPosition -  Mot3.TarPosition;
	  else Mot3.TarPulseNum = 0;
	
		printf("M3.TarPulseNum=%d\n", Mot3.TarPulseNum);
		
		if( Mot3.TarPulseNum != 0 ) {
			
			Mot3EnablePidControl();
			Mot3.Action = motor_backward;
		}
		
}

/**
 * 函数作用：处理电机3运动、限位刹车，并在停稳后更新当前位置和反馈角度。
 */
void Mot3ActionHandle(void)
{
	if( Mot3.Action == motor_forward ) {
		
		Mot3VelocityControl();
		
		if( Mot3.PulseNum >= Mot3.TarPulseNum )	{
			Mot3.BrakeType = pulse_forward_brake;
			Mot3DisablePidControl();
			Mot3.Action = motor_brake;
			printf("M3pulsebrake\n");
		}	
		else if( Mot3ReadDownHallState() != 0 ){

			Mot3.BrakeType = hall_forward_brake;
			Mot3DisablePidControl();
			Mot3.Action = motor_brake;
			printf("M3hallbrake\n");
		}		
	}		
	else if( Mot3.Action == motor_backward ) {
		
		Mot3VelocityControl();

		if( Mot3.PulseNum >= Mot3.TarPulseNum )	{
			
			Mot3.BrakeType = pulse_backward_brake;
			Mot3DisablePidControl();
			Mot3.Action = motor_brake;
			printf("M3pulsebrake\n");
		}	
		else if( Mot3ReadUpHallState() != 0 ){
			
			Mot3.BrakeType = hall_backward_brake;
			Mot3DisablePidControl();
			Mot3.Action = motor_brake;
			printf("M3hallbrake\n");
		}		
	}		
	else if( Mot3.Action == motor_brake )  {
		
			if( Mot3.PulseDebB >= C_MotorStopDeb )	{
				
				printf("M3 stop\n");
				printf("M3.PulseNum =%d\n",Mot3.PulseNum );
				printf("M3.TarPulseNum =%d\n ",Mot3.TarPulseNum );
				//Mot3.ErrPulseNum = Mot3.PulseNum - Mot3.TarPulseNum;
				//printf("Mot3.ErrPulseNum =%d\n",Mot3.ErrPulseNum);

				if( Mot3.BrakeType == pulse_forward_brake ) {
					Mot3.CurPosition += Mot3.PulseNum;
					if( Mot3.CurPosition > Mot3.MaxPulseNum ) Mot3.CurPosition = Mot3.MaxPulseNum;
				}
				else if( Mot3.BrakeType == hall_forward_brake ) {
					
					Mot3.CurPosition = Mot3.MaxPulseNum;
					
				}
				else if( Mot3.BrakeType == pulse_backward_brake ) {
					
						if( Mot3.CurPosition > Mot3.PulseNum )	Mot3.CurPosition -= Mot3.PulseNum;
					else Mot3.CurPosition = 0;
				}	
				else if( Mot3.BrakeType == hall_backward_brake ) {
					
					Mot3.CurPosition = 0;					
					
				}	
				printf("M3.CurPos=%d\n ",Mot3.CurPosition );
				printf("M3.CurAngle=%d\n", Mot3.CurAngle);
				if( Mot3.MaxPulseNum != 0 )
					Mot3.CurAngle = (u32)Mot3.CurPosition*(c_MaxAngle*10)/Mot3.MaxPulseNum;
				else Mot3.CurAngle = 0;
				Mot3.CurVelocity = 0;
				Mot3.Action = motor_stop;				
				/* 停稳后按“目标位置-当前记录位置”的残差做一次低速回拉补偿。 */
				Mot3PositionCorrectionHandle();
		}		
	}		
}	

/**
 * 函数作用：将电机3目标角度换算为脉冲位置，并按当前位置选择运动方向。
 * Angle：目标角度，单位为0.1度。
 * speed：目标运动速度。
 */
void Mot3SetAngleSpeed( u16 Angle, u16 speed )
{
		u32 sum; // 目标角度乘标定总脉冲数的中间量。
		Mot3.TarVelocity = speed;
	  if( Angle >= (c_MaxAngle*10) ) Angle=c_MaxAngle*10;
		printf("Angle =%d\n", Angle );
		printf("speed =%d\n", speed );
		printf("M3.PulseNumPerAngle =%d\n", Mot3.PulseNumPerAngle );
	
		sum = (u32)Angle*Mot3.MaxPulseNum;
	
		//printf("sum =%d\n", sum );
	
		if( Mot3.MaxPulseNum != 0 ) Mot3.TarPosition = sum/(c_MaxAngle*10);
		else Mot3.TarPosition = 0;
		printf("M3.TarPos=%d\n", Mot3.TarPosition );
	
	if( Mot3.TarPosition > Mot3.CurPosition ) {
			
			Mot3MoveForward( Mot3.TarPosition, speed ); 
		}	
		else if( Mot3.TarPosition < Mot3.CurPosition ) {
			
			Mot3MoveBackward( Mot3.TarPosition, speed ); 
		}	
}	
void Move3SetTask( u16 Angle, u16 Speed )
{
	if( Mot3.Reset != 0 ) return ;
	if( ( Angle == Move3.Angle )&&( Speed == Move3.Speed ) ) return ;
	Move3.Angle = Angle;
	Move3.Speed = Speed;
	printf("m3 task angle =%d\n",Angle );
	printf("m3 task speed =%d\n",Speed );
	Move3.Step = 0;
	Move3.En = Enable;
}	
void Mot3MoveTask(void)
{
		if( Move3.En == 0 ) return ;
		switch( Move3.Step )
		{
				case 0:
							if( Mot3ActionState() != motor_stop ) {
								Mot3MoveBrake();
							}	
							Move3.Step++;
							break;
				case 1:
							if( Mot3ActionState() == motor_stop ) {
								Mot3SetAngleSpeed( Move3.Angle, Move3.Speed );
								Move3.En = 0;
							}	
							break;
		}		
}	

/** 函数作用：首次执行两端标定，历史标定有效时只找基准限位后归位；无传入参数。 */
void Mot3MoveService(void)
{
		u32 total; // 实测全行程换算每角度脉冲的中间值。
		
		Mot3UpHallScan();
	
		Mot3DownHallScan();
	
		Mot3ActionHandle();
		
		#if Motor1Enable == 1
		if( Mot1.Reset ) return;
		#endif
	
		#if Motor2Enable == 1
		if( Mot2.Reset ) return;
		#endif
	
		if( Mot3.Reset ) {
			
			switch( Mot3.Step )
			{
				case 0:
							Mot3SetCurrentPosition(20000);
							Mot3MoveBackward( 0, 50 );
							Mot3.Step++;
							break;
				case 1:
							if( Mot3ActionState() == motor_stop ){
								if( Mot3.BrakeType != hall_backward_brake ) {
									Mot3.Faultflag = Hall1Fault;
									Mot3.Step = 4;
								}
								else {
									Mot3SetCurrentPosition(0);
									if( MotorCalibrationLoaded != 0 ) {
										// 历史标定有效：从零点侧限位（原反向运动对应上霍尔）按保存行程直接回中。
										Mot3MoveForward( Mot3.MaxPulseNum / 2, 50 );
										Mot3.Step = 3;
									}
									else {
										delay_ms(500);
										Mot3MoveForward( 20000, 50 );
										Mot3.Step++;
									}
								}
							}
							else if( Mot3ExcuteTime() > c_MoveTimeOver ){
									//printf("Mot3 move timer over \n");
									Mot3MoveStop();
									Mot3.Faultflag = Hall1Fault;
									Mot3.Step = 4;	
							}			
							break;
				case 2:
							if( Mot3ActionState() == motor_stop ){
								
								if( Mot3.BrakeType != hall_forward_brake ) {
									Mot3.Faultflag = Hall2Fault;
									Mot3.Step = 4;
								}
								else if( (Mot3.PulseNum < C_VerifyMinimumPulseNum) ) {
									Mot3.Faultflag = MotorFault;
									Mot3.Step = 4;
								}
								else {	
									Mot3.MaxPulseNum = Mot3.PulseNum;
									printf("M3.MaxPulseNum =%d\n", Mot3.MaxPulseNum );
									Mot3.CurPosition =  Mot3.MaxPulseNum; 
									total = (u32)Mot3.MaxPulseNum*c_PulseScaleFactor;
									Mot3.PulseNumPerAngle = total / c_MaxAngle;
									printf("M3.PulseNumPerAngle =%d\n", Mot3.PulseNumPerAngle );
									
									delay_ms(500);
									Mot3MoveBackward( Mot3.MaxPulseNum/2, 50 );
									Mot3.Step = 3;	
								}	
							}		
							else if( Mot3ExcuteTime() > c_MoveTimeOver )	{
									//printf("Mot3 move timer over\n");
									Mot3MoveStop();
									Mot3.Faultflag = Hall2Fault;
									Mot3.Step = 4;
							}										
							break;
				case 3:
							if( Mot3ActionState() == motor_stop ){
								// 归位被异常限位打断时不报成功，防止把未到中点的数据当成自检成功。
								if( (Mot3.BrakeType == ((MotorCalibrationLoaded != 0) ? pulse_forward_brake : pulse_backward_brake)) &&
					(Mot3.Hall1Sta == 0) && (Mot3.Hall2Sta == 0) ) {
									Mot3.En = 1;
									Mot3.Reset = 0;
								}
								else {
									Mot3.Faultflag = MotorFault;
									Mot3.Step = 4;
								}
							}
							else if( Mot3ExcuteTime() > c_MoveTimeOver ) {
								Mot3MoveStop();
								Mot3.Faultflag = MotorFault;
								Mot3.Step = 4;
							}
							break;
				case 4:
							printf("M3 fault =%d\n", Mot3.Faultflag );
							Mot3.Step++;
							break;
			}		
		}		
		else {
			
			if( Mot3.En ) {
				
				Mot3MoveTask();
			
			}	
				
		}	
}         

/**
 * 函数作用：把0.1度单位的角度偏差换算成脉冲数，供回拉死区判断使用。
 * Angle：角度偏差，单位0.1度。
 * 返回值：换算出的脉冲数；行程还没标定时返回0。
 */
static u16 Mot3AngleToPulse(u16 Angle)
{
	u32 sum; // 角度乘标定行程换算的中间量

	if( Mot3.MaxPulseNum == 0 ) return 0;
	sum = (u32)Angle * Mot3.MaxPulseNum;
	return (u16)(sum / (c_MaxAngle * 10));
}

/**
 * 函数作用：一次运动停稳后，用“目标位置-当前记录位置”的残差判断要不要低速反向回拉，
 *           只对脉冲制动生效；霍尔限位由开关定位，不做回拉。
 */
static void Mot3PositionCorrectionHandle(void)
{
	u8 brakeType;  // 本次停稳前的制动类型，用来区分脉冲制动和霍尔制动
	u16 deadband;  // 允许的残余脉冲死区，小于它就认为已经到位
	u16 maxCorr;   // 单次回拉允许的最大脉冲数，超过就判为异常
	u16 residual;  // 目标位置与当前记录位置之间差的脉冲数
	u16 stepPulse; // 本次实际回拉的脉冲数，按残差比例缩小。
	u16 targetPos; // 回拉前保存的真正目标位置，避免被中间目标覆盖。
	u8  wasCorr;   // 上一次停稳是否来自回拉运动，用来判断回拉有没有效果。

	wasCorr = Mot3PosCorrBusy;
	brakeType = Mot3.BrakeType;
	Mot3PosCorrBusy = 0;

	if( Mot3PosCorrEnable == 0 ) return;
	if( (brakeType != pulse_forward_brake) && (brakeType != pulse_backward_brake) ) return;
	/* 复位找行程阶段不参与回拉，只处理正常运行和复位回中(Step3)。 */
	if( (Mot3.Reset != 0) && (Mot3.Step != 3) ) return;
	if( Mot3.MaxPulseNum == 0 ) return;
	if( Mot3PosCorrTimes >= Mot3PosCorrMaxTimes ) return;

	deadband = Mot3AngleToPulse(Mot3PosCorrDeadbandAngle);
	if( deadband < 2 ) deadband = 2; /* 行程标定值小时不要按1个脉冲判断，避免来回抖。 */
	maxCorr = Mot3AngleToPulse(Mot3PosCorrMaxAngle);
	if( maxCorr < deadband ) maxCorr = deadband;

	if( Mot3.CurPosition > Mot3.TarPosition ) residual = Mot3.CurPosition - Mot3.TarPosition;
	else residual = Mot3.TarPosition - Mot3.CurPosition;

	if( residual < deadband ) return; /* 已经落在允许偏差内，不再动作。 */
	if( residual > maxCorr ) return;  /* 偏差过大不像是停机制动过冲，不做自动回拉。 */
	/* 上一次回拉没让残差变小(卡住或者拉过头)，直接结束回拉，避免越拉越偏。 */
	if( (wasCorr != 0) && (residual >= Mot3PosCorrLastResidual) ) return;
	Mot3PosCorrLastResidual = residual;

	targetPos = Mot3.TarPosition;
	stepPulse = (u16)(((u32)residual * Mot3PosCorrRatioPercent) / 100);
	if( stepPulse == 0 ) stepPulse = 1;
	Mot3PosCorrTimes++;
	Mot3PosCorrBusy = 1;
	printf("m3 poscorr residual=%d step=%d\n", residual, stepPulse);
	if( Mot3.CurPosition > targetPos ) {
		/* 记录位置比目标大，朝位置减小的方向(Backward)小幅回拉。 */
		Mot3MoveBackward( Mot3.CurPosition - stepPulse, Mot3PosCorrVelocity );
	}
	else {
		/* 记录位置比目标小，朝位置增大的方向(Forward)小幅回拉。 */
		Mot3MoveForward( Mot3.CurPosition + stepPulse, Mot3PosCorrVelocity );
	}
	/* Move函数会把TarPosition改成中间目标，这里改回真正的目标位置，供下一次残差判断。 */
	Mot3.TarPosition = targetPos;
}
