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

static int32_t Mot3PidPreviousError = 0;
static int32_t Mot3PidPreviousPreviousError = 0;
static int32_t Mot3PidOutputScaled = 0;

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
	if( sum != 0 ) {
		sum = (u32)(sum * c_AngleScaleFactor * c_PulseScaleFactor);
		Mot3.CurAngle = sum / Mot3.PulseNumPerAngle;
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
		if( (Mot3.Reset != 0) && (PidTargetVelocity != 0) &&
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
				Mot3.CurAngle = (u32)(Mot3.CurPosition*c_AngleScaleFactor*c_PulseScaleFactor)/Mot3.PulseNumPerAngle; 
				Mot3.CurVelocity = 0;
				Mot3.Action = motor_stop;				
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
		u32 sum; // 角度换算为脉冲位置的中间量。
		Mot3.TarVelocity = speed;
	  if( Angle >= (c_MaxAngle*10) ) Angle=c_MaxAngle*10;
		printf("Angle =%d\n", Angle );
		printf("speed =%d\n", speed );
		printf("M3.PulseNumPerAngle =%d\n", Mot3.PulseNumPerAngle );
	
		sum = (u32)Angle*Mot3.PulseNumPerAngle;
	
		//printf("sum =%d\n", sum );
	
		if( Angle >= (c_MaxAngle*10) ) Mot3.TarPosition = Mot3.MaxPulseNum;
		else if( sum != 0 ) Mot3.TarPosition = sum/(c_AngleScaleFactor*c_PulseScaleFactor);
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

void Mot3MoveService(void)
{
		u32 total;
		
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
								delay_ms(500);
								Mot3SetCurrentPosition(0);
								Mot3MoveForward( 20000, 50 ); 
								Mot3.Step++;
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
								
								if( Mot3.PulseNum < C_VerifyMinimumPulseNum ) {
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
									Mot3.En = 1;
									Mot3.Reset = 0;
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



