/*******************************************************************************

	include file
*******************************************************************************/
#include "sys.h"
/*******************************************************************************
	constant define
*******************************************************************************/
#define c_MaxHeight 	510							//最大高度,单位mm 
#define c_PulseScaleFactor 10					//10倍
#define c_HeightScaleFactor  1					//10倍
#define c_SpeedScaleFactor 1					//10倍
#define c_DefaultTotalPulseNum	3000	//总的脉冲数
#define c_DefaultPulseNumPerMm 20*c_PulseScaleFactor	//每度对应的脉冲数
#define c_MoveTimeOver 40000					//35s,马达不转无脉冲输出时间
#define Mot2TriggerAdc1 2300
#define Mot2TriggerAdc2 2500
//-----------------------------------------------------------------
//	ram 
//-----------------------------------------------------------------

Motor2St Mot2;
Move2St Move2;

void Mot2Init(void)
{
	Mot2.Reset = 1;
	Mot2.Action = motor_stop;
	Mot2.Step = 0;
	Mot2.PidTimer = 0;
	Mot2.PidTimerB = 0;
	Mot2.PidEn = 1;
	Mot2.PulseSta = GPIO_ReadInputDataBit( Mot2PulseIO, Mot2PulsePin);
	Mot2.PulseDeb = 0;
	Mot2.PulseDebB = 0;
	Mot2.PulseNum = 0;
	Mot2.PulseNumB = 0;
	Mot2.ErrPulseNum = 0;
	Mot2.BrakeType = 0;
	Mot2.TarPulseNum = 0;

	Mot2.MaxPulseNum = c_DefaultTotalPulseNum;
	Mot2.PulseNumPerMm = c_DefaultPulseNumPerMm;
	Mot2.TarVelocity = 0;
	Mot2.CurVelocity = 0;
	Mot2.TarPosition = 0;
	Mot2.CurPosition = 0;	
	Mot2.CurHeight = 0;
	Mot2.TarHeight = 0;
	Mot2.Hall1Sta = 0;
	Mot2.Hall1Deb = 0;
	Mot2.Hall2Sta = 0;
	Mot2.Hall2Deb = 0;
	Mot2.Faultflag = 0;
	Mot2.En = 0;
	
	Move2.En = 0;
	Move2.Step = 0;
}

void Mot2UpHallScan(void)
{
	if( Mot2.Hall1Sta == 0 ) {
		if( Adc.Mot2.Hall1 >= Mot2TriggerAdc1 ) {

			if( Mot2.Hall1Deb >= 20 )	{	
				Mot2.Hall1Sta = 1;
			}	
		}	
		else {
			Mot2.Hall1Deb = 0;
		}	
  }
  else {
		if( Adc.Mot2.Hall1 < Mot2TriggerAdc1 ) {
			if( Mot2.Hall1Deb >= 20 )	{	
				Mot2.Hall1Sta = 0;
			}	
		}	
		else {
			Mot2.Hall1Deb = 0;
		}	
  }
}

u8 Mot2ReadUpHallState(void)
{
	
	return Mot2.Hall1Sta;
}

void Mot2DownHallScan(void)
{
	if( Mot2.Hall2Sta == 0 ) {
		if( Adc.Mot2.Hall2 >= Mot2TriggerAdc2 ) {
			if( Mot2.Hall2Deb >= 20 )	{	
				Mot2.Hall2Sta = 1;
			}	
		}	
		else {
			Mot2.Hall2Deb = 0;
		}	
  }
  else {
		if( Adc.Mot2.Hall2 < Mot2TriggerAdc2 ) {
			if( Mot2.Hall2Deb >= 20 )	{	
				Mot2.Hall2Sta = 0;
			}	
		}	
		else {
			Mot2.Hall2Deb = 0;
		}	
  }
}	

u8 Mot2ReadDownHallState(void)
{
	
	return Mot2.Hall2Sta;
}

/* 增量式速度PID，增益按百分之一缩放，Ki按每个50ms控制周期累加。 */
#define Mot2PidScale          100
#define Mot2VelocityKp        100
#define Mot2VelocityKi        2
#define Mot2VelocityKd        50
#define Mot2PidStartDuty      2500 // MOT2启动时的PID初始PWM值。
#define Mot2PidMinimumDuty    2500 // MOT2运动时的PID最低PWM值，停止时仍输出0。

static int32_t Mot2PidPreviousError = 0;
static int32_t Mot2PidPreviousPreviousError = 0;
static int32_t Mot2PidOutputScaled = 0;

/**
 * 函数作用：清除电机2速度PID的历史误差，并设置本次启动占空比。
 * InitialPwm：本次运动开始时的PWM占空比，范围为0到最大PWM。
 */
static void Mot2PidReset(u16 InitialPwm)
{
	Mot2PidPreviousError = 0;
	Mot2PidPreviousPreviousError = 0;
	if( InitialPwm > c_MaxPwmDuty ) InitialPwm = c_MaxPwmDuty;
	Mot2PidOutputScaled = (int32_t)InitialPwm * Mot2PidScale;
}

/**
 * 函数作用：根据目标速度和当前速度执行电机2增量式PID并输出限幅PWM。
 * TargetVelocity：本周期目标速度，单位与Mot2.CurVelocity相同。
 * CurrentVelocity：由电机2编码脉冲计算出的当前速度。
 * 返回值：范围为0到最大PWM的占空比。
 */
static u16 Mot2VelocityFeedbackControl(u16 TargetVelocity, u16 CurrentVelocity)
{
	int32_t Error;
	int32_t DeltaError;
	int32_t SecondDifference;
	int32_t DeltaOutputScaled;
	int32_t OutputLimitScaled;

	if( TargetVelocity == 0 ) {
		Mot2PidReset(0);
		return 0;
	}

	Error = (int32_t)TargetVelocity - (int32_t)CurrentVelocity;
	DeltaError = Error - Mot2PidPreviousError;
	SecondDifference = Error - 2 * Mot2PidPreviousError + Mot2PidPreviousPreviousError;
	DeltaOutputScaled = Mot2VelocityKp * DeltaError + Mot2VelocityKi * Error + Mot2VelocityKd * SecondDifference;
	OutputLimitScaled = (int32_t)c_MaxPwmDuty * Mot2PidScale;
	Mot2PidOutputScaled += DeltaOutputScaled;
	if( Mot2PidOutputScaled < 0 ) Mot2PidOutputScaled = 0;
	if( Mot2PidOutputScaled > OutputLimitScaled ) Mot2PidOutputScaled = OutputLimitScaled;

	Mot2PidPreviousPreviousError = Mot2PidPreviousError;
	Mot2PidPreviousError = Error;
	return (u16)(Mot2PidOutputScaled / Mot2PidScale);
}

void Mot2Encode(void)
{
	u8 state;
	state = GPIO_ReadInputDataBit( Mot2PulseIO, Mot2PulsePin ) ;
	if( state != Mot2.PulseSta ) {
		Mot2.PulseDebB = 0;
		Mot2.PulseDeb++;
		if( Mot2.PulseDeb >= 10 ) {
				
			Mot2.PulseSta = state;
      if( state != 0 ) {
					 if( ( Mot2.Hall1Sta == 0 )&&( Mot2.Hall2Sta == 0 ) ) { 
							Mot2.PulseNum++;
							Mot2.PulseNumB++;
					 } 
        }    
     }
  }
  else
  {
		Mot2.PulseDeb = 0;
		if( Mot2.PulseDebB < C_MotorStopDeb ) Mot2.PulseDebB++;
  }
}
u16 Mot2ReadEncode(void)
{
	u16 PulseNum;
	__disable_irq();
	PulseNum = Mot2.PulseNumB;
	Mot2.PulseNumB = 0;
	__enable_irq();
	return PulseNum; 
}	
void Mot2ClearEncode(void)
{
	Mot2.PulseNum = 0;
	Mot2.PulseNumB = 0;
	Mot2.BrakeType = 0;
	Mot2.PidTimer = 0;
	Mot2.PidTimerB = 0;
}	
/**
 * 函数作用：启动电机2运动前清零测速脉冲、重置PID状态并立即输出满占空比；无传入参数。
 */
void Mot2EnablePidControl(void)
{
	Mot2ClearEncode();
	Mot2.CurVelocity = 0;
	Mot2PidReset(Mot2PidStartDuty);
	Mot2.PidEn = Enable;
	TIM1_SetDuty_CH1(c_MaxPwmDuty); // 有效运动开始时立即输出满占空比。
}
/**
 * 函数作用：关闭电机2速度PID并清零PWM和历史控制状态。
 */
void Mot2DisablePidControl(void)
{
	TIM1_SetDuty_CH1(0);
	Mot2.PidEn = Disable;
	Mot2PidReset(0);
}
void Mot2SetCurrentPosition( u16 num )
{
	Mot2.CurPosition = num;
}	
void Mot2SetTargetPosition( u16 num )
{
	Mot2.TarPosition = num;
}	
/**
 * 函数作用：按固定周期更新电机2位置和编码器速度，运动期间固定输出满占空比；无传入参数。
 */
void Mot2VelocityControl(void)
{
	u16 Pwm, PidTargetVelocity;
	u32 sum, encode_count, remainingPulseNum;

	if( Mot2.PidEn == 0 ) return;
	if( Mot2.PidTimer < C_PID_RATE ) return;

	Mot2.PidTimer = Mot2.PidTimer - C_PID_RATE;
	sum = 0;
	if( Mot2.Action == motor_forward ) {
		sum = Mot2.CurPosition + Mot2.PulseNum;
		if( sum > Mot2.MaxPulseNum ) sum = Mot2.MaxPulseNum;
	}
	else if( Mot2.Action == motor_backward ) {
		if( Mot2.CurPosition > Mot2.PulseNum ) sum = Mot2.CurPosition - Mot2.PulseNum;
	}
	if( sum != 0 ) {
		sum = (u32)(sum * c_HeightScaleFactor * c_PulseScaleFactor);
		Mot2.CurHeight = sum / Mot2.PulseNumPerMm;
	}
	else Mot2.CurHeight = 0;

	encode_count = Mot2ReadEncode() * (1000 / C_PID_RATE);
	Mot2.PulseNumPerMin = encode_count * 60;
	encode_count = (u32)(encode_count * c_SpeedScaleFactor * c_PulseScaleFactor);
	if( encode_count != 0 ) Mot2.CurVelocity = (u32)(encode_count / Mot2.PulseNumPerMm);
	else Mot2.CurVelocity = 0;

	if( Mot2.TarPulseNum > Mot2.PulseNum ) {
		remainingPulseNum = Mot2.TarPulseNum - Mot2.PulseNum;
		PidTargetVelocity = Mot2.TarVelocity;
		if( (remainingPulseNum < 20) && (PidTargetVelocity > 20) ) PidTargetVelocity = 20;
		Pwm = Mot2VelocityFeedbackControl(PidTargetVelocity, Mot2.CurVelocity);
		if( (PidTargetVelocity != 0) &&
			(Mot2PidOutputScaled < (int32_t)Mot2PidMinimumDuty * Mot2PidScale) ) {
			Mot2PidOutputScaled = (int32_t)Mot2PidMinimumDuty * Mot2PidScale; // 同步PID内部输出，避免下一周期掉回最低值以下。
			Pwm = Mot2PidMinimumDuty;
		}
	}
	else {
		Pwm = Mot2VelocityFeedbackControl(0, Mot2.CurVelocity);
	}
	Pwm = (Mot2.TarPulseNum > Mot2.PulseNum) ? c_MaxPwmDuty : 0; // 运动期间满占空比，达到目标后输出零。
	TIM1_SetDuty_CH1(Pwm);
#if 1
	printf("M2.TarVel =%d\n", Mot2.TarVelocity);
	printf("M2.CurVel =%d\n", Mot2.CurVelocity);
	printf("M2.PulseNum =%d\n", Mot2.PulseNum);
	printf("M2.CurHeight =%d\n", Mot2.CurHeight);
	printf("M2.PulseNumPerMin=%d\n", Mot2.PulseNumPerMin);
	printf("pwm =%d\n", Pwm);
#endif
}		


u8 Mot2ActionState(void)
{
		return Mot2.Action;
}	
u16 Mot2ExcuteTime(void)
{
		return Mot2.PidTimerB;
}	
void Mot2MoveStop(void)
{
		TIM1_SetDuty_CH1(0);
		Mot2.Action = motor_stop;
		Mot2DisablePidControl();
}
void Mot2MoveBrake(void)
{
		if( Mot2.Action == motor_forward ) {
			Mot2.BrakeType = pulse_forward_brake;
			Mot2DisablePidControl();
			Mot2.Action = motor_brake;
			printf("M2pulsebrake\n");
		}	
		else if( Mot2.Action == motor_backward ) {
			Mot2.BrakeType = pulse_backward_brake;
			Mot2DisablePidControl();
			Mot2.Action = motor_brake;
			printf("M2pulsebrake\n");
		}	
}	
void Mot2MoveForward( u16 TarPosition, u16 Velocity )
{
		printf("M2 forward\n");
		//Mot2DirPin = NegativeMove;
		GPIO_SetBits(Mot2DirIO, Mot2DirPin);
	
		Mot2.TarPosition = TarPosition;
	  Mot2.TarVelocity = Velocity;
		
		printf("M2.CurPos=%d\n", Mot2.CurPosition);
		printf("M2.TarPos=%d\n", Mot2.TarPosition);
	
		if(  Mot2.TarPosition >  Mot2.CurPosition )	Mot2.TarPulseNum =  Mot2.TarPosition -  Mot2.CurPosition;
	  else Mot2.TarPulseNum = 0;
		
		printf("M2.TarPulseNum=%d\n", Mot2.TarPulseNum);
		if( Mot2.TarPulseNum != 0 ) {
		
			Mot2EnablePidControl();
			Mot2.Action = motor_forward;
		
		}
			Mot2.PidTimerB = 0;
}

void Mot2MoveBackward( u16 TarPosition, u16 Velocity)
{
		printf("M2 backward\n");
		//Mot2DirPin = PositiveMove;
		GPIO_ResetBits(Mot2DirIO, Mot2DirPin);
	
		Mot2.TarPosition = TarPosition;
	  Mot2.TarVelocity = Velocity;
		
		printf("M2.CurPos=%d\n", Mot2.CurPosition);
		printf("M2.TarPos=%d\n", Mot2.TarPosition);
	
	  if( Mot2.TarPosition != 0 ) {
			if(  Mot2.CurPosition >  Mot2.TarPosition )	Mot2.TarPulseNum =  Mot2.CurPosition -  Mot2.TarPosition;
			else Mot2.TarPulseNum = 0;
		} else {
				Mot2.TarPulseNum = Mot2.MaxPulseNum*2;
		}	
		printf("M2.TarPulseNum=%d\n", Mot2.TarPulseNum);
		
		if( Mot2.TarPulseNum != 0 ) {
			
			Mot2EnablePidControl();
			Mot2.Action = motor_backward;
		}
		Mot2.PidTimerB = 0;
}

void Mot2ActionHandle(void)
{
	if( Mot2.Action == motor_forward ) {
		
		Mot2VelocityControl();
		
		if( Mot2.PulseNum >= Mot2.TarPulseNum )	{
			Mot2.BrakeType = pulse_forward_brake;
			Mot2DisablePidControl();
			Mot2.Action = motor_brake;
			printf("M2 pulse fw brake\n");
		}	
		else if( Mot2ReadUpHallState() != 0 ){

			Mot2.BrakeType = hall_forward_brake;
			Mot2DisablePidControl();
			Mot2.Action = motor_brake;
			printf("M2hallfwbrake\n");
		}		
	}		
	else if( Mot2.Action == motor_backward ) {
		
		Mot2VelocityControl();

		if( Mot2.PulseNum >= Mot2.TarPulseNum )	{
			
			Mot2.BrakeType = pulse_backward_brake;
			Mot2DisablePidControl();
			Mot2.Action = motor_brake;
			printf("M2pulsebwbrake\n");
		}	
		if( Mot2ReadDownHallState() != 0 ){
			if( Mot2.DelayTimer >= c_1s ) {
				Mot2.BrakeType = hall_backward_brake;
				Mot2DisablePidControl();
				Mot2.Action = motor_brake;
				printf("M2hallbwbrake\n");
			}	
		} else {
			Mot2.DelayTimer = 0;
		}		
	}		
	else if( Mot2.Action == motor_brake )  {
		
			if( Mot2.PulseDebB >= C_MotorStopDeb )	{
				
				printf("M2 stop\n");
				printf("M2.PulseNum = %d\n",Mot2.PulseNum );
				printf("M2.TarPulseNum = %d\n",Mot2.TarPulseNum );
				//Mot2.ErrPulseNum = Mot2.PulseNum - Mot2.TarPulseNum;
				//printf("Mot2.ErrPulseNum =%d\n ",Mot2.ErrPulseNum);
				printf("M2.CurPos=%d\n ",Mot2.CurPosition );
				if( Mot2.BrakeType == pulse_forward_brake ) {
					Mot2.CurPosition += Mot2.PulseNum;
					if( Mot2.CurPosition > Mot2.MaxPulseNum ) Mot2.CurPosition = Mot2.MaxPulseNum;
				}
				else if( Mot2.BrakeType == hall_forward_brake ) {
					
					Mot2.CurPosition = Mot2.MaxPulseNum;
					
				}
				else if( Mot2.BrakeType == pulse_backward_brake ) {
					
						if( Mot2.CurPosition > Mot2.PulseNum )	Mot2.CurPosition -= Mot2.PulseNum;
					  else Mot2.CurPosition = 0;
				}	
				else if( Mot2.BrakeType == hall_backward_brake ) {
					
					Mot2.CurPosition = 0;					
					
				}	
				printf("M2.CurPos=%d\n ",Mot2.CurPosition );
				printf("M2.CurHeight =%d\n ", Mot2.CurHeight );
				Mot2.CurHeight = (u32)(Mot2.CurPosition*c_HeightScaleFactor*c_PulseScaleFactor)/Mot2.PulseNumPerMm; 
				Mot2.CurVelocity = 0;
				Mot2.Action = motor_stop;				
		}		
	}		
}	

void Mot2SetHeightSpeed( u16 Height, u16 speed )
{
		u32 sum;
	if( Height >= (c_MaxHeight*10) ) Height=c_MaxHeight*10;
		printf("speed =%d\n", speed );
		printf("Height =%d\n", Height );
		Mot2.TarVelocity = speed;
	
		printf("M2.PulseNumPerMm =%d\n", Mot2.PulseNumPerMm );
	
		sum = (u32)Height*Mot2.PulseNumPerMm;
	
		//printf("sum =%d\n", sum );
	
		if( sum != 0 )	Mot2.TarPosition = sum/(c_HeightScaleFactor*c_PulseScaleFactor);
	  else Mot2.TarPosition = 0;
		
		printf("M2.TarPos=%d\n", Mot2.TarPosition );
	
	if( Mot2.TarPosition > Mot2.CurPosition ) {
			
			Mot2MoveForward( Mot2.TarPosition, speed ); 
		}	
		else if( Mot2.TarPosition < Mot2.CurPosition ) {
			
			Mot2MoveBackward( Mot2.TarPosition, speed ); 
		}	
}	
void Move2SetTask( u16 Height, u16 Speed )
{
	if( Mot2.Reset != 0 ) return ;
	if( ( Height == Move2.Height )&&( Speed == Move2.Speed ) ) return ;
	Move2.Height = Height;
	Move2.Speed = Speed;
	printf("m2 task height =%d\n",Height );
	printf("m2 task speed =%d\n",Speed );
	Move2.Step = 0;
	Move2.En = Enable;
}	
void Mot2MoveTask(void)
{
		if( Move2.En == 0 ) return ;
		switch( Move2.Step )
		{
				case 0:
							if( Mot2ActionState() != motor_stop ) {
								Mot2MoveBrake();
							}	
							Move2.Step++;
							break;
				case 1:
							if( Mot2ActionState() == motor_stop ) {
								Mot2SetHeightSpeed( Move2.Height, Move2.Speed );
								Move2.En = 0;
							}	
							break;
		}		
}	

void Mot2MoveService(void)
{
		u32 total;
		
		Mot2UpHallScan();
	
		Mot2DownHallScan();
	
		Mot2ActionHandle();
		
		#if Motor1Enable == 1
		if( Mot1.Reset ) return;
		#endif
	
		if( Mot2.Reset ) {
			
			switch( Mot2.Step )
			{
				case 0:
							Mot2SetCurrentPosition(20000);
							Mot2MoveBackward( 0, 30 );
							Mot2.Step++;
							break;
				case 1:
							if( Mot2ActionState() == motor_stop ){
								delay_ms(200);
								Mot2SetCurrentPosition(0);
								Mot2MoveForward( 20000, 20 ); 
								Mot2.Step++;
							}		
							else if( Mot2ExcuteTime() > c_MoveTimeOver ){
									//printf("Mot2 move timer over \n");
									Mot2MoveStop();
									Mot2.Faultflag = Hall1Fault;
									Mot2.Step = 4;	
							}			
							break;
				case 2:
							if( Mot2ActionState() == motor_stop ){
								
								if( Mot2.PulseNum < C_VerifyMinimumPulseNum ) {
									Mot2.Faultflag = MotorFault;
									Mot2.Step = 4;
								}
								else {	
									Mot2.MaxPulseNum = Mot2.PulseNum;
									printf("M2.MaxPulseNum =%d\n", Mot2.MaxPulseNum );
									Mot2.CurPosition =  Mot2.MaxPulseNum; 
									total = (u32)Mot2.MaxPulseNum*c_PulseScaleFactor;
									Mot2.PulseNumPerMm = total / c_MaxHeight;
									printf("M2.PulseNumPerMm =%d\n", Mot2.PulseNumPerMm );
									
									delay_ms(500);
									Mot2MoveBackward( 0, 30 );
									Mot2.Step = 3;
									
								}	
							}		
							else if( Mot2ExcuteTime() > c_MoveTimeOver )	{
									//printf("Mot2 move timer over\n");
									Mot2MoveStop();
									Mot2.Faultflag = Hall2Fault;
									Mot2.Step = 4;
							}										
							break;
				case 3:
							if( Mot2ActionState() == motor_stop ){
								Mot2.En = 1;
								Mot2.Reset = 0;
							}	
							break;
				case 4:
							printf("M2 fault =%d\n", Mot2.Faultflag );
							Mot2.Step++;
							break;
			}		
		}		
		else {
			
			if( Mot2.En ) {
				
				Mot2MoveTask();
			
			}	
				
		}	
}         



