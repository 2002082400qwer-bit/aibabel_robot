/*******************************************************************************

	include file
*******************************************************************************/
#include "mot1.h"

/*******************************************************************************
	constant define
*******************************************************************************/
#define c_MaxAngle 	180								//最大角度180度
#define c_PulseScaleFactor 10				//10倍
#define c_AngleScaleFactor 10					//10倍
#define c_RadianScaleFactor 10				//10倍
#define c_DefaultTotalPulseNum	3600	//初始化脉冲数，后续会根据标定结果覆盖
#define c_DefaultPulseNumPerAngle 20*c_PulseScaleFactor	//初始化每度对应的脉冲数，后续会根据标定结果计算后覆盖
#define c_MoveTimeOver 20000					//20s,马达不转无脉冲输出时间
#define Mot1TriggerAdc 2800
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

#define Mot1PidScale 100 						// 增量式PID增益缩放，Ki按每个50ms周期累加。
#define Mot1VelocityKp 100 						// 比例增益Kp=1.00，按Mot2现有PID单位缩放。
#define Mot1VelocityKi 2 						// 积分增益Ki=0.02，每个50ms控制周期累加。
#define Mot1VelocityKd 100 						// 微分增益Kd=0.50，提高速度变化时的制动响应。
#define Mot1PidStartDuty 1800 					// 每次启动PID时的初始PWM基准。
#define Mot1PidApproachPulseNum 500 			// 最多提前500脉冲减速，短行程使用本次目标脉冲数的一半。
#define Mot1PidApproachMinVelocity 20 			// 接近目标时的最低速度指令。
#define Mot1PidMinimumDuty 1800 				// Mot1运行期间的PWM下限，初始化和普通定位均生效。
static int32_t Mot1PidPreviousError = 0; 		// 上一个控制周期的速度误差。
static int32_t Mot1PidPreviousPreviousError = 0;// 上上个控制周期的速度误差。
static int32_t Mot1PidOutputScaled = 0; 		// 按PID缩放比例保存的PWM输出.


/**
 * 函数作用：根据目标速度和当前速度误差调整Mot1的PWM输出。
 * TargetVelocity：目标速度，单位与当前速度一致。
 * CurrentVelocity：编码器反馈的当前速度。
 * 返回值：限幅后的PWM占空比。
 */
u16 Mot1VelocityFeedbackControl(u16 TargetVelocity, u16 CurrentVelocity)
{
    int32_t Error; // 本周期目标速度与实测速度的误差。
    int32_t DeltaError; // 本周期与上周期速度误差的变化量。
    int32_t SecondDifference; // 速度误差的二阶差分。
    int32_t ProportionalDeltaScaled; // 比例项对应的PWM增量。
    int32_t IntegralDeltaScaled; // 积分项对应的PWM增量。
    int32_t DerivativeDeltaScaled; // 微分项对应的PWM增量。
    int32_t CandidateOutputScaled; // 加入本周期PID增量后的候选PWM。
    int32_t OutputLimitScaled; // 最大PWM对应的缩放值。

    if( TargetVelocity == 0 ) {
        Mot1PidPreviousError = 0;
        Mot1PidPreviousPreviousError = 0;
        Mot1PidOutputScaled = 0;
        return 0;
    }

    Error = (int32_t)TargetVelocity - (int32_t)CurrentVelocity;
    DeltaError = Error - Mot1PidPreviousError;
    SecondDifference = Error - 2 * Mot1PidPreviousError + Mot1PidPreviousPreviousError;
    ProportionalDeltaScaled = Mot1VelocityKp * DeltaError;
    IntegralDeltaScaled = Mot1VelocityKi * Error;
    DerivativeDeltaScaled = Mot1VelocityKd * SecondDifference;
    OutputLimitScaled = (int32_t)c_MaxPwmDuty * Mot1PidScale;
    CandidateOutputScaled = Mot1PidOutputScaled + ProportionalDeltaScaled + IntegralDeltaScaled + DerivativeDeltaScaled;

    if( ((CandidateOutputScaled > OutputLimitScaled) && (Error > 0)) ||
        ((CandidateOutputScaled < 0) && (Error < 0)) ) {
        CandidateOutputScaled = Mot1PidOutputScaled + ProportionalDeltaScaled + DerivativeDeltaScaled;
    }
    Mot1PidOutputScaled = CandidateOutputScaled;
    if( Mot1PidOutputScaled < 0 ) Mot1PidOutputScaled = 0;
    if( Mot1PidOutputScaled > OutputLimitScaled ) Mot1PidOutputScaled = OutputLimitScaled;

    Mot1PidPreviousPreviousError = Mot1PidPreviousError;
    Mot1PidPreviousError = Error;
    return (u16)(Mot1PidOutputScaled / Mot1PidScale);
}
void Mot1Encode(void)
{
	u8 state;
	state = GPIO_ReadInputDataBit( Mot1PulseIO, Mot1PulsePin ) ;
	if( state != Mot1.PulseSta ) {
		Mot1.PulseDebB = 0;
		Mot1.PulseDeb++;
		if( Mot1.PulseDeb >= 10 ) {
				
			Mot1.PulseSta = state;
      if( state != 0 ) {
					 if( ( Mot1.Hall1Sta == 0 )&&( Mot1.Hall2Sta == 0 ) ) { 
							Mot1.PulseNum++;
							Mot1.PulseNumB++;
					 } 
        }    
     }
  }
  else
  {
		Mot1.PulseDeb = 0;
		if( Mot1.PulseDebB < C_MotorStopDeb ) Mot1.PulseDebB++;
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
 * 函数作用：清零编码器状态并启用Mot1速度闭环，无传入参数。
 */
void Mot1EnablePidControl(void)
{
    Mot1ClearEncode();
    Mot1.CurVelocity = 0;
    Mot1PidPreviousError = 0;
    Mot1PidPreviousPreviousError = 0;
    Mot1PidOutputScaled = (int32_t)Mot1PidStartDuty * Mot1PidScale;
    Mot1.PidEn = Enable;
    // TIM1_SetDuty_CH3(c_testPwmDuty); // 测试模式：有效运动启动时立即输出满占空比。
}
/**
 * 函数作用：关闭Mot1速度PID并清零PWM和历史控制状态，无传入参数。
 */
void Mot1DisablePidControl(void)
{
    TIM1_SetDuty_CH3(0);
    Mot1.PidEn = Disable;
    Mot1PidPreviousError = 0;
    Mot1PidPreviousPreviousError = 0;
    Mot1PidOutputScaled = 0;
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
 * 函数作用：每个速度控制周期更新Mot1速度反馈、位置角度和PWM，无传入参数。
 */
void Mot1VelocityControl(void)
{
	u16 Pwm; // 本周期计算得到的PWM占空比。
	 u16 PidTargetVelocity; // 按剩余脉冲计算的PID速度目标。
	 u32 sum; // 当前位置计算的脉冲中间值。
	 u32 encode_count; // 本周期编码器速度换算中间值。
	 u32 remainingPulseNum; // 当前目标剩余的编码器脉冲数。
	 u32 brakePulseNum; // 本次运动采用的减速窗口脉冲数。
	u8 isHomingToLimit; // 初始化阶段是否正在寻找机械限位。

	
	if( Mot1.PidEn == 0 )	return;	
	
	if( Mot1.PidTimer >= C_PID_RATE ) {
	
		Mot1.PidTimer = Mot1.PidTimer - C_PID_RATE;
		isHomingToLimit = (Mot1.Reset != 0) && (Mot1.Step < 3);
		//printf("Mot1.PidTimer1 =%d\n", Mot1.PidTimer);
		
		if( Mot1.Action == motor_forward )  {		
			sum = Mot1.CurPosition + Mot1.PulseNum;
			if( sum > Mot1.MaxPulseNum ) sum = Mot1.MaxPulseNum;
		}	
		else if(  Mot1.Action == motor_backward )
		{
			if( Mot1.CurPosition > Mot1.PulseNum ) sum = Mot1.CurPosition - Mot1.PulseNum;
			else sum = 0;
		}		
		 
		if( sum != 0 ) {
			sum = (u32)(sum*c_AngleScaleFactor*c_PulseScaleFactor);
			Mot1.CurAngle =  sum/Mot1.PulseNumPerAngle;  
		}	
		else Mot1.CurAngle = 0;
		
		encode_count = Mot1ReadEncode()*(1000/C_PID_RATE);
		Mot1.PulseNumPerMin = encode_count*60;
		encode_count = (u32)(encode_count*c_RadianScaleFactor*c_PulseScaleFactor);
		if( encode_count != 0 )	Mot1.CurVelocity = (u32)(encode_count/Mot1.PulseNumPerAngle);
		else Mot1.CurVelocity = 0;
				
		if( (Mot1.TarVelocity == 0) || (Mot1.TarPulseNum <= Mot1.PulseNum) ) {
            Pwm = Mot1VelocityFeedbackControl(0, Mot1.CurVelocity);
        }
        else {
            remainingPulseNum = Mot1.TarPulseNum - Mot1.PulseNum;
            brakePulseNum = Mot1.TarPulseNum / 2;
            if( brakePulseNum == 0 ) brakePulseNum = 1; // 至少保留1脉冲窗口，避免速度插值除零。
            if( brakePulseNum > Mot1PidApproachPulseNum ) brakePulseNum = Mot1PidApproachPulseNum;
            PidTargetVelocity = Mot1.TarVelocity;
            if( (isHomingToLimit == 0) && (PidTargetVelocity > Mot1PidApproachMinVelocity) && (remainingPulseNum < brakePulseNum) ) {
                PidTargetVelocity = Mot1PidApproachMinVelocity +
                    (u16)(((u32)(Mot1.TarVelocity - Mot1PidApproachMinVelocity) * remainingPulseNum) / brakePulseNum);
            }
            Pwm = Mot1VelocityFeedbackControl(PidTargetVelocity, Mot1.CurVelocity);
            if( Mot1PidOutputScaled < (int32_t)Mot1PidMinimumDuty * Mot1PidScale ) {
                Mot1PidOutputScaled = (int32_t)Mot1PidMinimumDuty * Mot1PidScale; // 仅低于下限时校正内部输出，避免丢失Ki积分。
                Pwm = Mot1PidMinimumDuty;
            }
        }
        // Pwm = (Mot1.TarPulseNum > Mot1.PulseNum) ? c_testPwmDuty : 0; // 测试模式：运动期间固定满占空比，到达目标脉冲后输出0。
        TIM1_SetDuty_CH3(Pwm);
		#if 1
		printf("M1.TarVel =%d\n", Mot1.TarVelocity);
		printf("M1.CurVel =%d\n", Mot1.CurVelocity);
		printf("M1.PulseNum =%d\n", Mot1.PulseNum);
		printf("M1.CurAngle=%d\n", Mot1.CurAngle);
		printf("M1.PulseNumPerMin=%d\n", Mot1.PulseNumPerMin);
		printf("pwm =%d\n", Pwm );
		#endif
		//printf("Mot1.PidTimer2 =%d\n", Mot1.PidTimer);
	}	
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
/** 函数作用：电机1正向移动；TarPosition为目标脉冲坐标，Velocity为目标速度。 */
void Mot1MoveForward( u16 TarPosition, u16 Velocity )
{
	printf("m1 forward\n");

	//Mot1DirPin = NegativeMove;
	GPIO_ResetBits(Mot1DirIO, Mot1DirPin);	

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

/** 函数作用：电机1反向移动；TarPosition为目标脉冲坐标，Velocity为目标速度。 */
void Mot1MoveBackward( u16 TarPosition, u16 Velocity)
{
		printf("m1 backward\n");
		//Mot1DirPin = PositiveMove;
	  GPIO_SetBits(Mot1DirIO, Mot1DirPin);
	
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
		
			if( Mot1.PulseDebB >= C_MotorStopDeb )	{
				
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
				Mot1.CurAngle = (u32)(Mot1.CurPosition*c_AngleScaleFactor*c_PulseScaleFactor)/Mot1.PulseNumPerAngle; 
				Mot1.CurVelocity = 0;
				Mot1.Action = motor_stop;				
		}		
	}		
}	

void Mot1SetAngleSpeed( u16 Angle, u16 speed )
{
		u32 sum;
	
	 if( Angle >= (c_MaxAngle*10) ) Angle=c_MaxAngle*10;
	
		Mot1.TarVelocity = speed;
	
		printf("M1.PulseNumPerAngle =%d\n", Mot1.PulseNumPerAngle );
	
		sum = (u32)Angle*Mot1.PulseNumPerAngle;
	
		//printf("sum =%d\n", sum );
		if( sum != 0 ) 	Mot1.TarPosition = sum/(c_AngleScaleFactor*c_PulseScaleFactor);
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

/**
 * 函数作用：首次执行两端标定，历史校准有效时先找零点侧限位再回中；无传入参数。
 */
void Mot1MoveService(void)
{
		u32 total; // 根据实测全行程计算每角度脉冲数的中间值。
		
		Mot1LeftHallScan();
	
		Mot1RightHallScan();
	
		Mot1ActionHandle();
	
		if( Mot1.Reset ) {

			switch( Mot1.Step )
			{
				case 0:
							Mot1SetCurrentPosition(20000);
							Mot1MoveBackward( 0, 900 );
							Mot1.Step++;
							break;
				case 1:
							if( Mot1ActionState() == motor_stop ){
								if( Mot1.BrakeType != hall_backward_brake ) {
									Mot1.Faultflag = Hall1Fault;
									Mot1.Step = 4;
								}
								else {
									Mot1SetCurrentPosition(0);
									if( MotorCalibrationLoaded != 0 ) {
										// 历史标定有效：从左限位按保存行程直接回中，省去另一端标定。
										Mot1MoveForward( Mot1.MaxPulseNum / 2, 900 );
										Mot1.Step = 3;
									}
									else {
										Mot1MoveForward( 20000, 900 );
										Mot1.Step++;
									}
								}
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
								
								if( Mot1.BrakeType != hall_forward_brake ) {
									Mot1.Faultflag = Hall2Fault;
									Mot1.Step = 4;
								}
								else if( Mot1.PulseNum < C_VerifyMinimumPulseNum ) {
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
									Mot1MoveBackward( Mot1.MaxPulseNum/2, 900 ); 
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
								// 归位被异常限位打断时不报成功，防止把未到中点的数据当成自检成功。
								if( (Mot1.BrakeType == ((MotorCalibrationLoaded != 0) ? pulse_forward_brake : pulse_backward_brake)) &&
					(Mot1.Hall1Sta == 0) && (Mot1.Hall2Sta == 0) ) {
									Mot1.En = 1;
									Mot1.Reset = 0;
								}
								else {
									Mot1.Faultflag = MotorFault;
									Mot1.Step = 4;
								}
							}
							else if( Mot1ExcuteTime() > c_MoveTimeOver ) {
								Mot1MoveStop();
								Mot1.Faultflag = MotorFault;
								Mot1.Step = 4;
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



