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
#define Mot3TriggerAdc 2600
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
		if( Adc.Mot3.Hall1 >= ( Mot3TriggerAdc+600 ) ) {
			if( Mot3.Hall1Deb >= 20 )	{	
				Mot3.Hall1Sta = 1;
				printf("MOT3 up hall trigger\r\n");
			}	
		}	
		else {
			Mot3.Hall1Deb = 0;
		}	
  }
  else {
		if( Adc.Mot3.Hall1 < ( Mot3TriggerAdc+600 ) ) {
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

#define Mot3PidScale 100 							// 增量式PID增益缩放，Ki按每个50ms周期累加。
#define Mot3VelocityKp 100 							// 比例增益Kp=1.00，按Mot2现有PID单位缩放。
#define Mot3VelocityKi 2 							// 积分增益Ki=0.02，每个50ms控制周期累加。
#define Mot3VelocityKd 100 							// 微分增益Kd=1.00，与MOT1当前参数一致。
#define Mot3PidMinimumDuty 4800 					// MOT3有效运动期间的最低PWM，停止或到达目标时不启用。
#define Mot3HomingDuty 4800 						// 首次标定和后续寻零的固定低PWM，不受速度PID积分影响。
#define Mot3PidStartDuty 4800 						// PID启动初值，与MOT1当前参数一致。
#define Mot3PidApproachPulseNum 500 				// 最多提前500脉冲减速，短行程使用本次目标脉冲数的一半。
#define Mot3PidApproachMinVelocity 20 				// 接近目标时的最低速度指令。
static int32_t Mot3PidPreviousError = 0; 			// 上一个控制周期的速度误差。
static int32_t Mot3PidPreviousPreviousError = 0; 	// 上上个控制周期的速度误差。
static int32_t Mot3PidOutputScaled = 0; 			// 按PID缩放比例保存的PWM输出.


/**
 * 函数作用：根据目标速度和当前速度误差调整Mot3的PWM输出。
 * TargetVelocity：目标速度，单位与当前速度一致。
 * CurrentVelocity：编码器反馈的当前速度。
 * 返回值：限幅后的PWM占空比。
 */
u16 Mot3VelocityFeedbackControl(u16 TargetVelocity, u16 CurrentVelocity)
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
        Mot3PidPreviousError = 0;
        Mot3PidPreviousPreviousError = 0;
        Mot3PidOutputScaled = 0;
        return 0;
    }

    Error = (int32_t)TargetVelocity - (int32_t)CurrentVelocity;
    DeltaError = Error - Mot3PidPreviousError;
    SecondDifference = Error - 2 * Mot3PidPreviousError + Mot3PidPreviousPreviousError;
    ProportionalDeltaScaled = Mot3VelocityKp * DeltaError;
    IntegralDeltaScaled = Mot3VelocityKi * Error;
    DerivativeDeltaScaled = Mot3VelocityKd * SecondDifference;
    OutputLimitScaled = (int32_t)c_MaxPwmDuty * Mot3PidScale;
    CandidateOutputScaled = Mot3PidOutputScaled + ProportionalDeltaScaled + IntegralDeltaScaled + DerivativeDeltaScaled;

    if( ((CandidateOutputScaled > OutputLimitScaled) && (Error > 0)) ||
        ((CandidateOutputScaled < 0) && (Error < 0)) ) {
        CandidateOutputScaled = Mot3PidOutputScaled + ProportionalDeltaScaled + DerivativeDeltaScaled;
    }
    Mot3PidOutputScaled = CandidateOutputScaled;
    if( Mot3PidOutputScaled < 0 ) Mot3PidOutputScaled = 0;
    if( Mot3PidOutputScaled > OutputLimitScaled ) Mot3PidOutputScaled = OutputLimitScaled;

    Mot3PidPreviousPreviousError = Mot3PidPreviousError;
    Mot3PidPreviousError = Error;
    return (u16)(Mot3PidOutputScaled / Mot3PidScale);
}
void Mot3Encode(void)
{
	u8 state;
	state = GPIO_ReadInputDataBit( Mot3PulseIO, Mot3PulsePin) ;
	if( state != Mot3.PulseSta ) {
		Mot3.PulseDebB = 0;
		Mot3.PulseDeb++;
		if( Mot3.PulseDeb >= 10 ) {
				
			Mot3.PulseSta = state;
      if( state != 0 ) {
					 if( ( Mot3.Hall1Sta == 0 )&&( Mot3.Hall2Sta == 0 ) ) { 
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
 * 函数作用：清零编码器及PID状态并启用运动控制；寻限位立即输出低PWM，回中及普通定位启用速度PID，无传入参数。
 */
void Mot3EnablePidControl(void)
{
    Mot3ClearEncode();
    Mot3.CurVelocity = 0;
    Mot3PidPreviousError = 0;
    Mot3PidPreviousPreviousError = 0;
    Mot3PidOutputScaled = (Mot3.TarVelocity != 0) ? (int32_t)Mot3PidStartDuty * Mot3PidScale : 0; // 零速度指令不产生启动PWM。
    Mot3.PidEn = Enable;
    if( (Mot3.Reset != 0) && (Mot3.Step < 3) ) {
        TIM1_SetDuty_CH2((Mot3.TarVelocity != 0 && Mot3.TarPulseNum != 0) ? Mot3HomingDuty : 0); // 寻限位立即输出低PWM，零速度或零行程保持0。
    }
	TIM1_SetDuty_CH2(c_testPwmDutyMot3); // 测试模式：有效运动启动时立即输出固定占空比。
}
/**
 * 函数作用：关闭Mot3速度PID并清零PWM和历史控制状态，无传入参数。
 */
void Mot3DisablePidControl(void)
{
    TIM1_SetDuty_CH2(0);
    Mot3.PidEn = Disable;
    Mot3PidPreviousError = 0;
    Mot3PidPreviousPreviousError = 0;
    Mot3PidOutputScaled = 0;
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
 * 函数作用：每周期更新速度、角度和PWM；首次标定及后续寻零固定低PWM，回中和普通定位使用PID及距离减速；停止输出0，无传入参数。
 */
void Mot3VelocityControl(void)
{
	u16 Pwm; // 本周期计算得到的PWM占空比。
	 u16 PidTargetVelocity; // 按剩余脉冲计算的PID速度目标。
	 u32 sum; // 当前位置计算的脉冲中间值。
	 u32 encode_count; // 本周期编码器速度换算中间值。
	 u32 remainingPulseNum; // 当前目标剩余的编码器脉冲数。
	 u32 brakePulseNum; // 本次运动采用的减速窗口脉冲数。

	
	if( Mot3.PidEn == 0 )	return;	
	
	if( Mot3.PidTimer >= C_PID_RATE ) {
	
		Mot3.PidTimer = Mot3.PidTimer - C_PID_RATE;
		//printf("Mot3.PidTimer1 =%d\n", Mot3.PidTimer);
		
		if( Mot3.Action == motor_forward )  {		
			sum = Mot3.CurPosition + Mot3.PulseNum;
			if( sum > Mot3.MaxPulseNum ) sum = Mot3.MaxPulseNum;
		}	
		else if(  Mot3.Action == motor_backward )
		{
			if( Mot3.CurPosition > Mot3.PulseNum ) sum = Mot3.CurPosition - Mot3.PulseNum;
			else sum = 0;
		}		
		//printf("sum =%d\n", sum); 
		if( sum != 0 ) {
			sum = (u32)(sum*c_AngleScaleFactor*c_PulseScaleFactor);
			Mot3.CurAngle =  sum/Mot3.PulseNumPerAngle;  
		}	
		else Mot3.CurAngle = 0;
		
		
		
		encode_count = Mot3ReadEncode()*(1000/C_PID_RATE);
		Mot3.PulseNumPerMin = encode_count*60;
		encode_count = (u32)(encode_count*c_RadianScaleFactor*c_PulseScaleFactor);
		if( encode_count != 0 )	Mot3.CurVelocity = (u32)(encode_count/Mot3.PulseNumPerAngle);
		else Mot3.CurVelocity = 0;
				
		#if 1
		        if( (Mot3.TarVelocity == 0) || (Mot3.TarPulseNum <= Mot3.PulseNum) ) {
            Pwm = Mot3VelocityFeedbackControl(0, Mot3.CurVelocity);
        }
        else if( (Mot3.Reset != 0) && (Mot3.Step < 3) ) {
            Pwm = Mot3HomingDuty; // 两侧寻限位全程固定低PWM，按编码器累计实际行程，不运行速度PID。
        }
        else {
            remainingPulseNum = Mot3.TarPulseNum - Mot3.PulseNum;
            brakePulseNum = Mot3.TarPulseNum / 2;
            if( brakePulseNum == 0 ) brakePulseNum = 1; // 至少保留1脉冲窗口，避免速度插值除零。
            if( brakePulseNum > Mot3PidApproachPulseNum ) brakePulseNum = Mot3PidApproachPulseNum;
            PidTargetVelocity = Mot3.TarVelocity;
            if( (PidTargetVelocity > Mot3PidApproachMinVelocity) && (remainingPulseNum < brakePulseNum) ) {
                PidTargetVelocity = Mot3PidApproachMinVelocity +
                    (u16)(((u32)(Mot3.TarVelocity - Mot3PidApproachMinVelocity) * remainingPulseNum) / brakePulseNum);
            }
            Pwm = Mot3VelocityFeedbackControl(PidTargetVelocity, Mot3.CurVelocity);
            if( Mot3PidOutputScaled < (int32_t)Mot3PidMinimumDuty * Mot3PidScale ) {
                Mot3PidOutputScaled = (int32_t)Mot3PidMinimumDuty * Mot3PidScale; // 仅低于下限时同步内部输出，保留正常Ki积分。
                Pwm = Mot3PidMinimumDuty;
            }
        }
		#else
		if( Mot3.TarVelocity == 0 ) Pwm = 0;
		else Pwm = 2400;
		#endif
		Pwm = (Mot3.TarPulseNum > Mot3.PulseNum) ? c_testPwmDutyMot3 : 0; // 测试模式：运动期间输出固定pwm，到达目标脉冲后输出0。
		TIM1_SetDuty_CH2(Pwm);
		#if 1
			printf("M3.TarVel =%d\n", Mot3.TarVelocity);
			printf("M3.CurVel =%d\n", Mot3.CurVelocity);
			printf("M3.PulseNum =%d\n", Mot3.PulseNum);
			printf("M3.CurAngle=%d\n", Mot3.CurAngle);
			printf("M3.PulseNumPerMin=%d\n", Mot3.PulseNumPerMin);
			printf("pwm =%d\n", Pwm );
		#endif
		//printf("Mot3.PidTimer2 =%d\n", Mot3.PidTimer);
	}	
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
/** 函数作用：电机3正向移动；TarPosition为目标脉冲坐标，Velocity为目标速度。 */
void Mot3MoveForward( u16 TarPosition, u16 Velocity )
{
		printf("M3 forward\n");
		//Mot3DirPin = PositiveMove;
	  GPIO_SetBits( Mot3DirIO, Mot3DirPin );
		 
		Mot3.TarPosition = TarPosition;
	  Mot3.TarVelocity = Velocity;
		
		// printf("M3.CurPos=%d\n", Mot3.CurPosition);
		// printf("M3.TarPos=%d\n", Mot3.TarPosition);
	
		if(  Mot3.TarPosition >  Mot3.CurPosition )	Mot3.TarPulseNum =  Mot3.TarPosition -  Mot3.CurPosition;
	  else Mot3.TarPulseNum = 0;
		
		// printf("M3.TarPulseNum=%d\n", Mot3.TarPulseNum);
		if( Mot3.TarPulseNum != 0 ) {
		
			Mot3EnablePidControl();
			Mot3.Action = motor_forward;
		
		}
			
}

/** 函数作用：电机3反向移动；TarPosition为目标脉冲坐标，Velocity为目标速度。 */
void Mot3MoveBackward( u16 TarPosition, u16 Velocity)
{
		printf("M3 backward\n");
		//Mot3DirPin =  NegativeMove;
		
		GPIO_ResetBits( Mot3DirIO, Mot3DirPin );
	
		Mot3.TarPosition = TarPosition;
	  Mot3.TarVelocity = Velocity;
		
		// printf("M3.CurPos=%d\n", Mot3.CurPosition);
		// printf("M3.TarPos=%d\n", Mot3.TarPosition);
	
		if(  Mot3.CurPosition >  Mot3.TarPosition )	Mot3.TarPulseNum =  Mot3.CurPosition -  Mot3.TarPosition;
	  else Mot3.TarPulseNum = 0;
	
		// printf("M3.TarPulseNum=%d\n", Mot3.TarPulseNum);
		
		if( Mot3.TarPulseNum != 0 ) {
			
			Mot3EnablePidControl();
			Mot3.Action = motor_backward;
		}
		
}

/** 函数作用：处理运动和停稳位置；寻限位时霍尔优先，未检测到霍尔的保护脉冲停机不确认限位；无传入参数。 */

void Mot3ActionHandle(void)
{
    // [调试关闭] u16 pulseAtBrake; // [MOT3刹停诊断] 保存下发PWM零输出前的脉冲数，用于区分判断延迟和停机后滑行。
    u8 isHomingToLimit = (Mot3.Reset != 0) && (Mot3.Step < 3); // 寻限位阶段优先确认已消抖的对应霍尔，脉冲仅作保护停机。
	if( Mot3.Action == motor_forward ) {
		
		Mot3VelocityControl();
		
		if( (Mot3.PulseNum >= Mot3.TarPulseNum) &&
            ((isHomingToLimit == 0) || (Mot3ReadDownHallState() == 0)) ) {
			Mot3.BrakeType = pulse_forward_brake;
			Mot3DisablePidControl();
			Mot3.Action = motor_brake;
			printf("M3pulsebrake\n");
		}	
		else if( Mot3ReadDownHallState() != 0 ){

			Mot3.BrakeType = hall_forward_brake;
			Mot3DisablePidControl();
			Mot3.Action = motor_brake;
			printf("M3hallbrake down: adc=%d, pulse=%d\r\n", Adc.Mot3.Hall2, Mot3.PulseNum); // 正向另一端由下霍尔确认，记录触发时的原始信号。
		}		
	}		
	else if( Mot3.Action == motor_backward ) {
		
		Mot3VelocityControl();

		if( (Mot3.PulseNum >= Mot3.TarPulseNum) &&
            ((isHomingToLimit == 0) || (Mot3ReadUpHallState() == 0)) ) {
			
			Mot3.BrakeType = pulse_backward_brake;
			// [调试关闭] pulseAtBrake = Mot3.PulseNum;
			Mot3DisablePidControl();
			Mot3.Action = motor_brake;
			// [调试关闭] printf("M3pulsebrake: pulse=%d, target=%d, elapsed=%d, hall1=%d, hall2=%d, pwm=0\r\n", pulseAtBrake, Mot3.TarPulseNum, Mot3.PidTimerB, Mot3.Hall1Sta, Mot3.Hall2Sta);
		}	
		else if( Mot3ReadUpHallState() != 0 ){
			
			Mot3.BrakeType = hall_backward_brake;
			Mot3DisablePidControl();
			Mot3.Action = motor_brake;
			printf("M3hallbrake up: adc=%d, pulse=%d\r\n", Adc.Mot3.Hall1, Mot3.PulseNum); // 反向寻零由上霍尔确认，记录触发时的原始信号。
		}		
	}		
	else if( Mot3.Action == motor_brake )  {
		
			if( Mot3.PulseDebB >= C_MotorStopDeb )	{
				
				printf("M3 stop\n");
				//printf("M3.PulseNum =%d\n",Mot3.PulseNum );
				//printf("M3.TarPulseNum =%d\n ",Mot3.TarPulseNum );
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
				//printf("M3.CurPos=%d\n ",Mot3.CurPosition );
				Mot3.CurAngle = (u32)(Mot3.CurPosition*c_AngleScaleFactor*c_PulseScaleFactor)/Mot3.PulseNumPerAngle; 
				//printf("M3.CurAngle=%d\n", Mot3.CurAngle);
				Mot3.CurVelocity = 0;
				Mot3.Action = motor_stop;				
		}		
	}		
}	

// 函数作用：设置电机目标角度和速度；Angle为目标角度，speed为目标速度，单位沿用原接口。
void Mot3SetAngleSpeed( u16 Angle, u16 speed )
{
		u32 sum;
		Mot3.TarVelocity = speed;
	  if( Angle >= (c_MaxAngle*10) ) Angle=c_MaxAngle*10;
		// printf("Angle =%d\n", Angle );
		// printf("speed =%d\n", speed );
		// printf("M3.PulseNumPerAngle =%d\n", Mot3.PulseNumPerAngle );
	
		sum = (u32)Angle*Mot3.PulseNumPerAngle;
	
		//printf("sum =%d\n", sum );
	
		if( sum != 0 ) Mot3.TarPosition = sum/(c_AngleScaleFactor*c_PulseScaleFactor);
		else Mot3.TarPosition = 0;
		// printf("M3.TarPos=%d\n", Mot3.TarPosition );
	
	if( Mot3.TarPosition > Mot3.CurPosition ) {
			
			Mot3MoveForward( Mot3.TarPosition, speed ); 
		}	
		else if( Mot3.TarPosition < Mot3.CurPosition ) {
			
			Mot3MoveBackward( Mot3.TarPosition, speed ); 
		}	
}	
// 函数作用：登记电机运动任务；Angle或Height为目标角度或高度，Speed为目标速度，单位沿用原接口。
void Move3SetTask( u16 Angle, u16 Speed )
{
	if( Mot3.Reset != 0 ) return ;
	if( ( Angle == Move3.Angle )&&( Speed == Move3.Speed ) ) return ;
	Move3.Angle = Angle;
	Move3.Speed = Speed;
	// printf("m3 task angle =%d\n",Angle );
	// printf("m3 task speed =%d\n",Speed );
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

/** 函数作用：首次执行两端标定，历史校准有效时先找零点侧限位再回中；无传入参数。 */
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
                                    printf("M3 zero confirmed: loaded=%d, max=%d\r\n", MotorCalibrationLoaded, Mot3.MaxPulseNum); // 核对寻零后实际采用完整标定还是历史半程回中。
									if( MotorCalibrationLoaded != 0 ) {
										// 历史标定有效：从零点侧限位（原反向运动对应上霍尔）按保存行程直接回中。
										Mot3MoveForward( Mot3.MaxPulseNum / 2, 50 );
										Mot3.Step = 3;
									}
									else {
										delay_ms(500);
										Mot3MoveForward( 20000, 100 );
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
									Mot3MoveBackward( (Mot3.MaxPulseNum/2), 50 );
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
								// [MOT3回中诊断] 回中停稳后打印一次刹停原因、两路霍尔状态和实际/目标脉冲，定位机械位置偏差。
								// [调试关闭] printf("M3 center check: brake=%d, expected=%d, hall1=%d, hall2=%d, adc1=%d, adc2=%d, pulse=%d, target=%d, max=%d\r\n", Mot3.BrakeType, (MotorCalibrationLoaded != 0) ? pulse_forward_brake : pulse_backward_brake, Mot3.Hall1Sta, Mot3.Hall2Sta, Adc.Mot3.Hall1, Adc.Mot3.Hall2, Mot3.PulseNum, Mot3.TarPulseNum, Mot3.MaxPulseNum);
								// 归位被异常限位打断时不报成功，防止把未到中点的数据当成自检成功。
								if( (Mot3.BrakeType == ((MotorCalibrationLoaded != 0) ? pulse_forward_brake : pulse_backward_brake)) &&
					(Mot3.Hall1Sta == 0) && (Mot3.Hall2Sta == 0) ) {
									Mot3.En = 1;
									Mot3.Reset = 0;
									printf("MOT3 init complete\r\n"); // 归位确认成功后打印一次初始化完成信息。
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



