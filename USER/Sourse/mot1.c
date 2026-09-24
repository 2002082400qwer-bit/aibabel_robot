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

static int32_t Mot1PidPreviousError = 0;
static int32_t Mot1PidPreviousPreviousError = 0;
static int32_t Mot1PidOutputScaled = 0;

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

    if( Mot1.PidEn == 0 ) return;
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

    // 回中行程不足两个固定窗口时，提前在半程处开始降速。
    approachPulseNum = Mot1PidApproachPulseNum;
    if( (Mot1.Reset != 0) && (Mot1.Step == 3) &&
        (Mot1.TarPulseNum < 2 * approachPulseNum) ) {
        approachPulseNum = Mot1.TarPulseNum / 2;
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
                MinimumPwm = Mot1PidApproachMinimumPwm +
                    (Mot1PidHostMinimumPwm - Mot1PidApproachMinimumPwm) *
                    remainingPulseNum / approachPulseNum;
                MaximumPwm = Mot1PidApproachMaximumPwm +
                    (c_MaxPwmDuty - Mot1PidApproachMaximumPwm) *
                    remainingPulseNum / approachPulseNum;
                if( Pwm > MaximumPwm ) Pwm = MaximumPwm;
            }
            if( Pwm < MinimumPwm ) Pwm = MinimumPwm;
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



