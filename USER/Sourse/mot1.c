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

#define Mot1VelocityKp    1

u16 Mot1VelocityFeedbackControl(u16 TargetVelocity, u16 CurrentVelocity)
{
		#if 1
		u16 Bias;                          
    static u16 ControlVelocity;

		if( TargetVelocity >= CurrentVelocity )	{	
				Bias = TargetVelocity - CurrentVelocity;
				Bias = Bias*Mot1VelocityKp;
				ControlVelocity += Bias;
				if( ControlVelocity > c_MaxPwmDuty ) ControlVelocity = c_MaxPwmDuty;
		}
		else	{
				Bias = CurrentVelocity - TargetVelocity;
				Bias = Bias*Mot1VelocityKp;
				if(ControlVelocity > Bias) ControlVelocity -= Bias;    
				else ControlVelocity = 0;
		}		
    return ControlVelocity;
		#else
		static u16 Bias, Bias_last;                          
    static u16 ControlVelocity;
	  
		if( TargetVelocity >= CurrentVelocity )	{	
				Bias = TargetVelocity - CurrentVelocity;
				if( Bias > Bias_last
				ControlVelocity += Mot1VelocityKp*Bias + Mot1VelocityKd*(Bias-Bias_last);
				Bias_last = Bias;
				if( ControlVelocity > c_MaxPwmDuty ) ControlVelocity = c_MaxPwmDuty;
		}
		else	{
				Bias = CurrentVelocity - TargetVelocity;
				Bias = Bias*Mot1VelocityKp;
				if(ControlVelocity > Bias) ControlVelocity -= Bias;    
				else ControlVelocity = 0;
		}		
    return ControlVelocity;		
		#endif
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
void Mot1EnablePidControl(void)
{
	Mot1ClearEncode();
	Mot1.CurVelocity = 0;
	Mot1.PidEn = Enable;
}
void Mot1DisablePidControl(void)
{
	TIM1_SetDuty_CH3(0);
	Mot1.PidEn = Disable;
}
void Mot1SetCurrentPosition( u16 num )
{
	Mot1.CurPosition = num;
}	
void Mot1SetTargetPosition( u16 num )
{
	Mot1.TarPosition = num;
}	
void Mot1VelocityControl(void)
{
	u16 Pwm;
	u32 sum, encode_count;	
	
	if( Mot1.PidEn == 0 )	return;	
	
	if( Mot1.PidTimer >= C_PID_RATE ) {
	
		Mot1.PidTimer = Mot1.PidTimer - C_PID_RATE;
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
				
		
		
		if( Mot1.TarPulseNum > Mot1.PulseNum ) {	
				if( ( Mot1.TarPulseNum - Mot1.PulseNum ) >= 50 ) Pwm = 2400;// Mot1VelocityFeedbackControl( Mot1.TarVelocity, Mot1.CurVelocity );
				else Pwm = Mot1VelocityFeedbackControl( 50, Mot1.CurVelocity );
		}
		if( Mot1.TarVelocity == 0 ) Pwm = 0;
		else if(  ( Mot1.TarPulseNum - Mot1.PulseNum)  >= 50 ) Pwm = 2000;
		else Pwm =1500;
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



