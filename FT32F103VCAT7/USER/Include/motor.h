#ifndef __MOTOR_H__
#define __MOTOR_H__ // 电机公共接口头文件保护标志。

#ifdef __cplusplus
extern "C" {
#endif

#include "sys.h"

#define Motor1Enable  1 // 启用电机1初始化及服务。
#define Motor2Enable  1 // 启用电机2初始化及服务。
#define Motor3Enable  1 // 启用电机3初始化及服务。
#define C_PID_RATE			  	50 		//单位豪秒
#define C_MotorStopDeb  		1000  //50ms
#define C_MinimumTarPulseNum	60 // 动作最小目标脉冲数。
#define C_VerifyMinimumPulseNum 20 // 自检有效行程的最小脉冲数。

#define PositiveMove	0 // 正向运动的方向电平标志。
#define NegativeMove 	1 // 反向运动的方向电平标志。

#define Mot1EnIO			GPIOE 		// 电机1PWM使能端口。
#define Mot1EnPin 			GPIO_Pin_13 // 电机1PWM使能引脚。
#define Mot1DirIO 			GPIOD 		// 电机1方向控制端口。
#define Mot1DirPin 			GPIO_Pin_14 // 电机1方向控制引脚。
#define Mot1PulseIO 		GPIOE 		// 电机1编码器脉冲端口。
#define Mot1PulsePin 		GPIO_Pin_15 // 电机1编码器脉冲引脚。
#define Mot1LeftSwIO		GPIOA 		// 电机1左限位霍尔端口。
#define Mot1LeftSwPin		GPIO_Pin_5 	// 电机1左限位霍尔引脚，原备选GPIO_Pin_7。
#define Mot1MiddleSwIO		GPIOC 		// 电机1中位霍尔端口。
#define Mot1MiddleSwPin		GPIO_Pin_4 	// 电机1中位霍尔引脚。
#define Mot1RightSwIO		GPIOC 		// 电机1右限位霍尔端口。
#define Mot1RightSwPin		GPIO_Pin_7 	// 电机1右限位霍尔引脚，原备选GPIO_Pin_5。

#define Mot2EnIO			GPIOE 		// 电机2PWM使能端口。
#define Mot2EnPin 			GPIO_Pin_9 	// 电机2PWM使能引脚。
#define Mot2DirIO 			GPIOD 		// 电机2方向控制端口。
#define Mot2DirPin 			GPIO_Pin_13 // 电机2方向控制引脚。
#define Mot2PulseIO 		GPIOD 		// 电机2编码器脉冲端口。
#define Mot2PulsePin 		GPIO_Pin_12 // 电机2编码器脉冲引脚。
#define Mot2UpSwIO			GPIOA 		// 电机2上限位霍尔端口。
#define Mot2UpSwPin			GPIO_Pin_5 	// 电机2上限位霍尔引脚。
#define Mot2DownSwIO		GPIOA 		// 电机2下限位霍尔端口。
#define Mot2DownSwPin		GPIO_Pin_6 	// 电机2下限位霍尔引脚。

#define Mot3EnIO 			GPIOE 		// 电机3PWM使能端口。
#define Mot3EnPin 			GPIO_Pin_11 // 电机3PWM使能引脚。
#define Mot3DirIO 			GPIOA 		// 电机3方向控制端口。
#define Mot3DirPin 			GPIO_Pin_8 	// 电机3方向控制引脚。
#define Mot3PulseIO 		GPIOC 		// 电机3编码器脉冲端口。
#define Mot3PulsePin 		GPIO_Pin_9 	// 电机3编码器脉冲引脚。
#define Mot3UpSwIO			GPIOA 		// 电机3上限位霍尔端口。
#define Mot3UpSwPin			GPIO_Pin_4 	// 电机3上限位霍尔引脚。
#define Mot3DownSwIO		GPIOA 		// 电机3下限位霍尔端口。
#define Mot3DownSwPin		GPIO_Pin_3 	// 电机3下限位霍尔引脚。

#define Mot1MaxAngle    180 //单位度
#define Mot2MaxDistance 500 //单位毫米
#define Mot3MaxAngle    65	//单位度

#define Enable 	1 // 通用使能标志。
#define Disable 0 // 通用禁用标志。

//电机状态
enum motorState
{
	motor_stop = 0,
	motor_brake,
	motor_forward,
	motor_backward,
};
//电机减速类型
enum motorBrake
{
	pulse_forward_brake=0,	//脉冲减速
	hall_forward_brake,		//霍尔减速
	pulse_backward_brake,	//脉冲减速
	hall_backward_brake,	//霍尔减速
};

enum motorFault
{
	NoFault = 0,
	Hall1Fault,
	Hall2Fault,
	Hall3Fualt,
	MotorFault,
};

extern uint8_t MotorEn; // 电机服务初始化完成标志。
extern u8 MotorCalibrationLoaded; // 本次启动是否读取到有效的历史校准数据。
#define Motor2CalibrationHeight 510 // 电机2原有标定高度，单位毫米。

void MotorInit(void);
void MotorService(void);



#ifdef __cplusplus
}
#endif

#endif
