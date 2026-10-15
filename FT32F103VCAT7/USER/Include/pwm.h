#ifndef __PWM_H
#define	__PWM_H

#include "sys.h"


#define c_MaxPwmDuty 4800
#define c_testPwmDuty (4800*0.7)
#define c_testPwmDutyMot3 (4800*0.5)
void PwmInit(void); // [编译修正] 初始化三路电机PWM并清零占空比；无传入参数。
void TIM1_PWM_Init(uint16_t psc,uint16_t arr);
void TIM1_SetDuty_CH1(uint16_t compare);
void TIM1_SetDuty_CH2(uint16_t compare);
void TIM1_SetDuty_CH3(uint16_t compare);

#endif
