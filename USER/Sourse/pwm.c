#include "pwm.h"

/*
	F_pwm = ftimer/(PSC+1)*(ARR+1)	
*/

/* 15K PWM, DUTY: 0~4800 */

#define C_PSC 0
#define C_ARR 4799 

void PwmInit(void)
{
		TIM1_PWM_Init( C_PSC, C_ARR);
		//GPIO_SetBits(Mot2DirIO, Mot2DirPin);
		//GPIO_ResetBits(Mot2DirIO, Mot2DirPin);
		TIM1_SetDuty_CH1( 0 );	//MOT2		
		TIM1_SetDuty_CH2( 0 );	//MOT3
		TIM1_SetDuty_CH3( 0 );	//MOT1
}	

// TIM1 3路PWM初始化 PE9 CH1 / PE11 CH2 / PE13 CH3
// arr:自动重装载值, psc:预分频
void TIM1_PWM_Init(uint16_t psc,uint16_t arr)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    
    // 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO,ENABLE);
    
    // TIM1完全重映射！关键：将TIM1通道映射到PE口
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);
    
    // PE9 PE11 PE13 复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE,&GPIO_InitStructure);
    
    // 时基配置
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
    
    // PWM模式配置
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    // TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;        // 低电平有效
    // CH1
    TIM_OC1Init(TIM1,&TIM_OCInitStructure);
    // CH2
    TIM_OC2Init(TIM1,&TIM_OCInitStructure);
    // CH3
    TIM_OC3Init(TIM1,&TIM_OCInitStructure);

    
    // 使能预装载
    TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1,ENABLE);
    
    // 高级定时器必须使能MOE主输出！否则无波形输出
    TIM_CtrlPWMOutputs(TIM1,ENABLE);
    TIM_Cmd(TIM1,ENABLE);
}

// 设置各路占空比
// compare范围：0 ~ arr
void TIM1_SetDuty_CH1(uint16_t compare)
{
    TIM_SetCompare1(TIM1,compare);
}
void TIM1_SetDuty_CH2(uint16_t compare)
{
    TIM_SetCompare2(TIM1,compare);
}
void TIM1_SetDuty_CH3(uint16_t compare)
{
    TIM_SetCompare3(TIM1,compare);
}
