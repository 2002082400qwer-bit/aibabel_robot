#include "adc.h"

#define debug 1

//采集通道总数：外部13路 + 温度 + 内部基准 = 15路
//DMA缓存数组，存放5路ADC原始值
volatile  uint16_t adc_dma_buf[ADC_CH_COUNT];
float vol3v3=3.3f;
AdcSt	Adc;

void ADC1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
    ADC_InitTypeDef ADC_InitStruct;
	DMA_InitTypeDef         DMA_InitStruct;
	
    // 1. 开启时钟：GPIOA、GPIOC,ADC1、DMA1
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1, ENABLE);
	// 2. PA1~PA7,PC0~PC5设置为模拟输入
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN; //模拟输入
    GPIO_Init(GPIOA, &GPIO_InitStruct);		
	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN; //输入浮空
    GPIO_Init(GPIOC, &GPIO_InitStruct);		
	
	// 3. ADC时钟配置 APB2=72MHz，72/6=12MHz < 14MHz上限
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
    // 4. ADC基础配置：扫描模式+连续转换+DMA
    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStruct.ADC_ScanConvMode = ENABLE;       // 多通道扫描开启
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE; // 连续循环转换
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 软件触发
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfChannel = ADC_CH_COUNT; // 一共15个通道
    ADC_Init(ADC1, &ADC_InitStruct);

	// 5. 配置规则通道转换顺序(Rank 1~15)
    //通道0，序列1，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);
    //通道0，序列2，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_239Cycles5);
    //通道0，序列3，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_239Cycles5);
    //通道0，序列4，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_239Cycles5);
    //通道0，序列5，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 5, ADC_SampleTime_239Cycles5);
    //通道0，序列6，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 6, ADC_SampleTime_239Cycles5);
    //通道0，序列7，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 7, ADC_SampleTime_239Cycles5);
    //通道0，序列8，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 8, ADC_SampleTime_239Cycles5);
    //通道0，序列9，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 9, ADC_SampleTime_239Cycles5);
    //通道0，序列10，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 10, ADC_SampleTime_239Cycles5);
    //通道0，序列11，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 11, ADC_SampleTime_239Cycles5);
    //通道0，序列12，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 12, ADC_SampleTime_239Cycles5);
    //通道0，序列13，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 13, ADC_SampleTime_239Cycles5);
    //通道0，序列14，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 14, ADC_SampleTime_239Cycles5);
    //通道0，序列15，采样时间239.5周期（采样时间长，抗干扰好）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 15, ADC_SampleTime_239Cycles5);

	ADC_TempSensorVrefintCmd(ENABLE);
		
    // 开启ADC DMA输出
    ADC_DMACmd(ADC1, ENABLE);

	// 6. DMA1 CH1 循环模式
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)adc_dma_buf;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStruct.DMA_BufferSize = ADC_CH_COUNT;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStruct);
	DMA_Cmd(DMA1_Channel1, ENABLE);

    // 7. 使能ADC并执行校准（关键，提升精度）
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
		
	// 8. 启动ADC软件转换，之后自动循环扫描所有通道
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

// 根据ADC原始值计算实际电压，VREF=3.3V
float ADC_Get_Vol(uint16_t adc_raw)
{
	//Printf("adc raw=%d\r\n",adc_raw);
	float vol;
	vol = (float)adc_raw * vol3v3 / 4095.0f;
    return vol;
}

// 获取芯片内部温度（单位℃）
//float ADC_Get_Temp(void)
//{
//    uint16_t temp_adc = adc_dma_buf[CH_TEMP];
//    uint16_t vref_adc = adc_dma_buf[CH_VREF];

//    // 真实内部基准电压 1.2V，修正3.3V参考误差
//    float vref_real = 1.2f * 4095.0f / vref_adc;
//    float temp_vol = (float)temp_adc * vref_real / 4095.0f;

//    // FT32内部温度传感器标准参数：V25=1.43V，斜率4.3mV/℃
//    float temp = (temp_vol - 1.43f) / 0.0043f + 25.0f;
//    return temp;
//}

void AdcInit(void)
{
	
	ADC1_Init();
	
	Adc.Mot1.Vout = 0;
	Adc.Mot1.Vref = 0;
	Adc.Mot1.Hall1 = 0;
	Adc.Mot1.Hall2 = 0;
	Adc.Mot1.Hall3 = 0;
	
	Adc.Mot2.Vout = 0;
	Adc.Mot2.Vref = 0;
	Adc.Mot2.Hall1 = 0;
	Adc.Mot2.Hall2 = 0;
	Adc.Mot2.Hall3 = 0;
	
	Adc.Mot3.Vout = 0;
	Adc.Mot3.Vref = 0;
	Adc.Mot3.Hall1 = 0;
	Adc.Mot3.Hall2 = 0;
	Adc.Mot3.Hall3 = 0;
	
	Adc.idx = 0;
}	


// [打印精简] 函数作用：周期采集电流和霍尔信号；无传入参数。
void AdcService(void)
{
		uint16_t adc_raw;
		if( Adc.Timer >= c_10ms )
		{
				Adc.Timer = 0;
				switch( Adc.idx++ )
				{
					case 0:
							adc_raw = adc_dma_buf[CH_VREF];
							vol3v3 = 1.2f * 4095.0f / adc_raw;
							break;
					case 1:						
							Adc.Mot1.Vout = ADC_Get_Vol( adc_dma_buf[CH_PA1]);
							Adc.Mot1.Vref = ADC_Get_Vol( adc_dma_buf[CH_PA2]);
							//5A    （零电流电压-AD值电压）/246mV 计算结果A
						    Adc.Mot1.Current = ( (Adc.Mot1.Vref - Adc.Mot1.Vout ) / 0.246 )*1000;
							Adc.Mot1.Hall1 = adc_dma_buf[CH_PA7];							
							Adc.Mot1.Hall2 = adc_dma_buf[CH_PC4];
							Adc.Mot1.Hall3 = adc_dma_buf[CH_PC5];
							#if debug == 1
							// [打印精简] 暂停调试数据，按需恢复： Printf("Adc.Mot1.Hall1=%d\r\n",Adc.Mot1.Hall1);
							// [打印精简] 暂停调试数据，按需恢复： Printf("Adc.Mot1.Hall2=%d\r\n",Adc.Mot1.Hall2);
							// [打印精简] 暂停调试数据，按需恢复： Printf("Adc.Mot1.Hall3=%d\r\n",Adc.Mot1.Hall3);
							#endif
							break;
					case 2:
							Adc.Mot2.Vout = ADC_Get_Vol( adc_dma_buf[CH_PC2]);
							Adc.Mot2.Vref = ADC_Get_Vol( adc_dma_buf[CH_PC3]);
							//5A    （零电流电压-AD值电压）/246mV 计算结果A
						    Adc.Mot2.Current = ( (Adc.Mot2.Vref - Adc.Mot2.Vout ) / 0.246 )*1000;
							Adc.Mot2.Hall1 = adc_dma_buf[CH_PA5];
							Adc.Mot2.Hall2 = adc_dma_buf[CH_PA6];
							#if debug == 1
							// [打印精简] 暂停调试数据，按需恢复： Printf("Adc.Mot2.Hall2=%d\r\n",Adc.Mot2.Hall2);
							// [打印精简] 暂停调试数据，按需恢复： Printf("Adc.Mot2.Hall1=%d\r\n",Adc.Mot2.Hall1);
							#endif
							break;
					case 3:
							Adc.Mot3.Vout = ADC_Get_Vol( adc_dma_buf[CH_PC0]);
							Adc.Mot3.Vref = ADC_Get_Vol( adc_dma_buf[CH_PC1]);
							//5A    （零电流电压-AD值电压）/246mV 计算结果A
						    Adc.Mot3.Current = ( (Adc.Mot3.Vref - Adc.Mot3.Vout ) / 0.246 )*1000;
							Adc.Mot3.Hall1 = adc_dma_buf[CH_PA4];
							Adc.Mot3.Hall2 = adc_dma_buf[CH_PA3];
							#if debug == 1
							// [打印精简] 暂停调试数据，按需恢复： Printf("Adc.Mot3.Hall1=%d\r\n",Adc.Mot3.Hall1);
							// [打印精简] 暂停调试数据，按需恢复： Printf("Adc.Mot3.Hall2=%d\r\n",Adc.Mot3.Hall2);
							#endif
							Adc.idx = 1;
							break;
				}	
		}		
}	
