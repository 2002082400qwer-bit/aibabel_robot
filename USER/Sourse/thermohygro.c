//-----------------------------------------------------------------
//	include 
//-----------------------------------------------------------------
	
#include "thermohygro.h"	
//-----------------------------------------------------------------
//	constant 
//-----------------------------------------------------------------
#define GXHT40_ADDR   0x44

#define GXHT40_SCL_Pin 	GPIO_Pin_11
#define GXHT40_SDA_Pin 	GPIO_Pin_10

/*
#define GXHT40_SCL_H  GPIO_SetBits( GPIOD, GXHT40_SCL_Pin );
#define GXHT40_SCL_L  GPIO_ResetBits( GPIOD, GXHT40_SCL_Pin );
#define GXHT40_SDA_H  GPIO_InitStruct.GPIO_Pin=GXHT40_SDA_Pin;GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING; GPIO_Init( GPIOD, &GPIO_InitStruct );	
#define GXHT40_SDA_L  GPIO_InitStruct.GPIO_Pin=GXHT40_SDA_Pin;GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;GPIO_Init(GPIOD,&GPIO_InitStruct);GPIO_ResetBits(GPIOD,GXHT40_SDA_Pin);
#define GXHT40_SDA_READ  GPIO_ReadInputDataBit( GPIOD, GXHT40_SDA_Pin )
*/

#define debug 0

//-----------------------------------------------------------------
//	ram 
//-----------------------------------------------------------------
//static GPIO_InitTypeDef GPIO_InitStruct;
uint16_t Temperatrue;
int16_t Humidity;
uint8_t  THTimer;
uint8_t  THStep;
void GXHT40_SCL_H(void)
{
	GPIO_SetBits( GPIOD, GXHT40_SCL_Pin );
}	
void GXHT40_SCL_L(void)
{
	GPIO_ResetBits( GPIOD, GXHT40_SCL_Pin );
}	
void GXHT40_SDA_H(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin=GXHT40_SDA_Pin;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING; 
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // 必须！
	GPIO_Init( GPIOD, &GPIO_InitStruct );	
}	
void GXHT40_SDA_L(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin=GXHT40_SDA_Pin;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // 必须！
	GPIO_Init(GPIOD,&GPIO_InitStruct);
	GPIO_ResetBits(GPIOD,GXHT40_SDA_Pin);
}	
uint8_t GXHT40_SDA_READ(void)
{
	return GPIO_ReadInputDataBit( GPIOD, GXHT40_SDA_Pin );
}	
static void SW_I2C_Start(void)
{
    GXHT40_SDA_H();
    GXHT40_SCL_H();
    delay_us(10);
    GXHT40_SDA_L();
    delay_us(10);
    GXHT40_SCL_L();
}

static void SW_I2C_Stop(void)
{
	GXHT40_SCL_L();
    GXHT40_SDA_L();
    GXHT40_SCL_H();
    delay_us(5);
    GXHT40_SDA_H();
}

static void SW_I2C_WriteByte( uint8_t dat )
{
    uint8_t i;
    for(i=0; i<8; i++)
    {
        GXHT40_SCL_L();
        delay_us(5);
        if(dat & 0x80) 
		{
			GXHT40_SDA_H();
		}	
        else            
		{
			GXHT40_SDA_L();
		}	
        dat <<= 1;
        delay_us(5);
        GXHT40_SCL_H();
        delay_us(5);
    }
    GXHT40_SCL_L(); // 等待ACK
    GXHT40_SDA_H(); // 释放SDA
    delay_us(5);
    GXHT40_SCL_H();
	delay_us(5);
	GXHT40_SCL_L();
	delay_us(5);
}

static uint8_t SW_I2C_ReadByte(void)
{
    uint8_t i, dat=0;
    GXHT40_SDA_H();
    for(i=0; i<8; i++)
    {
        GXHT40_SCL_L();
        delay_us(5);
        GXHT40_SCL_H();
        delay_us(5);
        dat <<= 1;
        if(GXHT40_SDA_READ()) dat |= 1;
    }
	GXHT40_SCL_L();	
	GXHT40_SDA_L();
	delay_us(5);
	GXHT40_SCL_H();
	delay_us(5);
	GXHT40_SCL_L();
	delay_us(5);

    return dat;
}

void GXHT40_Reset(void)
{
    SW_I2C_Start();
    SW_I2C_WriteByte(GXHT40_ADDR<<1);
    SW_I2C_WriteByte(0x94); // reset
    SW_I2C_Stop();
    delay_ms(10);
}

void GXHT40Init(void)
{
	
    GXHT40_Reset();
	THStep = 0;
}	

void ThermohygroInit(void)
{
	
	THTimer = 0;
	Temperatrue = 0;
	Humidity = 0;
	GXHT40Init();
}	

// [打印精简] 函数作用：读取并换算温湿度数据；无传入参数。
void ThermohygroService(void)
{
	u8 buf[6],i;
	float f_temp,f_humi; 
	if( THTimer >= c_20ms )
	{
		THTimer = 0;
		if( THStep == 0 )
		{	
			//发送测量命令
			SW_I2C_Start();
			SW_I2C_WriteByte(GXHT40_ADDR<<1);
			SW_I2C_WriteByte(0xFD); // 高精度测量
			SW_I2C_Stop();
			THStep = 1;
		}
		else
		{
			// 读取6字节
			SW_I2C_Start();
			SW_I2C_WriteByte((GXHT40_ADDR<<1)|1);
			for( i=0; i<5; i++) buf[i]=SW_I2C_ReadByte();
			buf[5]=SW_I2C_ReadByte();
			SW_I2C_Stop();
			u16 t_raw = (buf[0]<<8)|buf[1];
			u16 h_raw = (buf[3]<<8)|buf[4];
//				printf("t_raw=%d\n",t_raw);
//				printf("h_raw=%d\n",h_raw);
			// 转换公式(GXHT40/SHT40 标准)
			f_temp =  -45.0f + 175.0f * t_raw / 65535.0f ;
			f_humi =  -6.0f + 125.0f * h_raw / 65535.0f;
//				printf("f_temp=%0.1f\n",f_temp);
//				printf("f_humi=%0.1f\n",f_humi);
			Temperatrue = f_temp*10;
			Humidity = f_humi*10;
			
			
			if(Humidity >1000 ) Humidity = 1000;
			if(Humidity <0 )   Humidity = 0;

			#if debug == 1
			// [打印精简] 暂停调试数据，按需恢复： printf("temp=%d\n",Temperatrue);
			// [打印精简] 暂停调试数据，按需恢复： printf("humi=%d\n",Humidity);								
			#endif
			
			THStep = 0;
		}		
	}		
}	
