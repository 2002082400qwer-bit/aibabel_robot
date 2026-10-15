#include "cs1237.h"

#define SCK1_IO 	GPIOB
#define SCK1_Pin 	GPIO_Pin_8
#define SDA1_IO 	GPIOB
#define SDA1_Pin 	GPIO_Pin_9

#define SCK2_IO 	GPIOE
#define SCK2_Pin  GPIO_Pin_0
#define SDA2_IO   GPIOE
#define SDA2_Pin  GPIO_Pin_1

#define rcmd 0x56
#define wcmd 0x65 
				
#define REFO_OFF (1<<6)
#define REFO_ON  (0<<1)

#define SPEED_SEL_10HZ 	(0<<4)
#define SPEED_SEL_40HZ 	(1<<4)
#define SPEED_SEL_640HZ (2<<4)
#define SPEED_SEL_128HZ (3<<4)

#define PAG_SEL_1 	(0<<2)
#define PAG_SEL_2 	(1<<2)
#define PAG_SEL_64 	(2<<2)
#define PAG_SEL_128 (3<<2)

#define CHA_SEL_A 			(0<<0)
#define CHA_SEL_REV 		(1<<0)
#define CHA_SEL_TEMP 		(2<<0)
#define CHA_SEL_NEIDUN 	(3<<0)

#define w_1g 2000

#define w_gross 398000

#define debug 1

cs1237St uCs;

/*****************************************************************************
	CS1237 1 
*****************************************************************************/
void Sck1Output(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin   = SCK1_Pin;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP; 
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // 必须！
  GPIO_Init( SCK1_IO, &GPIO_InitStruct );	
}	
void Sck1Low(void)
{
	GPIO_ResetBits( SCK1_IO, SCK1_Pin );
}	
void Sck1High(void)
{
	GPIO_SetBits( SCK1_IO, SCK1_Pin );
}	
void Sda1Input(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin   = SDA1_Pin;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING; //输入浮空
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // 必须！
  GPIO_Init( SDA1_IO, &GPIO_InitStruct );	
}		
uint8_t Sda1Data(void)
{
	return GPIO_ReadInputDataBit( SDA1_IO, SDA1_Pin );
}	
void Sda1Output(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin   = SDA1_Pin;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP; 
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // 必须！
  GPIO_Init(SDA1_IO, &GPIO_InitStruct);	
}		
void Sda1Low(void)
{
	GPIO_ResetBits( SDA1_IO, SDA1_Pin );
}	
void Sda1High(void)
{
	GPIO_SetBits( SDA1_IO, SDA1_Pin );
}	
/*****************************************************************************
	CS1237 2 
*****************************************************************************/
void Sck2Output(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin   = SCK2_Pin;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP; 
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // 必须！
  GPIO_Init( SCK2_IO, &GPIO_InitStruct );	
}	
void Sck2Low(void)
{
	GPIO_ResetBits( SCK2_IO, SCK2_Pin );
}	
void Sck2High(void)
{
	GPIO_SetBits( SCK2_IO, SCK2_Pin );
}	
void Sda2Input(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin   = SDA2_Pin;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING; //输入浮空
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // 必须！
  GPIO_Init(SDA2_IO, &GPIO_InitStruct);	
}		
uint8_t Sda2Data(void)
{
	return GPIO_ReadInputDataBit( SDA2_IO, SDA2_Pin );
}	
void Sda2Output(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin   = SDA2_Pin;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP; 
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // 必须！
  GPIO_Init( SDA2_IO, &GPIO_InitStruct );	
}		
void Sda2Low(void)
{
	GPIO_ResetBits( SDA2_IO, SDA2_Pin );
}	
void Sda2High(void)
{
	GPIO_SetBits( SDA2_IO, SDA2_Pin );
}	


void Cs1237Init(void)
{
	uCs.seq = 0;
	uCs.idx = 0;
	uCs.buf[0] = 0;
	uCs.buf[1] = 0;
	uCs.buf[2] = 0;
	uCs.buf[3] = 0;
	uCs.u1g = 0;
	uCs.u2g = 0;
	uCs.timer = 0;
	
	Sck1Output();
	Sck1Low();
	delay_ms(20);
	Sck1High();
	Sda1Output();
	Sda1High();
	
	Sck2Output();
	Sck2Low();
	delay_ms(20);
	Sck2High();
	Sda2Output();
	Sda2High();
}	

// [打印精简] 函数作用：读取两路称重数据并换算重量；无传入参数。
void Cs1237Service(void)
{
	//if( uCs.timer < c_10ms ) return;
	
	//uCs.timer = 0;		
	
	u32 cs_adc;
	int32_t s_adc;

	uint8_t state1,state2,Config,wData;
	
	switch( uCs.seq )
	{
		case 0:
				Sda1Input();
				Sck1Low();
				Sda2Input();
				Sck2Low();
				uCs.idx = 0;
				uCs.seq = 1;
				break;
/*****************************************************************************
	CS1237 1 10hz 采样
*****************************************************************************/

		case 1:
				if( Sda1Data() ) return;
				cs_adc = 0;
				delay_us(1);
				//1~24 clock
				for( uint8_t i =0; i < 24 ; i++ ) {
					Sck1High();
					delay_us(1);
					cs_adc <<= 1;
					if( Sda1Data() ){
						cs_adc |= 0x00000001;
					}    		
					Sck1Low();
					delay_us(1);
				}
				
				//25 clock
				state1 = 0;
				Sck1High();
				delay_us(1);
				if( Sda1Data() ){
						state1 = 1;
				}    		
				Sck1Low();
				delay_us(1);
				
				//26 clock
				state2 = 0;
				Sck1High();
				delay_us(1);
				cs_adc <<= 1;
				if( Sda1Data() ){
						state2 = 1;
				}    		
				Sck1Low();
				delay_us(1);
				
				//27 clock
				Sck1High();
				Sck1Low();		
				
				//sda output
				Sda1Output();	
				Sda1Low();
				
				//28 clock
				Sck1High();		
				Sck1Low();
				
				//29 clock
				Sck1High();		
				Sck1Low();
				
				//30~36 CLOCK

				wData = (wcmd<<1);
				
				for( uint8_t i =0; i<7; i++ ) {
					if( (wData & 0x80) !=0 ) 	Sda1High();	
					else Sda1Low();
					wData = wData<<1;
					Sck1High();
					Sck1Low();
				}	
				
				//37 clock
				Sck1High();		
				Sck1Low();
				
				//38~45 CLOCK
				Config = REFO_ON|SPEED_SEL_40HZ|PAG_SEL_128|CHA_SEL_A;
				for( uint8_t i =0; i<8; i++ ) {
					if( Config & 0x80 ) 	Sda1High();	
					else Sda1Low();
					Config = Config<<1;
					Sck1High();
					Sck1Low();
				}
				
				Sda1Input();	
				
				//46 Clock
				Sck1High();
				Sck1Low();
				
				//if( (cs_adc&0x800000 ) !=0 ) return;
				
				s_adc = ((int32_t)cs_adc << 8 )>>8;
				
				#if debug == 1
				// [打印精简] 暂停调试数据，按需恢复： printf( "s_adc = %ld\n", s_adc );
				#endif
				uCs.buf[uCs.idx] = abs(s_adc);
				
				
				uCs.idx++;
				
				if( uCs.idx < 4 ) {
					return;
				}
				else {
					uint32_t total_adc = 0;
					for( uint8_t i=0; i<4; i++ ) {
						total_adc += uCs.buf[i];
					}	
					
					total_adc=total_adc>>2;
					
					if( total_adc > w_gross ) {
						uCs.u1g = total_adc - w_gross;
					}	
					else {
						uCs.u1g = 0;
					}
					uCs.u1g/=w_1g;
					#if debug == 1
					// [打印精简] 暂停调试数据，按需恢复： printf("uCs.u1g=%ld\n",uCs.u1g);
					#endif
					uCs.idx = 0;
					uCs.seq = 2;
				}	
				break;
/*****************************************************************************
	CS1237 2 10hz 采样
*****************************************************************************/
		case 2:
				if( Sda2Data() ) return;
				cs_adc = 0;
				delay_us(1);
				//1~24 clock
				for( uint8_t i =0; i < 24 ; i++ ) {
					Sck2High();
					delay_us(1);
					cs_adc <<= 1;
					if( Sda2Data() ){
						cs_adc |= 0x00000001;
					}    		
					Sck2Low();
					delay_us(1);
				}
				
				//25 clock
				state1 = 0;
				Sck2High();
				delay_us(1);
				if( Sda2Data() ){
						state1 = 1;
				}    		
				Sck2Low();
				delay_us(1);
				
				//26 clock
				state2 = 0;
				Sck2High();
				delay_us(1);
				cs_adc <<= 1;
				if( Sda2Data() ){
						state2 = 1;
				}    		
				Sck2Low();
				delay_us(1);
				
				//27 clock
				Sck2High();
				Sck2Low();		
				
				//sda output
				Sda2Output();	
				Sda2Low();
				
				//28 clock
				Sck2High();		
				Sck2Low();
				
				//29 clock
				Sck2High();		
				Sck2Low();
				
				//30~36 CLOCK

				wData = (wcmd<<1);
				
				for( uint8_t i =0; i<7; i++ ) {
					if( (wData & 0x80) !=0 ) 	Sda2High();	
					else Sda2Low();
					wData = wData<<1;
					Sck2High();
					Sck2Low();
				}	
				
				//37 clock
				Sck2High();		
				Sck2Low();
				
				//38~45 CLOCK
				Config = REFO_ON|SPEED_SEL_40HZ|PAG_SEL_128|CHA_SEL_A;
				for( uint8_t i =0; i<8; i++ ) {
					if( Config & 0x80 ) 	Sda2High();	
					else Sda2Low();
					Config = Config<<1;
					Sck2High();
					Sck2Low();
				}
				
				Sda2Input();	
				
				//46 Clock
				Sck2High();
				Sck2Low();
				
				//if( (cs_adc&0x800000 ) !=0 ) return;
				
				s_adc = ((int32_t)cs_adc << 8 )>>8;
				#if debug == 1
				// [打印精简] 暂停调试数据，按需恢复： printf( "s_adc = %ld\n", s_adc );
				#endif
				uCs.buf[uCs.idx] = abs(s_adc);
				
				
				uCs.idx++;
				
				if( uCs.idx < 4 ) {
					return;
				}
				else {
					uint32_t total_adc = 0;
					for( uint8_t i=0; i<4; i++ ) {
						total_adc += uCs.buf[i];
					}	
					
					total_adc=total_adc>>2;
					
					if( total_adc > w_gross ) {
						uCs.u2g = total_adc - w_gross;
					}	
					else {
						uCs.u2g = 0;
					}
					uCs.u2g/=w_1g;
					#if debug == 1
					// [打印精简] 暂停调试数据，按需恢复： printf("uCs.u2g=%ld\n",uCs.u2g);
					#endif
					uCs.idx = 0;
					uCs.seq = 1;	
				}		
				break;				
	}	
}	
