#include "com.h"

#define  Head1 0x5a
#define  Head2 0xa5

ComSt uCom;

void ComInit(void)
{
	for( uint8_t i=0; i<BufLen; i++ ) {
		uCom.SendBuf[i] = 0;
		uCom.RecBuf[i] = 0;
	}	 
	uCom.SendBuf[0] = 0;
	uCom.SendBuf[1] = 0;
	uCom.SendBuf[2] = 0x5a;
	uCom.SendBuf[3] = 0xa5;

	SPI2_SLAVE_GPIO_Init();
  SPI2_SLAVE_Init();
  SPI2_DMA_Init();
	SPI2_Slave_Start_DMA();
}	

// [打印精简] 函数作用：处理SPI收发和控制命令；无传入参数。
void ComService(void)
{
	uint8_t i;
	uint16_t angle, height, speed;
	
	if( spi_dma_rx_complete ) {
		
		//enableInterrupts();	
		
		// [打印精简] 暂停调试数据，按需恢复： printf("recieve spi dma data.\n");
		#if 1
		for( i=0; i<BufLen; i++ )  {
			uCom.RecBuf[i] = spi_rx_buf[i];
			// [打印精简] 暂停调试数据，按需恢复： printf("%d,",uCom.RecBuf[i]);
		}
		#endif

		uCom.SendBuf[0] = 0;
		uCom.SendBuf[1] = 0;
		uCom.SendBuf[2] = 0x5a;
		uCom.SendBuf[3] = 0xa5;
		
		uCom.SendBuf[4]  = Touch.val;                  // Touch key value

		uCom.SendBuf[5]  = (Temperatrue>>0)&0x00ff;    // Temperature LSB, uint16
		uCom.SendBuf[6]  = (Temperatrue>>8)&0x00ff;    // Temperature MSB

		uCom.SendBuf[7]  = (Humidity>>0)&0x00ff;        // Humidity LSB, uint16
		uCom.SendBuf[8]  = (Humidity>>8)&0x00ff;        // Humidity MSB

		uCom.SendBuf[9]  = (Ult.Dist1>>0)&0x00ff;      // Ultrasonic1 distance LSB(mm), uint16
		uCom.SendBuf[10] = (Ult.Dist1>>8)&0x00ff;      // Ultrasonic1 distance MSB

		uCom.SendBuf[11] = (Ult.Dist2>>0)&0x00ff;      // Ultrasonic2 distance LSB(mm), uint16
		uCom.SendBuf[12] = (Ult.Dist2>>8)&0x00ff;      // Ultrasonic2 distance MSB

		uCom.SendBuf[13] = (tof8806_distance_mm>>0)&0x00ff;  // TOF8806 distance LSB(mm), uint16
		uCom.SendBuf[14] = (tof8806_distance_mm>>8)&0x00ff;  // TOF8806 distance MSB
		uCom.SendBuf[15] = tof8806_temperature;              // TOF chip internal temperature, uint8

		uCom.SendBuf[16] = (uCs.u1g>>0)&0x000000ff;    // Weight?1 byte0(LSB), uint32
		uCom.SendBuf[17] = (uCs.u1g>>8)&0x000000ff;    // Weight?1 byte1
		uCom.SendBuf[18] = (uCs.u1g>>16)&0x000000ff;   // Weight?1 byte2
		uCom.SendBuf[19] = (uCs.u1g>>24)&0x000000ff;   // Weight?1 byte3(MSB)

		uCom.SendBuf[20] = (uCs.u2g>>0)&0x000000ff;    // Weight?2 byte0(LSB), uint32
		uCom.SendBuf[21] = (uCs.u2g>>8)&0x000000ff;    // Weight?2 byte1
		uCom.SendBuf[22] = (uCs.u2g>>16)&0x000000ff;   // Weight?2 byte2
		uCom.SendBuf[23] = (uCs.u2g>>24)&0x000000ff;   // Weight?2 byte3(MSB)

		uCom.SendBuf[24] = Mot1.Action;                // Motor1 action status code, uint8
		uCom.SendBuf[25] = (Mot1.CurAngle>>0)&0x00ff;  // Motor1 current angle LSB, uint16
		uCom.SendBuf[26] = (Mot1.CurAngle>>8)&0x00ff;  // Motor1 current angle MSB
		uCom.SendBuf[27] = (Mot1.CurVelocity>>0)&0x00ff;// Motor1 angular velocity LSB, uint16
		uCom.SendBuf[28] = (Mot1.CurVelocity>>8)&0x00ff;// Motor1 angular velocity MSB

		uCom.SendBuf[29]  = (Adc.Mot1.Current>>0)&0x00ff;  // Motor1 ADC current LSB, uint16
		uCom.SendBuf[30]  = (Adc.Mot1.Current>>8)&0x00ff;  // Motor1 ADC current MSB
		uCom.SendBuf[31]  = (Adc.Mot1.Hall1>>0)&0x00ff;    // Motor1 Hall?1 LSB, uint16
		uCom.SendBuf[32]  = (Adc.Mot1.Hall1>>8)&0x00ff;    // Motor1 Hall?1 MSB
		uCom.SendBuf[33]  = (Adc.Mot1.Hall2>>0)&0x00ff;    // Motor1 Hall?2 LSB, uint16
		uCom.SendBuf[34]  = (Adc.Mot1.Hall2>>8)&0x00ff;    // Motor1 Hall?2 MSB
		uCom.SendBuf[35]  = (Adc.Mot1.Hall3>>0)&0x00ff;    // Motor1 Hall?3 LSB, uint16
		uCom.SendBuf[36]  = (Adc.Mot1.Hall3>>8)&0x00ff;    // Motor1 Hall?3 MSB

		uCom.SendBuf[37] = Mot2.Action;                 // Motor2 action status code, uint8
		uCom.SendBuf[38] = (Mot2.CurHeight>>0)&0x00ff; // Motor2 current height LSB, uint16
		uCom.SendBuf[39] = (Mot2.CurHeight>>8)&0x00ff; // Motor2 current height MSB
		uCom.SendBuf[40] = (Mot2.CurVelocity>>0)&0x00ff;// Motor2 velocity LSB, uint16
		uCom.SendBuf[41] = (Mot2.CurVelocity>>8)&0x00ff;// Motor2 velocity MSB

		uCom.SendBuf[42]  = (Adc.Mot2.Current>>0)&0x00ff; // Motor2 ADC current LSB, uint16
		uCom.SendBuf[43]  = (Adc.Mot2.Current>>8)&0x00ff; // Motor2 ADC current MSB
		uCom.SendBuf[44]  = (Adc.Mot2.Hall1>>0)&0x00ff;   // Motor2 Hall?1 LSB, uint16
		uCom.SendBuf[45]  = (Adc.Mot2.Hall1>>8)&0x00ff;   // Motor2 Hall?1 MSB
		uCom.SendBuf[46]  = (Adc.Mot2.Hall2>>0)&0x00ff;   // Motor2 Hall?2 LSB, uint16
		uCom.SendBuf[47]  = (Adc.Mot2.Hall2>>8)&0x00ff;   // Motor2 Hall?2 MSB

		uCom.SendBuf[48] = Mot3.Action;                 // Motor3 action status code, uint8
		uCom.SendBuf[49] = (Mot3.CurAngle>>0)&0x00ff;  // Motor3 current angle LSB, uint16
		uCom.SendBuf[50] = (Mot3.CurAngle>>8)&0x00ff;  // Motor3 current angle MSB
		uCom.SendBuf[51] = (Mot3.CurVelocity>>0)&0x00ff;// Motor3 angular velocity LSB, uint16
		uCom.SendBuf[52] = (Mot3.CurVelocity>>8)&0x00ff;// Motor3 angular velocity MSB

		uCom.SendBuf[53]  = (Adc.Mot3.Current>>0)&0x00ff; // Motor3 ADC current LSB, uint16
		uCom.SendBuf[54]  = (Adc.Mot3.Current>>8)&0x00ff; // Motor3 ADC current MSB
		uCom.SendBuf[55]  = (Adc.Mot3.Hall1>>0)&0x00ff;   // Motor3 Hall?1 LSB, uint16
		uCom.SendBuf[56]  = (Adc.Mot3.Hall1>>8)&0x00ff;   // Motor3 Hall?1 MSB
		uCom.SendBuf[57]  = (Adc.Mot3.Hall2>>0)&0x00ff;   // Motor3 Hall?2 LSB, uint16
		uCom.SendBuf[58]  = (Adc.Mot3.Hall2>>8)&0x00ff;   // Motor3 Hall?2 MSB

		uCom.SendBuf[59] = 0;                           // Reserved error status 1
		uCom.SendBuf[60] = 0;                           // Reserved error status 2
					
		for( i=0; i<BufLen; i++ )  spi_tx_buf[i] = uCom.SendBuf[i];
		
		SPI2_Slave_Start_DMA(); 
		
		for(  i=0; i<BufLen; i++ ) {
			if( ( uCom.RecBuf[i] == Head1 )&&( uCom.RecBuf[i+1] == Head2 ) ) {
				i++;
				i++;
				break;
			}	
		}		
		if( i >= 8 ) return;
		
		angle = uCom.RecBuf[i++];
		angle |= ( uCom.RecBuf[i++]<<8 );
		
		speed = uCom.RecBuf[i++];
		speed |= ( uCom.RecBuf[i++]<<8 );
		if( MotorEn )		Move1SetTask( angle, speed);
		
		height = uCom.RecBuf[i++];
		height |= ( uCom.RecBuf[i++]<<8 );
		
		speed = uCom.RecBuf[i++];
		speed |= ( uCom.RecBuf[i++]<<8 );
		
		if( MotorEn )	Move2SetTask( height, speed);
		
		angle = uCom.RecBuf[i++];
		angle |= ( uCom.RecBuf[i++]<<8 );
		
		speed = uCom.RecBuf[i++];
		speed |= ( uCom.RecBuf[i++]<<8 );
		
		if( MotorEn )	Move3SetTask( angle, speed);
		
		if( uCom.RecBuf[i++] == 0 ) {
			
			MotorEn = 1;
			
		}	else {
			
			if( MotorEn != 0 ) {
				MotorEn = 0;
				Mot1.CurVelocity = 0;
				Mot2.CurVelocity = 0;
				Mot3.CurVelocity = 0;
			}				
		}				
			
		if( uCom.RecBuf[i++] == 0 ){
			if( stateTmf8806 != TMF8806_STATE_MEASURE ) {
				measure();
			}	
		} else {
			if( stateTmf8806 != TMF8806_STATE_STOPPED ) {
					stop();
			}	
		}	
		
		MotorEn = uCom.RecBuf[i++];
		
	}	
}	
