/*
*********************************************************************************************************
*
*	文件名称 : bsp_ADXL355.c
*	版    本 : V1.0
*	说    明 : ACC-ADXL355 模块驱动文件
*	作    者 : Mr. Albert Chen		  
*
*********************************************************************************************************
*/
#include "bsp_ADXL355.h"
#include "spi.h"

uint8_t SPI_ReadWriteByte(uint8_t TData)   //通过SPI读写一个字节
{
	uint8_t TxData,RxData;
	
	TxData = TData;
  if(HAL_SPI_TransmitReceive(&hspi1,&TxData,&RxData,1,0xff)!=HAL_OK)
     RxData=0xFF; 
  return RxData;
}

/*
  函数功能：通过SPI 向 ADXL355 WRITE一个字节的数据
  adress：地址 
  data  ：数据
  写一个字节的数据    
*/
void SPI_ADXL355_WRITE(uint8_t adress, uint8_t data)
{

	uint8_t writeadress;                                          
		
  CS_ADXL355_LOW;                                             //CS = LOW  片选信号拉低
	//HAL_Delay(1);					                                    //CS to SCLK set up time 片选到时钟信号的建立时间tcss
	writeadress = adress<<1;                                            //寄存器的地址7位 左移1位 右边补0表示写数据  即 xxxxxxx0共8位
	SPI_ReadWriteByte(writeadress);		                                //write adress写入地址
	SPI_ReadWriteByte(data);                                            //data to be written写入数据

//	HAL_Delay(1);							                            //CS to SCLK set up time
	CS_ADXL355_HIGH;                                    		//CS = High	 片选信号拉高
//	HAL_Delay(1);                                                       //Wait Time
}

/*
  函数功能：通过SPI 从 ADXL355读取一个字节的数据
  adress：地址 
  读取一个字节的数据    
*/
uint8_t SPI_ADXL355_READ1BYTE(uint8_t adress)
{

	uint32_t result;
	uint8_t readadress;
	
  CS_ADXL355_LOW;                                          //CS = LOW
//	HAL_Delay(1);						                               //CS to SCLK set up time
	readadress = adress<<1;                                            //寄存器的地址7位 左移1位 
	readadress |= 0x01;                                                //寄存器的地址7位 左移1位 右边与1或表示读数据  即 xxxxxxx1共8位
	result=SPI_ReadWriteByte(readadress);                              //read adress读地址
    result=SPI_ReadWriteByte(0xff);                                    //Read 8bit data (Send Dummy data 0xff)
	
//  HAL_Delay(1);			                                           //CS to SCLK set up time
	CS_ADXL355_HIGH;                                    		     //CS = High	 片选信号拉高	
//	HAL_Delay(1);				                                       //Wait Time

	return result;
}

/*
  函数功能：通过SPI 从 ADXL355读取3个字节的数据
  adress：地址 
  读取3个字节的数据    
*/
 uint32_t SPI_ADXL355_READ3BYTE(uint8_t adress)
{

	uint32_t result;
	uint8_t readadress;

  CS_ADXL355_LOW;                                          //CS = LOW
//	HAL_Delay(1);					                                  //CS to SCLK set up time
	readadress = adress<<1;                                           //寄存器的地址7位 左移1位 
	readadress |= 0x01;                                               //寄存器的地址7位 左移1位 右边与1或表示读数据  即 xxxxxxx1共8位
	result=SPI_ReadWriteByte(readadress);                             //read adress
    result=SPI_ReadWriteByte(0xff);                                   //Read 8bit data读取一个字节的数据 8位 00000000 00000000 00000000 xxxxxxxx
	result=result<<8|SPI_ReadWriteByte(0xff);                         //Read 16bit data读取2个字节的数据 16位00000000 00000000 xxxxxxxx xxxxxxxx
	result=result<<8|SPI_ReadWriteByte(0xff);                         //Read 24bit data读取3个字节的数据 24位00000000 xxxxxxxx xxxxxxxx xxxxxxxx

	//HAL_Delay(1);			                                          //CS to SCLK set up time
	CS_ADXL355_HIGH;                                    		     //CS = High	 片选信号拉高	
	//HAL_Delay(1);			                                          //Wait Time

	return result;
}

/*
  函数功能: ADXL355 初始化
  配置相关寄存器  XL355_RANGE、XL355_SYNC等 
  判断传感器是否连接正常
*/
void ADXL355_Init(void)
{
  
	SPI_ADXL355_WRITE(XL355_RANGE,0x81);          /* Set sensor range within RANGE register */ //设置为 1000 0001 高速模式  ±2g
	delay_ms(20);
	SPI_ADXL355_WRITE(XL355_SYNC,0x03);     
	delay_ms(20);
//  uint32_t a=SPI_ADXL355_READ1BYTE(XL355_RANGE);
//	a=SPI_ADXL355_READ1BYTE(XL355_SYNC);
	
//    uint32_t volatile ui32test  = SPI_ADXL355_READ1BYTE(XL355_DEVID_AD);              /* Read the ID register */ 
//    uint32_t volatile ui32test2 = SPI_ADXL355_READ1BYTE(XL355_DEVID_MST);             /* Read the ID register */
//    uint32_t volatile ui32test3 = SPI_ADXL355_READ1BYTE(XL355_PARTID);                /* Read the ID register */
//    uint32_t volatile ui32test4 = SPI_ADXL355_READ1BYTE(XL355_REVID);                 /* Read the ID register */

//	/* 验证ADXL355设备是否连接正常*/
//		if(SPI_ADXL355_READ1BYTE(XL355_PARTID)==0xED)	

//	  {
//		  HAL_Delay(5);
//		  printf("\r\n设备正常!\r\n");
//		  printf("XL355_DEVID_AD:%x\r XL355_DEVID_MST:%x\r \n",ui32test,ui32test2);
//		  printf("XL355_PARTID:%x\r XL355_REVID:%x\r \n",ui32test3,ui32test4);
//	  }
//	  else
//	  {
//		  HAL_Delay(5);
//		  printf("\r\n设备不正常，请检查！\r\n");
//	  }	 
//	  
	  
}

/*
  函数功能: ADXL355 传感器开始工作
*/
void ADXL355_Start_Sensor(void)
{
   uint8_t ui8temp;
   ui8temp = (uint8_t)SPI_ADXL355_READ1BYTE(XL355_POWER_CTL);       /* Read POWER_CTL register, before modifying it */
   ui8temp = ui8temp & 0xFE;                                        /* Set measurement bit in POWER_CTL register */
   SPI_ADXL355_WRITE(XL355_POWER_CTL, ui8temp);                     /* Write the new value to POWER_CTL register */
 
}

/*
  函数功能： ADXL355加速度数据转换
*/
int32_t ADXL355_Acceleration_Data_Conversion (uint32_t ui32SensorData)
{
   int32_t volatile i32Conversion = 0;

	//加速度数据为20位 读取的3个字节（24位）加速度数据其中最低4位无效[3:0]     00000000 xxxxxxxx xxxxxxxx xxxxxxxx
   ui32SensorData = (ui32SensorData  >> 4);                  //右移4位
   ui32SensorData = (ui32SensorData & 0x000FFFFF);           //得到32位真实的数据 00000000 0000xxxx xxxxxxxx xxxxxxxx
   if((ui32SensorData & 0x00080000)  == 0x00080000)          //       如果数据是  00000000 00001xxx xxxxxxxx xxxxxxxx
	 {
         i32Conversion = (ui32SensorData | 0xFFF00000);      //      则数据转换成 11111111 11111xxx xxxxxxxx xxxxxxxx
   }
   else
	 {
          i32Conversion = ui32SensorData;                   //      否则数据转换成 00000000 0000xxxx xxxxxxxx xxxxxxxx
   }
   return i32Conversion;
}

/*
  函数功能： ADXL355数据扫描
  Reads the accelerometer data.
  发送ADXL355采集的数据   
*/
void ADXL355_Data_Scan(int32_t *AccData_original_32)
{
	  uint32_t SensorX,SensorY,SensorZ;

	  SensorX=SPI_ADXL355_READ3BYTE(XL355_XDATA3);                    //XL355_XDATA3为X轴3数据寄存器的首地址  ui32SensorX为3个字节的数据  00000000 xxxxxxxx xxxxxxxx xxxxxxxx
	  SensorY=SPI_ADXL355_READ3BYTE(XL355_YDATA3);                    //XL355_YDATA3为Y轴3数据寄存器的首地址
	  SensorZ=SPI_ADXL355_READ3BYTE(XL355_ZDATA3);                    //XL355_ZDATA3为Z轴3数据寄存器的首地址
	   // t2=HAL_GetTick() - t1;//
      AccData_original_32[0] = ADXL355_Acceleration_Data_Conversion(SensorX);     //将AD采集到的数据进行转换
      AccData_original_32[1] = ADXL355_Acceleration_Data_Conversion(SensorY);
      AccData_original_32[2] = ADXL355_Acceleration_Data_Conversion(SensorZ);		

} 

