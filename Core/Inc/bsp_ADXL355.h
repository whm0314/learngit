#ifndef _BSP_ADXL355_H
#define _BSP_ADXL355_H

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"
#include "delay.h"

/***************************************************/
/* SPI端口软件操作宏定义 */

#define	CS_ADXL355_HIGH	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
#define	CS_ADXL355_LOW	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

/***************************************************/
/* ADXL355寄存器地址定义 */
#define XL355_DEVID_AD			    0x00
#define XL355_DEVID_MST			    0x01
#define XL355_PARTID				    0x02
#define XL355_REVID					    0x03
#define XL355_STATUS				    0x04
#define XL355_FIFO_ENTRIES	    0x05
#define XL355_TEMP2					0x06
#define XL355_TEMP1					0x07
#define XL355_XDATA3				0x08
#define XL355_XDATA2				0x09
#define XL355_XDATA1				0x0A
#define XL355_YDATA3				0x0B
#define XL355_YDATA2				0x0C
#define XL355_YDATA1				0x0D
#define XL355_ZDATA3				0x0E
#define XL355_ZDATA2				0x0F
#define XL355_ZDATA1				0x10
#define XL355_FIFO_DATA			    0x11
#define XL355_OFFSET_X_H		    0x1E
#define XL355_OFFSET_X_L		    0x1F
#define XL355_OFFSET_Y_H		    0x20
#define XL355_OFFSET_Y_L		    0x21
#define XL355_OFFSET_Z_H		    0x22
#define XL355_OFFSET_Z_L		    0x23
#define XL355_ACT_EN				    0x24
#define XL355_ACT_THRESH_H	        0x25
#define XL355_ACT_THRESH_L	        0x26
#define XL355_ACT_COUNT			        0x27
#define XL355_FILTER				        0x28
#define XL355_FIFO_SAMPLES	        0x29
#define XL355_INT_MAP				0x2A
#define XL355_SYNC					0x2B
#define XL355_RANGE					0x2C
#define XL355_POWER_CTL			    0x2D
#define XL355_SELF_TEST			    0x2E
#define XL355_RESET					    0x2F

/* Accelerometer write command */
#define ADXL355_WRITE         0x0

/* Accelerometer read command */
#define ADXL355_READ          0x1

/***************************************************/
/* 函数声明 */
void ADXL355_Init(void);
void ADXL355_Start_Sensor(void);
int32_t ADXL355_Acceleration_Data_Conversion (uint32_t ui32SensorData);
void ADXL355_Data_Scan(int32_t *AccData_original_32);


#endif
