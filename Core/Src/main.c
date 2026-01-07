/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
#include "delay.h"
#include <string.h>
#include "bsp_ADXL355.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t rx7_flag;
extern uint8_t rx7_size;          //receive length
extern uint8_t  rx7_buf[32];
//extern __attribute__((section (".RAM_D1"))) uint8_t  RX7_Data[32];
extern uint8_t  RX7_Data[32];
int32_t gx,gy,gz,wx,wy,wz;
float fgx,fgy,fgz;
int16_t te;
uint8_t zjs;
uint8_t GyroyOk;
int32_t ax,ay,az;
int32_t acce_data[3];

typedef struct {
    float b0, b1, b2, a1, a2;
    float b0_2, b1_2, b2_2, a1_2, a2_2;
    
    float x1, x2;       
    float y1, y2;       
    float x1_2, x2_2;   
    float y1_2, y2_2;   
} IIRFilter3rdOrder;

void IIRFilter3rdOrder_Init(IIRFilter3rdOrder *filter) {
    filter->b0 = 0.0097087378640777;
    filter->b1 = 0.029126213592233;
    filter->b2 = 0.029126213592233;
    filter->a1 = -1.561018075800718;
    filter->a2 = 0.641351538057563;
    
    filter->b0_2 = 1.0;
    filter->b1_2 = -1.7786312485217;
    filter->b2_2 = 0.800800564739853;
    filter->a1_2 = -1.8477590650226;
    filter->a2_2 = 0.8612094900000;
    
    filter->x1 = filter->x2 = 0.0f;
    filter->y1 = filter->y2 = 0.0f;
    filter->x1_2 = filter->x2_2 = 0.0f;
    filter->y1_2 = filter->y2_2 = 0.0f;
}

float IIRFilter3rdOrder_Process(IIRFilter3rdOrder *filter, float input) {
    float w = input - filter->a1 * filter->x1 - filter->a2 * filter->x2;
    float y = filter->b0 * w + filter->b1 * filter->x1 + filter->b2 * filter->x2;
    
    filter->x2 = filter->x1;
    filter->x1 = w;
    filter->y2 = filter->y1;
    filter->y1 = y;
    
    float w2 = y - filter->a1_2 * filter->x1_2 - filter->a2_2 * filter->x2_2;
    float y2 = filter->b0_2 * w2 + filter->b1_2 * filter->x1_2 + filter->b2_2 * filter->x2_2;
    
    filter->x2_2 = filter->x1_2;
    filter->x1_2 = w2;
    filter->y2_2 = filter->y1_2;
    filter->y1_2 = y2;
    
    return y2;
}

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
#define SEND_BUF_SIZE 64	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.
uint8_t  SendBuff[SEND_BUF_SIZE]; //�������ݻ�����
int32_t nIMU[9];

void uart2_send();

// ...existing code...
/* USER CODE BEGIN 4 */

/**
 * A: 3x3 矩阵，类型 float
 * b: 3x1 矩阵，表示为二维数组 float b[3][1]
 * res: 输出 3x1 矩阵，表示为二维数组 float res[3][1]
 */
float A[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};
float res[3][1];
float b[3][1] = {
    {1.0f},
    {2.0f},
    {3.0f}
};
void mat3x3_mul_3x1(const float A[3][3], const float b[3][1], float res[3][1])
{
    for (int i = 0; i < 3; ++i) {
        res[i][0] = A[i][0] * b[0][0]
                  + A[i][1] * b[1][0]
                  + A[i][2] * b[2][0];
    }
}

/* USER CODE END 4 */
// ...existing code...

void led_run()
{
	static u32 cnt=0;
	static u32 cnt1=0;
	static u32 cnt2=0;
	
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	
	if(cnt>=9999)
	{
		cnt=0;
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
	}
	else
	{
		cnt++;
	}
	
//	if(cnt2>=9)
//	{
//		cnt2=0;
//		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);
//	}
//	else
//	{
//		cnt2++;
//	}
	
	if(cnt1>=99)
	{
		cnt1=0;
//		ADXL355_Data_Scan(acce_data);
//		ax=acce_data[0];
//		ay=acce_data[1];
//		az=acce_data[2];
//		uart2_send();
	}
	else
	{
		cnt1++;
	}
}

void timer_run()
{

}

void uart2_send()
{
	static uint8_t cnt=0;
	
	SendBuff[0]=0xAA;
	SendBuff[1]=0xAA;
	SendBuff[2]=0x00;
	SendBuff[3]=0xFF;
	SendBuff[30]=0;
	SendBuff[31]=0xBB;
	cnt++;
	SendBuff[2]=cnt;
	
  b[0][0]=(float)gx;
  b[1][0]=(float)gy;  
  b[2][0]=(float)gz;
  mat3x3_mul_3x1(A, b, res);
  nIMU[3]=(int32_t)(res[2][0]);
  nIMU[4]=(int32_t)(res[1][0]);
  nIMU[5]=(int32_t)(res[0][0]);

//	nIMU[3]=gz;
//	nIMU[4]=gy;
//	nIMU[5]=gx;
	nIMU[0]=ax;
	nIMU[1]=ay;
	nIMU[2]=az;
	memcpy(SendBuff+4,nIMU,24);
	memcpy(SendBuff+28,&te,2);
	
	//SendBuff[26]=zjs;
	for(int i=2;i<30;i++)
  {
	   SendBuff[30] = SendBuff[30]+SendBuff[i];
	}
	HAL_UART_Transmit_DMA(&huart3,(uint8_t *)SendBuff,32);		
//	SendBuff[0]=0xA5;
//	SendBuff[1]=0;
//	SendBuff[39]=0;
//	cnt++;
//	SendBuff[1]=cnt;
//	
//	nIMU[0]=wz;
//	nIMU[1]=wy;
//	nIMU[2]=wx;
//	nIMU[3]=ax;
//	nIMU[4]=ay;
//	nIMU[5]=az;
//	nIMU[6]=gz;
//	nIMU[7]=gy;
//	nIMU[8]=gx;
//	memcpy(SendBuff+2,nIMU,36);
//	
//	SendBuff[38]=zjs;
//	for(int i=1;i<39;i++)
//  {
//	   SendBuff[39] = SendBuff[39]+SendBuff[i];
//	}
	HAL_UART_Transmit_DMA(&huart2,(uint8_t *)SendBuff,28);
//	HAL_UART_Transmit_DMA(&huart3,(uint8_t *)SendBuff,28);
//	HAL_UART_Transmit_DMA(&huart4,(uint8_t *)SendBuff,28);
}

//void gyro_rec(void)
//{
//	uint8_t sum=0;
//	uint8_t i = 0;
//	uint32_t gdata;
//	uint16_t tdata;
//	static int32_t gx_o=0,gy_o=0,gz_o=0; 
//	static uint8_t zjs_o=0;

//	if(rx7_flag==1)
//	{
//		rx7_flag = 0;

//		if(RX7_Data[0]==0xA5)		
//		{
//			sum =  0;
//			
//			for(i = 1;i < 16; i++)
//			{
//				sum += RX7_Data[i];
//			}
//			if(((sum & 0xff) == RX7_Data[16]))
//			{
//				GyroyOk = 1;
//				gdata  = (uint32_t)(RX7_Data[4]);
//				gdata |= (uint32_t)(RX7_Data[3])<<8;
//				gdata |= (uint32_t)(RX7_Data[2])<<16;
//				gdata |= (uint32_t)(RX7_Data[1])<<24;
//				memcpy(&gx,&gdata,4);
//				gdata  = (uint32_t)(RX7_Data[8]);
//				gdata |= (uint32_t)(RX7_Data[7])<<8;
//				gdata |= (uint32_t)(RX7_Data[6])<<16;
//				gdata |= (uint32_t)(RX7_Data[5])<<24;
//				memcpy(&gy,&gdata,4);
//				gdata  = (uint32_t)(RX7_Data[12]);
//				gdata |= (uint32_t)(RX7_Data[11])<<8;
//				gdata |= (uint32_t)(RX7_Data[10])<<16;
//				gdata |= (uint32_t)( RX7_Data[9])<<24;
//				memcpy(&gz,&gdata,4);
//				
//				tdata  = (uint32_t)(RX7_Data[14]);
//				tdata |= (uint32_t)(RX7_Data[13])<<8;
//				memcpy(&te,&tdata,2);
//				
//				zjs = RX7_Data[15];
//				
////				wx=gx-gx_o;
////				wy=gy-gy_o;
////				wz=gz-gz_o;
////				gx_o=gx;
////				gy_o=gy;
////				gz_o=gz;
//		  }
//	  }
//	}
//}

void gyro_rec(void)
{
	uint8_t sum=0;
	uint8_t i = 0;
	uint32_t gdata;
	uint16_t tdata;

	if(rx7_flag==1)
	{
		rx7_flag = 0;

		if(RX7_Data[0]==0x80)		
		{
			sum =  0;
			
			for(i = 1;i < 16; i++)
			{
				sum += RX7_Data[i];
			}
			if(((sum & 0x7f) == RX7_Data[16]))
			{
				GyroyOk = 1;
				gdata  = (uint32_t)(RX7_Data[4] & 0x7f)<<4;
				gdata |= (uint32_t)(RX7_Data[3] & 0x7f)<<11;
				gdata |= (uint32_t)(RX7_Data[2] & 0x7f)<<18;
				gdata |= (uint32_t)(RX7_Data[1] & 0x7f)<<25;
				memcpy(&gx,&gdata,4);
				gx=gx>>4;
				gdata  = (uint32_t)(RX7_Data[8] & 0x7f)<<4;
				gdata |= (uint32_t)(RX7_Data[7] & 0x7f)<<11;
				gdata |= (uint32_t)(RX7_Data[6] & 0x7f)<<18;
				gdata |= (uint32_t)(RX7_Data[5] & 0x7f)<<25;
				memcpy(&gy,&gdata,4);
				gy=gy>>4;
				gdata  = (uint32_t)(RX7_Data[12] & 0x7f)<<4;
				gdata |= (uint32_t)(RX7_Data[11] & 0x7f)<<11;
				gdata |= (uint32_t)(RX7_Data[10] & 0x7f)<<18;
				gdata |= (uint32_t)(RX7_Data[ 9] & 0x7f)<<25;
				memcpy(&gz,&gdata,4);
				gz=gz>>4;
				
				tdata  = (uint32_t)(RX7_Data[14] & 0x7f)<<2;
				tdata |= (uint32_t)(RX7_Data[13] & 0x7f)<<9;
				memcpy(&te,&tdata,2);
				te=te>>2;							
		  }
	  }
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART7_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	ADXL355_Init();
	delay_ms(30);
	ADXL355_Start_Sensor();
	
	IIRFilter3rdOrder filter;
  IIRFilter3rdOrder_Init(&filter);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		gyro_rec();
		if(GyroyOk)
		{
			GyroyOk=0;
			ADXL355_Data_Scan(acce_data);
			ax=acce_data[0];
			ay=acce_data[1];
			az=acce_data[2];
			//fgx = IIRFilter3rdOrder_Process(&filter, (float)gx);
			//fgy = IIRFilter3rdOrder_Process(&filter, (float)gy);
			//fgz = IIRFilter3rdOrder_Process(&filter, (float)gz);
			uart2_send();
			//uart2_send();
		}	
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
