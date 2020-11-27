/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "i2c_lib.h"
#include "usbd_cdc_if.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PCA9532_ADDRESS 0x60
#define ADS7830_ADDRESS 0x48
#define CH0 0x8C // C-internal reference(2.5V) ; 4- external reference
#define CH1 0xCC
#define CH2 0x9C
#define CH3 0xDC
#define CH4 0xAC
#define CH5 0xEC
#define CH6 0xBC
#define CH7 0xFC
#define OK 0x01
#define ERROR 0x02

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t adc_command[8] = {CH0,CH1,CH2,CH3,CH4,CH5,CH6,CH7};
uint8_t adc_receive[8];
float voltage_receive[8];
const float step_adc_voltage=2.49/256;
char char_com_receive=0x00;
const float deviation_percent = 5.0;

void START_ADC_MEASURING()
{
	for(int i=0;i<8;i++)
		{
			I2C1_Write_Byte(ADS7830_ADDRESS,adc_command[i]);
			adc_receive[i]=I2C1_Read_Byte(ADS7830_ADDRESS,adc_receive[i]);

			if(i==0||i==1||i==7)
			{
				if(i==7)
				voltage_receive[i]=adc_receive[i]*step_adc_voltage*3;
				else
				voltage_receive[i]=adc_receive[i]*step_adc_voltage*2;
			}
			else
				voltage_receive[i]=adc_receive[i]*step_adc_voltage;
		}

}

uint8_t CH_STATUS[8];

//STATUS OK when voltage in allowable range (should be 5% from PMIC voltage(deviation_percent), STATUS ERROR when voltage out of range
void START_VOLTAGE_COMPARING()
{
	
		for(int i=0;i<8;i++)
	{
		CH_STATUS[i]=ERROR;
	}

		//	CH0 - 2.5VFPGA (PMIC 2.5V)
		//	CH1 - 3v3_SFP (PMIC 3.15V)
		//	CH2 - 1v1_TRA (PMIC 1.1V)
		// 	CH3 - 1v1_FPGA (PMIC 1.1V)
		//	CH4 - 1v35_DDR (PMIC 1.35V)
		//	CH5 - 1v275_IMX (PMIC 1.275V)
		//	CH6 - VREF_DDR (PMIC 0.675V)
		//	CH7 - 5V (PMIC 5V)

	// standart values {2.5,3.15,1.1,1.1,1.35,1.275,0.675,5};
	
	const float CH_VOLTAGE_PMIC[8]={2.5,3.15,1.1,1.1,1.35,1.275,0.675,5};

	float min_CH_limit[8];
	float max_CH_limit[8];
	

	for(int i=0;i<8;i++)
	{
		min_CH_limit[i]=CH_VOLTAGE_PMIC[i]-(CH_VOLTAGE_PMIC[i]*deviation_percent/100);
		max_CH_limit[i]=CH_VOLTAGE_PMIC[i]+(CH_VOLTAGE_PMIC[i]*deviation_percent/100);
		if(voltage_receive[i]>min_CH_limit[i]&&voltage_receive[i]<max_CH_limit[i])
		{
		CH_STATUS[i]=OK;	
		}
		else
		CH_STATUS[i]=ERROR;
	}
}


void LED_CLEAN_ALL()
{
	uint8_t buffer_led[3]={0x16,0x00,0x00};
	I2C1_Write_Array(PCA9532_ADDRESS,buffer_led,3);
}

void LED_WRITE()
{
	uint8_t LS0[4] = {0x01,0x04,0x10,0x40};
	uint8_t LS1[4] = {0x01,0x04,0x10,0x40};
	uint8_t LS0_CFG_TO_SEND=0x00;
	uint8_t LS1_CFG_TO_SEND=0x00;
	
	for(int i=0;i<4;i++)
	{
		if(CH_STATUS[i]==ERROR)
		{
			LS0[i]=0x00;//LED off
		}
		LS0_CFG_TO_SEND|=LS0[i];
	}
	for(int i=4;i<8;i++)
	{
		if(CH_STATUS[i]==ERROR)
		{
			LS1[i-4]=0x00;//LED 
		}
		LS1_CFG_TO_SEND|=LS1[i-4];
	}

	uint8_t buffer_led[3]={0x16,LS0_CFG_TO_SEND,LS1_CFG_TO_SEND};
	I2C1_Write_Array(PCA9532_ADDRESS,buffer_led,3);
	
}
/*
void LED_1_ON()
{
		uint8_t buffer_led[2]={0x06,0x01};
		I2C1_Write_Array(PCA9532_ADDRESS,buffer_led,2);
}
*/
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END 0 */
		uint8_t strCH0[] = "\r\nCH0 2.5VFPGA (PMIC 2.5V) - ";
		uint8_t strCH1[] = "CH1 3v3_SFP (PMIC 3.15V) - ";
		uint8_t strCH2[] = "CH2 1v1_TRA (PMIC 1.1V) - ";
		uint8_t strCH3[] = "CH3 1v1_FPGA (PMIC 1.1V) - ";
		uint8_t strCH4[] = "CH4 1v35_DDR (PMIC 1.35V) - ";
		uint8_t strCH5[] = "CH5 1v275_IMX (PMIC 1.275V) - ";
		uint8_t strCH6[] = "CH6 VREF_DDR (PMIC 0.675V) - ";
		uint8_t strCH7[] = "CH7 5V (PMIC 5V)- ";
		uint8_t strCH_ERROR[] = "[1;97;41mERROR! Current voltage =[0m ";//change background color to red and reset (it doesn't work in minicom. Use PUTTY)
		uint8_t strCH_OK[] = "[1;97;42mOK! Current voltage =[0m ";//change background color to green and reset (it doesn't work in minicom. Use PUTTY)


		
void TRANSMIT_CH_STATUS()
{
		char bufx[100];
//	
						sprintf(bufx, "\n\nIf voltage deviation is greater than %.2f %% - channel ERROR\n",deviation_percent);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);

	//------------------------------------------------------------------------
						if(CH_STATUS[0]==OK)
						{
						CDC_Transmit_FS(strCH0,sizeof(strCH0));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_OK,sizeof(strCH_OK));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[0]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}
						if(CH_STATUS[0]==ERROR)
						{
						CDC_Transmit_FS(strCH0,sizeof(strCH0));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_ERROR,sizeof(strCH_ERROR));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[0]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}
				//---------------------------------------------------------------
						if(CH_STATUS[1]==OK)
						{
						CDC_Transmit_FS(strCH1,sizeof(strCH1));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_OK,sizeof(strCH_OK));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[1]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}
						if(CH_STATUS[1]==ERROR)
						{
						CDC_Transmit_FS(strCH1,sizeof(strCH1));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_ERROR,sizeof(strCH_ERROR));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[1]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}		
				//---------------------------------------------------------------

						if(CH_STATUS[2]==OK)
						{
						CDC_Transmit_FS(strCH2,sizeof(strCH2));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_OK,sizeof(strCH_OK));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[2]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}
						if(CH_STATUS[2]==ERROR)
						{
						CDC_Transmit_FS(strCH2,sizeof(strCH2));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_ERROR,sizeof(strCH_ERROR));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[2]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}		
				//---------------------------------------------------------------

						if(CH_STATUS[3]==OK)
						{
						CDC_Transmit_FS(strCH3,sizeof(strCH3));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_OK,sizeof(strCH_OK));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[3]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);	
						}
						if(CH_STATUS[3]==ERROR)
						{
						CDC_Transmit_FS(strCH3,sizeof(strCH3));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_ERROR,sizeof(strCH_ERROR));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[3]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}		
				//---------------------------------------------------------------

						if(CH_STATUS[4]==OK)
						{
						CDC_Transmit_FS(strCH4,sizeof(strCH4));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_OK,sizeof(strCH_OK));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[4]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}
						if(CH_STATUS[4]==ERROR)
						{
						CDC_Transmit_FS(strCH4,sizeof(strCH4));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_ERROR,sizeof(strCH_ERROR));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[4]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}	
						
				//---------------------------------------------------------------		
						if(CH_STATUS[5]==OK)
						{
						CDC_Transmit_FS(strCH5,sizeof(strCH5));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_OK,sizeof(strCH_OK));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[5]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}
						if(CH_STATUS[5]==ERROR)
						{
						CDC_Transmit_FS(strCH5,sizeof(strCH5));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_ERROR,sizeof(strCH_ERROR));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[5]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}	
						
				//---------------------------------------------------------------	
						if(CH_STATUS[6]==OK)
						{
						CDC_Transmit_FS(strCH6,sizeof(strCH6));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_OK,sizeof(strCH_OK));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[6]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}
						if(CH_STATUS[6]==ERROR)
						{
						CDC_Transmit_FS(strCH6,sizeof(strCH6));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_ERROR,sizeof(strCH_ERROR));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[6]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}	
						
				//---------------------------------------------------------------	
						if(CH_STATUS[7]==OK)
						{
						CDC_Transmit_FS(strCH7,sizeof(strCH7));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_OK,sizeof(strCH_OK));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[7]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}
						if(CH_STATUS[7]==ERROR)
						{
						CDC_Transmit_FS(strCH7,sizeof(strCH7));
						HAL_Delay(5);
						CDC_Transmit_FS(strCH_ERROR,sizeof(strCH_ERROR));
						HAL_Delay(5);
						sprintf(bufx, "%.3f\r\n\n",voltage_receive[7]);
						CDC_Transmit_FS((unsigned char*)bufx,strlen(bufx));
						HAL_Delay(5);
						}	
						
				//---------------------------------------------------------------							
						char_com_receive=0x00;
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
//MX_TIM6_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */



  /* USER CODE END 2 */

 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		HAL_Delay(100);	//delay for excepting transition process(sometimes external ADC starts before STM32) 
		START_ADC_MEASURING();
		START_VOLTAGE_COMPARING();
		LED_CLEAN_ALL();
		LED_WRITE();
		if(char_com_receive)
		{	
			uint8_t pressf[] = "Press F to start the test \n\r";
			CDC_Transmit_FS(pressf,sizeof(pressf));
			HAL_Delay(10);
			
			if(char_com_receive=='F'||char_com_receive=='f')
			{	
				TRANSMIT_CH_STATUS();
			}
			char_com_receive=0x00;
		}
    /* USER CODE BEGIN 3 */
  }
}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time 
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
