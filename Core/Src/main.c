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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
volatile log_state NeedUpdate_INV_STATE=YES;
volatile INV_STATE MX3_INV_State=OFF;

volatile log_state NeedUpdate_RELAY_STATE=YES;
volatile log_state RELAY_OVER_TEMP=YES;
volatile log_state RELAY_LOW_AB=YES;
volatile log_state RELAY_VERYLOW_AB=YES;
volatile log_state RELAY_NO_ACIN=YES;
volatile log_state RELAY_NO_ACOUT=YES;

volatile uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

volatile uint16_t Global_IN_AC=0;
volatile uint32_t Global_IN_AVG=0;
volatile uint32_t Global_IN_SUM=0;
volatile uint16_t Global_IN_CNT=0;

volatile uint16_t Global_AB=0;
volatile uint32_t Global_AB_AVG=0;
volatile uint32_t Global_AB_SUM=0;
volatile uint16_t Global_AB_CNT=0;

volatile uint16_t Global_TEMP=0;
volatile uint32_t Global_TEMP_AVG=0;
volatile uint32_t Global_TEMP_SUM=0;
volatile uint16_t Global_TEMP_CNT=0;

volatile uint16_t Global_OUT_AC=0;
volatile uint32_t Global_OUT_AVG=0;
volatile uint32_t Global_OUT_SUM=0;
volatile uint16_t Global_OUT_CNT=0;

volatile log_state Blocked_by_PVD=YES;
volatile log_state Blocked_by_TEMP=YES;
volatile log_state Blocked_by_AB=YES;

volatile log_state RESTART_CMD_FLAG=NO;

uint32_t adc_conv_stamp=0;
//uint32_t adc_half_stamp=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

void Enable_SH_DEBUG(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (HAL_GPIO_ReadPin(RESTART_CMD_GPIO_Port,RESTART_CMD_Pin)==GPIO_PIN_RESET) {
		RESTART_CMD_FLAG=YES;
	} else {
		RESTART_CMD_FLAG=NO;
	}
}

void HAL_PWR_PVDCallback(void)
{
	if __HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) {
		Blocked_by_PVD=YES;
	} else {
		Blocked_by_PVD=NO;
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
	adc_conv_stamp=xTaskGetTickCount()-adc_conv_stamp;

	Global_TEMP=aADCxConvertedData[2]/ADC_OVERSAMPLING;
	Global_TEMP_CNT++;
	Global_TEMP_SUM=Global_TEMP_SUM+Global_TEMP;

	Global_OUT_AC=aADCxConvertedData[3]/ADC_OVERSAMPLING;
	Global_OUT_CNT++;
	Global_OUT_SUM=Global_OUT_SUM+Global_OUT_AC;

	Global_IN_AC=aADCxConvertedData[0]/ADC_OVERSAMPLING;
	Global_IN_CNT++;
	Global_IN_SUM=Global_IN_SUM+Global_IN_AC;

	Global_AB=aADCxConvertedData[1]/ADC_OVERSAMPLING;
	Global_AB_CNT++;
	Global_AB_SUM=Global_AB_SUM+Global_AB;

	if (xTaskGetTickCount()-adc_conv_stamp>=DELAY_ADC_SUM) {
			Global_TEMP_AVG=(uint32_t) (Global_TEMP_SUM/Global_TEMP_CNT);
			Global_TEMP_CNT=0;
			Global_TEMP_SUM=0;

			Global_OUT_AVG=(uint32_t) (Global_OUT_SUM/Global_OUT_CNT);
			Global_OUT_CNT=0;
			Global_OUT_SUM=0;

			Global_AB_AVG=(uint32_t) (Global_AB_SUM/Global_AB_CNT);
			Global_AB_CNT=0;
			Global_AB_SUM=0;

			Global_IN_AVG=(uint32_t) (Global_IN_SUM/Global_IN_CNT);
			Global_IN_CNT=0;
			Global_IN_SUM=0;

			adc_conv_stamp=xTaskGetTickCount();

			if (Global_AB_AVG<=BUZZER_OPORA) {
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			}
		}
}

/*void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1)
{
	Global_IN_AC=aADCxConvertedData[0]/ADC_OVERSAMPLING;
	Global_IN_CNT++;
	Global_IN_SUM=Global_IN_SUM+Global_IN_AC;

	Global_AB=aADCxConvertedData[1]/ADC_OVERSAMPLING;
	Global_AB_CNT++;
	Global_AB_SUM=Global_AB_SUM+Global_AB;

	if (xTaskGetTickCount()-adc_half_stamp>=DELAY_ADC_SUM) {
		Global_AB_AVG=(uint32_t) (Global_AB_SUM/Global_AB_CNT);
		Global_AB_CNT=0;
		Global_AB_SUM=0;

		Global_IN_AVG=(uint32_t) (Global_IN_SUM/Global_IN_CNT);
		Global_IN_CNT=0;
		Global_IN_SUM=0;


		adc_half_stamp=xTaskGetTickCount();

		//check LED
					    if (Global_AB_AVG<=BUZZER_OPORA) {
							HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
						} else {
							HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
						}
	}
}*/

void Enable_SH_DEBUG(void) {
	__HAL_RCC_DBGMCU_CLK_ENABLE();
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
    DBGMCU->APB2FZ = 0xFFFFFFFF;
    DBGMCU->APB1FZ = 0xFFFFFFFF;
    DBGMCU->CR |=DBGMCU_CR_DBG_STOP;
    __HAL_DBGMCU_FREEZE_IWDG();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

   #ifdef DEBUG_MODE
		Enable_SH_DEBUG();
	#endif

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
  MX_IWDG_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init(); 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
