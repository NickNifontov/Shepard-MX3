/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "iwdg.h"
#include "adc.h"
#include "math.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId InvTaskHandle;
osThreadId IN_ACTaskHandle;
osThreadId OUT_ACTaskHandle;
osThreadId TempTaskHandle;
osThreadId RelayTaskHandle;
osThreadId RestartCMDTaskHandle;
//osThreadId LEDTaskHandle;
osThreadId ABTaskHandle;

uint16_t IN_OK=0;
uint16_t IN_BAD=0;

uint16_t OUT_OK=0;
uint16_t OUT_BAD=0;

uint16_t Start_Attept_CNT=0;

INV_STATE New_INV_STATE=OFF;

#define TOTAL_CMP 10000
#define TOTAL_LEVEL 100

// resistance at 25 degrees C
#define THERMISTORNOMINAL 1000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 4700

double average=0;
double steinhart=0;


uint32_t temp_stamp=0;
uint32_t ab_stamp=0;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Inv_OFF(void);
void Inv_ON(void);
uint8_t CheckStamp(uint32_t time_stamp, uint8_t time_base);

uint8_t CheckStamp(uint32_t time_stamp, uint8_t time_base) {
	if (xTaskGetTickCount()-time_stamp>time_base*1000) {
		return 1;
	} else {
		return 0;
	}
}

void Inv_OFF(void) {
    HAL_GPIO_WritePin(GPIOA, BLOCK_PORT_Pin, GPIO_PIN_RESET); // set 8V block
	HAL_GPIO_WritePin(BLOCK_POWER_GPIO_Port, BLOCK_POWER_Pin, GPIO_PIN_RESET); //off 15v
}

void Inv_ON(void) {
	#ifndef NOT_USE_8V_BLOCK
		HAL_GPIO_WritePin(GPIOA, BLOCK_PORT_Pin, GPIO_PIN_SET); // reset 8v Block
	#endif
	HAL_GPIO_WritePin(BLOCK_POWER_GPIO_Port, BLOCK_POWER_Pin, GPIO_PIN_SET); //on 15v
}
/* USER CODE END FunctionPrototypes */

void StartInvTask(void const * argument);
void StartIN_ACTask(void const * argument);
void StartOUT_ACTask(void const * argument);
void StartTempTask(void const * argument);
void StartRelayTask(void const * argument);
void StartRestartCMDTask(void const * argument);
//void StartLEDTask(void const * argument);
void StartABTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of InvTask */
  osThreadDef(InvTask, StartInvTask, osPriorityNormal, 0, 128);
  InvTaskHandle = osThreadCreate(osThread(InvTask), NULL);

  /* definition and creation of IN_ACTask */
  osThreadDef(IN_ACTask, StartIN_ACTask, osPriorityNormal, 0, 128);
  IN_ACTaskHandle = osThreadCreate(osThread(IN_ACTask), NULL);

  /* definition and creation of OUT_ACTask */
  osThreadDef(OUT_ACTask, StartOUT_ACTask, osPriorityNormal, 0, 128);
  OUT_ACTaskHandle = osThreadCreate(osThread(OUT_ACTask), NULL);

  /* definition and creation of TempTask */
  osThreadDef(TempTask, StartTempTask, osPriorityNormal, 0, 128);
  TempTaskHandle = osThreadCreate(osThread(TempTask), NULL);

  /* definition and creation of RelayTask */
  osThreadDef(RelayTask, StartRelayTask, osPriorityNormal, 0, 128);
  RelayTaskHandle = osThreadCreate(osThread(RelayTask), NULL);

  /* definition and creation of RestartCMDTask */
  osThreadDef(RestartCMDTask, StartRestartCMDTask, osPriorityNormal, 0, 128);
  RestartCMDTaskHandle = osThreadCreate(osThread(RestartCMDTask), NULL);

  /* definition and creation of LEDTask */
  //osThreadDef(LEDTask, StartLEDTask, osPriorityNormal, 0, 128);
  //LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of ABTask */
  osThreadDef(ABTask, StartABTask, osPriorityIdle, 0, 128);
  ABTaskHandle = osThreadCreate(osThread(ABTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartInvTask */
/**
  * @brief  Function implementing the InvTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartInvTask */
void StartInvTask(void const * argument)
{
  /* USER CODE BEGIN StartInvTask */
  /* Infinite loop */

  // default state - OFF INV
  Inv_OFF();

  /* ###  - Start conversion in DMA mode ################################# */
  HAL_ADC_Start_DMA(&hadc,(uint32_t *)aADCxConvertedData,ADC_CONVERTED_DATA_BUFFER_SIZE);

  HAL_PWR_PVDCallback();

  HAL_GPIO_EXTI_Callback(RESTART_CMD_Pin);

  //MX3_INV_State=ON;
  MX3_INV_State=OFF;
  NeedUpdate_INV_STATE=YES;

  for(;;)
  {
	if ( (Blocked_by_PVD==NO) && (Blocked_by_AB==NO)  && (Blocked_by_TEMP==NO)) {
		New_INV_STATE=ON;
	} else {
		New_INV_STATE=OFF;
	}

	if (New_INV_STATE!=MX3_INV_State) {
		NeedUpdate_INV_STATE=YES;
	}

	if (NeedUpdate_INV_STATE==YES) {
		NeedUpdate_INV_STATE=NO;
		if (New_INV_STATE==OFF) {
			Inv_OFF();
			MX3_INV_State=OFF;
		} else {
			Inv_ON();
			MX3_INV_State=ON;
		}
	}

	HAL_IWDG_Refresh(&hiwdg);

	osDelay(1);
  }
  /* USER CODE END StartInvTask */
}

/* USER CODE BEGIN Header_StartIN_ACTask */
/**
* @brief Function implementing the IN_ACTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIN_ACTask */
void StartIN_ACTask(void const * argument)
{
  /* USER CODE BEGIN StartIN_ACTask */
  /* Infinite loop */
  uint32_t  temp_IN=Global_IN_AVG;

  for(;;)
  {

	temp_IN=Global_IN_AVG;
	if (temp_IN>=4000) {
		IN_BAD++;
	} else {
		IN_OK++;
	}

	if (IN_BAD+IN_OK>=TOTAL_CMP) {

		if (IN_OK>=TOTAL_LEVEL) {
			if (RELAY_NO_ACIN==YES) {
				RELAY_NO_ACIN=NO;
				NeedUpdate_RELAY_STATE=YES;
			}
		} else {
			if (RELAY_NO_ACIN==NO) {
				RELAY_NO_ACIN=YES;
				NeedUpdate_RELAY_STATE=YES;
			}
		}

		IN_BAD=0;
		IN_OK=0;
	}

	osDelay(1);
  }
  /* USER CODE END StartIN_ACTask */
}

/* USER CODE BEGIN Header_StartOUT_ACTask */
/**
* @brief Function implementing the OUT_ACTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOUT_ACTask */
void StartOUT_ACTask(void const * argument)
{
  /* USER CODE BEGIN StartOUT_ACTask */
  /* Infinite loop */
	  uint32_t  temp_OUT=Global_OUT_AVG;

	  for(;;)
	  {

		temp_OUT=Global_OUT_AVG;
		if (temp_OUT>=4000) {
			OUT_BAD++;
		} else {
			OUT_OK++;
		}

		if (OUT_BAD+OUT_OK>=TOTAL_CMP) {
			if (OUT_OK>=TOTAL_LEVEL) {
				// YES AC
				Start_Attept_CNT=0;
				if (RELAY_NO_ACOUT==YES) {
					RELAY_NO_ACOUT=NO;
					NeedUpdate_RELAY_STATE=YES;
				}
			} else {
				// NO AC
				if ((Start_Attept_CNT>=5) && (RELAY_NO_ACOUT==NO)) {
					RELAY_NO_ACOUT=YES;
					NeedUpdate_RELAY_STATE=YES;
				} else {
					if ( (Blocked_by_PVD==NO) && (Blocked_by_AB==NO)  && (Blocked_by_TEMP==NO) && (RELAY_NO_ACOUT==NO)) {
									Start_Attept_CNT++;
									//
									Inv_OFF();
									//MX3_INV_State=OFF;
									osDelay(1000);
									Inv_ON();
									//MX3_INV_State=ON;
					}
					osDelay(60000);
				}
			}

			OUT_BAD=0;
			OUT_OK=0;
		}

		osDelay(1);
	  }
  /* USER CODE END StartOUT_ACTask */
}

/* USER CODE BEGIN Header_StartTempTask */
/**
* @brief Function implementing the TempTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTempTask */

void Calc_Temp() {
    // convert the value to resistance
    average = Global_TEMP_AVG;
    average = 3300 / average - 1;
    average = SERIESRESISTOR * average;


    steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C
    steinhart += 43;
}

void StartTempTask(void const * argument)
{
  /* USER CODE BEGIN StartTempTask */
  /* Infinite loop */

	  Blocked_by_TEMP=YES;
	  RELAY_OVER_TEMP=YES;

	  NeedUpdate_RELAY_STATE=YES;

	  osDelay(3000);

	  Calc_Temp();

	  while (steinhart>=TEMP_COLRSTART) {
			  Blocked_by_TEMP=YES;
			  Calc_Temp();
			  osDelay(1000);
	  }

	  Blocked_by_TEMP=NO;
	  RELAY_OVER_TEMP=NO;
	  NeedUpdate_RELAY_STATE=YES;

	  for(;;)
	  {
		    Calc_Temp();

		    if (steinhart<TEMP_MAX) {
			  if ((Blocked_by_TEMP==YES) && (steinhart<=TEMP_ROLLBACK)) {
				  if (temp_stamp==0)  {
					  temp_stamp=xTaskGetTickCount();
				  }
				  if (CheckStamp(temp_stamp,TEMP_DELAY_LENGTH)==1) {
					  Blocked_by_TEMP=NO;
					  if (RELAY_OVER_TEMP==YES) {
						  RELAY_OVER_TEMP=NO;
					  		NeedUpdate_RELAY_STATE=YES;
					  }
					  temp_stamp=0;
				 }
			  } else {
				  //Blocked_by_TEMP=NO;
				  if (RELAY_OVER_TEMP==YES) {
					  RELAY_OVER_TEMP=NO;
				  		NeedUpdate_RELAY_STATE=YES;
				  }
				  temp_stamp=0;
			  }
		    } else {
		  	  Blocked_by_TEMP=YES;
			  if (RELAY_OVER_TEMP==NO) {
				  RELAY_OVER_TEMP=YES;
			  		NeedUpdate_RELAY_STATE=YES;
			  }
		  	  temp_stamp=0;
		  	  osDelay(TEMP_ROLLBACK_DELAY_LENGTH*1000); //sec
		    }
		    osDelay(1000);
	  }

  /* USER CODE END StartTempTask */
}

/* USER CODE BEGIN Header_StartRelayTask */
/**
* @brief Function implementing the RelayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRelayTask */
void StartRelayTask(void const * argument)
{
  /* USER CODE BEGIN StartRelayTask */
  /* Infinite loop */
	//ALL ON
	//HAL_GPIO_WritePin(GPIOA, RELAY_5_Pin|RELAY_4_Pin|RELAY_3_Pin
	//                          |RELAY_1_Pin, GPIO_PIN_RESET);

	//HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_RESET);

	RELAY_OVER_TEMP=YES;
	RELAY_LOW_AB=YES;
	RELAY_VERYLOW_AB=YES;
	RELAY_NO_ACIN=YES;
	RELAY_NO_ACOUT=YES;

	/*RELAY_OVER_TEMP=NO;
	RELAY_LOW_AB=NO;
	RELAY_VERYLOW_AB=NO;
	RELAY_NO_ACIN=NO;
	RELAY_NO_ACOUT=NO;*/

	NeedUpdate_RELAY_STATE=YES;

	osDelay(3000);

  for(;;)
  {
	if (NeedUpdate_RELAY_STATE==YES) {
		NeedUpdate_RELAY_STATE=NO;
		// TEMP
		if (RELAY_OVER_TEMP==YES) {
			HAL_GPIO_WritePin(GPIOA, RELAY_1_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOA, RELAY_1_Pin, GPIO_PIN_RESET);
		}
		// LOW AB
		if (RELAY_LOW_AB==YES) {
			HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_RESET);
		}
		// RELAY_VERYLOW_AB
		if (RELAY_VERYLOW_AB==YES) {
			HAL_GPIO_WritePin(GPIOA, RELAY_3_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOA, RELAY_3_Pin, GPIO_PIN_RESET);
		}
		//RELAY_NO_ACIN
		if (RELAY_NO_ACIN==YES) {
			HAL_GPIO_WritePin(GPIOA, RELAY_4_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOA, RELAY_4_Pin, GPIO_PIN_RESET);
		}
		// RELAY_NO_ACOUT
		if (RELAY_NO_ACOUT==YES) {
			HAL_GPIO_WritePin(GPIOA, RELAY_5_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOA, RELAY_5_Pin, GPIO_PIN_RESET);
		}
	}
    osDelay(2000);
  }
  /* USER CODE END StartRelayTask */
}

/* USER CODE BEGIN Header_StartRestartCMDTask */
/**
* @brief Function implementing the RestartCMDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRestartCMDTask */
void StartRestartCMDTask(void const * argument)
{
  /* USER CODE BEGIN StartRestartCMDTask */
  /* Infinite loop */
  osDelay(10000);  // 10 sec

  for(;;)
  {
	if (RESTART_CMD_FLAG==YES) {
		Start_Attept_CNT=0;

		RESTART_CMD_FLAG=NO;

		osDelay(100);

		if (HAL_GPIO_ReadPin(RESTART_CMD_GPIO_Port,RESTART_CMD_Pin)==GPIO_PIN_RESET) {

			//if (MX3_INV_State==OFF) {
				//MX3_INV_State=OFF;
				Inv_OFF();

				//10msec
							volatile uint32_t sec_delay=30000;
							//volatile uint16_t sec_delay=250;

							for (uint16_t i=0; i<sec_delay; ++i) {
										// 1 microsec
										for (int j = 0; j < 32; ++j) {
											__asm__ __volatile__("nop\n\t":::"memory");
										}
							}
			//}



			// reset temp
			Calc_Temp();
			if (steinhart<TEMP_COLRSTART) {
				Blocked_by_TEMP=NO;
				temp_stamp=0;

				//reset AB
							if ((Global_AB_AVG>=AB_COLDRUN) && (Global_AB_AVG<=AB_MAX)) {
								Blocked_by_AB=NO;
								ab_stamp=0;

								Inv_ON();

								New_INV_STATE=ON;
							}


			} else {
				Blocked_by_TEMP=YES;
			}


			//New_INV_STATE=ON;

			//osDelay(1000);

			//MX3_INV_State=ON;
			//Inv_ON();

			//osDelay(5000);
		}

	}
    osDelay(3000);
  }
  /* USER CODE END StartRestartCMDTask */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDTask */
/*void StartLEDTask(void const * argument)
{
  uint32_t  temp_ledAB=0;

  for(;;)
  {
	temp_ledAB=Global_AB_AVG;

	if (temp_ledAB<=BUZZER_OPORA) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}
    //
	HAL_IWDG_Refresh(&hiwdg);
    osDelay(500);
  }
}
*/

/* USER CODE BEGIN Header_StartABTask */
/**
* @brief Function implementing the ABTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartABTask */
void StartABTask(void const * argument)
{
  /* USER CODE BEGIN StartABTask */
  /* Infinite loop */

   Blocked_by_AB=YES;
   RELAY_VERYLOW_AB=YES;

   osDelay(3000);

   uint32_t  temp_AB=Global_AB_AVG;

   NeedUpdate_RELAY_STATE=YES;

  	while (temp_AB<AB_COLDRUN) {
  		  Blocked_by_AB=YES;
  		  temp_AB=Global_AB_AVG;
  		  osDelay(1000);
  	  }

  	Blocked_by_AB=NO;
  	RELAY_VERYLOW_AB=NO;
  	NeedUpdate_RELAY_STATE=YES;


    for(;;)
    {
    	temp_AB=Global_AB_AVG;
		if ( (Blocked_by_AB==NO) && ((temp_AB<=AB_LOW)  || (temp_AB>=AB_MAX) ) ) {
			Blocked_by_AB=YES;
			ab_stamp=0;
			osDelay(3000);
		}
		if ( (Blocked_by_AB==YES) && (temp_AB>=AB_ROLLBACK) &&   (temp_AB<=AB_COLDRUN_FROM_MAX) ) {
			if (ab_stamp==0) {
				ab_stamp=xTaskGetTickCount();
			}
			if (CheckStamp(ab_stamp,AB_MAX_LENGTH)==1)  {
				 Blocked_by_AB=NO;
				 ab_stamp=0;
			}
		} else {
			ab_stamp=0;
		}

		if ((Blocked_by_AB==NO) && (temp_AB<=BUZZER_OPORA)){
			if (RELAY_LOW_AB==NO) {
				NeedUpdate_RELAY_STATE=YES;
				RELAY_LOW_AB=YES;
			}
		} else {
			if (RELAY_LOW_AB==YES) {
				NeedUpdate_RELAY_STATE=YES;
				RELAY_LOW_AB=NO;
			}
		}

		if (Blocked_by_AB==YES) {
			if (RELAY_VERYLOW_AB==NO) {
				NeedUpdate_RELAY_STATE=YES;
				RELAY_VERYLOW_AB=YES;
			}
		} else {
			if (RELAY_VERYLOW_AB==YES) {
				NeedUpdate_RELAY_STATE=YES;
				RELAY_VERYLOW_AB=NO;
			}
		}

		osDelay(1);
    }
  /* USER CODE END StartABTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
