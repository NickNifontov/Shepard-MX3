/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEBUG_MODE
//#define NOT_USE_8V_BLOCK

#define DELAY_ADC_SUM 20 // 20 msec averege value summator


#define BUZZER_OPORA 1000 // 1.0V - 0.2V
#define BUZZER_OPORA_3p ((uint16_t) (BUZZER_OPORA*1.03)) // 1.0V - 0.2V
#define AB_LOW ((uint16_t) (BUZZER_OPORA*0.92))
#define AB_LOW_3p ((uint16_t) (BUZZER_OPORA*0.95))
#define AB_ROLLBACK ((uint16_t) (BUZZER_OPORA*1.1))
#define AB_MAX ((uint16_t) (BUZZER_OPORA*1.30))
#define AB_COLDRUN ((uint16_t) (BUZZER_OPORA*1.05))
#define AB_COLDRUN_FROM_MAX ((uint16_t) (AB_MAX*0.95))
#define AB_MAX_LENGTH 120 //2 min

#define TEMP_DELAY_LENGTH 120  //2 min
#define TEMP_ROLLBACK_DELAY_LENGTH 120  //2 min

#define TEMP_MAX 70
#define TEMP_COLRSTART 50
#define TEMP_ROLLBACK 45


#define RESTART_CMD_Pin GPIO_PIN_14
#define RESTART_CMD_GPIO_Port GPIOC
#define RESTART_CMD_EXTI_IRQn EXTI4_15_IRQn
#define RELAY_2_Pin GPIO_PIN_15
#define RELAY_2_GPIO_Port GPIOC
#define IN_AC_Pin GPIO_PIN_0
#define IN_AC_GPIO_Port GPIOA
#define AB_Pin GPIO_PIN_1
#define AB_GPIO_Port GPIOA
#define TEMP_ADC_Pin GPIO_PIN_2
#define TEMP_ADC_GPIO_Port GPIOA
#define OUT_AC_Pin GPIO_PIN_3
#define OUT_AC_GPIO_Port GPIOA
#define RELAY_5_Pin GPIO_PIN_4
#define RELAY_5_GPIO_Port GPIOA
#define RELAY_4_Pin GPIO_PIN_5
#define RELAY_4_GPIO_Port GPIOA
#define BLOCK_PORT_Pin GPIO_PIN_6
#define BLOCK_PORT_GPIO_Port GPIOA
#define RELAY_3_Pin GPIO_PIN_7
#define RELAY_3_GPIO_Port GPIOA
#define BLOCK_POWER_Pin GPIO_PIN_1
#define BLOCK_POWER_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_9
#define LED_GPIO_Port GPIOA
#define RELAY_1_Pin GPIO_PIN_10
#define RELAY_1_GPIO_Port GPIOA
#define DIO_Pin GPIO_PIN_13
#define DIO_GPIO_Port GPIOA
#define CLK_Pin GPIO_PIN_14
#define CLK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
typedef enum { ON, OFF } INV_STATE;
typedef enum { YES, NO } log_state;

extern volatile INV_STATE MX3_INV_State;
extern volatile log_state NeedUpdate_INV_STATE;

extern volatile log_state NeedUpdate_RELAY_STATE;
extern volatile log_state RELAY_OVER_TEMP;
extern volatile log_state RELAY_LOW_AB;
extern volatile log_state RELAY_VERYLOW_AB;
extern volatile log_state RELAY_NO_ACIN;
extern volatile log_state RELAY_NO_ACOUT;

#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  4)   /* Size of array aADCxConvertedData[] */

#define ADC_OVERSAMPLING 16

/* Variable containing ADC conversions data */
extern volatile uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

extern volatile uint16_t Global_IN_AC;
extern volatile uint32_t Global_IN_AVG;
extern volatile uint32_t Global_IN_SUM;
extern volatile uint16_t Global_IN_CNT;

extern volatile uint16_t Global_AB;
extern volatile uint32_t Global_AB_AVG;
extern volatile uint32_t Global_AB_SUM;
extern volatile uint16_t Global_AB_CNT;

extern volatile uint16_t Global_TEMP;
extern volatile uint32_t Global_TEMP_AVG;
extern volatile uint32_t Global_TEMP_SUM;
extern volatile uint16_t Global_TEMP_CNT;

extern volatile uint16_t Global_OUT_AC;
extern volatile uint32_t Global_OUT_AVG;
extern volatile uint32_t Global_OUT_SUM;
extern volatile uint16_t Global_OUT_CNT;

extern volatile log_state Blocked_by_PVD;
extern volatile log_state Blocked_by_AB;
extern volatile log_state Blocked_by_TEMP;

extern volatile log_state RESTART_CMD_FLAG;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
