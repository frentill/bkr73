/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct ACD_s {
	// Порядок елеметів відповідає порядку Rank у налаштуванні ADC
	int16_t SIN;    // IN0
	int16_t COS;    // IN1
	int16_t Omega1; // IN2
	int16_t Omega2; // IN3
	int16_t F1;     // IN9
	int16_t Az;     // IN10
	int16_t F2;     // IN11
	int16_t D;      // IN13
}ACD_t;

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
#define SD_Pin GPIO_PIN_13
#define SD_GPIO_Port GPIOC
#define KT1_Pin GPIO_PIN_14
#define KT1_GPIO_Port GPIOC
#define KT2_Pin GPIO_PIN_15
#define KT2_GPIO_Port GPIOC
#define Az_Pin GPIO_PIN_0
#define Az_GPIO_Port GPIOC
#define F2_Pin GPIO_PIN_1
#define F2_GPIO_Port GPIOC
#define WDI_Pin GPIO_PIN_2
#define WDI_GPIO_Port GPIOC
#define D_Pin GPIO_PIN_3
#define D_GPIO_Port GPIOC
#define SIN_Pin GPIO_PIN_0
#define SIN_GPIO_Port GPIOA
#define COS_Pin GPIO_PIN_1
#define COS_GPIO_Port GPIOA
#define Omega1_Pin GPIO_PIN_2
#define Omega1_GPIO_Port GPIOA
#define Omega2_Pin GPIO_PIN_3
#define Omega2_GPIO_Port GPIOA
#define FRAM_CS_Pin GPIO_PIN_4
#define FRAM_CS_GPIO_Port GPIOA
#define DAC_L_Pin GPIO_PIN_4
#define DAC_L_GPIO_Port GPIOC
#define DAC_CS_Pin GPIO_PIN_5
#define DAC_CS_GPIO_Port GPIOC
#define G_Pin GPIO_PIN_0
#define G_GPIO_Port GPIOB
#define F1_Pin GPIO_PIN_1
#define F1_GPIO_Port GPIOB
#define MONITOR_USART3_TX_Pin GPIO_PIN_10
#define MONITOR_USART3_TX_GPIO_Port GPIOB
#define MONITOR_USART3_RX_Pin GPIO_PIN_11
#define MONITOR_USART3_RX_GPIO_Port GPIOB
#define OK_Pin GPIO_PIN_12
#define OK_GPIO_Port GPIOB
#define PPS_Pin GPIO_PIN_13
#define PPS_GPIO_Port GPIOB
#define ON1_Pin GPIO_PIN_14
#define ON1_GPIO_Port GPIOB
#define A2_Pin GPIO_PIN_15
#define A2_GPIO_Port GPIOB
#define A1_Pin GPIO_PIN_6
#define A1_GPIO_Port GPIOC
#define A0_Pin GPIO_PIN_7
#define A0_GPIO_Port GPIOC
#define Q0_Pin GPIO_PIN_8
#define Q0_GPIO_Port GPIOC
#define Q2_Pin GPIO_PIN_9
#define Q2_GPIO_Port GPIOC
#define OPC_Pin GPIO_PIN_8
#define OPC_GPIO_Port GPIOA
#define BD_Pin GPIO_PIN_9
#define BD_GPIO_Port GPIOA
#define TC_Pin GPIO_PIN_10
#define TC_GPIO_Port GPIOA
#define ON2_Pin GPIO_PIN_11
#define ON2_GPIO_Port GPIOA
#define Q1_Pin GPIO_PIN_12
#define Q1_GPIO_Port GPIOA
#define REMOTE_UART4_TX_Pin GPIO_PIN_10
#define REMOTE_UART4_TX_GPIO_Port GPIOC
#define REMOTE_UART4_RX_Pin GPIO_PIN_11
#define REMOTE_UART4_RX_GPIO_Port GPIOC
#define RESERVE_UART5_TX_Pin GPIO_PIN_12
#define RESERVE_UART5_TX_GPIO_Port GPIOC
#define RESERVE_UART5_RX_Pin GPIO_PIN_2
#define RESERVE_UART5_RX_GPIO_Port GPIOD
#define IRQ2_Pin GPIO_PIN_5
#define IRQ2_GPIO_Port GPIOB
#define IRQ1_Pin GPIO_PIN_6
#define IRQ1_GPIO_Port GPIOB
#define REMOTE_DE1_Pin GPIO_PIN_7
#define REMOTE_DE1_GPIO_Port GPIOB
#define RESERVE_DE2_Pin GPIO_PIN_9
#define RESERVE_DE2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define NUM_CHANNELS 8
#define NUM_SAMPLES 8

extern ACD_t adcData;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart4;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
