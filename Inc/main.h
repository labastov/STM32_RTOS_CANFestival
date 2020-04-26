/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug.h"
// #include "usart.h"
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
int ss;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define F_Pin GPIO_PIN_3
#define F_GPIO_Port GPIOA
#define A_Pin GPIO_PIN_4
#define A_GPIO_Port GPIOA
#define B_Pin GPIO_PIN_5
#define B_GPIO_Port GPIOA
#define C_Pin GPIO_PIN_6
#define C_GPIO_Port GPIOA
#define DP_Pin GPIO_PIN_7
#define DP_GPIO_Port GPIOA
#define PWM_Pin GPIO_PIN_0
#define PWM_GPIO_Port GPIOB
#define Led1_Pin GPIO_PIN_1
#define Led1_GPIO_Port GPIOB
#define E_Pin GPIO_PIN_2
#define E_GPIO_Port GPIOB
#define D_Pin GPIO_PIN_10
#define D_GPIO_Port GPIOB
#define G_Pin GPIO_PIN_11
#define G_GPIO_Port GPIOB
#define Addr3_Pin GPIO_PIN_12
#define Addr3_GPIO_Port GPIOB
#define Addr2_Pin GPIO_PIN_13
#define Addr2_GPIO_Port GPIOB
#define Addr1_Pin GPIO_PIN_14
#define Addr1_GPIO_Port GPIOB
#define Addr0_Pin GPIO_PIN_15
#define Addr0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* объявляем режим отладки через SWO, только в этом случае
   функции включаются в пошивку */
#define DEBUG_SWO

/* если используете отладку через UART. не забываем менять номер
   используемого порта */
// #define DEBUG_UART huart3

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
