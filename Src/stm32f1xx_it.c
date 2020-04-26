/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_it.h"
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "can_driver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void) {
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void) {
  /* USER CODE BEGIN HardFault_IRQn 0 */
  Debug_view_char('E');
  /* USER CODE END HardFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void) {
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void) {
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void) {
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void) {
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void) {
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */
  static CanRxMsgTypeDef RxMsg;

  CAN_RxHeaderTypeDef _RxHeader;
  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &_RxHeader, RxMsg.Data);
  // DEBUG_NAME_FUNCTION;
  if (_RxHeader.IDE == 0) {
    RxMsg.StdId = _RxHeader.StdId;
    RxMsg.RTR   = _RxHeader.RTR;
    RxMsg.DLC   = _RxHeader.DLC;
    // RxMsg.FMI = _RxHeader.FilterMatchIndex;
    CAN_Rcv_DateFromISR(&RxMsg);
#ifdef DEBUG_CAN
    debug_printf("CAN_RX_FIFO0 StdId:0x%04x\tRTR:0x%x\tDLC:0x%x\tFilterMatchIndex:%02x\n", _RxHeader.StdId,
                 _RxHeader.RTR, _RxHeader.DLC, _RxHeader.FilterMatchIndex);
#endif // DEBUG_CAN
  }
  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void) {
/* USER CODE BEGIN CAN1_RX1_IRQn 0 */

#ifdef ENABLE_SECOND_CAN_FILTER
  static CanRxMsgTypeDef RxMsg;

  CAN_RxHeaderTypeDef _RxHeader;
  HAL_CAN_GetRxMessage(&_hcan, CAN_RX_FIFO1, &_RxHeader, RxMsg.Data);
  DEBUG_NAME_FUNCTION;
  // т.к для каждого FIFI своя нумерация FilterMatchIndex, не надо удивляться почему тут тоже 0
  can_debug_printf("\tCAN_RX_FIFO1 StdId:0x%04x\tExtId:%x\t IDE:%x\tRTR:0x%x\tDLC:0x%x\tFilterMatchIndex:%02x\n",
                   _RxHeader.StdId, _RxHeader.ExtId, _RxHeader.IDE, _RxHeader.RTR, _RxHeader.DLC,
                   _RxHeader.FilterMatchIndex);
#endif
  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void) {
  /* USER CODE BEGIN TIM2_IRQn 0 */
  // DEBUG_NAME_FUNCTION;

  if (__HAL_TIM_GET_IT_SOURCE(&hCAN_TIMx, TIM_IT_UPDATE)) {

    // TIM_ClearITPendingBit(CANOPEN_TIMx, TIM_IT_Update);
    __HAL_TIM_CLEAR_IT(&hCAN_TIMx, TIM_IT_UPDATE);

    TIMx_DispatchFromISR();
    // DEBUG_NAME_FUNCTION;
  }
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void) {
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void CAN_RX_IRQHandler(void) {

  // static CanRxMsg RxMsg;

  // CAN_RxHeaderTypeDef cRxHeader;
  // HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO1, &cRxHeader, RxMsg.Data);
  // DEBUG_NAME_FUNCTION;
  // if (cRxHeader.IDE == 0) {
  //     RxMsg.StdId = cRxHeader.StdId;
  //     RxMsg.RTR = cRxHeader.RTR;
  //     RxMsg.DLC = cRxHeader.DLC;
  //     // RxMsg.ExtId = _iRxHeader.ExtId;
  //     // RxMsg.FMI = _iRxHeader.FilterMatchIndex;
  //     // RxMsg.IDE = _iRxHeader.IDE;
  //     // _iRxHeader.Timestamp;
  //     CANRcv_DateFromISR(&RxMsg);
  // }
}

void CANOPEN_TIM_IRQ_Handler(void) {
  if (__HAL_TIM_GET_IT_SOURCE(&hCAN_TIMx, TIM_IT_UPDATE)) {

    __HAL_TIM_CLEAR_IT(&hCAN_TIMx, TIM_IT_UPDATE);

    TIMx_DispatchFromISR();
    DEBUG_NAME_FUNCTION;
  }
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
