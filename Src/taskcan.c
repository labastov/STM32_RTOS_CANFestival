/**
 ******************************************************************************
 * @file  : taskcan.c
 * @brief : This file starting task for CAN driver CANFestival of STM32
 * @version  V1.0.0
 * @date     24 April 2020
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */

#include "taskcan.h"
#include "can_driver.h"

/*
 ******************************************************************************
 *         CAN_Start_Task
 * @brief    Task determinated on CubeMX. Ерыш task initialization can Driver.
 * After initialization task deleted.
 * @param    - *
 * @return  - none
 * @author  LVA
 ******************************************************************************
 */
void CAN_Start_Task(void *argument) {

  CAN_Configuration();
  HAL_TIM_Base_Start_IT(&hCANOPEN_TIMx); // включаем таймер  для CAN в режиме прерывания.

  // extern void CAN_Driver_Init(void); // from can_driver.c
  CAN_Init_Driver();
  CAN_Start_App();

  // DEBUG_NAME_FUNCTION;

    extern osThreadId CAN_TaskHandle;
  vTaskDelete(CAN_TaskHandle);// Мавр сделел свое дело, его можно навкерное убить.
}

/*
 ******************************************************************************
 *         CAN_Configuration
 * @brief   Staring configutsating Can environment.
 *          - initialisation Can_id variables (can_id_delta and old_can_id)
 *          - Setting Canfilter
 *          - Staring CAN on microcontroller
 *          - enable interups
 * @param   - none
 * @return  - none
 * @author  LVA
 ******************************************************************************
 */
HAL_StatusTypeDef CAN_Configuration(void) {

  extern uint8_t Slave_bDeviceNodeId; // from Slave.c
  extern uint8_t can_id;              // from Slave.c
  extern int8_t can_id_delta;         // from can_driver.c
  extern int8_t old_can_id;           // from can_driver.c

  /* initialisation Can_id variables */
  if (can_id > 127) {
    can_id = Slave_bDeviceNodeId;
  }
  can_id_delta = can_id - Slave_bDeviceNodeId;
  old_can_id   = can_id;

  /* Setting Canfilter */
  CAN_SetFilter(&_hcan, can_id); /*настрииваем фильтр */

  
  HAL_CAN_Start(&_hcan);// start CAN
  can_debug_printf("HAL_CAN_Start state: ");
  if (hcan.State == HAL_CAN_STATE_LISTENING) {
    can_debug_printf("%s\t OK.\n", "HAL_CAN_STATE_LISTENING");
  } else if (hcan.State == HAL_CAN_STATE_READY) {
    can_debug_printf("%s\t OK.\n", "HAL_CAN_STATE_READY");
  } else {
    can_debug_printf("%\t ERROR.\n", hcan.State);
  }

  /* enable interrupts */
  if (HAL_CAN_ActivateNotification(&_hcan, CAN_IT_RX_FIFO0_MSG_PENDING)) {
    can_debug_printf("HAL_CAN_ActivateNotification ERROR\n");
    return HAL_ERROR;
  }
  
  #ifdef ENABLE_SECOND_CAN_FILTER
  // на втором канале разрешаем для отладки фильтра
  if (HAL_CAN_ActivateNotification(&_hcan, CAN_IT_RX_FIFO1_MSG_PENDING)) {
    can_debug_printf("HAL_CAN_ActivateNotification ERROR\n");
    return HAL_ERROR;
  }
  #endif

  return HAL_OK;
}
/*
 ******************************************************************************
 *         CAN_Start_App
 * @brief   Staring Canfestival aplication code.
 *          - initialisation callbask function
 *          - initialisation Canfestival function
 *          - Staring CAN on microcontroller
 * @param   - none
 * @return  - none
 * @author  LVA
 ******************************************************************************
 */
void CAN_Start_App(void) {

  // register the callbacks for event to other code
  RegisterSetODentryCallBack(&Slave_Data, 0x2000, 0, callback_on_position); // can_id
  RegisterSetODentryCallBack(&Slave_Data, 0x2004, 0, callback_on_position); // Dac
  Slave_Data.NMT_Slave_Node_Reset_Callback = NMT_Slave_Node_Reset_Callback_Function;

  extern uint8_t Slave_bDeviceNodeId; //  из slave.c
  setNodeId(&Slave_Data, Slave_bDeviceNodeId);
  setState(&Slave_Data, Initialisation);
  setState(&Slave_Data, Operational);
}