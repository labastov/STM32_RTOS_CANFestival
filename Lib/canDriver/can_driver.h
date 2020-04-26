
/**
 ******************************************************************************
 * @file  : can_driver.h
 * @brief : This file CAN driver CANFestival for STM32
 * @version  V1.0.0
 * @date     24 April 2020
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */

#ifndef _CAN_DRIVER_H
#define _CAN_DRIVER_H

/* --------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "can.h"
#include "data.h" // для callback NMT
#include "semphr.h"
#include "task.h"
#include "tim.h"
// #include "taskcan.h"

// #include "bsp_can.h"

/*
 ******************************************************************************
 * так как используем HAL а внем другие структуры для передачи приемо сообщений
 * надо вводить старые струкутры далее придется переность данные из одной 
 * структуры в другую применяем эти стурктуры т.к. они меньше и не надо 
 * раздувать очередь (применяется uint8_t вместо uint32_t)
 ******************************************************************************
 */

typedef struct {
  uint16_t StdId; /*!< Specifies the standard identifier.
                       This parameter can be a value between 0 to 0x7FF. */

  uint8_t RTR; /*!< Specifies the type of frame for the message that will
                    be transmitted. This parameter can be a value of
                    @ref CAN_remote_transmission_request */

  uint8_t DLC; /*!< Specifies the length of the frame that will be
                    transmitted. This parameter can be a value between
                    0 to 8 */

  uint8_t Data[8]; /*!< Contains the data to be transmitted. It ranges from 0
                        to 0xFF. */
} CanTxMsgTypeDef; // аналог CAN_TxHeaderTypeDef

typedef struct {
  uint16_t StdId; /*!< Specifies the standard identifier.
                       This parameter can be a value between 0 to 0x7FF. */

  uint8_t RTR; /*!< Specifies the type of frame for the received message.
                    This parameter can be a value of
                    @ref CAN_remote_transmission_request */

  uint8_t DLC; /*!< Specifies the length of the frame that will be received.
                    This parameter can be a value between 0 to 8 */

  uint8_t Data[8]; /*!< Contains the data to be received. It ranges from 0 to
                        0xFF. */

  // uint8_t FMI; /*!< Specifies the index of the filter the message stored in
  //                   the mailbox passes through. This parameter can be a
  //                   value between 0 to 0xFF */
} CanRxMsgTypeDef; // аналог CAN_RxHeaderTypeDef;

/*  Config CAN tasks parametrs----------------------------------------------------------------*/
#define CAN_TX_QUEUE_LEN 10  //
#define CAN_RX_QUEUE_LEN 10  // 
#define CAN_TX_STACK_SIZE 256  // 
#define CAN_TX_TASK_PRIORITY 3 // 
#define CAN_RX_STACK_SIZE 256  // 
#define CAN_RX_TASK_PRIORITY 3 // 

#define CAN_OPEN_TIM_PERIOD 65535 //��ʱ����

// #define  ENABLE_SECOND_CAN_FILTER // включаем второй фильтр пропускающй все 

#define hCANOPEN_TIMx htim2 
#define _hcan hcan // CAN handle Structure definition for CAN_HandleTypeDef

/* определяем переменную для хранения адреса устройства на шине CAN сама переменая хранится в OD_CAN*/
extern uint8_t can_id;  // from Slave.c
extern uint8_t Slave_bDeviceNodeId; // from Slave.c


/*
*/
// #define DEBUG_CAN
#ifdef DEBUG_CAN
#define can_debug_printf(args...) \
  portENTER_CRITICAL();                                                        \
  debug_printf(args);                                                   \
  portEXIT_CRITICAL()
#else
#define can_debug_printf(args...)                                                                                       \
  { ; }
#endif // DEBUG_CAN

// #define DEBUG_CAN_TASKS
#ifdef DEBUG_CAN_TASKS
#define can_tasks_debug_printf(args...)                                                                                      \
  portENTER_CRITICAL();                                                                                                \
  debug_printf(args);                                                                                                  \
  portEXIT_CRITICAL()
#else
#define can_tasks_debug_printf(args...)                                                                                \
  { ; }
#endif // DEBUG_CAN

/*  function declarating ------------------------------------------------------------------*/

void CAN_Init_Driver(void);
void CAN_Send_Date(CanTxMsgTypeDef TxMsg);
void CAN_Rcv_DateFromISR(CanRxMsgTypeDef *RxMsg);
void TIMx_DispatchFromISR(void);

/* создаем функцию т.к. такой в HAL Нет*/
HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *_hcan, CanTxMsgTypeDef *TxMsg);

/*настраиваем фильтр CAN */
HAL_StatusTypeDef CAN_SetFilter(CAN_HandleTypeDef *_hcan, uint8_t _can_id);

/* callback for NMT */
void NMT_Slave_Node_Reset_Callback_Function(CO_Data *);

/* callback for PDO and SDO */
UNS32 callback_on_position(CO_Data *d, const indextable *idxtab, UNS8 bSubindex);

#endif /* _CAN_DRIVER_H */
