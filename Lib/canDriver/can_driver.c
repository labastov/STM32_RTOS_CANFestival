/**
 ******************************************************************************
 * @file  : can_driver.c
 * @brief : This file CAN driver CANFestival for STM32
 * @version  V1.0.0
 * @date     24 April 2020
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */

#include "can_driver.h"
#include "FreeRTOS.h" // для отладки
#include "cmsis_os.h" // для отладки

#include "Slave.h"
#include "data.h" // для callback NMT

// /* заглушка в звязи с перименованием файла генерируемого генерартоом словаря автоматически */
// #define   Slave_Data node_Data

/*-----------------------------------------------------------------------*/
static xQueueHandle xCANSendQueue = NULL; //
static xQueueHandle xCANRcvQueue  = NULL; //
TaskHandle_t *CAN_Tx_Task_Header;
TaskHandle_t *CAN_Rx_Task_Header;

/*  */
static TIMEVAL last_counter_val = 0;
static TIMEVAL elapsed_time     = 0;

/*  дельта текущего ID со словарем  и старый ID для отслеживания его смены и перезагрузки фильтра */
int8_t can_id_delta;
int8_t old_can_id;

// структуры в соответвии с HAL
extern TIM_HandleTypeDef hCANOPEN_TIMx;

// CAN_RxHeaderTypeDef pRxHeader; // declare header for message reception

uint32_t TxMailbox;

/*-----------------------------------------------------------------------*/
static void CAN_Tx_Task(void *pvParameters);
static void CAN_Rx_Task(void *pvParameters);

/**
 ******************************************************************************
 *     CAN_Init_Driver
 *  @brief  create Tx and Rx queues create Tx and Rx tasks
 * @author  SWUST Tom
 ******************************************************************************
 */

void CAN_Init_Driver(void) {
  // BaseType_t xReturn;

  /*  */
  if (xCANSendQueue == NULL) {
    xCANSendQueue = xQueueCreate(CAN_TX_QUEUE_LEN, sizeof(CanTxMsgTypeDef));
    if (xCANSendQueue == NULL) {
      can_debug_printf("CANSendQueue create failed\n");
      return; //
    }
  }

  if (xCANRcvQueue == NULL) {
    xCANRcvQueue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(CanRxMsgTypeDef));
    if (xCANRcvQueue == NULL) {
      can_debug_printf("CANRcvQueue create failed\n");
      return; //
    }
  }

  /*  */
  if (xTaskCreate(CAN_Tx_Task, "CAN_Tx_Task", CAN_TX_STACK_SIZE, NULL, CAN_TX_TASK_PRIORITY, CAN_Tx_Task_Header) !=
      pdPASS) {
    can_debug_printf("CANSend_Task create failed\n Need increase heap size!");
    return; //
  }

  if (xTaskCreate(CAN_Rx_Task, "CAN_Rx_Task", CAN_RX_STACK_SIZE, NULL, CAN_RX_TASK_PRIORITY, CAN_Rx_Task_Header) !=
      pdPASS) {
    can_debug_printf("CANRcv_Task create failed\n");
    return;
  }
}

/**
 ******************************************************************************
 *     CANSend_Date
 * @brief  Send date from Canfestival for Transmit and
 *         insert it to TX Queue
 * TxMsg --- short TX structure
 *
 * @author  SWUST Tom. LVA Editing for HAL support
 ******************************************************************************
 */

void CAN_Send_Date(CanTxMsgTypeDef TxMsg) {
  if (xQueueSend(xCANSendQueue, &TxMsg, 100) != pdPASS) {
    can_debug_printf("CANSendQueue failed\n");
  }
}

/**
 ******************************************************************************
 *      CAN_Tx_Task
 * @brief   Read TX queue and Tranmit date to CAN Bus
 * pvParameters - must delete?
 * @author  SWUST Tom
 ******************************************************************************
 */
static void CAN_Tx_Task(void *pvParameters) {
  static CanTxMsgTypeDef TxMsg;

  /*  */
  for (;;) {
    if (xQueueReceive(xCANSendQueue, &TxMsg, 100) == pdTRUE) {
      if (uxQueueMessagesWaiting(xCANSendQueue)) {
        can_debug_printf("xCANSendQueue size:%d\n", uxQueueMessagesWaiting(xCANSendQueue));
      }
      if (CAN_Transmit(&_hcan, &TxMsg) != HAL_OK) {
        can_debug_printf("CAN_Transmit is NOT OK repeat until 10 ms!\n");
        // вставляем в начало очереди
        if (xQueueSendToFront(xCANSendQueue, &TxMsg, 100) != pdPASS) {
          can_debug_printf("CAN_Transmit FULL. We lost packet !\n");
          vTaskDelay(10);
        }
      }
    }
  }
}

/**
 ******************************************************************************
 *     CANRcv_DateFromISR
 * @brief   Recieved date from Can bus over ISR and штыке it in queue
 *  RxMsg   short RX structure
 *
 * @author  SWUST Tom
 ******************************************************************************
 */
void CAN_Rcv_DateFromISR(CanRxMsgTypeDef *RxMsg) {
  static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  if (NULL != xCANRcvQueue) {
    // xQueueSendFromISR
    if (xQueueSendToBackFromISR(xCANRcvQueue, RxMsg, &xHigherPriorityTaskWoken) != pdPASS) {
      can_debug_printf("Insert Date to Rx QuxQueue is not OK\n");
    }
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
  }
}

/**
 ******************************************************************************
 *     CANRcv_Task
 * @brief Main Task Theare work Canfestival code.
 *        Received date from queure an generation other events
 * pvParameters  - must delete ?
 *
 * @author  SWUST Tom. LVA Editing for HAL support
 *******************************************************************************
 */

static void CAN_Rx_Task(void *pvParameters) {
  static CanRxMsgTypeDef RxMsg;
  static Message msg;

  uint8_t i = 0;

  for (;;) {
    if (xQueueReceive(xCANRcvQueue, &RxMsg, 100) == pdTRUE) {
      /*т.к. Canfestival не позволяет менять адрес устроцйства (dj всяком случае у меня не получилось) делаем это в
      драйвере,
      необходимо чтобы внутри функций Canfestival  cob_id всегда =0x00 */
      if (RxMsg.StdId == 0x00) {
        if (RxMsg.Data[1] == 0x00) {
          ; // ничего не далеам это шировоещательная команда
        } else if (RxMsg.Data[1] == can_id_delta + (uint8_t)*Slave_Data.bDeviceNodeId) {
          RxMsg.Data[1] = RxMsg.Data[1] - can_id_delta; // подменям ID в NMT сообщении т.к оно пероснальное
        } else {
          RxMsg.Data[0] = 0xFF; // делаем сообщение невалидным чтобы небыло ошибочной отработки
        }
        msg.cob_id = 0;
      } else {
        msg.cob_id = RxMsg.StdId - can_id_delta; // подменям ID в прочих сообщениях
      }

      if (CAN_RTR_REMOTE == RxMsg.RTR)
        msg.rtr = 1;
      else
        msg.rtr = 0;

      msg.len = (UNS8)RxMsg.DLC;

      for (i        = 0; i < RxMsg.DLC; i++)
        msg.data[i] = RxMsg.Data[i];
      // TIM_ITConfig(CANOPEN_TIMx, TIM_IT_Update, DISABLE);
      __HAL_TIM_DISABLE_IT(&hCANOPEN_TIMx, TIM_IT_UPDATE);
      canDispatch(&Slave_Data, &msg);
      // TIM_ITConfig(CANOPEN_TIMx, TIM_IT_Update, ENABLE);
      __HAL_TIM_ENABLE_IT(&hCANOPEN_TIMx, TIM_IT_UPDATE);

      can_tasks_debug_printf("Task:%s free Stack:%d\n", pcTaskGetName(xTaskGetCurrentTaskHandle()),
                       uxTaskGetStackHighWaterMark(xTaskGetCurrentTaskHandle()));
    }
  }
}

/**
 ******************************************************************************
 * canSend
 * @brief  function mediator between Canfestival code and other code
 *         Recived date from Canfestival code and insert it to Tx queure
 * @author  SWUST Tom
 ******************************************************************************
 */
unsigned char canSend(CAN_PORT notused, Message *m) {
  uint8_t i;
  static CanTxMsgTypeDef TxMsg;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  /*т.к. Canfestival не позволяет менять адрес устроцйства делаем это в драйвере,
  nfrbv образом внутри функций Canfestival  cob_id всегда =0x00 */
  TxMsg.StdId = m->cob_id + can_id_delta;

  if (m->rtr)
    TxMsg.RTR = CAN_RTR_REMOTE;
  else
    TxMsg.RTR = CAN_RTR_DATA;

  // TxMsg.IDE = CAN_ID_STD;
  TxMsg.DLC = m->len;
  for (i          = 0; i < m->len; i++)
    TxMsg.Data[i] = m->data[i];

  /*  */
  if (0 == __get_CONTROL()) {
    if (xQueueSendFromISR(xCANSendQueue, &TxMsg, &xHigherPriorityTaskWoken) != pdPASS) {
      return 0xFF;
    }
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
  } else {
    if (xQueueSend(xCANSendQueue, &TxMsg, 100) != pdPASS) {
      return 0xFF;
    }
  }
  return 0;
}

/**
 ******************************************************************************
 *     setTimer
 * @brief   Set the timer for the next alarm.
 * @author  SWUST Tom. LVA Editing for HAL support
 ******************************************************************************
 */

void setTimer(TIMEVAL value) { __HAL_TIM_SET_AUTORELOAD(&hCANOPEN_TIMx, value); }

/**
 ******************************************************************************
 *     getElapsedTime
 * @brief Return the elapsed time to tell the Stack how much time is spent
 *        since last call.
 * @author  SWUST Tom. LVA Editing for HAL support
 ******************************************************************************
 */
TIMEVAL getElapsedTime(void) {
  // uint32_t timer = TIM_GetCounter(CANOPEN_TIMx); // Copy the value of the running timer
  uint32_t timer = __HAL_TIM_GET_COUNTER(&hCANOPEN_TIMx); // Copy the value of the running timer
  if (timer < last_counter_val)
    timer += CAN_OPEN_TIM_PERIOD;
  TIMEVAL elapsed = timer - last_counter_val + elapsed_time;
  return elapsed;
}

/**
 ******************************************************************************
 * TIMx_DispatchFromISR
 *
 *  @author SWUST Tom
 ******************************************************************************
 */

void TIMx_DispatchFromISR(void) {
  last_counter_val = 0;
  elapsed_time     = 0;
  TimeDispatch();
}

/**
 ******************************************************************************
 *     CAN_Transmit
 * @brief Received date in short Tx structure from CAN_Tx_Task convert it to
 * long TX HAL structur and sent to bus.
 * @param  - CAN handle Structure definition
 *         - short Tx structure
 * @return - standart HAL error code
 * @author     LVA
 ******************************************************************************
 */
HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *_hcan, CanTxMsgTypeDef *TxMsg) {
  // обявляем структуру в соотвевии с HAL
  CAN_TxHeaderTypeDef pTXHeader;
  pTXHeader.DLC                = (uint32_t)TxMsg->DLC;
  pTXHeader.ExtId              = 0U;
  pTXHeader.IDE                = CAN_ID_STD;
  pTXHeader.RTR                = (uint32_t)TxMsg->RTR;
  pTXHeader.StdId              = (uint32_t)TxMsg->StdId;
  pTXHeader.TransmitGlobalTime = 0U;

  if (HAL_CAN_AddTxMessage(_hcan, &pTXHeader, TxMsg->Data, &TxMailbox) != HAL_OK) {
    can_debug_printf("%s -> HAL_CAN_AddTxMessage HAL_ERROR\n", __FUNCTION__);
    can_debug_printf("TxMailbox:%d\t", TxMailbox);
    // uint32_t transmitmailbox = (READ_REG(hcan->Instance->TSR) & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
    // читаем состояние ящиков из регисторв
    can_debug_printf("transmitmailbox:%d\t%d|%d|%d = %d\n",
                     ((READ_REG(_hcan->Instance->TSR) & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos),
                     ((READ_REG(_hcan->Instance->TSR) & CAN_TSR_TME0) >> CAN_TSR_TME0_Pos),
                     ((READ_REG(_hcan->Instance->TSR) & CAN_TSR_TME1) >> CAN_TSR_TME1_Pos),
                     ((READ_REG(_hcan->Instance->TSR) & CAN_TSR_TME2) >> CAN_TSR_TME2_Pos),
                     ((READ_REG(_hcan->Instance->TSR) & CAN_TSR_TME) >> CAN_TSR_TME_Pos));
    osDelay(1000); // для отладки
    return HAL_ERROR;
  }
  can_debug_printf("StdId:0x%x\tRTR:0x%x\tDLC:0x%x\tData0x:%02x %02x %02x %02x %02x %02x %02x %02x\n",
                   pTXHeader.StdId, pTXHeader.RTR, pTXHeader.DLC, TxMsg->Data[0], TxMsg->Data[1], TxMsg->Data[2],
                   TxMsg->Data[3], TxMsg->Data[4], TxMsg->Data[5], TxMsg->Data[6], TxMsg->Data[7]);
  can_tasks_debug_printf("TX Task Free Stack:%d\n",
                   uxTaskGetStackHighWaterMark(xTaskGetCurrentTaskHandle())); // CAN_Tx_Task_Header
  // Debug_view_int(TxMailbox);

  return HAL_OK;
}

/**
 ******************************************************************************
 *         CAN_SetFilter
 * @brief Initialization hardware Rx Can filter
 * @param  - CAN handle Structure definition
 *         - Can Id
 * @return - standart HAL error code
 * @author     LVA
 ******************************************************************************
 */
HAL_StatusTypeDef CAN_SetFilter(CAN_HandleTypeDef *_hcan, uint8_t _can_id) {

#ifdef ENABLE_SECOND_CAN_FILTER
#define CAN_FILTERS 2
#else
#define CAN_FILTERS 1
#endif

  CAN_FilterTypeDef CAN_FilterConfig[CAN_FILTERS] = {
      {.FilterIdHigh         = (_can_id << 5),
       .FilterIdLow          = (0x00 << 5), // NMT messages
       .FilterMaskIdHigh     = (0x7F << 5),
       .FilterMaskIdLow      = (0x7F << 5),
       .FilterFIFOAssignment = 0,
       .FilterBank           = 1,
       .FilterMode           = CAN_FILTERMODE_IDMASK,
       .FilterActivation     = ENABLE,
       .SlaveStartFilterBank = 1},
#ifdef ENABLE_SECOND_CAN_FILTER
      {.FilterIdHigh         = 0x0000,
       .FilterIdLow          = 0x0000,
       .FilterMaskIdHigh     = 0x0000,
       .FilterMaskIdLow      = 0x0000,
       .FilterFIFOAssignment = 1U,
       .FilterBank           = 2,
       .FilterMode           = CAN_FILTERMODE_IDMASK,
       .FilterActivation     = ENABLE,
       .SlaveStartFilterBank = 0},
#endif
  };

  can_debug_printf("_can_id:%02x IdHigh:%04x  MaskIdHigh:%04x IdLow: %04x MaskIdLow:%04x\n", _can_id,
                   CAN_FilterConfig[0].FilterIdHigh, CAN_FilterConfig[0].FilterMaskIdHigh,
                   CAN_FilterConfig[0].FilterIdLow, CAN_FilterConfig[0].FilterMaskIdLow);

  if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfig[0]) != HAL_OK) { // configure CAN filter
    can_debug_printf("HAL_CAN_ConfigFilter0 ERROR\n");
    return HAL_ERROR;
  }
  // только если нужен второй фильтр для отладки первого
  // if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfig[1]) != HAL_OK) { // configure CAN filter
  //   can_debug_printf("HAL_CAN_ConfigFilter0 ERROR\n");
  //   return HAL_ERROR;
  // }
  return HAL_OK;
}

/**
 ******************************************************************************
 *         NMT_Slave_Node_Reset_Callback_Function
 * @brief Callback for NMT messages node reset. Reload microcontroller.
 *
 * @param  - Object dictonary structure from Canfestival
 * @return - none
 * @author     LVA
 ******************************************************************************
 */
void NMT_Slave_Node_Reset_Callback_Function(CO_Data *m) {
  can_debug_printf("NMT_command to Reload\n");
  HAL_NVIC_SystemReset();
}

/**
 ******************************************************************************
 *         state_change function
 * @brief CA callback called when node state changes
 * @param  - Object dictonary structure from Canfestival
 * @return - none
 * @author     from example Canfestival code
 ******************************************************************************
 */
void state_change(CO_Data *d) {
  if (d->nodeState == Initialisation) {
    can_debug_printf("Node state is now  : Initialisation\n");
  } else if (d->nodeState == Disconnected) {
    can_debug_printf("Node state is now  : Disconnected\n");
  } else if (d->nodeState == Connecting) {
    can_debug_printf("Node state is now  : Connecting\n");
  } else if (d->nodeState == Preparing) {
    can_debug_printf("Node state is now  : Preparing\n");
  } else if (d->nodeState == Stopped) {
    can_debug_printf("Node state is now  : Stopped\n");
  } else if (d->nodeState == Operational) {
    can_debug_printf("Node state is now  : Operational\n");
  } else if (d->nodeState == Pre_operational) {
    can_debug_printf("Node state is now  : Pre_operational\n");
  } else if (d->nodeState == Unknown_state) {
    can_debug_printf("Node state is now  : Unknown_state\n");
  } else {
    can_debug_printf("Error : unexpected node state\n");
  }
}

/*
 ******************************************************************************
*         callback_on_position
* @brief    A callback called when position is written
* @param    - Object dictonary structure from Canfestival
 *          - indextable structure
 *          - subindex
 * @return  - none
 * @author  LVA
 ******************************************************************************
 */
UNS32 callback_on_position(CO_Data *d, const indextable *idxtab, UNS8 bSubindex) {

  /* Dac 0x2004sub0x00 */
  if (idxtab->index == 0x2004 && bSubindex == 0x00) {
    can_debug_printf("Dac have been set to %d (0x%x)\n", Dac, Dac);
  }

  /* can_id 0x2004sub0x00*/
  if (idxtab->index == 0x2000 && bSubindex == 0x00) {
    can_debug_printf("Can_id have been set to %d (0x%x)\n", can_id, can_id);

    if (can_id != old_can_id) { /* проверяем что can_id не изменился */
      if (can_id > 127) {
        can_id = old_can_id;
        can_debug_printf("Can_id is not valid. set to %d\n", can_id);
      } else {
        can_id_delta = can_id - Slave_bDeviceNodeId;
        old_can_id   = can_id;
        CAN_SetFilter(&_hcan, can_id);
      }
    }
  }
  return 0;
}