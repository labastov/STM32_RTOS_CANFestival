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

/*-----------------------------------------------------------------------*/
static xQueueHandle xCANSendQueue = NULL; //
static xQueueHandle xCANRcvQueue  = NULL; //

TaskHandle_t *CAN_Tx_Task_Header;
TaskHandle_t *CAN_Rx_Task_Header;
static void CAN_Tx_Task(void *pvParameters);
static void CAN_Rx_Task(void *pvParameters);

static TIMEVAL last_counter_val = 0;
static TIMEVAL elapsed_time     = 0;

/*  дельта текущего ID со словарем  и старый ID для отслеживания его смены и перезагрузки фильтра */
int8_t can_id_delta;
int8_t old_can_id;

// // структуры в соответствии с HAL
// extern TIM_HandleTypeDef hCANOPEN_TIMx;

/*
 ******************************************************************************
 *         CAN_Start_Task
 * @brief    Task determinate on CubeMX. Task is initialization can Driver.
 *           After initialization can Driver task deleted. Task calling from
 *           freertos.c file.
 * @param    - *
 * @return  - none
 * @author  LVA
 ******************************************************************************
 */
void CAN_Start_Task(void *argument) {

  CAN_Configuration();
  HAL_TIM_Base_Start_IT(&hCAN_TIMx); // включаем таймер  для CAN в режиме прерывания.

  // extern void CAN_Driver_Init(void); // from can_driver.c
  CAN_Init_Driver();
  CAN_Start_App();

  // DEBUG_NAME_FUNCTION;

  extern osThreadId CAN_TaskHandle;
  vTaskDelete(CAN_TaskHandle); // Мавр сделал свое дело, его можно убить.
}

/*
 ******************************************************************************
 *         CAN_Configuration
 * @brief   Staring configuratins Can environment.
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

  // extern uint8_t Slave_bDeviceNodeId; // from Slave.c
  extern uint8_t can_id; // from Slave.c

  /* initialisation Can_id variables */
  if (can_id > 127) {
    can_id = Slave_bDeviceNodeId;
  }
  can_id_delta = can_id - Slave_bDeviceNodeId;
  old_can_id   = can_id;

  /* Setting Canfilter */
  CAN_SetFilter(&_hcan, can_id); /*настраиваем фильтр */

  HAL_CAN_Start(&_hcan); // start CAN
  can_debug_printf("HAL_CAN_Start state: ");
  if (_hcan.State == HAL_CAN_STATE_LISTENING) {
    can_debug_printf("%s\t OK.\n", "HAL_CAN_STATE_LISTENING");
  } else if (_hcan.State == HAL_CAN_STATE_READY) {
    can_debug_printf("%s\t OK.\n", "HAL_CAN_STATE_READY");
  } else {
    can_debug_printf("%\t ERROR.\n", _hcan.State);
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

/**
 ******************************************************************************
 *     CAN_Init_Driver
 *  @brief  create Tx and Rx queues create Tx and Rx tasks
 * @author  SWUST Tom
 ******************************************************************************
 */
/*-----------------------------------------------------------------------*/
static void CAN_Tx_Task(void *pvParameters);
static void CAN_Rx_Task(void *pvParameters);

void CAN_Init_Driver(void) {
  uint8_t Errors = TRUE;
  while (Errors) {
    /* Queues created */
    Errors = FALSE;
    if (xCANSendQueue == NULL) {
      xCANSendQueue = xQueueCreate(CAN_TX_QUEUE_LEN, sizeof(CanTxMsgTypeDef));
      if (xCANSendQueue == NULL) {
        can_debug_printf("CANSendQueue create failed\n");
        Errors = TRUE;
      }
    }
    if (xCANRcvQueue == NULL) {
      xCANRcvQueue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(CanRxMsgTypeDef));
      if (xCANRcvQueue == NULL) {
        can_debug_printf("CANRcvQueue create failed\n");
        Errors = TRUE;
      }
    }
    /* Tasks created */
    if (xTaskCreate(CAN_Tx_Task, "CAN_Tx_Task", CAN_TX_STACK_SIZE, NULL, CAN_TX_TASK_PRIORITY, CAN_Tx_Task_Header) !=
        pdPASS) {
      can_debug_printf("CANSend_Task create failed\n Need increase heap size!");
      Errors = TRUE;
    }
    if (xTaskCreate(CAN_Rx_Task, "CAN_Rx_Task", CAN_RX_STACK_SIZE, NULL, CAN_RX_TASK_PRIORITY, CAN_Rx_Task_Header) !=
        pdPASS) {
      can_debug_printf("CANRcv_Task create failed\n");
      Errors = TRUE;
    }
  }
}

/*
 ******************************************************************************
 *         CAN_Start_App
 * @brief   Staring Canfestival application code.
 *          - initialisation callbask function
 *          - initialisation Canfestival function
 *          - Staring CAN on microcontroller
 * @param   - none
 * @return  - none
 * @author  LVA
 ******************************************************************************
 */
void CAN_Start_App(void) {

  /* register the callbacks for event to other code */
  RegisterSetODentryCallBack(&Slave_Data, 0x2000, 0, callback_on_position); // can_id
  RegisterSetODentryCallBack(&Slave_Data, 0x2004, 0, callback_on_position); // Dac
  Slave_Data.NMT_Slave_Node_Reset_Callback = NMT_Slave_Node_Reset_Callback_Function;

  setNodeId(&Slave_Data, Slave_bDeviceNodeId);
  setState(&Slave_Data, Initialisation);
  setState(&Slave_Data, Operational);
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

#ifdef ENABLE_SECOND_CAN_FILTER // see can_driver.h
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

  if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfig[0]) != HAL_OK) { // configure main CAN filter
    can_debug_printf("HAL_CAN_ConfigFilter0 ERROR\n");
    return HAL_ERROR;
  }
#ifdef ENABLE_SECOND_CAN_FILTER
  if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfig[1]) != HAL_OK) { // configure debug CAN filter
    can_debug_printf("HAL_CAN_ConfigFilter0 ERROR\n");
    return HAL_ERROR;
  }
#endif
  return HAL_OK;
}

/**
 ******************************************************************************
 *      CAN_Tx_Task
 * @brief   Read TX queue and Tranmit date to CAN Bus
 * pvParameters - must delete?
 * @author  SWUST Tom updated LVA
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
 *     CANRcv_Task
 * @brief Main Task Theare work Canfestival code.
 *        Received date from queue an generation other events
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
      /*
      т.к. Canfestival не позволяет менять адрес устройства (во всяком случае,
       у меня не получилось) делаем это в драйвере, необходимо чтобы внутри
       функций Canfestival  cob_id всегда =0x00
      */
      if (RxMsg.StdId == 0x00) {
        if (RxMsg.Data[1] == 0x00) {
          ; /* ничего не далаем это широковещательная команда */
        } else if (RxMsg.Data[1] == can_id_delta + (uint8_t)*Slave_Data.bDeviceNodeId) {
          RxMsg.Data[1] = RxMsg.Data[1] - can_id_delta; /* подменям ID в NMT сообщении т.к оно персональное */
        } else {
          RxMsg.Data[0] = 0xFF; /* делаем сообщение невалидным чтобы не было ошибочной отработки*/
        }
        msg.cob_id = 0;
      } else {
        msg.cob_id = RxMsg.StdId - can_id_delta; /* подменям ID в прочих сообщениях */
      }

      if (CAN_RTR_REMOTE == RxMsg.RTR)
        msg.rtr = 1;
      else
        msg.rtr = 0;

      msg.len = (UNS8)RxMsg.DLC;

      for (i        = 0; i < RxMsg.DLC; i++)
        msg.data[i] = RxMsg.Data[i];
      __HAL_TIM_DISABLE_IT(&hCAN_TIMx, TIM_IT_UPDATE);
      canDispatch(&Slave_Data, &msg);
      __HAL_TIM_ENABLE_IT(&hCAN_TIMx, TIM_IT_UPDATE);

      can_tasks_debug_printf("Task:%s free Stack:%d\n", pcTaskGetName(xTaskGetCurrentTaskHandle()),
                             uxTaskGetStackHighWaterMark(xTaskGetCurrentTaskHandle()));
    }
  }
}

/**
 ******************************************************************************
 * canSend
 * @brief  function mediator between Canfestival code and other code
 *         Received date from Canfestival code and insert it to Tx queue
 * @author  SWUST Tom
 ******************************************************************************
 */
unsigned char canSend(CAN_PORT notused, Message *m) {
  uint8_t i;
  static CanTxMsgTypeDef TxMsg;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  /*
  т.к. Canfestival не позволяет менять адрес устройства делаем это в драйвере,
  таким образом внутри функций Canfestival  cob_id всегда =0x00 
  */
  TxMsg.StdId = m->cob_id + can_id_delta;

  if (m->rtr)
    TxMsg.RTR = CAN_RTR_REMOTE;
  else
    TxMsg.RTR = CAN_RTR_DATA;

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

/*
 ******************************************************************************
 *     CAN_Transmit
 * @brief Received date in short Tx structure from CAN_Tx_Task convert it to
 * long TX HAL structure and sent to bus.
 * @param  - CAN handle Structure definition
 *         - short Tx structure
 * @return - standart HAL error code
 * @author     LVA
 ******************************************************************************
 */
uint32_t TxMailbox;
HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *_hcan, CanTxMsgTypeDef *TxMsg) {
  /* объявляем структуру в соответствии с HAL */
  CAN_TxHeaderTypeDef pTXHeader;
  pTXHeader.DLC                = (uint32_t)TxMsg->DLC;
  pTXHeader.ExtId              = 0U;
  pTXHeader.IDE                = CAN_ID_STD;
  pTXHeader.RTR                = (uint32_t)TxMsg->RTR;
  pTXHeader.StdId              = (uint32_t)TxMsg->StdId;
  pTXHeader.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(_hcan, &pTXHeader, TxMsg->Data, &TxMailbox) != HAL_OK) {
    can_debug_printf("%s -> HAL_CAN_AddTxMessage HAL_ERROR\n", __FUNCTION__);
    can_debug_printf("TxMailbox:%d\t", TxMailbox);
    /* uint32_t transmitmailbox = (READ_REG(hcan->Instance->TSR) & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos; */
    /* читаем состояние ящиков из регистров */
    can_debug_printf("transmitmailbox:%d\t%d|%d|%d = %d\n",
                     ((READ_REG(_hcan->Instance->TSR) & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos),
                     ((READ_REG(_hcan->Instance->TSR) & CAN_TSR_TME0) >> CAN_TSR_TME0_Pos),
                     ((READ_REG(_hcan->Instance->TSR) & CAN_TSR_TME1) >> CAN_TSR_TME1_Pos),
                     ((READ_REG(_hcan->Instance->TSR) & CAN_TSR_TME2) >> CAN_TSR_TME2_Pos),
                     ((READ_REG(_hcan->Instance->TSR) & CAN_TSR_TME) >> CAN_TSR_TME_Pos));
    osDelay(1000); /* для отладки */
    return HAL_ERROR;
  }
  can_debug_printf("StdId:0x%x\tRTR:0x%x\tDLC:0x%x\tData0x:%02x %02x %02x %02x %02x %02x %02x %02x\n", pTXHeader.StdId,
                   pTXHeader.RTR, pTXHeader.DLC, TxMsg->Data[0], TxMsg->Data[1], TxMsg->Data[2], TxMsg->Data[3],
                   TxMsg->Data[4], TxMsg->Data[5], TxMsg->Data[6], TxMsg->Data[7]);
  can_tasks_debug_printf("TX Task Free Stack:%d\n",
                         uxTaskGetStackHighWaterMark(xTaskGetCurrentTaskHandle())); /* CAN_Tx_Task_Header */
  // Debug_view_int(TxMailbox);

  return HAL_OK;
}

/*
 ******************************************************************************
 *     CANRcv_DateFromISR
 * @brief   Received date from Can bus over ISR and штыке it in queue
 *  RxMsg   short RX structure
 *
 * @author  SWUST Tom
 ******************************************************************************
 */
void CAN_Rcv_DateFromISR(CanRxMsgTypeDef *RxMsg) {
  static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  if (NULL != xCANRcvQueue) {
    if (xQueueSendToBackFromISR(xCANRcvQueue, RxMsg, &xHigherPriorityTaskWoken) != pdPASS) {
      can_debug_printf("Insert Date to Rx QuxQueue is not OK\n");
    }
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
  }
}

/*
 ******************************************************************************
 *     setTimer
 * @brief   Set the timer for the next alarm.
 * @author  SWUST Tom. LVA Editing for HAL support
 ******************************************************************************
 */
void setTimer(TIMEVAL value) { __HAL_TIM_SET_AUTORELOAD(&hCAN_TIMx, value); }

/*
 ******************************************************************************
 *     getElapsedTime
 * @brief Return the elapsed time to tell the Stack how much time is spent
 *        since last call.
 * @author  SWUST Tom. LVA Editing for HAL support
 ******************************************************************************
 */
TIMEVAL getElapsedTime(void) {
  uint32_t timer = __HAL_TIM_GET_COUNTER(&hCAN_TIMx); // Copy the value of the running timer
  if (timer < last_counter_val)
    timer += CAN_OPEN_TIM_PERIOD;
  TIMEVAL elapsed = timer - last_counter_val + elapsed_time;
  return elapsed;
}

/*
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

/*
 ******************************************************************************
 *         NMT_Slave_Node_Reset_Callback_Function
 * @brief Callback for NMT messages node reset. Reload microcontroller.
 *
 * @param  - Object dictionary structure from Canfestival
 * @return - none
 * @author     LVA
 ******************************************************************************
 */
void NMT_Slave_Node_Reset_Callback_Function(CO_Data *m) {
  can_debug_printf("NMT_command to Reload\n");
  HAL_NVIC_SystemReset();
}

/*
 ******************************************************************************
 *         state_change function
 * @brief CA callback called when node state changes
 * @param  - Object dictionary structure from Canfestival
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
* @param    - Object dictionary structure from Canfestival
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