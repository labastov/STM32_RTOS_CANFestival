/* 
функция автономной проверки отправки CAN пакетов 
в проекте не учавстсвует, при необходимости 
вствляется после инициализации всей переферии.
*/

#include "main.h"
#include "debug.h"
#include "can.h"

void CanTestTX(CAN_HandleTypeDef hcan) {

  uint32_t TxMailbox;
  debug_printf("test CAN TX packets\n");

  CAN_FilterTypeDef CanFilter;  // declare CAN filter structure
  CAN_FilterTypeDef CanFilter2; // declare CAN filter structure

  //=========================== filter 1
  //======================================
  // адресс совпадает с адресом в отрпавляемых пакетах
  // CanFilter.FilterIdHigh = pHeader.StdId << 5;
  // CanFilter.FilterIdHigh = 0x0000; // старшая и младшая части фильтра
  CanFilter.FilterIdHigh = 0x244 << 5; // the ID that the filter looks

  CanFilter.FilterIdLow = 0x0000;

  // маска разрешает прием только своих пакетов
  // CanFilter.FilterMaskIdHigh = 0x0000; // части маски не
  CanFilter.FilterMaskIdHigh = 0x0111 << 5; // части маски

  // маска разрешает приме любых пакетов
  CanFilter.FilterMaskIdLow = 0x0000;

  // номер буфера для применения в нем фильтрации
  CanFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;

  CanFilter.FilterBank = 0;
  /* !< Specifies the filter bank which will be initialized.
                  For single CAN instance(14 dedicated filter banks),
                  this parameter must be a number between Min_Data = 0 and
            Max_Data = 13.
                  For dual CAN instances(28 filter banks shared),
                  this parameter must be a number between Min_Data = 0 and
            Max_Data = 27. */

  CanFilter.FilterMode = CAN_FILTERMODE_IDMASK; // режим работы фильтра

  CanFilter.FilterScale = CAN_FILTERSCALE_32BIT; // разрядность

  CanFilter.FilterActivation = CAN_FILTER_ENABLE;
  // for

  if (HAL_CAN_ConfigFilter(&hcan, &CanFilter)) // configure CAN filter
    debug_printf("HAL_CAN_ConfigFilter0 ERROR\n");

  //=========================== filter 2 ======================================
  CanFilter2.FilterMode = CAN_FILTERMODE_IDMASK;  // режим работы фильтра
  CanFilter2.FilterScale = CAN_FILTERSCALE_32BIT; // разрядность
  // sFilterConfig.FilterIdHigh = 0x0000; // старшая и младшая части фильтра

  // адресс совпадает с адресом в отрпавляемых пакетах
  CanFilter2.FilterIdHigh = 0x07FF << 5;

  // маска разрешает прием только своих пакетов
  CanFilter2.FilterMaskIdHigh = 0x0111 << 5; // части маски

  // маска разрешает приме любых пакетов
  // CanFilter2.FilterMaskIdHigh = 0x0000 << 5; // части маски не

  // CanFilter2.FilterIdHigh = 0x245 << 5; // the ID that the filter looks
  // for
  CanFilter2.FilterIdLow = 0x0000;
  CanFilter2.FilterMaskIdLow = 0x0000;
  CanFilter2.FilterActivation = CAN_FILTER_ENABLE;

  // номер буфера для применения в нем фильтрации
  CanFilter2.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  CanFilter2.FilterBank = 1;

  if (HAL_CAN_ConfigFilter(&hcan, &CanFilter2)) // configure CAN filter
    debug_printf("HAL_CAN_ConfigFilter2 ERROR\n");

  // start CAN
  HAL_CAN_Start(&hcan);
  debug_printf("HAL_CAN_Start ERROR state:%s\n", hcan.State);

  // enable interrupts
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING))
    debug_printf("HAL_CAN_ActivateNotification ERROR\n");


    CAN_TxHeaderTypeDef pTXHeaderDEBUG; // надо бы его сделлать локлаьным.
    uint8_t Data[8];
    uint32_t TxMailbox;

    pTXHeaderDEBUG.DLC = 8;            // give message size of 1 byte
    pTXHeaderDEBUG.IDE = CAN_ID_STD;   // set identifier to standard
    pTXHeaderDEBUG.RTR = CAN_RTR_DATA; // set data type to remote transmission request?
    pTXHeaderDEBUG.StdId = 0x244;      // define a standard identifier, used for message
    for (uint8_t i = 0; i < pTXHeaderDEBUG.DLC; i++) {
      Data[i] = 0xAA;
    }
    uint8_t r;

    for (;;) {
      r = HAL_CAN_AddTxMessage(&hcan, &pTXHeaderDEBUG, Data, &TxMailbox);
      if (r != HAL_OK) {
        debug_printf("%s -> HAL_CAN_AddTxMessage return %d\n", r);
      }
      HAL_Delay(1000);
    }
    /*
    */

}