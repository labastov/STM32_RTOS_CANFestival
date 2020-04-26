/**
 ******************************************************************************
 * @file  : taskcan.h
 * @brief : This file starting task for CAN driver CANFestival of STM32
 * @version  V1.0.0
 * @date     24 April 2020
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASKCAN_H
#define __TASKCAN_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "can.h"
#include "cmsis_os.h"
// #include "debug.h"
#include "main.h"
#include "task.h"
// #include "nodeconfig.h"
#include "data.h"
// #include "can_driver.h"
// #include "Slave.h" // для Slave_bDeviceNodeId

 extern CAN_HandleTypeDef hcan;
 extern CO_Data Slave_Data;  // from Slave.c
 extern TIM_HandleTypeDef hCANOPEN_TIMx;

 extern void CAN_Start_Task(void *argument);

 HAL_StatusTypeDef CAN_Configuration(void);
 void CAN_Start_App(void);

#ifdef __cplusplus
}
#endif
#endif /*__TASKCAN_H */
