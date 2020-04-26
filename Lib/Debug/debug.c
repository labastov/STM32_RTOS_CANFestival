/**
 ******************************************************************************
 * @file  : debug.c
 * @brief : output debug information to SDO ок UART for STM32
 * @version  V1.0.0 Initial Study Phase
 * @date     24 April 2020
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */

#include "debug.h"

/*********************************************************************
*
*       Debug_PrintChar()
*
*/

#ifdef DEBUG_SWO
// #warning definitd DEBUG_SWO

void Debug_PrintChar(char c) {
  //
  // Check if ITM_TCR.ITMENA is set
  //
  if ((ITM_TCR & 1) == 0) {
    return;
  }
  //
  // Check if stimulus port is enabled
  //
  if ((ITM_ENA & 1) == 0) {
    return;
  }
  //
  // Wait until STIMx is ready,
  // then send data
  //
  while ((ITM_STIM_U8 & 1) == 0)
    ;
  ITM_STIM_U8 = c;
}

/*
 ********************************************************************
 *
 *       Debug_PrintString()
 *
 * Function description
 *   Print a string via Debug_PrintChar.
 **********************************************************************
 */
void Debug_PrintString(const char *s) {
  //
  // Print out character per character
  //
  while (*s) {
    Debug_PrintChar(*s++);
  }
}
#endif

#ifdef DEBUG_UART

#warning definitd DEBUG_UART

void Debug_PrintChar(char c) {

  if (c == 0x0A) {
    uint8_t a = ((uint8_t)'\r');
    HAL_UART_Transmit(&DEBUG_UART, &a, 1, 100);
  }

  HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)&c, 1, 100);
}
#endif // DEBUG_UART

#if !defined DEBUG_UART && !defined DEBUG_SWO

void Debug_PrintString(const char *s) { ; }
void Debug_PrintChar(char c) { ; }

#endif // DEBUG_SWO

#if defined DEBUG_UART && defined DEBUG_SWO

#error DEFINED DEBUG_UART ADN DEBUG_SWO. Switch off any one

#endif // DEBUG_SWO

#ifdef DEBUG_PRINTF

int fputc(int ch, FILE *f) {
  Debug_PrintChar((uint8_t)ch);

  return ch;
}
#endif // DEBUG_PRINTF

/*
 ********************************************************************
 *
 *       Debug_view_int()
 * Output debug information on 7-segments indicator internal count from 0 to F
 * if digit below F indicator switch off
 * DOT segment used for char indication
 **********************************************************************
 */
/*
// if not Defined in Cube MX
#define A_GPIO_Port
#define B_GPIO_Port
#define C_GPIO_Port
#define D_GPIO_Port
#define F_GPIO_Port
#define E_GPIO_Port
#define G_GPIO_Port
#define DP_GPIO_Port
#define A_Pin
#define B_Pin
#define C_Pin
#define D_Pin
#define E_Pin
#define F_Pin
#define G_Pin
#define DP_Pin
*/


void Debug_view_int(uint8_t digit) {

  switch (digit) {
  case 0x00:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x01:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x02:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x03:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x04:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x05:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x06:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x07:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x08:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x09:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x0A:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x0B:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x0C:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x0D:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x0E:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  case 0x0F:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  default:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  }
}

/*
 ********************************************************************
 *
 *       Debug_view_char()
 * Output debug char `E` or `. ` on 7-segments indicator
 *
 **********************************************************************
 */
void Debug_view_char(char dchar) {

  switch (dchar) {
  case 'E':
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_RESET);
    break;
  case '.':
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_RESET);
    break;
  default:
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(F_GPIO_Port, F_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DP_GPIO_Port, DP_Pin, GPIO_PIN_SET);
    break;
  }
  }
  