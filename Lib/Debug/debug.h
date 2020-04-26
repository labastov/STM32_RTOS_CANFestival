/* --------------------------------------------------------------------------
 * Portions Copyright � 2020 V.Labstov All rights
 *reserved.
 * --------------------------------------------------------------------------
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Project:      STM32-VSC-Debig
 * Title:        debug.h header file
 *
 * Version 0.01
 *    Initial Proposal Phase

 *---------------------------------------------------------------------------*/

#ifndef __DEBUG_H_
#define __DEBUG_H_
#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif //__cplusplus

#include "FreeRTOS.h"
#include "main.h"
#include "printf-stdarg.h" // for sprintf
#include <stdarg.h>        // for sprintf
#include <stdio.h>
#include <string.h> // for sprintf

#define DEBUG_PRINTF

void Debug_view_int(uint8_t);
void Debug_view_char(char);

void Debug_PrintChar(char c);
void Debug_PrintString(const char *s);

#define DEBUG_NAME_FUNCTION debug_printf("%s\n", __FUNCTION__)

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNU_C__ */

#ifdef DEBUG_SWO

/*********************************************************************
*
*       Defines for Cortex-M debug unit
*/
#define ITM_STIM_U32 (*(volatile unsigned int *)0xE0000000) // Stimulus Port Register word acces
#define ITM_STIM_U8 (*(volatile char *)0xE0000000)          // Stimulus Port Register byte acces
#define ITM_ENA (*(volatile unsigned int *)0xE0000E00)      // Trace Enable Ports Register
#define ITM_TCR (*(volatile unsigned int *)0xE0000E80)      // Trace control register

#endif // DEBUG_SWO

#ifdef DEBUG_PRINTF

#define printf debug_printf  

int fputc(int ch, FILE *f);

#endif // DEBUG_PRINTF

#endif //__DEBUG_H_