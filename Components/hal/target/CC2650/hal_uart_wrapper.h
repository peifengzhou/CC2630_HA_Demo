/*******************************************************************************
  Filename:       hal_uart_wrapper.h
  Revised:        $Date: 2011-11-16 14:40:35 -0800 (Wed, 16 Nov 2011) $
  Revision:       $Revision: 28381 $

  Description:    This file contains the routines that wrap the Stellarisware
                  HAL drivers so that they look and feel like the CC25xx HAL
                  drivers.

Copyright 2009-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*******************************************************************************/

#ifndef HAL_UART_WRAPPER_H
#define HAL_UART_WRAPPER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * INCLUDES
 */

#include "hal_types.h"
#include "hal_uart.h"
#include <inc/hw_uart.h>
#include <inc/hw_memmap.h>

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */
#define UART_PORT_0                    UART0_BASE
#define UART_PORT_1                    UART0_BASE // UART1_BASE gone from memory map? 02162012
//
#define UART_FIFO_SIZE                 32
#define UART_HALF_FIFO_SIZE            (UART_FIFO_SIZE / 2)
//
#define UART_BAUD_9600                 9600
#define UART_BAUD_19200                19200
#define UART_BAUD_38400                38400
#define UART_BAUD_57600                57600
#define UART_BAUD_115200               115200

// Software FIFO Related Defines
#define SW_FIFO_SUCCESS                0
#define SW_FIFO_ERROR_FULL             1
#define SW_FIFO_ERROR_EMPTY            2
//
#define MAX_SW_FIFO_SIZE               128
#define NUM_UART_DEVICES               2

// UART Registers
#define UART0DR                        HWREG(UART0_BASE + UART_O_DR)
#define UART0SR                        HWREG(UART0_BASE + UART_O_RSR)
#define UART0ECR                       HWREG(UART0_BASE + UART_O_ECR)
#define UART0FR                        HWREG(UART0_BASE + UART_O_FR)
#define UART0IBRD                      HWREG(UART0_BASE + UART_O_IBRD)
#define UART0FBRD                      HWREG(UART0_BASE + UART_O_FBRD)
#define UART0ILCRH                     HWREG(UART0_BASE + UART_O_LCRH)
#define UART0CTL                       HWREG(UART0_BASE + UART_O_CTL)
#define UART0IFLS                      HWREG(UART0_BASE + UART_O_IFLS)
#define UART0IMSC                      HWREG(UART0_BASE + UART_O_IMSC)
#define UART0RIS                       HWREG(UART0_BASE + UART_O_RIS)
#define UART0MIS                       HWREG(UART0_BASE + UART_O_MIS)
#define UART0ICR                       HWREG(UART0_BASE + UART_O_ICR)
#define UART0DMACTL                    HWREG(UART0_BASE + UART_O_DMACTL)
#define UART0IPERID0                   HWREG(UART0_BASE + UART_O_PeriphID0)
#define UART0IPERID1                   HWREG(UART0_BASE + UART_O_PeriphID1)
#define UART0IPERID2                   HWREG(UART0_BASE + UART_O_PeriphID2)
#define UART0IPERID3                   HWREG(UART0_BASE + UART_O_PeriphID3)
#define UART0CELLD0                    HWREG(UART0_BASE + UART_O_PCellID0)
#define UART0CELLD1                    HWREG(UART0_BASE + UART_O_PCellID1)
#define UART0CELLD2                    HWREG(UART0_BASE + UART_O_PCellID2)
#define UART0CELLD3                    HWREG(UART0_BASE + UART_O_PCellID3)

/*******************************************************************************
* TYPEDEFS
*/
typedef struct
{
  uint8          head;
  uint8          tail;
  uint8          len;
  uint8          buf[ MAX_SW_FIFO_SIZE ];
  halUARTCBack_t callBackFunc;
} swFifo_t;

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

extern void  swFifoInit( swFifo_t *swFifo );
extern uint8 swFifoWriteRx( swFifo_t *swFifo, uint8 data );
extern uint8 swFifoReadRx( swFifo_t *swFifo, uint8 *data );

#ifdef __cplusplus
}
#endif

#endif /* HAL_UART_WRAPPER_H */
