/*******************************************************************************
  Filename:       hal_uart_wrapper.c
  Revised:        $Date: 2011-12-01 18:45:35 -0800 (Thu, 01 Dec 2011) $
  Revision:       $Revision: 28536 $

  Description:    This file contains the routines that wrap the Stellarisware
                  HAL drivers so that they look and feel like the CC25xx HAL
                  drivers.

  Copyright 2011-2012 Texas Instruments Incorporated. All rights reserved.

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

/*******************************************************************************
 * INCLUDES
 */

#include <inc/hw_types.h>
#include <inc/hw_sysctl.h>
#include <driverlib/uart.h>
//#include "hal_uart.h"
#include <inc/hw_uart.h>
#include "hal_uart_wrapper.h"

// Prototypes
uint8 readErrCkUartData( void );

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */
#define BREAK_LOOP                     0
#define CONTINUE_LOOP                  1

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

swFifo_t rxUartBuf;
swFifo_t *pRxUARTBuf = &rxUartBuf;
//
swFifo_t txUartBuf;
swFifo_t *pTxUARTBuf = &txUartBuf;

/*******************************************************************************
 * GLOBAL VARIABLES
 */

const uint32 port2addr[] = { UART_PORT_0,
                             UART_PORT_1 };

const uint32 baud2baud[] = { UART_BAUD_9600,
                             UART_BAUD_19200,
                             UART_BAUD_38400,
                             UART_BAUD_57600,
                             UART_BAUD_115200 };

// CC25xx UART HAL API
//extern void HalUARTClose ( uint8 port );
//extern uint8 HalUARTIoctl ( uint8 port, uint8 cmd, halUARTIoctl_t *pIoctl );
//extern void HalUARTPoll( void );
//extern uint16 Hal_UART_TxBufLen ( uint8 port );
//extern void Hal_UART_FlowControlSet ( uint8 port, bool status );
//extern uint8 HalUART_HW_Init(uint8 port);
//extern void HalUARTSuspend(void);
//extern void HalUARTResume(void);

/*
** Software FIFO Application Programming Interface
*/

/*******************************************************************************
 * @fn          UART0 ISR
 *
 * @brief       This routine handles UART0 Interrupts.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void UART0_ISR( void )
{
  CLEAR_SLEEP_MODE();

  // check if a RX timeout occurred
  if ( UART0MIS & UART_IMSC_RTIM )
  {
    // copy data from RX FIFO until empty or RX buffer is full
    while ( !(UART0FR & UART_FR_RXFE) )
    {
      // check whether to continue or break from this loop
      if ( readErrCkUartData() == CONTINUE_LOOP )
      {
        continue;
      }
      else // BREAK_LOOP
      {
        break;
      }
    }

    // clear interrupt
    UART0ICR |= UART_IMSC_RTIM;
  }

  // check if a RX threshold reasched occurred
  if ( UART0MIS & UART_IMSC_RXIM )
  {
    uint8 i;

    // copy 1/2(FIFO SIZE) - 1 or RX buffer is full
    // Note: Leave at least one byte in the FIFO to prevent missing the
    //       the RX Timeout.
    for (i=0; i<UART_HALF_FIFO_SIZE; i++)
    {
      // check whether to continue or break from this loop
      if ( readErrCkUartData() == CONTINUE_LOOP )
      {
        continue;
      }
      else // BREAK_LOOP
      {
        break;
      }
    }

    // clear interrupt
    UART0ICR |= UART_IMSC_RXIM;
  }

  // TODO: RACE CONDITION CLEARING INTERRUPTS LIKE THIS?

  return;
}


/*******************************************************************************
 * @fn          readErrCkUartData ISR
 *
 * @brief       This routine reads and error checks the UART RX FIFO data. If
 *              there's an error (parity, framing, overrun, or break), the data
 *              is tossed. Otherwise, it is stored in the rx buffer.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
* @return       0=Break, 1=Continue
 */
uint8 readErrCkUartData( void )
{
  uint32 data;

  // check for parity/overrun/break/framing error
  data = UART0DR;

  // check error bits
  if ( data & (UART_DR_OE | UART_DR_BE | UART_DR_PE | UART_DR_FE) )
  {
    // clear the interrupt...
    UART0ICR |= (UART_DR_OE | UART_DR_BE | UART_DR_PE | UART_DR_FE);

    // ...and toss the data
    return( CONTINUE_LOOP );
  }

  if ( swFifoWriteRx( pRxUARTBuf, (uint8)data ) == SW_FIFO_ERROR_FULL )
  {
    // no more buffer available, so blow out of here
    return( BREAK_LOOP );
  }

  return( CONTINUE_LOOP );
}

/*******************************************************************************
 * @fn          swFifoInit
 *
 * @brief       This routine
 *
 * input parameters
 *
 * @param
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void swFifoInit( swFifo_t *swFifo )
{
  swFifo->head = 0;
  swFifo->tail = 0;
  swFifo->len  = 0;

#ifdef DEBUG
  {
    uint8 i;
    for (i=0; i< MAX_SW_FIFO_SIZE; i++)
    {
      swFifo->buf[i] = 0;
    }
  }
#endif // DEBUG

  return;
}


/*******************************************************************************
 * @fn          swFifoWriteRx
 *
 * @brief       This routine
 *
 * input parameters
 *
 * @param
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
uint8 swFifoWriteRx( swFifo_t *swFifo, uint8 data )
{
  //halIntState_t cs;

  // check for framing/overrun/break/parity error

  // check if the FIFO is full
  if ( swFifo->len == MAX_SW_FIFO_SIZE-1 )
  {
    return( SW_FIFO_ERROR_FULL );
  }
  else // not full, so...
  {
    //HAL_ENTER_CRITICAL_SECTION(cs);

    // ...okay to write the data
    swFifo->buf[ ++swFifo->tail ] = data;

    // check for wrap
    if ( swFifo->tail == MAX_SW_FIFO_SIZE-1 )
    {
      swFifo->tail = 0;
    }

    // update length
    swFifo->len++;

    //HAL_EXIT_CRITICAL_SECTION(cs);

    return( SW_FIFO_SUCCESS );
  }
}


/*******************************************************************************
 * @fn          swFifoReadRx
 *
 * @brief       This routine
 *
 * input parameters
 *
 * @param
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
uint8 swFifoReadRx( swFifo_t *swFifo, uint8 *data )
{
  //halIntState_t cs;

  // check if the FIFO is full
  if ( swFifo->len == 0 )
  {
    return( SW_FIFO_ERROR_EMPTY );
  }
  else // not empty, so...
  {
    //HAL_ENTER_CRITICAL_SECTION(cs);

    // ...okay to read the data
    *data = swFifo->buf[ ++swFifo->head ];

    // check for wrap
    if ( swFifo->head == MAX_SW_FIFO_SIZE-1 )
    {
      swFifo->head = 0;
    }

    // update length
    swFifo->len--;

    //HAL_EXIT_CRITICAL_SECTION(cs);

    return( SW_FIFO_SUCCESS );
  }
}


/*
** UART Application Programming Interface
*/

/*******************************************************************************
 * @fn          HalUARTInit
 *
 * @brief       This routine is used to as a wrapper to the CC25xx styled HAL
 *              UART API.
 *
 * input parameters
 *
 * @param
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void HalUARTInit( void )
{
  // TODO: IN CC25xx WORLD, IT IS ISR, DMA, OR USB. NOT SURE HERE YET.

  return;
}


/*******************************************************************************
 * @fn          HalUARTOpen
 *
 * @brief       This routine is used to as a wrapper to the CC25xx styled HAL
 *              UART API.
 *
 * input parameters
 *
 * @param
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
uint8 HalUARTOpen( uint8 port, halUARTCfg_t *config )
{
  swFifoInit( pTxUARTBuf );

  // TODO: ONLY PORT 0 USED HERE.
  // TODO: OTHER halUARTCfg_t PARAMETERS NOT USED.

  // mask all interrupts
  UART0IMSC = 0;

  // configure the UART
  // TODO: NEED TO MAP REST...
  UARTConfigSetExpClk( port2addr[ port ],
                       GET_MCU_CLOCK,
                       baud2baud[ config->baudRate ],
                       ( UART_CONFIG_WLEN_8   |
                         UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE ) );

  // save call back function
  pRxUARTBuf->callBackFunc = config->callBackFunc;

  // flush FIFOs
  UART0ECR &= ~UART_LCRH_FEN;

  // use the 32 byte FIFO
  UARTFIFOEnable( port2addr[ port ] );

  // set the RX FIFO threshold
  UARTFIFOLevelSet( port2addr[ port ],
                    UART_FIFO_TX7_8,
                    UART_FIFO_RX6_8 );

  // empty the RX FIFO
  while( !(UART0FR & UART_FR_RXFE) )
  {
    // toss the data
    UART0DR;
  }

  // clear all interrupts
  UART0ICR = 0xFFFFFFFF;

  // unmask
  UART0IMSC |= (UART_IMSC_RXIM | UART_IMSC_RTIM );

  // Start the UART
  UARTEnable( port2addr[ port ] );

  // enable UART0 interrupts
  IntEnable( INT_UART0 );

  return( 0 );
}


/*******************************************************************************
 * @fn          Hal_UART_RxBufLen
 *
 * @brief       This routine is used to as a wrapper to the CC25xx styled HAL
 *              UART API.
 *
 * input parameters
 *
 * @param
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
uint16 Hal_UART_RxBufLen( uint8 port )
{
  // ENTER CRITICAL SECTION

  // TODO: HANDLE PORT

  // EXIT CRITICAL SECTION

  return( pRxUARTBuf->len );
}


/*******************************************************************************
 * @fn          HalUARTPoll
 *
 * @brief       This routine is used to check if there is any received data
 *              to process.
 *
 * input parameters
 *
 * @param
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void HalUARTPoll( void )
{
  // check if there is any input data to be processed
  // TODO: HCI_UART_PORT HARDCODED HERE.
  if ( Hal_UART_RxBufLen( 0 ) != 0 )
  {
    // TODO: SAVE PORT NUMBER SOMEWHERE?
    //       EVNET NOT USED, SO JUST PASS ZERO.
    // TODO: USE CALLBACK STORED DURING HAL UART OPEN!
    // TODO: HCI_UART_PORT HARDCODED HERE.
    (pRxUARTBuf->callBackFunc)(0, 0);
  }

  return;
}


/*******************************************************************************
 * @fn          HalUARTRead
 *
 * @brief       Read a buffer from the UART.
 *
 *              Note: This routine is used to as a wrapper for the CC25xx styled
 *                    HAL UART API.
 * input parameters
 *
 * @param       port    - UART port
 * @param       pBuffer - pointer valid data buffer at least 'len' bytes in size
 * @param       len     - max length number of bytes to copy to 'buf'
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      length of buffer that was read
 */
uint16 HalUARTRead( uint8 port, uint8 *pBuffer, uint16 length )
{
  swFifo_t *swFifo = pRxUARTBuf;
  uint8    *buf    = pBuffer;
  uint8     i;

  // TODO: HANDLE PORT

  for (i=0; i<length; i++)
  {
    if ( swFifoReadRx( swFifo, buf++ ) != SW_FIFO_SUCCESS )
    {
      // read failed, so exit loop
      break;
    }
  }

  return( i );
}


/*******************************************************************************
 * @fn          HalUARTWrite
 *
 * @brief       Write a buffer to the UART.
 *
 *              Note: This routine is used to as a wrapper for the CC25xx styled
 *                    HAL UART API.
 *
 * input parameters
 *
 * @param       port    - UART port
 * @param       pBuffer - pointer to the buffer that will be written, not freed
 * @param       len     - length of buffer to write
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Length of the buffer that was sent.
 */
uint16 HalUARTWrite( uint8 port, uint8 *pBuffer, uint16 length )
{
  // loop while there are more characters to send
  // TODO: SETUP DMA FOR THIS! DOES DMA HANDLE FLOW CONTROL FROM TX FIFO FULL?!
  while( length-- )
  {
    // write the next character to the UART, using blocking
    UARTCharPut( port2addr[ port ], *pBuffer++ );
  }

  return( length );
}


/*******************************************************************************
 */


