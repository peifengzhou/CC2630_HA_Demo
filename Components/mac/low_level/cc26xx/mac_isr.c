/*******************************************************************************
  Filename:       mac_isr.c
  Revised:        $Date: 2015-02-17 14:17:44 -0800 (Tue, 17 Feb 2015) $
  Revision:       $Revision: 42683 $

  Description:    This file contains the Interrupt Service Routines (ISR)
                  for the Bluetooth Low Energy CC26xx RF Core Firmware
                  Specification.

  Copyright 2009-2015 Texas Instruments Incorporated. All rights reserved.

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

#include "hal_types.h"
#include "saddr.h"

#include "mb.h"
#include "mac.h"
#include "rfHal.h"
#include "mac_isr.h"
#include "mac_rx.h"
#include "mac_tx.h"
#include "mac_assert.h"

#ifdef DEBUG_SW_TRACE
#define DBG_ENABLE
#include "dbgid_sys_mst.h"
#endif /* DEBUG_SW_TRACE */


/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */


/**************************************************************************************************
 * @fn          cpe0IntCback
 *
 * @brief       This ISR Callback handles the Mailbox CPE 0 interrupt.  
 *
 * input parameters
 *
 * None         None
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void cpe0IntCback( void )
{
  uint32 cpeifg = MB_RFCPEIFG_REG;
  
  /* The radio CU has observed an unexpected error. A reset of the radio CPU is needed */
  if ( cpeifg & (uint32)MB_INTERNAL_ERROR )
  {
    MB_FwDebugDump();
    MAC_ASSERT(0);
  }

  /* cpe0IntCback is dedicated to FG command done and BG command done interrupts. */
  
  /* Tx_Frame and Tx_Ack interrupts are mutually exclusive */
  if ( cpeifg & MB_FG_COMMAND_DONE_INT )
  {
    while (MB_RFCPEIFG_REG & MB_FG_COMMAND_DONE_INT)
    {
      MB_RFCPEIFG_REG = ~MB_FG_COMMAND_DONE_INT;
    }
    macFgCmdDoneIsr();
  }

  /* This is to handle BG command done and RX errors, i.e. IEEE_ERROR_SYNTH_PROG */
  if ( cpeifg & MB_COMMAND_DONE_INT )
  {
    while (MB_RFCPEIFG_REG & MB_COMMAND_DONE_INT)
    {
      MB_RFCPEIFG_REG = ~MB_COMMAND_DONE_INT;
    }
    macBgCmdDoneIsr();
  }

  return;
}


/**************************************************************************************************
 * @fn          cpe1IntCback
 *
 * @brief       This ISR Callback handles the Mailbox CPE 1 interrupt.  
 *
 * input parameters
 *
 * None         None
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void cpe1IntCback( void )
{
  uint32 cpeifg = MB_RFCPEIFG_REG;
  
  /* This interrupt handler is mapped to Tx_Frame, Tx_Ack, Rx_Frame, and Rx_Nok 
   * interrupt.
   */
    
  /* Rx_Frame and Rx_Nok interrupts are mutually exclusive */
  if ( cpeifg & MB_RX_OK_INT )
  {
    /* A HW bug has been found that could prevent a CPE interrupt from being cleared. 
     * The problem occurs when the CM0 is setting an interrupt in the same clock cycle 
     * as when the CM3 is trying to clear an interrupt. The software workaround that 
     * we have found to work is to just clear twice.
     */
    while (MB_RFCPEIFG_REG & MB_RX_OK_INT)
    {
      MB_RFCPEIFG_REG = ~MB_RX_OK_INT;
    }
    DBG_PRINT0(DBGSYS, "MB_RX_OK_INT - macRxFrameIsr()");
    macRxFrameIsr();
  }  
  else if ( cpeifg & MB_RX_NOK_INT )
  {
    while (MB_RFCPEIFG_REG & MB_RX_NOK_INT)
    {
      MB_RFCPEIFG_REG = ~MB_RX_NOK_INT;
    }
    DBG_PRINT0(DBGSYS, "MB_RX_NOK_INT - macRxNokIsr()");
    macRxNokIsr();
  }
  else if ( cpeifg & MB_RX_BUF_FULL_INT )
  {
    while (MB_RFCPEIFG_REG & MB_RX_BUF_FULL_INT)
    {
      MB_RFCPEIFG_REG = ~MB_RX_BUF_FULL_INT;
    }
    
    /* This will attempt to allocate another OSAL buffer for the last RX frame
     * which was stuck in the macRxDataEntryQueue. 
     */
    DBG_PRINT0(DBGSYS, "MB_RX_BUF_FULL_INT - macRxFrameIsr()");
    macRxFrameIsr();
  }
    /* Tx_Frame and Tx_Ack interrupts are mutually exclusive */
  else if ( cpeifg & MB_TX_DONE_INT )
  {
    while (MB_RFCPEIFG_REG & MB_TX_DONE_INT)
    {
      MB_RFCPEIFG_REG = ~MB_TX_DONE_INT;
    }
    DBG_PRINT0(DBGSYS, "MB_TX_DONE_INT - macTxFrameIsr()");
    macTxFrameIsr();
  }

  else if ( cpeifg & MB_TX_ACK_INT )
  {
    while (MB_RFCPEIFG_REG & MB_TX_ACK_INT)
    {
      MB_RFCPEIFG_REG = ~MB_TX_ACK_INT;
    }
    DBG_PRINT0(DBGSYS, "MB_TX_ACK_INT - macTxAckIsr()");
    macTxAckIsr();
  }
  else if ( cpeifg & MB_RX_IGNORED_INT )
  {
    while (MB_RFCPEIFG_REG & MB_RX_IGNORED_INT)
    {
      MB_RFCPEIFG_REG = ~MB_RX_IGNORED_INT;
    }
    DBG_PRINT0(DBGSYS, "MB_RX_IGNORED_INT - No Action!");
  }  
  
  return;
}


/**************************************************************************************************
 * @fn          hwIntCback
 *
 * @brief       This ISR Callback handles the Mailbox HW interrupt.  
 *              Public function defined in Mailbox.h
 *
 * input parameters
 *
 * None         None
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void hwIntCback( void )
{
  uint8  i;
  uint32 hwInt;
  uint32 ratChanIntMask = ( RAT_CHAN_0_IRQ |
                            RAT_CHAN_1_IRQ |
                            RAT_CHAN_2_IRQ |
                            RAT_CHAN_3_IRQ |
                            RAT_CHAN_4_IRQ |
                            RAT_CHAN_5_IRQ |
                            RAT_CHAN_6_IRQ |
                            RAT_CHAN_7_IRQ );

  /* read the hardware interrupts */
  hwInt = MB_RFHWIFG_REG;

  /* mask out only those we enabled */
  hwInt &= MB_RFHWIEN_REG;

  /* clear our pending hardware interrupts */
  MB_RFHWIFG_REG &= ~hwInt;

  /* check if a RAT Channel interrupt is pending */
  if ( hwInt & ratChanIntMask )
  {
    /* check for all available RAT channels */
    for (i=0; i<MAX_NUM_SW_RAT_CHANS; i++)
    {
      /* check if RAT channel is free */
      if ( (ratChanInfo[i].ratChanStat == RAT_CHAN_BUSY) && (hwInt & (1L << (ratChanInfo[i].ratChanNum + 12))) )
      {
        /* call callback for this RAT channel */
        (ratChanInfo[i].ratChanCBack)();
      }
    }
  }
  return;
}


/*******************************************************************************
 */


