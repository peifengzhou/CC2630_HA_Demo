/**************************************************************************************************
  Filename:       mac_radio_tx.c
  Revised:        $Date: 2015-02-17 14:17:44 -0800 (Tue, 17 Feb 2015) $
  Revision:       $Revision: 42683 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2006-2015 Texas Instruments Incorporated. All rights reserved.

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
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */

/* hal */
#include "hal_types.h"
#include "hal_mcu.h"

/* high-level */
#include "mac_spec.h"
#include "mac_pib.h"

/* exported low-level */
#include "mac_low_level.h"

/* mb */
#include "mb.h"

/* low-level specific */
#include "mac_radio_tx.h"
#include "mac_tx.h"
#include "mac_rx.h"
#include "mac_rx_onoff.h"

/* target specific */
#include "mac_radio_defs.h"

/* MAC CSMA */
#include "mac_csma.h"

/* debug */
#include "mac_assert.h"

/* Common ROM jump tables */
#include "R2F_CommonFlashJT.h"
   
/* TIMAC ROM jump tables */
#include "R2R_FlashJT.h"
#include "R2F_FlashJT.h"


/* ------------------------------------------------------------------------------------------------
 *                                         Defines
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                     Local Programs
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */


/**************************************************************************************************
 * @fn          macRadioTxPrepCsmaUnslotted
 *
 * @brief       Prepare CM0 for "Unslotted CSMA" transmit.  
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRadioTxPrepCsmaUnslotted(void)
{
  /* Setup MAC transmit command with non-slotted CSMA/CA */
  macSetupCsmaCaTxCmd( FALSE );
}


/**************************************************************************************************
 * @fn          macRadioTxPrepCsmaSlotted
 *
 * @brief       Prepare CM0 for "Slotted CSMA" transmit.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRadioTxPrepCsmaSlotted(void)
{
  /* Setup MAC transmit command with CSMA/CA */
  macSetupCsmaCaTxCmd( TRUE );
}


/**************************************************************************************************
 * @fn          macRadioTxGoCsma
 *
 * @brief       Run previously radio command for CSMA transmit.  Handles unslotted CSMA transmits.  
 *              When transmission has finished, this ISR will call macTxDoneCallback().
 *
 * @param       none
 *
 * @return      none 
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRadioTxGoCsma(void)
{
  macRxOn();

  /* Send radio command away */
  macRadioSendCsmaCaCmd();
}


/**************************************************************************************************
 * @fn          macRadioTxGoSlottedCsma
 *
 * @brief       Run previously radio command for CSMA transmit.  Handles slotted CSMA transmits.  
 *              When transmission has finished, this ISR will call macTxDoneCallback().
 *
 * @param       none
 *
 * @return      TRUE - cmd sent successfully, otherwise FALSE. 
 **************************************************************************************************
 */
MAC_INTERNAL_API bool macRadioTxGoSlottedCsma(void)
{
  /* Send radio command away only if there is at least one backoff available for TX
   * and the end time is greater than the start time. 
   */
  return ( macDataTxTimeAvailable() && 
           macCsmaCaCmd.endTime > macCsmaCaCmd.rfOpCmd.startTime && 
           macRadioSendCsmaCaCmd() == CMDSTA_DONE );
}


/**************************************************************************************************
 * @fn          macRadioTxPrepSlotted
 *
 * @brief       Prepare CM0 for "Slotted" (non-CSMA) transmit.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRadioTxPrepSlotted(void)
{
  /* Setup MAC transmit command with slotted mode */
  macSetupSlottedTxCmd();  
}


/**************************************************************************************************
 * @fn          macRadioTxGoSlotted
 *
 * @brief       Run previously loaded radio command for non-CSMA slotted transmit.   When 
 *              transmission has finished, an interrupt occurs and macTxDoneCallback() is called.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRadioTxGoSlotted(void)
{
  /* Always turn on RX before slotted TX */
  macRxOn();
  
  /* Send radio command away */
  MB_SendCommand( (uint32)&macTxCmd );
}


/**************************************************************************************************
 * @fn          macRadioTxPrepGreenPower
 *
 * @brief       Prepare radio for "Green Power" transmit.  Load radio TX command with GP parameters.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRadioTxPrepGreenPower(void)
{
  /* Get the Green Power TX command ready */
  macSetupGreenPowerTxCmd();
}


/**************************************************************************************************
 * @fn          macRadioTxGoGreenPower
 *
 * @brief       Run previously loaded CM0 radio command for Green Power transmit.  When 
 *              transmission has finished, macTxDoneCallback() is called.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRadioTxGoGreenPower(void)
{
  /* Send radio command away */
  MB_SendCommand( (uint32)&macTxCmd );
}


/**************************************************************************************************
 *                                  Compile Time Integrity Checks
 **************************************************************************************************
 */

#if (MAC_TX_TYPE_SLOTTED_CSMA != 0)
#error "WARNING!  This define value changed.  It was selected for optimum performance."
#endif


/**************************************************************************************************
*/
