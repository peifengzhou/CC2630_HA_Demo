/*******************************************************************************
  Filename:       mac_block.c
  Revised:        $Date: 2015-01-22 14:11:03 -0800 (Thu, 22 Jan 2015) $
  Revision:       $Revision: 41968 $

  Description:    This file contains the MAC CSMA features.

  Copyright 2015 Texas Instruments Incorporated. All rights reserved.

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

#include "mb.h"
#include "mac.h"

#ifdef FEATURE_PLATFORM_BLOCK

#include "hw_memmap.h"
#include "hw_fcfg1.h"
#include "flash.h"
#include "hal_defs.h"
#include "hal_mcu.h"
#include "mac_csma.h"


/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * CONSTANTS
 */

#define FACTORY_CFG_USER_ID_PROTOCOL_RF4CE           0x00002000
#define FACTORY_CFG_USER_ID_PROTOCOL_TIMAC_ZIGBEE    0x00004000
#define FACTORY_CFG_USER_ID_PROTOCOL_BLE_PROPRIETORY 0x00009000


/*******************************************************************************
 * LOCAL VARIABLES
 */


/*******************************************************************************
 * GLOBAL VARIABLES
 */


/*******************************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 msgData[28];


/*******************************************************************************
 * @fn          macValidateProtocolID
 *
 * @brief       This call is used to check if chip protocol ID is allowed 
 *              for TIMAC to run.
 *
 * input parameters
 *
 * @param       None
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
static void macValidateProtocolID( void )
{
  uint32 ui32ChipId = HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_USER_ID);
  
  if ( (ui32ChipId & FACTORY_CFG_USER_ID_PROTOCOL_TIMAC_ZIGBEE) == 0 )
  {
    uint32 ui32DeviceId = HWREG(FLASH_CFG_BASE + FCFG1_OFFSET + FCFG1_O_ICEPICK_DEVICE_ID);

    /* Platform lock, for any CSMA data transmission, replace it with 
     * Beacon Frame from 0x11CC to 0xAABB with Beacon payload of 
     * "WrongChip"+DEVICE_ID+USER_ID
     */
    msgData[0]  = 0x00;
    msgData[1]  = 0x80;
    msgData[2]  = 0x01;
    msgData[3]  = 0xCC;
    msgData[4]  = 0x11;
    msgData[5]  = 0xBB;
    msgData[6]  = 0xAA;
    msgData[7]  = 0xFF;
    msgData[8]  = 0xCF;
    msgData[9]  = 0x00;
    msgData[10] = 0x00;
    msgData[11] = 'W';
    msgData[12] = 'r';
    msgData[13] = 'o';
    msgData[14] = 'n';
    msgData[15] = 'g';
    msgData[16] = 'C';
    msgData[17] = 'h';
    msgData[18] = 'i';
    msgData[19] = 'p';
    msgData[20] = BREAK_UINT32( ui32DeviceId, 3 );
    msgData[21] = BREAK_UINT32( ui32DeviceId, 2 );
    msgData[22] = BREAK_UINT32( ui32DeviceId, 1 );
    msgData[23] = BREAK_UINT32( ui32DeviceId, 0 );
    msgData[24] = BREAK_UINT32( ui32ChipId, 3 );
    msgData[25] = BREAK_UINT32( ui32ChipId, 2 );
    msgData[26] = BREAK_UINT32( ui32ChipId, 1 );
    msgData[27] = BREAK_UINT32( ui32ChipId, 0 );
    
    /* Override payload */
    macTxCmd.payloadLen = sizeof(msgData);;
    macTxCmd.pPayload = msgData;
  }
}


/**************************************************************************************************
 * @fn          macRadioSendCsmaCaCmd
 *
 * @brief       Send CMD_IEEE_CSMA radio command to CM0. If the chip protocol ID
 *              check failed, this will still return CMDSTA_DONE status. The
 *              macValidateProtocolID() will override TX payload and reset after
 *              certain TX count.
 *
 * @param       none
 *
 * @return      A CMDSTA value, or CMDSTA_MAILBOX_BUSY. 
 **************************************************************************************************
 */
uint8 macRadioSendCsmaCaCmd(void)
{
  /* Validate chip protocol ID */
  macValidateProtocolID();

  /* Send radio command away */
  return ( MB_SendCommand( (uint32)&macCsmaCaCmd ) );
}

#else /* FEATURE_PLATFORM_BLOCK */

uint8 macRadioSendCsmaCaCmd(void)
{
  /* RF4CE should never turn on chip block feature */
  return( MB_SendCommand( (uint32)&macCsmaCaCmd ) );
}

#endif /* FEATURE_PLATFORM_BLOCK */

/*******************************************************************************
 */


