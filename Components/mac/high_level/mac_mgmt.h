/**************************************************************************************************
  Filename:       mac_mgmt.h
  Revised:        $Date: 2014-05-09 18:50:29 -0700 (Fri, 09 May 2014) $
  Revision:       $Revision: 38492 $

  Description:    Internal interface file for the MAC management module.


  Copyright 2005-2013 Texas Instruments Incorporated. All rights reserved.

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

#ifndef MAC_MGMT_H
#define MAC_MGMT_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "mac_low_level.h"
#include "mac_high_level.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* Mask this value with the srcAddrMode when source PAN ID is broadcast */
#define MAC_SRC_PAN_ID_BROADCAST    0x80

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* Management info struct */
typedef struct
{
  bool              networkStarted;
} macMgmt_t;

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

/* MAC mangement info */
extern macMgmt_t macMgmt;

/* Action sets */
extern macAction_t macMgmtAction1[7];
extern macAction_t macMgmtAction2[5];

/* ------------------------------------------------------------------------------------------------
 *                                           Function Prototypes
 * ------------------------------------------------------------------------------------------------
 */

/* Utility functions */
MAC_INTERNAL_API void macGetCoordAddress(sAddr_t *pAddr);
MAC_INTERNAL_API uint8 macGetMyAddrMode(void);
MAC_INTERNAL_API bool macDestAddrCmp(uint8 *p1, uint8 *p2);
MAC_INTERNAL_API bool macDestSAddrCmp(sAddr_t *pAddr, uint16 panId, uint8 *p);
MAC_INTERNAL_API bool macCoordAddrCmp(sAddr_t *pAddr);
MAC_INTERNAL_API bool macCoordDestAddrCmp(uint8 *p);
MAC_INTERNAL_API uint8 macBuildHeader(macTx_t *pBuf, uint8 srcAddrMode, sAddr_t *pDstAddr, uint16 dstPanId);
MAC_INTERNAL_API uint8 macBuildDisassociateReq(macEvent_t *pEvent);
MAC_INTERNAL_API uint8 macBuildCommonReq(uint8 cmd, uint8 srcAddrMode, sAddr_t *pDstAddr,
                                        uint16 dstPanId, uint16 txOptions, macSec_t *sec);
MAC_INTERNAL_API uint8 macBuildEnhanceBeaconReq(macTx_t *pMsg, sAddr_t *pDstAddr, macSec_t *sec,
                                               uint8 requestField, uint8 linkQuality,
                                               uint8 percentFilter);
MAC_INTERNAL_API void macBuildBeaconNotifyInd(macMlmeBeaconNotifyInd_t *pNotifyInd, macEvent_t *pEvent);
MAC_INTERNAL_API macTx_t *macBuildBeacon(uint8 beaconOrder, uint8 superframeOrder, bool battLifeExt);
MAC_INTERNAL_API void macSecCpy(macSec_t *pDst, macSec_t *pSrc);
MAC_INTERNAL_API void macConflictSyncLossInd(void);

/* Action functions */
MAC_INTERNAL_API void macNoAction(macEvent_t *pEvent);
MAC_INTERNAL_API void macApiUnsupported(macEvent_t *pEvent);
MAC_INTERNAL_API void macDefaultAction(macEvent_t *pEvent);
MAC_INTERNAL_API void macApiBadState(macEvent_t *pEvent);
MAC_INTERNAL_API void macApiPending(macEvent_t *pEvent);
MAC_INTERNAL_API void macCommStatusInd(macEvent_t *pEvent);
MAC_INTERNAL_API void macApiDisassociateReq(macEvent_t *pEvent);
MAC_INTERNAL_API void macDisassocComplete(macEvent_t *pEvent);
MAC_INTERNAL_API void macRxDisassoc(macEvent_t *pEvent);
MAC_INTERNAL_API void macApiRxEnableReq(macEvent_t *pEvent);
MAC_INTERNAL_API void macRxBeacon(macEvent_t *pEvent);

/**************************************************************************************************
*/

#endif /* MAC_MGMT_H */

