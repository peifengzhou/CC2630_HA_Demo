/**************************************************************************************************
  Filename:       mac_coord_PATCH.c
  Revised:        $Date: 2014-11-06 11:03:55 -0800 (Thu, 06 Nov 2014) $
  Revision:       $Revision: 41021 $

  Description:    This module implements MAC device management procedures for a coordinator.


  Copyright 2005-2014 Texas Instruments Incorporated. All rights reserved.

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
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_mcu.h"
#include "mac_api.h"
#include "mac_spec.h"
#include "mac_main.h"
#include "mac_timer.h"
#include "mac_mgmt.h"
#include "mac_data.h"
#include "mac_pib.h"
#include "mac_coord.h"
#include "mac_security.h"
#include "mac_low_level.h"
#include "OSAL.h"

#include "R2R_FlashJT.h"
#if defined (CC26XX)
#include "R2F_FlashJT.h"
#endif /* CC26XX */

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

uint8 macPendAddrLen(uint8 *maxPending);
uint8 *macBuildPendAddr(uint8 *p, uint8 maxPending, bool *pBroadcast);
void macIndirectMark(macTx_t *pMsg);
bool macIndirectSend(sAddr_t *pAddr, uint16 panId);
void macIndirectTxFrame(macTx_t *pMsg);
void macIndirectRequeueFrame(macTx_t *pMsg);

#ifdef FEATURE_EXTENDED_CHAN29
/**************************************************************************************************
 * @fn          macStartBegin
 *
 * @brief       This function handles begins the processing of a start request.
 *              Here the validity of the parameters are checked and the pStartReq
 *              is allocated.
 *
 * input parameters
 *
 * @param       pEvent - Pointer to event data.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
uint8 macStartBegin(macEvent_t *pEvent)
{
  uint8 status = MAC_SUCCESS;

  /* check parameters */
  MAC_PARAM_STATUS((pEvent->api.mac.startReq.logicalChannel >= MAC_CHAN_BEG) &&
                   (pEvent->api.mac.startReq.logicalChannel <= MAC_CHAN_29), status);
  MAC_PARAM_STATUS(pEvent->api.mac.startReq.beaconOrder <= MAC_BO_NON_BEACON, status);
  MAC_PARAM_STATUS((pEvent->api.mac.startReq.beaconOrder == MAC_BO_NON_BEACON)
                   || (macMain.featureMask & MAC_FEAT_BEACON_COORD_MASK), status);
  MAC_PARAM_STATUS((pEvent->api.mac.startReq.superframeOrder <= pEvent->api.mac.startReq.beaconOrder)
                   || (pEvent->api.mac.startReq.superframeOrder == MAC_SO_NONE), status);

  /* verify we have a short address */
  if (pMacPib->shortAddress == MAC_SHORT_ADDR_NONE)
  {
    status = MAC_NO_SHORT_ADDRESS;
  }

  /* allocate buffer for start parameters */
  if ((macCoord.pStartReq = osal_mem_alloc(sizeof(macMlmeStartReq_t))) != NULL)
  {
    osal_memcpy(macCoord.pStartReq, &pEvent->api.mac.startReq, sizeof(macMlmeStartReq_t));
  }
  else
  {
    status = MAC_NO_RESOURCES;
  }

  return status;
}
#endif /* FEATURE_EXTENDED_CHAN29 */

/**************************************************************************************************
 * @fn          macRxDataReq
 *
 * @brief       This function processes a data request command frame.  If the frame was ack'ed
 *              with pending, the indirect frame is searched and frame sent to the requester.
 *              If no frame is found, a zero length data frame is sent. The MAC_MLME_POLL_IND
 *              is sent regardless of ack pending status.  The upper layer can use the noRsp 
 *              flag to determine whether to respond to the poll indication with  MAC_McpsDataReq() 
 *              or not. If noRsp is set to FALSE, the upp layer can respond to the poll 
 *              indication with MAC_McpsDataReq().
 *
 *
 * input parameters
 *
 * @param       pEvent - Pointer to event data.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
MAC_INTERNAL_API void macRxDataReq(macEvent_t *pEvent)
{
  macMlmePollInd_t pollInd;
  macSec_t         sec;
  uint8            noRsp = TRUE;

  /* if frame was ack'ed with pending */
  if (pEvent->rx.internal.flags & MAC_RX_FLAG_ACK_PENDING)
  {
    /* find and send requested frame */
    if (!MAP_macIndirectSend(&pEvent->rx.mac.srcAddr, pEvent->rx.mac.srcPanId))
    {
      if (macCfg.appPendingQueue == TRUE)
      {
        /* Short address matches, set noRsp flag to FALSE so the upper layer
         * can respond to the poll indication with MAC_McpsDataReq().
         */
        noRsp = FALSE;
      }
      else
      {
        /* copy the security settings for zero length data */
        osal_memcpy(&sec, &pEvent->rx.sec , sizeof(macSec_t));

        /* if not found send empty data frame */
        MAP_macBuildCommonReq(MAC_INTERNAL_ZERO_DATA, MAP_macGetMyAddrMode(), &pEvent->rx.mac.srcAddr,
                              pEvent->rx.mac.srcPanId, MAC_TXOPTION_NO_RETRANS | MAC_TXOPTION_NO_CNF, &sec);
      }
    }
  }

  /* Send MAC_MLME_POLL_IND regardless of ack pending status */
  pollInd.hdr.event    = MAC_MLME_POLL_IND;
  MAP_sAddrCpy(&pollInd.srcAddr, &pEvent->rx.mac.srcAddr);
  pollInd.srcPanId     = pEvent->rx.mac.srcPanId;
  pollInd.noRsp        = noRsp;
  MAC_CbackEvent((macCbackEvent_t *) &pollInd);
}

