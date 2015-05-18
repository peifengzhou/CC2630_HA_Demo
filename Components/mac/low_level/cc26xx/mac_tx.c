/**************************************************************************************************
  Filename:       mac_tx.c
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
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

/* hal */
#include "hal_types.h"
#include "hal_defs.h"
#include "hal_mcu.h"
#include "hal_mac_cfg.h"

/* high-level */
#include "mac_spec.h"
#include "mac_pib.h"

/* exported low-level */
#include "mb.h"
#include "rfHal.h"
#include "mac_low_level.h"
#include "mac.h"

/* low-level specific */
#include "mac_tx.h"
#include "mac_backoff_timer.h"
#include "mac_rx.h"
#include "mac_rx_onoff.h"
#include "mac_radio.h"
#include "mac_radio_tx.h"

/* target specific */
#include "mac_radio_defs.h"

/* debug */
#include "mac_assert.h"

/* Common ROM jump tables */
#include "R2F_CommonFlashJT.h"

/* TIMAC ROM jump tables */
#include "R2R_FlashJT.h"
#include "R2F_FlashJT.h"

#ifdef DEBUG_SW_TRACE
#define DBG_ENABLE
#include "dbgid_sys_mst.h"
#endif /* DEBUG_SW_TRACE */

/* include rssi */
#include "Macs.h"

/* ------------------------------------------------------------------------------------------------
 *                                            Defines
 * ------------------------------------------------------------------------------------------------
 */
#define MFR_LEN                   MAC_FCS_FIELD_LEN
#define PREPENDED_BYTE_LEN        1
/* value for promiscuous off, must not conflict with other mode variants from separate include files */
#define PROMISCUOUS_MODE_OFF  0x00

/* This is passed from App */
#define WIFI_RX_ACTIVE            (((macUserCfg_t *)macRadioConfig)->rfWifiRxActive)

/* ------------------------------------------------------------------------------------------------
 *                                         Global Constants
 * ------------------------------------------------------------------------------------------------
 */

/*
 *  This is the time, in backoffs, required to set up a slotted transmit.
 *  It is exported to high level so that code can schedule enough time
 *  for slotted transmits.
 *
 *  A default is provided if a value is not specified.  If the default
 *  is not appropriate, a #define should be added within hal_mac_cfg.h.
 */
#ifndef HAL_MAC_TX_SLOTTED_DELAY
#define HAL_MAC_TX_SLOTTED_DELAY    3
#endif
uint8 const macTxSlottedDelay = HAL_MAC_TX_SLOTTED_DELAY;


/* ------------------------------------------------------------------------------------------------
 *                                         Global Variables
 * ------------------------------------------------------------------------------------------------
 */
uint8 macTxActive;
uint8 macTxType;
uint8 macTxBe;
uint8 macTxCsmaBackoffDelay;
uint8 macTxGpInterframeDelay;
extern uint8  rxPromiscuousMode;


/* ------------------------------------------------------------------------------------------------
 *                                         Local Variables
 * ------------------------------------------------------------------------------------------------
 */
static uint8 txSeqn;
static uint8 txAckReq;
static uint8 txRetransmitFlag;


/* ------------------------------------------------------------------------------------------------
 *                                         Local Prototypes
 * ------------------------------------------------------------------------------------------------
 */
static void txCsmaPrep(void);
static void txGreenPowerPrep(void);
static void txGo(void);
static void txCsmaGo(void);
static void txComplete(uint8 status);


/**************************************************************************************************
 * @fn          macTxInit
 *
 * @brief       Initialize variables for tx module. OSAL must be initialized.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macTxInit(void)
{
  macTxActive      = MAC_TX_ACTIVE_NO_ACTIVITY;
  txRetransmitFlag = 0;
  
  /* Power management state may change. Hence, vote. */
  macPwrVote();
}


/**************************************************************************************************
 * @fn          macTxFrame
 *
 * @brief       Transmit the frame pointed to by pMacDataTx with the specified type.
 *              NOTE! It is not legal to call this function from interrupt context.
 *
 * @param       txType - type of transmit
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macTxFrame(uint8 txType)
{
  MAC_ASSERT(!macTxActive);            /* transmit on top of transmit */

  /* mark transmit as active */
  macTxActive = MAC_TX_ACTIVE_INITIALIZE;

  /*
   *  The MAC will not enter sleep mode if there is an active transmit.  However, if macSleep() is
   *  ever called from interrupt context, it possible to enter sleep state after a transmit is
   *  intiated but before macTxActive is set.  To recover from this, the transmit must be aborted
   *  and proper notificiation given to high-level.
   */

  /* save transmit type */
  macTxType = txType;

  /*-------------------------------------------------------------------------------
   *  Prepare for transmit.
   */
  if (macTxType == MAC_TX_TYPE_SLOTTED)
  {
    MAC_RADIO_TX_PREP_SLOTTED();
  }

#ifdef FEATURE_GREEN_POWER
  else if (macTxType == MAC_TX_TYPE_GREEN_POWER)
  {
    txGreenPowerPrep();
  }
#endif /* #ifdef FEATURE_GREEN_POWER */
  
  else
  {
    MAC_ASSERT((macTxType == MAC_TX_TYPE_SLOTTED_CSMA) || (macTxType == MAC_TX_TYPE_UNSLOTTED_CSMA));
    macTxBe = (pMacDataTx->internal.txOptions & MAC_TXOPTION_ALT_BE) ? pMacPib->altBe : pMacPib->minBe;
  
    if ((macTxType == MAC_TX_TYPE_SLOTTED_CSMA) && (pMacPib->battLifeExt))
    {
      macTxBe = MIN(2, macTxBe);
    }
  
    txCsmaPrep();
  }

  /*-------------------------------------------------------------------------------
   *  Load transmit FIFO unless this is a retransmit.  No need to write
   *  the FIFO again in that case.
   */
  if (!txRetransmitFlag)
  {
    uint8 * p;
    uint8   lenMhrMsdu;

    MAC_ASSERT(pMacDataTx != NULL); /* must have data to transmit */

    /* save needed parameters */
    txAckReq = MAC_ACK_REQUEST(pMacDataTx->msdu.p);
    txSeqn   = MAC_SEQ_NUMBER(pMacDataTx->msdu.p);
    
    if ( txAckReq )
    {
      /* ACK is requested */
      macSetupRxAckCmd();
      
      /* Set the ACK sequence number to the command structure */
      macSetupRxAckSeqNo( txSeqn );
    }
    
    /* set length of frame (note: use of term msdu is a misnomer, here it's actually mhr + msdu) */
    lenMhrMsdu = pMacDataTx->msdu.len;

    /* calling code guarantees an unused prepended byte but not used in CC26xx */
    p = pMacDataTx->msdu.p;

    /*
     *  Flush the TX FIFO.  This is necessary in case the previous transmit was never
     *  actually sent (e.g. CSMA failed without strobing TXON).  If bytes are written to
     *  the FIFO but not transmitted, they remain in the FIFO to be transmitted whenever
     *  a strobe of TXON does happen.
     */
    MAC_RADIO_FLUSH_TX_FIFO();

    /* Set TX payload length */
    macTxCmd.payloadLen = lenMhrMsdu;

    /* Write payload */
    macTxCmd.pPayload = p;
  }

  /*-------------------------------------------------------------------------------
   *  If not receiving, start the transmit.  If receive is active
   *  queue up the transmit.  Transmit slotted type frames regardlessly,
   *  i.e. Beacon frame in beacon mode.
   *
   *  Critical sections around the state change prevents any sort of race condition
   *  with  macTxStartQueuedFrame().  This guarantees function txGo() will only be
   *  called once.
   */
  {
    halIntState_t  s;

    HAL_ENTER_CRITICAL_SECTION(s);
    if ((macTxType == MAC_TX_TYPE_SLOTTED) || (!macRxActive && !macRxOutgoingAckFlag))
    {
      macTxActive = MAC_TX_ACTIVE_GO;
      HAL_EXIT_CRITICAL_SECTION(s);
      DBG_PRINT2(DBGSYS, "MAC_TX_ACTIVE_GO, macTxType=%i, LEN=%i", macTxType, macTxCmd.payloadLen);
      DBG_PRINT4(DBGSYS, "DATA[0..3]=0x%X, 0x%X, 0x%X, 0x%X", macTxCmd.pPayload[0], macTxCmd.pPayload[1], macTxCmd.pPayload[2], macTxCmd.pPayload[3]);
      DBG_PRINT4(DBGSYS, "DATA[4..7]=0x%X, 0x%X, 0x%X, 0x%X", macTxCmd.pPayload[4], macTxCmd.pPayload[5], macTxCmd.pPayload[6], macTxCmd.pPayload[7]);
      DBG_PRINT4(DBGSYS, "DATA[8..11]=0x%X, 0x%X, 0x%X, 0x%X", macTxCmd.pPayload[8], macTxCmd.pPayload[9], macTxCmd.pPayload[10], macTxCmd.pPayload[11]);
      txGo();
    }
    else
    {
      macTxActive = MAC_TX_ACTIVE_QUEUED;
      HAL_EXIT_CRITICAL_SECTION(s);
      DBG_PRINT3(DBGSYS, "MAC_TX_ACTIVE_QUEUED, macTxType=%i, macRxActive=0x%X, macRxOutgoingAckFlag=0x%X", macTxType, macRxActive, macRxOutgoingAckFlag);
    }
  }

  /* Power management state may change. Hence, vote. */
  macPwrVote();
}


/*=================================================================================================
 * @fn          txCsmaPrep
 *
 * @brief       Prepare/initialize for a CSMA transmit.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void txCsmaPrep(void)
{
  if (macTxType == MAC_TX_TYPE_SLOTTED_CSMA)
  {
    MAC_RADIO_TX_PREP_CSMA_SLOTTED();
  }
  else
  {
    MAC_RADIO_TX_PREP_CSMA_UNSLOTTED();
  }
}


#ifdef FEATURE_GREEN_POWER
/*=================================================================================================
 * @fn          txGreenPowerPrep
 *
 * @brief       Prepare/initialize for a Green Power transmit.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void txGreenPowerPrep(void)
{
  /* Re-use macTxCsmaBackoffDelay for Green Power number of transmissions */
  macTxCsmaBackoffDelay  = pMacDataTx->internal.gpNumOfTx;
  macTxGpInterframeDelay = pMacDataTx->internal.gpInterframeDelay;

  if (macTxGpInterframeDelay == 0)
  {
    macTxGpInterframeDelay = 1;
  }

  MAC_RADIO_TX_PREP_GREEN_POWER();
}
#endif /* #ifdef FEATURE_GREEN_POWER */


/*=================================================================================================
 * @fn          txGo
 *
 * @brief       Start a transmit going.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void txGo(void)
{
  /*
   *  If execution has reached this point, any transmitted ACK has long since completed.  It is
   *  possible though that there is still a pending callback.  If so, it is irrelevant and needs to
   *  be canceled at this point.
   */
  MAC_RADIO_CANCEL_ACK_TX_DONE_CALLBACK();
  macRxOutgoingAckFlag = 0;

  /* based on type of transmit, call the correct "go" functionality */
  if (macTxType == MAC_TX_TYPE_SLOTTED)
  {
    MAC_RADIO_TX_GO_SLOTTED();
  }

#ifdef FEATURE_GREEN_POWER
  else if (macTxType == MAC_TX_TYPE_GREEN_POWER)
  {
    MAC_RADIO_TX_GO_GREEN_POWER();
  }
#endif /* #ifdef FEATURE_GREEN_POWER */
  
  else
  {
    txCsmaGo();
  }

  /* Power management state may change. Hence, vote. */
  macPwrVote();
}


/*=================================================================================================
 * @fn          txCsmaGo
 *
 * @brief       Start a CSMA transmit going.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void txCsmaGo(void)
{
  if (macTxType == MAC_TX_TYPE_SLOTTED_CSMA)
  {
    if (MAC_RADIO_TX_GO_SLOTTED_CSMA() == FALSE)
    {
      DBG_PRINT0(DBGSYS, "Slotted TX start time too late - txComplete(MAC_NO_TIME)");
      txComplete(MAC_NO_TIME);
    }
  }
  else
  {
    MAC_RADIO_TX_GO_CSMA();
  }
}


/**************************************************************************************************
 * @fn          macTxFrameRetransmit
 *
 * @brief       Retransmit the last frame.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macTxFrameRetransmit(void)
{
  txRetransmitFlag = 1;
  
#if defined ( FEATURE_SYSTEM_STATS )
  /* Update Diagnostics counter */
  macLowLevelDiags(MAC_DIAGS_TX_UCAST_RETRY);
#endif

  macTxFrame(macTxType);
}


/**************************************************************************************************
 * @fn          macTxStartQueuedFrame
 *
 * @brief       See if there is a queued frame waiting to transmit.  If so, initiate
 *              the transmit now.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macTxStartQueuedFrame(void)
{
  halIntState_t  s;

  MAC_ASSERT(!macRxActive && !macRxOutgoingAckFlag); /* queued frames should not transmit in middle of a receive */
  DBG_PRINT0(DBGSYS, "macTxStartQueuedFrame()");

  /*
   *  Critical sections around the state change prevents any sort of race condition
   *  with macTxFrame().  This guarantees function txGo() will only be be called once.
   */
  HAL_ENTER_CRITICAL_SECTION(s);
  if (macTxActive == MAC_TX_ACTIVE_QUEUED)
  {
    macTxActive = MAC_TX_ACTIVE_GO;
    HAL_EXIT_CRITICAL_SECTION(s);
    
    /* Prepare the CSMA and TX timing again for queued TX */
    txCsmaPrep();
    txGo();
    DBG_PRINT0(DBGSYS, "Queued TX Sent");
  }
  else
  {
    HAL_EXIT_CRITICAL_SECTION(s);
    DBG_PRINT0(DBGSYS, "Queued TX not Sent");
  }
}


/**************************************************************************************************
 * @fn          macTxFrameIsr
 *
 * @brief       This callback is executed when transmit completes.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macTxFrameIsr(void)
{
#ifdef FEATURE_GREEN_POWER
  if (macTxType == MAC_TX_TYPE_GREEN_POWER)
  {
    DBG_PRINT1(DBGSYS, "Green Power MB_TX_DONE_INT %u", macTxCsmaBackoffDelay);
    
    /* The macTxCsmaBackoffDelay is reused as gpNumOfTx in Green Power transmissions */
    if (macTxCsmaBackoffDelay > 1)
    {
      macTxCsmaBackoffDelay--;
      MAC_RADIO_TX_GO_GREEN_POWER();
    }
    else
    {
      /* The transmission bursts have been completed. 
       * TX command and call macTxDoneCallback(). 
       */
      macTxTimestampCallback();
      macTxDoneCallback();
    }
  }
  else
#endif /* #ifdef FEATURE_GREEN_POWER */
  {
    macTxTimestampCallback();
    macTxDoneCallback();
  }
}


/**************************************************************************************************
 * @fn          macFgCmdDoneIsr
 *
 * @brief       This callback is executed when FG command completes. Note that all End of FG
 *              commands get here. The macTxActive flag is used to determine the proper 
 *              callback to be called.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macFgCmdDoneIsr(void)
{
  uint16 status;
  int8   rssiDbm;
  uint8  corr;
  
  /* CMD_IEEE_RX_ACK Foreground Command was done */
  if ( macTxActive == MAC_TX_ACTIVE_LISTEN_FOR_ACK && rxPromiscuousMode == PROMISCUOUS_MODE_OFF)
  {
    
    status = macRxAckCmd.rfOpCmd.status;
    
    /* Get RSSI and correlation values from the ACK frame */
    rssiDbm = macRxOutput.lastRssi + MAC_RADIO_RSSI_OFFSET;
    corr = 0; /* Not available and not used */
    
    /* Populate these from the ACK frame */
    pMacDataTx->internal.mpduLinkQuality = macRadioComputeLQI(rssiDbm, corr);
    pMacDataTx->internal.correlation = corr;
    pMacDataTx->internal.rssi = rssiDbm;
    
    /* Put the status back to idle so other FG command completing won't trigger this */
    macRxAckCmd.rfOpCmd.status = MACSTAT_IDLE;
    switch (status)
    {
    case MACSTAT_IEEE_DONE_ACK:
      DBG_PRINT1(DBGSYS, "RX ACK IEEE_DONE_ACK - macTxAckReceivedCallback(%i)", macRxAckCmd.seqNum); 
      macTxAckReceivedCallback(macRxAckCmd.seqNum, FALSE);
      break;
      
    case MACSTAT_IEEE_DONE_ACKPEND:
      DBG_PRINT1(DBGSYS, "RX ACK IEEE_DONE_ACKPEND - macTxAckReceivedCallback(%i)", macRxAckCmd.seqNum);
      macTxAckReceivedCallback(macRxAckCmd.seqNum, TRUE);
      break;
      
    case MACSTAT_IEEE_DONE_TIMEOUT:
      DBG_PRINT0(DBGSYS, "RX ACK IEEE_DONE_TIMEOUT - macTxAckNotReceivedCallback()");
      macTxAckNotReceivedCallback();
      break;
      
    default:
      /* Not handled End of RX ACK Operation */
      DBG_PRINT1(DBGSYS, "Unhandled End of RX ACK Operation, 0x%X - No Action!", status);
      break;
    }
  }
  else if(macTxActive == MAC_TX_ACTIVE_LISTEN_FOR_ACK && rxPromiscuousMode != PROMISCUOUS_MODE_OFF)
  {
    macTxAckNotReceivedCallback();
  }
  else if (macTxActive == MAC_TX_ACTIVE_GO)
  {
    if (macTxType == MAC_TX_TYPE_UNSLOTTED_CSMA || macTxType == MAC_TX_TYPE_SLOTTED_CSMA)
    {
      /* CMD_IEEE_CSMA Foreground Commands were done */
      status = macCsmaCaCmd.rfOpCmd.status;
    
      /* Put the status back to idle so other FG command completing won't trigger this */
      macCsmaCaCmd.rfOpCmd.status = MACSTAT_IDLE;
    
      switch (status)
      {
        case MACSTAT_IEEE_DONE_BUSY:
          DBG_PRINT0(DBGSYS, "CSMA CMD IEEE_DONE_BUSY - txComplete(MAC_CHANNEL_ACCESS_FAILURE)"); 
          txComplete(MAC_CHANNEL_ACCESS_FAILURE);
          break;
        
        case MACSTAT_IEEE_DONE_TIMEOUT:
          DBG_PRINT0(DBGSYS, "CSMA CMD IEEE_DONE_TIMEOUT - txComplete(MAC_NO_TIME)");
          txComplete(MAC_NO_TIME);
          break;
      
        case MACSTAT_IEEE_DONE_OK:
          DBG_PRINT0(DBGSYS, "CSMA CMD IEEE_DONE_OK - No Action!");
          break;
          
        case MACSTAT_IEEE_DONE_BGEND:
          /* For example: if Rx overflow is observed, the background task would 
           * be stopped and IEEE_DONE_BGEND would be generated.
           */
          DBG_PRINT0(DBGSYS, "CSMA CMD IEEE_DONE_BGEND - txComplete(MAC_TX_ABORTED)");
          txComplete(MAC_TX_ABORTED);
          
          /* Simply restart RX. CM0 will flush the RX FIFO. */ 
          macRxOff();
          macRxOnRequest();
          break;
          
        default:
          /* This includes IEEE_DONE_STOPPED, IEEE_DONE_ABORT, and IEEE_ERROR_PAR. 
           */
          DBG_PRINT1(DBGSYS, "Unhandled End of CSMA-CA Operation, 0x%X - No Action!", status);
          break;
      }
    }
    else
    {
      /* CMD_IEEE_TX (including Green Power) Foreground Commands were done.
       * This includes IEEE_DONE_OK, IEEE_DONE_STOPPED, IEEE_DONE_ABORT, 
       * and IEEE_ERROR_PAR. Put the status back to idle.
       */
      status = macTxCmd.rfOpCmd.status;

      macTxCmd.rfOpCmd.status = MACSTAT_IDLE;

      /* If the TX command is chained to itself, and if another ISR is on at least 
       * the same priority or a protected area, the radio CPU may set the status 
       * to PENDING before CM3 has time to read the status. 
       */
      switch (status)
      {
        case MACSTAT_IEEE_DONE_OK:
          DBG_PRINT0(DBGSYS, "TX CMD IEEE_DONE_OK - No Action!"); 
          break;
        
        default:
          /* This includes IEEE_DONE_STOPPED, IEEE_DONE_ABORT and IEEE_ERROR_PAR. 
           * PENDING status may get here because of the chained command. 
           * No trace necessary.
           */
          DBG_PRINT1(DBGSYS, "Unhandled End of TX Operation, 0x%X - No Action!", status);
          break;
      }
    }
  }
  else
  {
    DBG_PRINT0(DBGSYS, "Unhandled End of FG Operation - No Action!");
  }
}


/**************************************************************************************************
 * @fn          macTxAckIsr
 *
 * @brief       This callback is executed when ACK transmit completes.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macTxAckIsr(void)
{
  macRxAckTxDoneCallback();
}


/**************************************************************************************************
 * @fn          macTxDoneCallback
 *
 * @brief       This callback is executed when transmit completes.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macTxDoneCallback(void)
{
  halIntState_t  s;

  /*
   *  There is a small chance this function could be called twice for a single transmit.
   *  To prevent logic from executing twice, the state variable macTxActive is used as
   *  a gating mechanism to guarantee single time execution.
   */
  HAL_ENTER_CRITICAL_SECTION(s);
  if (macTxActive == MAC_TX_ACTIVE_GO)
  {
    /* see if ACK was requested */
    if (!txAckReq)
    {
      macTxActive = MAC_TX_ACTIVE_DONE;
      HAL_EXIT_CRITICAL_SECTION(s);

      /* ACK was not requested, transmit is complete */
      txComplete(MAC_SUCCESS);
    }
    else
    {
      /*
       *  ACK was requested - must wait to receive it.  The macRxAckCmd end time 
       *  is set to expire after the timeout duration for waiting for an ACK.
       *  If an ACK is received, the function macTxAckReceived() is called.
       *  If an ACK is not received within the timeout period,
       *  the function macTxAckNotReceivedCallback() is called.
       */
      macTxActive = MAC_TX_ACTIVE_LISTEN_FOR_ACK;
      HAL_EXIT_CRITICAL_SECTION(s);
  
      /* start ACK receive task and timeout */
      MB_SendCommand( (uint32)&macRxAckCmd );
    }
  }
  else
  {
    HAL_EXIT_CRITICAL_SECTION(s);
  }
}


/**************************************************************************************************
 * @fn          macTxAckReceivedCallback
 *
 * @brief       This function is called by the receive logic when an ACK is received and
 *              transmit logic is listening for an ACK.
 *
 * @param       seqn        - sequence number of received ACK
 * @param       pendingFlag - set if pending flag of ACK is set, cleared otherwise
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macTxAckReceivedCallback(uint8 seqn, uint8 pendingFlag)
{
  halIntState_t  s;

  /* only process if listening for an ACK; critical section prevents race condition problems */
  HAL_ENTER_CRITICAL_SECTION(s);
  if (macTxActive == MAC_TX_ACTIVE_LISTEN_FOR_ACK)
  {
    macTxActive = MAC_TX_ACTIVE_POST_ACK;
    HAL_EXIT_CRITICAL_SECTION(s);

    /* see if the sequence number of received ACK matches sequence number of packet just sent */
    if (seqn == txSeqn)
    {
      /*
       *  Sequence numbers match so transmit is successful.  Return appropriate
       *  status based on the pending flag of the received ACK.
       */
      if (pendingFlag)
      {
        txComplete(MAC_ACK_PENDING);
      }
      else
      {
        txComplete(MAC_SUCCESS);
      }
    }
    else
    {
      /* sequence number did not match; per spec, transmit failed at this point */
      txComplete(MAC_NO_ACK);
    }
  }
  else
  {
    HAL_EXIT_CRITICAL_SECTION(s);
  }
}


/**************************************************************************************************
 * @fn          macTxAckNotReceivedCallback
 *
 * @brief       This function is called by the receive logic when transmit is listening
 *              for an ACK but something else is received.  It is also called if the
 *              listen-for-ACK timeout is reached.
 *
 * @brief
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macTxAckNotReceivedCallback(void)
{
  halIntState_t  s;

  /* only process if listening for an ACK; critical section prevents race condition problems */
  HAL_ENTER_CRITICAL_SECTION(s);
  if (macTxActive == MAC_TX_ACTIVE_LISTEN_FOR_ACK)
  {
    macTxActive = MAC_TX_ACTIVE_POST_ACK;
    HAL_EXIT_CRITICAL_SECTION(s);

    /* a non-ACK was received when expecting an ACK, per spec transmit is over at this point */
    txComplete(MAC_NO_ACK);
  }
  else
  {
    HAL_EXIT_CRITICAL_SECTION(s);
  }
}


/*=================================================================================================
 * @fn          txComplete
 *
 * @brief       Transmit has completed.  Perform needed maintenance and return status of
 *              the transmit via callback function.
 *
 * @param       status - status of the transmit that just went out
 *
 * @return      none
 *=================================================================================================
 */
static void txComplete(uint8 status)
{
  /* reset the retransmit flag */
  txRetransmitFlag = 0;

  /* update tx state; turn off receiver if nothing is keeping it on */
  macTxActive = MAC_TX_ACTIVE_NO_ACTIVITY;
    
  /* turn off receive if allowed */
  macRxOffRequest();

  /* update transmit power in case there was a change */
  macRadioUpdateTxPower();

  /*
   *  Channel cannot change during transmit so update it here.  (Channel *can* change during
   *  a receive.  The update function resets receive logic and any partially received
   *  frame is purged.)
   */
  macRadioUpdateChannel();
  
  /* return status of transmit via callback function */
  macTxCompleteCallback(status);

  /* Power management state may change. Hence, vote. */
  macPwrVote();
}


/**************************************************************************************************
 * @fn          macTxTimestampCallback
 *
 * @brief       This callback function records the timestamp into the receive data structure.
 *              It should be called as soon as possible after there is a valid timestamp.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macTxTimestampCallback(void)
{
  MAC_ASSERT(pMacDataTx != NULL); /* transmit structure must be there */

  DBG_PRINTL1(DBGSYS, "TX Time Stamp=0x%X", macTxCmd.timeStamp);

  /* The time, in aUnitBackoffPeriod units, at which the frame was transmitted. */
  pMacDataTx->internal.timestamp  = MAC_RADIO_BACKOFF_CAPTURE( macTxCmd.timeStamp );
  
  /* The time, in RAT ticks, at which the frame was transmitted. */
  pMacDataTx->internal.timestamp2 = MAC_RADIO_TIMER_CAPTURE( macTxCmd.timeStamp );
}


/**************************************************************************************************
 *                                  Compile Time Integrity Checks
 **************************************************************************************************
 */
#if (MAC_TX_ACTIVE_NO_ACTIVITY != 0x00)
#error "ERROR! Zero is reserved value of macTxActive. Allows boolean operations, e.g !macTxActive."
#endif

/**************************************************************************************************
*/
