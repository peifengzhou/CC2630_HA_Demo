/*******************************************************************************
  Filename:       mac.c
  Revised:        $Date: 2015-02-17 14:23:30 -0800 (Tue, 17 Feb 2015) $
  Revision:       $Revision: 42685 $

  Description:    This file contains the data structures and APIs for CC26xx
                  RF Core Firmware Specification for IEEE 802.15.4.

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
  PROVIDED ``AS IS'' WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "hw_memmap.h"
#include "hw_aon_rtc.h"
#include "hal_types.h"
#include "hal_defs.h"
#include "mb.h"
#include "rfHal.h"

#include "saddr.h"

#include "mac.h"
#include "mac_isr.h"
#include "mac_spec.h"
#include "mac_mem.h"
#include "mac_pib.h"
#include "mac_rx.h"
#include "mac_tx.h"

#include "mac_radio_defs.h"
#include "mac_backoff_timer.h"
#include "mac_autopend.h"

#include "mac_high_level.h"
#include "mac_low_level.h"

#include "osal.h"

#include "hal_assert.h"

/* High Level MAC */
#include "mac_timer.h"
#include "mac_beacon.h"
#include "mac_main.h"
#include "mac_data.h"

#include "Macs.h"
   
/* debug */
#include "mac_assert.h"

/* Common ROM jump tables */
#include "R2F_CommonFlashJT.h"

/* CM0 Patch */
#include "rfcore_15_4_patch.h"

#ifdef DEBUG_SW_TRACE
#define DBG_ENABLE
#include "dbgid_sys_mst.h"
#endif /* DEBUG_SW_TRACE */


/*******************************************************************************
 * MACROS
 */

/*
 *  Number of bits used for aligning a slotted transmit to the backoff count (plus
 *  derived values).  There are restrictions on this value.  Compile time integrity
 *  checks will catch an illegal setting of this value.  A full explanation accompanies
 *  this compile time check (see bottom of this file).
 */
#define SLOTTED_TX_MAX_BACKOFF_COUNTDOWN_NUM_BITS     4
#define SLOTTED_TX_MAX_BACKOFF_COUNTDOWN              (1 << SLOTTED_TX_MAX_BACKOFF_COUNTDOWN_NUM_BITS)
#define SLOTTED_TX_BACKOFF_COUNT_ALIGN_BIT_MASK       (SLOTTED_TX_MAX_BACKOFF_COUNTDOWN - 1)


/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */


/*******************************************************************************
 * GLOBAL VARIABLES
 */

/* Source Matching lists */
extAddrList_t    macSrcExtAddrList;
shortAddrList_t  macSrcShortAddrList;

/* CSMA/CA Connand */
csmaCaCmd_t macCsmaCaCmd;

/* Transmit Command */
txCmd_t macTxCmd;

/* Receive ACK Command */
rxAckCmd_t macRxAckCmd;

/* edCmd and rxCmd share the same memory */
rxEdScan_t macRxEdScan;

/* Reveive output buffer */
output_t macRxOutput;

/* MAC RAT channel A */
uint8 macRatChanA;

/* MAC RAT channel B */
uint8 macRatChanB;

/* MAC RF front end configuration. Use a pointer to void since
 * the format changes from package to package.
 */
void *macRadioConfig;

/* MAC RX command values */
const rxCmd_t macSetupReceiveCmdValues = {
  {  
    CMD_IEEE_RX,
    MACSTAT_IDLE,
    NULL,
    0x0000,
    TRIGTYPE_NOW,
    CONDTYPE_NEVER_RUN_NEXT_CMD
  },
  MAC_RADIO_CHANNEL_INVALID,
  ( SET_RX_AUTO_FLUSH_CRC_BIT | 
    SET_RX_AUTO_FLUSH_FRAME_FILTERING_BIT |
    SET_RX_INCLUDE_PHY_HDR_BIT | 
    SET_RX_APPEND_RSSI_BIT | 
    SET_RX_APPEND_CORR_CRC_BIT |
    SET_RX_APPEND_SRC_INDEX_BIT |
    SET_RX_APPEND_TIMESTAMP_BIT ),
  NULL,
  &macRxOutput,
  ( SET_FRAME_FILTER_ON_BIT |
    SET_AUTO_ACK_ENABLE_BIT |
    SET_AUTO_PEND_ENABLE_BIT | 
    SET_DATA_REQ_AUTO_PEND_BIT |
    SET_FRAME_FILTER_MAX_VERSION_TWO |
    SET_STRICT_ACK_LEN_BIT ),
  ( SET_ACCEPT_BEACON_BIT |
    SET_ACCEPT_DATA_BIT | 
    SET_ACCEPT_MAC_CMD_BIT |
    SET_ACCEPT_FT5_BIT ),
  ( SET_ENABLE_ENERGY_SCAN_SOURCE_BIT |
    SET_ENABLE_CORRELATOR_SOURCE_BIT |
    SET_ENABLE_SYNC_FOUND_SOURCE_BIT |
    SET_CORRELATOR_THRESHOLD_THREE ),
#if defined( USE_FPGA )
  0xC4,
#else /* USE_FPGA */
  0xB0, /* CCA RSSI threshold = -80 dBm */
#endif /* USE_FPGA */
  0,
  MAC_SRCMATCH_EXT_MAX_NUM_ENTRIES,
  MAC_SRCMATCH_SHORT_MAX_NUM_ENTRIES,
  &macSrcExtAddrList,
  &macSrcShortAddrList,
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  0x0000,
  0x0000,
  {0x00, 0x00, 0x00},
  TRIGTYPE_NEVER,
  0x0000  
};

/* MAC ED command values */
const edScanCmd_t macSetupEdCmdValues = {
  {  
    CMD_IEEE_ED_SCAN,
    MACSTAT_IDLE,
    NULL,
    0x0000,
    TRIGTYPE_NOW,
    CONDTYPE_NEVER_RUN_NEXT_CMD
  },
  MAC_RADIO_CHANNEL_INVALID,
  SET_ENABLE_ENERGY_SCAN_SOURCE_BIT,
  0x00,
  0x00,
  TRIGTYPE_NEVER,
  0x0000  
};

/* MAC CSMA/CA command values. The default is for non-slotted */
const csmaCaCmd_t macSetupCsmaCaCmdValues = {
  {  
    CMD_IEEE_CSMA,
    MACSTAT_IDLE,
    (rfOpCmd_t *)&macTxCmd,
#if defined( CC26XX_PG1 )
    ( 320*MAC_RAT_MHZ ),
    ( TRIGTYPE_REL_CMD_SUBMIT |
      SET_RFOP_PAST_TRIG_BIT ),
#elif defined( CC26XX )
    0x0000,
    ( TRIGTYPE_NOW | 
      SET_RFOP_PAST_TRIG_BIT ),
#else /* unknown device */
  #error "ERROR: Unknown device!"
#endif /* CC26XX_PG1 */
    CONDTYPE_RUN_TRUE_STOP_FALSE
  },
  0x0000,
  0x00,
  0x00,
#if defined( CC26XX_PG1 )
  0x01,
#elif defined( CC26XX )
  /* Set RX off mode to Mode 0:
   * Mode 0: RX stays on during CSMA backoffs.
   * Mode 1: RX off if no frame is being received.
   * Mode 2: RX off if no frame is being received or after finishing auto ACK. 
   * Mode 3: RX off during CSMA backoffs.
   */
  ( 0x01 | 0<<6 ),
#else /* unknown device */
  #error "ERROR: Unknown device!"
#endif /* CC26XX_PG1 */
  0x00,
  0x00,
  0x00,
  0x00,
  TRIGTYPE_NEVER,
  0x0000,
  0x0000
};

/* MAC TX command values. This only covers the Radio Operation Command 
 * Common Structure. As the length and pointer cannot be overridden.
 */
const rfOpCmd_t macTxCmdValues = {
  CMD_IEEE_TX,
  MACSTAT_IDLE,
  NULL,
  0x0000,
  TRIGTYPE_NOW,
  CONDTYPE_NEVER_RUN_NEXT_CMD
};

/* MAC RX ACK command values */
const rxAckCmd_t macRxAckCmdValues = {
  {  
    CMD_IEEE_RX_ACK,
    MACSTAT_IDLE,
    NULL,
    0x0000,
    TRIGTYPE_NOW,
    CONDTYPE_NEVER_RUN_NEXT_CMD
  },
  0x00,
  TRIGTYPE_REL_CMD_START,
  0x0000  
};

/*
** RF Hardware Abstraction Layer Application Programming Interface
*/

/*******************************************************************************
 * LOCAL FUNCTIONS
 */

/*******************************************************************************
 * @fn          macGenericRadioCmd
 *
 * @brief       This call is used to program a generic CM0 radio command.
 *
 * input parameters
 *
 * @param       cmd - radio command number.
 *              rfOpCmd - pointer to RF Operation Command
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
static void macGenericRadioCmd( uint16 cmd, rfOpCmd_t *rfOpCmd )
{
  rfOpCmd->cmdNum    = cmd;
  rfOpCmd->status    = RFSTAT_IDLE;
  rfOpCmd->pNextRfOp = NULL;
  rfOpCmd->startTime = 0;
  CLR_RFOP_ALT_TRIG_CMD( rfOpCmd->startTrig );
  CLR_RFOP_PAST_TRIG( rfOpCmd->startTrig );
  SET_RFOP_TRIG_TYPE( rfOpCmd->startTrig, TRIGTYPE_NOW );
  SET_RFOP_COND_RULE( rfOpCmd->condition, CONDTYPE_NEVER_RUN_NEXT_CMD );
}


/*******************************************************************************
 * @fn          macSynchRadioCommand
 *
 * @brief       This function sends a command to the radio firmware via the
 *              Mailbox interface, and waits until the command is executed
 *              to completion.
 *              This function is thread-safe as far as threads do not use
 *              the same memory space for commands (i.e., unless the thread-
 *              safety is broken outside this function).
 *              This function does not rely on COMMAND DONE interrupt or
 *              LAST COMMAND DONE interrupt as they may be fired asynchronously,
 *              e.g., due to failure of a previous background command.
 *              This function instead relies on the status field of
 *              the command structure to be updated by CM0.
 *              Do not use a non-radio command, that is, either a direct command
 *              or an immediate command with this function.
 *
 *              Note that TIMAC must not use MB_SendCommandSynch() function,
 *              since the function intends to clear unmasked interrupts
 *              while TIMAC has to handle unmasked interrupts asynchronously.
 *
 * input parameters
 *
 * @param       pCmd - pointer to the radio command.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      CMDSTA register value or CMDSTA_MAILBOX_BUSY if the command
 *              execution fails due to detected asynchronous unrecoverable
 *              error, which should require CM0 reset.
 */
static uint8 macSynchRadioCommand(rfOpCmd_t *pCmd)
{
  uint8 cmdstaStatus;

  /* This function handles radio commands only */
  MAC_ASSERT((pCmd->cmdNum & 0x800) != 0);

  /* Clear the status field before issuing the command */
  pCmd->status = RFSTAT_IDLE;

  /* Submit the command */
  cmdstaStatus = MB_SendCommand((uint32) pCmd);
  
  /* If all TIMAC code uses MB_SendCommand(), there shouldn't be a case where
   * the command fails with mailbox busy status.
   * If the following assertion fails, it means that either there is a bug
   * in TIMAC code that uses command register directly without proper protection
   * or the hardware doorbell mechanism has a bug.
   */
  MAC_ASSERT(cmdstaStatus != CMDSTA_MAILBOX_BUSY);

  if (cmdstaStatus != CMDSTA_DONE)
  {
    return cmdstaStatus;
  }

  for (;;)
  {
    /* TODO: Add check for asynchronous unrecoverable error detected by a radio
     *       interrupt service routine, in which case the CM0 has to be reset
     *       and hence this function should stop blocking.
     *       It can be done by the radio ISR setting up a global variable
     *       and checking the value here.
     */
    uint16 cmdstatus = *(volatile uint16 *) &(pCmd->status);
    if ((cmdstatus & 0x0C00) != 0)
    {
      /* Command is complete. */
      break;
    }
  }

  return cmdstaStatus;
}


/*******************************************************************************
 * @fn          macSetCcaRssiThreshold
 *
 * @brief       This routine is used to set ccaRssiThr value. 
 *
 * input parameters
 *
 * @param       ccaRssiThr - RSSI threshold for CCA.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void macSetCcaRssiThreshold( uint8 ccaRssiThreshold )
{
  macRxEdScan.rxCmd.ccaRssiThreshold = ccaRssiThreshold;
}


/*******************************************************************************
 * @fn          macSetupReceiveCmd
 *
 * @brief       This routine is used to setup MAC receive command. 
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
void macSetupReceiveCmd( void )
{
  /* copy the command values to command buffer */
  osal_memcpy( &macRxEdScan, &macSetupReceiveCmdValues, sizeof( rxCmd_t ) );

  /* Set receive queue */
  macRxEdScan.rxCmd.pRXQ = macRxDataEntryQueue;
}


/*******************************************************************************
 * @fn          macSendReceiveCmd
 *
 * @brief       This routine is used to send MAC receive command. 
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
void macSendReceiveCmd( void )
{
  /* Configure channel and PanCoordinator */
  MAC_RADIO_SET_CHANNEL(pMacPib->logicalChannel);
  MAC_RADIO_SET_PAN_COORDINATOR(macPanCoordinator);
  
  /* Set local addresses and PANID are send by macSendReceiveCmd() */
  MAC_RADIO_SET_PAN_ID(pMacPib->panId);
  MAC_RADIO_SET_SHORT_ADDR(pMacPib->shortAddress);
  MAC_RADIO_SET_IEEE_ADDR(pMacPib->extendedAddress.addr.extAddr);
 
  /* Send the cmd */
  MB_SendCommand( (uint32)&macRxEdScan.rxCmd );
}


/*******************************************************************************
 * @fn          macSendEdCmd
 *
 * @brief       This routine is used to send MAC energy detection command. 
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
void macSendEdCmd( void )
{
  /* Send the cmd */
  MB_SendCommand( (uint32)&macRxEdScan.edCmd );
}


/*******************************************************************************
 * @fn          macSetupCsmaCaTxCmd
 *
 * @brief       This routine is used to setup MAC transmit command chained to 
 *              the CSMA/CA command. 
 *
 * input parameters
 *
 * @param       slotted - specify slotted or non-slotted CSMA
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void macSetupCsmaCaTxCmd( bool slotted )
{
  /* Copy the command values to command buffer */
  osal_memcpy( &macCsmaCaCmd, &macSetupCsmaCaCmdValues, sizeof( csmaCaCmd_t ) );

#if defined( FEATURE_BEACON_MODE )
  /* Set start time and trigger for slotted */
  if (slotted == TRUE)
  {
    uint8  backoffCountdown;
    uint8  lowByteOfBackoffCount;

    /* Compute the number of backoffs until time to strobe transmit.  The strobe should
     * occur right before the SFD pin is expected to go high.  So, the forumla for the
     * countdown value is to determine when the lower bits would rollover and become zero.
     */
    lowByteOfBackoffCount = (uint8)(MAC_RADIO_BACKOFF_COUNT() & 0x0FF);
    backoffCountdown = SLOTTED_TX_MAX_BACKOFF_COUNTDOWN - (lowByteOfBackoffCount & SLOTTED_TX_BACKOFF_COUNT_ALIGN_BIT_MASK);
   
    /* Set start time and trigger, the start time needs to be set before the cmd is sent. */
    macCsmaCaCmd.rfOpCmd.startTime = backoffCountdown * MAC_BACKOFF_TO_RAT_RATIO + MAC_RAT_COUNT;
    SET_RFOP_TRIG_TYPE(macCsmaCaCmd.rfOpCmd.startTrig, TRIGTYPE_AT_ABS_TIME);
    
    SET_CSMA_SLOTTED(macCsmaCaCmd.csmaConfig);
    SET_CSMA_INIT_CW(macCsmaCaCmd.csmaConfig, 2);
    
    /* Set end Trigger */
    SET_RFOP_TRIG_TYPE(macCsmaCaCmd.endTrigger, TRIGTYPE_AT_ABS_TIME);
    CLR_RFOP_ALT_TRIG_CMD(macCsmaCaCmd.endTrigger);
    SET_RFOP_TRIG_NUM(macCsmaCaCmd.endTrigger, 0);
    
    /* Set end Time. TBD: BLE end time needs to be added. 
     * Only the CAP end time is implemented for now.
     * unit in RAT ticks. TBD: Need to fine tune the one backoff early timeout. 
     */
    macCsmaCaCmd.endTime = (macBeacon.capEnd - 1) * MAC_BACKOFF_TO_RAT_RATIO + macPrevPeriodRatCount;
    DBG_PRINTL2(DBGSYS, "macBeacon.capEnd=%i, MAC_RADIO_BACKOFF_COUNT()=%i", macBeacon.capEnd, MAC_RADIO_BACKOFF_COUNT());
    DBG_PRINTL2(DBGSYS, "macCsmaCaCmd.endTime=0x%X, Delta=%i", macCsmaCaCmd.endTime, (macCsmaCaCmd.endTime - macCsmaCaCmd.rfOpCmd.startTime) / MAC_BACKOFF_TO_RAT_RATIO);
  }
#endif  
  
  /* Program CSMA/CA parameters */
  macCsmaCaCmd.macMaxBE = pMacPib->maxBe;
  macCsmaCaCmd.macMaxCSMABackoffs = pMacPib->maxCsmaBackoffs;
  macCsmaCaCmd.BE = macTxBe;

  /* Copy the command values to command buffer */
  osal_memcpy( &macTxCmd, &macTxCmdValues, sizeof( rfOpCmd_t ) );
  
#if defined( CC26XX )
  /* The CSMA/CA Cmd assumes a coordinator, for End Device in PG2,
   * RX off mode should be Mode 2 to save power. For Beacom mode, 
   * keep RX on during CSMA backoffs.
   */
  if (!macPanCoordinator && (slotted == FALSE))
  {
    /* For End Device on PG2, set RX off mode to Mode 2:
     * Mode 0: RX stays on during CSMA backoffs.
     * Mode 1: RX off if no frame is being received.
     * Mode 2: RX off if no frame is being received or after finishing auto ACK. 
     * Mode 3: RX off during CSMA backoffs.
     */
    SET_CSMA_RX_OFF_MODE(macCsmaCaCmd.csmaConfig, 2);
  }
#endif /* CC26XX */

}


/*******************************************************************************
 * @fn          macSetupSlottedTxCmd
 *
 * @brief       This routine is used to setup slotted MAC transmit command. 
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
void macSetupSlottedTxCmd( void )
{
#if defined( FEATURE_BEACON_MODE )
  uint8  backoffCountdown;
  uint8  lowByteOfBackoffCount;

  /* Copy the command values to command buffer */
  osal_memcpy( &macTxCmd, &macTxCmdValues, sizeof( rfOpCmd_t ) );

  /* Set start time and trigger, the start time needs to be set before the cmd is sent.
   * Compute the number of backoffs until time to strobe transmit.  The strobe should
   * occur right before the SFD pin is expected to go high.  So, the forumla for the
   * countdown value is to determine when the lower bits would rollover and become zero.
   */
  lowByteOfBackoffCount = (uint8)(MAC_RADIO_BACKOFF_COUNT() & 0x0FF);
  backoffCountdown = SLOTTED_TX_MAX_BACKOFF_COUNTDOWN - (lowByteOfBackoffCount & SLOTTED_TX_BACKOFF_COUNT_ALIGN_BIT_MASK);
  macTxCmd.rfOpCmd.startTime = backoffCountdown * MAC_BACKOFF_TO_RAT_RATIO + MAC_RAT_COUNT;

  SET_RFOP_TRIG_TYPE(macTxCmd.rfOpCmd.startTrig, TRIGTYPE_AT_ABS_TIME);
#endif /* FEATURE_BEACON_MODE */
}


/*******************************************************************************
 * @fn          macSetupGreenPowerTxCmd
 *
 * @brief       This routine is used to setup MAC transmit command for 
 *              Green Power. 
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
void macSetupGreenPowerTxCmd( void )
{
  /* Copy the command values to command buffer */
  osal_memcpy( &macTxCmd, &macTxCmdValues, sizeof( rfOpCmd_t ) );

  /* Set start time and trigger */  
  macTxCmd.rfOpCmd.startTime = MAC_RADIO_TIMER_TICKS_PER_BACKOFF() * macTxGpInterframeDelay;
  SET_RFOP_TRIG_TYPE(macTxCmd.rfOpCmd.startTrig, TRIGTYPE_REL_CMD_SUBMIT);
}


/*******************************************************************************
 * @fn          macSetupRxAckCmd
 *
 * @brief       This routine is used to setup MAC receive ACK command. 
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
void macSetupRxAckCmd( void )
{
  /* Copy the command values to command buffer */
  osal_memcpy( &macRxAckCmd, &macRxAckCmdValues, sizeof( rxAckCmd_t ) );

  /* Set end Time, RAT Ticks = 4MHz or 0.25us */
  macRxAckCmd.endTime = MAC_RAT_MHZ * MAC_SPEC_USECS_PER_SYMBOL * pMacPib->ackWaitDuration;
}


/*******************************************************************************
 * @fn          macSetupRxAckSeqNo
 *
 * @brief       This routine is used to setup MAC receive ACK Sequence Number. 
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       seqNo - ACK sequence number.
 *
 * @return      None.
 */
void macSetupRxAckSeqNo( uint8 seqNo )
{
  macRxAckCmd.seqNum = seqNo;
}


/*******************************************************************************
 * @fn          macSetupEdCmd
 *
 * @brief       This routine is used to setup MAC Energy Detect command. 
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
void macSetupEdCmd( void )
{
  /* copy the command values to command buffer */
  osal_memcpy( &macRxEdScan, &macSetupEdCmdValues, sizeof( edScanCmd_t ) );

  /* Set scan channel */
  macRxEdScan.edCmd.channel = pMacPib->logicalChannel;
}


/*******************************************************************************
 * @fn          macSetupRadio
 *
 * @brief       This call is used to setup the CC26xx radio for 15.4, and to 
 *              issue any hardware or firmware register overrides required
 *              for initialization.
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
void macSetupRadio( void )
{
  rfOpCmd_RadioSetup_t rfCmd;

#if defined( USE_FPGA )
  regOverride_t hwRegs[] = { 0x00015164, 0x00041110, 0x00000083, 0x02C402A3, 
                             0x00800403, 0xFFFFFFFF } ;
#else
#if defined( CC26XX_PG1 )  
  regOverride_t hwRegs[] = { 
    0x00001107,
    0x002082C3,
    0x0000401C,
    0x40014019,
    0x84000000,
    0x4001402D,
    0x00608202,
    0x40014035,
    0x177F4804,
    0x00604064,
    0x25E00423,
    0xC00C0061,
    0x007F4128,
    0x01343010,
    0x0001F800,
    0x04000243,
    0x07A00B43,
    0x00000323,
    0x00000343,
    0x00606098, // 0dDm output power 
    0xFFFFFFFF
  };
#elif defined( CC26XX )
  /* Cast the void pointer to macUserCfg_t pointer */
  macUserCfg_t *config = (macUserCfg_t *)macRadioConfig;
#else /* unknown device */
  #error "Error: Unknown device!"
#endif /* CC26XX_PG1 */
#endif /* USE_FPGA */
  
  /* setup a radio command CMD_RADIO_SETUP */
  macGenericRadioCmd( CMD_RADIO_SETUP, &rfCmd.rfOpCmd ); 

#if defined( CC26XX_PG1 )
  rfCmd.mode              = (uint16)RADIO_SETUP_MODE_15_4;
  rfCmd.pRegOverride      = &hwRegs[0];
#elif defined( CC26XX )
  rfCmd.mode              = (uint8)RADIO_SETUP_MODE_15_4;
  rfCmd.config            = config->rfFeModeBias;
  
  /* TX power will be set to PIB value */
  rfCmd.txPower           = macGetRadioTxPowerReg( &pMacPib->phyTransmitPower );
  rfCmd.pRegOverride      = (regOverride_t *)config->rfOverrideTable;
  
  if(config->rfFeIOD[0] != NULL)
  {
    for(uint8 i =0; i < config->rfFeIODNum; i++ )
    {
      HWREG(IOC_BASE + (config->rfFeIOD[i])) = config->rfFeIODVal[i];
    }
  }
  
#else /* unknown device */
  #error "ERROR: Unknown device!"
#endif /* CC26XX_PG1 */
  
  /* issue radio command synchronously */
  macSynchRadioCommand( &rfCmd.rfOpCmd );

  MAC_ASSERT( rfCmd.rfOpCmd.status == RFSTAT_DONE_OK );
  
  /* Use TRNG to generate 16 random bits.  */
  macCsmaCaCmd.randomState = macMcuRandomWord(); 
 
  return;
}


/*******************************************************************************
 * @fn          macStopCmd
 *
 * @brief       This call is used to gracefully stop all foreground and 
 *              background level commands that are running. This will cause 
 *              the BG and FG commands to stop as soon as possible after 
 *              finishing ongoing operations.
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
void macStopCmd( void )
{
  /* setup a direct command CMD_STOP */
  MB_SendCommand( BUILD_DIRECT_CMD( CMD_STOP ) );

  return;
}


/*******************************************************************************
 * @fn          macPowerDownCmd
 *
 * @brief       This call is used to gracefully park CM0 before cutting off
 *              CM0 power domain.
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
void macPowerDownCmd( void )
{
  rfOpCmd_freqSynthDown_t rfCmd;

  /* Configure generic command header */
  macGenericRadioCmd( CMD_FS_POWERDOWN, &rfCmd.rfOpCmd ); 

  /* issue radio command synchronously */
  macSynchRadioCommand( &rfCmd.rfOpCmd );

  MAC_ASSERT( rfCmd.rfOpCmd.status == RFSTAT_DONE_OK );
}


/*******************************************************************************
 * @fn          macStopRAT
 *
 * @brief       This function stops the RAT timer.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      RAT value synchronized to RTC.
 */
uint32 macStopRAT( void )
{
  rfOpCmd_SynchStartStopRat_t rfCmd;

  // call this instruction before CMD_SYNC_STOP_RAT or CMD_SYNC_START_RAT
  HWREG(AON_RTC_BASE + AON_RTC_O_CTL) |= AON_RTC_CTL_RTC_UPD_EN;

  // synch the RAT from the RTC and save the RAT value
  macGenericRadioCmd( CMD_SYNC_STOP_RAT, &rfCmd.rfOpCmd ); 
  
  {
    uint8 result = macSynchRadioCommand( &rfCmd.rfOpCmd );

    MAC_ASSERT(rfCmd.rfOpCmd.status == RFSTAT_DONE_OK );
  }

  // call this instruction after CMD_SYNC_STOP_RAT or CMD_SYNC_START_RAT
  HWREG(AON_RTC_BASE + AON_RTC_O_CTL) &= ~AON_RTC_CTL_RTC_UPD_EN;

  // Save RAT value
  return( rfCmd.ratVal );
}


/*******************************************************************************
 * @fn          macSyncStartRAT
 *
 * @brief       This function starts the RAT timer.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      RAT value synchronized to RTC.
 */
void macSyncStartRAT( uint32 ratval )
{
  rfOpCmd_SynchStartStopRat_t rfCmd;

  // call this instruction before CMD_SYNC_STOP_RAT or CMD_SYNC_START_RAT
  HWREG(AON_RTC_BASE + AON_RTC_O_CTL) |= AON_RTC_CTL_RTC_UPD_EN;

  // synch the RAT from the RTC using the saved RAT value
  macGenericRadioCmd( CMD_SYNC_START_RAT, &rfCmd.rfOpCmd ); 
  rfCmd.ratVal = ratval;
  
  {
    uint8 result = macSynchRadioCommand( &rfCmd.rfOpCmd );

    MAC_ASSERT( rfCmd.rfOpCmd.status == RFSTAT_DONE_OK );
  }

  // call this instruction after CMD_SYNC_STOP_RAT or CMD_SYNC_START_RAT
  HWREG(AON_RTC_BASE + AON_RTC_O_CTL) &= ~AON_RTC_CTL_RTC_UPD_EN;

  return;
}

/*******************************************************************************
 * @fn          macRxQueueFlushCmd
 *
 * @brief       On reception, the radio CPU shall flush the RX queue. Note that
 *              for radio RX overflow, the RX queue is flushed by CM0.
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
void macRxQueueFlushCmd( void )
{
  rfOpImmedCmd_AddRemoveFlushQueue_t rfCmd;

  /* setup an immediate command CMD_FLUSH_QUEUE */
  rfCmd.cmdNum   = CMD_FLUSH_QUEUE;
  rfCmd.pQueue   = macRxDataEntryQueue;
  rfCmd.pEntry   = (uint8 *)macRxDataEntryQueue->pCurEntry;

  /* issue command */
  MB_SendCommand( (uint32)&rfCmd );
  
  return;
}


/*******************************************************************************
 * @fn          macWriteRfRegCmd
 *
 * @brief       On reception, the radio CPU shall write the value to the address.
 *
 * input parameters
 *
 * @param       addr - RF hardware address.
 *              value - value to be written
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void macWriteRfRegCmd( uint16 addr, uint32 value )
{
  rfOpImmedCmd_RW_RfReg_t rfCmd;

  /* setup an immediate command CMD_WRITE_RFREG */
  rfCmd.cmdNum   = CMD_WRITE_RFREG;
  rfCmd.address  = addr;
  rfCmd.value    = value;

  /* issue command */
  MB_SendCommand( (uint32)&rfCmd );
  
  return;
}


/*******************************************************************************
 * @fn          macFreqSynthCmd
 *
 * @brief       This radio command is used to control Frequency Synthesizer. 
 *
 * input parameters
 *
 * @param       channel - logic channel number.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void macFreqSynthCmd( uint8 channel )
{
  /* For test only, if Tx is needed without any underlying Rx operation 
   * (i.e., no CSMA-CA), then CMD_FS (synthConf.bTxMode = 1) is necessary, 
   * followed by CMD_IEEE_TX and CMD_FS_OFF. This would not be used frequently, 
   * except maybe for testing, as Tx without CSMA-CA is only allowed for beacon 
   * frames and ACKs. 
   */
  (void) channel;
  return;
}


/*******************************************************************************
 * @fn          macGetCurrRssi
 *
 * @brief       On reception, the radio CPU shall read the current RSSI from CM0.
 *              CMD_IEEE_CCA_REQ should only be sent while an Rx or energy 
 *              detect scan operation is running.
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @param       Current RSSI.
 *
 * @return      None.
 */
uint8 macGetCurrRssi( void )
{
  requestCcaStateCmd_t macRequestCcaStateCmd;
    
  /* setup an immediate command CMD_IEEE_CCA_REQ */
  macRequestCcaStateCmd.cmdNum = CMD_IEEE_CCA_REQ;

  /* issue command */
  MB_SendCommand( (uint32)&macRequestCcaStateCmd );
  
  return( macRequestCcaStateCmd.currRssi );
}


/*******************************************************************************
 * @fn          macSetTxPowerVal
 *
 * @brief       On reception, the radio CPU shall set the TX power to 
 *              txPowerVal.
 *
 * input parameters
 *
* @param       txPowerVal - TX power value raw: IB, GC, (tempCoeff).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void macSetTxPowerVal( uint16 txPowerVal )
{
#if defined( CC26XX_PG1 )
  rfOpImmedCmd_RW_RfReg_t rfCmd;

  rfCmd.cmdNum   = CMD_WRITE_RFREG;
  rfCmd.address  = 0x6098; // RF24_RFE:TRIM5
  rfCmd.value    = (uint8)(txPowerVal & 0xFF);   
#elif defined( CC26XX )
  rfOpImmedCmd_SetTxPwr_t rfCmd;

  /* setup a radio command */
  rfCmd.cmdNum   = CMD_SET_TX_POWER;
  rfCmd.txPower  = txPowerVal;
  
  /* issue immediate command */
  MB_SendCommand( (uint32)&rfCmd );
#else /* unknown device */
  #error "ERROR: Unknown device!"
#endif /* CC26XX_PG1 */
 
  /* issue immediate command */
  MB_SendCommand( (uint32)&rfCmd );

  return;
}


/*******************************************************************************
 * @fn          macSetupMailbox
 *
 * @brief       This call is used to initialize the Mailbox, map interrupts to
 *              handle for IEEE 802.15.4, and register interrupt callbacks.
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
void macSetupMailbox( void )
{
  /* map interrupt handlers for CPE0, CPE1, and HW interrupts */
  MB_RegisterIsrCback( RF_CPE_0_INTERRUPT, cpe0IntCback );
  MB_RegisterIsrCback( RF_CPE_1_INTERRUPT, cpe1IntCback );
  MB_RegisterIsrCback( RF_HW_INTERRUPT, hwIntCback );

  /* setup the Mailbox CPE0, CPE1, and HW interrupts */
  MB_EnableInts( MB_COMMAND_DONE_INT | MB_FG_COMMAND_DONE_INT | MB_INTERNAL_ERROR,
                 MB_TX_DONE_INT | MB_TX_ACK_INT | MB_RX_OK_INT | MB_RX_NOK_INT | MB_RX_BUF_FULL_INT,
                 0 );

  return;
}


/*******************************************************************************
 * @fn          macSetupRfHal
 *
 * @brief       This call is used to initialize the RF HAL.
 *
 *              Note: Should be called before macSetupMailbox.
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
void macSetupRfHal( void )
{
  uint8_t frontEndMode    = ((macUserCfg_t *)macRadioConfig)->rfFeModeBias & 0x03;
  bool    singleEndedMode = (frontEndMode != RF_FE_DIFFERENTIAL) ? TRUE : FALSE;
    
  /* init the RF HAL */
  RFHAL_InitRfHal();
  
#if !defined( USE_FPGA )

  // Suppress COMMAND_DONE_INT interrupt.
  // Otherwise, patchRfCore() will stuck polling for the interrupt status.
  MB_RFCPEIEN_REG &= ~MB_COMMAND_DONE_INT;

  /* Patch CM0 */
  patchRfCore( singleEndedMode );
  
  // Re-enable the interrupt
  MB_RFCPEIEN_REG |= MB_COMMAND_DONE_INT;

#endif // !USE_FPGA 

  /* get a RAT channel for each desired function */
  if ( (macRatChanA = RFHAL_AllocRatChan()) == RAT_CHAN_INVALID )
  {
    HAL_ASSERT( FALSE );
  }

  /* register a handler for this RAT channel */
  RFHAL_RegisterRatChanCallback( macRatChanA, macRatChanCBack_A );
  
  /* get a RAT channel for each desired function */
  if ( (macRatChanB = RFHAL_AllocRatChan()) == RAT_CHAN_INVALID )
  {
    HAL_ASSERT( FALSE );
  }
  
  /* register a handler for this RAT channel */
  RFHAL_RegisterRatChanCallback( macRatChanB, macRatChanCBack_B );

  /* enable HW interruts that correspond to RAT channels */
  MB_EnableHWInts( RFHAL_MapRatChansToInt() );

  return;
}


/*******************************************************************************
 * @fn          macSetupRATChanCompare
 *
 * @brief       This call is used to setup a RAT channel compare that will
 *              generate a HW interrupt.
 *
 * input parameters
 *
 * @param       ratChan - RAT channel.
 * @param       compareTim - RAT Compare Time
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void macSetupRATChanCompare( uint8 ratChan, uint32 compareTime )
{
  rfOpImmedCmd_RatChanComp_t rfCmd;

  /* setup a radio command CMD_RADIO_SETUP */
  rfCmd.cmdNum   = CMD_SET_RAT_COMPARE;
  rfCmd.ratChan  = ratChan;
  rfCmd.compTime = compareTime;

  /* issue radio command */
  if ( MB_SendCommand( (uint32)&rfCmd ) != CMDSTA_DONE)
  {
#ifdef DEBUG_SW_TRACE
    uint32_t curvalue;
    curvalue = MAC_RAT_COUNT;
    DBG_PRINTL2(DBGSYS, "ratf %x %x", compareTime, curvalue);
#endif /* DEBUG_SW_TRACE */
    MAC_ASSERT(0);
  }

  /* ARM the RAT channel - no need to arm the channel according to CC26xx spec. */
  return;
}


/*******************************************************************************
 * @fn          macRatChanCBack_A Callback
 *
 * @brief       This callback is used to carry out the processing for the HW
 *              interrupt that corresponds to an allocated RAT channel.
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
void macRatChanCBack_A( void )
{
  /* call function for dealing with the timer period interrupt */
  macBackoffTimerPeriodIsr();

  return;
}


/*******************************************************************************
 * @fn          macRatChanCBack_B Callback
 *
 * @brief       This callback is used to carry out the processing for the HW
 *              interrupt that corresponds to an allocated RAT channel.
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
void macRatChanCBack_B( void )
{
  /* call function for dealing with the timer compare interrupt */
  macBackoffTimerCompareIsr();

  return;
}

/*******************************************************************************
 * @fn          macSetUserConfig
 *
 * @brief       This call is used to setup user radio configurations.
 *
 * input parameters
 *
 * @param       arg - pointer to user configuration from App.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void macSetUserConfig( void *arg )
{
  macUserCfg_t *userCfg; 
  
  userCfg = (macUserCfg_t *)arg;

  if ( userCfg != NULL )
  {
    /* RF Front End Mode and Bias (based on package) */
    macRadioConfig = userCfg;
  }  
}


/**************************************************************************************************
 * @fn          macCheckCommandDone
 *
 * @brief       Check and wait until the MAC radio command is completed.
 *
 * @param       rfOpCmd
 *
 * @return      none
 **************************************************************************************************
 */
void macCheckCommnadDone(rfOpCmd_t *rfOpCmd)
{
  /* Wait till previous radio command is complete before issuing another radio related
   * command which may fail if issued beforehand. */
  for (;;)
  {
    uint16 rxstatus = *(volatile uint16 *) &rfOpCmd->status;
    if (rxstatus != MACSTAT_ACTIVE &&
        rxstatus != MACSTAT_PENDING &&
        rxstatus != MACSTAT_SUSPENDED)
    {
      break;
    }
  }
}


#define BACKOFFS_PER_BASE_SUPERFRAME  (MAC_A_BASE_SLOT_DURATION * MAC_A_NUM_SUPERFRAME_SLOTS)
#if (((BACKOFFS_PER_BASE_SUPERFRAME - 1) & SLOTTED_TX_BACKOFF_COUNT_ALIGN_BIT_MASK) != SLOTTED_TX_BACKOFF_COUNT_ALIGN_BIT_MASK)
#error "ERROR!  The specified bit mask for backoff alignment of slotted transmit does not rollover 'cleanly'."
/*
 *  In other words, the backoff count for the number of superframe rolls over before the
 *  specified number of bits rollover.  For example, if backoff count for a superframe
 *  rolls over at 48, the binary number immediately before a rollover is 00101111.
 *  In this case four bits would work as an alignment mask.  Five would not work though as
 *  the lower five bits would go from 01111 to 00000 (instead of the value 10000 which
 *  would be expected) because it a new superframe is starting.
 */
#endif
#if (SLOTTED_TX_MAX_BACKOFF_COUNTDOWN_NUM_BITS < 2)
#error "ERROR!  Not enough backoff countdown bits to be practical."
#endif

/*******************************************************************************
 */


