/**************************************************************************************************
  Filename:       mac_mcu.c
  Revised:        $Date: 2014-09-15 14:13:20 -0700 (Mon, 15 Sep 2014) $
  Revision:       $Revision: 40150 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2006-2014 Texas Instruments Incorporated. All rights reserved.

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
#include "hal_defs.h"
#include "hal_mcu.h"
#include "hal_trng_wrapper.h"

/* low-level specific */
#include "mac_rx.h"
#include "mac_tx.h"
#include "mac_backoff_timer.h"
#include "mac_rx_onoff.h"
#include "mac_low_level.h"

/* target specific */
#include "mac_mcu.h"
#include "mac_radio_defs.h"

/* MAC */
#include "mb.h"
#include "mac.h"

/* debug */
#include "mac_assert.h"
   
/* Common ROM jump tables */
#include "R2F_CommonFlashJT.h"

#ifdef DEBUG_SW_TRACE
#define DBG_ENABLE
#include "dbgid_sys_mst.h"
#endif /* DEBUG_SW_TRACE */
   
/* ------------------------------------------------------------------------------------------------
 *                                           Defines
 * ------------------------------------------------------------------------------------------------
 */

/* for optimized indexing of uint32's */
#if HAL_MCU_LITTLE_ENDIAN()
#define UINT32_NDX0   0
#define UINT32_NDX1   1
#define UINT32_NDX2   2
#define UINT32_NDX3   3
#else
#define UINT32_NDX0   3
#define UINT32_NDX1   2
#define UINT32_NDX2   1
#define UINT32_NDX3   0
#endif

/* ------------------------------------------------------------------------------------------------
 *                                        Local Variables
 * ------------------------------------------------------------------------------------------------
 */

/* Function pointer for the random seed callback */
static macRNGFcn_t pRandomSeedCB = NULL;

/* ------------------------------------------------------------------------------------------------
 *                                       Local Prototypes
 * ------------------------------------------------------------------------------------------------
 */

void MAC_SetRandomSeedCB(macRNGFcn_t pCBFcn);
void macMcuRfErrIsr(void);
/**************************************************************************************************
 * @fn          MAC_SetRandomSeedCB
 *
 * @brief       Set the function pointer for the random seed callback.
 *
 * @param       pCBFcn - function pointer of the random seed callback
 *
 * @return      none
 **************************************************************************************************
 */
void MAC_SetRandomSeedCB(macRNGFcn_t pCBFcn)
{
  pRandomSeedCB = pCBFcn;
}

/**************************************************************************************************
 * @fn          macMcuInit
 *
 * @brief       Initialize the MCU.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macMcuInit(void)
{
 /*----------------------------------------------------------------------------------------------
  *  Initialize random seed value.
  */
  
  /* start RAT and use it for Random Number */
  MB_SendCommand( BUILD_DIRECT_CMD( CMD_START_RAT ) );

  /* Read 32 random bytes and store them in flash for future use in random
   * key generation for CBKE key establishment 
   */
  if ( pRandomSeedCB )
  {
    uint8 randomSeed[MAC_RANDOM_SEED_LEN];
    uint8 i;
    /* 32 random bytes read. */
    for ( i = 0;  i < MAC_RANDOM_SEED_LEN; i+= 4)
    {
      uint8 j;
      uint32 random32 = HalTRNG_GetTRNG();
      
      for ( j = 0; j < 4; ++j)
      {
        randomSeed[i + j] = BREAK_UINT32( random32, j );
      }
    }
    
    /* Store random bytes in flash. */
    pRandomSeedCB( randomSeed );
  }
    
  /* Turn on autoack */
  MAC_RADIO_TURN_ON_AUTO_ACK();

  /* Initialize SRCEXTPENDEN and SRCSHORTPENDEN to zeros */
  MAC_RADIO_SRC_MATCH_INIT_EXTPENDEN();
  MAC_RADIO_SRC_MATCH_INIT_SHORTPENDEN();
}


/**************************************************************************************************
 * @fn          macMcuRandomByte
 *
 * @brief       Returns a random byte using a special hardware feature that generates new
 *              random values based on the truly random seed set earlier.
 *
 * @param       none
 *
 * @return      a random byte
 **************************************************************************************************
 */
MAC_INTERNAL_API uint8 macMcuRandomByte(void)
{
  return (HalTRNG_GetTRNG() & 0xFF);
}


/**************************************************************************************************
 * @fn          macMcuRandomWord
 *
 * @brief       Returns a random word using a special hardware feature that generates new
 *              random values based on the truly random seed set earlier.
 *
 * @param       none
 *
 * @return      a random word
 **************************************************************************************************
 */
MAC_INTERNAL_API uint16 macMcuRandomWord(void)
{
  return (HalTRNG_GetTRNG() & 0xFFFF);
}


/**************************************************************************************************
 * @fn          macMcuTimerCapture
 *
 * @brief       Returns the last timer capture.  This capture should have occurred at the
 *              receive time of the last frame (the last time SFD transitioned to active).
 *
 * @param       timeStamp - time stamp from RX or TX frames
 *
 * @return      last capture of hardware timer (full 16-bit value)
 **************************************************************************************************
 */
MAC_INTERNAL_API uint16 macMcuTimerCapture(uint32 timeStamp)
{
  /* The time, in RAT ticks, at which the frame was received */
  return( ( timeStamp - macPrevPeriodRatCount ) % MAC_BACKOFF_TO_RAT_RATIO );
}


/**************************************************************************************************
 * @fn          macMcuOverflowCapture
 *
 * @brief       Returns the last capture of the overflow counter.  A special hardware feature
 *              captures the overflow counter when the regular hardware timer is captured.
 *
 * @param       timeStamp - time stampe from RX or TX frames
 *
 * @return      last capture of overflow count
 **************************************************************************************************
 */
MAC_INTERNAL_API uint32 macMcuOverflowCapture(uint32 timeStamp)
{
  /* The time, in aUnitBackoffPeriod units, at which the frame was sent. */
  return( ( timeStamp - macPrevPeriodRatCount ) / MAC_BACKOFF_TO_RAT_RATIO );
}


/**************************************************************************************************
 * @fn          macMcuOverflowSetCount
 *
 * @brief       Sets the value of the hardware overflow counter.
 *
 * @param       count - new overflow count value
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macMcuOverflowSetCount(uint32 count)
{
  /* Set the next RAT count. This is equivalent to zero out the RAT count 
   * and then set a new count.
   */
  MAC_RAT_ADJ = count * MAC_BACKOFF_TO_RAT_RATIO - MAC_RAT_COUNT; 
}


/**************************************************************************************************
 * @fn          macMcuOverflowSetCompare
 *
 * @brief       Set overflow count compare value.  An interrupt is triggered when the overflow
 *              count equals this compare value.
 *
 * @param       count - overflow count compare value
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macMcuOverflowSetCompare(uint32 count)
{
  uint32 compare = macPrevPeriodRatCount + count * MAC_BACKOFF_TO_RAT_RATIO;
  uint32 current;

  MAC_ASSERT( count <= MAC_BACKOFF_MAXIMUM_ROLLOVER );
  
  /* Note that critical section is not entered in lead time computation
   * since only in critical section of a caller,
   * the RAT compare might be close to the current RAT counter. */
  current = MAC_RAT_COUNT;
  if (((compare - current - 100 * MAC_RAT_MHZ) & 0x80000000) &&
      ((current - compare - MAC_BACKOFF_PAST_WINDOW) & 0x80000000))
  {
    /* Either time in the past for a second window
     * or not enough lead room to handle race condition. */
    compare = current + 100*MAC_RAT_MHZ;
  }

  /* Convert count to relative RAT count and set MAC Channel B Compare. */
  macSetupRATChanCompare( macRatChanB, compare );
}


/**************************************************************************************************
 * @fn          macMcuOverflowSetPeriod
 *
 * @brief       Set overflow count period value.  An interrupt is triggered when the overflow
 *              count equals this period value.  
 *
 * @param       count - overflow count compare value
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macMcuOverflowSetPeriod(uint32 count)
{
  uint32 compare, current;
  MAC_ASSERT( count <= MAC_BACKOFF_MAXIMUM_ROLLOVER );
 
  /* Note that critical section is not entered in lead time computation
   * since only in critical section of a caller,
   * the RAT compare might be close to the current RAT counter. */
  compare = count * MAC_BACKOFF_TO_RAT_RATIO + macPrevPeriodRatCount;
  current = MAC_RAT_COUNT;
  if (((compare - current - 100 * MAC_RAT_MHZ) & 0x80000000) &&
      ((current - compare - MAC_BACKOFF_PAST_WINDOW) & 0x80000000))
  {
    /* Either time in the past for a second window
     * or not enough lead room to handle race condition. */
    compare = current + 100*MAC_RAT_MHZ;
  }
  /* Convert count to relative RAT count and set MAC Channel A Compare */
  macSetupRATChanCompare( macRatChanA, compare );
}


/**************************************************************************************************
 * @fn          macMcuPrecisionCount
 *
 * @brief       This function is used by higher layer to read a free running counter driven by
 *              MAC timer.
 *
 * @param       none
 *
 * @return      MSW of MAC_RAT_COUNT
 **************************************************************************************************
 */
uint32 macMcuPrecisionCount(void)
{
  return ((uint16)(MAC_RAT_COUNT & 0xFFFF));
}


/**************************************************************************************************
 * @fn          macMcuRfErrIsr
 *
 * @brief       Interrupt service routine that handles all RF Error interrupts.  Only the RX FIFO
 *              overflow condition is handled.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macMcuRfErrIsr(void)
{
  // TDOD: Add error handling and call macRxFifoOverflowIsr()
}


/**************************************************************************************************
 * @fn          macMcuRecordMaxRssiStart
 *
 * @brief       Starts recording of the maximum received RSSI value.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macMcuRecordMaxRssiStart(void)
{
  MB_SendCommand( BUILD_DIRECT_CMD(CMD_ABORT) );
  
#if defined( CC26XX )  
  /* 
   * PG1 radio bug workaround - RSSIMAXVAL never resets automatically.
   */
  macWriteRfRegCmd(0x605C, (uint32) -128 );
  DBG_PRINT0(DBGSYS, "macMcuRecordMaxRssiStart() - Reset RSSIMAXVAL to -128");
#endif /* CC26XX */
  
  /* Setup and start MAC scan command */
  macSetupEdCmd();
  macSendEdCmd();
}


/**************************************************************************************************
 * @fn          macMcuRecordMaxRssiStop
 *
 * @brief       Stops recording of the maximum received RSSI.  It returns the maximum value
 *              received since starting the recording.
 *
 * @param       none
 *
 * @return      maximum received RSSI value
 **************************************************************************************************
 */
MAC_INTERNAL_API int8 macMcuRecordMaxRssiStop(void)
{
  DBG_PRINT1(DBGSYS, "macMcuRecordMaxRssiStop(), maxRssi=0x%X", macRxEdScan.edCmd.maxRssi);
  return(macRxEdScan.edCmd.maxRssi);
}


/**************************************************************************************************
 *                                  Compile Time Integrity Checks
 **************************************************************************************************
 */

#if defined (FEATURE_CC253X_LOW_POWER_RX) && !(defined (HAL_MCU_CC2530) || defined (HAL_MCU_CC2533))
#error "ERROR: FEATURE_CC253X_LOW_POWER_RX can only be used with CC2530 or CC2533."
#endif

/**************************************************************************************************
*/
