/**************************************************************************************************
  Filename:       mac_backoff_timer.c
  Revised:        $Date: 2015-02-17 14:23:30 -0800 (Tue, 17 Feb 2015) $
  Revision:       $Revision: 42685 $

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
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */

/* hal */
#include "hal_types.h"
#include "hal_mcu.h"

/* high-level specific */
#include "mac_spec.h"

/* exported low-level */
#include "mac_low_level.h"

/* low-level specific */
#include "mac_backoff_timer.h"
#include "mac_tx.h"
#include "mac_rx_onoff.h"

/* target specific */
#include "mac_radio_defs.h"

/* mac mcu */
#include "mac_mcu.h"

#include "mb.h"
#include "mac.h"

/* debug */
#include "mac_assert.h"

/* access PIB */
#include "mac_pib.h"

#ifdef DEBUG_SW_TRACE
#define DBG_ENABLE
#include "dbgid_sys_mst.h"
#endif /* DEBUG_SW_TRACE */

#ifdef USE_ICALL
#include <ICall.h>
#include <ICallCC26xxDefs.h>
#include <hw_rfc_pwr.h>
#include <OSAL.h>
#include "mac_radio.h"
#endif /* USE ICALL */

#ifdef OSAL_PORT2TIRTOS
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <ti/sysbios/knl/Clock.h>
#include <hw_rfc_pwr.h>
#include <OSAL.h>
#include "mac_radio.h"
extern uint8 macTaskId;
#endif /* OSAL_PORT2TIRTOS */

#include "R2R_FlashJT.h"
#include "R2F_FlashJT.h"

/* ------------------------------------------------------------------------------------------------
 *                                            Defines
 * ------------------------------------------------------------------------------------------------
 */
#define COMPARE_STATE_ROLLOVER_BV                 BV(0)
#define COMPARE_STATE_TRIGGER_BV                  BV(1)
#define COMPARE_STATE_ARM_BV                      BV(2)

#define COMPARE_STATE_TRIGGER                     COMPARE_STATE_TRIGGER_BV
#define COMPARE_STATE_ROLLOVER                    COMPARE_STATE_ROLLOVER_BV
#define COMPARE_STATE_ROLLOVER_AND_TRIGGER        (COMPARE_STATE_ROLLOVER_BV | COMPARE_STATE_TRIGGER_BV)
#define COMPARE_STATE_ROLLOVER_AND_ARM_TRIGGER    (COMPARE_STATE_ROLLOVER_BV | COMPARE_STATE_ARM_BV)


/*
 *  The datasheet mentions a small delay on both receive and transmit side of approximately
 *  two microseconds.  The precise characterization is given below.
 *  (This data is not given in rev 1.03 datasheet)
 */
#define RX_TX_PROP_DELAY_AVG_USEC         ((MAC_RADIO_RX_TX_PROP_DELAY_MIN_USEC + MAC_RADIO_RX_TX_PROP_DELAY_MAX_USEC) / 2)
#define RX_TX_PROP_DELAY_AVG_TIMER_TICKS  ((uint16)(MAC_RADIO_TIMER_TICKS_PER_USEC() * RX_TX_PROP_DELAY_AVG_USEC + 0.5))

/*
 *  For slotted receives, the SFD signal is expected to occur on a specifc symbol boundary.
 *  This does *not* correspond to the backoff boundary.  The SFD signal occurs at an
 *  offset from the backoff boundary.  This is done for efficiency of related algorithms.
 *
 *  Once transmit is strobed there is a fixed delay until the SFD signal occurs.  The frame
 *  does not start over-the-air transmit until after an internal radio delay of 12 symbols.
 *  Once transmitting over-the-air, the preamble is sent (8 symbols) followed by the
 *  SFD field (2 symbols). After the SFD field completes, the SFD signal occurs.  This
 *  adds up to a total of 22 symbols from strobe to SFD signal.
 *
 *  Since 22 symbols spans more than a backoff (20 symbols) the modulus operation is used
 *  to find the symbol offset which is 2 symbols.
 *
 *  This math is derived formally via the pre-processor.
 */
#define SYMBOLS_FROM_STROBE_TO_PREAMBLE   12 /* from datasheet */
#define SYMBOLS_FROM_PREAMBLE_TO_SFD      (MAC_SPEC_PREAMBLE_FIELD_LENGTH + MAC_SPEC_SFD_FIELD_LENGTH)
#define SYMBOLS_FROM_STROBE_TO_SFD        (SYMBOLS_FROM_STROBE_TO_PREAMBLE + SYMBOLS_FROM_PREAMBLE_TO_SFD)
#define SYMBOLS_EXPECTED_AT_SFD           (SYMBOLS_FROM_STROBE_TO_SFD % MAC_A_UNIT_BACKOFF_PERIOD)

/* after all that formal math, make sure the result is as expected */
#if (SYMBOLS_EXPECTED_AT_SFD != 2)
#error "ERROR! Internal problem with pre-processor math of slotted alignment."
#endif


/*
 *  The expected SFD signal occurs at the symbol offset *plus* a small internal propagation delay
 *  internal to the radio.  This delay is given as the sum of a receive side delay and a transmit
 *  side delay.  When this delay is subtracted from the internal timer, the internal time base
 *  actually becomes the actual receive time *minus* the transmit delay.  This works out though.
 *  The transmit logic does *not* take into account this delay.  Since the timer is skewed by the
 *  transmit delay already, the transmits go out precisely on time.
 */
#define TIMER_TICKS_EXPECTED_AT_SFD   ((SYMBOLS_EXPECTED_AT_SFD * MAC_RADIO_TIMER_TICKS_PER_SYMBOL()) \
                                          + RX_TX_PROP_DELAY_AVG_TIMER_TICKS)

#if defined USE_ICALL || defined OSAL_PORT2TIRTOS

#ifndef MAC_BACKOFF_TIMER_ADDITIONAL_WAKEUP_LATENCY
#if defined( CC26XX_PG1 )
/** Additional wakeup latency in term of microseconds */
#define MAC_BACKOFF_TIMER_ADDITIONAL_WAKEUP_LATENCY    4600
#elif defined( CC26XX )
/** Additional wakeup latency in term of microseconds */
#define MAC_BACKOFF_TIMER_ADDITIONAL_WAKEUP_LATENCY    2300
#else /* unknown device */
  #error "ERROR: Unknown device!"
#endif /* CC26XX_PG1 */

#endif /* USE_ICALL || OSAL_PORT2TIRTOS */

#define MAC_BACKOFF_TIMER_UPDATE_WAKEUP() macBackoffTimerUpdateWakeup()

/* Synchronization events */
#define MAC_BACKOFF_TIMER_EVENT_POWER_WAKEUP              0x01
#define MAC_BACKOFF_TIMER_EVENT_POWER_TIMER_EXP           0x02

#else /* defined USE_ICALL || defined OSAL_PORT2TIRTOS */
#define MAC_BACKOFF_TIMER_UPDATE_WAKEUP()
#endif /* defined USE_ICALL || defined OSAL_PORT2TIRTOS */

/* ------------------------------------------------------------------------------------------------
 *                                         External Variables
 * ------------------------------------------------------------------------------------------------
 */
extern uint32 CODE macTimerRolloverValue[];

/* ------------------------------------------------------------------------------------------------
 *                                         Global Variables
 * ------------------------------------------------------------------------------------------------
 */
uint32 macBackoffTimerRollover;
uint32 macPrevPeriodRatCount;
#if defined USE_ICALL || defined OSAL_PORT2TIRTOS
uint8 macBackoffTimerImpending;
#endif /* defined USE_ICALL || defined OSAL_PORT2TIRTOS */


/* ------------------------------------------------------------------------------------------------
 *                                         Local Variables
 * ------------------------------------------------------------------------------------------------
 */
static uint32 backoffTimerTrigger;

#ifdef USE_ICALL
/* ICall timer ID used to replicate backoff timer to affect power module */
static ICall_TimerID macBackoffTimerICallTimerID = ICALL_INVALID_TIMER_ID;

/* Data for power state transition notify function registration */
static ICall_PwrNotifyData macBackoffTimerICallPwrNotifyData;
#elif defined OSAL_PORT2TIRTOS
static Power_NotifyObj macBackoffPwrNotifyObj;
static Clock_Handle macBackoffWakeupClock = NULL;
#endif /* defined OSAL_PORT2TIRTOS */

#if defined USE_ICALL || defined OSAL_PORT2TIRTOS

/* RAT value stored from RAT stop command */
static uint32 macRATValue;

/* Synchronization events */
static uint8 macBackoffTimerEvents;

/* ------------------------------------------------------------------------------------------------
 *                                         Forward References
 * ------------------------------------------------------------------------------------------------
 */
static void macBackoffTimerUpdateWakeup(void);

/**************************************************************************************************
 * @fn          macBackoffTimerEventHandler
 *
 * @brief       backoff timer synchronized event handler function
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
#ifdef USE_ICALL
static
#endif /* USE_ICALL */
void macBackoffTimerEventHandler(void)
{
  halIntState_t is;
  uint8 events;
  HAL_ENTER_CRITICAL_SECTION(is);
  events = macBackoffTimerEvents;
  macBackoffTimerEvents = 0;
  HAL_EXIT_CRITICAL_SECTION(is);

  if (events & MAC_BACKOFF_TIMER_EVENT_POWER_WAKEUP)
  {
    // Wakeup radio
    // Turning on radio domain before clock set up seems to cause
    // unexpected interrupt.
    // Hence interrupt shall be disabled here.
    MB_DisableInts();

    // Enable clocks for all radio internal modules.
    // Use Non-Buff access for safety and check for sanity
    HWREG(RFC_PWR_NONBUF_BASE + RFC_PWR_O_PWMCLKEN) = 0x7FF;

    /* Setup mailbox */
    macSetupMailbox();

#ifdef DEBUG_SW_TRACE
    /* re-enable RF trace output for FPGA */
    MB_SendCommand( BUILD_DIRECT_PARAM_EXT_CMD( CMD_ENABLE_DEBUG, 0x1D40 ) ); /* or 0x1940 for less trace */
    DBG_PRINT0(DBGSYS, "RF Trace Resumes...");
#endif /* DEBUG_SW_TRACE */

    /* Start off CM0. Patch it. */
    macSetupRfHal();

    /* Restore states */
    MAC_RADIO_SET_CHANNEL(macPhyChannel);
    MAC_RADIO_SET_PAN_COORDINATOR(macPanCoordinator);
    MAC_RADIO_SET_PAN_ID(pMacPib->panId);
    MAC_RADIO_SET_SHORT_ADDR(pMacPib->shortAddress);
    MAC_RADIO_SET_IEEE_ADDR(pMacPib->extendedAddress.addr.extAddr);

#if !defined( USE_FPGA )
#ifdef USE_ICALL
    // Switch back to HFOSC.
    while (!ICall_pwrIsStableXOSCHF());
    ICall_pwrSwitchXOSCHF();
#endif /* USE_ICALL */
#ifdef OSAL_PORT2TIRTOS
    // Switches back to HFOSC.
    while (!Power_isStableXOSC_HF());
    Power_switchXOSC_HF();
#endif /* OSAL_PORT2TIRTOS */
#endif /* !defined( USE_FPGA ) */

    /* Synchronize RAT timer */
    macSyncStartRAT(macRATValue);

    /* Turn on autoack */
    MAC_RADIO_TURN_ON_AUTO_ACK();

    /* Initialize SRCEXTPENDEN and SRCSHORTPENDEN to zeros */
    MAC_RADIO_SRC_MATCH_INIT_EXTPENDEN();
    MAC_RADIO_SRC_MATCH_INIT_SHORTPENDEN();

    /* Start 15.4 Radio */
    macSetupRadio();

    /* Restore timer comparators */
    MAC_RADIO_BACKOFF_SET_PERIOD(macBackoffTimerRollover);
    MAC_RADIO_BACKOFF_SET_COMPARE(backoffTimerTrigger);

#if 0 /* Following code should be disabled normally */
    /* Code for wakeup lead time calibration */
    {
      static uint32 macBackoffTimerMinMargin = 0xffffffffu;
      uint32 delta = macPrevPeriodRatCount +
        backoffTimerTrigger * MAC_BACKOFF_TO_RAT_RATIO - MAC_RAT_COUNT;
      if (delta < macBackoffTimerMinMargin)
      {
        macBackoffTimerMinMargin = delta;
      }
    }
#endif
  }

  /* Note that MAC_BACKOFF_TIMER_EVENT_POWER_TIMER_EXP handling must always
   * occur after handling of MAC_BACKOFF_TIMER_EVENT_POWER_WAKEUP event
   * because the device might be waking up upon the timer event itself
   * in which case, radio has to be turned on before updating the RAT timer.
   */
  if (events & MAC_BACKOFF_TIMER_EVENT_POWER_TIMER_EXP)
  {
    /* Update wakeup schedule, which most likely would vote not to enter
     * sleep state. */
    HAL_ENTER_CRITICAL_SECTION(is);
    MAC_BACKOFF_TIMER_UPDATE_WAKEUP();
    HAL_EXIT_CRITICAL_SECTION(is);
  }
}

/**************************************************************************************************
 * @fn          macBackoffTimerICallTimerCback
 *
 * @brief       ICall timer callback function
 *
 * @param       arg   meaningless
 *
 * @return      none
 **************************************************************************************************
 */
static void macBackoffTimerICallTimerCback(void *arg)
{
  /* Timer must be synchronized to the potential handling of
   * wakeup from sleep state in case the timer callback is made
   * as soon as the device wakes up before radio is turned back on.
   * Hence, signal the event to the OSAL thread. */
  halIntState_t is;
  HAL_ENTER_CRITICAL_SECTION(is);
  macBackoffTimerEvents |= MAC_BACKOFF_TIMER_EVENT_POWER_TIMER_EXP;
  HAL_EXIT_CRITICAL_SECTION(is);
#ifdef USE_ICALL
  ICall_signal(osal_semaphore);
#endif /* USE_ICALL */
#ifdef OSAL_PORT2TIRTOS
  osal_set_event(macTaskId, 0);
#endif /* OSAL_PORT2TIRTOS */
}

#ifdef USE_ICALL
/**************************************************************************************************
 * @fn          macBackoffTimerICallPwrNotify
 *
 * @brief       power state transition notify callback function
 *
 * @param       pwrTrans  power transition
 * @param       data      custom data - not used
 *
 * @return      none
 **************************************************************************************************
 */
static void macBackoffTimerICallPwrNotify(ICall_PwrTransition pwrTrans,
                                          ICall_PwrNotifyData *data)
{
  if (pwrTrans == ICALL_PWR_AWAKE_FROM_STANDBY)
  {
    /* Wakeup must be handled from the thread context.
     * Signal the event to the OSAL thread. */
    halIntState_t is;

    HAL_ENTER_CRITICAL_SECTION(is);
    macBackoffTimerEvents |= MAC_BACKOFF_TIMER_EVENT_POWER_WAKEUP;
    HAL_EXIT_CRITICAL_SECTION(is);
    ICall_signal(osal_semaphore);
  }
  else if (pwrTrans == ICALL_PWR_ENTER_STANDBY)
  {
    /* Stop RAT timer */
    macRATValue = macStopRAT();

    /* Park CM0 */
    MAC_RADIO_POWER_DOWN();

    /* The following calls are necessary to prevent a race condition in 
     * pg2_leakage_workaround that causes CM3 to constantly firing up CPE1
     * interrupts during power up until CM3 crashes.
     */
    ICall_disableInt( INT_RF_CPE0 );
    ICall_disableInt( INT_RF_CPE1 );
    ICall_disableInt( INT_RF_HW );
    ICall_disableInt( INT_RF_CMD_ACK );
  }
  else if (pwrTrans == ICALL_PWR_ENTER_SHUTDOWN)
  {
    /* Park CM0 */
    MAC_RADIO_POWER_DOWN();
  }
}
#endif /* USE_ICALL */

#ifdef OSAL_PORT2TIRTOS
/**************************************************************************************************
 * @fn          macBackoffTimerPwrNotify
 *
 * @brief       power state transition notify callback function
 *
 * @param       eventType transition event type
 * @param       data      not used
 *
 * @return      Power_NOTIFYDONE
 **************************************************************************************************
 */
static Power_NotifyResponse macBackoffTimerPwrNotify(Power_Event eventType,
                                                     UArg data)
{
  if (eventType == Power_AWAKE_STANDBY)
  {
    /* Wakeup must be handled from the thread context.
     * Signal the event to the OSAL thread. */
    halIntState_t is;

    HAL_ENTER_CRITICAL_SECTION(is);
    macBackoffTimerEvents |= MAC_BACKOFF_TIMER_EVENT_POWER_WAKEUP;
    HAL_EXIT_CRITICAL_SECTION(is);
    osal_set_event(macTaskId, 0);
  }
  else if (eventType == Power_ENTERING_STANDBY)
  {
    /* Stop RAT timer */
    macRATValue = macStopRAT();

    /* Park CM0 */
    MAC_RADIO_POWER_DOWN();

    Hwi_disableInterrupt( INT_RF_CPE0 );
    Hwi_disableInterrupt( INT_RF_CPE1 );
    Hwi_disableInterrupt( INT_RF_HW );
    Hwi_disableInterrupt( INT_RF_CMD_ACK );
  }
  else if (eventType == Power_ENTERING_SHUTDOWN)
  {
    /* Park CM0 */
    MAC_RADIO_POWER_DOWN();
  }
  return Power_NOTIFYDONE;
}
#endif /* OSAL_PORT2TIRTOS */

/**************************************************************************************************
 * @fn          macBackoffTimerUpdateWakeup
 *
 * @brief       Recomputes and provides weakup timing hint to power module.
 *              This function does not read timing variables in a critical
 *              section. The caller must call this function within a critical
 *              section where the timing variable is modified in order
 *              to have more accurate notification.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void macBackoffTimerUpdateWakeup(void)
{
  int_fast64_t ticks;

  /* Replicate the timer in order to inform power module to wake up properly
   * in case the device enters sleep state in the future. */

  /* First find out which RAT channel between backoffTimerTrigger and
   * macBackoffTimerRollover is supposed to expire next. */
  ticks = MAC_RADIO_BACKOFF_COUNT();

  if (backoffTimerTrigger > ticks &&
      backoffTimerTrigger < macBackoffTimerRollover)
  {
    ticks = backoffTimerTrigger - ticks;
  }
  else if (macBackoffTimerRollover > ticks)
  {
    ticks = macBackoffTimerRollover - ticks;
  }
  else
  {
    /* Note that current count might have exceeded already set thresholds,
     * as the code is executed while interrupt is flagged.
     * In such a case, this function shall be called again anyways and
     * hence this condition can be ignored. */
    return;
  }

  /* TODO: For subG, backoff timer unit is not necessarily backoff period
   *       but fixed 320us and hence the following constant
   *       (MAC_SPEC_USECS_PER_BACKOFF) should be replaced with backoff
   *       timer unit period in usecs. */

  /* Convert backoff timer unit to RTOS tick unit */
  ticks *= MAC_SPEC_USECS_PER_BACKOFF;

#ifdef USE_ICALL
  ticks -= (MAC_BACKOFF_TIMER_ADDITIONAL_WAKEUP_LATENCY + ICall_pwrGetXOSCStartupTime(ticks/1000));
#elif defined OSAL_PORT2TIRTOS
  ticks -= MAC_BACKOFF_TIMER_ADDITIONAL_WAKEUP_LATENCY +
           Power_getXoscStartupTime(ticks/1000);
#endif /* defined OSAL_PORT2TIRTOS */

  if (ticks > 0)
  {
#ifdef USE_ICALL
    ticks /= osal_tickperiod;
#elif defined OSAL_PORT2TIRTOS
    ticks /= Clock_tickPeriod;
#endif  /* defined OSAL_PORT2TIRTOS */
  }

  if (ticks > 0)
  {
#ifdef USE_ICALL
    ICall_setTimer((uint_fast32_t) ticks, macBackoffTimerICallTimerCback,
                   NULL, &macBackoffTimerICallTimerID);
#endif /* USE_ICALL */
#ifdef OSAL_PORT2TIRTOS
    Clock_stop(macBackoffWakeupClock);
    Clock_setTimeout(macBackoffWakeupClock, (UInt32) ticks);
    Clock_start(macBackoffWakeupClock);
#endif /* OSAL_PORT2TIRTOS */

    /* Allow entering sleep state */
    macBackoffTimerImpending = FALSE;
#ifdef DEBUG_SW_TRACE
    DBG_PRINTL1(DBGSYS, "BO %u", ticks);
#endif /* DEBUG_SW_TRACE */
  }
  else
  {
    /* Timing is too close. Suppress sleep. */
    macBackoffTimerImpending = TRUE;
  }
  macPwrVote();
}

/**************************************************************************************************
 * @fn          macBackoffSetupPwrMgmt
 *
 * @brief       Intializes backoff timer power management logic.
 *              Note that this function must be called only once.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macBackoffSetupPwrMgmt(void)
{
#ifdef USE_ICALL
  /* Register power transition notification */
  if (ICall_pwrRegisterNotify(macBackoffTimerICallPwrNotify,
                              &macBackoffTimerICallPwrNotifyData) !=
      ICALL_ERRNO_SUCCESS)
  {
    MAC_ASSERT(0);
  }
#endif /* USE_ICALL */
#ifdef OSAL_PORT2TIRTOS
  /* Register power transition notification */
  Power_registerNotify(&macBackoffPwrNotifyObj,
                       (Power_ENTERING_STANDBY |
                        Power_ENTERING_SHUTDOWN |
                        Power_AWAKE_STANDBY),
                       (xdc_Fxn) macBackoffTimerPwrNotify,
                       (UArg) NULL, 0);
#endif /* OSAL_PORT2TIRTOS */
}
#endif /* defined USE_ICALL || defined OSAL_PORT2TIRTOS */


/**************************************************************************************************
 * @fn          macBackoffTimerInit
 *
 * @brief       Intializes backoff timer.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macBackoffTimerInit(void)
{
  macPrevPeriodRatCount = macBackoffTimerRollover = 0;

  /* backoffTimerTrigger has to be set to maximum possible value of
   * macBackoffTimerRollover value initially.
   * Otherwise, incorrect backoffTimerTrigger value shall be compared
   * all the time in macBackoffTimerUpdateWakeup() function, casting
   * incorrect vote.
   */
  backoffTimerTrigger = MAC_BACKOFF_TIMER_DEFAULT_NONBEACON_ROLLOVER;

  MAC_RADIO_CLEAR_BACKOFF_COUNT();
#if defined USE_ICALL || defined OSAL_PORT2TIRTOS
  // Clear events
  macBackoffTimerEvents = 0;

#ifdef USE_ICALL
  // Register hook function to handle events.
  if (!osal_eventloop_hook)
  {
    /* Don't overwrite if the hook is already set up.
     * Note that the hook might have been set up to perform other things
     * as well in which case the other hook function has to call
     * macBackoffTimerEventHandler.
     */
    osal_eventloop_hook = macBackoffTimerEventHandler;
  }
#endif /* USE_ICALL */

  macBackoffTimerImpending = FALSE;

#ifdef USE_ICALL
  /* Start timer just to initialize the timer ID to reuse in the module.
   * This also serves the purpose of allocating resources upfront,
   * in order to prevent a case of running out of timer resource
   * when the timer has to be started.
   * Note that macBackoffTimerSetRollover() may trigger setting timer
   * and hence the timer set up must happen before macBackoffTimerSetRollover()
   * call. */
  if (ICall_setTimer(1, macBackoffTimerICallTimerCback, NULL,
                     &macBackoffTimerICallTimerID) != ICALL_ERRNO_SUCCESS)
  {
    MAC_ASSERT(0);
  }
#endif /* USE_ICALL */
#ifdef OSAL_PORT2TIRTOS
  if (!macBackoffWakeupClock)
  {
    /* Creates a wakeup clock */
    Clock_Params params;
    Clock_Params_init(&params);
    params.startFlag = FALSE;
    params.period = 0;
    macBackoffWakeupClock =
      Clock_create((Clock_FuncPtr) macBackoffTimerICallTimerCback,
                   1, &params, NULL);
    MAC_ASSERT(macBackoffWakeupClock);
    /* No need to stop clock, the clock event will reprogram next wake time */
  }
#endif /* OSAL_PORT2TIRTOS */

  /* Note that macPwrVote() is called done from macBackoffTimerSetRollover()
   * call and hence there is no need to make the call here. */
#endif /* defined USE_ICALL || defined OSAL_PORT2TIRTOS */

  macBackoffTimerSetRollover(MAC_BACKOFF_TIMER_DEFAULT_NONBEACON_ROLLOVER);

  /* Since interrupt disable/enable mechanism is not implemented for backoff
   * timer trigger interrupt for CC26xx, comparator value has to be set
   * so that the timer trigger interrupt is not triggered.
   * See comment inside macBackoffTimerCancelTrigger() for relevant
   * information */
  MAC_RADIO_BACKOFF_SET_COMPARE(backoffTimerTrigger);
}


/**************************************************************************************************
 * @fn          macBackoffTimerReset
 *
 * @brief       Resets backoff timer.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macBackoffTimerReset(void)
{
  macBackoffTimerInit();
}


/**************************************************************************************************
 * @fn          macBackoffTimerSetRollover
 *
 * @brief       Set rollover count of backoff timer. 
 *
 * @param       rolloverBackoff - backoff count where count is reset to zero
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macBackoffTimerSetRollover(uint32 rolloverBackoff)
{
  halIntState_t  s;

  /* Normally called on initialization but timer realign also calls this. */
  MAC_ASSERT(rolloverBackoff > MAC_RADIO_BACKOFF_COUNT());  /* rollover value must be greater than count */
  DBG_PRINTL1(DBGSYS, "MAC_RADIO_BACKOFF_SET_PERIOD(%u)", rolloverBackoff);
  
  HAL_ENTER_CRITICAL_SECTION(s);
  macBackoffTimerRollover = rolloverBackoff;
  MAC_RADIO_BACKOFF_SET_PERIOD(rolloverBackoff);
  MAC_BACKOFF_TIMER_UPDATE_WAKEUP();
  HAL_EXIT_CRITICAL_SECTION(s);
}


/**************************************************************************************************
 * @fn          macBackoffTimerSetCount
 *
 * @brief       Sets the count of the backoff timer.
 *
 * @param       backoff - new count
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macBackoffTimerSetCount(uint32 backoff)
{
#if defined( FEATURE_BEACON_MODE )
  halIntState_t  s;

  MAC_ASSERT(backoff < macBackoffTimerRollover);  /* count must be less than rollover value */
  MAC_ASSERT(!(backoff & 0x80000000));  /* count must not represent negative value for int32 */
  DBG_PRINT2(DBGSYS, "MAC_RADIO_BACKOFF_SET_COUNT(%u), RAT BACKOFF = %u", backoff, MAC_RADIO_BACKOFF_COUNT());

  HAL_ENTER_CRITICAL_SECTION(s);
  MAC_RADIO_BACKOFF_SET_COUNT(backoff);
  MAC_BACKOFF_TIMER_UPDATE_WAKEUP();
  HAL_EXIT_CRITICAL_SECTION(s);
#endif /* FEATURE_BEACON_MODE */  
}


/**************************************************************************************************
 * @fn          macBackoffTimerCount
 *
 * @brief       Returns the current backoff count.
 *
 * @param       none
 *
 * @return      current backoff count
 **************************************************************************************************
 */
MAC_INTERNAL_API uint32 macBackoffTimerCount(void)
{
  halIntState_t  s;
  uint32 backoffCount;
  
  HAL_ENTER_CRITICAL_SECTION(s);
  backoffCount = MAC_RADIO_BACKOFF_COUNT();
  HAL_EXIT_CRITICAL_SECTION(s);  

  return(backoffCount);
}


/**************************************************************************************************
 * @fn          macBackoffTimerGetTrigger
 *
 * @brief       Returns the trigger set for the backoff timer.
 *
 * @param       none
 *
 * @return      backoff count of trigger
 **************************************************************************************************
 */
MAC_INTERNAL_API uint32 macBackoffTimerGetTrigger(void)
{
  return(backoffTimerTrigger);
}


/**************************************************************************************************
 * @fn          macBackoffTimerSetTrigger
 *
 * @brief       Sets the trigger count for the backoff counter.  A callback is exectuted when
 *              the backoff count reaches the trigger
 *
 * @param       triggerBackoff - backoff count for new trigger
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macBackoffTimerSetTrigger(uint32 triggerBackoff)
{
  halIntState_t  s;

  MAC_ASSERT(triggerBackoff < macBackoffTimerRollover); /* trigger backoff must be less than rollover backoff */
  DBG_PRINT1(DBGSYS, "MAC_RADIO_BACKOFF_SET_COMPARE(%u)", triggerBackoff);
  
  HAL_ENTER_CRITICAL_SECTION(s);
  backoffTimerTrigger = triggerBackoff;
  MAC_RADIO_BACKOFF_SET_COMPARE(triggerBackoff);
  MAC_BACKOFF_TIMER_UPDATE_WAKEUP();
  HAL_EXIT_CRITICAL_SECTION(s);
}


/**************************************************************************************************
 * @fn          macBackoffTimerCancelTrigger
 *
 * @brief       Cancels the trigger for the backoff counter - obselete for CC253x and CC26xx.
 *              For CC253x and CC26xx, the timer trigger should never be late, therefore, no
 *              need to cancel.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macBackoffTimerCancelTrigger(void)
{
  MAC_RADIO_BACKOFF_COMPARE_CLEAR_INTERRUPT();

#if defined USE_ICALL || defined OSAL_PORT2TIRTOS
  /* backoffTimerTrigger must be set to a value
   * to properly use rollover value for the next wakeup time.
   */
  {
    halIntState_t intState;
    HAL_ENTER_CRITICAL_SECTION(intState);
    /* This code assumes that backoff timer callback does not cause
     * a problem when the callback is made at the rollover even if
     * no timer is associated with it. At the time the following
     * code is written, mac_timer.c can live with such a callback.
     * Setting backoff comparator value one greater than rollver value
     * might be conceived here to lift the above constraint,
     * but it would have to ensure that rollover value is never
     * the highest counter value, which is a more dangerous assumption.
     */
    backoffTimerTrigger = macBackoffTimerRollover;

    /* Note that MAC_RADIO_BACKOFF_COMPARE_CLEAR_INTERRUPT() is not implemented
     * correctly and hence backoff timer trigger interrupt can still occur.
     * Instead of fixing MAC_RADIO_BACKOFF_COMPARE_CLEAR_INTERRUPT() macro,
     * comparator is set again, to simplify interrupt handling.
     */
    MAC_RADIO_BACKOFF_SET_COMPARE(backoffTimerTrigger);

    MAC_BACKOFF_TIMER_UPDATE_WAKEUP();
    HAL_EXIT_CRITICAL_SECTION(intState);
  }
#endif /* defined USE_ICALL || defined OSAL_PORT2TIRTOS */

}


/**************************************************************************************************
 * @fn          macBackoffTimerRealign
 *
 * @brief       
 *
 *  Realignment is accomplished by adjusting the internal time base to align with the expected
 *  reception time of an incoming frame.  The difference between the expected reception time and
 *  the actual reception time is computed and this difference is used to adjust the hardware
 *  timer count and backoff count.
 *
 *  The realignment is based on the SFD signal for the incoming frame.  The timer is aligned
 *  by adjusting it with the difference between the expected SFD time and the actual SFD time.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API int32 macBackoffTimerRealign(macRx_t *pMsg)
{
#if defined( FEATURE_BEACON_MODE )
  uint32 macRatCount = MAC_RAT_COUNT;
  int32  ratDelta;

  /* Find the RAT delta. This delta is the time difference between the beacon timestamp
   * and local time base.
   */
  ratDelta = macRxOutput.beaconTimeStamp - macPrevPeriodRatCount;
  
  /* Convert to number of backoffs into the current superframe.
   * Note that this is not ideal but the macBackoffTimerRollover has not been configured at this point.
   */
  ratDelta = (ratDelta / MAC_BACKOFF_TO_RAT_RATIO) % macTimerRolloverValue[pMacPib->beaconOrder];

  DBG_PRINTL1(DBGSYS, "!!! macBackoffTimerRealign(%li)", ratDelta);
  DBG_PRINTL2(DBGSYS, "Beacon Timestamp = 0x%X, Previous Period RAT Count = 0x%X", macRxOutput.beaconTimeStamp, macPrevPeriodRatCount);
  
  /* The beacon frame is sent out when the coordinator backoff timer has just rolled over. 
   * Setting the previous period RAT count will synchroze both time bases.
   */
  macPrevPeriodRatCount = macRxOutput.beaconTimeStamp;
  return( ratDelta );
#else
  return( 0 );
#endif /* FEATURE_BEACON_MODE */
}


/**************************************************************************************************
 * @fn          macBackoffTimerCompareIsr
 *
 * @brief       Interrupt service routine that fires when the backoff count is equal
 *              to the trigger count.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macBackoffTimerCompareIsr(void)
{
  halIntState_t s;

  DBG_PRINT1(DBGSYS, "macRatChanB Compare ISR, Backoff RAT count = %u", MAC_RADIO_BACKOFF_COUNT());

  macBackoffTimerTriggerCallback();

  HAL_ENTER_CRITICAL_SECTION(s);
  MAC_BACKOFF_TIMER_UPDATE_WAKEUP();
  HAL_EXIT_CRITICAL_SECTION(s);
}

/**************************************************************************************************
 * @fn          macBackoffTimerPeriodIsr
 *
 * @brief       Interrupt service routine that fires when the backoff count rolls over on
 *              overflow period.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macBackoffTimerPeriodIsr(void)
{
  halIntState_t s;
  uint32 macRatCount           =  MAC_RAT_COUNT;
  uint32 backoffRolloverRat    =  macBackoffTimerRollover * MAC_BACKOFF_TO_RAT_RATIO;
  uint32 ratCompensation       = (macRatCount - macPrevPeriodRatCount) % MAC_BACKOFF_TO_RAT_RATIO;
      
  MAC_ASSERT( macBackoffTimerRollover <= MAC_BACKOFF_MAXIMUM_ROLLOVER );

  if (macRatCount < backoffRolloverRat)
  {
    /* RAT wraparound has occurred. This would occur once in a blue moon (1073.74 seconds).
     */
    DBG_PRINTL1(DBGSYS, "!!! RAT wraparound !!! RAT = %u", macRatCount);
  }

  DBG_PRINTL2(DBGSYS, "macRatChanA Period ISR, Rollover Period = %u, RAT Compensation = %u", macBackoffTimerRollover, ratCompensation);
  
  /* Convert count to RAT count and set MAC Channel A Compare. The modulus calculation will 
   * compensate the math or interrupt latency error and prevent it from being accumulated.
   * Note that MAC_BACKOFF_TO_RAT_RATIO is used as part of compensation. This means the 
   * maximum error that can be compensated is 320us. If the interrupt latency is greater 
   * than 320us, more elaborated compensation scheme must be used for Beacon mode. 
   * Non-beacon mode does not require absolute timing. Longer interrupt latency can be 
   * tolerated.
   */
  macPrevPeriodRatCount = macRatCount - ratCompensation;
  macSetupRATChanCompare( macRatChanA, backoffRolloverRat + macPrevPeriodRatCount );
  macBackoffTimerRolloverCallback();
  HAL_ENTER_CRITICAL_SECTION(s);
  MAC_BACKOFF_TIMER_UPDATE_WAKEUP();
  HAL_EXIT_CRITICAL_SECTION(s);
}


/**************************************************************************************************
*/
