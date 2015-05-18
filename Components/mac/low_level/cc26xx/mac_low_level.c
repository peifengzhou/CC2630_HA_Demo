/**************************************************************************************************
  Filename:       mac_low_level.c
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

/* PRCM */
#include "hw_prcm.h"

/* driverlib */
#include "hw_types.h"
#include "interrupt.h"

/* hal */
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_mcu.h"

#include "saddr.h"

/* mailbox */
#include "mb.h"
#include "rfHal.h"
#include "mac.h"

/* exported low-level */
#include "mac_low_level.h"

/* low-level specific */
#include "mac_isr.h"

#include "mac_radio.h"
#include "mac_rx.h"
#include "mac_tx.h"
#include "mac_rx_onoff.h"
#include "mac_backoff_timer.h"

/* high-level specific */
#include "mac_main.h"

/* target specific */
#include "mac_radio_defs.h"

#ifdef USE_ICALL
#include <ICall.h>
#include <ICallCC26xxDefs.h>
#include <hw_rfc_pwr.h>
#endif /* USE_ICALL */

#ifdef OSAL_PORT2TIRTOS
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <hw_rfc_pwr.h>
#endif /* OSAL_PORT2TIRTOS */

/* debug */
#include "mac_assert.h"
 
/* Common ROM jump tables */
#include "R2F_CommonFlashJT.h"

#ifdef DEBUG_SW_TRACE
#define DBG_ENABLE
#include "dbgid_sys_mst.h"
#endif /* DEBUG_SW_TRACE */
   
/**************************************************************************************************
 * @fn          macLowLevelInit
 *
 * @brief       Initialize low-level MAC.  Called only once on system power-up.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macLowLevelInit(void)
{
#ifdef USE_ICALL
#if !defined( USE_FPGA )
  /* set the dependencies */
  ICall_pwrRequire(ICALL_PWR_D_PERIPH_RFCORE);
  ICall_pwrRequire(ICALL_PWR_D_XOSC_HF);
  ICall_pwrRequire(ICALL_PWR_C_NEED_FLASH_IN_IDLE);
#endif /* !USE_FPGA */
  // Enable clocks for all radio internal modules.
  // Use Non-Buff access for safety and check for sanity
  HWREG(RFC_PWR_NONBUF_BASE + RFC_PWR_O_PWMCLKEN) = 0x7FF;
#endif /* USE_ICALL */

#ifdef OSAL_PORT2TIRTOS
#if !defined( USE_FPGA )
  /* set the dependencies */
  Power_setDependency(PERIPH_RFCORE);
  Power_setDependency(XOSC_HF);
  Power_setConstraint(Power_NEED_FLASH_IN_IDLE);
#endif /* !USE_FPGA */
  /* Enable clocks for all radio internal modules.
   * Use Non-Buff access for safety and check for sanity
   */
  HWREG(RFC_PWR_NONBUF_BASE + RFC_PWR_O_PWMCLKEN) = 0x7FF;
#endif /* OSAL_PORT2TIRTOS */

  /* disable CPE0 and CPE1 interrupts */
  MB_Init();
  
  /* setup the Mailbox */
  macSetupMailbox();
  
#ifdef DEBUG_SW_TRACE 
  /* enable RF trace output for FPGA: 
   * 0x1D40 for more radio trace or 
   * 0x1940 for less radio trace. 
   */
  MB_SendCommand( BUILD_DIRECT_PARAM_EXT_CMD( CMD_ENABLE_DEBUG, 0x1940 ) );
  DBG_PRINT0(DBGSYS, "RF Trace Start...");
#endif /* DEBUG_SW_TRACE */

  /* setup RF HAL */
  macSetupRfHal();
  
  /* initialize mcu before anything else */
  MAC_RADIO_MCU_INIT();
  
  /* software initialziation */
  macRadioInit();
  
  macRxOnOffInit();

  /* This delay is under investigation but it appears to be necessary for 6Mesh. 
   * Without the delay, 1 out of 20 resets may hang.
   */
  { int i; for (i = 0; i < 0x1000; i++) asm("NOP"); }
  
  /* MAC radio setup */
  macSetupRadio();

  /* Initialize backoff timers */
  macBackoffTimerInit();

#if defined USE_ICALL || defined OSAL_PORT2TIRTOS
#if !defined( USE_FPGA )
  /* Initialize backoff timer power management */
  macBackoffSetupPwrMgmt();
#endif /* !USE_FPGA */
#endif /* defined USE_ICALL || defined OSAL_PORT2TIRTOS */
}


/**************************************************************************************************
 * @fn          macLowLevelBufferInit
 *
 * @brief       Initialize low-level MAC.  Called only once on system power-up. OSAL must be 
 *              initialized before this function is called.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macLowLevelBufferInit(void)
{
  macRxInit();
  macTxInit();
}


/**************************************************************************************************
 * @fn          macLowLevelReset
 *
 * @brief       Reset low-level MAC.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macLowLevelReset(void)
{
  MB_SendCommand( BUILD_DIRECT_CMD(CMD_ABORT) );
  
  /* reset timer */
  macBackoffTimerReset();
}


/**************************************************************************************************
 * @fn          macLowLevelResume
 *
 * @brief       Resume the low-level MAC after a successful return from macLowLevelYield.
 *              Note: assuming interrupts are disabled.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
MAC_INTERNAL_API void macLowLevelResume(void)
{
  macRadioInit();
}


/**************************************************************************************************
 * @fn          macLowLevelYield
 *
 * @brief       Check whether or not the low-level MAC is ready to yield.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE or FALSE whether the low-level MAC is ready to yield.
 **************************************************************************************************
 */
MAC_INTERNAL_API bool macLowLevelYield(void)
{
  bool rtrn = TRUE;
  halIntState_t  s;
  HAL_ENTER_CRITICAL_SECTION(s);

  // If RX or TX is active or any RX enable flags are set, it's not OK to yield.
  if (macRxActive || macRxOutgoingAckFlag || macTxActive || (macRxEnableFlags & ~MAC_RX_WHEN_IDLE))
  {
    rtrn = FALSE;
  }

  HAL_EXIT_CRITICAL_SECTION(s);
  return rtrn;
}


/**************************************************************************************************
 * @fn          macLowLevelDiags
 *
 * @brief       Increments a specified diagnostic counter (stored in the PIB).
 *
 * @param       pibAttribute - PIB attribute to be incremented.
 *
 * @return      none
 **************************************************************************************************
 */
MAC_INTERNAL_API void macLowLevelDiags( uint8 pibAttribute )
{
#if defined ( FEATURE_SYSTEM_STATS )
  if ( ( pibAttribute >= MAC_DIAGS_RX_CRC_PASS ) &&
       ( pibAttribute <= MAC_DIAGS_TX_UCAST_FAIL ) )
  {
    uint32 value;

    /* Update Diagnostics counter */
    MAC_MlmeGetReq( pibAttribute, &value );
    value++;
    MAC_MlmeSetReq( pibAttribute, &value );
  }
#endif
}


/**************************************************************************************************
*/
