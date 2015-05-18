/*********************************************************************
 Filename:       osaltasks.c
 Revised:        $Date: 2014-03-25 11:23:30 -0700 (Tue, 25 Mar 2014) $
 Revision:       $Revision: 37868 $

 Description:    This file contains all the settings and other functions
 that the user should set and change.


 Copyright 2014 - 2015 Texas Instruments Incorporated. All rights reserved.

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
 PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
 */

/*********************************************************************
 * INCLUDES
 */
#include <ICall.h>
#include <ICallCC26xxDefs.h>
#include "ZComDef.h"

#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_Nv.h"

#include "nwk.h"
#include "APS.h"
#include "aps_frag.h"
#include "ZDApp.h"
#include "ZDNwkMgr.h"
#include "stub_aps.h"

#include "hal_mcu.h"
#include "hw_ints.h"
#include "hal_trng_wrapper.h"

#include "mac_low_level.h"
#ifdef FEATURE_MAC_SECURITY
#include "hal_aes.h"
#endif

#if defined( CC26XX )
#include "R2R_FlashJT.h"
#include "R2F_FlashJT.h"
#endif // CC26XX

#if defined DEBUG_SW_TRACE || defined DBG_ENABLE
/* Header files required for tracer enabling */
#include <ioc.h>
#include <hw_ioc.h>
#include <hw_memmap.h>
#endif // defined DEBUG_SW_TRACE || defined DBG_ENABLE

#include "OnBoard.h"
#include "zstacktask.h"
#include "zstackconfig.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*
 * The order in this table must be identical to the task initialization calls
 * below in osalInitTask.
 */
const pTaskEventHandlerFn tasksArr[] =
{
  macEventLoop,
  nwk_event_loop,
  APS_event_loop,
#if defined ( ZIGBEE_FRAGMENTATION )
  APSF_ProcessEvent,
#endif
  ZDApp_event_loop,
#if defined ( ZIGBEE_FREQ_AGILITY ) || defined ( ZIGBEE_PANID_CONFLICT )
  ZDNwkMgr_event_loop,
#endif
#if defined( INTER_PAN )
  StubAPS_ProcessEvent,
#endif
  ZStackTaskProcessEvent
};

/* Number of tasks in the OSAL system */
const uint8 tasksCnt = sizeof(tasksArr) / sizeof(tasksArr[0]);

/* Pointer to the task events, one for each OSAL task */
uint16 *tasksEvents;

/* Pointer to the passed in configuration structure */
zstack_Config_t *pZStackCfg = NULL;

/*********************************************************************
 * FUNCTIONS
 */

static void zmain_ext_addr( uint8 *pExtAddr );

/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   none
 *
 * @return  none
 */
void osalInitTasks( void )
{
  uint8 taskID = 0;

  tasksEvents = (uint16 *) osal_mem_alloc( sizeof(uint16) * tasksCnt );
  osal_memset( tasksEvents, 0, (sizeof(uint16) * tasksCnt) );

  /* Initialize OSAL tasks */
  macTaskInit( taskID++ );
  nwk_init( taskID++ );
  APS_Init( taskID++ );
#if defined ( ZIGBEE_FRAGMENTATION )
  APSF_Init( taskID++ );
#endif
  ZDApp_Init( taskID++ );
#if defined ( ZIGBEE_FREQ_AGILITY ) || defined ( ZIGBEE_PANID_CONFLICT )
  ZDNwkMgr_Init( taskID++ );
#endif
#if defined( INTER_PAN )
  StubAPS_Init( taskID++ );
#endif

  ZStackTaskInit( taskID );
}

/*********************************************************************
 * @fn      stack_main
 *
 * @brief   Main entry function for the stack image
 *
 * @param   pInputParams - pointer to the passed in configuration
 *                         parameters
 *
 * @return  0, but should never return
 */
int stack_main( void *pInputParams )
{
  halIntState_t state;
  ICall_Errno errorno;
  uint8 *pExtAddr = NULL;

  /* Save the configurations parameters passed in from the app image */
  pZStackCfg = (zstack_Config_t *) pInputParams;
  if ( pZStackCfg )
  {
    pZStackCfg->initFailStatus = ZSTACKCONFIG_INIT_SUCCESS;
  }

  /* User App Reconfiguration of TIMAC */
  macSetUserConfig( &(pZStackCfg->macConfig) );

#if defined( DEBUG_SW_TRACE )
#if defined(HAL_UART) && (HAL_UART==TRUE)
  // Enable tracer output on DIO24/ATEST1.
  // Set max drive strength and reduced slew rate.
  // PortID = 46 = RFcore tracer.
  HWREG(IOC_BASE + IOC_O_IOCFG24) = (3 << IOC_IOCFG24_IOSTR_S) | (1 << IOC_IOCFG24_SLEW_RED_S) | 46;

#else // no UART so allow trace on Tx pin - Debug Package only
  // Enable tracer output on DIO23/ATEST0.
  // Set max drive strength and reduced slew rate.
  // PortID = 46 = RFcore tracer.
  HWREG(IOC_BASE + IOC_O_IOCFG23) = (3 << IOC_IOCFG23_IOSTR_S) | (1 << IOC_IOCFG24_SLEW_RED_S) | 46;
#endif // HAL_UART
#endif // DEBUG_SW_TRACE

  /* Register an entity to use timer service to implement OSAL timer */
  errorno = ICall_registerApp( &osal_entity, &osal_semaphore );
  if ( errorno != ICALL_ERRNO_SUCCESS )
  {
    /* abort */
    ICall_abort();
  }

  HAL_ENTER_CRITICAL_SECTION( state );

  // Have to turn on the TRNG power before HalTRNG_InitTRNG
  // but must not repeat it every time the device wakes up
  // hence the call cannot be added to HalTRNG_InitTRNG();
  ICall_pwrRequire (ICALL_PWR_D_PERIPH_TRNG);

  // Initialize the true random number generator
  HalTRNG_InitTRNG();

  HalAesInit();

  {
    /* Work around for MAC ROM jump table problem, the MAC references
     * taskEvent (extern to ROM code) incorrectly through the jump table.
     * This causes a problem because the taskEvent list isn't allocated
     * and taskCnt initialized to more than 0.
     */
    uint16 temp[sizeof(tasksArr) / sizeof(uint16)];
    osal_memset( temp, 0, (sizeof(uint16) * tasksCnt) );
    tasksEvents = temp;

    /* Initialize MAC */
    ZMacInit();
  }

  if ( pZStackCfg )
  {
    uint8 dummyAddr[8];
    osal_memset( dummyAddr, 0xFF, Z_EXTADDR_LEN );
    if ( osal_memcmp( pZStackCfg->extendedAddress, dummyAddr, 8 ) != TRUE )
    {
      pExtAddr = pZStackCfg->extendedAddress;
    }
  }

  // Determine the extended address
  zmain_ext_addr( pExtAddr );

  // Initialize basic NV items
  zgInit();

  // Since the AF isn't a task, call it's initialization routine
  afInit();

  /* Initialize MAC buffer */
  macLowLevelBufferInit();

  // Initialize the operating system
  osal_init_system();

  /* Enable interrupts */
  HAL_EXIT_CRITICAL_SECTION( state );

  osal_start_system(); // No Return from here

  return 0;  // Shouldn't get here.
}

/*********************************************************************
 * @fn          zmain_ext_addr
 *
 * @brief       Execute a prioritized search for a valid extended address and
 *              write the results into the OSAL NV system for use by the system.
 *              Temporary address not saved to NV.
 *
 * @param       pExtAddr - Pointer to passed in extended address.
 *
 * @return      None.
 */
static void zmain_ext_addr( uint8 *pExtAddr )
{
  uint8 nullAddr[Z_EXTADDR_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  if ( (SUCCESS != osal_nv_item_init( ZCD_NV_EXTADDR, Z_EXTADDR_LEN, NULL ))
      || (SUCCESS
          != osal_nv_read( ZCD_NV_EXTADDR, 0, Z_EXTADDR_LEN, aExtendedAddress )) )
  {
    if ( pExtAddr )
    {
        if (!osal_memcmp(aExtendedAddress, nullAddr, Z_EXTADDR_LEN)) 
        {
            osal_memcpy( aExtendedAddress, pExtAddr, 8 );
            (void)osal_nv_write(ZCD_NV_EXTADDR, 0, Z_EXTADDR_LEN, aExtendedAddress);
        }
    }
    else
    {
      uint8 idx;

      /* Attempt to create a sufficiently random extended address for expediency.
       * Note: this is only valid/legal in a test environment and
       *       must never be used for a commercial product.
       */
      for ( idx = 0; idx < (Z_EXTADDR_LEN - 2); )
      {
        uint16 randy = osal_rand();
        aExtendedAddress[idx++] = LO_UINT16( randy );
        aExtendedAddress[idx++] = HI_UINT16( randy );
      }

      // Next-to-MSB identifies ZigBee devicetype.
#if ZG_BUILD_COORDINATOR_TYPE && !ZG_BUILD_JOINING_TYPE
      aExtendedAddress[idx++] = 0x10;
#elif ZG_BUILD_RTRONLY_TYPE
      aExtendedAddress[idx++] = 0x20;
#else
      aExtendedAddress[idx++] = 0x30;
#endif
      // MSB has historical significance.
      aExtendedAddress[idx] = 0xF8;
      
#if defined ( NV_RESTORE )
      (void)osal_nv_write(ZCD_NV_EXTADDR, 0, Z_EXTADDR_LEN, aExtendedAddress);
#endif
    }
  }
  // Set the MAC PIB extended address according to results from above.
  (void) ZMacSetReq( MAC_EXTENDED_ADDRESS, aExtendedAddress );
}

/*********************************************************************
 *********************************************************************/
