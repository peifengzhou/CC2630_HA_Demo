/*******************************************************************************
  Filename:       hal_key_wrapper.c
  Revised:        $Date: 2013-09-06 12:23:39 -0700 (Fri, 06 Sep 2013) $
  Revision:       $Revision: 35245 $

  Description:    This file contains the implementations to the HAL KEY Service.
                  This is a HAL wrapper file for the BSP key driver.

  Copyright 2009-2013 Texas Instruments Incorporated. All rights reserved.

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

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_key.h"
#include "osal.h"
#include "bsp.h"
#include "bsp_key.h"

#if (defined HAL_KEY) && (HAL_KEY == TRUE)

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

#define HAL_KEY_DEBOUNCE_VALUE  25

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

static uint8         halKeySavedKeys;         /* used to store previous key state in polling mode */
static halKeyCBack_t pHalKeyProcessFunction;
bool                 Hal_KeyIntEnable;        /* interrupt enable/disable flag */

/*******************************************************************************
 * Prototypes
 */

void HalKey_ISR(void);

/*******************************************************************************
 * API
 */

/*******************************************************************************
 * @fn          HalKeyInit API
 *
 * @brief       Initilize the key service.
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
void HalKeyInit( void )
{
  // Initialize bsp_key.
  bspKeyInit( HAL_KEY_MODE );

  // Initialize callback function.
  pHalKeyProcessFunction = NULL;

  // Register Hal Key ISR.
  bspKeyIntRegister( BSP_KEY_ALL, &HalKey_ISR );

  // Enable interrupts on all keys
  bspKeyIntEnable( BSP_KEY_ALL );

  return;
}


/*******************************************************************************
 * @fn          HalKeyConfig API
 *
 * @brief       Configure the Key service. Used to configure and reconfigure
 *              state machine based on the current value of the interruptEnable.
 *
 * input parameters
 *
 * @param       interruptEnable - TRUE/FALSE, enable/disable interrupt.
 * @param       cback           - Pointer to the CallBack function
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void HalKeyConfig( bool interruptEnable, halKeyCBack_t cback )
{
  // Enable/Disable Interrupt or
  Hal_KeyIntEnable = interruptEnable;

  // Register the callback fucntion
  pHalKeyProcessFunction = cback;

  if ( !Hal_KeyIntEnable ) // Interrupts disabled
  {
    // Start polling.
    osal_set_event( Hal_TaskID, HAL_KEY_EVENT );
  }

  return;
}


/*******************************************************************************
 * @fn          HalKeyPoll API
 *
 * @brief       Called by hal_driver to poll the keys.
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
void HalKeyPoll( void )
{
  uint8 keys   = 0;
  uint8 notify = 0;

  // Read keys.
  keys = HalKeyRead();

  /* If interrupts are not enabled, previous key status and current key status
   * are compared to find out if a key has changed status.
   */
  if ( !Hal_KeyIntEnable)
  {
    if ( keys == halKeySavedKeys )
    {
      /* Exit - since no keys have changed */
      notify = 0;
      return;
    }
    else
    {
      notify = 1;
    }
  }
  else
  {
    // key interrupt handled here
    notify = (keys)?1:0;
  }

  /* Store the current keys for comparation next time */
  halKeySavedKeys = keys;

  /* Invoke Callback if new keys were depressed */
  if ( notify && (pHalKeyProcessFunction) )
  {
    (pHalKeyProcessFunction)(keys, HAL_KEY_STATE_NORMAL);

  }

  return;
}


/*******************************************************************************
 * @fn          HalKeyRead API
 *
 * @brief       Read the current value of the keys.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Current keys status.
 */
uint8 HalKeyRead( void )
{
  // Returns a bitmask of all keys that were pushed.
  uint8 keyBitMask = 0;
  uint32 keysPushed = bspKeyPushed( BSP_KEY_ALL );

  // Convert bitmask into a bitmask understood by application.
  // Select
  if ( keysPushed & BSP_KEY_SELECT )
  {
    keyBitMask |= HAL_KEY_SW_5;
  }
  // Left
  if ( keysPushed & BSP_KEY_LEFT )
  {
    keyBitMask |= HAL_KEY_SW_4;
  }
  // Right
  if ( keysPushed & BSP_KEY_RIGHT )
  {
    keyBitMask |= HAL_KEY_SW_2;
  }
  // Up
  if ( keysPushed & BSP_KEY_UP )
  {
    keyBitMask |= HAL_KEY_SW_1;
  }
  // Down
  if ( keysPushed & BSP_KEY_DOWN )
  {
    keyBitMask |= HAL_KEY_SW_3;
  }

  return( keyBitMask );
}


#ifdef POWER_SAVING
/*******************************************************************************
 * @fn          HalKeyEnterSleep API
 *
 * @brief       Called before entering sleep.
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
void HalKeyEnterSleep( void )
{
  /* nothing to do */
  return;
}


/*******************************************************************************
 * @fn          HalKeyExitSleep API
 *
 * @brief       Called after exiting sleep.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Value of keys.
 */
uint8 HalKeyExitSleep( void )
{
  return ( HalKeyRead() );
}
#endif /* POWER_SAVING */


/*******************************************************************************
 * @fn          HalKey_ISR
 *
 * @brief       Interrupt Service Routine for HAL Keys.
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
void HalKey_ISR( void )
{
  CLEAR_SLEEP_MODE();

  // To account for debounce, wait before reading.
  osal_start_timerEx( Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE );

  return;
}

#endif // (defined HAL_KEY) && (HAL_KEY == TRUE)

/*******************************************************************************
 */
