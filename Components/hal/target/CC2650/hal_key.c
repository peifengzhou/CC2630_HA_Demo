/**************************************************************************************************
  Filename:       hal_key.c
  Revised:        $Date: 2013-01-14 10:39:06 -0800 (Mon, 14 Jan 2013) $
  Revision:       $Revision: 32772 $

  Description:    This file contains the interface to the HAL KEY Service.


  Copyright 2007-2012 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_types.h"
#include "hal_key.h"
#include "hal_sleep.h"
#include "osal.h"
#include "OnBoard.h"
#include "hal_drivers.h"
#include "hal_mcu.h"

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/
#define HAL_KEY_WAKE_INIT()

/* Rising edge trigger for key */
#define HAL_KEY_INT_INIT()                                   \
{                                                            \
  /* Set the key ports as input */                           \
  GPIOPinTypeGPIOInput(BTN_PORT_R, HAL_BTN_R_L_U_D);         \
  GPIOPinTypeGPIOInput(BTN_PORT_S, HAL_BTN_S);               \
                                                             \
  /* Set the PAD Config Override Output Enable */            \
  IOCPadConfigSet(BTN_PORT_R, HAL_BTN_R_L_U_D,               \
                  IOC_OVERRIDE_PUE);                         \
  IOCPadConfigSet(BTN_PORT_S, HAL_BTN_S,                     \
                  IOC_OVERRIDE_PUE);                         \
                                                             \
  /* Set the interrupt Override Output Enable */             \
  IntPrioritySet(INT_GPIOC, HAL_INT_PRIOR_KEY);              \
  IntPrioritySet(INT_GPIOA, HAL_INT_PRIOR_KEY);              \
                                                             \
  /* Enable the key Interrupts */                            \
  IntEnable(INT_GPIOC);                                      \
  IntEnable(INT_GPIOA);                                      \
                                                             \
  /* Set Key Interrupt as falling edge */                    \
  GPIOIntTypeSet(BTN_PORT_R, HAL_BTN_R_L_U_D,                \
                 GPIO_FALLING_EDGE);                         \
  GPIOIntTypeSet(BTN_PORT_S, HAL_BTN_S,                      \
                 GPIO_FALLING_EDGE);                         \
}                 

/* Key interrupt configuration */
#define HAL_ENABLE_KEY_INT()                                 \
{                                                            \
  GPIOPinIntEnable(BTN_PORT_R, HAL_BTN_R_L_U_D);             \
  GPIOPinIntEnable(BTN_PORT_S, HAL_BTN_S);                   \
}

/* Disable Key in interrupt configuration */
#define HAL_DISABLE_KEY_INT()                                \
{                                                            \
  GPIOPinIntDisable(BTN_PORT_R, HAL_BTN_R_L_U_D);            \
  GPIOPinIntDisable(BTN_PORT_S, HAL_BTN_S);                  \
}

/* Clear interrupt in Key*/
#define HAL_CLEAR_KEY_INT()                                  \
{                                                            \
  GPIOPinIntClear(BTN_PORT_R, HAL_BTN_R_L_U_D);              \
  GPIOPinIntClear(BTN_PORT_S, HAL_BTN_S);                    \
}

/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
#if (HAL_KEY == TRUE)
static uint8 halSavedKeys;
static uint8 halIntKeys;
static halKeyCBack_t pHal_KeyProcessFunction;
bool Hal_KeyIntEnable;
#endif /* HAL_KEY */

/**************************************************************************************************
 *                                        EXTERNAL VARIABLES
 **************************************************************************************************/

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/
void interrupt_keybd(void);

/**************************************************************************************************
 * @fn      HalKeyInit
 *
 * @brief   Initilize Key Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalKeyInit( void )
{
#if (HAL_KEY == TRUE)
  /* Initialize previous key to 0 */
  halSavedKeys = 0;

  /* Initialize callback function */
  pHal_KeyProcessFunction  = NULL;
#endif /* HAL_KEY */

}

/* Interrupt key read for Key Select */
uint8 hal_key_int_key_S(void)                                      
{ 
  uint8 x;
  x  = (FALSE /* TODO: (uint8)(GPIOPinIntStatus(BTN_PORT_S, 0) & HAL_BTN_S)*/);  
  return x;
}

/* Interrupt key read for left right up and down */
uint8 hal_key_int_key_R_L_U_D(void)                                 
{                                                                  
  uint8 x = 0;
  x  = (FALSE /* TODO: (uint8)(GPIOPinIntStatus(BTN_PORT_R, 0) & HAL_BTN_R_L_U_D)*/);
  return x;
}

/**************************************************************************************************
 * @fn      hal_key_keys()
 *
 * @brief   Determine if key was pressed and which key was pressed
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
uint8 hal_key_keys(void)                                           
{                                                                 
  uint8 x = 0;

#if 0 // TODO:  
  /* Determine if Key right was pressed */
  if(!(GPIOPinRead(BTN_PORT_R, HAL_BTN_R) & HAL_BTN_R))
  {
    x  |= HAL_BTN_R;
  } 
  
  /* Determine if Key light was pressed */
  if(!(GPIOPinRead(BTN_PORT_R, HAL_BTN_L) & HAL_BTN_L))
  {
    x  |= HAL_BTN_L;
  }
  
  /* Determine if Key up was pressed */
  if(!(GPIOPinRead(BTN_PORT_R, HAL_BTN_U) & HAL_BTN_U))
  {
    x  |= HAL_BTN_U;
  } 
  
  /* Determine if Key down was pressed */
  if(!(GPIOPinRead(BTN_PORT_R, HAL_BTN_D) & HAL_BTN_D))
  {
    x  |= HAL_BTN_D;
  } 
  
  /* Determine if Key select was pressed */
  if(!(GPIOPinRead(BTN_PORT_S, HAL_BTN_S) & HAL_BTN_S))
  {
    x  |= HAL_BTN_S;
  } 
#endif
  return x;
}

/**************************************************************************************************
 * @fn      hal_key_keys()
 *
 * @brief   Determine if key was and which key was pressed during interrupt
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
uint8 hal_key_int_keys(void)                                       
{                                                                 
  uint8 x = 0;

#if 0 // TODO  
  /* Determine if Key Select was pressed */
  x |= ((uint8)(GPIOPinIntStatus(BTN_PORT_S, 0) & HAL_BTN_S));   
  
  /* Determine if Key Right, Up, Left, Down was pressed */
  x |= ((uint8)(GPIOPinIntStatus(BTN_PORT_R, 0) & HAL_BTN_R_L_U_D));
#endif  
  return x;
}

/**************************************************************************************************
 * @fn      HalKeyConfig
 *
 * @brief   Configure the Key serivce
 *
 * @param   interruptEnable - TRUE/FALSE, enable/disable interrupt
 *          cback - pointer to the CallBack function
 *
 * @return  None
 **************************************************************************************************/
void HalKeyConfig( bool interruptEnable, halKeyCBack_t cback)
{
#if (HAL_KEY == TRUE)
  /* Enable/Disable Interrupt or */
  Hal_KeyIntEnable = interruptEnable;

  /* Register the callback fucntion */
  pHal_KeyProcessFunction = cback;

  /* Initialize GPIO */
  HAL_KEY_INT_INIT();

  /* Determine if interrupt is enable or not */
  if ( Hal_KeyIntEnable )
  {
    /* Clear interrupt flags */
    HAL_CLEAR_KEY_INT();

    /* Enable interrupts */
    HAL_ENABLE_KEY_INT();

    /* Cancel polling if there is one active */
    osal_stop_timerEx(Hal_TaskID, HAL_KEY_EVENT);
  }
  else
  {
    /* Disable interrupts */
    HAL_DISABLE_KEY_INT();

    if( cback != NULL)
    {
      /* Start polling if callback function is setup*/
      osal_set_event(Hal_TaskID, HAL_KEY_EVENT);
    }
  }
#endif /* HAL_KEY */
}

/**************************************************************************************************
 * @fn      HalKeyRead
 *
 * @brief   Read the current value of a key
 *
 * @param   None
 *
 * @return  keys - current keys status
 **************************************************************************************************/
uint8 HalKeyRead( void )
{
  uint8 keys = 0;

#if (HAL_KEY == TRUE)
  if (!Hal_KeyIntEnable)
  {
    keys = hal_key_keys();
  }
  else
  {
    keys = hal_key_int_keys();
  }
#endif /* HAL_KEY */

  return keys;
}

/**************************************************************************************************
 * @fn      HalKeyPoll
 *
 * @brief   Send call back if key(s) is pressed
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalKeyPoll( void )
{
#if (HAL_KEY == TRUE)
  uint8 keys = 0;
  
  /* if polling is using */
  if (!Hal_KeyIntEnable)
  {
    /* Get keys */
    keys = hal_key_keys ();

    /* If same as before, no keys */
    if ( keys == halSavedKeys )
    {
      return;
    }

    /* Store the current keys for comparation next time */
    halSavedKeys = keys;

  }
  else
  {
    /* halIntKeys is filled in ISR.
     * Compare halIntKeys with present keys.
     * If both are not the same then the interrupt is a false interrupt. 
     */
    keys = hal_key_keys(); 
    if(keys != halIntKeys)
    {
      return;
    }   
  }

  /* Callback */
 if (keys && (pHal_KeyProcessFunction))
  {
    (pHal_KeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
  }

#endif /* HAL_KEY */
}

#ifdef POWER_SAVING
/**************************************************************************************************
 * @fn      HalKeyEnterSleep
 *
 * @brief  - Get called to enter sleep mode
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void HalKeyEnterSleep ( void )
{
  /* nothing to do */
}

/**************************************************************************************************
 * @fn      HalKeyExitSleep
 *
 * @brief   - Get called when sleep is over
 *
 * @param
 *
 * @return  - keys
 **************************************************************************************************/
uint8 HalKeyExitSleep ( void )
{
  uint8 keys = 0;

  /* Get keys */
  if (!Hal_KeyIntEnable)
  {
    keys = hal_key_keys();
  }
  else
  {
    keys = hal_key_int_keys();
  }
  return ( keys );
}
#endif /* POWER_SAVING */

/**************************************************************************************************
 * @fn      INTERRUPT_KEYBD
 *
 * @brief   Interrupt Service Routine for keyboard
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void interrupt_keybd(void)
{
#if (HAL_KEY == TRUE)

#ifdef POWER_SAVING
  /* Must allow key interrupt to cancel sleep */
  //halSleepExit();
#endif

  /* Read the key before it gone */
  halIntKeys = hal_key_int_keys();

  /* Clear depending interrupt */
  HAL_CLEAR_KEY_INT();

  /* A key is pressed, let HalKeyPoll routing handle the keys at a later time */
  osal_start_timerEx( Hal_TaskID, HAL_KEY_EVENT, 25 );
#endif /* HAL_KEY */
  
  CLEAR_SLEEP_MODE();
}

/**************************************************************************************************
**************************************************************************************************/
