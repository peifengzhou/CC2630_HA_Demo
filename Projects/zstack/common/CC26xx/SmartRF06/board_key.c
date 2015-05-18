/*****************************************************************************
 Filename:       board_key.c
 Revised:        $Date: 2015-01-28 10:11:21 -0800 (Wed, 28 Jan 2015) $
 Revision:       $Revision: 42083 $

 Description:    TThis file contains the interface to the SRF06EB Key Service.

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
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <stdbool.h>
#include <ti/sysbios/knl/Clock.h>

#include "util.h"
#include "board_key.h"

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void board_key_changeHandler(UArg a0);
static void board_key_keyFxn(PIN_Handle keyPinHandle, PIN_Id keyPinId);

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Value of keys Pressed
static uint8_t keysPressed;

// Key debounce clock
static Clock_Struct keyChangeClock;

// Pointer to application callback
Board_Key_keysPressedCB_t appKeyChangeHandler = NULL;

/* Create the MSA KEY pin table. This will override the key attributes in
 * BoardGpioInitTable[].
 */
static PIN_Config keyPinTable[] =
{
    Board_KEY_SELECT | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE, /* KEY
                                                                      SELECT
                                                                          */
    Board_KEY_UP | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,     /* KEY UP

                                                                             */
    Board_KEY_DOWN | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,   /* KEY DOWN
                                                                            */
#if defined (MODULE_CC26XX_7X7)
    Board_KEY_LEFT | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,   /* KEY LEFT
                                                                            */
    Board_KEY_RIGHT | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,  /* KEY
                                                                      RIGHT
                                                                           */
#elif defined (MODULE_CC26XX_5X5)
    Board_KEY_LEFT | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,   /* KEY LEFT
                                                                            */
    Board_KEY_RIGHT | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,  /* KEY
                                                                      RIGHT
                                                                           */
#elif defined (MODULE_CC26XX_4X4)
    /* Board_KEY_LEFT and Board_KEY_RIGHT are not connected to
      MODULE_CC26XX_4X4. */
#elif
#error \
    "Must define either 'MODULE_CC26XX_7X7', 'MODULE_CC26XX_5X5', or 'MODULE_CC26XX_4X4'."
#endif /* MODULE_CC26XX_7X7 */
    PIN_TERMINATE /* Terminate list */
};

/* KEY pin state */
static PIN_State keyPinState;

/* KEY Pin Handle */
PIN_Handle keyPinHandle;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * Enable interrupts for keys on PIN.
 *
 * Public function defined in board_key.h
 */
void Board_Key_initialize(Board_Key_keysPressedCB_t appKeyCB)
{
    /* Setup KEY ISR */
    keyPinHandle = PIN_open(&keyPinState, keyPinTable);

    /* Register Callbacks */
    PIN_registerIntCb(keyPinHandle, board_key_keyFxn);

    // Setup keycallback for keys
    Util_constructClock(&keyChangeClock, board_key_changeHandler,
                        KEY_DEBOUNCE_TIMEOUT, 0, false, 0);

    // Set the application callback
    appKeyChangeHandler = appKeyCB;
}

/*********************************************************************
 * @fn      board_key_keyFxn
 *
 * @brief   Interrupt handler for a Key press
 *
 */
void board_key_keyFxn(PIN_Handle keyPinHandle, PIN_Id keyPinId)
{
    (void)keyPinHandle;

    if(keyPinId == Board_KEY_SELECT)
    {
        keysPressed |= KEY_SELECT;
    }
    else if(keyPinId == Board_KEY_UP)
    {
        keysPressed |= KEY_UP;
    }
    else if(keyPinId == Board_KEY_DOWN)
    {
        keysPressed |= KEY_DOWN;
    }
#if defined (MODULE_CC26XX_7X7) || defined (MODULE_CC26XX_5X5)
    else if(keyPinId == Board_KEY_LEFT)
    {
        keysPressed |= KEY_LEFT;
    }
    else if(keyPinId == Board_KEY_RIGHT)
    {
        keysPressed |= KEY_RIGHT;
    }
#endif // (MODULE_CC26XX_7X7) || (MODULE_CC26XX_5X5)

    if(Util_isClockActive(&keyChangeClock) != true)
    {
        Util_startClock(&keyChangeClock);
    }
}

/*********************************************************************
 * @fn      board_key_changeHandler
 *
 * @brief   Handler for key change
 *
 * @param   UArg a0 - ignored
 *
 * @return  none
 */
static void board_key_changeHandler(UArg a0)
{
    if(appKeyChangeHandler != NULL)
    {
        // Notify the application
        (*appKeyChangeHandler)(keysPressed);

        // Clear keys
        keysPressed = 0;
    }
}

/*********************************************************************
 *********************************************************************/
