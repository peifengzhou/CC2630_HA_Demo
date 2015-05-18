/******************************************************************************
 Filename:       board_key.c
 Revised:        $Date: 2014-10-22 10:53:04 -0700 (Wed, 22 Oct 2014) $
 Revision:       $Revision: 40729 $

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
 * CONSTANTS
 */
/** Debounce timeout in milliseconds */
#define KEY_DELAY  10

#define FACTORY_RESET_PRESS_PERIOD      6000

/*********************************************************************
 * TYPEDEFS
 */
typedef struct
{
  uint32_t tStart;
  uint32_t tStop;
} KeyPress_t;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Board_keyChangeHandler(UArg a0);
static void keyFxn(PIN_Handle keyPinHandle, PIN_Id keyPinId);

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Value of keys Pressed
static uint8_t keysPressed = 0;

// Key debounce clock
static Clock_Struct keyChangeClock;
static Clock_Handle keyClkHandle;

// Pointer to application callback
Board_Key_keysPressedCB_t appKeyChangeHandler = NULL;

/* Create the MSA KEY pin table. This will override the key attributes in
 * BoardGpioInitTable[].
 */
static PIN_Config keyPinTable[] =
{
    Board_KEY_LEFT | PIN_INPUT_EN | PIN_PULLUP |
                     PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS, // KEY LEFT
    Board_KEY_RIGHT | PIN_INPUT_EN | PIN_PULLUP |
                      PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS, // KEY RIGHT
    PIN_TERMINATE /* Terminate list */
};

/* KEY pin state */
static PIN_State keyPinState;

/* KEY Pin Handle */
PIN_Handle keyPinHandle;

static KeyPress_t rightKey = {0};
static KeyPress_t leftKey = {0};


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
    PIN_registerIntCb(keyPinHandle, keyFxn);

    // Setup keycallback for keys
    keyClkHandle = Util_constructClock(&keyChangeClock, Board_keyChangeHandler,
             (FACTORY_RESET_PRESS_PERIOD * TIMER_MS_ADJUSTMENT), 0, false, 0);

    // Set the application callback
    appKeyChangeHandler = appKeyCB;
}

/*********************************************************************
 * @fn      keyFxn
 *
 * @brief   Interrupt handler for a Key press
 *
 */
void keyFxn(PIN_Handle keyPinHandle, PIN_Id keyPinId)
{
    uint_t rightActive = 0;
    uint_t leftActive = 0;

    // Not using the input parameter
    (void)keyPinHandle;

    if(keyPinId == Board_KEY_RIGHT)
    {
        // Get the PIN value, it's active low
        if(PIN_getInputValue(Board_KEY_RIGHT) == 0)
        {
            // The right button is active
            rightActive = true;
        }

        // Look for the first press of the right key
        if(rightActive && ((keysPressed & KEY_RIGHT)==0))
        {
            keysPressed |= KEY_RIGHT;
            rightKey.tStart = Clock_getTicks();
        }

        // Look for the first release of the right key
        if((rightActive == false)
            && (keysPressed & KEY_RIGHT)
              && (rightKey.tStop == 0))
        {
            rightKey.tStop = Clock_getTicks();
        }

    }
    else if(keyPinId == Board_KEY_LEFT)
    {
        // Get the PIN value, it's active low
        if(PIN_getInputValue(Board_KEY_LEFT) == 0)
        {
            // The left button is active
            leftActive = true;
        }

        // Look for the first press of the left key
        if(leftActive && ((keysPressed & KEY_LEFT)==0))
        {
            keysPressed |= KEY_LEFT;
            leftKey.tStart = Clock_getTicks();
        }

        // Look for the first release of the left key
        if((leftActive == false)
            && (keysPressed & KEY_LEFT)
              && (leftKey.tStop == 0))
        {
            leftKey.tStop = Clock_getTicks();
        }
    }


    // Trigger a key press event
    if((leftActive == false) && (rightActive==false)
       && (keysPressed != 0))
    {
        // A key has been pressed and released
        // trigger a key press event immediately
        Board_keyChangeHandler(0);
    }
    else
    {
        // start the press and hold timer
        if(Util_isClockActive(&keyChangeClock) != true)
        {
            Util_stopClock(&keyChangeClock);
        }
        Clock_setTimeout(keyClkHandle,
                         (FACTORY_RESET_PRESS_PERIOD * TIMER_MS_ADJUSTMENT));
        Util_startClock(&keyChangeClock);
    }
}

/*********************************************************************
 * @fn      Board_keyChangeHandler
 *
 * @brief   Handler for key change
 *
 * @param   UArg a0 - ignored
 *
 * @return  none
 */
static void Board_keyChangeHandler(UArg a0)
{
    // Did we get here because of a press and hold timer
    if(Util_isClockActive(&keyChangeClock) != true)
    {
        // Yes, enter press stop values for left and right
        // keys if they were pressed.
        if((keysPressed & KEY_LEFT) && (leftKey.tStop == 0))
        {
            leftKey.tStop = Clock_getTicks();
        }
        if((keysPressed & KEY_RIGHT) && (rightKey.tStop == 0))
        {
            rightKey.tStop = Clock_getTicks();
        }
        Util_stopClock(&keyChangeClock);
    }

    if(appKeyChangeHandler != NULL)
    {
        // Have both keys been pressed?
        if(keysPressed == (KEY_LEFT | KEY_RIGHT))
        {
            int rightDuration = 0;
            int leftDuration = 0;

            // Calculate the key press duration for both keys
            if(rightKey.tStop > rightKey.tStart)
            {
                rightDuration = ((rightKey.tStop - rightKey.tStart)
                                * Clock_tickPeriod) / 1000;
            }
            if(leftKey.tStop > leftKey.tStart)
            {
                leftDuration = ((leftKey.tStop - leftKey.tStart)
                                * Clock_tickPeriod) / 1000;
            }

            // Have either key been pressed long enough
            // to trigger a factory reset
            if((rightDuration >= FACTORY_RESET_PRESS_PERIOD)
                || (leftDuration >= FACTORY_RESET_PRESS_PERIOD))
            {
              // Indicate to application that a factory reset
              // should happen by saying that all the keys possible
              // have been pressed, even tho the SensorTag only has
              // the left and right keys defined.
              keysPressed = (KEY_SELECT | KEY_UP |
                             KEY_DOWN | KEY_LEFT | KEY_RIGHT);
            }
        }

        // Notify the application
        (*appKeyChangeHandler)(keysPressed);

        // Clear keys
        keysPressed = 0;
        rightKey.tStart = 0;
        rightKey.tStop = 0;
        leftKey.tStart = 0;
        leftKey.tStop = 0;
    }
}

/*********************************************************************
 *********************************************************************/
