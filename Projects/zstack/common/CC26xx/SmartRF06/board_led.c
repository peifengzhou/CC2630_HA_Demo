/*****************************************************************************
 Filename:       board_led.c
 Revised:        $Date: 2015-01-24 16:45:23 -0800 (Sat, 24 Jan 2015) $
 Revision:       $Revision: 42018 $

 Description:    TThis file contains the interface to the SRF06EB LED Service.

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
 ****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <xdc/std.h>

#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/PIN.h>

#include "Board.h"

#include "util.h"
#include "board_led.h"

/*********************************************************************
 * TYPEDEFS
 */
#define BOARD_LED_BLINK_PERIOD 500     // in milliseconds
#define MAX_LEDS 4

typedef enum
{
    BLINKING_STATUS_ON,
    BLINKING_STATUS_OFF,
    BLINKING_STATUS_DONE
} blinkStatus;

typedef struct
{
    board_led_state state;    // Off, On or Blink
    blinkStatus status;       // What is led status (on, off, or done)
} board_led_status_t;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void board_led_blinkTimeoutCB(UArg a0);
static bool board_led_anyBlinking(void);
static void board_led_blinkLed(void);
static unsigned int board_led_convertLedType(board_led_type led);
static uint32_t board_led_convertLedValue(board_led_state state);

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static Clock_Struct blinkClkStruct;

static board_led_status_t ledStatus[MAX_LEDS];

/* SensorTag LED has exactly the same attributes as that of
 * BoardGpioInitTable[]. There is no need to create a new one.
 */
static PIN_Config ledPinTable[] =
{
#if defined (MODULE_CC26XX_7X7)
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED1 initially off */
    Board_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED2 initially off */
#elif defined (MODULE_CC26XX_5X5)
    /* Board_LED1 and Board_LED2 are not connected to MODULE_CC26XX_5X5. */
#elif defined (MODULE_CC26XX_4X4)
    /* Board_LED1 and Board_LED2 are not connected to MODULE_CC26XX_4X4. */
#elif
#error "Must define either 'MODULE_CC26XX_7X7', 'MODULE_CC26XX_5X5', or 'MODULE_CC26XX_4X4'."
#endif /* MODULE_CC26XX_7X7 */
    Board_LED3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED3 initially off */
    Board_LED4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX, /* LED4 initially off */
    PIN_TERMINATE /* Terminate list     */
};

/* LED pin state */
static PIN_State ledPinState;

/* LED Pin Handle */
PIN_Handle ledPinHandle;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * Initialize LEDs
 *
 * Public function defined in board_led.h
 */
void Board_Led_initialize(void)
{
    uint8_t x;
    unsigned int index;
    uint32_t value;

    /* Open LED PIN driver */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);

    value = board_led_convertLedValue(board_led_state_OFF);

    for(x = 0; x < MAX_LEDS; x++)
    {
        ledStatus[x].state = board_led_state_OFF;
        ledStatus[x].status = BLINKING_STATUS_DONE;

        index = board_led_convertLedType( (board_led_type)x );

        PIN_setOutputValue(ledPinHandle, index, value);
    }

    Util_constructClock(&blinkClkStruct, board_led_blinkTimeoutCB,
                        BOARD_LED_BLINK_PERIOD, 0, false, 0);
}

/**
 * Control the state of an LED
 *
 * Public function defined in board_led.h
 */
void Board_Led_control(board_led_type led, board_led_state state)
{
    unsigned int gpioType;
    uint32_t value;

    // Look for invalid parameters
    if( (led >= MAX_LEDS) || (state > board_led_state_BLINKING) )
    {
        return;
    }

    // Convert to GPIO types
    gpioType = board_led_convertLedType(led);
    value = board_led_convertLedValue(state);

    // Save state and status
    ledStatus[led].state = state;
    if( (state == board_led_state_BLINK)
        || (state == board_led_state_BLINKING) )
    {
        ledStatus[led].status = BLINKING_STATUS_ON;
    }

    // Update hardware LEDs
    PIN_setOutputValue(ledPinHandle, gpioType, value);

    // Are any LEDs are blinking?
    if( board_led_anyBlinking() )
    {
        if(Util_isClockActive(&blinkClkStruct) == false)
        {
            Util_startClock(&blinkClkStruct);
        }
    }
    else
    {
        if(Util_isClockActive(&blinkClkStruct) == true)
        {
            Util_stopClock(&blinkClkStruct);
        }
    }
}

/**
 * Toggle the state of an LED
 *
 * Public function defined in board_led.h
 */
void Board_Led_toggle(board_led_type led)
{
    board_led_state newState = board_led_state_OFF;

    // Look for invalid parameter
    if(led < MAX_LEDS)
    {
        // Toggle state
        if(ledStatus[led].state == board_led_state_OFF)
        {
            newState = board_led_state_ON;
        }

        // Set new state
        Board_Led_control(led, newState);
    }
}

/***************************************************************************
 * @fn      board_led_blinkTimeoutCB
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void board_led_blinkTimeoutCB(UArg a0)
{
    // Update blinking LEDs
    board_led_blinkLed();

    if( board_led_anyBlinking() )
    {
        // Setup for next time
        Util_startClock(&blinkClkStruct);
    }
}

/***************************************************************************
 * @fn      board_led_anyBlinking
 *
 * @brief   Are there any blinking LEDs?
 *
 * @param   none
 *
 * @return  true, yes at least one.  false if none
 */
static bool board_led_anyBlinking(void)
{
    uint8_t x;

    for(x = 0; x < MAX_LEDS; x++)
    {
        if( (ledStatus[x].state == board_led_state_BLINKING)
            || ( (ledStatus[x].state == board_led_state_BLINK)
                 && (ledStatus[x].status != BLINKING_STATUS_DONE) ) )
        {
            return(true);
        }
    }

    return(false);
}

/***************************************************************************
 * @fn      board_led_blinkLed
 *
 * @brief   Blink LEDs
 *
 * @param   none
 *
 * @return  none
 */
static void board_led_blinkLed(void)
{
    uint8_t x;

    for(x = 0; x < MAX_LEDS; x++)
    {
        unsigned int index;
        uint32_t value;

        if(ledStatus[x].state == board_led_state_BLINKING)
        {
            index = board_led_convertLedType( (board_led_type)x );

            if(ledStatus[x].status == BLINKING_STATUS_OFF)
            {
                value = board_led_convertLedValue(board_led_state_ON);
                ledStatus[x].status = BLINKING_STATUS_ON;
            }
            else
            {
                value = board_led_convertLedValue(board_led_state_OFF);
                ledStatus[x].status = BLINKING_STATUS_OFF;
            }

            PIN_setOutputValue(ledPinHandle, index, value);
        }
        else if( (ledStatus[x].state == board_led_state_BLINK)
                 && (ledStatus[x].status != BLINKING_STATUS_DONE) )
        {
            index = board_led_convertLedType( (board_led_type)x );
            value = board_led_convertLedValue(board_led_state_OFF);
            ledStatus[x].status = BLINKING_STATUS_DONE;

            PIN_setOutputValue(ledPinHandle, index, value);
        }
    }
}

/***************************************************************************
 * @fn      board_led_convertLedType
 *
 * @brief   Convert from board_led type to PIN led type
 *
 * @param   led - board_led type
 *
 * @return  PIN Led Type
 */
static unsigned int board_led_convertLedType(board_led_type led)
{
    unsigned int ledType = Board_LED3;

    switch(led)
    {
#if defined (MODULE_CC26XX_7X7)
        case board_led_type_LED1:
            ledType = Board_LED1;
            break;

        case board_led_type_LED2:
            ledType = Board_LED2;
            break;
#endif // MODULE_CC26XX_7X7

        case board_led_type_LED4:
            ledType = Board_LED4;
            break;
    }

    return(ledType);
}

/***************************************************************************
 * @fn      board_led_convertLedValue
 *
 * @brief   Convert from board_led value to GPIO value
 *
 * @param   led - board_led value
 *
 * @return  GPIO value
 */
static uint32_t board_led_convertLedValue(board_led_state state)
{
    uint32_t value = Board_LED_OFF;

    switch(state)
    {
        case board_led_state_ON:
        case board_led_state_BLINK:
        case board_led_state_BLINKING:
            value = Board_LED_ON;
            break;
    }

    return(value);
}

/*********************************************************************
 *********************************************************************/
