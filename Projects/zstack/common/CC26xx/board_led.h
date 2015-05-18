/*******************************************************************************
 Filename:       board_led.h
 Revised:        $Date: 2014-09-30 11:22:21 -0700 (Tue, 30 Sep 2014) $
 Revision:       $Revision: 40345 $

 Description:    This file contains the LED Service definitions
 and prototypes.

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
 *******************************************************************************/

#ifndef BOARD_LED_H
#define BOARD_LED_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 \defgroup BoardLED Board LED Functions
 <BR>
 This module is a collection of functions to control LEDs.
 <BR>
 */

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 *  EXTERNAL VARIABLES
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/**
 * \ingroup BoardLED
 * @{
 */

/** LED Identifications */
typedef enum
{
    board_led_type_LED1,
    board_led_type_LED2,
    board_led_type_LED3,
    board_led_type_LED4,
} board_led_type;

/** LED States */
typedef enum
{
    board_led_state_OFF,
    board_led_state_ON,
    board_led_state_BLINK,
    board_led_state_BLINKING,
} board_led_state;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * API FUNCTIONS
 */

/**
 * @brief   Initialize LED System
 */
void Board_Led_initialize(void);

/**
 * @brief   Control the state of an LED
 *
 * @param   led - Which LED
 * @param   state - control, set to board_led_state_BLINK for blink once,
 *                  set to board_led_state_BLINKING to continual blinking.
 */
void Board_Led_control(board_led_type led, board_led_state state);

/**
 * @brief   Toggle the state of an LED from off to on, and from on
 *          or blinking to off.
 *
 * @param   led - Which LED
 */
void Board_Led_toggle(board_led_type led);

/** @} end group BoardLED */

/*********************************************************************
 *********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BOARD_LED_H */
