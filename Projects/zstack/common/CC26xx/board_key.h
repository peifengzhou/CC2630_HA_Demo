/******************************************************************************
 Filename:       board_key.h
 Revised:        $Date: 2014-09-30 11:22:21 -0700 (Tue, 30 Sep 2014) $
 Revision:       $Revision: 40345 $

 Description:    This file contains the Key press Service definitions
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
 *****************************************************************************/

#ifndef BOARD_KEY_H
#define BOARD_KEY_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 \defgroup BoardKey Keypress Functions
 <BR>
 This module handles the definition, initilization and key bounce for key
 presses.
 To use this module, call Board_Key_initialize() with a callback function as
 the parameter, then when a key or keys are pressed, the callback function
 will be called (after a debounce period).  Single or multiple key presses are
 detected and passed to the callback function.
 <BR>
 */

/*********************************************************************
 * INCLUDES
 */
#include "Board.h"

/*********************************************************************
 *  EXTERNAL VARIABLES
 */

/*********************************************************************
 * CONSTANTS
 */

/**
 * \ingroup BoardKey
 * @{
 */

/** Select Key ID */
#define KEY_SELECT            0x01
/** Up Key ID */
#define KEY_UP                0x02
/** Down Key ID */
#define KEY_DOWN              0x04
/** Left Key ID */
#define KEY_LEFT              0x08
/** Right Key ID */
#define KEY_RIGHT             0x10

/** Debounce timeout in milliseconds */
#define KEY_DEBOUNCE_TIMEOUT  200

/*********************************************************************
 * TYPEDEFS
 */

/** Key Press Callback function typedef */
typedef void (*Board_Key_keysPressedCB_t)(uint8_t keysPressed);

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * API FUNCTIONS
 */

/**
 * @brief   Enable interrupts for keys on GPIOs.
 *
 * @param   appKeyCB - application key pressed callback
 */
void Board_Key_initialize(Board_Key_keysPressedCB_t appKeyCB);

/** @} end group BoardKey */

/*********************************************************************
 *********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BOARD_KEY_H */
