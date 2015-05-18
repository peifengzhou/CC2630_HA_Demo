/**
 @file  appterminft.h
 @brief Application Terminal Interface structures

 <!--
 Copyright 2015 Texas Instruments Incorporated. All rights reserved.

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
 PROVIDED ``AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
 -->
 */
#ifndef APPTERMINTF_H
#define APPTERMINTF_H

#include <stdbool.h>
#include <stdint.h>
#include <ICall.h>

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
// Constants and definitions
//*****************************************************************************

typedef enum
{
    APPTERM_CmdIDs_INIT_REQ = 0x00,
    APPTERM_CmdIDs_KEYPRESS = 0x01,
    APPTERM_CmdIDs_LCD_REQ = 0x02,
    APPTERM_CmdIDs_LED_REQ = 0x03,
} APPTERM_CmdIDs;

//*****************************************************************************
// Structures - Building blocks for the AppTerm API structures
//*****************************************************************************

/** LED Identifications */
typedef enum
{
    APPTERM_ledType_LED1,
    APPTERM_ledType_LED2,
    APPTERM_ledType_LED3,
    APPTERM_ledType_LED4,
} APPTERM_ledType;

/** LED States */
typedef enum
{
    APPTERM_led_state_OFF,
    APPTERM_led_state_ON,
    APPTERM_led_state_BLINK,
    APPTERM_led_state_BLINKING,
    APPTERM_led_state_TOGGLE
} APPTERM_ledState;

/** Select Key ID */
#define APPTERM_KEY_SELECT            0x01
/** Up Key ID */
#define APPTERM_KEY_UP                0x02
/** Down Key ID */
#define APPTERM_KEY_DOWN              0x04
/** Left Key ID */
#define APPTERM_KEY_LEFT              0x08
/** Right Key ID */
#define APPTERM_KEY_RIGHT             0x10

//*****************************************************************************
// System Interface Request Structures
//*****************************************************************************

/**
 * Header structure for ZNP iCall Message.
 */
typedef struct _appterm_hdr_t
{
    /** event */
    uint_least8_t event;
    /** Will hold the default response status field. */
    uint_least8_t status;
} AppTerm_Hdr_t;

/**
 * Structure to send an Initialization request from the AppTerm.
 */
typedef struct _appterm_initreq_t
{

    AppTerm_Hdr_t hdr;

} AppTerm_InitReq_t;

/**
 * Structure to send an keypress indication from the AppTerm.
 */
typedef struct _appterm_keypress_t
{

    AppTerm_Hdr_t hdr;

    /**
     * Bit mask - example Key Up and Key Down  is
     *  APPTERM_KEY_UP | APPTERM_KEY_DOWN
     */
    uint8_t keypress;

} AppTerm_Keypress_t;

/**
 * Structure to send an LCD request to the AppTerm.
 */
typedef struct _appterm_lcdreq_t
{

    AppTerm_Hdr_t hdr;

    /**
     * LCD string size
     */
    uint8_t lcdLen;
    /**
     * LCD String
     */
    uint8_t *pLcdString;

    /**
     * LCD Line number
     */
    uint8_t lineNumber;
} AppTerm_LcdReq_t;

/**
 * Structure to send an LED request to the AppTerm.
 */
typedef struct _appterm_ledreq_t
{

    AppTerm_Hdr_t hdr;

    /** LED */
    APPTERM_ledType ledType;

    /** LED State */
    APPTERM_ledState ledState;

} AppTerm_LedReq_t;

//*****************************************************************************
//*****************************************************************************

#ifdef __cplusplus
}
#endif

#endif /* ZNP_H */

