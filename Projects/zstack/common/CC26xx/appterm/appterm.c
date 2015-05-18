/**
 @file  appterm.c
 @brief TI RTOS application terminal
 interfacing with ZStack.

 The application interacts with the ZStack Thread
 via both messaging interface and C function interface.

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
 -->
 */

/**


 KEY UP (or S1)
 -
 |
 KEY LEFT (or S4)               |      KEY RIGHT(or S2)
 -                            --+-----  -
 |
 |
 KEY DOWN
 -

 KEY_SELECT (or S5)
 -

 */

//*****************************************************************************
// Includes
//*****************************************************************************
#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <string.h>
#include <inc/hw_ints.h>
#include "ICall.h"

#include <ioc.h>

#include "Board.h"

#include "board_key.h"
#include "board_lcd.h"
#include "board_led.h"

#include "appterm.h"
#include "apptermintf.h"

//*****************************************************************************
// Constants
//*****************************************************************************

#define APPTERM_ABORT() ICall_abort()

/* Event IDs */
#define APPTERM_KEY_EVENT                 0x0020

/* Debounce timeout in ticks */
#define KEY_DEBOUNCE_TIMEOUT      200

//*****************************************************************************
// Global Variables
//*****************************************************************************

// Semaphore globally used to post events to the application thread
ICall_Semaphore sem;
ICall_EntityID appTermEntity;

//*****************************************************************************
// Local Variables
//*****************************************************************************

NVINTF_nvFuncts_t *pfnAppTermNV = NULL;

// Task pending events
static uint16_t events = 0;

/* Key press parameters */
static uint8_t keys;

/**
 * @internal A semaphore used to wait on a clock event
 */
extern ti_sysbios_knl_Semaphore_Handle semaphore0;

// Handle for LCD.
LCD_Handle lcdHandle;

//*****************************************************************************
// Local Function Prototypes
//*****************************************************************************

static void appTermInit(void);
static void appTermInitClocks(void);
static void appTermProcess(void);
static uint16_t appTermProcessEvent(AppTerm_InitReq_t *pMsg);
static void appTermHandleKeys(uint8_t keys);

static void appTermKeyChangeCB(uint8_t keysPressed);

bool appTermFreeMsg(void *pMsg);
void appTermSendInitReq(void);
void appTermSendKeyPress(uint8_t keys);

/**
 * @internal Clock handler function
 * @param a0 ignored
 */
Void tirtosapp_clock(UArg a0)
{
    /* Wake up the application thread when it waits for clock event */
    Semaphore_post(semaphore0);
}

/*******************************************************************************
 * @fn          appterm_task
 *
 * @brief       Application task entry point for the App Terminal.
 *
 * @param       pfnNV - pointer to the NV functions
 *
 * @return      none
 */
void appterm_task(NVINTF_nvFuncts_t *pfnNV)
{
    // Save and register the function pointers to the NV drivers
    pfnAppTermNV = pfnNV;

    // Initialize application
    appTermInit();

    // No return from task process
    appTermProcess();
}

/*******************************************************************************
 * @fn          appTermInit
 *
 * @brief       Initialize the application
 *
 * @param       none
 *
 * @return      none
 */
static void appTermInit(void)
{
    appTermInitClocks();

    /* Initialize keys */
    Board_Key_initialize(appTermKeyChangeCB);

    /* Initialize the LCD */
    Board_LCD_open();
    LCD_WRITE_STRING("Application Terminal", LCD_PAGE1);

    /* Initialize the LEDS */
    Board_Led_initialize();

    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&appTermEntity, &sem);

    appTermSendInitReq();
}

/*******************************************************************************
 * @fn      appTermInitClocks
 *
 * @brief   Initialize Clocks
 *
 * @param   none
 *
 * @return  none
 */
static void appTermInitClocks(void)
{
    // Initialize timers
}

/*******************************************************************************
 * @fn      appTermProcess
 *
 * @brief   Application task processing start.
 *
 * @param   none
 *
 * @return  void
 */
static void appTermProcess(void)
{
    /* Forever loop */
    for (;;)
    {
        ICall_ServiceEnum stackid;
        ICall_EntityID dest;
        AppTerm_InitReq_t *pMsg;

        /* Wait for response message */
        if (ICall_wait(ICALL_TIMEOUT_FOREVER) == ICALL_ERRNO_SUCCESS)
        {
            /* Retrieve the response message */
            if (ICall_fetchServiceMsg(&stackid, &dest, (void **) &pMsg)
                    == ICALL_ERRNO_SUCCESS)
            {
                if ((stackid == ICALL_SERVICE_CLASS_ZSTACK)
                        && (dest == appTermEntity))
                {
                    if (pMsg)
                    {
                        appTermProcessEvent(pMsg);

                        // Free any seperately allocated memory
                        appTermFreeMsg(pMsg);
                    }
                }

                if (pMsg)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            if (events & APPTERM_KEY_EVENT)
            {
                // Process Key Presses
                appTermHandleKeys(keys);
                keys = 0;
                events &= ~APPTERM_KEY_EVENT;
            }
        }
    }
}

/*******************************************************************************
 * @fn      appTermProcessEvent
 *
 * @brief   Process event from Stack
 *
 * @param   zstackmsg_genericReq_t
 *
 * @return  status
 */
static uint16_t appTermProcessEvent(AppTerm_InitReq_t *pMsg)
{
    switch(pMsg->hdr.event)
    {
        case APPTERM_CmdIDs_LCD_REQ:
            {
                AppTerm_LcdReq_t *pInd = (AppTerm_LcdReq_t *) pMsg;

                LCD_WRITE_STRING((char *) pInd->pLcdString,
                        (uint8_t) pInd->lineNumber);
            }
            break;

        case APPTERM_CmdIDs_LED_REQ:
            {
                AppTerm_LedReq_t *pInd = (AppTerm_LedReq_t *) pMsg;

                if (pInd->ledState == APPTERM_led_state_TOGGLE)
                {
                    Board_Led_toggle((board_led_type) pInd->ledType);
                }
                else
                {
                    Board_Led_control((board_led_type) pInd->ledType,
                            (board_led_state) pInd->ledState);
                }
            }
            break;

        default:
            break;
    }

    return 0;
}

/*********************************************************************
 * @fn      appTermKeyChangeCB
 *
 * @brief   Key event handler function
 *
 * @param   keysPressed
 *
 * @return  none
 */
static void appTermKeyChangeCB(uint8_t keysPressed)
{
    keys = keysPressed;

    events |= APPTERM_KEY_EVENT;

    // Wake up the application thread when it waits for clock event
    Semaphore_post(sem);
}

/*******************************************************************************
 * @fn      appTermHandleKeys
 *
 * @brief   Callback service for keys
 *
 * @param   keys  - keys that were pressed
 *          state - shifted
 *
 * @return  void
 */
static void appTermHandleKeys(uint8_t keys)
{
    appTermSendKeyPress(keys);
}

/*******************************************************************************
 * @fn      appTermFreeMsg
 *
 * @brief   Call to free an iCall message
 *
 * @param   pMsg  - pointer to message to free
 *
 * @return  true if freed, flase if not
 */
bool appTermFreeMsg(void *pMsg)
{
    bool processed = true;
    AppTerm_InitReq_t *pTemp = (AppTerm_InitReq_t *) pMsg;

    switch(pTemp->hdr.event)
    {
        case APPTERM_CmdIDs_LCD_REQ:
            {
                AppTerm_LcdReq_t *pInd = (AppTerm_LcdReq_t *) pMsg;

                if (pInd->pLcdString)
                {
                    ICall_free(pInd->pLcdString);
                }
                ICall_freeMsg(pMsg);
            }
            break;

        case APPTERM_CmdIDs_INIT_REQ:
        case APPTERM_CmdIDs_LED_REQ:
        case APPTERM_CmdIDs_KEYPRESS:
            ICall_freeMsg(pMsg);
            break;

        default:  // Ignore the other messages
            processed = false;
            break;
    }

    return (processed);
}

/*******************************************************************************
 * @fn      appTermSendInitReq
 *
 * @brief   Call to send an init request from the AppTerm to the ZStack
 *
 * @param   none
 *
 * @return  none
 */
void appTermSendInitReq(void)
{
    /* Allocate message buffer space */
    AppTerm_InitReq_t *pMsg = (AppTerm_InitReq_t *) ICall_allocMsg(
            sizeof(AppTerm_InitReq_t));

    if (pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = APPTERM_CmdIDs_INIT_REQ;
        pMsg->hdr.status = 0;

        /* Send the message */
        (void) ICall_sendServiceMsg(appTermEntity, ICALL_SERVICE_CLASS_ZSTACK,
                ICALL_MSG_FORMAT_KEEP, pMsg);
    }
}

/*******************************************************************************
 * @fn      appTermSendKeyPress
 *
 * @brief   Call to send an keypress indication from the AppTerm to the ZStack
 *
 * @param   keys - what keys were pressed
 *
 * @return  none
 */
void appTermSendKeyPress(uint8_t keys)
{
    /* Allocate message buffer space */
    AppTerm_Keypress_t *pMsg = (AppTerm_Keypress_t *) ICall_allocMsg(
            sizeof(AppTerm_Keypress_t));

    if (pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = APPTERM_CmdIDs_KEYPRESS;
        pMsg->hdr.status = 0;

        pMsg->keypress = keys;

        /* Send the message */
        (void) ICall_sendServiceMsg(appTermEntity, ICALL_SERVICE_CLASS_ZSTACK,
                ICALL_MSG_FORMAT_KEEP, pMsg);
    }
}

/*******************************************************************************
 ******************************************************************************/
