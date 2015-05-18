/******************************************************************************
 Filename:       util.c
 Revised:        $Date: 2015-01-24 16:45:23 -0800 (Sat, 24 Jan 2015) $
 Revision:       $Revision: 42018 $

 Description:    This file contains utility functions commonly used by
 ZStack applications for CC26xx with TIRTOS.

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
#include <string.h>

#include <stdbool.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <ICall.h>

#include "util.h"

/*********************************************************************
 * TYPEDEFS
 */

// RTOS queue for profile/app messages.
typedef struct _queueRec_
{
    Queue_Elem _elem;    // queue element
    uint8_t *pData;      // pointer to app data
} queueRec_t;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * Initialize a TIRTOS Clock instance.
 *
 * Public function defined in util.h
 */
Clock_Handle Util_constructClock(Clock_Struct *pClock, Clock_FuncPtr clockCB,
                                 uint32_t clockDuration, uint32_t clockPeriod,
                                 uint8_t startFlag,
                                 UArg arg)
{
    Clock_Params clockParams;

    // Convert clockDuration in milliseconds to ticks.
    uint32_t clockTicks = clockDuration * (1000 / Clock_tickPeriod);

    // Setup parameters.
    Clock_Params_init(&clockParams);

    // Setup argument.
    clockParams.arg = arg;

    // If period is 0, this is a one-shot timer.
    clockParams.period = clockPeriod * (1000 / Clock_tickPeriod);

    // Starts immediately after construction if true, otherwise wait for a
    // call to start.
    clockParams.startFlag = startFlag;

    // Initialize clock instance.
    Clock_construct(pClock, clockCB, clockTicks, &clockParams);

    return Clock_handle(pClock);
}

/**
 * Start a clock.
 *
 * Public function defined in util.h
 */
void Util_startClock(Clock_Struct *pClock)
{
    Clock_Handle handle = Clock_handle(pClock);

    // Start clock instance
    Clock_start(handle);
}

/**
 * Determine if a clock is currently active.
 *
 * Public function defined in util.h
 */
bool Util_isClockActive(Clock_Struct *pClock)
{
    Clock_Handle handle = Clock_handle(pClock);

    // Start clock instance
    return Clock_isActive(handle);
}

/**
 * Stop a clock.
 *
 * Public function defined in util.h
 */
void Util_stopClock(Clock_Struct *pClock)
{
    Clock_Handle handle = Clock_handle(pClock);

    // Start clock instance
    Clock_stop(handle);
}

/**
 * Converts from a uint16 to ascii hex string.
 *
 * Public function defined in util.h
 */
void Util_uint16toa(uint16_t u, char *string)
{
    if(string == NULL)
    {
        return;
    }

    // add preceding zeros
    if(u < 0x1000)
    {
        *string++ = '0';
    }

    if(u < 0x0100)
    {
        *string++ = '0';
    }

    if(u < 0x0010)
    {
        *string++ = '0';
    }

    Util_ltoa( (unsigned long)u, (unsigned char *)string, 16 );
}

/**
 * Convert a 16bit number to ASCII
 *
 * Public function defined in util.h
 */
void Util_itoa(uint16_t num, uint8_t *buf, uint8_t radix)
{
    char c, i;
    uint8_t *p, rst[5];

    p = rst;
    for(i = 0; i < 5; i++, p++)
    {
        c = num % radix;  // Isolate a digit
        *p = c + ( (c < 10) ? '0' : '7' );  // Convert to Ascii
        num /= radix;
        if(!num)
        {
            break;
        }
    }

    for(c = 0; c <= i; c++)
    {
        *buf++ = *p--;  // Reverse character order
    }

    *buf = '\0';
}

/**
 * Convert a long unsigned int to a string.
 *
 * Public function defined in util.h
 */
unsigned char *Util_ltoa(uint32_t l, uint8_t *buf, uint8_t radix)
{
#if defined (__GNUC__)
    return( (char *)ltoa(l, buf, radix) );
#else
    unsigned char tmp1[10] = "", tmp2[10] = "", tmp3[10] = "";
    unsigned short num1, num2, num3;
    unsigned char i;

    buf[0] = '\0';

    if(radix == 10)
    {
        num1 = l % 10000;
        num2 = (l / 10000) % 10000;
        num3 = (unsigned short)(l / 100000000);

        if(num3)
        {
            Util_itoa(num3, tmp3, 10);
        }
        if(num2)
        {
            Util_itoa(num2, tmp2, 10);
        }
        if(num1)
        {
            Util_itoa(num1, tmp1, 10);
        }

        if(num3)
        {
            strcpy( (char *)buf, (char const *)tmp3 );
            for(i = 0; i < 4 - strlen( (char const *)tmp2 ); i++)
            {
                strcat( (char *)buf, "0" );
            }
        }
        strcat( (char *)buf, (char const *)tmp2 );
        if(num3 || num2)
        {
            for(i = 0; i < 4 - strlen( (char const *)tmp1 ); i++)
            {
                strcat( (char *)buf, "0" );
            }
        }
        strcat( (char *)buf, (char const *)tmp1 );
        if(!num3 && !num2 && !num1)
        {
            strcpy( (char *)buf, "0" );
        }
    }
    else if(radix == 16)
    {
        num1 = l & 0x0000FFFF;
        num2 = l >> 16;

        if(num2)
        {
            Util_itoa(num2, tmp2, 16);
        }
        if(num1)
        {
            Util_itoa(num1, tmp1, 16);
        }

        if(num2)
        {
            strcpy( (char *)buf, (char const *)tmp2 );
            for(i = 0; i < 4 - strlen( (char const *)tmp1 ); i++)
            {
                strcat( (char *)buf, "0" );
            }
        }
        strcat( (char *)buf, (char const *)tmp1 );
        if(!num2 && !num1)
        {
            strcpy( (char *)buf, "0" );
        }
    }
    else
    {
        return(NULL);
    }

    return(buf);
#endif
}

/*********************************************************************
 *********************************************************************/
