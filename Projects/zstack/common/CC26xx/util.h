/******************************************************************************
 Filename:       util.h
 Revised:        $Date: 2015-01-24 16:45:23 -0800 (Sat, 24 Jan 2015) $
 Revision:       $Revision: 42018 $

 Description:    This file contains function declarations common to CC26xx
 TIRTOS Applications.

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

#ifndef UTIL_H
#define UTIL_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 \defgroup UtilMisc Utility and Miscellaneous
 <BR>
 Clock and miscellaneous string conversion functions.
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

//! Adjustment for the timers
#define TIMER_MS_ADJUSTMENT     100


/**
 * \ingroup UtilMisc
 * @{
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * API FUNCTIONS
 */

/**
 * @brief   Initialize a TIRTOS Clock instance.
 *
 * @param   pClock        - pointer to clock instance structure.
 * @param   clockCB       - callback function upon clock expiration.
 * @param   clockDuration - longevity of clock timer in milliseconds
 * @param   clockPeriod   - duration of a periodic clock, used continuously
 *                          after clockDuration expires.
 * @param   startFlag     - TRUE to start immediately, FALSE to wait.
 * @param   arg           - argument passed to callback function.
 *
 * @return  Clock_Handle  - a handle to the clock instance.
 */
extern Clock_Handle Util_constructClock(Clock_Struct *pClock,
                                        Clock_FuncPtr clockCB,
                                        uint32_t      clockDuration,
                                        uint32_t      clockPeriod,
                                        uint8_t       startFlag,
                                        UArg          arg);

/**
 * @brief   Start a clock.
 *
 * @param   pClock - pointer to clock struct
 */
extern void Util_startClock(Clock_Struct *pClock);

/**
 * @brief   Determine if a clock is currently active.
 *
 * @param   pClock - pointer to clock struct
 *
 * @return  TRUE or FALSE
 */
extern bool Util_isClockActive(Clock_Struct *pClock);

/**
 * @brief   Stop a clock.
 *
 * @param   pClock - pointer to clock struct
 */
extern void Util_stopClock(Clock_Struct *pClock);

/**
 * @brief   Converts from a uint16 to ascii hex string.
 *          The # will be exactly 4 hex digits (e.g. 0x0000 or 0x1E3F).
 *          NULL terminates the string.
 *
 * @param   u - Number to be converted
 * @param   string - pointer to coverted string
 */
extern void Util_uint16toa(uint16_t u, char *string);

/**
 * @brief   convert a 16bit number to ASCII
 *
 * @param   num - number to convert
 * @param   buf - buffer to write ASCII
 * @param   radix - base to convert to (ie. 10 or 16)
 */
extern void Util_itoa(uint16_t num, uint8_t *buf, uint8_t radix);

/**
 * @brief  convert a long unsigned int to a string.
 *
 * @param  l - long to convert
 * @param  buf - buffer to convert to
 * @param  radix - 10 dec, 16 hex
 *
 * @return  pointer to buffer
 */
extern unsigned char *Util_ltoa(uint32_t l, uint8_t *buf, uint8_t radix);

/** @} end group UtilMisc */

/*********************************************************************
 *********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* UTIL_H */
