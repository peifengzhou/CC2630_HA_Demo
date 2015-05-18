/*******************************************************************************
 Filename:       board_lcd.h
 Revised:        $Date: 2014-09-30 11:22:21 -0700 (Tue, 30 Sep 2014) $
 Revision:       $Revision: 40345 $

 Description:    This file contains the LCD Service definitions
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

#ifndef BOARD_LCD_H
#define BOARD_LCD_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 \defgroup BoardLCD Board LCD Functions
 <BR>
 This module is a collection of functions to control the LCD.
 <BR>
 */

/*********************************************************************
 * INCLUDES
 */
#include <ti/drivers/lcd/LCDDogm1286.h>

/*********************************************************************
 *  EXTERNAL VARIABLES
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */
/**
 * \ingroup BoardLCD
 * @{
 */

// LCD macros
#if defined (TI_DRIVERS_LCD_INCLUDED)
/** Macro definition to write a string to the LCD */
#define LCD_WRITE_STRING(str, line) Board_Lcd_writeString(str, line)
/** Macro definition to write a string with a value to the LCD */
#define LCD_WRITE_STRING_VALUE(str, value, format, line) \
    Board_Lcd_writeStringValue(str, value, format, line)
#else
/** Macro definition to write a string to the LCD */
#define LCD_WRITE_STRING(str, line)
/** Macro definition to write a string with a value to the LCD */
#define LCD_WRITE_STRING_VALUE(str, value, format, line)
#endif

/*********************************************************************
 * API FUNCTIONS
 */

#if defined (TI_DRIVERS_LCD_INCLUDED)
/**
 * @brief   Open LCD peripheral on SRF06EB.
 */
extern void Board_LCD_open(void);
#else
#define Board_LCD_open()
#endif

/**
 * @brief   Write a string on the LCD display.
 *
 * @param   str - string to print
 * @param   line - line (page) to write (0-7)
 */
extern void Board_Lcd_writeString(char *str, uint8_t line);

/**
 * @brief   Write a string and value on the LCD display.
 *
 * @param   str - string to print
 * @param   value - value to print
 * @param   format - base of the value to print (2,8,16 etc)
 * @param   line - line (page) to write (0-7)
 */
extern void Board_Lcd_writeStringValue(char *str, uint16_t value,
                                       uint8_t format,
                                       uint8_t line);

/** @} end group BoardLCD */

/*********************************************************************
 *********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BOARD_LCD_H */
