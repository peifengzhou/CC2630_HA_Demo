/*****************************************************************************
 Filename:       board_lcd.c
 Revised:        $Date: 2015-01-24 16:45:23 -0800 (Sat, 24 Jan 2015) $
 Revision:       $Revision: 42018 $

 Description:    This file contains the interface to the SRF06EB LCD Service.

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

#include <ti/drivers/PIN.h>
#include "Board.h"
#include "board_lcd.h"

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
#if defined (TI_DRIVERS_LCD_INCLUDED)

// LCD parameter
static LCD_Handle lcdHandle = NULL;

PIN_State pinState;

// Drive Board_3V3_EN high to enable 3.3V domain on SRF06EB
PIN_Config pinList[] =
{
    PIN_GPIO_OUTPUT_EN | PIN_PUSHPULL | PIN_GPIO_HIGH | Board_3V3_EN,
    PIN_TERMINATE
};

/* LCD Pin Handle */
PIN_Handle lcdPinHandle;

Char lcdBuffer0[LCD_BYTES] =
{ 0 };

LCD_Buffer lcdBuffers[] =
{
    { lcdBuffer0, LCD_BYTES, NULL },
};

#endif // TI_DRIVERS_LCD_INCLUDED

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

#if defined (TI_DRIVERS_LCD_INCLUDED)
/**
 * Open LCD peripheral on SRF06EB.
 *
 * Public function defined in board_lcd.h
 */
void Board_LCD_open(void)
{
    LCD_Params lcdParams;

    lcdPinHandle = PIN_open(&pinState, pinList);

    LCD_Params_init(&lcdParams);

    // Open LCD peripheral
    lcdHandle = LCD_open(&lcdBuffers[0], 1, &lcdParams);

    LCD_bufferClear(lcdHandle, 0);
    LCD_update(lcdHandle, 0);
}
#endif // TI_DRIVERS_LCD_INCLUDED

/**
 * Write a string on the LCD display.
 *
 * Public function defined in board_lcd.h
 */
void Board_Lcd_writeString(char *str, uint8_t line)
{
#if defined (TI_DRIVERS_LCD_INCLUDED)
    if(lcdHandle != NULL)
    {
        LCD_bufferClearPage(lcdHandle, 0, (LCD_Page)line);
        LCD_bufferPrintString(lcdHandle, 0, str, 0, (LCD_Page)line);
        LCD_update(lcdHandle, 0);
    }
#endif // TI_DRIVERS_LCD_INCLUDED
}

/**
 * Write a string and value on the LCD display.
 *
 * Public function defined in board_lcd.h
 */
void Board_Lcd_writeStringValue(char *str, uint16_t value, uint8_t format,
                                uint8_t line)
{
#if defined (TI_DRIVERS_LCD_INCLUDED)
    if(lcdHandle != NULL)
    {
        LCD_writeLine(lcdHandle, 0, str, value, format, line);
    }
#endif // TI_DRIVERS_LCD_INCLUDED
}

/*********************************************************************
 *********************************************************************/
