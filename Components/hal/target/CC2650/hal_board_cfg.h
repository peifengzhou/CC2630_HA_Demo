/**************************************************************************************************
  Filename:       hal_board_cfg.h
  Revised:        $Date: 2011-07-01 10:03:18 -0700 (Fri, 01 Jul 2011) $
  Revision:       $Revision: 26540 $

  Description:    Platform-specific definitions for the DK-LM3S9B96


  Copyright 2011-2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
**************************************************************************************************/

#ifndef HAL_BOARD_CFG_H
#define HAL_BOARD_CFG_H

/*
 *     =============================================================
 *     |                  CC26xxEM EVAL BOARD                      |
 *     | --------------------------------------------------------- |
 *     |  mcu   : Texas Instruments cc26xx                         |
 *     |  clock : 24 MHz                                           |
 *     =============================================================
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_assert.h"
#include "bsp.h"
#include "bsp_led.h"
#include "bsp_key.h"

/* ------------------------------------------------------------------------------------------------
 *                                       Board Indentifiers
 * ------------------------------------------------------------------------------------------------
 */

/* Identify specific target */
#define HAL_BOARD_CC2538

/* ------------------------------------------------------------------------------------------------
 *                                          Clock Speed
 *
 *   Note that when the HAL_CPU_CLOCK_MHZ is changed, the HAL_CLOCK_INIT
 *   macro must also be changed.
 * ------------------------------------------------------------------------------------------------
 */

#define HAL_CPU_CLOCK_MHZ       24

/* ------------------------------------------------------------------------------------------------
 *                                        Interrupt Priorities
 * ------------------------------------------------------------------------------------------------
 */

/* sleep timer interrupt */
#define HAL_INT_PRIOR_ST        (4 << 5)

/* MAC interrupts */
#define HAL_INT_PRIOR_MAC       (4 << 5)

/* UART interrupt */
#define HAL_INT_PRIOR_UART      (5 << 5)

/* Normally, the KEY interrupt priority can be lower than ST, MAC, and UART,
 * but LM3S9B96 shares the TX_DONE interrupt with KEY interrupt at PortJ.
 * Make the KEY interrupt priority the same as that of MAC. Otherwise,
 * a FIFOP interrupt may preempt the TX_DONE interrupt and cause an assert.
 */
/* Keyboard interrupt */
#define HAL_INT_PRIOR_KEY       (4 << 5)

#define HAL_CLOCK_STABLE()    st( while (CLOCK_STA != (CLK_STA_32MHZ | OSC_32KHZ)); )

/* ------------------------------------------------------------------------------------------------
 *                                       LED Configuration
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_NUM_LEDS            4

#define HAL_LED_BLINK_DELAY()   st( { volatile uint32 i; for (i=0; i<0x34000; i++) { }; } )

/* LED1 - RED */
#define LED1_BV           BV(0)
#define LED1_PORT         GPIO_C_BASE
#define LED1_DDR

/* LED2 - YELLOW */
#define LED2_BV           BV(1)
#define LED2_PORT         GPIO_C_BASE
#define LED2_DDR

/* LED3 - GREEN */
#define LED3_BV           BV(2)
#define LED3_PORT         GPIO_C_BASE
#define LED3_DDR

/* LED4 - RED ORANGE */
#define LED4_BV           BV(3)
#define LED4_PORT         GPIO_C_BASE
#define LED4_DDR



/* ------------------------------------------------------------------------------------------------
 *                                       LCD Configuration
 * ------------------------------------------------------------------------------------------------
 */

/* Board LCD defines */
#define BSP_LCD_PWR_BASE        GPIO_B_BASE
#define BSP_LCD_PWR             GPIO_PIN_4      // PB4

/* LCD Max Chars and Buffer */
#define HAL_LCD_MAX_CHARS   (LCD_COLS / LCD_CHAR_WIDTH)
#define HAL_LCD_MAX_BUFF    (LCD_BYTES / LCD_CHAR_WIDTH)

/* ------------------------------------------------------------------------------------------------
 *                                       KEY Configuration
 * ------------------------------------------------------------------------------------------------
 */

/* Interrupt option - Enable or disable */
// TODO: MOVE TO hal_key.h FOR CC26xx?
//#define HAL_KEY_INTERRUPT_DISABLE    BSP_KEY_MODE_POLL
//#define HAL_KEY_INTERRUPT_ENABLE     BSP_KEY_MODE_ISR

// Use Interrupts rather than polling.
// TOOD: USE HAL_KEY_INTERRUPT_ENABLE IF PLACED IN hal_key.h?
#define HAL_KEY_MODE                   BSP_KEY_MODE_ISR
//#define HAL_KEY_MODE                   BSP_KEY_MODE_POLL

/* ------------------------------------------------------------------------------------------------
 *                                    Push Button Configuration
 * ------------------------------------------------------------------------------------------------
 */

#define ACTIVE_LOW        !
#define ACTIVE_HIGH       !!    /* double negation forces result to be '1' */

/* Buttons */
#define HAL_BTN_R      BV(4)  /* Button right */
#define HAL_BTN_L      BV(5)  /* Button left */
#define HAL_BTN_U      BV(6)  /* Button up */
#define HAL_BTN_D      BV(7)  /* Button down */
#define HAL_BTN_S      BV(3)  /* Button select */

#define HAL_BTN_R_L_U_D ((HAL_BTN_R) | (HAL_BTN_L) | (HAL_BTN_U) | (HAL_BTN_D))

#define BTN_PORT_R         GPIO_C_BASE
#define BTN_PORT_L         BTN_PORT_R
#define BTN_PORT_U         BTN_PORT_R
#define BTN_PORT_D         BTN_PORT_R
#define BTN_PORT_S         GPIO_A_BASE

#define PUSH_BTN_POLARITY    ACTIVE_LOW

/* ------------------------------------------------------------------------------------------------
 *                         OSAL NV implemented by internal flash pages.
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_NV_PAGE_END           30

// Flash consists of 32 pages of 4 KB.
#define HAL_FLASH_PAGE_SIZE       4096
#define HAL_FLASH_WORD_SIZE       4

// Z-Stack uses flash pages for NV
#define HAL_NV_PAGE_CNT           2
#define HAL_NV_PAGE_BEG           (HAL_NV_PAGE_END-HAL_NV_PAGE_CNT+1)
//TODO changed from (FLASH_BASE)
#define HAL_NV_START_ADDR         ((FLASHMEM_BASE) + (HAL_NV_PAGE_BEG * HAL_FLASH_PAGE_SIZE))


// First half of last page of flash is used for factory commissioning
// Commissioning items are aligned and padded to HAL_FLASH_WORD_SIZE
#define HAL_FLASH_COMM_PAGE  ( 30 )
#define HAL_FLASH_HALF_PAGE  ((HAL_FLASH_PAGE_SIZE) / 2)
#define HAL_FLASH_COMM_ADDR  ((FLASH_BASE) + (HAL_FLASH_COMM_PAGE * HAL_FLASH_PAGE_SIZE) + (HAL_FLASH_HALF_PAGE))

// 8-byte IEEE address (unique for each device)
#define HAL_FLASH_IEEE_SIZE  ( 8 )  // Re-defining Z_EXTADDR_LEN here
#define HAL_FLASH_IEEE_ADDR  (HAL_FLASH_COMM_ADDR - HAL_FLASH_IEEE_SIZE)

// 21-byte Device Private Key (for Key Establishment)
#define HAL_FLASH_DEV_PRIVATE_KEY_SIZE  ( 21 + 3 )  // 3-bytes of pad
#define HAL_FLASH_DEV_PRIVATE_KEY_ADDR  (HAL_FLASH_IEEE_ADDR - HAL_FLASH_DEV_PRIVATE_KEY_SIZE)

// 22-byte CA Public Key (for Key Establishment)
#define HAL_FLASH_CA_PUBLIC_KEY_SIZE  ( 22 + 2 )  // 2-bytes of pad
#define HAL_FLASH_CA_PUBLIC_KEY_ADDR  (HAL_FLASH_DEV_PRIVATE_KEY_ADDR - HAL_FLASH_CA_PUBLIC_KEY_SIZE)

// 48-byte Implicit Certificate (for Key Establishment)
#define HAL_FLASH_IMPLICIT_CERT_SIZE  ( 48 )
#define HAL_FLASH_IMPLICIT_CERT_ADDR  (HAL_FLASH_CA_PUBLIC_KEY_ADDR - HAL_FLASH_IMPLICIT_CERT_SIZE)

/* ------------------------------------------------------------------------------------------------
 *                                            Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ----------- RF-frontend Connection Initialization ---------- */
#if defined HAL_PA_LNA || defined HAL_PA_LNA_CC2590
extern void MAC_RfFrontendSetup(void);
#define HAL_BOARD_RF_FRONTEND_SETUP() MAC_RfFrontendSetup()
#else
#define HAL_BOARD_RF_FRONTEND_SETUP()
#endif

/* ----------- Cache Prefetch control ---------- */
#define FLASH_CTRL_FCTL_PREFETCH_ENABLE                  0x08
#define FLASH_CTRL_FCTL_CACHE_ENABLE                     0x04

#define PREFETCH_ENABLE()     st((HWREG(FLASH_CTRL_FCTL)) = FLASH_CTRL_FCTL_PREFETCH_ENABLE; )
#define PREFETCH_DISABLE()    st((HWREG(FLASH_CTRL_FCTL)) = FLASH_CTRL_FCTL_CACHE_ENABLE; )

#define HAL_CLOCK_INIT()  SysCtrlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | \
                                         SYSCTL_XTAL_16MHZ)

/* ----------- Board Initialization ---------- */
#define HAL_BOARD_INIT()                                                    \
{                                                                           \
  /* Turn on cache prefetch mode */                                         \
  /* Reset all the peripherals used in the mac */                           \
  /* Reset Timer0.                                                          \
   * Timer0A is used as 320us tick.                                         \
   * timer0B used to implement the timer API                                \
   * in mac_mcu_timer.h                                                     \
   */                                                                       \
  /* Reset Timer1(used as a sleep timer) */                                 \
  /* Reset the Push button ports */                                         \
  /* Reset the SPI peripheral */                                            \
  /* Enable the Led port (port f) */                                        \
  /* Configure sleep settings */                                            \
  /* Configure the LEDs as outputs */                                       \
  HAL_TURN_OFF_LED1();                                                      \
  HAL_TURN_OFF_LED2();                                                      \
  HAL_TURN_OFF_LED3();                                                      \
  HAL_TURN_OFF_LED4();                                                      \
}

/* ----------- Debounce ---------- */
#define HAL_DEBOUNCE(expr)    { int i; for (i=0; i<500; i++) { if (!(expr)) i = 0; } }


/* ----------- Push Buttons ---------- */
/*
#define HAL_PUSH_BTN_RIGHT() (PUSH_BTN_POLARITY (GPIOPinRead(BTN_PORT_R, HAL_BTN_R)))
#define HAL_PUSH_BTN_LEFT()  (PUSH_BTN_POLARITY (GPIOPinRead(BTN_PORT_L, HAL_BTN_L)))
#define HAL_PUSH_BTN_SELECT()(PUSH_BTN_POLARITY (GPIOPinRead(BTN_PORT_S, HAL_BTN_S)))
#define HAL_PUSH_BTN_UP()    (PUSH_BTN_POLARITY (GPIOPinRead(BTN_PORT_U, HAL_BTN_U)))
#define HAL_PUSH_BTN_DOWN()  (PUSH_BTN_POLARITY (GPIOPinRead(BTN_PORT_D, HAL_BTN_D)))

#define HAL_PUSH_BUTTON1     HAL_PUSH_BTN_SELECT
*/

#define HAL_PUSH_BUTTON1()   TRUE

/* ----------- LED's ---------- */

/*
#define HAL_TURN_OFF_LED1()       GPIOPinWrite(LED1_PORT, LED1_BV, 0)
#define HAL_TURN_OFF_LED2()       GPIOPinWrite(LED2_PORT, LED2_BV, 0)
#define HAL_TURN_OFF_LED3()       GPIOPinWrite(LED3_PORT, LED3_BV, 0)
#define HAL_TURN_OFF_LED4()       GPIOPinWrite(LED4_PORT, LED4_BV, 0)

#define HAL_TURN_ON_LED1()        GPIOPinWrite(LED1_PORT, LED1_BV, LED1_BV)
#define HAL_TURN_ON_LED2()        GPIOPinWrite(LED2_PORT, LED2_BV, LED2_BV)
#define HAL_TURN_ON_LED3()        GPIOPinWrite(LED3_PORT, LED3_BV, LED3_BV)
#define HAL_TURN_ON_LED4()        GPIOPinWrite(LED4_PORT, LED4_BV, LED4_BV)

#define HAL_TOGGLE_LED1()         GPIOPinWrite(LED1_PORT, LED1_BV, ~(GPIOPinRead(LED1_PORT, LED1_BV)))
#define HAL_TOGGLE_LED2()         GPIOPinWrite(LED2_PORT, LED2_BV, ~(GPIOPinRead(LED2_PORT, LED2_BV)))
#define HAL_TOGGLE_LED3()         GPIOPinWrite(LED3_PORT, LED3_BV, ~(GPIOPinRead(LED3_PORT, LED3_BV)))
#define HAL_TOGGLE_LED4()         GPIOPinWrite(LED4_PORT, LED4_BV, ~(GPIOPinRead(LED4_PORT, LED4_BV)))

#define HAL_STATE_LED1()          GPIOPinRead(LED1_PORT, LED1_BV)
#define HAL_STATE_LED2()          GPIOPinRead(LED2_PORT, LED2_BV)
#define HAL_STATE_LED3()          GPIOPinRead(LED3_PORT, LED3_BV)
#define HAL_STATE_LED4()          GPIOPinRead(LED4_PORT, LED4_BV)
*/

#if defined(HAL_LED) && (HAL_LED==TRUE)
#define HAL_TURN_OFF_LED1()       bspLedClear(BSP_LED_1 )
#define HAL_TURN_OFF_LED2()       bspLedClear(BSP_LED_2 )
#define HAL_TURN_OFF_LED3()       bspLedClear(BSP_LED_3 )
#define HAL_TURN_OFF_LED4()       bspLedClear(BSP_LED_4 )

#define HAL_TURN_ON_LED1()        bspLedSet( BSP_LED_1 )
#define HAL_TURN_ON_LED2()        bspLedSet( BSP_LED_2 )
#define HAL_TURN_ON_LED3()        bspLedSet( BSP_LED_3 )
#define HAL_TURN_ON_LED4()        bspLedSet( BSP_LED_4 )

#else // !HAL_LED

#define HAL_TURN_OFF_LED1()
#define HAL_TURN_OFF_LED2()
#define HAL_TURN_OFF_LED3()
#define HAL_TURN_OFF_LED4()

#define HAL_TURN_ON_LED1()
#define HAL_TURN_ON_LED2()
#define HAL_TURN_ON_LED3()
#define HAL_TURN_ON_LED4()

#endif //

#define HAL_TOGGLE_LED1()
#define HAL_TOGGLE_LED2()
#define HAL_TOGGLE_LED3()
#define HAL_TOGGLE_LED4()

#define HAL_STATE_LED1()          (0)
#define HAL_STATE_LED2()          (0)
#define HAL_STATE_LED3()          (0)
#define HAL_STATE_LED4()          (0)

/* ------------------------------------------------------------------------------------------------
 *                                     Driver Configuration
 * ------------------------------------------------------------------------------------------------
 */

/* Set to TRUE enable H/W TIMER usage, FALSE disable it */
#ifndef HAL_TIMER
#define HAL_TIMER FALSE
#endif

/* Set to TRUE enable ADC usage, FALSE disable it */
#ifndef HAL_ADC
#define HAL_ADC FALSE
#endif

/* Set to TRUE enable LCD usage, FALSE disable it */
#ifndef HAL_LCD
#define HAL_LCD TRUE
#endif

/* Set to TRUE enable LED usage, FALSE disable it */
#ifndef HAL_LED
#define HAL_LED TRUE
#endif
#if (!defined BLINK_LEDS) && (HAL_LED == TRUE)
#define BLINK_LEDS
#endif

/* Set to TRUE enable KEY usage, FALSE disable it */
#ifndef HAL_KEY
#define HAL_KEY FALSE
#endif

/* Set to TRUE enable UART usage, FALSE disable it */
#ifndef HAL_UART
#if (defined ZAPP_P1) || (defined ZAPP_P2) || (defined ZTOOL_P1) || (defined ZTOOL_P2)
#define HAL_UART TRUE
#else
#define HAL_UART TRUE
#endif /* ZAPP, ZTOOL */
#endif /* HAL_UART */

#endif
/*******************************************************************************************************
*/
