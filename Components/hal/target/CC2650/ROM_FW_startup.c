/*******************************************************************************
  Filename:       ROM_FW_startup.c
  Revised:        $Date: 2012-02-15 14:12:20 -0800 (Wed, 15 Feb 2012) $
  Revision:       $Revision: 29309 $

  Description:    This file contains the interrupt vector table and default
                  interrupt service routines for the System firmware for the
                  ARM Cortex M-3. It is the bare minimum boot software
		  need to transfer control from the ROM to the System software
		  in flash memory.

  Copyright 2011-2012 Texas Instruments Incorporated. All rights reserved.

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
*******************************************************************************/

// Enable the IAR extensions for this source file.
#pragma language=extended

/*******************************************************************************
 * INCLUDES
 */

#include "comdef.h"
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_nvic.h>
#include <inc/hw_flash.h>
#include <inc/hw_prcm.h>
#include "hw_aon_sysctrl12.h"
#include <inc/hw_aon_wuc.h>

// TEMP
#define PRCM_O_PERBUSCPUCLK_CG  0x00000034  // PERBUSCPU clock clock gate
#define PRCM_O_PERBUSDMACLK_CG  0x00000038  // PERBUSDMA clock clock gate


/*******************************************************************************
 * EXTERNS
 */

extern void EnterFlash( unsigned long ulAppVectorTableAddr,
                        unsigned long ulRamSize,
                        unsigned long ulDapTapConfig );

/*******************************************************************************
 * PROTOTYPES
 */

static void fwReset_ISR(void);
static void fwNMI_ISR(void);
static void fwHardFault_ISR(void);
static void fwDefault_ISR(void);
static void TurnEverythingOn(void);

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

#define SYS_STACK_SIZE 512
#define SYS_STACK      (intvecHandler)((uint32)fwStack + sizeof(fwStack))

//*****************************************************************************
//
// Definition used for checking SHUTDOWN reset source in the AON_SYSCTRL12
// RESET-register.
//
//*****************************************************************************
#define RST_SRC_SHUTDOWN      0x0000000C

/*******************************************************************************
 * TYPEDEFS
 */

typedef void (*intvecHandler)(void);

/*******************************************************************************
 * LOCAL VARIABLES
 */

#pragma section = ".stack_fw"
static uint32 fwStack[SYS_STACK_SIZE/4] @ ".stack_fw";

/*******************************************************************************
 * GLOBAL VARIABLES
 */

//
// Interrupt Vector Table
//
// NOTE: This table needs to be mapped to physical address 0x00000000 (i.e. at
//       the start of ROM.
//
#pragma section = ".intvec_fw"
__root const intvecHandler __fw_vector_table[] @ ".intvec_fw" =
{
    SYS_STACK,                              //  0: initial stack pointer
    fwReset_ISR,                            //  1: reset handler
    fwNMI_ISR,                              //  2: NMI handler
    fwHardFault_ISR,                        //  3: hard fault handler
    fwDefault_ISR,                          //  4: MPU fault handler
    fwDefault_ISR,                          //  5: bus fault handler
    fwDefault_ISR,                          //  6: usage fault handler
    fwDefault_ISR,                          //  7: Reserved
    fwDefault_ISR,                          //  8: Reserved
    fwDefault_ISR,                          //  9: Reserved
    fwDefault_ISR,                          // 10: Reserved
    fwDefault_ISR,                          // 11: SVCall handler
    fwDefault_ISR,                          // 12: debug monitor handler
    fwDefault_ISR,                          // 13: Reserved
    fwDefault_ISR,                          // 14: PendSV handler
    fwDefault_ISR,                          // 15: SysTick handler
    fwDefault_ISR,                          // 16: AON edge detect
};


/*******************************************************************************
 * @fn          fwReset_ISR
 *
 * @brief       Reset Interrupt Service Routine
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void fwReset_ISR(void)
{
  volatile uint8 i = 1;

  //
  // Make sure we redirect any interrupts to ROM by setting up the vector
  // table offset.
  //
  HWREG(NVIC_VTABLE) = BROM_BASE;

  //***********************************************************************//
  //                                                                       //
  //             Check if pad sleep mode must be disabled.                 //
  //             Pad sleep mode is disabled for all reset                  //
  //             sources except for shutdown.                              //
  //             The RESETSRC-bit field in the RESET                       //
  //             RESET bits[3:1] will have the value bx110                 //
  //             for the SHUTDOWN reset source.                            //
  //                                                                       //
  //***********************************************************************//
  if((HWREG(AON_SYSCTRL12_BASE + AON_SYSCTRL12_O_RESET) &
      AON_SYSCTRL12_RESET_RESETSRC_M) != RST_SRC_SHUTDOWN)
  {
      HWREG(AON_SYSCTRL12_BASE + AON_SYSCTRL12_O_SLEEP) =
        AON_SYSCTRL12_SLEEP_SLEEPDIS;
  }

  // initialize system and flash controller
  TurnEverythingOn();

  // TEMP
  //while(i);

  // redirect interrupt vector table to flash table
  HWREG(NVIC_VTABLE) = FLASHMEM_BASE;

  // transfer control to flash
  EnterFlash( FLASHMEM_BASE,                  // Pointer to flash vector table
              0x03,                           // SRAM size; 3 = 16 KB
              0x7F );                         // 0x7F = Enable all DAP/TAPs

  return;
}


//*****************************************************************************
//
//! \internal
//! Turn everything on for pre PG ROM images.
//!
//! \return None.
//
//*****************************************************************************
static void TurnEverythingOn(void)
{
  //
  // Enable all peripherals on die
  //
  HWREG(PRCM_BASE + PRCM_O_CONFIG_SEC_DMA) = 0x00000103;
  HWREG(PRCM_BASE + PRCM_O_CONFIG_GPIO) = 0x00000001;
  HWREG(PRCM_BASE + PRCM_O_CONFIG_GPTM) = 0x0000000F;
  HWREG(PRCM_BASE + PRCM_O_CONFIG_I2C) = 0x00000001;
  HWREG(PRCM_BASE + PRCM_O_CONFIG_UART) = 0x00000001;
  HWREG(PRCM_BASE + PRCM_O_CONFIG_SSI) = 0x00000003;
  HWREG(PRCM_BASE + PRCM_O_CONFIG_I2S) = 0x00000001;

  //
  // Make sure all changes take effect
  //
  HWREG(PRCM_BASE + PRCM_O_CLKCTRL) = 0x1;

  //
  // Enable all clock gating
  //
  HWREG(PRCM_BASE + PRCM_O_RFCORECLK_CG) = 0x00000001;
  HWREG(PRCM_BASE + PRCM_O_FLASHCLK_CG) = 0x00000003;
  HWREG(PRCM_BASE + PRCM_O_SEC_DMA_RCG) = 0x00000103;
  HWREG(PRCM_BASE + PRCM_O_SEC_DMA_SCG) = 0x00000103;
  HWREG(PRCM_BASE + PRCM_O_SEC_DMA_DSCG) = 0x00000103;
  HWREG(PRCM_BASE + PRCM_O_GPIO_RCG) = 0x00000001;
  HWREG(PRCM_BASE + PRCM_O_GPIO_SCG) = 0x00000001;
  HWREG(PRCM_BASE + PRCM_O_GPIO_DSCG)= 0x00000001;
  HWREG(PRCM_BASE + PRCM_O_GPTM_RCG) = 0x0000000F;
  HWREG(PRCM_BASE + PRCM_O_GPTM_SCG) = 0x0000000F;
  HWREG(PRCM_BASE + PRCM_O_GPTM_DSCG) = 0x0000000F;
  HWREG(PRCM_BASE + PRCM_O_I2C_RCG) = 0x00000001;
  HWREG(PRCM_BASE + PRCM_O_I2C_SCG) = 0x00000001;
  HWREG(PRCM_BASE + PRCM_O_I2C_DSCG) = 0x00000001;
  HWREG(PRCM_BASE + PRCM_O_UART_RCG) = 0x00000001;
  HWREG(PRCM_BASE + PRCM_O_UART_SCG) = 0x00000001;
  HWREG(PRCM_BASE + PRCM_O_UART_DSCG) = 0x00000001;
  HWREG(PRCM_BASE + PRCM_O_SSI_RCG) = 0x00000003;
  HWREG(PRCM_BASE + PRCM_O_SSI_SCG) = 0x00000003;
  HWREG(PRCM_BASE + PRCM_O_SSI_DSCG) = 0x00000003;
  HWREG(PRCM_BASE + PRCM_O_I2S_RCG) = 0x00000001;

  //
  // Turn on the power domains
  //
  HWREG(PRCM_BASE + PRCM_O_SWPWR_CTRL) = 0x0000000F;
  HWREG(PRCM_BASE + PRCM_O_MNGPWR_CTRL) = 0x0000001F;

  //
  // Force power and clock on just to be sure
  //
  HWREG(PRCM_BASE + PRCM_O_CLOCK_OVR) = 0x000FFFFF;
  HWREG(PRCM_BASE + PRCM_O_PD_FORCE_ON) = 0x0000000F;

  //
  // Make sure all changes take effect
  //
  HWREG(PRCM_BASE + PRCM_O_CLKCTRL) = 0x1;

  //
  // Signal that flash module now has a calibrated clock.
  //
  HWREG(AON_WUC_BASE + AON_WUC_O_MCUCLK) |= AON_WUC_MCUCLK_CALIBRATED;

  HWREG(FLASH_BASE + FLASH_O_WAITSTATES)         = 3;
  HWREG(FLASH_BASE + FLASH_O_SAMPLE_PERIOD)      = 0x000003E7;
  HWREG(FLASH_BASE + FLASH_O_PRE_SAMPLE)         = 0x0000002F;
  //    __asm("InsertLabelOne: \n");
  HWREG(FLASH_BASE + FLASH_O_SAMHOLD_SA)         = 0x0000002F;
  HWREG(FLASH_BASE + FLASH_O_SAMHOLD_SU)         = 0x0000002F;
  HWREG(FLASH_BASE + FLASH_O_BANK_0_EFUSE)       = 0x000906C5;
  HWREG(FLASH_BASE + FLASH_O_WAIT_SYSCODE)       = 0x00000003;
  //    __asm("InsertLabelTwo: \n");

  HWREG(FLASH_BASE + FLASH_O_FRDCNTL)            = 0x00000200;
  HWREG(FLASH_BASE + FLASH_O_FSPRD)              = 0x00000000;
  HWREG(FLASH_BASE + FLASH_O_FEDACCTRL1)         = 0x00000000;

  HWREG(FLASH_BASE + FLASH_O_FSM_WR_ENA)         = 5;
  HWREG(FLASH_BASE + FLASH_O_FSM_GLBCTRL) = 1;
  HWREG(FLASH_BASE + FLASH_O_FSM_PE_OSU)         = 0x00001616;
  HWREG(FLASH_BASE + FLASH_O_FSM_VSTAT)          = 0x00003000;
  HWREG(FLASH_BASE + FLASH_O_FSM_PE_VSU)         = 0x00001919;
  HWREG(FLASH_BASE + FLASH_O_FSM_CMP_VSU)        = 0x00002000;
  HWREG(FLASH_BASE + FLASH_O_FSM_EX_VAL)         = 0x00000216;
  HWREG(FLASH_BASE + FLASH_O_FSM_RD_H)           = 0x0000005A;
  HWREG(FLASH_BASE + FLASH_O_FSM_P_OH)           = 0x00002300;
  HWREG(FLASH_BASE + FLASH_O_FSM_ERA_OH)         = 0x00000160;
  HWREG(FLASH_BASE + FLASH_O_FSM_PE_VH)          = 0x00000300;
  HWREG(FLASH_BASE + FLASH_O_FSM_PRG_PW)         = 0x0000002D;
  HWREG(FLASH_BASE + FLASH_O_FSM_ERA_PW)         = 0x00005D01;
  HWREG(FLASH_BASE + FLASH_O_FSM_PRG_PUL)        = 0x00040006;
  HWREG(FLASH_BASE + FLASH_O_FSM_ERA_PUL)        = 0x00040007;
  HWREG(FLASH_BASE + FLASH_O_FSM_STEP_SIZE)      = 0x00000000;
  HWREG(FLASH_BASE + FLASH_O_FSM_EC_STEP_HEIGHT) = 0x00000000;
  HWREG(FLASH_BASE + FLASH_O_FSM_WR_ENA)         = 2;

  HWREG(FLASH_BASE + FLASH_O_FLOCK)              = 0x0000aaaa;
  HWREG(FLASH_BASE + FLASH_O_FVHVCT1)            = 0x00840080;
  HWREG(FLASH_BASE + FLASH_O_FVHVCT2)            = 0x00a20000;
  HWREG(FLASH_BASE + FLASH_O_FVSLP)              = 0x00008000;
  HWREG(FLASH_BASE + FLASH_O_FLOCK)              = 0x000055aa;

  // Set no of sectors
  HWREG(FLASH_BASE + FLASH_O_FLASH_SIZE)         = 32;
  // stay in active mode
  HWREG(FLASH_BASE + FLASH_O_FBFALLBACK) = 3; // bank power active
  HWREG(FLASH_BASE + FLASH_O_FBAC1) = 1 | HWREG(FLASH_BASE + FLASH_O_FBAC1); // set pumppwr active

  // stay in active mode
  HWREG(FLASH_BASE + FLASH_O_FBFALLBACK) = 3; // bank power active
  HWREG(FLASH_BASE + FLASH_O_FBAC1) = 1 | HWREG(FLASH_BASE + FLASH_O_FBAC1); // set pumppwr active

  return;
}


/*******************************************************************************
 * @fn          fwNMI_ISR
 *
 * @brief       Non-Maskable Interrupt Interrupt Service Routine
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
static void fwNMI_ISR(void)
{
  volatile uint8 i = 1;

  // Enter an infinite loop.
  while(i);

  return;
}

/*******************************************************************************
 * @fn          fwHardFault_ISR
 *
 * @brief       Hard Fault Interrupt Service Routine
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
static void fwHardFault_ISR(void)
{
  volatile uint8 i = 1;

  // Enter an infinite loop.
  while(i);

  return;
}

/*******************************************************************************
 * @fn          fwDefault_ISR
 *
 * @brief       MPU Fault Interrupt Service Routine
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
static void fwDefault_ISR(void)
{
  volatile uint8 i = 1;

  // Enter an infinite loop.
  while(i);

  return;
}

/*******************************************************************************
*/
