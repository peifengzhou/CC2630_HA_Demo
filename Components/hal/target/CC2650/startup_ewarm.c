/*******************************************************************************
  Filename:       startup_ewarm.c
  Revised:        $Date: 2012-02-15 14:12:20 -0800 (Wed, 15 Feb 2012) $
  Revision:       $Revision: 29309 $

  Description:    This file contains the interrupt vector table and default
                  interrupt service routines for the System Software for the
                  ARM Cortex M-3.

  Copyright 2011-2014 Texas Instruments Incorporated. All rights reserved.

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

// We need intrinsic functions for IAR (if used in source code)
#ifdef __IAR_SYSTEMS_ICC__
#include <intrinsics.h>
#endif

/*******************************************************************************
 * INCLUDES
 */

#include "comdef.h"
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_nvic.h>
#include "hal_sleep.h"
#include "hal_assert.h"

/*******************************************************************************
 * EXTERNS
 */

// code
extern void UART0_ISR( void );
extern void mbCmdAckIsr( void );
extern void mbCpe0Isr( void );
extern void mbCpe1Isr( void );
extern void mbHwIsr( void );
extern void SysTickIntHandler( void );
extern void RTC_ISR( void );
//
extern void __iar_program_start(void);
extern uint8 FPB_ProcessSVC( uint32 * );

// data
extern uint32 FPB_AddrTable[];

/*******************************************************************************
 * PROTOTYPES
 */

static void Reset_ISR(void);
static void NMI_ISR(void);
static void HardFault_ISR(void);
static void MPUFault_ISR(void);
static void BusFault_ISR(void);
static void UsageFault_ISR(void);
static void SVC_ISR(void);
static void Default_ISR(void);

#if !defined( PATCHED_CODE_WORD_ALIGNED )
static void Default_SVC_Handler(void);
#endif // !PATCHED_CODE_WORD_ALIGNED

// TEMP?
#include <inc/hw_prcm.h>
#include <driverlib/ioc.h>
#include <driverlib/prcm.h>
#include <driverlib/interrupt.h>
#include <inc/hw_ints.h>
#include <driverlib/gpio.h>

#ifdef BACKDOOR_TRAP
#include "backdoor.h"
void BackdoorInit(uint32_t ui32Pin, uint32_t ui32TrigLevel, uint32_t ui32Mode);
void BackdoorOpen(void);
void BackdoorCheck(void);
#endif // BACKDOOR_TRAP

/*******************************************************************************
 * MACROS
 */

#ifdef BACKDOOR_TRAP
#define BACKDOOR_CHECK()  BackdoorCheck();
#else
#define BACKDOOR_CHECK()
#endif // BACKDOOR_TRAP

/*******************************************************************************
 * CONSTANTS
 */

#define SYS_STACK_SIZE 512
#define SYS_STACK      (intvecHandler)((uint32)sysStack + sizeof(sysStack))

// SBC: System Handler Control and State Register
#define SHCSR_REG      HWREG(NVIC_SYS_HND_CTRL)
#define CCR_REG        HWREG(NVIC_CFG_CTRL)

// Defines for Customer Configuration Area
// Note: Do not remove!
//#define CCA_BDADDR_0            0xCCDDEEFF
//#define CCA_BDADDR_1            0xFFFFAABB
#define CCA_BOOTLOADER_CONFIG   0xC5FFFFFF
#define CCA_ERASE_CONFIG        0xFFFFFFFF
#define CCA_BACKDOOR_CONFIG     0xFFFFFFC5
#define CCA_TAPDAP_CONFIG_0     0xFFC5C5C5
#define CCA_TAPDAP_CONFIG_1     0xFFC5C5C5
#define CCA_FLASH_IMAGE_VALID   0x00000000
#define CCA_SECTOR_PROT_0       0xFFFFFFFF
#define CCA_SECTOR_PROT_1       0xFFFFFFFF
#define CCA_SECTOR_PROT_2       0xFFFFFFFF
#define CCA_SECTOR_PROT_3       0xFFFFFFFF

/*******************************************************************************
 * TYPEDEFS
 */

// interrupt vector
typedef void (*intvecHandler)(void);

// Type for Customer Configuration Area
// Note: Do not remove!
// Note: If the size of this table changes, the linker configuratin file must
//       be updated!
// TODO: PASS SIZE OF THIS TABLE AS LINKER CONFIG FILE PARAMETER?
typedef struct
{
    //uint32 ulBDADDR0;
    //uint32 ulBDADDR1;
    uint32 ulBlConfig;
    uint32 ulEraseConfig;
    uint32 ulTiBackdoor;
    uint32 ulTapDap0;
    uint32 ulTapDap1;
    uint32 ulImageValid;
    uint32 ulSectorProt0;
    uint32 ulSectorProt1;
    uint32 ulSectorProt2;
    uint32 ulSectorProt3;
} ccaFlashPage_t;

#ifdef BACKDOOR_TRAP
typedef struct tBackdoor {
    uint32_t ui32Pin;
    uint32_t ui32Level;
} tBackdoor;
#endif // BACKDOOR_TRAP

/*******************************************************************************
 * LOCAL VARIABLES
 */

// TODO: WOULD LIKE THE STACK SIZE TO BE DEFINED BY THE LINKER CONFIGURATION
//       FILE; PROBLELM IS, YOU CAN'T DEFINE AN ARRAY WITH A VARIABLE.

// System Stack
static uint32 sysStack[SYS_STACK_SIZE/4] @ ".stack";

/*******************************************************************************
 * GLOBAL VARIABLES
 */

#if !defined( PATCHED_CODE_WORD_ALIGNED )
// Address of SVC handler (used to load C function address in inline asm).
const uint32 SVC_Handler = (uint32)FPB_ProcessSVC;

// Address of Default SVC handler.
const uint32 SVC_DefaultHandler = (uint32)Default_SVC_Handler;
#endif // !PATCHED_CODE_WORD_ALIGNED

// Customer Configuration Area in Lock Page
// Note: Do not remove!
#ifdef __IAR_SYSTEMS_ICC__
__root const ccaFlashPage_t __cca @ ".flashcca" =
#else
const ccaFlashPage_t __cca __attribute__((section("flashcca")))=
#endif
{
    //CCA_BDADDR_0,
    //CCA_BDADDR_1,
    CCA_BOOTLOADER_CONFIG,
    CCA_ERASE_CONFIG,
    CCA_BACKDOOR_CONFIG,
    CCA_TAPDAP_CONFIG_0,
    CCA_TAPDAP_CONFIG_1,
    CCA_FLASH_IMAGE_VALID,
    CCA_SECTOR_PROT_0,
    CCA_SECTOR_PROT_1,
    CCA_SECTOR_PROT_2,
    CCA_SECTOR_PROT_3,
};

//
// Interrupt Vector Table
//
// NOTE: This table needs to be mapped to physical address 0x00000000 (i.e. at
//       the start of ROM.
//
__root const intvecHandler __vector_table[] @ ".intvec" =
{
    SYS_STACK,                              //  0: Stack Pointer
    Reset_ISR,                              //  1: Reset
    NMI_ISR,                                //  2: NMI
    HardFault_ISR,                          //  3: Hard Fault
    MPUFault_ISR,                           //  4: MPU Fault
    BusFault_ISR,                           //  5: Bus Fault
    UsageFault_ISR,                         //  6: Usage Fault
    0,                                      //  7: Reserved
    0,                                      //  8: Reserved
    0,                                      //  9: Reserved
    0,                                      // 10: Reserved
    SVC_ISR,                                // 11: SVCall handler
    Default_ISR,                            // 12: Debug Monitor
    0,                                      // 13: Reserved
    Default_ISR,                            // 14: PendSV handler
    SysTickIntHandler,                      // 15: SysTick handler
    //
    Default_ISR,                            // 16: AON Edge Detect
    Default_ISR,                            // 17: I2C
    mbCpe1Isr,                              // 18: RF Core CPE 1
    Default_ISR,                            // 19: AON SPIS Tx, Rx, and CS
    RTC_ISR,                                // 20: AON RTC
    UART0_ISR,                              // 21: UART0 Rx and Tx
    Default_ISR,                            // 22: UART1 Rx and Tx
    Default_ISR,                            // 23: SSI0 Rx and Tx
    Default_ISR,                            // 24: SSI1 Rx and Tx
    mbCpe0Isr,                              // 25: RF Core CPE 0
    mbHwIsr,                                // 26: RF Core HW
    mbCmdAckIsr,                            // 27: RF Core Command ACK
    Default_ISR,                            // 28: I2S
    Default_ISR,                            // 29: Unassigned
    Default_ISR,                            // 30: Watchdog Timer
    Default_ISR,                            // 31: Timer 0 subtimer A
    Default_ISR,                            // 32: Timer 0 subtimer B
    Default_ISR,                            // 33: Timer 1 subtimer A
    Default_ISR,                            // 34: Timer 1 subtimer B
    Default_ISR,                            // 35; Timer 2 subtimer A
    Default_ISR,                            // 36: Timer 2 subtimer B
    Default_ISR,                            // 37: Timer 3 subtimer A
    Default_ISR,                            // 38: Timer 3 subtimer B
    Default_ISR,                            // 39: Crypto Core Result Available
    Default_ISR, //uDMAIntHandler,          // 40: DMA Software
    Default_ISR,                            // 41: DMA Error
    Default_ISR,                            // 42: Flash Controller
    Default_ISR, //uDMAIntHandler,          // 43: Software Event 0
    Default_ISR,                            // 44: MCU Event Combined
    Default_ISR,                            // 45: AON Programmable 0
    Default_ISR,                            // 46: Power Reset Clock Management (PRCM)
    Default_ISR,                            // 47: AUX Compare
    Default_ISR,                            // 48: AUX ADC IRQ
    Default_ISR,                            // 49: TRNG Event
};

#ifdef BACKDOOR_TRAP
static tBackdoor g_tBackdoor;
bool g_bBackdoorActivated;
#endif // BACKDOOR_TRAP

/*
** Basic Interrupt Service Routines
*/

/*******************************************************************************
 * @fn          Reset_ISR
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
static void Reset_ISR( void )
{
  volatile uint8 i = 1;

  // redirect interrupt vector table to flash table
  // TEMP: This is a temporary fix. For some reason, the VTOR doesn't get set
  //       when the debugger attaches.
  HWREG(NVIC_VTABLE) = FLASHMEM_BASE;

  // enable usage, bus, and memory manage fault as separate faults
  // Note: If memory mange is MPU, then not relevant.
  SHCSR_REG |= ( NVIC_SYS_HND_CTRL_USAGE |
                 NVIC_SYS_HND_CTRL_BUS   |
                 NVIC_SYS_HND_CTRL_MEM );

  // use 4 byte stack alignment to simplify using SVC
  // Note: Eight bit alignment may add an extra word, which could throw off
  //       where things are located (like the return address) in the stack.
  // Note: Eight bit alignment is necessary for ARM Binary Interface (ABI), and
  //       is supported in hardware after revision 1 (we're revision 2).
  //CCR_REG &= ~NVIC_CFG_CTRL_STKALIGN;

  // make sure unaligned word or halfword accesses are trapped
  CCR_REG |= NVIC_CFG_CTRL_UNALIGNED;

#ifdef POWER_SAVING
  // check if we are booting cold (i.e. out of a hardware reset)
  if ( halBootState != HAL_BOOT_WARM )
  {
    // check if backdoor trap is being used
    BACKDOOR_CHECK();

    // software entry point
    __iar_program_start();
  }
  else // warm boot
  {
    // wake from Power Down Mode via PRCMDeepSleep
    halWarmBoot();;
  }

#else // !POWER_SAVING

  // check if backdoor trap is being used
  BACKDOOR_CHECK();

  // software entry point
  __iar_program_start();
#endif // POWER_SAVING

  return;
}


/*******************************************************************************
 * @fn          NMI_ISR
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
static void NMI_ISR( void )
{
  volatile uint8 i = 1;

  // Enter an infinite loop.
  while(i);

  return;
}

/*******************************************************************************
 * @fn          HardFault_ISR
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
static void HardFault_ISR( void )
{
  volatile uint8 i = 1;

  // Enter an infinite loop.
  while(i);

  return;
}

/*******************************************************************************
 * @fn          MPUFault_ISR
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
static void MPUFault_ISR( void )
{
  volatile uint8 i = 1;

  // Enter an infinite loop.
  while(i);

  return;
}

/*******************************************************************************
 * @fn          BusFault_ISR
 *
 * @brief       Bus Fault Interrupt Service Routine
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
static void BusFault_ISR( void )
{
  volatile uint8 i = 1;

  // Enter an infinite loop.
  while(i);

  return;
}

/*******************************************************************************
 * @fn          UsageFault_ISR
 *
 * @brief       Usage Fault Interrupt Service Routine
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
static void UsageFault_ISR( void )
{
  volatile uint8 i = 1;

  // Enter an infinite loop.
  while(i);

  return;
}

/*******************************************************************************
 * @fn          SVC_ISR
 *
 * @brief       Supervision Call Interrupt Service Routine
 *
 *              This routine first determines whether the main or process stack
 *              is to be used. It then saves the value of EXE_RETURN and the
 *              SVC stack pointer and calls the SVC_Handler, which returns the
 *              SVC number. If the SVC number is not 0-7 (the reserved SVC
 *              numbers for FPB), then control is passed to a default handler.
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
static void SVC_ISR( void )
{
#if defined( FLASH_ONLY_BUILD )
  return;
#else
#if !defined( PATCHED_CODE_WORD_ALIGNED )
  // determine which stack to use based on CONTROL.SPSEL bit in LR
  // zero: load the main stack pointer
  // one:  load the process stack pointer
  asm("TST LR, #4");
  asm("ITE EQ");
  asm("MRSEQ R0, MSP");
  asm("MRSNE R0, PSP");

  // save EXC_RETURN and SVC stack pointer
  asm("PUSH {LR, R0}");

  // get address of SVCall Handler in C code
  asm("LDR R1, SVC_Handler");

  // call handler to find SVC offset and setup call to patch in stack
  // Note: SVC Number returned in R0.
  asm("BLX R1");

  // check if SVC Number is not FPB related (always restore the SVC stack
  // pointer), and if so, load the generic SVC handler, and call handler
  asm("CMP R0, #8");
  asm("POP {R0}");
  asm("ITT GE");
  asm("LDRGE R1, SVC_DefaultHandler");
  asm("BLXGE R1");

  // restore EXC_RETURN
  asm("POP {LR}");

  // return from exception
  // Note: If the SVC number was between 0 and 7, and there was a patched
  //       routine, then the exception stack was modified such that the return
  //       from this handler will transfer control to the patched routine. If
  //       there was no match, then return from this handler will transfer
  //       control to the routine that called the patch routine. If the SVC
  //       number was between 8 and 255, then control was passed to the default
  //       SVC handler, and return from this handler will be to the instruction
  //       after the SVC.
  return;

#else // PATCHED_CODE_WORD_ALIGNED
  // return from exception
  return;

#endif // !PATCHED_CODE_WORD_ALIGNED
#endif // !FLASH_ONLY_BUILD
}

/*******************************************************************************
 * @fn          Default_ISR
 *
 * @brief       Default Interrrupt Service Routine
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
static void Default_ISR( void )
{
  volatile uint8 i = 1;

  // Enter an infinite loop.
  while(i);

  return;
}

#if !defined( PATCHED_CODE_WORD_ALIGNED )
/*******************************************************************************
 * @fn          Default_SVC_Handler
 *
 * @brief       Default Supervision Call Handler
 *
 *              This routine is used to handle SVC processor for numbers other
 *              than 0-7, which are reserved for FPB.
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
static void Default_SVC_Handler( void )
{
  return;
}
#endif // PATCHED_CODE_WORD_ALIGNED


#ifdef BACKDOOR_TRAP
/*******************************************************************************
 * @fn          BackdoorCheck
 *
 * @brief       Used to trap the device in a spin when a button is pressed out
 *              of reset. Can be used to get control of the device in the event
 *              any flash issue occurs that prevents proper boot or a debugger
 *              connection.
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
void BackdoorCheck( void )
{
  // Turn on peripheral domain
  HWREG( PRCM_BASE + PRCM_O_SWPWR_CTRL ) = PRCM_SWPWR_CTRL_PERIPH;
  while (( HWREG( PRCM_BASE + PRCM_O_SWPWR_STATUS ) & PRCM_SWPWR_CTRL_PERIPH ) != PRCM_SWPWR_CTRL_PERIPH ) { }

  BackdoorInit( (1<<IOID_11), 0, BACKDOOR_POLLED );

  BackdoorOpen();

  return;
}


//*****************************************************************************
//
//! Initializes a GPIO to function as backdoor.
//!
//! \param ui32Pin is the pin to setup as backdoor
//! \param iu32TrigLevel is the pin value that triggers the backdoor.
//! \param iu32Mode is the pin value that triggers the backdoor.
//!
//! This function will initialize the specified GPIO to function as a backdoor.
//! The backdoor should be initialized and checked as the first thing the
//! program does.
//!
//! The 'backdoor' can run in polled or interrupt mode. If using polled mode,
//! the program should query the backdoor before executing dobious code that
//! might lock up the chip. If using interrupt mode, the backdoor can be
//! triggered at any point in the program (before the chip locks...).
//! The two modes are selected using
//! - \b BACKDOOR_POLLED
//! - \b BACKDOOR_INTERRUPT
//!
//! The 'backdoor' is basically and endless loop, that conserves the chip in the
//! state it was in when the backdoor was triggered, allowing the debugger to
//! access the device and erase faulty code.
//!
//! It is assumed that the caller has previously turned on the peripheral power
//! domain such that the GPIO module is powered.
//!
//! \return None.
//
//*****************************************************************************
void
BackdoorInit(uint32_t ui32Pin, uint32_t ui32TrigLevel, uint32_t ui32Mode)
{
    uint32_t ui32IoConfig;
    uint32_t ui32IoId;

    //
    // Enable the GPIO peripheral module in run mode
    //
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
    PRCMPeripheralSleepEnable(PRCM_PERIPH_GPIO);
    PRCMPeripheralDeepSleepEnable(PRCM_PERIPH_GPIO);
    PRCMLoadSet();
    while(!PRCMLoadGet())
    { }

    //
    // Convert the pin number go an IO Id
    //
#ifdef __IAR_SYSTEMS_ICC__
    ui32IoId = 31 - __CLZ(ui32Pin);
#else
    ui32IoId = 0;
    while(!(ui32Pin & (1 << ui32IoId)))
    {
    	ui32IoId++;
    }
#endif

    //
    // Disable and clear any pending interrupts
    //
    IOCIntDisable(ui32IoId);
    IOCIntClear(ui32IoId);

    //
    // Configure the IO...
    //
    ui32IoConfig = IOC_STD_INPUT & ~(IOC_NO_IOPULL | IOC_BOTH_EDGES);

    //
    // Check if using interrupt mode
    //
    if(ui32Mode == BACKDOOR_INTERRUPT)
    {
        g_bBackdoorActivated = false;
        ui32IoConfig |= IOC_INT_ENABLE;
        if(ui32TrigLevel == 1)
        {
            ui32IoConfig |= (IOC_RISING_EDGE | IOC_IOPULL_DOWN);
        }
        else
        {
            ui32IoConfig |= (IOC_FALLING_EDGE | IOC_IOPULL_UP);
        }

        //
        // Enable global interrupts and edge interrupt
        //
        IntEnable(INT_EDGE_DETECT);
        IntMasterEnable();
    }
    else
    {
        if(ui32TrigLevel == 1)
        {
            ui32IoConfig |= IOC_IOPULL_DOWN;
        }
        else
        {
            ui32IoConfig |= IOC_IOPULL_UP;
        }
    }

    //
    // Write the configuration
    //
    GPIODirModeSet(ui32Pin, GPIO_DIR_MODE_IN);
    IOCPortConfigureSet(ui32IoId, IOC_PORT_GPIO, ui32IoConfig);

    //
    // Update the struct
    //
    g_tBackdoor.ui32Level = ui32TrigLevel;
    g_tBackdoor.ui32Pin = ui32Pin;
}


//*****************************************************************************
//
//! Query the state of the backdoor
//!
//! If the backdoor is open, this function will never return.
//!
//! \return None
//
//*****************************************************************************
void BackdoorOpen(void)
{
    //
    // Check backdoor
    //
    if((GPIOPinRead(g_tBackdoor.ui32Pin) ? 1 : 0) == g_tBackdoor.ui32Level)
    {
      halBootState = HAL_BOOT_COLD;
      while(1);
    }

    PRCMPeripheralRunDisable(PRCM_PERIPH_GPIO);
    PRCMPeripheralSleepDisable(PRCM_PERIPH_GPIO);
    PRCMPeripheralDeepSleepDisable(PRCM_PERIPH_GPIO);
    PRCMLoadSet();
    // turn off periperal domain
    HWREG( PRCM_BASE + PRCM_O_SWPWR_CTRL ) = 0;
}
#endif // BACKDOOR_TRAP

/*******************************************************************************
*/
