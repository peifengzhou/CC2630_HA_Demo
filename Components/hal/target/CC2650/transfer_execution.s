//-----------------------------------------------------------------------------
// TI Confidential – NDA Restrictions
//
// Copyright (c) 2012 Texas Instruments, Inc.
//
//    This is an unpublished work created in the year stated above.
//    Texas Instruments owns all rights in and to this work and
//    intends to maintain and protect it as an unpublished copyright.
//    In the event of either inadvertent or deliberate publication,
//    the above stated date shall be treated as the year of first
//    publication. In the event of such publication, Texas Instruments
//    intends to enforce its rights in the work under the copyright
//    laws as a published work.
//
//
//  Content        : Code for tansferring code execution from ROM to FLASH
//  Created By     : Low Power RF Wireless Business Unit
//
//-----------------------------------------------------------------------------
//  File           : $Id: $
//-----------------------------------------------------------------------------

#include "asmdefs.h"
#include "hw_memmap.h"
#include "hw_prcm.h"
#include "hw_aon_wuc.h"
#include "hw_aon_rtc.h"

//*****************************************************************************
//
// This portion of the file goes into the text section.
//
//*****************************************************************************
    __TEXT__

//*****************************************************************************
//
//! Transferres code execution to flash.
//!
//! \param r0 is the start address of the vector table in flash
//! \param r1 is the size of the SRAM to be configured for the device
//! \param r2 is the DAP/TAPs configuration to be used for the device
//!
//! This function will update the stack pointer (SP) with the initial stack
//! pointer value read from the flash vector table pointed to by r0.
//! The the SRAM size is configured to he size given by r1 followed by a lock
//! of the PRCM configuration. The the DAP/TAPs configuration given by r2 is
//! set and the security system is locked.
//! Finaly the a branch is done to the reset ISR of the flash image. This
//! address is found in the flash vector table pointed to by r0.
//!
//! This function is writen in assembly in order to asure that stack is not
//! used after switching SRAM size.
//!
//! \return None
//
//*****************************************************************************
    __EXPORT__ EnterFlash
    __THUMB_LABEL__
EnterFlash __LABEL__
   //
   // Set initial stack pointer value for flash image
   //
   ldr    sp, [r0, #0x0]

   //
   // Set SRAM size
   //
   movw    r3, #((PRCM_BASE + PRCM_O_CONFIG_RAMSIZE) & 0xffff)
   movt    r3, #((PRCM_BASE + PRCM_O_CONFIG_RAMSIZE) >> 16)
   str     r1,[r3, #0]

   //
   // Since the two bits in the RAMSIZE register are not syncronous they are
   // not latched before a load of setting to CLKCTRL is done
   //
   movw    r3, #((PRCM_BASE + PRCM_O_CLKCTRL) & 0xffff)
   movt    r3, #((PRCM_BASE + PRCM_O_CLKCTRL) >> 16)
   mov     r1, #PRCM_CLKCTRL_LOAD
   str     r1,[r3, #0]

   //
   // Set the DAP/TAPs configuration with read-modify-wrt operation
   //
   movw    r3, #((AON_WUC_BASE + AON_WUC_O_JTAGCFG) & 0xffff)
   movt    r3, #((AON_WUC_BASE + AON_WUC_O_JTAGCFG) >> 16)
   ldr     r1, [r3, #0]
   orr     r4, r1, r2
   str     r4,[r3, #0]

   //
   // Asure that there are no outstanding write requests between AONIF and AON
   //
   movw    r3, #((AON_RTC_BASE + AON_RTC_O_SYNC) & 0xffff)
   movt    r3, #((AON_RTC_BASE + AON_RTC_O_SYNC) >> 16)
   ldr     r2, [r3, #0]

   //
   // Lock the DAP/TAPs configuration with read-modify-wrt operation
   //
   movw    r3, #((AON_WUC_BASE + AON_WUC_O_LOCKCFG) & 0xffff)
   movt    r3, #((AON_WUC_BASE + AON_WUC_O_LOCKCFG) >> 16)
   ldr     r2, [r3, #0]
   orr     r1, r2, #AON_WUC_LOCKCFG_LOCK
   str     r1,[r3, #0]

   //
   // Signal to ICEPick that security settings is done
   //
   movw    r3, #((AON_WUC_BASE + AON_WUC_O_LOCKCFG) & 0xffff)
   movt    r3, #((AON_WUC_BASE + AON_WUC_O_LOCKCFG) >> 16)
   ldr     r2, [r3, #0]
   orr     r1, r2, #AON_WUC_LOCKCFG_SECURITYINITIALIZED
   str     r1,[r3, #0]

   //
   // Lock PRCM configuration and signal to AON that boot is done
   //
   movw    r3, #((PRCM_BASE + PRCM_O_CONFIG_PRCM) & 0xffff)
   movt    r3, #((PRCM_BASE + PRCM_O_CONFIG_PRCM) >> 16)
   movs    r1, #PRCM_CONFIG_PRCM_LOCK
   str     r1,[r3, #0]

   //
   // Set the LR register to a value causing a processor fault in case of
   // an accidental return from the application image in flash
   //
   movw    r2, #(0xffffffff & 0xffff)
   movt    r2, #(0xffffffff >> 16)
   mov     lr, r2

   //
   // Branch to application reset ISR of the flash image
   //
   ldr     r1, [r0, #4]
   bx      r1

//*****************************************************************************
//
// This is the end of the file.
//
//*****************************************************************************
    __END__
