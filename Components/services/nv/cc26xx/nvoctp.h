/**
  @file nvoctp.h
  $Date: 2015-02-15 18:34:56 -0800 (Sun, 15 Feb 2015) $
  $Revision: 42615 $

  @brief NV definitions for CC26xx devices - On-Chip Two-Page Flash Memory

  <!--
  Copyright 2014-2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
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
  -->
*/
#ifndef NVOCTP_H
#define NVOCTP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "nvintf.h"

//*****************************************************************************
// Constants and definitions
//*****************************************************************************

// NV driver item ID definitions
#define NVOCTP_NVID_DIAG {NVINTF_SYSID_NVDRVR, 1, 0}

//*****************************************************************************
// Typedefs
//*****************************************************************************

// NV driver diagnostic data
typedef struct
{
    uint32_t compacts;  // Number of page compactions
    uint16_t resets;    // Number of driver resets (power on)
    uint16_t available; // Number of available bytes after last compaction
    uint16_t active;    // Number of active items after last compaction
    uint16_t reserved;  // Number of reserved items after last compaction
}
NVOCTP_diag_t;

//*****************************************************************************
// Functions
//*****************************************************************************

extern void NVOCTP_loadApiPtrs(NVINTF_nvFuncts_t *pfnNV);

// Exception function can be defined to handle NV corruption issues
// If none provided, NV module attempts to proceeed ignoring problem
#if !defined (NVOCTP_EXCEPTION)
#define NVOCTP_EXCEPTION(pg, err)
#endif


//*****************************************************************************
//*****************************************************************************

#ifdef __cplusplus
}
#endif

#endif /* NVOCTP_H */

