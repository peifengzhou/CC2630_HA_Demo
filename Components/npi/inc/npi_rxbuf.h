//******************************************************************************
//! \file           npi_rxbuf.h
//! \brief          NPI RX Buffer and utilities
//
//   Revised        $Date: 2015-01-29 11:51:00 -0800 (Thu, 29 Jan 2015) $
//   Revision:      $Revision: 42121 $
//
//  Copyright 2015 Texas Instruments Incorporated. All rights reserved.
//
// IMPORTANT: Your use of this Software is limited to those specific rights
// granted under the terms of a software license agreement between the user
// who downloaded the software, his/her employer (which must be your employer)
// and Texas Instruments Incorporated (the "License").  You may not use this
// Software unless you agree to abide by the terms of the License. The License
// limits your use, and you acknowledge, that the Software may not be modified,
// copied or distributed unless used solely and exclusively in conjunction with
// a Texas Instruments radio frequency device, which is integrated into
// your product.  Other than for the foregoing purpose, you may not use,
// reproduce, copy, prepare derivative works of, modify, distribute, perform,
// display or sell this Software and/or its documentation for any purpose.
//
//  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
//  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,l
//  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
//  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
//  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
//  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
//  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
//  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
//  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
//  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
//  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
//
//  Should you have any questions regarding your right to use this Software,
//  contact Texas Instruments Incorporated at www.TI.com.
//******************************************************************************
#ifndef NPIRXBUF_H
#define NPIRXBUF_H

#ifdef __cplusplus
extern "C"
{
#endif

// ****************************************************************************
// includes
// ****************************************************************************
#include "hal_types.h"
#include "OSAL.h"
#include "npi_config.h"

// ****************************************************************************
// defines
// ****************************************************************************

// ****************************************************************************
// typedefs
// ****************************************************************************

//*****************************************************************************
// globals
//*****************************************************************************

//*****************************************************************************
// function prototypes
//*****************************************************************************

// -----------------------------------------------------------------------------
//! \brief      NPIRxBuf_Read
//!
//! \param[in]  len -
//!
//! \return     uint16 -
// -----------------------------------------------------------------------------
uint16 NPIRxBuf_Read(uint16);

// -----------------------------------------------------------------------------
//! \brief      Returns number of bytes that are unparsed in RxBuf
//!
//! \return     uint16 -
// -----------------------------------------------------------------------------
uint16 NPIRxBuf_GetRxBufLen();

// -----------------------------------------------------------------------------
//! \brief      NPIRxBuf_ReadFromRxBuf
//!
//! \return     uint16 -
// -----------------------------------------------------------------------------
uint16 NPIRxBuf_ReadFromRxBuf(uint8_t *buf, uint16 len);

#ifdef NPI_USE_SPI
// -----------------------------------------------------------------------------
//! \brief      NPIRxBuf_IgnoreFromRxBuf
//!
//! \return     uint16 - Number of bytes to ignore
// -----------------------------------------------------------------------------
void NPIRxBuf_IgnoreFromRxBuf(uint16 len);

// -----------------------------------------------------------------------------
//! \brief      Returns true if FCS is correct. Must be called when npiRxBufHead
//!             is at the index of the first byte of data in an SPI payload
//!
//! \return     uint16 - Length of bytes over which to calculate FCS
// -----------------------------------------------------------------------------
uint16 NPIRxBuf_VerifyFCS(uint8 len);
#endif //NPI_USE_SPI

#ifdef __cplusplus
}
#endif

#endif /* NPIRXBUF_H */
