//******************************************************************************
//! \file           npi_tl_spi.c
//! \brief          NPI Transport Layer Module for SPI
//
//   Revised        $Date: 2015-01-29 12:38:38 -0800 (Thu, 29 Jan 2015) $
//   Revision:      $Revision: 42124 $
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

// ****************************************************************************
// includes
// ****************************************************************************
#include <string.h>
#include <xdc/std.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "Board.h"
#include "hal_types.h"
#include "ICall.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

#include "inc/npi_config.h"
#include "inc/npi_tl.h"
#include "inc/npi_tl_spi.h"
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>

// ****************************************************************************
// defines
// ****************************************************************************
#define SPI_MAX_PKT_LEN NPI_TL_BUF_SIZE

// ****************************************************************************
// typedefs
// ****************************************************************************

//*****************************************************************************
// globals
//*****************************************************************************

//! \brief Handle for SPI object used by SPI driver
static SPI_Handle spiHandle;

//! \brief Structure that defines parameters of one SPI transaction
static SPI_Transaction spiTransaction;

//! \brief NPI TL call back function for the end of a SPI transaction
static npiCB_t npiTransmitCB = NULL;

//! \brief Pointer to NPI TL RX Buffer
static Char* TransportRxBuf;

//! \brief Pointer to NPI TL RX Buffer
static Char* TransportTxBuf;

//! \brief Length of bytes to send from NPI TL Tx Buffer
static uint16 TransportTxBufLen = 0;

//! \brief SPI Object. Initialized in board specific files
extern SPICC26XX_Object spiCC26XXDMAObjects[];

//*****************************************************************************
// function prototypes
//*****************************************************************************

//! \brief Call back function invoked at the end of a SPI transaction
static void NPITLSPI_CallBack(SPI_Handle handle, SPI_Transaction  *objTransaction);

// -----------------------------------------------------------------------------
//! \brief      This routine initializes the transport layer and opens the port
//!             of the device.
//!
//! \param[in]  tRxBuf - pointer to NPI TL Tx Buffer
//! \param[in]  tTxBuf - pointer to NPI TL Rx Buffer
//! \param[in]  npiCBack - NPI TL call back function to be invoked at the end of 
//!             a SPI transaction                     
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLSPI_initializeTransport(Char *tRxBuf, Char *tTxBuf, npiCB_t npiCBack)
{
    SPI_Params spiParams;

    TransportRxBuf = tRxBuf;
    TransportTxBuf = tTxBuf;
    npiTransmitCB = npiCBack;

    // Configure SPI parameters
    SPI_Params_init(&spiParams);

    // Slave mode
    spiParams.mode = SPI_SLAVE;
    spiParams.bitRate = SPI_SLAVE_BAUD_RATE;
    spiParams.frameFormat = SPI_POL1_PHA1;
    spiParams.transferMode = SPI_MODE_CALLBACK;
    spiParams.transferCallbackFxn = NPITLSPI_CallBack;

    // Attempt to open SPI
    spiHandle = SPI_open(NPI_SPI_CONFIG, &spiParams);
    return;
}

// -----------------------------------------------------------------------------
//! \brief      This routine stops any pending reads
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLSPI_stopTransfer()
{
    SPI_transferCancel(spiHandle);
    return;
}

// -----------------------------------------------------------------------------
//! \brief      This routine is called from the application context when MRDY is
//!             de-asserted
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLSPI_handleMrdyEvent()
{
    ICall_CSState key;
    key = ICall_enterCriticalSection();

    // If we have not already set up a write then we must be reading
    //    during this transaction
    if ( !TransportTxBufLen )
    {
      NPITLSPI_readTransport();
    }

    ICall_leaveCriticalSection(key);
    return;
}

// -----------------------------------------------------------------------------
//! \brief      This callback is invoked on transmission completion
//!
//! \param[in]  handle - handle to the SPI port
//! \param[in]  objTransaction    - handle for SPI transmission
//!
//! \return     void
// -----------------------------------------------------------------------------
static void NPITLSPI_CallBack(SPI_Handle handle, SPI_Transaction  *objTransaction)
{
    // Check if a packet was found while parsing the DMA rx buffer
    if ( npiTransmitCB )
    {
        npiTransmitCB(objTransaction->count,TransportTxBufLen);
    }

    //All bytes in TxBuf must be sent by this point
    TransportTxBufLen = 0;
}

// -----------------------------------------------------------------------------
//! \brief      This routine reads data from the transport layer
//!             and places it into the buffer.
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPITLSPI_readTransport()
{
    ICall_CSState key;
    key = ICall_enterCriticalSection();

    TransportTxBufLen = 0;

    // Clear DMA Rx buffer and clear extra Tx buffer bytes to ensure clean buffer 
    //    for next RX/TX
    memset(TransportRxBuf, 0, NPI_TL_BUF_SIZE);
    memset(&TransportTxBuf[TransportTxBufLen], 0, NPI_TL_BUF_SIZE - TransportTxBufLen);

    // set up the SPI Transaction
    spiTransaction.txBuf = TransportTxBuf;
    spiTransaction.rxBuf = TransportRxBuf;
    spiTransaction.count = NPI_TL_BUF_SIZE;
    SPI_transfer(spiHandle, &spiTransaction);

    ICall_leaveCriticalSection(key);
}

// -----------------------------------------------------------------------------
//! \brief      This routine initializes and begins a SPI transaction
//!
//! \param[in]  len - Number of bytes to write.
//!
//! \return     uint16 - number of bytes written to transport
// -----------------------------------------------------------------------------
uint16 NPITLSPI_writeTransport(uint16 len)
{
    ICall_CSState key;
    key = ICall_enterCriticalSection();

    TransportTxBufLen = len;

    // Clear DMA Rx buffer and clear extra Tx buffer bytes to ensure clean buffer
    //    for next RX/TX
    memset(TransportRxBuf, 0, NPI_TL_BUF_SIZE);
    memset(&TransportTxBuf[TransportTxBufLen], 0, NPI_TL_BUF_SIZE - TransportTxBufLen);

    // set up the SPI Transaction
    spiTransaction.count = NPI_TL_BUF_SIZE;
    spiTransaction.txBuf = TransportTxBuf;
    spiTransaction.rxBuf = TransportRxBuf;
    
    // Check to see if transport is successful. If not, reset TxBufLen to allow
    // another write to be processed
    if( ! SPI_transfer(spiHandle, &spiTransaction) )
    {
      TransportTxBufLen = 0;
    }

    ICall_leaveCriticalSection(key);

    return TransportTxBufLen;
}
