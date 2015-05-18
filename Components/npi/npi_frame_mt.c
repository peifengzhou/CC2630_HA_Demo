//******************************************************************************
//! \file           npi_frame_mt.c
//! \brief          This file contains the Network Processor Interface (NPI)
//!                 data frame specific function implementations for the MT
//!                 serial interface.
//
//   Revised        $Date: 2015-02-15 12:05:18 -0800 (Sun, 15 Feb 2015) $
//   Revision:      $Revision: 42613 $
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

#include "MT_RPC.h"
#include <string.h>
#include "inc/npi_frame.h"
#include "inc/npi_rxbuf.h"

// ****************************************************************************
// defines
// ****************************************************************************

//! \name State values for ZTool protocal
//@{
#define NPIFRAMEMT_SOP_STATE 0x00
#define NPIFRAMEMT_CMD_STATE1 0x01
#define NPIFRAMEMT_CMD_STATE2 0x02
#define NPIFRAMEMT_LEN_STATE 0x03
#define NPIFRAMEMT_DATA_STATE 0x04
#define NPIFRAMEMT_FCS_STATE 0x05
//@}

//! \brief Start-of-frame delimiter for UART transport
//!
#define MT_SOF 0xFE

// ****************************************************************************
// typedefs
// ****************************************************************************

//*****************************************************************************
// globals
//*****************************************************************************

//! \brief Call back function to call when a complete frame has been collected.
//!
static npiIncomingFrameCBack_t incomingFrameCBFunc = NULL;

//! \name MT serial collection globals
//@{
static uint8_t state = 0;
static uint8_t LEN_Token = 0;
static uint8_t FSC_Token = 0;
static uint8_t *pMsg = NULL;
static __no_init uint8_t tempDataLen;
//@}

//*****************************************************************************
// function prototypes
//*****************************************************************************

// -----------------------------------------------------------------------------
//! \brief      Calculates FCS for MT Frame
//!
//! \param[in]  msg_ptr   Pointer to message buffer.
//! \param[in]  len       Length of message pointer pointed to by msg_ptr.
//!
//! \return     uint8_t   Calculated FCS.
// -----------------------------------------------------------------------------
uint8_t npiframe_calcMTFCS(uint8_t *msg_ptr, uint8_t len);

// -----------------------------------------------------------------------------
//! \brief      Initialize Frame module with NPI callbacks.
//!
//! \param[in]  incomingFrameCB   Call back for complete inbound (from host)
//!                               messages
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPIFrame_initialize(npiIncomingFrameCBack_t incomingFrameCB)
{
    incomingFrameCBFunc = incomingFrameCB;
}

// -----------------------------------------------------------------------------
//! \brief      Bundles message into Transport Layer frame and NPIMSG_msg_t
//!             container.  This is the MT specific version of this function.
//!
//!             This function frames the passed in buffer with an MT SOF and FCS
//!             bytes.  It then bundles the message in an NPIMSG_msg_t
//!             container.
//!
//!             Note: becauase the SOF and FCS are added, the passed in buffer
//!             is copied to a new buffer and then the passed in buffer is
//!             free'd.
//!
//! \param[in]  pData     Pointer to message buffer.
//!
//! \return     void
// -----------------------------------------------------------------------------
NPIMSG_msg_t *NPIFrame_frameMsg(uint8_t *pIncomingMsg)
{
    uint8_t *payload;

    NPIMSG_msg_t *npiMsg =
        (NPIMSG_msg_t *)ICall_malloc( sizeof(NPIMSG_msg_t));

    if (npiMsg != NULL)
    {
        // extract the message length from the MT header bytes.
        uint8_t incomingMsgLen = pIncomingMsg[MT_RPC_POS_LEN] + MT_RPC_FRAME_HDR_SZ;

        // allocate a new buffer that is the incoming buffer length + 2
        // additional bytes for the SOF and FCS bytes.
        if ((npiMsg->pBuf = (uint8_t *) ICall_allocMsg(incomingMsgLen + 2)) != NULL)
        {
            // mark the start of frame.
            npiMsg->pBuf[0] = MT_SOF;
            payload = npiMsg->pBuf + 1;

            // copy the incoming buffer into the newly created buffer
            memcpy(payload, pIncomingMsg, incomingMsgLen);

            // calculate and capture the FCS in the final byte.
            npiMsg->pBuf[incomingMsgLen + 1] = npiframe_calcMTFCS( npiMsg->pBuf + 1,
                                                                   incomingMsgLen);
#if defined(NPI_SREQRSP)
            // document message type (SYNC or ASYNC) in the NPI container.
            if ((pIncomingMsg[MT_RPC_POS_CMD0] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SRSP)
            {
                npiMsg->msgType = NPIMSG_Type_SYNCRSP;
            }
            else
            {
                npiMsg->msgType = NPIMSG_Type_ASYNC;
            }
#else
            npiMsg->msgType = NPIMSG_Type_ASYNC;
#endif
            // capture the included buffer size in the NPI container.
            npiMsg->pBufSize = incomingMsgLen + 2;

            // the passed in buffer is no longer needed, free it.
            ICall_freeMsg(pIncomingMsg);
        }
        else
        {
            // abort and free allocated memory.
            ICall_freeMsg(pIncomingMsg);
            ICall_free(npiMsg);
        }
    }
    else
    {
        // abort and free allocated memory.

        ICall_freeMsg(pIncomingMsg);
    }

    return npiMsg;
}

// -----------------------------------------------------------------------------
//! \brief      Collects MT message buffer.  Used during serial data receipt.
//!
//!             | SOP | Data Length  |   CMD   |   Data   |  FCS  |
//!             |  1  |     1        |    2    |  0-Len   |   1   |
//!
//! \return     void
// -----------------------------------------------------------------------------
void NPIFrame_collectFrameData(void)
{
    uint8_t ch;
    uint8_t uint8_tsInRxBuffer;

    while (NPIRxBuf_GetRxBufLen())
    {
        NPIRxBuf_ReadFromRxBuf(&ch, 1);

        switch (state)
        {
            case NPIFRAMEMT_SOP_STATE:
                if (ch == MT_SOF)
                {
                    state = NPIFRAMEMT_LEN_STATE;
                }
                break;

            case NPIFRAMEMT_LEN_STATE:
                LEN_Token = ch;

                tempDataLen = 0;

                /* Allocate memory for the data */
                pMsg = (uint8_t *) ICall_allocMsg(MT_RPC_FRAME_HDR_SZ + LEN_Token);

                if (pMsg)
                {
                    pMsg[MT_RPC_POS_LEN] = LEN_Token;
                    state = NPIFRAMEMT_CMD_STATE1;
                }
                else
                {
                    state = NPIFRAMEMT_SOP_STATE;
                    return;
                }
                break;

            case NPIFRAMEMT_CMD_STATE1:
                pMsg[MT_RPC_POS_CMD0] = ch;
                state = NPIFRAMEMT_CMD_STATE2;
                break;

            case NPIFRAMEMT_CMD_STATE2:
                pMsg[MT_RPC_POS_CMD1] = ch;
                /* If there is no data, skip to FCS state */
                if (LEN_Token)
                {
                    state = NPIFRAMEMT_DATA_STATE;
                }
                else
                {
                    state = NPIFRAMEMT_FCS_STATE;
                }
                break;

            case NPIFRAMEMT_DATA_STATE:

                /* Fill in the buffer the first uint8_t of the data */
                pMsg[MT_RPC_FRAME_HDR_SZ + tempDataLen++] = ch;

                /* Check number of uint8_ts left in the Rx buffer */
                uint8_tsInRxBuffer = NPIRxBuf_GetRxBufLen();

                /* If the remain of the data is there, read them all, otherwise, just read enough */
                if (uint8_tsInRxBuffer <= LEN_Token - tempDataLen)
                {
                    NPIRxBuf_ReadFromRxBuf(&pMsg[MT_RPC_FRAME_HDR_SZ + tempDataLen], uint8_tsInRxBuffer);
                    tempDataLen += uint8_tsInRxBuffer;
                }
                else
                {
                    NPIRxBuf_ReadFromRxBuf(&pMsg[MT_RPC_FRAME_HDR_SZ + tempDataLen], LEN_Token - tempDataLen);
                    tempDataLen += (LEN_Token - tempDataLen);
                }

                /* If number of uint8_ts read is equal to data length, time to move on to FCS */
                if (tempDataLen == LEN_Token)
                {
                    state = NPIFRAMEMT_FCS_STATE;
                }

                break;

            case NPIFRAMEMT_FCS_STATE:

                FSC_Token = ch;

                /* Make sure it's correct */
                if ((npiframe_calcMTFCS((uint8_t *)&pMsg[0], MT_RPC_FRAME_HDR_SZ + LEN_Token) == FSC_Token))
                {

                    /* Determine if it's a SYNC or ASYNC message */
                    NPIMSG_Type msgType;
                    
#if defined(NPI_SREQRSP)
                    if ((pMsg[1] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SREQ)
                    {
                        msgType = NPIMSG_Type_SYNCREQ;
                    }
                    else
                    {
                        msgType = NPIMSG_Type_ASYNC;
                    }
#else
                    msgType = NPIMSG_Type_ASYNC;
#endif // NPI_SREQRSP
                    
                    if ( incomingFrameCBFunc )
                    {
                        incomingFrameCBFunc(MT_RPC_FRAME_HDR_SZ + LEN_Token, pMsg, msgType);
                    }

                }
                else
                {
                    /* deallocate the msg */
                    ICall_free(pMsg);
                }

                /* Reset the state, send or discard the buffers at this point */
                state = NPIFRAMEMT_SOP_STATE;

                break;

            default:
                break;
        }
    }
}

// -----------------------------------------------------------------------------
//! \brief      Calculate the FCS of a message buffer by XOR'ing each uint8_t.
//!             Remember to NOT include SOP and FCS fields, so start at the CMD
//!             field.
//!
//! \param[in]  msg_ptr   message pointer
//! \param[in]  len       ength (in uint8_ts) of message
//!
//! \return     uint8_t
// -----------------------------------------------------------------------------
uint8_t npiframe_calcMTFCS(uint8_t *msg_ptr, uint8_t len)
{
    uint8_t x;
    uint8_t xorResult;

    xorResult = 0;

    for (x = 0; x < len; x++, msg_ptr++)
    {
        xorResult = xorResult ^ *msg_ptr;
    }

    return (xorResult);
}
