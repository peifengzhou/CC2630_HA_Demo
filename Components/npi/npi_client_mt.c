//*****************************************************************************
//! \file           npi_client_mt.c
//! \brief          Utilities to redirect outbound NPI messages to NPI RTOS
//                  thread.
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
//*****************************************************************************


// ****************************************************************************
// includes
// ****************************************************************************
#include "inc/npi_client.h"
#include "inc/npi_data.h"
#include <ICall.h>
#include <stdint.h>
#include <string.h>
#include "MT_RPC.h"

// ****************************************************************************
// defines
// ****************************************************************************

// ****************************************************************************
// typedefs
// ****************************************************************************

// ****************************************************************************
// globals
// ****************************************************************************

static ICall_EntityID localEntityID;

// ****************************************************************************
// function prototypes
// ****************************************************************************

// ----------------------------------------------------------------------------
//! \brief      Overload the MT function to Build and Send ZTool Response.
//!             This function relays outgoing MT messages to the NPI task for
//!             further handling.
//!
//! \param[in]  cmdType -  MT Command field
//! \param[in]  cmdId - MT Command ID
//! \param[in]  datalen - lenght MT command
//! \param[in]  pData - pointer to MT commnad buffer
//!
//! \return     void
// ----------------------------------------------------------------------------
void MT_BuildAndSendZToolResponse(uint8_t cmdType, uint8_t cmdId,
                                  uint8_t dataLen, uint8_t *pData)
{
    // allocate an ICall message buffer to send message to NPI task.
    uint8_t *pRspMsg = ICall_allocMsg( dataLen + MT_RPC_FRAME_HDR_SZ);

    if(pRspMsg != NULL)
    {
        // populuate the MT header fields.
        pRspMsg[MT_RPC_POS_LEN] = dataLen;
        pRspMsg[MT_RPC_POS_CMD0] = cmdType;
        pRspMsg[MT_RPC_POS_CMD1] = cmdId;

        if(dataLen > 0)
        {
            memcpy(pRspMsg + MT_RPC_FRAME_HDR_SZ, pData, dataLen);
        }

        // Send the message
        ICall_sendServiceMsg(localEntityID, ICALL_SERVICE_CLASS_NPI,
                             ICALL_MSG_FORMAT_KEEP, pRspMsg);
    }

    return;
}

// ----------------------------------------------------------------------------
//! \brief      Save local ICall Entity IDs
//!
//! \param[in]  localEntityID ICall entity ID of the local stack task.
//!
//! \return     void
// ----------------------------------------------------------------------------
void NPIClient_saveLocalTaskInfo(ICall_EntityID localTaskEntityID)
{
    localEntityID = localTaskEntityID;
}

