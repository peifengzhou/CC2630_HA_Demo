/**
  @file  mtPipeClient.c
  @brief Utilities to redirect outbound MT messages to mtPipe RTOS thread.
         Includes overload of MT_BuildAndSendZToolResponse function.

  <!--
  Copyright 2014 Texas Instruments Incorporated. All rights reserved.

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
  -->
*/


/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
#include "mtPipeClient.h"


#include <ICall.h>
#include <stdint.h>
#include <string.h>
#include "rom.h"
#include "mtPipeData.h"

//*****************************************************************************
// Constants
//*****************************************************************************



//*****************************************************************************
// Global Variables
//*****************************************************************************
static ICall_EntityID mtPipeEntityID;
static ICall_EntityID localEntityID;

//*****************************************************************************
// Local Function Prototypes
//*****************************************************************************


/***************************************************************************************************
 * @fn      MT_BuildAndSendZToolResponse
 *
 * @brief   Build and send a ZTOOL msg
 * @param   uint8 cmdType - include type and subsystem
 *          uint8 cmdId - command ID
 *          byte dataLen
 *          byte *pData
 *
 * @return  void
 ***************************************************************************************************/
void MT_BuildAndSendZToolResponse(uint8_t cmdType, uint8_t cmdId, uint8_t dataLen, uint8_t *pData)
{
  
   /* Allocate message buffer space */
  MTPIPE_mTBufRsp_t *pRspMsg = 
    (MTPIPE_mTBufRsp_t *)ICall_allocMsg( sizeof( MTPIPE_mTBufRsp_t ) );

  if ( pRspMsg != NULL )
  {
    /* Fill in the message content */
    pRspMsg->hdr.event = MTPIPE_CmdIDs_MT_RSP;
    /* 
     * Set the pointer for the request and response structures.
     * The caller allocated space for the response 
     */
    pRspMsg->cmdType = cmdType;
    pRspMsg->cmdId = cmdId;
    pRspMsg->dataLen = dataLen;
    if (dataLen > 0)
    {
      pRspMsg->pData = ICall_malloc( dataLen );
      if (pRspMsg->pData == NULL)
      {
        return;
      }
      else
      {
        memcpy(pRspMsg->pData, pData, dataLen);
        //HapiMemcpy(pRspMsg->pData, pData, dataLen);
      }
    }
    else
    {
        pRspMsg->pData = NULL;
    }

    /* Send the message */
    ICall_send( localEntityID, mtPipeEntityID, ICALL_MSG_FORMAT_KEEP, pRspMsg );

  }

  return;      

}

/**
 * @brief       saveMTPipeTaskInfo
 *
 * @param       cmdType  
 * @param       cmdId
 * @param       dataLen
 * @param       pData
 *
 * @return      void
 */
void MTPipeClient_saveMTPipeTaskInfo(ICall_EntityID mtPipeTaskEntityID,
                                     ICall_EntityID localTaskEntityID)
{
  mtPipeEntityID = mtPipeTaskEntityID;
  localEntityID = localTaskEntityID;
}



void MTPipeClient_sendHeartBeat(void)
{
    // send heartbeat message to mtpipe
    MTPIPE_HeartBeatReq_t *pRspMsg = 
    (MTPIPE_HeartBeatReq_t *)ICall_allocMsg( sizeof( MTPIPE_HeartBeatReq_t ) );

    if ( pRspMsg != NULL )
    {
      /* Fill in the message content */
      pRspMsg->hdr.event = MTPIPE_CmdIDs_HEARTBEAT;
 
      /* Send the message */
      ICall_send( localEntityID, mtPipeEntityID, ICALL_MSG_FORMAT_KEEP, pRspMsg );
    }
}
