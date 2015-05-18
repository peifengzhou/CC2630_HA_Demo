/**
   @file  zstackapi.c
   @brief ZStack C interface implementation on top of
   dispatcher messaging interface.

   <!--
   Copyright 2014 - 2015 Texas Instruments Incorporated. All rights reserved.

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
   PROVIDED ``AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include <string.h>
#include <ICall.h>

#include "zstackapi.h"

//*****************************************************************************
// Structures
//*****************************************************************************
typedef struct _genericreqrsp_t
{
    /** message header<br>
     * event field must be set to @ref ICALL_CMD_EVENT.
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    void *pReq;

    /** Response fields (immediate response) */
    void *pRsp;

} GenericReqRsp_t;

//*****************************************************************************
// Local Functions
//*****************************************************************************

/**
 * Generic function to send a request message to the ZStack Thread
 * and wait for a "default" response message.
 *
 * @param appEntity - Application iCall Entity ID
 * @param cmdID - Command ID of the message
 * @param pReq - Pointer to the request's structure
 * @param msgSize - length of the message being sent
 * @param pFnMatch - Function pointer to the response matching function
 *
 * @return zstack_ZStatusValues
 */
static zstack_ZStatusValues sendReqDefaultRsp(ICall_EntityID appEntity,
                                              zstack_CmdIDs cmdID, void *pReq,
                                              int              msgSize,
                                              ICall_MsgMatchFn pFnMatch)
{
    zstack_ZStatusValues status = zstack_ZStatusValues_ZMemError;

    // Allocate message buffer space
    ICall_Errno errno;
    zstackmsg_genericReq_t *pMsg =
        (zstackmsg_genericReq_t *)ICall_allocMsg(msgSize);

    // Make sure the allocation was successful
    if(pMsg != NULL)
    {
        // Fill in the message header
        pMsg->hdr.event = cmdID;
        pMsg->hdr.status = 0;

        // Update the messges's request field
        pMsg->pReq = pReq;

        // Send the message
        errno = ICall_sendServiceMsg(appEntity, ICALL_SERVICE_CLASS_ZSTACK,
                                     ICALL_MSG_FORMAT_KEEP, pMsg);

        // Was the message sent successfully
        if(errno == ICALL_ERRNO_SUCCESS)
        {
            // Return status
            zstackmsg_genericReq_t *pCmdStatus = NULL;

            // Wait for the response message
            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER, pFnMatch, NULL,
                                    NULL, (void **)&pCmdStatus);

            // Was the response successful
            if(errno == ICALL_ERRNO_SUCCESS)
            {
                // setup return of
                status = (zstack_ZStatusValues)pMsg->hdr.status;
            }
            else
            {
                // Translate ICall Error status to ZStack API status
                if(errno == ICALL_ERRNO_NOMSG)
                {
                    status = zstack_ZStatusValues_ZIcallNoMsg;
                }
                else if(errno == ICALL_ERRNO_TIMEOUT)
                {
                    status = zstack_ZStatusValues_ZIcallTimeout;
                }
                else
                {
                    status = zstack_ZStatusValues_ZFailure;
                }
            }
        }

        // pCmdStatus is the same as pMsg
        ICall_freeMsg(pMsg);
    }

    // function status
    return(status);
}

/**
 * Generic function to send a request message to the ZStack Thread
 * and specific response message.
 *
 * @param appEntity - Application iCall Entity ID
 * @param cmdID - Command ID of the message
 * @param pReq - Pointer to the request's structure
 * @param pRsp - Pointer to the request's structure
 * @param msgSize - length of the message being sent
 * @param pFnMatch - Function pointer to the response matching function
 *
 * @return zstack_ZStatusValues
 */
static zstack_ZStatusValues sendReqRsp(ICall_EntityID appEntity,
                                       zstack_CmdIDs cmdID, void *pReq,
                                       void *pRsp, int              msgSize,
                                       ICall_MsgMatchFn pFnMatch)
{
    zstack_ZStatusValues status = zstack_ZStatusValues_ZMemError;

    // Allocate message buffer space
    ICall_Errno errno;
    GenericReqRsp_t *pMsg = (GenericReqRsp_t *)ICall_allocMsg(msgSize);

    // Make sure the allocation was successful
    if(pMsg != NULL)
    {
        // Fill in the message content
        pMsg->hdr.event = cmdID;
        pMsg->hdr.status = 0;

        /*
         * Set the pointer for the request and response structures.
         * The caller allocated space for the response
         */
        pMsg->pReq = pReq;
        pMsg->pRsp = pRsp;

        // Send the message
        errno = ICall_sendServiceMsg(appEntity, ICALL_SERVICE_CLASS_ZSTACK,
                                     ICALL_MSG_FORMAT_KEEP, pMsg);

        // Was the message sent successfully
        if(errno == ICALL_ERRNO_SUCCESS)
        {
            GenericReqRsp_t *pCmdStatus = NULL;

            // Wait for the response message
            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER, pFnMatch, NULL,
                                    NULL, (void **)&pCmdStatus);

            // Was the response successful
            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (zstack_ZStatusValues)pMsg->hdr.status;
            }
            else
            {
                // Translate ICall Error status to ZStack API status
                if(errno == ICALL_ERRNO_NOMSG)
                {
                    status = zstack_ZStatusValues_ZIcallNoMsg;
                }
                else if(errno == ICALL_ERRNO_TIMEOUT)
                {
                    status = zstack_ZStatusValues_ZIcallTimeout;
                }
                else
                {
                    status = zstack_ZStatusValues_ZFailure;
                }
            }
        }

        // pCmdStatus is the same as pMsg
        ICall_freeMsg(pMsg);
    }

    // Function status
    return(status);
}

/**
 * Compare a received a System Reset Default Status Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSysResetRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                             const void *msg)
{
    zstackmsg_sysResetReq_t *pMsg = (zstackmsg_sysResetReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event ==
             zstackmsg_CmdIDs_SYS_RESET_REQ) ? true : false );
}

/**
 * Compare a received a System Version Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSysVersionRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                               const void *msg)
{
    zstackmsg_sysVersionReq_t *pMsg = (zstackmsg_sysVersionReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event ==
             zstackmsg_CmdIDs_SYS_VERSION_REQ) ? true : false );
}

/**
 * Compare a received a System Config Read Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSysConfigReadRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                  const void *msg)
{
    zstackmsg_sysConfigReadReq_t *pMsg = (zstackmsg_sysConfigReadReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_SYS_CONFIG_READ_REQ) ?
            true : false );
}

/**
 * Compare a received a System Config Write Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSysConfigWriteRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                   const void *msg)
{
    zstackmsg_sysConfigWriteReq_t *pMsg =
        (zstackmsg_sysConfigWriteReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_SYS_CONFIG_WRITE_REQ) ?
            true : false );
}

/**
 * Compare a received a System Set Tx Power Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSysSetTxPowerRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                  const void *msg)
{
    zstackmsg_sysSetTxPowerReq_t *pMsg = (zstackmsg_sysSetTxPowerReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_SYS_SET_TX_POWER_REQ) ?
            true : false );
}

/**
 * Compare a received a System Force Link Status Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSysForceLinkStatusRsp(ICall_ServiceEnum src,
                                       ICall_EntityID    dest,
                                       const void *      msg)
{
    zstackmsg_sysForceLinkStatusReq_t *pMsg =
        (zstackmsg_sysForceLinkStatusReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_SYS_FORCE_LINK_STATUS_REQ) ?
            true : false );
}

/**
 * Compare a received a System Network Information Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSysNwkInfoRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                               const void *msg)
{
    zstackmsg_sysNwkInfoReadReq_t *pMsg =
        (zstackmsg_sysNwkInfoReadReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_SYS_NWK_INFO_READ_REQ) ?
            true : false );
}

/**
 * Compare a received a Device Start Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchDevStartRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                             const void *msg)
{
    zstackmsg_devStartReq_t *pMsg = (zstackmsg_devStartReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event ==
             zstackmsg_CmdIDs_DEV_START_REQ) ? true : false );
}

/**
 * Compare a received a Device Network Discovery Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchDevNwkDiscRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                               const void *msg)
{
    zstackmsg_devNwkDiscReq_t *pMsg = (zstackmsg_devNwkDiscReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_DEV_NWK_DISCOVERY_REQ) ?
            true : false );
}

/**
 * Compare a received a Device Join Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchDevJoinRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                            const void *msg)
{
    zstackmsg_devJoinReq_t *pMsg = (zstackmsg_devJoinReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_DEV_JOIN_REQ) ? true : false );
}

/**
 * Compare a received a Device Rejoin Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchDevRejoinRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                              const void *msg)
{
    zstackmsg_devRejoinReq_t *pMsg = (zstackmsg_devRejoinReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event ==
             zstackmsg_CmdIDs_DEV_REJOIN_REQ) ? true : false );
}

/**
 * Compare a received a Device Rejoin Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchDevZDOCBRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                             const void *msg)
{
    zstackmsg_devZDOCBReq_t *pMsg = (zstackmsg_devZDOCBReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event ==
             zstackmsg_CmdIDs_DEV_ZDO_CBS_REQ) ? true : false );
}

/**
 * Compare a received a Device Network Route Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchDevNwkRouteRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                const void *msg)
{
    zstackmsg_devNwkRouteReq_t *pMsg = (zstackmsg_devNwkRouteReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_DEV_NWK_ROUTE_REQ) ?
            true : false );
}

/**
 * Compare a received a Device Network Check Route Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchDevNwkCheckRouteRsp(ICall_ServiceEnum src,
                                     ICall_EntityID    dest,
                                     const void *      msg)
{
    zstackmsg_devNwkCheckRouteReq_t *pMsg =
        (zstackmsg_devNwkCheckRouteReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_DEV_NWK_CHECK_ROUTE_REQ) ?
            true : false );
}

/**
 * Compare a received a Device Update Neighbor's TxCost Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchDevUpdateNeighborTxCostRsp(ICall_ServiceEnum src,
                                            ICall_EntityID    dest,
                                            const void *      msg)
{
    zstackmsg_devUpdateNeighborTxCostReq_t *pMsg =
        (zstackmsg_devUpdateNeighborTxCostReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event ==
             zstackmsg_CmdIDs_DEV_UPDATE_NEIGHBOR_TXCOST_REQ) ? true :
            false );
}

/**
 * Compare a received a Device Force Network Settings Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchDevForceNetworkSettingsRsp(ICall_ServiceEnum src,
                                            ICall_EntityID    dest,
                                            const void *      msg)
{
    zstackmsg_devForceNetworkSettingsReq_t *pMsg =
        (zstackmsg_devForceNetworkSettingsReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event ==
             zstackmsg_CmdIDs_DEV_FORCE_NETWORK_SETTINGS_REQ) ? true :
            false );
}

/**
 * Compare a received a Device Force Network Update Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchDevForceNetworkUpdateRsp(ICall_ServiceEnum src,
                                          ICall_EntityID    dest,
                                          const void *      msg)
{
    zstackmsg_devForceNetworkUpdateReq_t *pMsg =
        (zstackmsg_devForceNetworkUpdateReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event ==
             zstackmsg_CmdIDs_DEV_FORCE_NETWORK_UPDATE_REQ) ? true : false );
}

/**
 * Compare a received a Device Force MAC Parameters Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchDevForceMacParamsRsp(ICall_ServiceEnum src,
                                      ICall_EntityID    dest,
                                      const void *      msg)
{
    zstackmsg_devForceMacParamsReq_t *pMsg =
        (zstackmsg_devForceMacParamsReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_DEV_FORCE_MAC_PARAMS_REQ) ?
            true : false );
}

/**
 * Compare a received an APS Remove Group Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchApsRemoveGroupRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                   const void *msg)
{
    zstackmsg_apsRemoveGroup_t *pMsg = (zstackmsg_apsRemoveGroup_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_APS_REMOVE_GROUP) ?
            true : false );
}

/**
 * Compare a received an APS Remove All Groups Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchApsRemoveAllGroupsRsp(ICall_ServiceEnum src,
                                       ICall_EntityID    dest,
                                       const void *      msg)
{
    zstackmsg_apsRemoveAllGroups_t *pMsg =
        (zstackmsg_apsRemoveAllGroups_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_APS_REMOVE_ALL_GROUPS) ?
            true : false );
}

/**
 * Compare a received an APS Find All Groups Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchApsFindAllGroupsRsp(ICall_ServiceEnum src,
                                     ICall_EntityID    dest,
                                     const void *      msg)
{
    zstackmsg_apsFindAllGroupsReq_t *pMsg =
        (zstackmsg_apsFindAllGroupsReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_APS_FIND_ALL_GROUPS_REQ) ?
            true : false );
}

/**
 * Compare a received an APS Find Group Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchApsFindGroupRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                 const void *msg)
{
    zstackmsg_apsFindGroupReq_t *pMsg = (zstackmsg_apsFindGroupReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_APS_FIND_GROUP_REQ) ?
            true : false );
}

/**
 * Compare a received an APS Add Group Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchApsAddGroupRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                const void *msg)
{
    zstackmsg_apsAddGroup_t *pMsg = (zstackmsg_apsAddGroup_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event ==
             zstackmsg_CmdIDs_APS_ADD_GROUP) ? true : false );
}

/**
 * Compare a received an APS Count All Groups Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchApsCountAllGroupsRsp(ICall_ServiceEnum src,
                                      ICall_EntityID    dest,
                                      const void *      msg)
{
    zstackmsg_apsCountAllGroups_t *pMsg =
        (zstackmsg_apsCountAllGroups_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_APS_COUNT_ALL_GROUPS) ?
            true : false );
}

/**
 * Compare a received an AF Register Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchAfRegisterRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                               const void *msg)
{
    zstackmsg_afRegisterReq_t *pMsg = (zstackmsg_afRegisterReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event ==
             zstackmsg_CmdIDs_AF_REGISTER_REQ) ? true : false );
}

/**
 * Compare a received an AF Unregister Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchAfUnregisterRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                 const void *msg)
{
    zstackmsg_afUnRegisterReq_t *pMsg = (zstackmsg_afUnRegisterReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_AF_UNREGISTER_REQ) ?
            true : false );
}

/**
 * Compare a received an AF Data Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchAfDataRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                           const void *msg)
{
    zstackmsg_afDataReq_t *pMsg = (zstackmsg_afDataReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_AF_DATA_REQ) ? true : false );
}

/**
 * Compare a received an AF InterPAN Control Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchAfInterpanCtlRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                  const void *msg)
{
    zstackmsg_afInterPanCtlReq_t *pMsg = (zstackmsg_afInterPanCtlReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_AF_INTERPAN_CTL_REQ) ?
            true : false );
}

/**
 * Compare a received an AF Config Get Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchAfConfigGetRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                const void *msg)
{
    zstackmsg_afConfigGetReq_t *pMsg = (zstackmsg_afConfigGetReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_AF_CONFIG_GET_REQ) ?
            true : false );
}

/**
 * Compare a received an AF Config Set Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchAfConfigSetRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                const void *msg)
{
    zstackmsg_afConfigSetReq_t *pMsg = (zstackmsg_afConfigSetReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_AF_CONFIG_SET_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Nwk Address Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoNwkAddrRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                               const void *msg)
{
    zstackmsg_zdoNwkAddrReq_t *pMsg = (zstackmsg_zdoNwkAddrReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_NWK_ADDR_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO IEEE Address Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoIeeeAddrRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                const void *msg)
{
    zstackmsg_zdoIeeeAddrReq_t *pMsg = (zstackmsg_zdoIeeeAddrReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_IEEE_ADDR_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Node Descriptor Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoNodeDescRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                const void *msg)
{
    zstackmsg_zdoNodeDescReq_t *pMsg = (zstackmsg_zdoNodeDescReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_NODE_DESC_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Power Descriptor Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoPowerDescRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                 const void *msg)
{
    zstackmsg_zdoPowerDescReq_t *pMsg = (zstackmsg_zdoPowerDescReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_POWER_DESC_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Simple Descriptor Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoSimpleDescRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                  const void *msg)
{
    zstackmsg_zdoSimpleDescReq_t *pMsg = (zstackmsg_zdoSimpleDescReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Active Endpoints Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoActiveEndpointRsp(ICall_ServiceEnum src,
                                      ICall_EntityID    dest,
                                      const void *      msg)
{
    zstackmsg_zdoActiveEndpointReq_t *pMsg =
        (zstackmsg_zdoActiveEndpointReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_ACTIVE_ENDPOINT_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Match Descriptor Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoMatchDescRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                 const void *msg)
{
    zstackmsg_zdoMatchDescReq_t *pMsg = (zstackmsg_zdoMatchDescReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_MATCH_DESC_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Complex Descriptor Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoComplexDescRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                   const void *msg)
{
    zstackmsg_zdoComplexDescReq_t *pMsg =
        (zstackmsg_zdoComplexDescReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_COMPLEX_DESC_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO User Descriptor Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoUserDescRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                const void *msg)
{
    zstackmsg_zdoUserDescReq_t *pMsg = (zstackmsg_zdoUserDescReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_USER_DESC_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Device Announce Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoDeviceAnnounceRsp(ICall_ServiceEnum src,
                                      ICall_EntityID    dest,
                                      const void *      msg)
{
    zstackmsg_zdoDeviceAnnounceReq_t *pMsg =
        (zstackmsg_zdoDeviceAnnounceReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO User Descriptor Set Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoUserDescSetRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                   const void *msg)
{
    zstackmsg_zdoUserDescSetReq_t *pMsg =
        (zstackmsg_zdoUserDescSetReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_USER_DESCR_SET_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Server Discovery Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoServerDiscRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                  const void *msg)
{
    zstackmsg_zdoServerDiscReq_t *pMsg = (zstackmsg_zdoServerDiscReq_t *)msg;

    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_SERVER_DISC_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO End Device Bind Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoEndDeviceBindRsp(ICall_ServiceEnum src,
                                     ICall_EntityID    dest,
                                     const void *      msg)
{
    zstackmsg_zdoEndDeviceBindReq_t *pMsg =
        (zstackmsg_zdoEndDeviceBindReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Bind Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoBindRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                            const void *msg)
{
    zstackmsg_zdoBindReq_t *pMsg = (zstackmsg_zdoBindReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_BIND_REQ) ? true : false );
}

/**
 * Compare a received a ZDO Unbind Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoUnbindRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                              const void *msg)
{
    zstackmsg_zdoUnbindReq_t *pMsg = (zstackmsg_zdoUnbindReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event ==
             zstackmsg_CmdIDs_ZDO_UNBIND_REQ) ? true : false );
}

/**
 * Compare a received a ZDO Mgmt Network Discovery Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoMgmtNwkDiscRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                   const void *msg)
{
    zstackmsg_zdoMgmtNwkDiscReq_t *pMsg =
        (zstackmsg_zdoMgmtNwkDiscReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Mgmt LQI Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoMgmtLqiRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                               const void *msg)
{
    zstackmsg_zdoMgmtLqiReq_t *pMsg = (zstackmsg_zdoMgmtLqiReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_MGMT_LQI_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Mgmt Routing Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoMgmtRtgRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                               const void *msg)
{
    zstackmsg_zdoMgmtRtgReq_t *pMsg = (zstackmsg_zdoMgmtRtgReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_MGMT_RTG_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Mgmt Bind Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoMgmtBindRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                const void *msg)
{
    zstackmsg_zdoMgmtBindReq_t *pMsg = (zstackmsg_zdoMgmtBindReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_MGMT_BIND_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Mgmt Leave Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoMgmtLeaveRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                 const void *msg)
{
    zstackmsg_zdoMgmtLeaveReq_t *pMsg = (zstackmsg_zdoMgmtLeaveReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Mgmt Direct Join Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoMgmtDirectJoinRsp(ICall_ServiceEnum src,
                                      ICall_EntityID    dest,
                                      const void *      msg)
{
    zstackmsg_zdoMgmtDirectJoinReq_t *pMsg =
        (zstackmsg_zdoMgmtDirectJoinReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Mgmt Permit Join Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoMgmtPermitJoinRsp(ICall_ServiceEnum src,
                                      ICall_EntityID    dest,
                                      const void *      msg)
{
    zstackmsg_zdoMgmtPermitJoinReq_t *pMsg =
        (zstackmsg_zdoMgmtPermitJoinReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_REQ) ?
            true : false );
}

/**
 * Compare a received a ZDO Mgmt Network Update Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchZdoMgmtNwkUpdateRsp(ICall_ServiceEnum src,
                                     ICall_EntityID    dest,
                                     const void *      msg)
{
    zstackmsg_zdoMgmtNwkUpdateReq_t *pMsg =
        (zstackmsg_zdoMgmtNwkUpdateReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_REQ) ?
            true : false );
}

/**
 * Compare a received a Security Network Key Get Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSecNwkKeyGetRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                 const void *msg)
{
    zstackmsg_secNwkKeyGetReq_t *pMsg = (zstackmsg_secNwkKeyGetReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_SEC_NWK_KEY_GET_REQ) ?
            true : false );
}

/**
 * Compare a received a Security Network Key Set Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSecNwkKeySetRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                 const void *msg)
{
    zstackmsg_secNwkKeySetReq_t *pMsg = (zstackmsg_secNwkKeySetReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_SEC_NWK_KEY_SET_REQ) ?
            true : false );
}

/**
 * Compare a received a Security Network Key Update Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSecNwkKeyUpdateRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                    const void *msg)
{
    zstackmsg_secNwkKeyUpdateReq_t *pMsg =
        (zstackmsg_secNwkKeyUpdateReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_SEC_NWK_KEY_UPDATE_REQ) ?
            true : false );
}

/**
 * Compare a received a Security Network Key Switch Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSecNwkKeySwitchRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                    const void *msg)
{
    zstackmsg_secNwkKeySwitchReq_t *pMsg =
        (zstackmsg_secNwkKeySwitchReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_SEC_NWK_KEY_SWITCH_REQ) ?
            true : false );
}

/**
 * Compare a received a Security APS Link Key Get Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSecApsLinkKeyGetRsp(ICall_ServiceEnum src,
                                     ICall_EntityID    dest,
                                     const void *      msg)
{
    zstackmsg_secApsLinkKeyGetReq_t *pMsg =
        (zstackmsg_secApsLinkKeyGetReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_SEC_APS_LINKKEY_GET_REQ) ?
            true : false );
}

/**
 * Compare a received a Security APS Link Key Set Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSecApsLinkKeySetRsp(ICall_ServiceEnum src,
                                     ICall_EntityID    dest,
                                     const void *      msg)
{
    zstackmsg_secApsLinkKeySetReq_t *pMsg =
        (zstackmsg_secApsLinkKeySetReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_SEC_APS_LINKKEY_SET_REQ) ?
            true : false );
}

/**
 * Compare a received a Security APS Link Key Remove Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSecApsLinkKeyRemoveRsp(ICall_ServiceEnum src,
                                        ICall_EntityID    dest,
                                        const void *      msg)
{
    zstackmsg_secApsLinkKeyRemoveReq_t *pMsg =
        (zstackmsg_secApsLinkKeyRemoveReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_SEC_APS_LINKKEY_REMOVE_REQ) ?
            true : false );
}

/**
 * Compare a received a Security APS Remove Request Response message
 * for a match.
 *
 * @param src   originator of the message as a service enumeration
 * @param dest  destination entity id of the message
 * @param msg   pointer to the message body
 *
 * @return TRUE when the message matches. FALSE, otherwise.
 */
static bool matchSecApsRemoveRsp(ICall_ServiceEnum src, ICall_EntityID dest,
                                 const void *msg)
{
    zstackmsg_secApsRemoveReq_t *pMsg = (zstackmsg_secApsRemoveReq_t *)msg;

    // Is this the correct response message
    return( (pMsg->hdr.event == zstackmsg_CmdIDs_SEC_APS_REMOVE_REQ) ?
            true : false );
}

//*****************************************************************************
// Public Functions
//*****************************************************************************

/**
 * Call to send a System Reset Request, through the iCall dispatcher to the
 * ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_sysResetReq(ICall_EntityID        appEntity,
                                           zstack_sysResetReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_SYS_RESET_REQ, pReq,
                              sizeof(zstackmsg_sysResetReq_t),
                              matchSysResetRsp) );
}

/**
 * Call to send a System Version Request, through the iCall dispatcher to the
 * ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_sysVersionReq(ICall_EntityID          appEntity,
                                             zstack_sysVersionRsp_t *pRsp)
{
    zstack_ZStatusValues status = zstack_ZStatusValues_ZMemError;
    ICall_Errno errno;

    // Allocate message buffer space
    zstackmsg_sysVersionReq_t *pMsg =
        (zstackmsg_sysVersionReq_t *)ICall_allocMsg( sizeof(*pMsg) );

    // Was the message allocated?
    if(pMsg != NULL)
    {
        /* Fill in the message content */
        pMsg->hdr.event = zstackmsg_CmdIDs_SYS_VERSION_REQ;
        pMsg->hdr.status = 0;

        /*
         * Set the pointer for the response structure.
         * The caller allocated space for the response
         */
        pMsg->pRsp = pRsp;

        // Send the message
        errno = ICall_sendServiceMsg(appEntity, ICALL_SERVICE_CLASS_ZSTACK,
                                     ICALL_MSG_FORMAT_KEEP, pMsg);

        // Was the message sent successfully
        if(errno == ICALL_ERRNO_SUCCESS)
        {
            zstackmsg_sysVersionReq_t *pCmdStatus = NULL;

            // Wait for the response message
            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER, matchSysVersionRsp,
                                    NULL, NULL, (void **)&pCmdStatus);

            // Was the response message recieved?
            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (zstack_ZStatusValues)pMsg->hdr.status;
            }
            else
            {
                // Translate ICall Error status to ZStack API status
                if(errno == ICALL_ERRNO_NOMSG)
                {
                    status = zstack_ZStatusValues_ZIcallNoMsg;
                }
                else if(errno == ICALL_ERRNO_TIMEOUT)
                {
                    status = zstack_ZStatusValues_ZIcallTimeout;
                }
                else
                {
                    status = zstack_ZStatusValues_ZFailure;
                }
            }
        }

        /* pCmdStatus is the same as pMsg */
        ICall_freeMsg(pMsg);
    }
    return(status);
}

/**
 * Call to send a System Version Request, through the iCall dispatcher to the
 * ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_sysConfigReadReq(
    ICall_EntityID appEntity, zstack_sysConfigReadReq_t *
    pReq, zstack_sysConfigReadRsp_t *pRsp)
{
    // Build and send the message, then wait of the response message
    return( sendReqRsp(appEntity, zstackmsg_CmdIDs_SYS_CONFIG_READ_REQ, pReq,
                       pRsp, sizeof(zstackmsg_sysConfigReadReq_t),
                       matchSysConfigReadRsp) );
}

/**
 * Call to send a System Reset Request, through the iCall dispatcher to the
 * ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_sysConfigWriteReq(
    ICall_EntityID appEntity, zstack_sysConfigWriteReq_t *
    pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_SYS_CONFIG_WRITE_REQ,
                              pReq, sizeof(zstackmsg_sysConfigWriteReq_t),
                              matchSysConfigWriteRsp) );
}

/**
 * Call to send a System Set TX Power Request, through the iCall dispatcher to
 * the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_sysSetTxPowerReq(
    ICall_EntityID appEntity, zstack_sysSetTxPowerReq_t *
    pReq, zstack_sysSetTxPowerRsp_t *pRsp)
{
    // Build and send the message, then wait of the response message
    return( sendReqRsp(appEntity, zstackmsg_CmdIDs_SYS_SET_TX_POWER_REQ, pReq,
                       pRsp, sizeof(zstackmsg_sysSetTxPowerReq_t),
                       matchSysSetTxPowerRsp) );
}

/**
 * Call to send a System Force Link Status Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_sysForceLinkStatusReq(ICall_EntityID appEntity)
{
    zstack_ZStatusValues status = zstack_ZStatusValues_ZMemError;
    ICall_Errno errno;

    // Allocate message buffer space
    zstackmsg_sysForceLinkStatusReq_t *pMsg =
        (zstackmsg_sysForceLinkStatusReq_t *)
        ICall_allocMsg( sizeof(*pMsg) );

    // Was the allocation successful?
    if(pMsg != NULL)
    {
        // Fill in the message content
        pMsg->hdr.event = zstackmsg_CmdIDs_SYS_FORCE_LINK_STATUS_REQ;
        pMsg->hdr.status = 0;

        // Send the message
        errno = ICall_sendServiceMsg(appEntity, ICALL_SERVICE_CLASS_ZSTACK,
                                     ICALL_MSG_FORMAT_KEEP, pMsg);

        // Was the message sent successful?
        if(errno == ICALL_ERRNO_SUCCESS)
        {
            zstackmsg_sysForceLinkStatusReq_t *pCmdStatus = NULL;

            // Wait for the response message
            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER,
                                    matchSysForceLinkStatusRsp, NULL, NULL,
                                    (void **)&pCmdStatus);

            // Was the response message received?
            if(errno == ICALL_ERRNO_SUCCESS)
            {
                // Transfer status to the return value
                status = (zstack_ZStatusValues)pMsg->hdr.status;
            }
            else
            {
                // Translate ICall Error status to ZStack API status
                if(errno == ICALL_ERRNO_NOMSG)
                {
                    status = zstack_ZStatusValues_ZIcallNoMsg;
                }
                else if(errno == ICALL_ERRNO_TIMEOUT)
                {
                    status = zstack_ZStatusValues_ZIcallTimeout;
                }
                else
                {
                    status = zstack_ZStatusValues_ZFailure;
                }
            }
        }

        /* pCmdStatus is the same as pMsg */
        ICall_freeMsg(pMsg);
    }

    return(status);
}

/**
 * Call to send a System Network Info Read Request, through the iCall dispatcher
 * to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_sysNwkInfoReadReq(
    ICall_EntityID appEntity, zstack_sysNwkInfoReadRsp_t *
    pRsp)
{
    zstack_ZStatusValues status = zstack_ZStatusValues_ZMemError;
    ICall_Errno errno;

    // Allocate message buffer space
    zstackmsg_sysNwkInfoReadReq_t *pMsg =
        (zstackmsg_sysNwkInfoReadReq_t *)ICall_allocMsg( sizeof(*pMsg) );

    if(pMsg != NULL)
    {
        // Fill in the message content
        pMsg->hdr.event = zstackmsg_CmdIDs_SYS_NWK_INFO_READ_REQ;
        pMsg->hdr.status = 0;

        /*
         * Set the pointer for the response structure.
         * The caller allocated space for the response
         */
        pMsg->pRsp = pRsp;

        // Send the message
        errno = ICall_sendServiceMsg(appEntity, ICALL_SERVICE_CLASS_ZSTACK,
                                     ICALL_MSG_FORMAT_KEEP, pMsg);

        // Was the message sent successfully?
        if(errno == ICALL_ERRNO_SUCCESS)
        {
            zstackmsg_sysNwkInfoReadReq_t *pCmdStatus = NULL;

            // Wait for the response message
            errno = ICall_waitMatch(ICALL_TIMEOUT_FOREVER, matchSysNwkInfoRsp,
                                    NULL, NULL, (void **)&pCmdStatus);

            // Was the response message received successfully?
            if(errno == ICALL_ERRNO_SUCCESS)
            {
                status = (zstack_ZStatusValues)pMsg->hdr.status;
            }
            else
            {
                // Translate ICall Error status to ZStack API status
                if(errno == ICALL_ERRNO_NOMSG)
                {
                    status = zstack_ZStatusValues_ZIcallNoMsg;
                }
                else if(errno == ICALL_ERRNO_TIMEOUT)
                {
                    status = zstack_ZStatusValues_ZIcallTimeout;
                }
                else
                {
                    status = zstack_ZStatusValues_ZFailure;
                }
            }
        }

        /* pCmdStatus is the same as pMsg */
        ICall_freeMsg(pMsg);
    }

    return(status);
}

/**
 * Call to send a Device Start Request, through the iCall dispatcher to the
 * ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_DevStartReq(ICall_EntityID        appEntity,
                                           zstack_devStartReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_DEV_START_REQ, pReq,
                              sizeof(zstackmsg_devStartReq_t),
                              matchDevStartRsp) );
}

/**
 * Call to send a Device Network Discovery Request, through the iCall dispatcher
 * to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_DevNwkDiscReq(ICall_EntityID          appEntity,
                                             zstack_devNwkDiscReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp
                (appEntity, zstackmsg_CmdIDs_DEV_NWK_DISCOVERY_REQ, pReq,
                sizeof(zstackmsg_devNwkDiscReq_t), matchDevNwkDiscRsp) );
}

/**
 * Call to send a Device Join Request, through the iCall dispatcher to the
 * ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_DevJoinReq(ICall_EntityID       appEntity,
                                          zstack_devJoinReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_DEV_JOIN_REQ, pReq,
                              sizeof(zstackmsg_devJoinReq_t), matchDevJoinRsp) );
}

/**
 * Call to send a Device Rejoin Request, through the iCall dispatcher to the
 * ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_DevRejoinReq(ICall_EntityID         appEntity,
                                            zstack_devRejoinReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_DEV_REJOIN_REQ, pReq,
                              sizeof(zstackmsg_devRejoinReq_t),
                              matchDevRejoinRsp) );
}

/**
 * Call to send a Device ZDO Callback Request, through the iCall dispatcher
 * to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_DevZDOCBReq(ICall_EntityID        appEntity,
                                           zstack_devZDOCBReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp
                (appEntity, zstackmsg_CmdIDs_DEV_ZDO_CBS_REQ, pReq,
                sizeof(zstackmsg_devZDOCBReq_t), matchDevZDOCBRsp) );
}

/**
 * Call to send a Device Network Route Request, through the iCall dispatcher
 * to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_DevNwkRouteReq(
    ICall_EntityID appEntity, zstack_devNwkRouteReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_DEV_NWK_ROUTE_REQ,
                              pReq, sizeof(zstackmsg_devNwkRouteReq_t),
                              matchDevNwkRouteRsp) );
}

/**
 * Call to send a Device Network Check Route Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_DevNwkCheckRouteReq(
    ICall_EntityID appEntity, zstack_devNwkCheckRouteReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_DEV_NWK_CHECK_ROUTE_REQ, pReq,
                              sizeof(zstackmsg_devNwkCheckRouteReq_t),
                              matchDevNwkCheckRouteRsp) );
}

/**
 * Call to send a Device Update Neighbor's TxCost Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_DevUpdateNeighborTxCostReq(
    ICall_EntityID
    appEntity, zstack_devUpdateNeighborTxCostReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_DEV_UPDATE_NEIGHBOR_TXCOST_REQ,
                              pReq,
                              sizeof(zstackmsg_devUpdateNeighborTxCostReq_t),
                              matchDevUpdateNeighborTxCostRsp) );
}

/**
 * Call to send a Device Force Network Settings Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_DevForceNetworkSettingsReq(
    ICall_EntityID
    appEntity, zstack_devForceNetworkSettingsReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_DEV_FORCE_NETWORK_SETTINGS_REQ,
                              pReq,
                              sizeof(zstackmsg_devForceNetworkSettingsReq_t),
                              matchDevForceNetworkSettingsRsp) );
}

/**
 * Call to send a Device Force Network Update Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_DevForceNetworkUpdateReq(
    ICall_EntityID
    appEntity, zstack_devForceNetworkUpdateReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_DEV_FORCE_NETWORK_UPDATE_REQ,
                              pReq,
                              sizeof(zstackmsg_devForceNetworkUpdateReq_t),
                              matchDevForceNetworkUpdateRsp) );
}

/**
 * Call to send a Device Force MAC Params Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_DevForceMacParamsReq(
    ICall_EntityID appEntity, zstack_devForceMacParamsReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_DEV_FORCE_MAC_PARAMS_REQ, pReq,
                              sizeof(zstackmsg_devForceMacParamsReq_t),
                              matchDevForceMacParamsRsp) );
}

/**
 * Call to send an APS Remove Group Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ApsRemoveGroupReq(
    ICall_EntityID appEntity, zstack_apsRemoveGroup_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_APS_REMOVE_GROUP,
                              pReq, sizeof(zstackmsg_apsRemoveGroup_t),
                              matchApsRemoveGroupRsp) );
}

/**
 * Call to send an APS Remove All Groups Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ApsRemoveAllGroupsReq(
    ICall_EntityID appEntity, zstack_apsRemoveAllGroups_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp
                (appEntity, zstackmsg_CmdIDs_APS_REMOVE_ALL_GROUPS, pReq,
                sizeof(zstackmsg_apsRemoveAllGroups_t),
                matchApsRemoveAllGroupsRsp) );
}

/**
 * Call to send an APS Find All Groups Request, through the iCall dispatcher to
 * the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ApsFindAllGroupsReq(
    ICall_EntityID appEntity, zstack_apsFindAllGroupsReq_t
    *pReq, zstack_apsFindAllGroupsRsp_t *pRsp)
{
    // Build and send the message, then wait of the response message
    return( sendReqRsp(appEntity, zstackmsg_CmdIDs_APS_FIND_ALL_GROUPS_REQ,
                       pReq, pRsp, sizeof(zstackmsg_apsFindAllGroupsReq_t),
                       matchApsFindAllGroupsRsp) );
}

/**
 * Call to send an APS Find Group Request, through the iCall dispatcher to
 * the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ApsFindGroupReq(
    ICall_EntityID appEntity, zstack_apsFindGroupReq_t *pReq,
    zstack_apsFindGroupRsp_t *pRsp)
{
    // Build and send the message, then wait of the response message
    return( sendReqRsp(appEntity, zstackmsg_CmdIDs_APS_FIND_GROUP_REQ, pReq,
                       pRsp, sizeof(zstackmsg_apsFindGroupReq_t),
                       matchApsFindGroupRsp) );
}

/**
 * Call to send an APS Add Group Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ApsAddGroupReq(ICall_EntityID        appEntity,
                                              zstack_apsAddGroup_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_APS_ADD_GROUP, pReq,
                              sizeof(zstackmsg_apsAddGroup_t),
                              matchApsAddGroupRsp) );
}

/**
 * Call to send an APS Count All Groups Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
int Zstackapi_ApsCountAllGroupsReq(ICall_EntityID appEntity)
{
    // Build and send the message, then wait of the response message
    return( (int)( sendReqDefaultRsp(appEntity,
                                     zstackmsg_CmdIDs_APS_COUNT_ALL_GROUPS,
                                     NULL,
                                     sizeof(zstackmsg_apsCountAllGroups_t),
                                     matchApsCountAllGroupsRsp) ) );
}

/**
 * Call to send an AF Register Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_AfRegisterReq(ICall_EntityID          appEntity,
                                             zstack_afRegisterReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp
                (appEntity, zstackmsg_CmdIDs_AF_REGISTER_REQ, pReq,
                sizeof(zstackmsg_afRegisterReq_t), matchAfRegisterRsp) );
}

/**
 * Call to send an AF Unregister Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_AfUnRegisterReq(
    ICall_EntityID appEntity, zstack_afUnRegisterReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_AF_UNREGISTER_REQ,
                              pReq, sizeof(zstackmsg_afUnRegisterReq_t),
                              matchAfUnregisterRsp) );
}

/**
 * Call to send an AF Data Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_AfDataReq(ICall_EntityID      appEntity,
                                         zstack_afDataReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_AF_DATA_REQ, pReq,
                              sizeof(zstackmsg_afDataReq_t), matchAfDataRsp) );
}

/**
 * Call to send an AF InterPAN Control Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_AfInterpanCtlReq(
    ICall_EntityID appEntity, zstack_afInterPanCtlReq_t *
    pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_AF_INTERPAN_CTL_REQ,
                              pReq, sizeof(zstackmsg_afInterPanCtlReq_t),
                              matchAfInterpanCtlRsp) );
}

/**
 * Call to send an AF Config Get Request, through the iCall dispatcher to
 * the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_AfConfigGetReq(
    ICall_EntityID appEntity, zstack_afConfigGetReq_t *pReq,
    zstack_afConfigGetRsp_t *pRsp)
{
    // Build and send the message, then wait of the response message
    return( sendReqRsp(appEntity, zstackmsg_CmdIDs_AF_CONFIG_GET_REQ, pReq,
                       pRsp, sizeof(zstackmsg_afConfigGetReq_t),
                       matchAfConfigGetRsp) );
}

/**
 * Call to send an AF Config Set Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_AfConfigSetReq(
    ICall_EntityID appEntity, zstack_afConfigSetReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_AF_CONFIG_SET_REQ,
                              pReq, sizeof(zstackmsg_afConfigSetReq_t),
                              matchAfConfigSetRsp) );
}

/**
 * Call to send a ZDO Network Address Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoNwkAddrReq(ICall_EntityID          appEntity,
                                             zstack_zdoNwkAddrReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_NWK_ADDR_REQ,
                              pReq, sizeof(zstackmsg_zdoNwkAddrReq_t),
                              matchZdoNwkAddrRsp) );
}

/**
 * Call to send a ZDO IEEE Address Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoIeeeAddrReq(
    ICall_EntityID appEntity, zstack_zdoIeeeAddrReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_IEEE_ADDR_REQ,
                              pReq, sizeof(zstackmsg_zdoIeeeAddrReq_t),
                              matchZdoIeeeAddrRsp) );
}

/**
 * Call to send a ZDO Node Descriptor Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoNodeDescReq(
    ICall_EntityID appEntity, zstack_zdoNodeDescReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_NODE_DESC_REQ,
                              pReq, sizeof(zstackmsg_zdoNodeDescReq_t),
                              matchZdoNodeDescRsp) );
}

/**
 * Call to send a ZDO Power Descriptor Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoPowerDescReq(
    ICall_EntityID appEntity, zstack_zdoPowerDescReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_POWER_DESC_REQ,
                              pReq, sizeof(zstackmsg_zdoPowerDescReq_t),
                              matchZdoPowerDescRsp) );
}

/**
 * Call to send a ZDO Simple Descriptor Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoSimpleDescReq(
    ICall_EntityID appEntity, zstack_zdoSimpleDescReq_t *
    pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_REQ,
                              pReq, sizeof(zstackmsg_zdoSimpleDescReq_t),
                              matchZdoSimpleDescRsp) );
}

/**
 * Call to send a ZDO Active Endpoint Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoActiveEndpointReq(
    ICall_EntityID appEntity, zstack_zdoActiveEndpointReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_ZDO_ACTIVE_ENDPOINT_REQ, pReq,
                              sizeof(zstackmsg_zdoActiveEndpointReq_t),
                              matchZdoActiveEndpointRsp) );
}

/**
 * Call to send a ZDO Match Descriptor Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoMatchDescReq(
    ICall_EntityID appEntity, zstack_zdoMatchDescReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_MATCH_DESC_REQ,
                              pReq, sizeof(zstackmsg_zdoMatchDescReq_t),
                              matchZdoMatchDescRsp) );
}

/**
 * Call to send a ZDO Complex Descriptor Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoComplexDescReq(
    ICall_EntityID appEntity, zstack_zdoComplexDescReq_t *
    pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_COMPLEX_DESC_REQ,
                              pReq, sizeof(zstackmsg_zdoComplexDescReq_t),
                              matchZdoComplexDescRsp) );
}

/**
 * Call to send a ZDO User Descriptor Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoUserDescReq(
    ICall_EntityID appEntity, zstack_zdoUserDescReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_USER_DESC_REQ,
                              pReq, sizeof(zstackmsg_zdoUserDescReq_t),
                              matchZdoUserDescRsp) );
}

/**
 * Call to send a ZDO Device Announce Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoDeviceAnnounceReq(
    ICall_EntityID appEntity, zstack_zdoDeviceAnnounceReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE_REQ, pReq,
                              sizeof(zstackmsg_zdoDeviceAnnounceReq_t),
                              matchZdoDeviceAnnounceRsp) );
}

/**
 * Call to send a ZDO User Descriptor Set Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoUserDescSetReq(
    ICall_EntityID appEntity, zstack_zdoUserDescSetReq_t *
    pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_ZDO_USER_DESCR_SET_REQ, pReq,
                              sizeof(zstackmsg_zdoUserDescSetReq_t),
                              matchZdoUserDescSetRsp) );
}

/**
 * Call to send a ZDO Server Discovery Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoServerDiscReq(
    ICall_EntityID appEntity, zstack_zdoServerDiscReq_t *
    pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_SERVER_DISC_REQ,
                              pReq, sizeof(zstackmsg_zdoServerDiscReq_t),
                              matchZdoServerDiscRsp) );
}

/**
 * Call to send a ZDO End Device Bind Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoEndDeviceBindReq(
    ICall_EntityID appEntity, zstack_zdoEndDeviceBindReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_REQ, pReq,
                              sizeof(zstackmsg_zdoEndDeviceBindReq_t),
                              matchZdoEndDeviceBindRsp) );
}

/**
 * Call to send a ZDO Bind Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoBindReq(ICall_EntityID       appEntity,
                                          zstack_zdoBindReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_BIND_REQ, pReq,
                              sizeof(zstackmsg_zdoBindReq_t), matchZdoBindRsp) );
}

/**
 * Call to send a ZDO Unbind Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoUnbindReq(ICall_EntityID         appEntity,
                                            zstack_zdoUnbindReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_UNBIND_REQ, pReq,
                              sizeof(zstackmsg_zdoUnbindReq_t),
                              matchZdoUnbindRsp) );
}

/**
 * Call to send a ZDO Mgmt Network Discovery Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoMgmtNwkDiscReq(
    ICall_EntityID appEntity, zstack_zdoMgmtNwkDiscReq_t *
    pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp
                (appEntity, zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_REQ, pReq,
                sizeof(zstackmsg_zdoMgmtNwkDiscReq_t),
                matchZdoMgmtNwkDiscRsp) );
}

/**
 * Call to send a ZDO Mgmt LQI Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoMgmtLqiReq(ICall_EntityID          appEntity,
                                             zstack_zdoMgmtLqiReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_MGMT_LQI_REQ,
                              pReq, sizeof(zstackmsg_zdoMgmtLqiReq_t),
                              matchZdoMgmtLqiRsp) );
}

/**
 * Call to send a ZDO Mgmt Routing Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoMgmtRtgReq(ICall_EntityID          appEntity,
                                             zstack_zdoMgmtRtgReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_MGMT_RTG_REQ,
                              pReq, sizeof(zstackmsg_zdoMgmtRtgReq_t),
                              matchZdoMgmtRtgRsp) );
}

/**
 * Call to send a ZDO Mgmt Bind Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoMgmtBindReq(
    ICall_EntityID appEntity, zstack_zdoMgmtBindReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_MGMT_BIND_REQ,
                              pReq, sizeof(zstackmsg_zdoMgmtBindReq_t),
                              matchZdoMgmtBindRsp) );
}

/**
 * Call to send a ZDO Mgmt Leave Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoMgmtLeaveReq(
    ICall_EntityID appEntity, zstack_zdoMgmtLeaveReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_REQ,
                              pReq, sizeof(zstackmsg_zdoMgmtLeaveReq_t),
                              matchZdoMgmtLeaveRsp) );
}

/**
 * Call to send a ZDO Mgmt Direct Join Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoMgmtDirectJoinReq(
    ICall_EntityID appEntity, zstack_zdoMgmtDirectJoinReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_REQ, pReq,
                              sizeof(zstackmsg_zdoMgmtDirectJoinReq_t),
                              matchZdoMgmtDirectJoinRsp) );
}

/**
 * Call to send a ZDO Mgmt Permit Join Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoMgmtPermitJoinReq(
    ICall_EntityID appEntity, zstack_zdoMgmtPermitJoinReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_REQ, pReq,
                              sizeof(zstackmsg_zdoMgmtPermitJoinReq_t),
                              matchZdoMgmtPermitJoinRsp) );
}

/**
 * Call to send a ZDO Mgmt Network Update Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_ZdoMgmtNwkUpdateReq(
    ICall_EntityID appEntity, zstack_zdoMgmtNwkUpdateReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_REQ, pReq,
                              sizeof(zstackmsg_zdoMgmtNwkUpdateReq_t),
                              matchZdoMgmtNwkUpdateRsp) );
}

/**
 * Call to send a Security Network Key Get Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_secNwkKeyGetReq(
    ICall_EntityID appEntity, zstack_secNwkKeyGetReq_t *pReq,
    zstack_secNwkKeyGetRsp_t *pRsp)
{
    // Build and send the message, then wait of the response message
    return( sendReqRsp(appEntity, zstackmsg_CmdIDs_SEC_NWK_KEY_GET_REQ, pReq,
                       pRsp, sizeof(zstackmsg_secNwkKeyGetReq_t),
                       matchSecNwkKeyGetRsp) );
}

/**
 * Call to send a Security Network Key Set Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_secNwkKeySetReq(
    ICall_EntityID appEntity, zstack_secNwkKeySetReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_SEC_NWK_KEY_SET_REQ,
                              pReq, sizeof(zstackmsg_secNwkKeySetReq_t),
                              matchSecNwkKeySetRsp) );
}

/**
 * Call to send a Security Network Key Update Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_secNwkKeyUpdateReq(
    ICall_EntityID appEntity, zstack_secNwkKeyUpdateReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_SEC_NWK_KEY_UPDATE_REQ, pReq,
                              sizeof(zstackmsg_secNwkKeyUpdateReq_t),
                              matchSecNwkKeyUpdateRsp) );
}

/**
 * Call to send a Security Network Key Switch Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_secNwkKeySwitchReq(
    ICall_EntityID appEntity, zstack_secNwkKeySwitchReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_SEC_NWK_KEY_SWITCH_REQ, pReq,
                              sizeof(zstackmsg_secNwkKeySwitchReq_t),
                              matchSecNwkKeySwitchRsp) );
}

/**
 * Call to send a Security APS Link Key Get Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_secApsLinkKeyGetReq(
    ICall_EntityID appEntity, zstack_secApsLinkKeyGetReq_t
    *pReq, zstack_secApsLinkKeyGetRsp_t *pRsp)
{
    // Build and send the message, then wait of the response message
    return( sendReqRsp(appEntity, zstackmsg_CmdIDs_SEC_APS_LINKKEY_GET_REQ,
                       pReq, pRsp, sizeof(zstackmsg_secApsLinkKeyGetReq_t),
                       matchSecApsLinkKeyGetRsp) );
}

/**
 * Call to send a Security APS Link Key Set Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_secApsLinkKeySetReq(
    ICall_EntityID appEntity, zstack_secApsLinkKeySetReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_SEC_APS_LINKKEY_SET_REQ, pReq,
                              sizeof(zstackmsg_secApsLinkKeySetReq_t),
                              matchSecApsLinkKeySetRsp) );
}

/**
 * Call to send a Security APS Link Key Remove Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_secApsLinkKeyRemoveReq(
    ICall_EntityID appEntity, zstack_secApsLinkKeyRemoveReq_t
    *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity,
                              zstackmsg_CmdIDs_SEC_APS_LINKKEY_REMOVE_REQ,
                              pReq, sizeof(zstackmsg_secApsLinkKeyRemoveReq_t),
                              matchSecApsLinkKeyRemoveRsp) );
}

/**
 * Call to send a Security APS Remove Request, through the iCall
 * dispatcher to the ZStack Thread.
 *
 * Public function defined in zstackapi.h
 */
zstack_ZStatusValues Zstackapi_secApsRemoveReq(
    ICall_EntityID appEntity, zstack_secApsRemoveReq_t *pReq)
{
    // Build and send the message, then wait of the response message
    return( sendReqDefaultRsp(appEntity, zstackmsg_CmdIDs_SEC_APS_REMOVE_REQ,
                              pReq, sizeof(zstackmsg_secApsRemoveReq_t),
                              matchSecApsRemoveRsp) );
}

/**
 * Call to free the memory used by an Indication message, messages
 * sent asynchronously from the ZStack thread.
 *
 * Public function defined in zstackapi.h
 */
bool Zstackapi_freeIndMsg(void *pMsg)
{
    bool processed = true;
    GenericReqRsp_t *pTemp = (GenericReqRsp_t *)pMsg;

    // Determine the type of message to free
    switch(pTemp->hdr.event)
    {
        // ZDO Source Routing Indicaiton
        case zstackmsg_CmdIDs_ZDO_SRC_RTG_IND:
            {
                zstackmsg_zdoSrcRtgInd_t *pInd =
                    (zstackmsg_zdoSrcRtgInd_t *)pMsg;

                // Does it have any relay addresses?
                if(pInd->req.pRelay)
                {
                    // Free relay data
                    ICall_free(pInd->req.pRelay);
                }

                // Free the message
                ICall_freeMsg(pMsg);
            }
            break;

        // ZDO Network Address Response
        case zstackmsg_CmdIDs_ZDO_NWK_ADDR_RSP:
            {
                zstackmsg_zdoNwkAddrRspInd_t *pInd =
                    (zstackmsg_zdoNwkAddrRspInd_t *)pMsg;

                // Does it have an associated device list?
                if(pInd->rsp.pAssocDevList)
                {
                    // Free the associated device list
                    ICall_free(pInd->rsp.pAssocDevList);
                }

                // Free the message
                ICall_freeMsg(pMsg);
            }
            break;

        // ZDO IEEE Address Response
        case zstackmsg_CmdIDs_ZDO_IEEE_ADDR_RSP:
            {
                zstackmsg_zdoIeeeAddrRspInd_t *pInd =
                    (zstackmsg_zdoIeeeAddrRspInd_t *)pMsg;

                // Does it have an associated device list?
                if(pInd->rsp.pAssocDevList)
                {
                    // Free the associated device list
                    ICall_free(pInd->rsp.pAssocDevList);
                }

                // Free the message
                ICall_freeMsg(pMsg);
            }
            break;

        // ZDO Simple Descriptor Response
        case zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_RSP:
            {
                zstackmsg_zdoSimpleDescRspInd_t *pInd =
                    (zstackmsg_zdoSimpleDescRspInd_t *)pMsg;

                // Free an input cluster list
                if(pInd->rsp.simpleDesc.pInputClusters)
                {
                    ICall_free(pInd->rsp.simpleDesc.pInputClusters);
                }
                // Free an output cluster list
                if(pInd->rsp.simpleDesc.pOutputClusters)
                {
                    ICall_free(pInd->rsp.simpleDesc.pOutputClusters);
                }

                // Free the message
                ICall_freeMsg(pMsg);
            }
            break;

        // ZDO Active Enpoint Response
        case zstackmsg_CmdIDs_ZDO_ACTIVE_EP_RSP:
            {
                zstackmsg_zdoActiveEndpointsRspInd_t *pInd =
                    (zstackmsg_zdoActiveEndpointsRspInd_t *)pMsg;

                // Free the active endpoint list
                if(pInd->rsp.pActiveEPList)
                {
                    ICall_free(pInd->rsp.pActiveEPList);
                }

                // Free the message
                ICall_freeMsg(pMsg);
            }
            break;

        // ZDO Match Descriptor Response
        case zstackmsg_CmdIDs_ZDO_MATCH_DESC_RSP:
            {
                zstackmsg_zdoMatchDescRspInd_t *pInd =
                    (zstackmsg_zdoMatchDescRspInd_t *)pMsg;

                // Free the match descriptor's endpoint list
                if(pInd->rsp.pMatchList)
                {
                    ICall_free(pInd->rsp.pMatchList);
                }

                // Free the message
                ICall_freeMsg(pMsg);
            }
            break;

        // ZDO User Descriptor Response
        case zstackmsg_CmdIDs_ZDO_USER_DESC_RSP:
            {
                zstackmsg_zdoUserDescRspInd_t *pInd =
                    (zstackmsg_zdoUserDescRspInd_t *)pMsg;

                // Free the descriptor
                if(pInd->rsp.pDesc)
                {
                    ICall_free(pInd->rsp.pDesc);
                }

                // Free the message
                ICall_freeMsg(pMsg);
            }
            break;

        // ZDO Mgmt Network Discovery Response
        case zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_RSP:
            {
                zstackmsg_zdoMgmtNwkDiscRspInd_t *pInd =
                    (zstackmsg_zdoMgmtNwkDiscRspInd_t *)pMsg;

                // Free the network list
                if(pInd->rsp.pNetList)
                {
                    ICall_free(pInd->rsp.pNetList);
                }

                // Free the message
                ICall_freeMsg(pMsg);
            }
            break;

        // ZDO Mgmt LQI Response
        case zstackmsg_CmdIDs_ZDO_MGMT_LQI_RSP:
            {
                zstackmsg_zdoMgmtLqiRspInd_t *pInd =
                    (zstackmsg_zdoMgmtLqiRspInd_t *)pMsg;

                // Free the LQI list
                if(pInd->rsp.pLqiList)
                {
                    ICall_free(pInd->rsp.pLqiList);
                }

                // Free the message
                ICall_freeMsg(pMsg);
            }
            break;

        // ZDO Mgmt Routing Response
        case zstackmsg_CmdIDs_ZDO_MGMT_RTG_RSP:
            {
                zstackmsg_zdoMgmtRtgRspInd_t *pInd =
                    (zstackmsg_zdoMgmtRtgRspInd_t *)pMsg;

                // Free the routing list
                if(pInd->rsp.pRtgList)
                {
                    ICall_free(pInd->rsp.pRtgList);
                }

                // Free the message
                ICall_freeMsg(pMsg);
            }
            break;

        // ZDO Mgmt Binding Response
        case zstackmsg_CmdIDs_ZDO_MGMT_BIND_RSP:
            {
                zstackmsg_zdoMgmtBindRspInd_t *pInd =
                    (zstackmsg_zdoMgmtBindRspInd_t *)pMsg;

                // Free the binding list
                if(pInd->rsp.pBindList)
                {
                    ICall_free(pInd->rsp.pBindList);
                }

                // Free the message
                ICall_freeMsg(pMsg);
            }
            break;

        // ZDO Mgmt Network Update Notifiy Indication
        case zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_NOTIFY:
            {
                zstackmsg_zdoMgmtNwkUpdateNotifyInd_t *pInd =
                    (zstackmsg_zdoMgmtNwkUpdateNotifyInd_t *)pMsg;

                // Free the Energy List
                if(pInd->rsp.pEnergyValuesList)
                {
                    ICall_free(pInd->rsp.pEnergyValuesList);
                }

                // Free the message
                ICall_freeMsg(pMsg);
            }
            break;

        // AF Incoming Message Indication
        case zstackmsg_CmdIDs_AF_INCOMING_MSG_IND:
            {
                zstackmsg_afIncomingMsgInd_t *pInd =
                    (zstackmsg_afIncomingMsgInd_t *)pMsg;

                // Free the message payload
                if(pInd->req.pPayload)
                {
                    ICall_free(pInd->req.pPayload);
                }

                // Free the message
                ICall_freeMsg(pMsg);
            }
            break;

        // All other messages
        case zstackmsg_CmdIDs_ZDO_CONCENTRATOR_IND:
        case zstackmsg_CmdIDs_ZDO_JOIN_CNF:
        case zstackmsg_CmdIDs_ZDO_LEAVE_IND:
        case zstackmsg_CmdIDs_DEV_PERMIT_JOIN_IND:
        case zstackmsg_CmdIDs_ZDO_TC_DEVICE_IND:
        case zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE:
        case zstackmsg_CmdIDs_ZDO_NODE_DESC_RSP:
        case zstackmsg_CmdIDs_ZDO_POWER_DESC_RSP:
        case zstackmsg_CmdIDs_ZDO_SERVER_DISC_RSP:
        case zstackmsg_CmdIDs_ZDO_BIND_RSP:
        case zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_RSP:
        case zstackmsg_CmdIDs_ZDO_UNBIND_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_RSP:
        case zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_RSP:
        case zstackmsg_CmdIDs_AF_DATA_CONFIRM_IND:
            // Free the message
            ICall_freeMsg(pMsg);
            break;

        default:               // Ignore the other messages
            processed = false;
            break;
    }

    return(processed);
}
