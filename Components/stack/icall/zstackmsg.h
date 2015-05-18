/**
   @file  zstackmsg.h
   $Date: 2015-02-18 09:35:16 -0800 (Wed, 18 Feb 2015) $
   $Revision: 42724 $

   @brief ZStack API messages

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
#ifndef ZSTACKMSG_H
#define ZSTACKMSG_H

#include <stdbool.h>
#include <stdint.h>

#include "zstack.h"

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
// Constants and definitions
//*****************************************************************************

/**
 * Command IDs - each of these are associated with Request,
 * Responses, Indications, and Confirm messages
 */
typedef enum
{
    zstackmsg_CmdIDs_SYS_RESET_REQ = 0x00,
    zstackmsg_CmdIDs_SYS_VERSION_REQ = 0x02,
    zstackmsg_CmdIDs_SYS_CONFIG_READ_REQ = 0x03,
    zstackmsg_CmdIDs_SYS_CONFIG_WRITE_REQ = 0x04,
    zstackmsg_CmdIDs_SYS_SET_TX_POWER_REQ = 0x07,
    zstackmsg_CmdIDs_SYS_NWK_INFO_READ_REQ = 0x08,
    zstackmsg_CmdIDs_DEV_START_REQ = 0x10,
    zstackmsg_CmdIDs_DEV_NWK_DISCOVERY_REQ = 0x11,
    zstackmsg_CmdIDs_DEV_JOIN_REQ = 0x12,
    zstackmsg_CmdIDs_DEV_REJOIN_REQ = 0x13,
    zstackmsg_CmdIDs_DEV_ZDO_CBS_REQ = 0x14,
    zstackmsg_CmdIDs_DEV_NWK_ROUTE_REQ = 0x15,
    zstackmsg_CmdIDs_DEV_NWK_CHECK_ROUTE_REQ = 0x16,
    zstackmsg_CmdIDs_DEV_JAMMER_IND = 0x17,
    zstackmsg_CmdIDs_APS_REMOVE_GROUP = 0x18,
    zstackmsg_CmdIDs_APS_REMOVE_ALL_GROUPS = 0x19,
    zstackmsg_CmdIDs_APS_FIND_ALL_GROUPS_REQ = 0x1B,
    zstackmsg_CmdIDs_APS_FIND_GROUP_REQ = 0x1C,
    zstackmsg_CmdIDs_APS_ADD_GROUP = 0x1E,
    zstackmsg_CmdIDs_APS_COUNT_ALL_GROUPS = 0x1F,
    zstackmsg_CmdIDs_AF_REGISTER_REQ = 0x20,
    zstackmsg_CmdIDs_AF_UNREGISTER_REQ = 0x21,
    zstackmsg_CmdIDs_AF_DATA_REQ = 0x22,
    zstackmsg_CmdIDs_AF_INTERPAN_CTL_REQ = 0x23,
    zstackmsg_CmdIDs_AF_CONFIG_GET_REQ = 0x24,
    zstackmsg_CmdIDs_AF_CONFIG_SET_REQ = 0x25,
    zstackmsg_CmdIDs_ZDO_NWK_ADDR_REQ = 0x26,
    zstackmsg_CmdIDs_ZDO_IEEE_ADDR_REQ = 0x27,
    zstackmsg_CmdIDs_ZDO_NODE_DESC_REQ = 0x28,
    zstackmsg_CmdIDs_ZDO_POWER_DESC_REQ = 0x29,
    zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_REQ = 0x2A,
    zstackmsg_CmdIDs_ZDO_ACTIVE_ENDPOINT_REQ = 0x2B,
    zstackmsg_CmdIDs_ZDO_MATCH_DESC_REQ = 0x2C,
    zstackmsg_CmdIDs_ZDO_COMPLEX_DESC_REQ = 0x38,
    zstackmsg_CmdIDs_ZDO_SERVER_DISC_REQ = 0x39,
    zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_REQ = 0x3A,
    zstackmsg_CmdIDs_ZDO_BIND_REQ = 0x3B,
    zstackmsg_CmdIDs_ZDO_UNBIND_REQ = 0x3C,
    zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_REQ = 0x3D,
    zstackmsg_CmdIDs_ZDO_MGMT_LQI_REQ = 0x3E,
    zstackmsg_CmdIDs_ZDO_MGMT_RTG_REQ = 0x3F,
    zstackmsg_CmdIDs_ZDO_MGMT_BIND_REQ = 0x40,
    zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_REQ = 0x41,
    zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_REQ = 0x42,
    zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_REQ = 0x43,
    zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_REQ = 0x44,
    zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE_REQ = 0x45,
    zstackmsg_CmdIDs_ZDO_USER_DESCR_SET_REQ = 0x46,
    zstackmsg_CmdIDs_ZDO_USER_DESC_REQ = 0x47,
    zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE = 0x48,
    zstackmsg_CmdIDs_ZDO_NWK_ADDR_RSP = 0x60,
    zstackmsg_CmdIDs_ZDO_IEEE_ADDR_RSP = 0x61,
    zstackmsg_CmdIDs_ZDO_NODE_DESC_RSP = 0x62,
    zstackmsg_CmdIDs_ZDO_POWER_DESC_RSP = 0x63,
    zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_RSP = 0x64,
    zstackmsg_CmdIDs_ZDO_ACTIVE_EP_RSP = 0x65,
    zstackmsg_CmdIDs_ZDO_MATCH_DESC_RSP = 0x66,
    zstackmsg_CmdIDs_ZDO_COMPLEX_DESC_RSP = 0x67,
    zstackmsg_CmdIDs_ZDO_USER_DESC_RSP = 0x68,
    zstackmsg_CmdIDs_ZDO_USER_DESC_SET_RSP = 0x69,
    zstackmsg_CmdIDs_ZDO_SERVER_DISC_RSP = 0x6A,
    zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_RSP = 0x6B,
    zstackmsg_CmdIDs_ZDO_BIND_RSP = 0x6C,
    zstackmsg_CmdIDs_ZDO_UNBIND_RSP = 0x6D,
    zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_RSP = 0x6E,
    zstackmsg_CmdIDs_ZDO_MGMT_LQI_RSP = 0x6F,
    zstackmsg_CmdIDs_ZDO_MGMT_RTG_RSP = 0x70,
    zstackmsg_CmdIDs_ZDO_MGMT_BIND_RSP = 0x71,
    zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_RSP = 0x72,
    zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_RSP = 0x73,
    zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_RSP = 0x74,
    zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_NOTIFY = 0x75,
    zstackmsg_CmdIDs_ZDO_SRC_RTG_IND = 0x80,
    zstackmsg_CmdIDs_ZDO_CONCENTRATOR_IND = 0x81,
    zstackmsg_CmdIDs_ZDO_NWK_DISC_CNF = 0x82,
    zstackmsg_CmdIDs_ZDO_BEACON_NOTIFY_IND = 0x83,
    zstackmsg_CmdIDs_ZDO_JOIN_CNF = 0x84,
    zstackmsg_CmdIDs_ZDO_LEAVE_CNF = 0x85,
    zstackmsg_CmdIDs_ZDO_LEAVE_IND = 0x86,
    zstackmsg_CmdIDs_SYS_RESET_IND = 0x90,
    zstackmsg_CmdIDs_AF_DATA_CONFIRM_IND = 0x91,
    zstackmsg_CmdIDs_AF_INCOMING_MSG_IND = 0x92,
    zstackmsg_CmdIDs_AF_REFLECT_ERROR_IND = 0x93,
    zstackmsg_CmdIDs_DEV_STATE_CHANGE_IND = 0x94,
    zstackmsg_CmdIDs_ZDO_TC_DEVICE_IND = 0x95,
    zstackmsg_CmdIDs_DEV_PERMIT_JOIN_IND = 0x96,
    zstackmsg_CmdIDs_SEC_NWK_KEY_GET_REQ = 0xA0,
    zstackmsg_CmdIDs_SEC_NWK_KEY_SET_REQ = 0xA1,
    zstackmsg_CmdIDs_SEC_NWK_KEY_UPDATE_REQ = 0xA2,
    zstackmsg_CmdIDs_SEC_NWK_KEY_SWITCH_REQ = 0xA3,
    zstackmsg_CmdIDs_SEC_APS_LINKKEY_GET_REQ = 0xA4,
    zstackmsg_CmdIDs_SEC_APS_LINKKEY_SET_REQ = 0xA5,
    zstackmsg_CmdIDs_SEC_APS_LINKKEY_REMOVE_REQ = 0xA6,
    zstackmsg_CmdIDs_SEC_APS_REMOVE_REQ = 0xA7,
    zstackmsg_CmdIDs_SYS_FORCE_LINK_STATUS_REQ = 0xA8,
    zstackmsg_CmdIDs_DEV_UPDATE_NEIGHBOR_TXCOST_REQ = 0xA9,
    zstackmsg_CmdIDs_DEV_FORCE_NETWORK_SETTINGS_REQ = 0xAA,
    zstackmsg_CmdIDs_DEV_FORCE_NETWORK_UPDATE_REQ = 0xAB,
    zstackmsg_CmdIDs_DEV_FORCE_MAC_PARAMS_REQ = 0xAC,
    zstackmsg_CmdIDs_RESERVED_1A = 0x1A,
    zstackmsg_CmdIDs_RESERVED_31 = 0x31,
    zstackmsg_CmdIDs_RESERVED_32 = 0x32,
    zstackmsg_CmdIDs_RESERVED_33 = 0x33,
    zstackmsg_CmdIDs_RESERVED_34 = 0x34,
    zstackmsg_CmdIDs_RESERVED_35 = 0x35,
    zstackmsg_CmdIDs_RESERVED_36 = 0x36,
    zstackmsg_CmdIDs_RESERVED_D0 = 0xD0,
    zstackmsg_CmdIDs_RESERVED_D1 = 0xD1,
    zstackmsg_CmdIDs_RESERVED_D2 = 0xD2,
    zstackmsg_CmdIDs_RESERVED_D3 = 0xD3,
    zstackmsg_CmdIDs_RESERVED_D4 = 0xD4,
    zstackmsg_CmdIDs_RESERVED_D5 = 0xD5,
    zstackmsg_CmdIDs_RESERVED_D6 = 0xD6,
    zstackmsg_CmdIDs_RESERVED_FD = 0xFD,
    zstackmsg_CmdIDs_RESERVED_FE = 0xFE,
} zstack_CmdIDs;

//*****************************************************************************
// Structures - Building blocks for the iCall messages
//*****************************************************************************

/**
 * Event message header.
 * This is to support ICALL_CMD_EVENT
 */
typedef struct _zstackmsg_HDR_t
{
    /** event */
    uint8_t event;
    /** Will hold the default response status field. */
    uint8_t status;
} zstackmsg_HDR_t;

//*****************************************************************************
// System Interface Messages
//*****************************************************************************

/**
 * Generic structure to use receive an unknown message.
 */
typedef struct _zstackmsg_genericreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    void *pReq;

} zstackmsg_genericReq_t;

/**
 * Send this message to Reset the ZStack Thread.  The command ID for this
 * message is zstackmsg_CmdIDs_SYS_RESET_REQ.
 */
typedef struct _zstackmsg_sysresetreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_sysResetReq_t *pReq;

} zstackmsg_sysResetReq_t;

/**
 * Send this message to request the version information from the ZStack Thread.
 * The command ID for this message is zstackmsg_CmdIDs_SYS_VERSION_REQ.
 */
typedef struct _zstackmsg_sysversionreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /* Message command fields - no fields for this request */
    // void *pReq;
    /** Response fields (immediate response) */
    zstack_sysVersionRsp_t *pRsp;

} zstackmsg_sysVersionReq_t;

/**
 * Send this message to request the configuration information from the
 * ZStack Thread.  Each bool item is individually selected.
 * The command ID for this message is zstackmsg_CmdIDs_SYS_CONFIG_READ_REQ.
 */
typedef struct _zstackmsg_sysconfigreadreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_sysConfigReadReq_t *pReq;

    /**
     * Response fields (immediate response) -
     * The pUserDesc will be allocated if the has_userDesc == true and
     * n_userDesc is > 0, so the application must deallocate it with
     * ICall_free(pRsp->pUserDesc).<br>
     * The pPreConfigKey will be allocated if the has_preConfigKey == true,
     * so the application must deallocate it with ICall_free(pRsp->pPreConfigKey).
     */
    zstack_sysConfigReadRsp_t *pRsp;

} zstackmsg_sysConfigReadReq_t;

/**
 * Send this message to write configuration information to the ZStack Thread.
 * Since all of the items are optional, you can write one or many
 * configuration items.
 * The command ID for this message is zstackmsg_CmdIDs_SYS_CONFIG_WRITE_REQ.
 */
typedef struct _zstackmsg_sysconfigwritereq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_sysConfigWriteReq_t *pReq;

} zstackmsg_sysConfigWriteReq_t;

/**
 * Send this message to set the TX Power on the ZStack Thread.
 * The command ID for this message is zstackmsg_CmdIDs_SYS_SET_TX_POWER_REQ.
 */
typedef struct _zstackmsg_syssettxpowerreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_sysSetTxPowerReq_t *pReq;

    /** Response fields (immediate response) */
    zstack_sysSetTxPowerRsp_t *pRsp;

} zstackmsg_sysSetTxPowerReq_t;

/**
 * Send this message to request the Network Information the ZStack Thread.
 * The command ID for this message is zstackmsg_CmdIDs_SYS_NWK_INFO_READ_REQ.
 */
typedef struct _zstackmsg_sysnwkinforeadreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /* Message command fields - no fields for this request */
    // zstack_sysNwkInfoReadReq_t *pReq;
    /** Response fields (immediate response) */
    zstack_sysNwkInfoReadRsp_t *pRsp;

} zstackmsg_sysNwkInfoReadReq_t;

/**
 * Send this message to request the version information from the ZStack Thread.
 * The command ID for this message is zstackmsg_CmdIDs_SYS_FORCE_LINK_STATUS_REQ.
 */
typedef struct _zstackmsg_sysforcelinkstatusreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /* Message command fields - no fields for this request */
    // void *pReq;
} zstackmsg_sysForceLinkStatusReq_t;

//*****************************************************************************
// Device Interface Messages
//*****************************************************************************

/**
 * Send this message to the ZStack Thread to request it to start.
 * This message should only be used if the ZStack Thread is compiled with
 * HOLD_AUTO_START.
 * The command ID for this message is zstackmsg_CmdIDs_DEV_START_REQ.
 */
typedef struct _zstackmsg_devstartreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_devStartReq_t *pReq;

} zstackmsg_devStartReq_t;

/**
 * Send this message to the ZStack Thread to initiate a network discovery.
 * The command ID for this message is zstackmsg_CmdIDs_DEV_NWK_DISCOVERY_REQ.
 */
typedef struct _zstackmsg_devnwkdiscreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_devNwkDiscReq_t *pReq;

} zstackmsg_devNwkDiscReq_t;

/**
 * Send this message to the ZStack Thread to initiate a network join.
 * The command ID for this message is zstackmsg_CmdIDs_DEV_JOIN_REQ.
 */
typedef struct _zstackmsg_devjoinreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_devJoinReq_t *pReq;

} zstackmsg_devJoinReq_t;

/**
 * Send this message to the ZStack Thread to initiate a network rejoin.
 * The command ID for this message is zstackmsg_CmdIDs_DEV_REJOIN_REQ.
 */
typedef struct _zstackmsg_devrejoinreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_devRejoinReq_t *pReq;

} zstackmsg_devRejoinReq_t;

/**
 * Send this message to the ZStack Thread to subscribe to one or more ZDO
 * callback indications, confirmations, or response messages.
 * The command ID for this message is zstackmsg_CmdIDs_DEV_ZDO_CBS_REQ.
 */
typedef struct _zstackmsg_devzdocbreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_devZDOCBReq_t *pReq;

} zstackmsg_devZDOCBReq_t;

/**
 * Send this message to the ZStack Thread to initiate a route request.
 * The command ID for this message is zstackmsg_CmdIDs_DEV_NWK_ROUTE_REQ.
 */
typedef struct _zstackmsg_devnwkroutereq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_devNwkRouteReq_t *pReq;

} zstackmsg_devNwkRouteReq_t;

/**
 * Send this message to the ZStack Thread to check a route request.
 * The command ID for this message is zstackmsg_CmdIDs_DEV_NWK_CHECK_ROUTE_REQ.
 */
typedef struct _zstackmsg_devnwkcheckroutereq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_devNwkCheckRouteReq_t *pReq;

} zstackmsg_devNwkCheckRouteReq_t;

/**
 * This message is sent from ZStack Thread to indication change in
 * jammer detection.
 * The command ID for this message is zstackmsg_CmdIDs_DEV_JAMMER_IND.
 */
typedef struct _zstackmsg_devjammerind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_devJammerInd_t req;

} zstackmsg_devJammerInd_t;

/**
 * This message is sent from ZStack Thread whenever the Permit Join state
 * changes (Off to on, on to off).
 * The command ID for this message is zstackmsg_CmdIDs_DEV_PERMIT_JOIN_IND.
 */
typedef struct _zstackmsg_devpermitjoinind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_devPermitJoinInd_t req;

} zstackmsg_devPermitJoinInd_t;

/**
 * Send this message to the ZStack Thread to Update a neighbor's txCost. If the
 * neighbor exists (in the neighbor table), it will update the cost, if it doesn't
 * exist, one will be created.  This function will only exist on routers.  Also,
 * you shouldn't call this function to add end devices to the neighbor table.
 * The command ID for this message is zstackmsg_CmdIDs_DEV_UPDATE_NEIGHBOR_TXCOST_REQ.
 */
typedef struct _zstackmsg_devupdateneighbortxcostreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_devUpdateNeighborTxCostReq_t *pReq;

} zstackmsg_devUpdateNeighborTxCostReq_t;

/**
 * Send this message to the ZStack Thread to Force Network Settings. DON'T USE
 * this function unless you know exactly what you are doing and can live with
 * the unpredictable consequences.  When this message is received, the ZStack
 * thread will force the values in the NIB then save the NIB.  It would be
 * better to let the ZStack thread set these items as they are determined.
 * The command ID for this message is
 * zstackmsg_CmdIDs_DEV_FORCE_NETWORK_SETTINGS_REQ.
 */
typedef struct _zstackmsg_devforcenetworksettingsreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_devForceNetworkSettingsReq_t *pReq;

} zstackmsg_devForceNetworkSettingsReq_t;

/**
 * Send this message to the ZStack Thread to Force Network Update. DON'T USE
 * this function unless you know exactly what you are doing and can live with
 * the unpredictable consequences.  When this message is received, the ZStack
 * thread will force the values in the NIB then save the NIB.  It would be
 * better to let the ZStack thread set these items as they are determined.
 * The command ID for this message is
 * zstackmsg_CmdIDs_DEV_FORCE_NETWORK_UPDATE_REQ.
 */
typedef struct _zstackmsg_devforcenetworkupdatereq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_devForceNetworkUpdateReq_t *pReq;

} zstackmsg_devForceNetworkUpdateReq_t;

/**
 * Send this message to the ZStack Thread to Force MAC Parameters. DON'T USE
 * this function unless you know exactly what you are doing and can live with
 * the unpredictable consequences.  When this message is received, the ZStack
 * thread will force the values in the MAC.  It would be better to let the
 * ZStack thread set these items as they are determined.
 * The command ID for this message is
 * zstackmsg_CmdIDs_DEV_FORCE_MAC_PARAMS_REQ.
 */
typedef struct _zstackmsg_devforcemacparamsreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_devForceMacParamsReq_t *pReq;

} zstackmsg_devForceMacParamsReq_t;

//*****************************************************************************
// APS Interface Request Messages
//*****************************************************************************

/**
 * Send this message to the ZStack Thread to remove an APS group.
 * The command ID for this message is zstackmsg_CmdIDs_APS_REMOVE_GROUP.
 */
typedef struct _zstackmsg_apsremovegroup_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_apsRemoveGroup_t *pReq;

} zstackmsg_apsRemoveGroup_t;

/**
 * Send this message to the ZStack Thread to remove all APS group.
 * The command ID for this message is zstackmsg_CmdIDs_APS_REMOVE_ALL_GROUPS.
 */
typedef struct _zstackmsg_apsremoveallgroups_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_apsRemoveAllGroups_t *pReq;

} zstackmsg_apsRemoveAllGroups_t;

/**
 * Send this message to the ZStack Thread to find all APS groups.
 * The command ID for this message is zstackmsg_CmdIDs_APS_FIND_ALL_GROUPS_REQ.
 */
typedef struct _zstackmsg_apsfindallgroupsreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_apsFindAllGroupsReq_t *pReq;

    /** Response fields (immediate response) */
    zstack_apsFindAllGroupsRsp_t *pRsp;

} zstackmsg_apsFindAllGroupsReq_t;

/**
 * Send this message to the ZStack Thread to find an APS group for a
 * given endpoint and group ID.
 * The command ID for this message is zstackmsg_CmdIDs_APS_FIND_GROUP_REQ.
 */
typedef struct _zstackmsg_apsfindgroupreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_apsFindGroupReq_t *pReq;

    /** Response fields (immediate response) */
    zstack_apsFindGroupRsp_t *pRsp;

} zstackmsg_apsFindGroupReq_t;

/**
 * Send this message to the ZStack Thread to add a group for a given endpoint.
 * The command ID for this message is zstackmsg_CmdIDs_APS_ADD_GROUP.
 */
typedef struct _zstackmsg_apsaddgroup_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_apsAddGroup_t *pReq;

} zstackmsg_apsAddGroup_t;

/**
 * Send this message to the ZStack Thread to count all APS groups for a given
 * endpoint.
 * The command ID for this message is zstackmsg_CmdIDs_APS_COUNT_ALL_GROUPS.
 */
typedef struct _zstackmsg_apscountallgroups_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /* Message command fields - no command fields, NULL field */
    void *pReq;

} zstackmsg_apsCountAllGroups_t;

//*****************************************************************************
// AF Interface Request Messages
//*****************************************************************************

/**
 * Send this message to the ZStack Thread to register an AF Enpoint
 * (including Simple Descriptor).
 * The command ID for this message is zstackmsg_CmdIDs_AF_REGISTER_REQ.
 */
typedef struct _zstackmsg_afregisterreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_afRegisterReq_t *pReq;

} zstackmsg_afRegisterReq_t;

/**
 * Send this message to the ZStack Thread to unregister an AF Enpoint.
 * The command ID for this message is zstackmsg_CmdIDs_AF_UNREGISTER_REQ.
 */
typedef struct _zstackmsg_afunregisterreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_afUnRegisterReq_t *pReq;

} zstackmsg_afUnRegisterReq_t;

/**
 * Send this message to the ZStack Thread to send a data message from an
 * endpoint.
 * The command ID for this message is zstackmsg_CmdIDs_AF_DATA_REQ.
 */
typedef struct _zstackmsg_afdatareq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_afDataReq_t *pReq;

} zstackmsg_afDataReq_t;

/**
 * Send this message to the ZStack Thread to setup the Inter-PAN controller.
 * The command ID for this message is zstackmsg_CmdIDs_AF_INTERPAN_CTL_REQ.
 */
typedef struct _zstackmsg_afinterpanctlreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_afInterPanCtlReq_t *pReq;

} zstackmsg_afInterPanCtlReq_t;

/**
 * Send this message to the ZStack Thread to get the
 * AF/APS Fragmentation parameters.
 * The command ID for this message is zstackmsg_CmdIDs_AF_CONFIG_GET_REQ.
 */
typedef struct _zstackmsg_afconfiggetreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_afConfigGetReq_t *pReq;

    /** Response fields (immediate response) */
    zstack_afConfigGetRsp_t *pRsp;

} zstackmsg_afConfigGetReq_t;

/**
 * Send this message to the ZStack Thread to set the
 * AF/APS Fragmentation parameters.
 * The command ID for this message is zstackmsg_CmdIDs_AF_CONFIG_SET_REQ.
 */
typedef struct _zstackmsg_afconfigsetreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_afConfigSetReq_t *pReq;

} zstackmsg_afConfigSetReq_t;

//*****************************************************************************
// AF Interface Indication Structures
//*****************************************************************************

/**
 * This message is sent from ZStack Thread to indicate that the message
 * (zstackmsg_afDataReq) was sent or not sent.
 * The command ID for this message is zstackmsg_CmdIDs_AF_DATA_CONFIRM_IND.
 */
typedef struct _zstackmsg_afdataconfirmind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_afDataConfirmInd_t req;

} zstackmsg_afDataConfirmInd_t;

/**
 * This message is sent from ZStack Thread to indicate an incoming
 * endpoint data message.
 * The command ID for this message is zstackmsg_CmdIDs_AF_INCOMING_MSG_IND.
 */
typedef struct _zstackmsg_afincomingmsgind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_afIncomingMsgInd_t req;

} zstackmsg_afIncomingMsgInd_t;

/**
 * This message is sent from ZStack Thread to indicate a reflection error.
 * The command ID for this message is zstackmsg_CmdIDs_AF_REFLECT_ERROR_IND.
 */
typedef struct _zstackmsg_afreflecterrorind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_afReflectErrorInd_t req;

} zstackmsg_afReflectErrorInd_t;

//*****************************************************************************
// ZDO Interface Request Structures
//*****************************************************************************

/**
 * Send this message to the ZStack Thread to send a broadcast
 * ZDO Network Address Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_NWK_ADDR_REQ.
 * When successful, the responding device will send back an asynchronous
 * ZStackmsg_zdoNwkAddrRspInd_t.
 */
typedef struct _zstackmsg_zdonwkaddrreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoNwkAddrReq_t *pReq;

} zstackmsg_zdoNwkAddrReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast
 * ZDO IEEE Address Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_IEEE_ADDR_REQ.
 * When successful, the responding device will send back an
 * asynchronous zstackmsg_zdoIeeeAddrRspInd_t.
 */
typedef struct _zstackmsg_zdoieeeaddrreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoIeeeAddrReq_t *pReq;

} zstackmsg_zdoIeeeAddrReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast
 * ZDO Node Descriptor Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_NODE_DESC_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoNodeDescRspInd_t.
 */
typedef struct _zstackmsg_zdonodedescreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoNodeDescReq_t *pReq;

} zstackmsg_zdoNodeDescReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast
 * ZDO Power Descriptor Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_POWER_DESC_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoPowerDescRspInd_t.
 */
typedef struct _zstackmsg_zdopowerdescreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoPowerDescReq_t *pReq;

} zstackmsg_zdoPowerDescReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast
 * ZDO Simple Descriptor Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoSimpleDescRspInd_t.
 */
typedef struct _zstackmsg_zdosimpledescreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoSimpleDescReq_t *pReq;

} zstackmsg_zdoSimpleDescReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast
 * ZDO Active Endpoint Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_ACTIVE_ENDPOINT_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoActiveEndpointRspInd_t.
 */
typedef struct _zstackmsg_zdoactiveendpointreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoActiveEndpointReq_t *pReq;

} zstackmsg_zdoActiveEndpointReq_t;

/**
 * Send this message to the ZStack Thread to send a ZDO Match Descriptor Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MATCH_DESC_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoMatchDescRspInd_t.
 */
typedef struct _zstackmsg_zdomatchdescreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMatchDescReq_t *pReq;

} zstackmsg_zdoMatchDescReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast
 * ZDO Complex Descriptor Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_COMPLEX_DESC_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoComplexDescRspInd_t.
 */
typedef struct _zstackmsg_zdocomplexdescreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoComplexDescReq_t *pReq;

} zstackmsg_zdoComplexDescReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast
 * ZDO User Descriptor Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_USER_DESC_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoUserDescRspInd_t.
 */
typedef struct _zstackmsg_zdouserdescreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoUserDescReq_t *pReq;

} zstackmsg_zdoUserDescReq_t;

/**
 * Send this message to the ZStack Thread to send a broadcast
 * ZDO Device Announce.<BR>
 * This command contains the device’s new 16-bit NWK address and its
 * 64-bit IEEE address, as well as the capabilities of the ZigBee device.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE_REQ.
 */
typedef struct _zstackmsg_zdodeviceannouncereq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoDeviceAnnounceReq_t *pReq;

} zstackmsg_zdoDeviceAnnounceReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast
 * ZDO Set User Descriptor Request.  The User Descriptor is a text string
 * describing the device in the network (ex. "Porch Light" ).
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_USER_DESCR_SET_REQ.
 */
typedef struct _zstackmsg_zdouserdescsetreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoUserDescSetReq_t *pReq;

} zstackmsg_zdoUserDescSetReq_t;

/**
 * Send this message to the ZStack Thread to send a
 * ZDO Server Discovery Request broadcast
 * to all RxOnWhenIdle devices.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_SERVER_DISC_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoServerDiscRspInd_t.
 */
typedef struct _zstackmsg_zdoserverdiscreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoServerDiscReq_t *pReq;

} zstackmsg_zdoServerDiscReq_t;

/**
 * Send this message to the ZStack Thread to send a ZDO End Device Bind Request
 * to the coordinator.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoEndDeviceBindRspInd_t.
 */
typedef struct _zstackmsg_zdoenddevicebindreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoEndDeviceBindReq_t *pReq;

} zstackmsg_zdoEndDeviceBindReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast ZDO Bind Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_BIND_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoBindRspInd_t.
 */
typedef struct _zstackmsg_zdobindreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoBindReq_t *pReq;

} zstackmsg_zdoBindReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast ZDO Unbind Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_UNBIND_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoUnbindRspInd_t.
 */
typedef struct _zstackmsg_zdounbindreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoUnbindReq_t *pReq;

} zstackmsg_zdoUnbindReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast
 * ZDO Management Network Discovery Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoMgmtNwkDiscRspInd_t.
 */
typedef struct _zstackmsg_zdomgmtnwkdiscreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtNwkDiscReq_t *pReq;

} zstackmsg_zdoMgmtNwkDiscReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast
 * ZDO Management LQI Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_LQI_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoMgmtLqiRspInd_t.
 */
typedef struct _zstackmsg_zdomgmtlqireq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtLqiReq_t *pReq;

} zstackmsg_zdoMgmtLqiReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast
 * ZDO Management Routing Table Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_RTG_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoMgmtRtgRspInd_t.
 */
typedef struct _zstackmsg_zdomgmtrtgreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtRtgReq_t *pReq;

} zstackmsg_zdoMgmtRtgReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast
 * ZDO Management Binding Table Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_BIND_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoMgmtBindRspInd_t.
 */
typedef struct _zstackmsg_zdomgmtbindreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtBindReq_t *pReq;

} zstackmsg_zdoMgmtBindReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast
 * ZDO Management Leave Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoMgmtLeaveRspInd_t.
 */
typedef struct _zstackmsg_zdomgmtleavereq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtLeaveReq_t *pReq;

} zstackmsg_zdoMgmtLeaveReq_t;

/**
 * Send this message to the ZStack Thread to send a unicast
 * ZDO Management Direct Join Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoMgmtDirectJoinRspInd_t.
 */
typedef struct _zstackmsg_zdomgmtdirectjoinreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtDirectJoinReq_t *pReq;

} zstackmsg_zdoMgmtDirectJoinReq_t;

/**
 * Send this message to the ZStack Thread to send a
 * ZDO Management Permit Join Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_REQ.
 * When successful, the responding device will send back an asynchronous
 * zstackmsg_zdoMgmtPermitJoinRspInd_t.
 */
typedef struct _zstackmsg_zdomgmtpermitjoinreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtPermitJoinReq_t *pReq;

} zstackmsg_zdoMgmtPermitJoinReq_t;

/**
 * Send this message to the ZStack Thread to send a ZDO Management
 * Network Update Request.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_REQ.
 */
typedef struct _zstackmsg_zdomgmtnwkupdatereq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtNwkUpdateReq_t *pReq;

} zstackmsg_zdoMgmtNwkUpdateReq_t;

//*****************************************************************************
// ZDO Interface Response Structures
//*****************************************************************************

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Network Address Response message is received.<BR>
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_NWK_ADDR_RSP.<BR>
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoNwkAddrReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdonwkaddrrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoNwkAddrRspInd_t rsp;

} zstackmsg_zdoNwkAddrRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO IEEE Address Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_IEEE_ADDR_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoIeeeAddrReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdoieeeaddrrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoIeeeAddrRspInd_t rsp;

} zstackmsg_zdoIeeeAddrRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Node Descriptor Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_NODE_DESC_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoNodeDescReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdonodedescrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoNodeDescRspInd_t rsp;

} zstackmsg_zdoNodeDescRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Power Descriptor Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_POWER_DESC_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoPowerDescReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdopowerdescrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoPowerDescRspInd_t rsp;

} zstackmsg_zdoPowerDescRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Simple Descriptor Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoSimpleDescReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdosimpledescrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoSimpleDescRspInd_t rsp;

} zstackmsg_zdoSimpleDescRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Active Endpoints Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_ACTIVE_EP_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoActiveEndpointsReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdoactiveendpointsrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoActiveEndpointsRspInd_t rsp;

} zstackmsg_zdoActiveEndpointsRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Match Descriptor Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MATCH_DESC_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoMatchDescReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdomatchdescrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMatchDescRspInd_t rsp;

} zstackmsg_zdoMatchDescRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO User Descriptor Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_USER_DESC_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoUserDescReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdouserdescrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoUserDescRspInd_t rsp;

} zstackmsg_zdoUserDescRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Server Discovery Descriptor Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_SERVER_DISC_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoServerDiscoveryReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdoserverdiscoveryrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoServerDiscoveryRspInd_t rsp;

} zstackmsg_zdoServerDiscoveryRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Bind Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_BIND_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoBindReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdobindrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoBindRspInd_t rsp;

} zstackmsg_zdoBindRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO End Device Bind Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoEndDeviceBindReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdoenddevicebindrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoEndDeviceBindRspInd_t rsp;

} zstackmsg_zdoEndDeviceBindRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Unbind Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_UNBIND_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoUnbindReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdounbindrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoUnbindRspInd_t rsp;

} zstackmsg_zdoUnbindRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Device Announce message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdodeviceannounceind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoDeviceAnnounceInd_t req;

} zstackmsg_zdoDeviceAnnounceInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Management Network Discovery Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoMgmtNwkDiscReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdomgmtnwkdiscrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtNwkDiscRspInd_t rsp;

} zstackmsg_zdoMgmtNwkDiscRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Management LQI Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_LQI_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoMgmtLqiReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdomgmtlqirspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtLqiRspInd_t rsp;

} zstackmsg_zdoMgmtLqiRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Management Routing Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_RTG_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoMgmtRtgReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdomgmtrtgrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtRtgRspInd_t rsp;

} zstackmsg_zdoMgmtRtgRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Management Binding Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_BIND_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoMgmtBindReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdomgmtbindrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtBindRspInd_t rsp;

} zstackmsg_zdoMgmtBindRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Management Leave Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoMgmtLeaveReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdomgmtleaverspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtLeaveRspInd_t rsp;

} zstackmsg_zdoMgmtLeaveRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Management Direct Join Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoMgmtDirectJoinReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdomgmtdirectjoinrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtDirectJoinRspInd_t rsp;

} zstackmsg_zdoMgmtDirectJoinRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Management Permit Join Response message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_RSP.
 * The source device sent this message in response to this device sending
 * zstackmsg_zdoMgmtPermitJoinReq_t.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdomgmtpermitjoinrspind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtPermitJoinRspInd_t rsp;

} zstackmsg_zdoMgmtPermitJoinRspInd_t;

/**
 * This response indication message is sent from ZStack Thread when a
 * ZDO Management Network Update Notify message is received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_NOTIFY.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdomgmtnwkupdatenotifyind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoMgmtNwkUpdateNotifyInd_t rsp;

} zstackmsg_zdoMgmtNwkUpdateNotifyInd_t;

/**
 * This response indication message is sent from ZStack Thread whenever the
 * Trust Center allows a device to join the network.  This message is will
 * only occur on a coordinator/trust center device.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_TC_DEVICE_IND.
 * <BR>
 * To receive this message, in your application, you must subscribe for it with
 * a call to Zstackapi_DevZDOCBReq().
 */
typedef struct _zstackmsg_zdotcdeviceind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoTcDeviceInd_t req;

} zstackmsg_zdoTcDeviceInd_t;

//*****************************************************************************
// ZDO Interface Confirm/Indication Structures
//*****************************************************************************

/**
 * This message is sent from ZStack Thread to indicate that a source route
 * was received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_SRC_RTG_IND.
 */
typedef struct _zstackmsg_zdosrcrtgind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoSrcRtgInd_t req;

} zstackmsg_zdoSrcRtgInd_t;

/**
 * This message is sent from ZStack Thread to indicate that concentrator
 * indication was received.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_CONCENTRATOR_IND.
 */
typedef struct _zstackmsg_zdocncntrtrind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoCncntrtrInd_t req;

} zstackmsg_zdoCncntrtrInd_t;

/**
 * This message is sent from ZStack Thread to indicate a
 * ZDO Network Discovery Confirmation.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_NWK_DISC_CNF.
 */
typedef struct _zstackmsg_zdonwkdisccnf_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoNwkDiscCnf_t req;

} zstackmsg_zdoNwkDiscCnf_t;

/**
 * This message is sent from ZStack Thread to indicate a beacon notification.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_BEACON_NOTIFY_IND.
 */
typedef struct _zstackmsg_zdobeaconnotifyind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoBeaconNotifyInd_t req;

} zstackmsg_zdoBeaconNotifyInd_t;

/**
 * This message is sent from ZStack Thread to indicate a join confirmation.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_JOIN_CNF.
 */
typedef struct _zstackmsg_zdojoinconf_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoJoinConf_t req;

} zstackmsg_zdoJoinConf_t;

/**
 * This message is sent from ZStack Thread to indicate a leave confirmation.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_LEAVE_CNF.
 */
typedef struct _zstackmsg_zdoleavecnf_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoLeaveCnf_t req;

} zstackmsg_zdoLeaveCnf_t;

/**
 * This message is sent from ZStack Thread to indicate a leave.
 * The command ID for this message is zstackmsg_CmdIDs_ZDO_LEAVE_IND.
 */
typedef struct _zstackmsg_zdoleaveind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_zdoLeaveInd_t req;

} zstackmsg_zdoLeaveInd_t;

//*****************************************************************************
// Device Interface Indication Structures
//*****************************************************************************

/**
 * This message is sent from ZStack Thread whenever a change in state occurs.
 * The command ID for this message is zstackmsg_CmdIDs_DEV_STATE_CHANGE_IND.
 */
typedef struct _zstackmsg_devstatechangeind_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_devStateChangeInd_t req;

} zstackmsg_devStateChangeInd_t;

//*****************************************************************************
// Security Interface Request Structures
//*****************************************************************************

/**
 * Send this message to the ZStack Thread to retrieve a network key.
 * The command ID for this message is zstackmsg_CmdIDs_SEC_NWK_KEY_GET_REQ.
 */
typedef struct _zstackmsg_secnwkkeygetreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_secNwkKeyGetReq_t *pReq;

    /** Response fields (immediate response) */
    zstack_secNwkKeyGetRsp_t *pRsp;

} zstackmsg_secNwkKeyGetReq_t;

/**
 * Send this message to the ZStack Thread to set a network key.
 * The command ID for this message is zstackmsg_CmdIDs_SEC_NWK_KEY_SET_REQ.
 */
typedef struct _zstackmsg_secnwkkeysetreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_secNwkKeySetReq_t *pReq;

} zstackmsg_secNwkKeySetReq_t;

/**
 * Send this message to the ZStack Thread to update a key in the network.
 * The command ID for this message is zstackmsg_CmdIDs_SEC_NWK_KEY_UPDATE_REQ.
 */
typedef struct _zstackmsg_secnwkkeyupdatereq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_secNwkKeyUpdateReq_t *pReq;

} zstackmsg_secNwkKeyUpdateReq_t;

/**
 * Send this message to the ZStack Thread to switch a key in the network.
 * The command ID for this message is zstackmsg_CmdIDs_SEC_NWK_KEY_SWITCH_REQ.
 */
typedef struct _zstackmsg_secnwkkeyswitchReq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_secNwkKeySwitchReq_t *pReq;

} zstackmsg_secNwkKeySwitchReq_t;

/**
 * Send this message to the ZStack Thread to retrieve an APS Link
 * Key (including TC Link Key).
 * The command ID for this message is zstackmsg_CmdIDs_SEC_APS_LINKKEY_GET_REQ.
 */
typedef struct _zstackmsg_secapslinkkeygetreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_secApsLinkKeyGetReq_t *pReq;

    /** Response fields (immediate response) */
    zstack_secApsLinkKeyGetRsp_t *pRsp;

} zstackmsg_secApsLinkKeyGetReq_t;

/**
 * Send this message to the ZStack Thread to set an APS Link
 * Key (including TC Link Key).
 * The command ID for this message is zstackmsg_CmdIDs_SEC_APS_LINKKEY_SET_REQ.
 */
typedef struct _zstackmsg_secapslinkkeysetreq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_secApsLinkKeySetReq_t *pReq;

} zstackmsg_secApsLinkKeySetReq_t;

/**
 * Send this message to the ZStack Thread to remove an APS Link
 * Key (including TC Link Key).
 * The command ID for this message is zstackmsg_CmdIDs_SEC_APS_LINKKEY_REMOVE_REQ.
 */
typedef struct _zstackmsg_secapslinkkeyremovereq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_secApsLinkKeyRemoveReq_t *pReq;

} zstackmsg_secApsLinkKeyRemoveReq_t;

/**
 * Send this message to the ZStack Thread to issue an APS Remove
 * Command (Trust Center Only).
 * The command ID for this message is zstackmsg_CmdIDs_SEC_APS_REMOVE_REQ.
 */
typedef struct _zstackmsg_secapsremovereq_t
{
    /** message header<br>
     * event field must be set to @ref zstack_CmdIDs
     */
    zstackmsg_HDR_t hdr;

    /** Message command fields */
    zstack_secApsRemoveReq_t *pReq;

} zstackmsg_secApsRemoveReq_t;

//*****************************************************************************
//*****************************************************************************

#ifdef __cplusplus
}
#endif

#endif /* ZSTACKMSG_H */

