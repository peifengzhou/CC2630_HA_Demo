/**
   @headerfile  zstackapi.h
   $Date: 2015-02-12 12:13:13 -0800 (Thu, 12 Feb 2015) $
   $Revision: 42527 $

   @mainpage Z-Stack TI-RTOS API

   Overview
   ============================

   To write an application for the Texas Instruments CC2630/CC2650 Z-Stack SDK
   you first need to understand that the embedded system for the CC2630/CC2650
   is divided into 2 seperate images.
   <BR>
   \image html HighLevelImages.PNG
   <BR>
   Each image is maintained as seperate projects that are built and downloaded
   to the CC2630/CC2650 seperately.
   <BR><BR>
   The functions defined in @ref zstackapi.h are functions that communicate
   with the ZStack image through ICall messages.  If an immediate response,
   not over the air, is expected, the function will block for the
   response ICall message.
   <BR><BR>
   To setup communicate with the ZStack thread, your application
   should do the following initialization(simplified):
   - Register one or more Zigbee Device Endpoints by calling
   Zstackapi_AfRegisterReq().
   - Register for ZStack Callback (asynchronous) messages by calling
   Zstackapi_DevZDOCBReq() with the wanted callbacks. For example, if you
   would like to receive device state change notifications, you will have to
   set the has_devStateChange and devStateChange fields to true in a
   zstack_devZDOCBReq_t, then call Zstackapi_DevZDOCBReq().
   - Write ZStack parameters by calling Zstackapi_sysConfigWriteReq() with
   the wanted parameters.  Example parameters are channel mask, PAN ID, poll
   rates, timeouts, etc...
   - Call Zstackapi_DevStartReq() to start the ZStack Thread's automatic
   joining process.

   When the joining process is complete, you'll receive a
   [zstackmsg_CmdIDs_DEV_STATE_CHANGE_IND](@ref zstack_CmdIDs)
   message with a state of:
   - [zstack_DevState_DEV_ZB_COORD](@ref zstack_DevState) - the
   device started as a coordinator.
   - [zstack_DevState_DEV_ROUTER](@ref zstack_DevState) - the device
   joined as a router.
   - [zstack_DevState_DEV_END_DEVICE](@ref zstack_DevState) - the
   device joined as an end device.

   You should then call zclport_getDeviceInfo(), or
   Zstackapi_sysNwkInfoReadReq() if your application isn't a ZCL project,
   to retrieve the device's joined network parameters.
   <BR>

   You're free to do device discovery through ZDO commands (ie.
   Zstackapi_ZdoMatchDescReq()) and send data
   either through ZCL commands or by calling Zstackapi_AfDataReq() to send
   application proprietary messages.
   <BR>

   Once you've registed an endpoint, your application will recieve any data
   message addressed to that endpoint as an AF Incoming Data Indication
   [[zstackmsg_afIncomingMsgInd_t](@ref _zstackmsg_afincomingmsgind_t) with a
   message event of [zstackmsg_CmdIDs_AF_INCOMING_MSG_IND](@ref zstack_CmdIDs)]
   through an ICall message.
   <BR>

   Also, any indications or response message that you have signed up for using
   Zstackapi_DevZDOCBReq() will be delivered to your application as an ICall
   message.  For example, the Device State Change message
   [[zstackmsg_devStateChangeInd_t](@ref _zstackmsg_devstatechangeind_t) with a
   message event of
   [zstackmsg_CmdIDs_DEV_STATE_CHANGE_IND](@ref zstack_CmdIDs)].
   <BR><BR>

   ZStack API
   ============================
   - <b>System Messages</b> - Configuration and miscellaneous commands
       - Zstackapi_sysVersionReq()
       - Zstackapi_sysConfigReadReq()
       - Zstackapi_sysConfigWriteReq()
       - Zstackapi_sysSetTxPowerReq()
       - Zstackapi_sysNwkInfoReadReq()
       - Zstackapi_sysForceLinkStatusReq()
   <BR><BR>
   - <b>Device Control</b><BR>
     The following group of APIs allow the application control the device
     in a Zigbee network: network discovery, device start, and control routing.
       - Zstackapi_DevStartReq()
       - Zstackapi_DevNwkDiscReq()
       - Zstackapi_DevJoinReq()
       - Zstackapi_DevZDOCBReq()
       - Zstackapi_DevNwkRouteReq()
       - Zstackapi_DevNwkCheckRouteReq()
       - Zstackapi_DevUpdateNeighborTxCostReq()
       - Zstackapi_DevForceNetworkSettingsReq()
       - Zstackapi_DevForceNetworkUpdateReq()
       - Zstackapi_DevForceMacParamsReq()
   <BR><BR>
   - <b>APS Inferace</b><BR>
       The APS group table is a link list defined with allocated RAM in the
       ZStack Thread, so as groups are added to the table the amount of heap
       used increases. The maximum table size can be changed by adjusting
       APS_MAX_GROUPS in f8wConfig.cfg (ZStack Thread Image), if this value
       is changed the ZStack Thread image must be recompiled and the Group
       table will take up more space in NV.
       - Zstackapi_ApsRemoveGroupReq()
       - Zstackapi_ApsRemoveAllGroupsReq()
       - Zstackapi_ApsFindAllGroupsReq()
       - Zstackapi_ApsFindGroupReq()
       - Zstackapi_ApsAddGroupReq()
       - Zstackapi_ApsCountAllGroupsReq()
   <BR><BR>
   - <b>Application Framework (AF)</b><BR>
     The Application Framework layer (withing the ZStack Thread) is the
     over-the-air data interface to the APS layer. This interface sends data out
     over the air (through the APS and NWK) layers. This layer is also the
     endpoint multiplexer for incoming data messages.<BR>
     Each device is a node in the Zigbee. Each node has a long and short
     address, the short address of the node is used by other nodes to send it
     data. Each node has 241 endpoint (0 reserved, 1-240 application assigned).
     Each endpoint is separately addressable; when a device sends data it
     must specify the destination node’s short address and the endpoint that
     will receive that data.<BR>
     An application must register one or more endpoints to send and receive
     data in a Zigbee network.<BR>
       - Zstackapi_AfRegisterReq()
       - Zstackapi_AfUnRegisterReq()
       - Zstackapi_AfDataReq()
       - Zstackapi_AfConfigGetReq()
       - Zstackapi_AfConfigSetReq()
   <BR><BR>
   - <b>Zigbee Device Objects (ZDO)</b><BR>
       This section enumerates all the function calls provided by the ZDO layer
       that are necessary for the implementation of all commands defined in
       ZigBee Device Profile (ZDP). <BR>
       ZDP describes how general ZigBee Device features are implemented within
       ZDO. It defines Device Description and Clusters which employ command.
       Through the definition of messages in command structure, ZDP provides
       the following functionality to the ZDO and applications: Device and
       Service Discovery; End Device Bind, Bind and Unbind Service; and Network
       Management Service. Device discovery is the process for a ZigBee device
       to discover other ZigBee Devices. One example of device discovery is the
       NWK address request which is broadcast and carries the known IEEE
       address as data payload. The device of interest should respond and
       inform its NWK address. Service discovery provides the ability for a
       device to discover the services offered by other ZigBee devices on the
       PAN. It utilizes various descriptors to specify device capabilities.<BR>
       End device bind, bind and unbind services offer ZigBee devices the
       binding capabilities. Typically, binding is used during the installation
       of a network when the user needs to bind controlling devices with
       devices being controlled, such as switches and lights. Particularly,
       end device bind supports a simplified binding method where user input
       is utilized to identify controlling/controlled device pairs. Bind and
       unbind services provide the ability for creation and deletion of
       binding table entry that map control messages to their intended
       destination.<BR>
       Network management services provide the ability to retrieve management
       information from the devices, including network discovery results,
       routing table contents, link quality to neighbor nodes, and binding
       table contents. It also provides the ability to control the network
       association by disassociating devices from the PAN. Network management
       services are designed majorly for user or commissioning tools to manage
       the network.<BR>
       - Zstackapi_ZdoNwkAddrReq()
       - Zstackapi_ZdoIeeeAddrReq()
       - Zstackapi_ZdoNodeDescReq()
       - Zstackapi_ZdoPowerDescReq()
       - Zstackapi_ZdoSimpleDescReq()
       - Zstackapi_ZdoActiveEndpointReq()
       - Zstackapi_ZdoMatchDescReq()
       - Zstackapi_ZdoComplexDescReq()
       - Zstackapi_ZdoServerDiscReq()
       - Zstackapi_ZdoEndDeviceBindReq()
       - Zstackapi_ZdoBindReq()
       - Zstackapi_ZdoUnbindReq()
       - Zstackapi_ZdoMgmtNwkDiscReq()
       - Zstackapi_ZdoMgmtLqiReq()
       - Zstackapi_ZdoMgmtRtgReq()
       - Zstackapi_ZdoMgmtBindReq()
       - Zstackapi_ZdoMgmtLeaveReq()
       - Zstackapi_ZdoMgmtDirectJoinReq()
       - Zstackapi_ZdoMgmtPermitJoinReq()
       - Zstackapi_ZdoMgmtNwkUpdateReq()
       - Zstackapi_ZdoDeviceAnnounceReq()
       - Zstackapi_ZdoUserDescSetReq()
       - Zstackapi_ZdoUserDescReq()
   <BR><BR>
   - <b>Security Interface</b> - Network and Link Key access commands
       - Zstackapi_secNwkKeyGetReq()
       - Zstackapi_secNwkKeySetReq()
       - Zstackapi_secNwkKeyUpdateReq()
       - Zstackapi_secNwkKeySwitchReq()
       - Zstackapi_secApsLinkKeyGetReq()
       - Zstackapi_secApsLinkKeySetReq()
       - Zstackapi_secApsLinkKeyRemoveReq()
       - Zstackapi_secApsRemoveReq()

   ZStack Indications (callbacks)
   ============================
   The following messages will be delivered to your application through an
   ICall message after you register at least one endpoint with
   Zstackapi_AfRegisterReq(), you must call Zstackapi_freeIndMsg() to
   free the message when you are done processing:
   <BR>

   - zstackmsg_CmdIDs_AF_DATA_CONFIRM_IND - [zstackmsg_afDataConfirmInd_t]
     (@ref _zstackmsg_afdataconfirmind_t)
   - zstackmsg_CmdIDs_AF_INCOMING_MSG_IND - [zstackmsg_afIncomingMsgInd_t]
     (@ref _zstackmsg_afincomingmsgind_t)

   The following messages will be delivered to your application through an
   ICall message after you request them by setting the correct fields
   (has and flag) to 'true' in
   [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) and calling
   Zstackapi_DevZDOCBReq(), you must call Zstackapi_freeIndMsg() to
   free the message when you are done processing:
   <BR>

   - zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE - [zstackmsg_zdoDeviceAnnounceReq_t]
     (@ref _zstackmsg_zdodeviceannouncereq_t)
   - zstackmsg_CmdIDs_ZDO_NWK_ADDR_RSP - [zstackmsg_zdoNwkAddrRspInd_t]
     (@ref _zstackmsg_zdonwkaddrrspind_t)
   - zstackmsg_CmdIDs_ZDO_IEEE_ADDR_RSP - [zstackmsg_zdoIeeeAddrRspInd_t]
     (@ref _zstackmsg_zdoieeeaddrrspind_t)
   - zstackmsg_CmdIDs_ZDO_NODE_DESC_RSP - [zstackmsg_zdoNodeDescRspInd_t]
     (@ref _zstackmsg_zdonodedescrspind_t)
   - zstackmsg_CmdIDs_ZDO_POWER_DESC_RSP - [zstackmsg_zdoPowerDescRspInd_t]
     (@ref _zstackmsg_zdopowerdescrspind_t)
   - zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_RSP - [zstackmsg_zdoSimpleDescRspInd_t]
     (@ref _zstackmsg_zdosimpledescrspind_t)
   - zstackmsg_CmdIDs_ZDO_ACTIVE_EP_RSP - [zstackmsg_zdoActiveEndpointsRspInd_t]
     (@ref _zstackmsg_zdoactiveendpointsrspind_t)
   - zstackmsg_CmdIDs_ZDO_MATCH_DESC_RSP - [zstackmsg_zdoMatchDescRspInd_t]
     (@ref _zstackmsg_zdomatchdescrspind_t)
   - zstackmsg_CmdIDs_ZDO_USER_DESC_RSP - [zstackmsg_zdoUserDescRspInd_t]
     (@ref _zstackmsg_zdouserdescrspind_t)
   - zstackmsg_CmdIDs_ZDO_SERVER_DISC_RSP - [zstackmsg_zdoServerDiscoveryRspInd_t]
     (@ref _zstackmsg_zdoserverdiscoveryrspind_t)
   - zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_RSP - [zstackmsg_zdoEndDeviceBindRspInd_t]
     (@ref _zstackmsg_zdoenddevicebindrspind_t)
   - zstackmsg_CmdIDs_ZDO_BIND_RSP - [zstackmsg_zdoBindRspInd_t]
     (@ref _zstackmsg_zdobindrspind_t)
   - zstackmsg_CmdIDs_ZDO_UNBIND_RSP - [zstackmsg_zdoUnbindRspInd_t]
     (@ref _zstackmsg_zdounbindrspind_t)
   - zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_RSP - [zstackmsg_zdoMgmtNwkDiscRspInd_t]
     (@ref _zstackmsg_zdomgmtnwkdiscrspind_t)
   - zstackmsg_CmdIDs_ZDO_MGMT_LQI_RSP - [zstackmsg_zdoMgmtLqiRspInd_t]
     (@ref _zstackmsg_zdomgmtlqirspind_t)
   - zstackmsg_CmdIDs_ZDO_MGMT_RTG_RSP - [zstackmsg_zdoMgmtRtgRspInd_t]
     (@ref _zstackmsg_zdomgmtrtgrspind_t)
   - zstackmsg_CmdIDs_ZDO_MGMT_BIND_RSP - [zstackmsg_zdoMgmtBindRspInd_t]
     (@ref _zstackmsg_zdomgmtbindrspind_t)
   - zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_RSP - [zstackmsg_zdoMgmtLeaveRspInd_t]
     (@ref _zstackmsg_zdomgmtleaverspind_t)
   - zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_RSP - [zstackmsg_zdoMgmtDirectJoinRspInd_t]
     (@ref _zstackmsg_zdomgmtdirectjoinrspind_t)
   - zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_RSP - [zstackmsg_zdoMgmtPermitJoinRspInd_t]
     (@ref _zstackmsg_zdomgmtpermitjoinrspind_t)
   - zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_NOTIFY - [zstackmsg_zdoMgmtNwkUpdateNotifyInd_t]
     (@ref _zstackmsg_zdomgmtnwkupdatenotifyind_t)
   - zstackmsg_CmdIDs_ZDO_SRC_RTG_IND - [zstackmsg_zdoSrcRtgInd_t]
     (@ref _zstackmsg_zdosrcrtgind_t)
   - zstackmsg_CmdIDs_ZDO_CONCENTRATOR_IND - [zstackmsg_zdoCncntrtrInd_t]
     (@ref _zstackmsg_zdocncntrtrind_t)
   - zstackmsg_CmdIDs_ZDO_NWK_DISC_CNF - [zstackmsg_zdoNwkDiscCnf_t]
     (@ref _zstackmsg_zdonwkdisccnf_t)
   - zstackmsg_CmdIDs_ZDO_BEACON_NOTIFY_IND - [zstackmsg_zdoBeaconNotifyInd_t]
     (@ref _zstackmsg_zdobeaconnotifyind_t)
   - zstackmsg_CmdIDs_ZDO_JOIN_CNF - [zstackmsg_zdoJoinConf_t]
     (@ref _zstackmsg_zdojoinconf_t)
   - zstackmsg_CmdIDs_ZDO_LEAVE_CNF - [zstackmsg_zdoLeaveCnf_t]
     (@ref _zstackmsg_zdoleavecnf_t)
   - zstackmsg_CmdIDs_ZDO_LEAVE_IND - [zstackmsg_zdoLeaveInd_t]
     (@ref _zstackmsg_zdoleaveind_t)
   - zstackmsg_CmdIDs_AF_REFLECT_ERROR_IND - [zstackmsg_afReflectErrorInd_t]
     (@ref _zstackmsg_afreflecterrorind_t)
   - zstackmsg_CmdIDs_DEV_STATE_CHANGE_IND - [zstackmsg_devStateChangeInd_t]
     (@ref _zstackmsg_devstatechangeind_t)
   - zstackmsg_CmdIDs_ZDO_TC_DEVICE_IND - [zstackmsg_zdoTcDeviceInd_t]
     (@ref _zstackmsg_zdotcdeviceind_t)
   - zstackmsg_CmdIDs_DEV_PERMIT_JOIN_IND - [zstackmsg_devPermitJoinInd_t]
     (@ref _zstackmsg_devpermitjoinind_t)
   <BR><BR>

   Application Helper Modules
   ============================
   - [Utilities](@ref UtilMisc) - Middleware Timer/Clock and integer-to-string
     functions.
   - [ZCL Porting functions](@ref ZclPort) - Functions needed by ZCL to
     communication with the ZStack Thread.
   - [Manual Device Startup](@ref ZStart) - Example module to manually control
     discovery and device joining.
   - [LED Support](@ref BoardLED) - Middleware to control the development board
     LEDs.
   - [LCD Support](@ref BoardLCD) - Middleware to control the development board
     LCD.
   - [Key Press Support](@ref BoardKey) - Middleware to setup and receive key
     press indications.

   <BR><BR><BR><BR>

   --------------------------------------------------------------------------------
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
 */
#ifndef ZSTACKAPI_H
#define ZSTACKAPI_H

#include <stdbool.h>
#include <stdint.h>
#include <ICall.h>

#include "zstackmsg.h"
#include "nvintf.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief       Call to send a System Reset Request, through the iCall
 *              dispatcher to the ZStack Thread.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_sysResetReq(
    ICall_EntityID        appEntity,
    zstack_sysResetReq_t *pReq);

/**
 * @brief       Call to send a System Version Request, through the iCall
 *              dispatcher to the ZStack Thread.<BR>
 *              Returns the core ZStack version (major, minor, maintenance).
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pRsp - Pointer to a place to put the response message.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_sysVersionReq(
    ICall_EntityID appEntity, zstack_sysVersionRsp_t *pRsp);

/**
 * @brief       Call to send a System Config Read Request, through the iCall
 *              dispatcher to the ZStack Thread.<BR>
 *              Setup pReq (zstack_sysConfigReadReq_t) with the parameters that
 *              you would like to read (you can set one or many),
 *              call this function, pRsp
 *              (zstack_sysConfigReadRsp_t) will have the requested parameters
 *              if the return value is zstack_ZStatusValues_ZSuccess.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 * @param       pRsp - Pointer to a place to put the response message.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_sysConfigReadReq(
    ICall_EntityID appEntity, zstack_sysConfigReadReq_t *pReq,
    zstack_sysConfigReadRsp_t *pRsp);

/**
 * @brief       Call to send a System Config Write Request, through the iCall
 *              dispatcher to the ZStack Thread.  zstack_sysConfigWriteReq_t
 *              contains all the parameters that an application can change.
 *              You can set as many parameters as you like, one, two, ten or
 *              all, in the request structure, with one API call.<BR>
 *              Each of the parameters has a "has_" field that must be set to
 *              true, along with the parameter data, for the field to be used
 *              by the ZStack thread.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_sysConfigWriteReq(
    ICall_EntityID appEntity, zstack_sysConfigWriteReq_t *pReq);

/**
 * @brief       Call to send a System Set TX Power Request, through the iCall
 *              dispatcher to the ZStack Thread.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 * @param       pRsp - Pointer to a place to put the response message.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_sysSetTxPowerReq(
    ICall_EntityID appEntity, zstack_sysSetTxPowerReq_t *pReq,
    zstack_sysSetTxPowerRsp_t *pRsp);

/**
 * @brief       Call to send a System Network Information Read Request,
 *              through the iCall dispatcher to the ZStack Thread.  After
 *              calling this function, pRsp will contain the device's
 *              network parameters if the return value is
 *              [zstack_ZStatusValues_ZSuccess](@ref zstack_ZStatusValues)
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pRsp - Pointer to a place to put the response message.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_sysNwkInfoReadReq(
    ICall_EntityID appEntity, zstack_sysNwkInfoReadRsp_t *pRsp);

/**
 * @brief       Call to send a System Force Link Status Request,
 *              through the iCall dispatcher to the ZStack Thread.  Calling
 *              this function forces a Link Status to be sent, but it doesn't
 *              interrupt the normal Link Status cycle.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_sysForceLinkStatusReq(
    ICall_EntityID appEntity);

/**
 * @brief       Call to send a Device Start Request, through the iCall
 *              dispatcher to the ZStack Thread.  Calling this function will
 *              start the device in the network. All of the network paramters
 *              [Zstackapi_sysConfigWriteReq()] must be set before calling
 *              this function.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_DevStartReq(
    ICall_EntityID        appEntity,
    zstack_devStartReq_t *pReq);

/**
 * @brief       Call to send a Device Network Discovery Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *
 *  NOTES:      Calling this request will temporarily set the
 *              beacon notification callback while the scan is active.
 *              When the scan is done, the beacon notification callback
 *              will be deactivated.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_DevNwkDiscReq(
    ICall_EntityID appEntity, zstack_devNwkDiscReq_t *pReq);

/**
 * @brief       Call to send a Device Join Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              Use this function call to join a specific device in the
 *              manual joining process, don't call this if you are using
 *              the ZStack Thread's automatic joining process.  Also, make
 *              sure to request a Join Confirm, by setting the has_joinCnfCB and
 *              joinCnfCB fields to true in zstack_devZDOCBReq_t can call
 *              Zstackapi_DevZDOCBReq(), to know when the join process is
 *              done.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_DevJoinReq(
    ICall_EntityID       appEntity,
    zstack_devJoinReq_t *pReq);

/**
 * @brief       Call to send a Device Rejoin Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_DevRejoinReq(
    ICall_EntityID         appEntity,
    zstack_devRejoinReq_t *pReq);

/**
 * @brief       Call to send a Device ZDO Callback Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              The zstack_devZDOCBReq_t contains all the
 *              callbacks/indications that an application can subscribe to.
 *              You can set as many callback/indications as you like, one,
 *              two, ten or all, in the request structure, with one API call.
 *              <BR>
 *              Each of the parameters has a "has_" field that must be set to
 *              true, along with the parameter data, for the field to be used
 *              by the ZStack thread.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_DevZDOCBReq(
    ICall_EntityID        appEntity,
    zstack_devZDOCBReq_t *pReq);

/**
 * @brief       Call to send a Device Network Route Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              Use this command to force a Route Request.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_DevNwkRouteReq(
    ICall_EntityID appEntity, zstack_devNwkRouteReq_t *pReq);

/**
 * @brief       Call to send a Device Network Check Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              Use this command to check if a route is active or not.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues_ZSuccess if active,
 *              zstack_ZStatusValues_ZFailure if not active
 */
extern zstack_ZStatusValues Zstackapi_DevNwkCheckRouteReq(
    ICall_EntityID appEntity, zstack_devNwkCheckRouteReq_t *pReq);

/**
 * @brief       Call to send a Device Update Neighbor's TxCost Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              Use this command to change a neighbor's TX Cost value.
 *              This command is only available in a router or coordinator.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_DevUpdateNeighborTxCostReq(
    ICall_EntityID appEntity, zstack_devUpdateNeighborTxCostReq_t *pReq);

/**
 * @brief       Call to send a Device Force Network Settings Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              DON'T USE this function unless you know exactly what you are
 *              doing and can live the unpredictable consequences.  When this
 *              message is received, the ZStack thread will force the values
 *              in the NIB then save the NIB.  It would be better to let the
 *              ZStack thread set these items as they are determined.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_DevForceNetworkSettingsReq(
    ICall_EntityID appEntity, zstack_devForceNetworkSettingsReq_t *pReq);

/**
 * @brief       Call to send a Device Force Network Update Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              DON'T USE this function unless you know exactly what you are
 *              doing and can live the unpredictable consequences.  When this
 *              message is received, the ZStack thread will force the values
 *              in the NIB then save the NIB.  It would be better to let the
 *              ZStack thread set these items as they are determined.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_DevForceNetworkUpdateReq(
    ICall_EntityID appEntity, zstack_devForceNetworkUpdateReq_t *pReq);

/**
 * @brief       Call to send a Device Force MAC Parameters Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              DON'T USE this function unless you know exactly what you are
 *              doing and can live the unpredictable consequences.  When this
 *              message is received, the ZStack thread will force the values
 *              in the MAC.  It would be better to let the ZStack thread set
 *              these items as they are determined.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_DevForceMacParamsReq(
    ICall_EntityID appEntity, zstack_devForceMacParamsReq_t *pReq);

/**
 * @brief       Call to send an APS Remove Group Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              Use this command to remove a group from an endpoint.
 *
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ApsRemoveGroupReq(
    ICall_EntityID appEntity, zstack_apsRemoveGroup_t *pReq);

/**
 * @brief       Call to send an APS Remove All Groups Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              Use this command to remove all groups from an endpoint.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ApsRemoveAllGroupsReq(
    ICall_EntityID appEntity, zstack_apsRemoveAllGroups_t *pReq);

/**
 * @brief       Call to send an APS Find All Groups Request, through the iCall
 *              dispatcher to the ZStack Thread.<BR>
 *              Use this command to return a list of groups that
 *              exists for the given endpoint.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 * @param       pRsp - Pointer to a place to put the response message.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ApsFindAllGroupsReq(
    ICall_EntityID appEntity, zstack_apsFindAllGroupsReq_t *pReq,
    zstack_apsFindAllGroupsRsp_t *pRsp);

/**
 * @brief       Call to send an APS Find Group Request, through the iCall
 *              dispatcher to the ZStack Thread.
 *              Use this command to return the group information (name) for
 *              a given group ID and endpoint.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 * @param       pRsp - Pointer to a place to put the response message.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ApsFindGroupReq(
    ICall_EntityID appEntity, zstack_apsFindGroupReq_t *pReq,
    zstack_apsFindGroupRsp_t *pRsp);

/**
 * @brief       Call to send an APS Add Group Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to add a group to an endpoint.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ApsAddGroupReq(
    ICall_EntityID appEntity, zstack_apsAddGroup_t *pReq);

/**
 * @brief       Call to send an APS Count All Groups Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              Use this command to retrieve the number of entries in
 *              group table.  For example if endpoint 1 and 2 both belong
 *              to group 1, then the count will be 2.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 *
 * @return      number of groups
 */
extern int Zstackapi_ApsCountAllGroupsReq(ICall_EntityID appEntity);

/**
 * @brief       Call to send an AF Register Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to register an endpoint descriptor with
 *              the ZStack thread.  This will allow the application to
 *              send and receive data messages.  These command is your
 *              registry for AF Data Indications.<BR>
 *              This command must be sent after every boot, this information
 *              is NOT saved in the ZStack Thread's non-volitile memory.<BR>
 *              The simple descriptor, which is part of the endpoint descriptor,
 *              is used during device discovery.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_AfRegisterReq(
    ICall_EntityID appEntity, zstack_afRegisterReq_t *pReq);

/**
 * @brief       Call to send an AF Unregister Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              Use this command to remove an endpoint descriptor from
 *              the ZStack Threads memory.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_AfUnRegisterReq(
    ICall_EntityID appEntity, zstack_afUnRegisterReq_t *pReq);

/**
 * @brief       Call to send an AF Data Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              Use this command to send over-the-air raw data messages.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_AfDataReq(ICall_EntityID appEntity,
                                                zstack_afDataReq_t *pReq);

/**
 * @brief       Call to send an AF InterPAN Control Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_AfInterpanCtlReq(
    ICall_EntityID appEntity, zstack_afInterPanCtlReq_t *pReq);

/**
 * @brief       Call to send an AF Config Get Request, through the iCall
 *              dispatcher to the ZStack Thread.<BR>
 *              Use this command to retrieve the AF fragmentation
 *              configuration information for the given endpoint.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 * @param       pRsp - Pointer to a place to put the response message.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_AfConfigGetReq(
    ICall_EntityID appEntity, zstack_afConfigGetReq_t *pReq,
    zstack_afConfigGetRsp_t *pRsp);

/**
 * @brief       Call to send an AF Config Set Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to set the AF fragmentation
 *              configuration information for the given endpoint.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_AfConfigSetReq(
    ICall_EntityID appEntity, zstack_afConfigSetReq_t *pReq);

/**
 * @brief       Call to send a ZDO Network Address Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Network Address
 *              Request message.  This function will not wait for the
 *              over-the-air response message, you will need to subscribe
 *              to response message by setting the has_nwkAddrRsp and
 *              nwkAddrRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              Calling this function will generate a message to ask for the
 *              16 bit address of the Remote Device based on its known IEEE
 *              address. This message is sent as a broadcast message to all
 *              devices in the network.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoNwkAddrReq(
    ICall_EntityID appEntity, zstack_zdoNwkAddrReq_t *pReq);

/**
 * @brief       Call to send a ZDO IEEE Address Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air IEEE Address
 *              Request message.  This function will not wait for the
 *              over-the-air response message, you will need to subscribe
 *              to response message by setting the has_ieeeAddrRsp and
 *              ieeeAddrRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              Calling this function will generate a message to ask for the
 *              64 bit address of the Remote Device based on its known 16 bit
 *              network address.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoIeeeAddrReq(
    ICall_EntityID appEntity, zstack_zdoIeeeAddrReq_t *pReq);

/**
 * @brief       Call to send a ZDO Node Descriptor Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Node Descriptor
 *              Request message.  This function will not wait for the
 *              over-the-air response message, you will need to subscribe
 *              to response message by setting the has_nodeDescRsp and
 *              nodeDescRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              This call will build and send a Node Descriptor Request to the
 *              Remote Device specified in t he destination address field.<BR>
 *              You can send this command to this device's address to perform
 *              the command locally.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoNodeDescReq(
    ICall_EntityID appEntity, zstack_zdoNodeDescReq_t *pReq);

/**
 * @brief       Call to send a ZDO Power Descriptor Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Power Descriptor
 *              Request message.  This function will not wait for the
 *              over-the-air response message, you will need to subscribe
 *              to response message by setting the has_powerDescRsp and
 *              powerDescRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              You can send this command to this device's address to perform
 *              the command locally.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoPowerDescReq(
    ICall_EntityID appEntity, zstack_zdoPowerDescReq_t *pReq);

/**
 * @brief       Call to send a ZDO Simple Descriptor Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Simple Descriptor
 *              Request message.  This function will not wait for the
 *              over-the-air response message, you will need to subscribe
 *              to response message by setting the has_simpleDescRsp and
 *              simpleDescRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoSimpleDescReq(
    ICall_EntityID appEntity, zstack_zdoSimpleDescReq_t *pReq);

/**
 * @brief       Call to send a ZDO Active Endpoint Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Active Endpoint
 *              Request message.  This function will not wait for the
 *              over-the-air response message, you will need to subscribe
 *              to response message by setting the has_activeEndpointRsp and
 *              activeEndpointRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              You can send this command to this device's address to perform
 *              the command locally.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoActiveEndpointReq(
    ICall_EntityID appEntity, zstack_zdoActiveEndpointReq_t *pReq);

/**
 * @brief       Call to send a ZDO Match Descriptor Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Match Descriptor
 *              Request message.  This function will not wait for the
 *              over-the-air response message, you will need to subscribe
 *              to response message by setting the has_matchDescRsp and
 *              matchDescRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoMatchDescReq(
    ICall_EntityID appEntity, zstack_zdoMatchDescReq_t *pReq);

/**
 * @brief       Call to send a ZDO Complex Descriptor Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Complex Descriptor
 *              Request message.  This function will not wait for the
 *              over-the-air response message, you will need to subscribe
 *              to response message by setting the has_complexDescRsp and
 *              complexDescRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              You can send this command to this device's address to perform
 *              the command locally.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoComplexDescReq(
    ICall_EntityID appEntity, zstack_zdoComplexDescReq_t *pReq);

/**
 * @brief       Call to send a ZDO User Descriptor Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air User Descriptor
 *              Request message.  This function will not wait for the
 *              over-the-air response message, you will need to subscribe
 *              to response message by setting the has_userDescRsp and
 *              userDescRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq(). <BR>
 *              You can send this command to this device's address to perform
 *              the command locally.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoUserDescReq(
    ICall_EntityID appEntity, zstack_zdoUserDescReq_t *pReq);

/**
 * @brief       Call to send a ZDO Device Announce Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              Use this command to send an over-the-air Device Announce
 *              message.  There is no over-the-air response message.<BR>
 *              This function builds and sends a Device_annce command for
 *              ZigBee end device to notify other ZigBee devices on the network
 *              that the end device has joined or rejoined the network. The
 *              command contains the device’s new 16-bit NWK address and its
 *              64-bit IEEE address, as well as the capabilities of the ZigBee
 *              device. It is sent out as broadcast message.<BR>
 *              On receipt of the Device_annce, all receivers shall check all
 *              internal references to the IEEE address supplied in the
 *              announce, and substitute the corresponding NWK address with the
 *              new one. No response will be sent back for Device_annce.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoDeviceAnnounceReq(
    ICall_EntityID appEntity, zstack_zdoDeviceAnnounceReq_t *pReq);

/**
 * @brief       Call to send a ZDO User Descriptor Set Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air User Descriptor Set
 *              Request message.  This function will not wait for the
 *              over-the-air response message, you will need to subscribe
 *              to response message by setting the has_userDescCnf and
 *              userDescCnf fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              You can send this command to this device's address to set the
 *              local user descriptor.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoUserDescSetReq(
    ICall_EntityID appEntity, zstack_zdoUserDescSetReq_t *pReq);

/**
 * @brief       Call to send a ZDO Server Discovery Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Server Discovery
 *              Request message.  This function will not wait for the
 *              over-the-air response message, you will need to subscribe
 *              to response message by setting the has_serverDiscoveryRsp and
 *              serverDiscoveryRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq(). <BR>
 *              The purpose of this request is to discover the locations of a
 *              particular system server or servers as indicated by the server
 *              mask. The message is broadcast to all device with RxOnWhenIdle.
 *              Remote devices will send responses back only if a match bit is
 *              found when comparing the received server mask with the mask
 *              stored in the local node descriptor, using unicast
 *              transmission.<BR>
 *              You can send this command to this device's address to perform
 *              the command locally.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoServerDiscReq(
    ICall_EntityID appEntity, zstack_zdoServerDiscReq_t *pReq);

/**
 * @brief       Call to send a ZDO End Device Bind Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air End Device Bind
 *              Request message.  This function will not wait for the
 *              over-the-air response message, you will need to subscribe
 *              to response message by setting the has_endDeviceBindRsp and
 *              endDeviceBindRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              Send this message to attempt a hand bind for this device. After
 *              hand binding your can send indirect (no address) message to the
 *              coordinator and the coordinator will send the message to the
 *              device that this message is bound to, or you will receive
 *              messages from your new bound device.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoEndDeviceBindReq(
    ICall_EntityID appEntity, zstack_zdoEndDeviceBindReq_t *pReq);

/**
 * @brief       Call to send a ZDO Bind Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Bind
 *              Request message.  This function will not wait for the
 *              over-the-air response message, you will need to subscribe
 *              to response message by setting the has_bindRsp and
 *              bindRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              Send this message to a device to build a binding record for/on
 *              that device.<BR>
 *              You can send this command to this device's address to create
 *              a local binding entry.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoBindReq(
    ICall_EntityID       appEntity,
    zstack_zdoBindReq_t *pReq);

/**
 * @brief       Call to send a ZDO Unbind Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Unbind
 *              Request message.  This function will not wait for the
 *              over-the-air response message, you will need to subscribe
 *              to response message by setting the has_unbindRsp and
 *              unbindRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              Send this message to a device to remove a binding record on
 *              that device.<BR>
 *              You can send this command to this device's address to unbind
 *              a local entry.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoUnbindReq(
    ICall_EntityID         appEntity,
    zstack_zdoUnbindReq_t *pReq);

/**
 * @brief       Call to send a ZDO Mgmt Network Discovery Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Management Network
 *              Discovery Request message.  This function will not wait for
 *              the over-the-air response message, you will need to subscribe
 *              to response message by setting the has_mgmtNwkDiscRsp and
 *              mgmtNwkDiscRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              If the destination device supports this command (optional),
 *              calling this function will generate the request for the
 *              destination device to perform a network scan and return the
 *              result in the response message.<BR>
 *              You can send this command to this device's address to perform
 *              the command locally.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoMgmtNwkDiscReq(
    ICall_EntityID appEntity, zstack_zdoMgmtNwkDiscReq_t *pReq);

/**
 * @brief       Call to send a ZDO Mgmt LQI Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Management LQI
 *              Discovery Request message.  This function will not wait for
 *              the over-the-air response message, you will need to subscribe
 *              to response message by setting the has_mgmtLqiRsp and
 *              mgmtLqiRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              If the destination device supports this command (optional),
 *              calling this function will generate the request for the
 *              destination device to return its neighbor list in the
 *              response message.<BR>
 *              You can send this command to this device's address to perform
 *              the command locally.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoMgmtLqiReq(
    ICall_EntityID appEntity, zstack_zdoMgmtLqiReq_t *pReq);

/**
 * @brief       Call to send a ZDO Mgmt Routing Request, through the iCall
 *              dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Management Routing
 *              Request message.  This function will not wait for
 *              the over-the-air response message, you will need to subscribe
 *              to response message by setting the has_mgmtRtgRsp and
 *              mgmtRtgRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              If the destination device supports this command (optional),
 *              calling this function will generate the request for the
 *              destination device to return its routing table in the
 *              response message.<BR>
 *              You can send this command to this device's address to perform
 *              the command locally.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoMgmtRtgReq(ICall_EntityID appEntity,
                                             zstack_zdoMgmtRtgReq_t *pReq);

/**
 * @brief       Call to send a ZDO Mgmt Bind Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Management Binding
 *              Request message.  This function will not wait for
 *              the over-the-air response message, you will need to subscribe
 *              to response message by setting the has_mgmtBindRsp and
 *              mgmtBindRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              If the destination device supports this command (optional),
 *              calling this function will generate the request for the
 *              destination device to return its binding table in the
 *              response message.<BR>
 *              You can send this command to this device's address to perform
 *              a local Mgmt Bind command.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoMgmtBindReq(
    ICall_EntityID appEntity, zstack_zdoMgmtBindReq_t *pReq);

/**
 * @brief       Call to send a ZDO Mgmt Leave Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Management Leave
 *              Request message.  This function will not wait for
 *              the over-the-air response message, you will need to subscribe
 *              to response message by setting the has_mgmtLeaveRsp and
 *              mgmtLeaveRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              If the destination device supports this command (optional),
 *              calling this function will generate the request for the
 *              destination device or another device to leave the network.<BR>
 *              You can send this command to this device's address to make
 *              this device leave.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoMgmtLeaveReq(
    ICall_EntityID appEntity, zstack_zdoMgmtLeaveReq_t *pReq);

/**
 * @brief       Call to send a ZDO Mgmt Direct Join Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Management Direct Join
 *              Request message.  This function will not wait for
 *              the over-the-air response message, you will need to subscribe
 *              to response message by setting the has_mgmtDirectJoinRsp and
 *              mgmtDirectJoinRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              If the destination device supports this command (optional),
 *              calling this function will generate the request for the
 *              destination device to direct join another device.<BR>
 *              You can send this command to this device's address to make a
 *              local direct join.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoMgmtDirectJoinReq(
    ICall_EntityID appEntity, zstack_zdoMgmtDirectJoinReq_t *pReq);

/**
 * @brief       Call to send a ZDO Mgmt Permit Join Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to send an over-the-air Management Permit Join
 *              Request message.  This function will not wait for
 *              the over-the-air response message, you will need to subscribe
 *              to response message by setting the has_mgmtPermitJoinRsp and
 *              mgmtPermitJoinRsp fields of
 *              [zstack_devZDOCBReq_t](@ref _zstack_devzdocbreq_t) to true and
 *              calling Zstackapi_DevZDOCBReq().<BR>
 *              If the destination device supports this command (optional),
 *              calling this function will generate the request for the
 *              destination device control permit joining.<BR>
 *              You can send this command to this device's address to locally
 *              control permit joining.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoMgmtPermitJoinReq(
    ICall_EntityID appEntity, zstack_zdoMgmtPermitJoinReq_t *pReq);

/**
 * @brief       Call to send a ZDO Mgmt Network Update Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              This command is provided to allow updating of network
 *              configuration parameters or to request information from devices
 *              on network conditions in the local operating environment.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_ZdoMgmtNwkUpdateReq(
    ICall_EntityID appEntity, zstack_zdoMgmtNwkUpdateReq_t *pReq);

/**
 * @brief       Call to send a Security Network Key Get Request, through
 *              the iCall dispatcher to the ZStack Thread.
 *              Use this command to retrieve the active or alternate network
 *              key from the ZStack Thread.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 * @param       pRsp - Pointer to a place to put the response message.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_secNwkKeyGetReq(
    ICall_EntityID appEntity, zstack_secNwkKeyGetReq_t *pReq,
    zstack_secNwkKeyGetRsp_t *pRsp);

/**
 * @brief       Call to send a Security Network Key Set Request,
 *              through the iCall dispatcher to the ZStack Thread.<BR>
 *              Use this command to set a network key (active or
 *              alternate).
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_secNwkKeySetReq(
    ICall_EntityID appEntity, zstack_secNwkKeySetReq_t *pReq);

/**
 * @brief       Call to send a Security Network Key Update Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              Use this command to send an over-the-air network
 *              key update message.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_secNwkKeyUpdateReq(
    ICall_EntityID appEntity, zstack_secNwkKeyUpdateReq_t *pReq);

/**
 * @brief       Call to send a Security Network Key Switch Request,
 *              through the iCall dispatcher to the ZStack Thread.
 *              Use this command to send an over-the-air network
 *              key switch message.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_secNwkKeySwitchReq(
    ICall_EntityID appEntity, zstack_secNwkKeySwitchReq_t *pReq);

/**
 * @brief       Call to send a Security APS Link Key Get Request, through
 *              the iCall dispatcher to the ZStack Thread.
 *              Use this command to get an APS Link key or TC Link key from
 *              the ZStack Thread's non-volatile memory.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 * @param       pRsp - Pointer to a place to put the response message.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_secApsLinkKeyGetReq(
    ICall_EntityID appEntity, zstack_secApsLinkKeyGetReq_t *pReq,
    zstack_secApsLinkKeyGetRsp_t *pRsp);

/**
 * @brief       Call to send a Security APS Link Key Set Request, through
 *              the iCall dispatcher to the ZStack Thread.
 *              Use this command to set an APS Link key or TC Link key into
 *              the ZStack Thread's non-volatile memory.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_secApsLinkKeySetReq(
    ICall_EntityID appEntity, zstack_secApsLinkKeySetReq_t *pReq);

/**
 * @brief       Call to send a Security APS Link Key Remove Request, through
 *              the iCall dispatcher to the ZStack Thread.
 *              Use this command to delete an APS Link key or TC Link key from
 *              the ZStack Thread's non-volatile memory.
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_secApsLinkKeyRemoveReq(
    ICall_EntityID appEntity, zstack_secApsLinkKeyRemoveReq_t *pReq);

/**
 * @brief       Call to send a Security APS Remove Request (to remove a
 *              a device from the network), through
 *              the iCall dispatcher to the ZStack Thread.  This only
 *              works if this device is a coordinator (Trust Center).
 *
 * @param       appEntity - Calling thread's iCall entity ID.
 * @param       pReq - Pointer to the Request structure.  Make sure
 *                    the structure is zeroed before filling in.
 *
 * @return      zstack_ZStatusValues
 */
extern zstack_ZStatusValues Zstackapi_secApsRemoveReq(
    ICall_EntityID appEntity, zstack_secApsRemoveReq_t *pReq);

/**
 * @brief       Call to free the memory used by an Indication message, messages
 *              sent asynchronously from the ZStack thread.
 *
 * @param       pMsg - Pointer to the indication message
 *
 * @return      true if processed (memory freed), false if not processed
 */
extern bool Zstackapi_freeIndMsg(void *pMsg);

#ifdef __cplusplus
}
;
#endif

#endif /* ZSTACKAPI_H */

