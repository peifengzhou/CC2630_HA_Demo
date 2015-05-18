/**
 @file  zstacktask.c
 @brief ZStack Thread implementation

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
#include <ICall.h>

#include "OSAL.h"
#include "OSAL_Nv.h"
#include "ZGlobals.h"
#include "AF.h"
#include "ZDConfig.h"
#include "ZDProfile.h"
#include "ZDObject.h"
#include "ZDApp.h"
#include "ZDSecMgr.h"
#include "rtg.h"
#include "nwk.h"
#include "aps_groups.h"

#include "zstack.h"
#include "zstackmsg.h"
#include "zstacktask.h"
#include "zsversion.h"

/* ------------------------------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------------------------------
 */

#define ZS_START_EVENT     0x0001

#if !defined (ZDSECMGR_TC_DEVICE_MAX)
#define ZDSECMGR_TC_DEVICE_MAX 1
#endif

#if !defined (MAX_SUPPORTED_ENDPOINTS)
#define MAX_SUPPORTED_ENDPOINTS 5
#endif

#define ZS_ZDO_SRC_RTG_IND_CBID             0x0001
#define ZS_ZDO_CONCENTRATOR_IND_CBID        0x0002
#define ZS_ZDO_NWK_DISCOVERY_CNF_CBID       0x0004
#define ZS_ZDO_BEACON_NOTIFY_IND_CBID       0x0008
#define ZS_ZDO_JOIN_CNF_CBID                0x0010
#define ZS_ZDO_LEAVE_CNF_CBID               0x0020
#define ZS_ZDO_LEAVE_IND_CBID               0x0040

#define ZS_ZDO_NWK_ADDR_RSP_CDID            0x00000001
#define ZS_ZDO_IEEE_ADDR_RSP_CDID           0x00000002
#define ZS_ZDO_NODE_DESC_RSP_CDID           0x00000004
#define ZS_ZDO_POWER_DESC_RSP_CDID          0x00000008
#define ZS_ZDO_SIMPLE_DESC_RSP_CDID         0x00000010
#define ZS_ZDO_ACTIVE_EP_RSP_CDID           0x00000020
#define ZS_ZDO_MATCH_DESC_RSP_CDID          0x00000040
#define ZS_ZDO_COMPLEX_DESC_RSP_CDID        0x00000080
#define ZS_ZDO_USER_DESC_RSP_CDID           0x00000100
#define ZS_ZDO_DISCOVERY_CACHE_RSP_CDID     0x00000200
#define ZS_ZDO_USER_DESC_CONF_CDID          0x00000400
#define ZS_ZDO_SERVER_DISCOVERY_RSP_CDID    0x00000800
/* can be used 0x00001000 */
#define ZS_ZDO_BIND_RSP_CDID                0x00002000
#define ZS_ZDO_END_DEVICE_BIND_RSP_CDID     0x00004000
#define ZS_ZDO_UNBIND_RSP_CDID              0x00008000
#define ZS_ZDO_MGMT_NWK_DISC_RSP_CDID       0x00010000
#define ZS_ZDO_MGMT_LQI_RSP_CDID            0x00020000
#define ZS_ZDO_MGMT_RTG_RSP_CDID            0x00040000
#define ZS_ZDO_MGMT_BIND_RSP_CDID           0x00080000
#define ZS_ZDO_MGMT_LEAVE_RSP_CDID          0x00100000
#define ZS_ZDO_MGMT_DIRECT_JOIN_RSP_CDID    0x00200000
#define ZS_ZDO_MGMT_PERMIT_JOIN_RSP_CDID    0x00400000
#define ZS_ZDO_MGMT_NWK_UPDATE_NOTIFY_CDID  0x00800000
#define ZS_ZDO_DEVICE_ANNOUNCE_CDID         0x01000000
#define ZS_DEV_STATE_CHANGE_CDID            0x02000000
#define ZS_DEV_JAMMER_IND_CDID              0x04000000
#define ZS_TC_DEVICE_IND_CDID               0x08000000
#define ZS_DEV_PERMIT_JOIN_IND_CDID         0x10000000

/* Capability Information */
#define CAPABLE_PAN_COORD       0x01  /* Device is capable of becoming a PAN
                                        coordinator */
#define CAPABLE_FFD             0x02  /* Device is an FFD */
#define CAPABLE_MAINS_POWER     0x04  /* Device is mains powered rather than
                                        battery powered */
#define CAPABLE_RX_ON_IDLE      0x08  /* Device has its receiver on when idle
                                        */
#define CAPABLE_SECURITY        0x40  /* Device is capable of sending and
                                        receiving secured frames */
#define CAPABLE_ALLOC_ADDR      0x80  /* Request allocation of a short address
                                        in the associate procedure */

/* ------------------------------------------------------------------------------------------------
 * Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef struct
{
  void *next;                // Next in the link List
  ICall_EntityID connection;
  uint16 zdoCBs;
  uint32 zdoRsps;
  endPointDesc_t epDesc;
} epItem_t;

// This is also defined in ZDSecMgr.c
typedef struct
{
  uint16 ami;
  uint16 keyNvId;   // index to the Link Key table in NV
  ZDSecMgr_Authentication_Option authenticateOption;
} ZDSecMgrEntry_t;

typedef void (*pfnZDOCB)( uint16 dstID, void *pStr );
typedef void (*pfnZDOMsgCB)( uint16 dstID, uint16 srcAddr, void *pStr );

/* ------------------------------------------------------------------------------------------------
 * Global Variables
 * ------------------------------------------------------------------------------------------------
 */
/* Save the ZStackTaskId */
uint8 ZStackTaskId;

uint8 maxSupportedEndpoints = MAX_SUPPORTED_ENDPOINTS;

uint8 zspbPermitJoin = TRUE;
uint8 nwkUseMultiCast = FALSE;

/* ------------------------------------------------------------------------------------------------
 * Local Variables
 * ------------------------------------------------------------------------------------------------
 */
static ICall_EntityID ZStackEntity;
static ICall_Semaphore ZStackSem;

static epItem_t *pEpTableHdr = NULL;

#if defined (HOLD_AUTO_START)
static devStates_t newDevState = DEV_HOLD;
#else
static devStates_t newDevState = DEV_INIT;
#endif

static bool manualJoin = FALSE;

/* ------------------------------------------------------------------------------------------------
 * External Function Prototypes
 * ------------------------------------------------------------------------------------------------
 */
extern ZStatus_t ZDSecMgrEntryLookupExt( uint8 *extAddr, ZDSecMgrEntry_t **entry );
extern void ZDApp_NodeProfileSync( uint8 stackProfile );

/* ------------------------------------------------------------------------------------------------
 * Local Function Prototypes
 * ------------------------------------------------------------------------------------------------
 */
static bool appMsg( ICall_EntityID srcEntityID, void *pMsg );
static void processAfDataConfirm( afDataConfirm_t *pkt );
static void processAfIncomingMsgInd( afIncomingMSGPacket_t *pkt );
static void processAfReflectErrorInd( afReflectError_t *pkt );

static void zsProcessZDOMsgs( zdoIncomingMsg_t *inMsg );

static uint8 epTableAddNewEntry( epItem_t *newEntry );
static epItem_t *epTableFindEntryEP( uint8 ep );
static epItem_t *epTableFindEntryConnection( int connection );
static void epTableRemoveEntry( epItem_t *entry );
static uint8 epTableNumEntries( void );
static void freeEpItem( epItem_t *pItem );

static void sendMsgToAllCBs( uint16 cbMask, void *pBuf, pfnZDOCB pFn );
static void sendMsgToAllCBMsgs( uint32 cbMask, uint16 srcAddr, void *pBuf, pfnZDOMsgCB pFn );
static void *zdoNwkDiscCnfCB( void *pStr );
static void sendNwkDiscCnf( uint16 dstID, void *pStr );
static void *zdoBeaconNotifyIndCB( void *pStr );
static void sendBeaconNotifyInd( uint16 dstID, void *pStr );
static void *zdoSrcRtgCB( void *pStr );
static void sendZdoSrcRtgInd( uint16 dstID, void *pStr );
static void *zdoConcentratorIndCB( void *pStr );
static void sendZdoConcentratorInd( uint16 dstID, void *pStr );
static void *zdoJoinCnfCB( void *pStr );
static void sendJoinCnfInd( uint16 dstID, void *pStr );
static void *zdoLeaveIndCB( void *pStr );
static void sendLeaveInd( uint16 dstID, void *pStr );
static void *zdoPermitJoinIndCB( void *pStr );
static void sendDevPermitJoinInd( uint16 dstID, uint16 srcAddr, void *pMsg );

static void *zdoTcDeviceIndCB( void *pStr );
static void sendTcDeviceInd( uint16 dstID, uint16 srcAddr, void *pMsg );

static void sendDeviceAnnounce( uint16 srcAddr, void *pMsg );
static void sendDeviceAnnounceInd( uint16 dstID, uint16 srcAddr, void *pMsg );

#if defined (ZDO_NWKADDR_REQUEST)
static void sendNwkAddrRsp( uint16 srcAddr, void *pMsg );
static void sendNwkAddrRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_IEEEADDR_REQUEST)
static void sendIeeeAddrRsp( uint16 srcAddr, void *pMsg );
static void sendIeeeAddrRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_NODEDESC_REQUEST)
static void sendNodeDescRsp( uint16 srcAddr, void *pMsg );
static void sendNodeDescRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_POWERDESC_REQUEST)
static void sendPowerDescRsp( uint16 srcAddr, void *pMsg );
static void sendPowerDescRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_SIMPLEDESC_REQUEST)
static void sendSimpleDescRsp( uint16 srcAddr, void *pMsg );
static void sendSimpleDescRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_ACTIVEEP_REQUEST)
static void sendActiveEPRsp( uint16 srcAddr, void *pMsg );
static void sendActiveEPRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_MATCH_REQUEST)
static void sendMatchDescRsp( uint16 srcAddr, void *pMsg );
static void sendMatchDescRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_USERDESC_REQUEST)
static void sendUserDescRsp( uint16 srcAddr, void *pMsg );
static void sendUserDescRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_SERVERDISC_REQUEST)
static void sendServerDiscRsp( uint16 srcAddr, void *pMsg );
static void sendServerDiscRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_BIND_UNBIND_REQUEST)
static void sendBindRsp( uint16 srcAddr, void *pMsg );
static void sendBindRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_BIND_UNBIND_REQUEST)
static void sendUnbindRsp( uint16 srcAddr, void *pMsg );
static void sendUnbindRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_ENDDEVICEBIND_REQUEST)
static void sendEndDeviceBindRsp( uint16 srcAddr, void *pMsg );
static void sendEndDeviceBindRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_MGMT_NWKDISC_REQUEST)
static void sendMgmtNwkDiscRsp( uint16 srcAddr, void *pMsg );
static void sendMgmtNwkDiscRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_MGMT_LQI_REQUEST)
static void sendMgmtLqiRsp( uint16 srcAddr, void *pMsg );
static void sendMgmtLqiRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_MGMT_RTG_REQUEST)
static void sendMgmtRtgRsp( uint16 srcAddr, void *pMsg );
static void sendMgmtRtgRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_MGMT_BIND_REQUEST)
static void sendMgmtBindRsp( uint16 srcAddr, void *pMsg );
static void sendMgmtBindRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_MGMT_LEAVE_REQUEST)
static void sendMgmtLeaveRsp( uint16 srcAddr, void *pMsg );
static void sendMgmtLeaveRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_MGMT_JOINDIRECT_REQUEST)
static void sendMgmtDirectJoinRsp( uint16 srcAddr, void *pMsg );
static void sendMgmtDirectJoinRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_MGMT_PERMIT_JOIN_REQUEST)
static void sendMgmtPermitJoinRsp( uint16 srcAddr, void *pMsg );
static void sendMgmtPermitJoinRspInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

#if defined (ZDO_MGMT_NWKUPDATE_REQUEST)
static void sendMgmtNwkUpdateNotify( uint16 srcAddr, void *pMsg );
static void sendMgmtNwkUpdateNotifyInd( uint16 dstID, uint16 srcAddr, void *pMsg );
#endif

static void processDevStateChange( uint16 srcAddr, void *pMsg );
static void sendDevStateChangeInd( uint16 dstID, uint16 srcAddr, void *pMsg );

static bool processSysVersionReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processDevStartReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processSysSetTxPowerReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processDevJoinReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processDevForceNetworkSettingsReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processDevForceNetworkUpdateReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processDevForceMacParamsReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processDevUpdateNeighborTxCostReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processApsRemoveGroup( ICall_EntityID srcEntityID, void *pMsg );
static bool processApsRemoveAllGroups( ICall_EntityID srcEntityID, void *pMsg );
static bool processApsFindAllGroupsReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processApsFindGroupReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processApsAddGroup( ICall_EntityID srcEntityID, void *pMsg );
static bool processApsCountAllGroups( ICall_EntityID srcEntityID, void *pMsg );
static bool processSecApsRemoveReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processSecNwkKeyUpdateReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processSecNwkKeySwitchReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processSecNwkKeySetReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processSecNwkKeyGetReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processSecApsLinkKeyGetReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processSecApsLinkKeySetReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processSecApsLinkKeyRemoveReq( ICall_EntityID srcEntityID, void *pMsg );

#if defined (RTR_NWK)
static bool processDevNwkRouteReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processDevNwkCheckRouteReq( ICall_EntityID srcEntityID, void *pMsg );
#endif // RTR_NWK

static bool processDevNwkDiscoveryReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processSysForceLinkStatusReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processSysConfigReadReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processSysConfigWriteReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processSysNwkInfoReadReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processDevZDOCBReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processAfRegisterReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processAfUnRegisterReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processAfConfigGetReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processAfConfigSetReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processAfDataReq( ICall_EntityID srcEntityID, void *pMsg );
static bool processZdoDeviceAnnounceReq( ICall_EntityID srcEntityID, void *pMsg );

#if defined (ZDO_NWKADDR_REQUEST)
static bool processZdoNwkAddrReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_IEEEADDR_REQUEST)
static bool processZdoIeeeAddrReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_NODEDESC_REQUEST)
static bool processZdoNodeDescReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_POWERDESC_REQUEST)
static bool processZdoPowerDescReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_SIMPLEDESC_REQUEST)
static bool processZdoSimpleDescReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_ACTIVEEP_REQUEST)
static bool processZdoActiveEndpointsReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_MATCH_REQUEST)
static bool processZdoMatchDescReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_COMPLEXDESC_REQUEST)
static bool processZdoComplexDescReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_SERVERDISC_REQUEST)
static bool processZdoServerDiscReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_ENDDEVICEBIND_REQUEST)
static bool processZdoEndDeviceBindReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_BIND_UNBIND_REQUEST)
static bool processZdoBindReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_BIND_UNBIND_REQUEST)
static bool processZdoUnbindReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_MGMT_NWKDISC_REQUEST)
static bool processZdoMgmtNwkDiscReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_MGMT_LQI_REQUEST)
static bool processZdoMgmtLqiReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_MGMT_RTG_REQUEST)
static bool processZdoMgmtRtgReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_MGMT_BIND_REQUEST)
static bool processZdoMgmtBindReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_MGMT_LEAVE_REQUEST)
static bool processZdoMgmtLeaveReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_MGMT_JOINDIRECT_REQUEST)
static bool processZdoMgmtDirectJoinReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_MGMT_PERMIT_JOIN_REQUEST)
static bool processZdoMgmtPermitJoinReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_MGMT_NWKUPDATE_REQUEST)
static bool processZdoMgmtNwkUpdateReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_USERDESCSET_REQUEST)
static bool processZdoUserDescSetReq( ICall_EntityID srcEntityID, void *pMsg );
#endif

#if defined (ZDO_USERDESC_REQUEST)
static bool processZdoUserDescReq( ICall_EntityID srcEntityID, void *pMsg );
#endif
static bool isDevicePartOfNetwork( void );

/**************************************************************************************************
 * @fn          ZStackTaskInit
 *
 * @brief       This function is called when OSAL is initialized.
 *
 * input parameters
 *
 * @param       taskId - OSAL task ID for ZStack Thread
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 */
void ZStackTaskInit( uint8 taskId )
{
  /* Enroll the service that this stack represents */
  ICall_enrollService( ICALL_SERVICE_CLASS_ZSTACK, NULL, &ZStackEntity, &ZStackSem );

  /* Enroll the obtained dispatcher entity and task ID to OSAL
   * so that OSAL can route the dispatcher message into
   * the appropriate OSAL task.
   */
  osal_enroll_dispatchid( taskId, ZStackEntity );

  /* Enroll the obtained service entity. This is to ensure
   * that non-task (such as ISR) can route the dispatcher message
   * into the appropriate alien task (application).
   */
  osal_enroll_notasksender( ZStackEntity );

  ZStackTaskId = taskId;

  // Register for ZDO Function Callbacks
  ZDO_RegisterForZdoCB( ZDO_SRC_RTG_IND_CBID, zdoSrcRtgCB );
  ZDO_RegisterForZdoCB( ZDO_CONCENTRATOR_IND_CBID, zdoConcentratorIndCB );
  ZDO_RegisterForZdoCB( ZDO_JOIN_CNF_CBID, zdoJoinCnfCB );
  ZDO_RegisterForZdoCB( ZDO_LEAVE_IND_CBID, zdoLeaveIndCB );
  ZDO_RegisterForZdoCB( ZDO_PERMIT_JOIN_CBID, zdoPermitJoinIndCB );
  ZDO_RegisterForZdoCB( ZDO_TC_DEVICE_CBID, zdoTcDeviceIndCB );

  // Register for ZDO Rsp messages
  ZDO_RegisterForZDOMsg( ZStackTaskId, ZDO_ALL_MSGS_CLUSTERID );
}

/**************************************************************************************************
 * @fn          ZStackTaskProcessEvent
 *
 * @brief       This function is the main event handling function of the ZStack Thread executing
 *              in task context.  This function is called by OSAL when an event or message
 *              is pending for the ZStack Thread.
 *
 * input parameters
 *
 * @param       taskId - OSAL task ID of this task.
 * @param       events - OSAL event mask.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
uint16 ZStackTaskProcessEvent( uint8 taskId, uint16 events )
{
  zstackmsg_sysResetReq_t *pMsg;

  (void)taskId;   // Intentionally unreferenced parameter

  // Process system messages
  if ( events & SYS_EVENT_MSG )
  {
    if ( ( pMsg = (zstackmsg_sysResetReq_t *)osal_msg_receive( ZStackTaskId ) ) != NULL )
    {
      bool send = FALSE;
      ICall_EntityID srcEntityID;

      switch ( pMsg->hdr.event )
      {
        case ZDO_CB_MSG:
          zsProcessZDOMsgs( (zdoIncomingMsg_t *)pMsg );
          break;

        case AF_DATA_CONFIRM_CMD:
          processAfDataConfirm( (afDataConfirm_t *)pMsg );
          break;

        case AF_REFLECT_ERROR_CMD:
          processAfReflectErrorInd( (afReflectError_t *)pMsg );
          break;

        case AF_INCOMING_MSG_CMD:
          processAfIncomingMsgInd( (afIncomingMSGPacket_t *)pMsg );
          break;

        case ZDO_STATE_CHANGE:
          {
            if ( newDevState != pMsg->hdr.status )
            {
              if ( (pMsg->hdr.status == DEV_ZB_COORD)
                  || (pMsg->hdr.status == DEV_ROUTER)
                  || (pMsg->hdr.status == DEV_END_DEVICE) )
              {
                zAddrType_t dstAddr;
                uint8 duration = 0;

                if ( zspbPermitJoin )
                {
                  duration = 0xFF;
                }

                dstAddr.addrMode = Addr16Bit;
                dstAddr.addr.shortAddr = NLME_GetShortAddr( );

                ZDP_MgmtPermitJoinReq( &dstAddr, duration, 0, 0 );
              }

              newDevState = (devStates_t)pMsg->hdr.status;
              processDevStateChange( 0, &newDevState );

              // Adjust the multicast setting
              _NIB.nwkUseMultiCast = nwkUseMultiCast;
            }
          }
          break;

        default:
          { // Assume it's a message from another thread
            osal_msg_hdr_t *pHdr;

            // Get the iCall message header
            pHdr = (osal_msg_hdr_t *)pMsg - 1;
            // Convert to the ICall entity ID
            srcEntityID = pHdr->srcentity;

            send = appMsg( srcEntityID, pMsg );
          }
          break;
      }

      if ( send )
      {
        // Send the message back to the originator
        (void)ICall_send( ZStackEntity, srcEntityID, ICALL_MSG_FORMAT_KEEP, (void *)pMsg );
      }
      else
      {
        // Release the memory
        osal_msg_deallocate( (uint8 *)pMsg );
      }
    }

    // Return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & ZS_START_EVENT )
  {
    ZDApp_StartJoiningCycle( );

    // return unprocessed events
    return (events ^ ZS_START_EVENT);
  }

  // When reaching here, the events are unknown
  // Discard them or make more handlers
  return 0;
}

/**************************************************************************************************
 * @fn          appMsg
 *
 * @brief       Process an incoming Application task message.
 *
 * @param       srcEntityID - source thread ID
 * @param       pMsg - pointer to the incoming message
 *
 * @return      TRUE to send the message back to the sender, FALSE if not
 */
static bool appMsg( ICall_EntityID srcEntityID, void *pMsg )
{
  bool resend = TRUE; // default to resend to app task
  // Temp convert to get the event
  zstackmsg_sysResetReq_t *pReq = (zstackmsg_sysResetReq_t *)pMsg;

  switch ( pReq->hdr.event )
  {
    case zstackmsg_CmdIDs_DEV_REJOIN_REQ:
    case zstackmsg_CmdIDs_AF_INTERPAN_CTL_REQ:
    case zstackmsg_CmdIDs_SYS_RESET_REQ:
      // Not supported yet
      pReq->hdr.status = zstack_ZStatusValues_ZUnsupportedMode;
      break;

    case zstackmsg_CmdIDs_DEV_FORCE_NETWORK_SETTINGS_REQ:
      resend = processDevForceNetworkSettingsReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_DEV_FORCE_NETWORK_UPDATE_REQ:
      resend = processDevForceNetworkUpdateReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_DEV_FORCE_MAC_PARAMS_REQ:
      resend = processDevForceMacParamsReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_DEV_UPDATE_NEIGHBOR_TXCOST_REQ:
      resend = processDevUpdateNeighborTxCostReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_DEV_JOIN_REQ:
      resend = processDevJoinReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_DEV_NWK_DISCOVERY_REQ:
      resend = processDevNwkDiscoveryReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_SYS_FORCE_LINK_STATUS_REQ:
      resend = processSysForceLinkStatusReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_SYS_VERSION_REQ:
      resend = processSysVersionReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_DEV_START_REQ:
      resend = processDevStartReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_SYS_SET_TX_POWER_REQ:
      resend = processSysSetTxPowerReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_APS_REMOVE_GROUP:
      resend = processApsRemoveGroup( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_APS_REMOVE_ALL_GROUPS:
      resend = processApsRemoveAllGroups( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_APS_FIND_ALL_GROUPS_REQ:
      resend = processApsFindAllGroupsReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_APS_FIND_GROUP_REQ:
      resend = processApsFindGroupReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_APS_ADD_GROUP:
      resend = processApsAddGroup( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_APS_COUNT_ALL_GROUPS:
      resend = processApsCountAllGroups( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_SEC_APS_REMOVE_REQ:
      resend = processSecApsRemoveReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_SEC_NWK_KEY_UPDATE_REQ:
      resend = processSecNwkKeyUpdateReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_SEC_NWK_KEY_SWITCH_REQ:
      resend = processSecNwkKeySwitchReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_SEC_NWK_KEY_SET_REQ:
      resend = processSecNwkKeySetReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_SEC_NWK_KEY_GET_REQ:
      resend = processSecNwkKeyGetReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_SEC_APS_LINKKEY_GET_REQ:
      resend = processSecApsLinkKeyGetReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_SEC_APS_LINKKEY_SET_REQ:
      resend = processSecApsLinkKeySetReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_SEC_APS_LINKKEY_REMOVE_REQ:
      resend = processSecApsLinkKeyRemoveReq( srcEntityID, pMsg );
      break;

#if defined (RTR_NWK)
    case zstackmsg_CmdIDs_DEV_NWK_ROUTE_REQ:
      resend = processDevNwkRouteReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_DEV_NWK_CHECK_ROUTE_REQ:
      resend = processDevNwkCheckRouteReq( srcEntityID, pMsg );
      break;
#endif // RTR_NWK

    case zstackmsg_CmdIDs_SYS_CONFIG_READ_REQ:
      resend = processSysConfigReadReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_SYS_CONFIG_WRITE_REQ:
      resend = processSysConfigWriteReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_SYS_NWK_INFO_READ_REQ:
      resend = processSysNwkInfoReadReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_DEV_ZDO_CBS_REQ:
      resend = processDevZDOCBReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_AF_REGISTER_REQ:
      resend = processAfRegisterReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_AF_UNREGISTER_REQ:
      resend = processAfUnRegisterReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_AF_CONFIG_GET_REQ:
      resend = processAfConfigGetReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_AF_CONFIG_SET_REQ:
      resend = processAfConfigSetReq( srcEntityID, pMsg );
      break;

    case zstackmsg_CmdIDs_AF_DATA_REQ:
      resend = processAfDataReq( srcEntityID, pMsg );
      break;

#if defined (ZDO_NWKADDR_REQUEST)
    case zstackmsg_CmdIDs_ZDO_NWK_ADDR_REQ:
      resend = processZdoNwkAddrReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_IEEEADDR_REQUEST)
    case zstackmsg_CmdIDs_ZDO_IEEE_ADDR_REQ:
      resend = processZdoIeeeAddrReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_NODEDESC_REQUEST)
    case zstackmsg_CmdIDs_ZDO_NODE_DESC_REQ:
      resend = processZdoNodeDescReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_MATCH_REQUEST)
    case zstackmsg_CmdIDs_ZDO_MATCH_DESC_REQ:
      resend = processZdoMatchDescReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_POWERDESC_REQUEST)
    case zstackmsg_CmdIDs_ZDO_POWER_DESC_REQ:
      resend = processZdoPowerDescReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_SIMPLEDESC_REQUEST)
    case zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_REQ:
      resend = processZdoSimpleDescReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_ACTIVEEP_REQUEST)
    case zstackmsg_CmdIDs_ZDO_ACTIVE_ENDPOINT_REQ:
      resend = processZdoActiveEndpointsReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_COMPLEXDESC_REQUEST)
    case zstackmsg_CmdIDs_ZDO_COMPLEX_DESC_REQ:
      resend = processZdoComplexDescReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_SERVERDISC_REQUEST)
    case zstackmsg_CmdIDs_ZDO_SERVER_DISC_REQ:
      resend = processZdoServerDiscReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_ENDDEVICEBIND_REQUEST)
    case zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_REQ:
      resend = processZdoEndDeviceBindReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_BIND_UNBIND_REQUEST)
    case zstackmsg_CmdIDs_ZDO_BIND_REQ:
      resend = processZdoBindReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_BIND_UNBIND_REQUEST)
    case zstackmsg_CmdIDs_ZDO_UNBIND_REQ:
      resend = processZdoUnbindReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_MGMT_NWKDISC_REQUEST)
    case zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_REQ:
      resend = processZdoMgmtNwkDiscReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_MGMT_LQI_REQUEST)
    case zstackmsg_CmdIDs_ZDO_MGMT_LQI_REQ:
      resend = processZdoMgmtLqiReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_MGMT_RTG_REQUEST)
    case zstackmsg_CmdIDs_ZDO_MGMT_RTG_REQ:
      resend = processZdoMgmtRtgReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_MGMT_BIND_REQUEST)
    case zstackmsg_CmdIDs_ZDO_MGMT_BIND_REQ:
      resend = processZdoMgmtBindReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_MGMT_LEAVE_REQUEST)
    case zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_REQ:
      resend = processZdoMgmtLeaveReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_MGMT_JOINDIRECT_REQUEST)
    case zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_REQ:
      resend = processZdoMgmtDirectJoinReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_MGMT_PERMIT_JOIN_REQUEST)
    case zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_REQ:
      resend = processZdoMgmtPermitJoinReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_MGMT_NWKUPDATE_REQUEST)
    case zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_REQ:
      resend = processZdoMgmtNwkUpdateReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_ENDDEVICE_ANNCE)
    case zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE_REQ:
      resend = processZdoDeviceAnnounceReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_USERDESCSET_REQUEST)
    case zstackmsg_CmdIDs_ZDO_USER_DESCR_SET_REQ:
      resend = processZdoUserDescSetReq( srcEntityID, pMsg );
      break;
#endif

#if defined (ZDO_USERDESC_REQUEST)
    case zstackmsg_CmdIDs_ZDO_USER_DESC_REQ:
      resend = processZdoUserDescReq( srcEntityID, pMsg );
      break;
#endif

    default:
      pReq->hdr.status = zstack_ZStatusValues_ZUnsupportedMode;
      break;
  }

  return (resend);
}

/**************************************************************************************************
 * @fn      processAfDataConfirm
 *
 * @brief   process incoming AF Data Confirm
 *
 * @param   pkt - pointer to data confirm message
 *
 * @return  none
 */
static void processAfDataConfirm( afDataConfirm_t *pkt )
{
  zstackmsg_afDataConfirmInd_t *pReq;
  epItem_t *pItem;

  pReq = (zstackmsg_afDataConfirmInd_t *)ICall_allocMsg( sizeof(zstackmsg_afDataConfirmInd_t) );
  if ( pReq == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pReq, 0, sizeof(zstackmsg_afDataConfirmInd_t) );

  pReq->hdr.event = zstackmsg_CmdIDs_AF_DATA_CONFIRM_IND;
  pReq->hdr.status = zstack_ZStatusValues_ZSuccess;

  pReq->req.endpoint = pkt->endpoint;
  pReq->req.status = (zstack_ZStatusValues)pkt->hdr.status;
  pReq->req.transID = pkt->transID;

  pItem = epTableFindEntryEP( pkt->endpoint );
  if ( pItem )
  {
    // Send to a subscriber
    (void)ICall_send( ZStackEntity, pItem->connection, ICALL_MSG_FORMAT_KEEP,
          (void *)pReq );
  }
}

/**************************************************************************************************
 * @fn      processAfReflectErrorInd
 *
 * @brief   process incoming AF Reflect Error Indication
 *
 * @param   pkt - pointer to AF Reflect Error message
 *
 * @return  none
 */
static void processAfReflectErrorInd( afReflectError_t *pkt )
{
  zstackmsg_afReflectErrorInd_t *pReq;
  epItem_t *pItem;

  pReq = (zstackmsg_afReflectErrorInd_t *)ICall_allocMsg( sizeof(zstackmsg_afReflectErrorInd_t) );
  if ( pReq == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pReq, 0, sizeof(zstackmsg_afReflectErrorInd_t) );

  pReq->hdr.event = zstackmsg_CmdIDs_AF_REFLECT_ERROR_IND;
  pReq->hdr.status = zstack_ZStatusValues_ZSuccess;

  pReq->req.status = (zstack_ZStatusValues)pkt->hdr.status;

  pReq->req.dstAddr.endpoint = pkt->endpoint;
  if ( pkt->dstAddrMode == zstack_AFAddrMode_GROUP )
  {
    pReq->req.dstAddr.addrMode = zstack_AFAddrMode_GROUP;
  }
  else
  {
    pReq->req.dstAddr.addrMode = zstack_AFAddrMode_SHORT;
  }
  pReq->req.dstAddr.addr.shortAddr = pkt->dstAddr;

  pReq->req.transID = pkt->transID;

  pItem = epTableFindEntryEP( pkt->endpoint );
  if ( pItem )
  {
    // Send to a subscriber
    (void)ICall_send( ZStackEntity, pItem->connection, ICALL_MSG_FORMAT_KEEP, (void *)pReq );
  }
}

/**************************************************************************************************
 * @fn      processAfIncomingMsgInd
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   pkt - pointer to incoming packet
 *
 * @return  none
 */
static void processAfIncomingMsgInd( afIncomingMSGPacket_t *pkt )
{
  zstackmsg_afIncomingMsgInd_t *pReq;
  epItem_t *pItem;

  pReq = (zstackmsg_afIncomingMsgInd_t *)ICall_allocMsg( sizeof(zstackmsg_afIncomingMsgInd_t) );
  if ( pReq == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pReq, 0, sizeof(zstackmsg_afIncomingMsgInd_t) );

  pReq->hdr.event = zstackmsg_CmdIDs_AF_INCOMING_MSG_IND;
  pReq->hdr.status = 0;

  pReq->req.srcAddr.addrMode = (zstack_AFAddrMode)pkt->srcAddr.addrMode;

  if ( pReq->req.srcAddr.addrMode == zstack_AFAddrMode_EXT )
  {
    osal_memcpy( &(pReq->req.srcAddr.addr.extAddr),
          pkt->srcAddr.addr.extAddr, Z_EXTADDR_LEN );
  }
  else
  {
    pReq->req.srcAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
  }

  pReq->req.srcAddr.endpoint = pkt->srcAddr.endPoint;
  pReq->req.srcAddr.panID = pkt->srcAddr.panId;

  pReq->req.groupID = pkt->groupId;
  pReq->req.clusterId = pkt->clusterId;
  pReq->req.macDestAddr = pkt->macDestAddr;
  pReq->req.endpoint = pkt->endPoint;
  pReq->req.wasBroadcast = pkt->wasBroadcast;
  pReq->req.securityUse = pkt->SecurityUse;
  pReq->req.linkQuality = pkt->LinkQuality;
  pReq->req.correlation = pkt->correlation;
  pReq->req.rssi = pkt->rssi;
  pReq->req.timestamp = pkt->timestamp;
  pReq->req.nwkSeqNum = pkt->nwkSeqNum;
  pReq->req.macSrcAddr = pkt->macSrcAddr;
  pReq->req.transSeqNum = pkt->cmd.TransSeqNumber;
  pReq->req.radius = pkt->radius;
  pReq->req.n_payload = pkt->cmd.DataLength;
  pReq->req.pPayload = ICall_malloc( pkt->cmd.DataLength );
  if ( pReq->req.pPayload )
  {
    osal_memcpy( pReq->req.pPayload, pkt->cmd.Data, pkt->cmd.DataLength );
  }

  pItem = epTableFindEntryEP( pkt->endPoint );
  if ( pItem )
  {
    // Send to a subscriber
    (void)ICall_send( ZStackEntity, pItem->connection,
          ICALL_MSG_FORMAT_KEEP, (void *)pReq );
  }
}

/**************************************************************************************************
 * @fn      zsProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   inMsg - incoming ZDO message
 *
 * @return  none
 */
static void zsProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case Device_annce:
      {
        ZDO_DeviceAnnce_t devAnn;

        osal_memset( &devAnn, 0, sizeof(ZDO_DeviceAnnce_t) );

        ZDO_ParseDeviceAnnce( inMsg, &devAnn );
        sendDeviceAnnounce( inMsg->srcAddr.addr.shortAddr, &devAnn );
      }
      break;

#if defined (ZDO_NWKADDR_REQUEST)
    case NWK_addr_rsp:
      {
        ZDO_NwkIEEEAddrResp_t *pAddrRsp = ZDO_ParseAddrRsp( inMsg );
        if ( pAddrRsp )
        {
          sendNwkAddrRsp( inMsg->srcAddr.addr.shortAddr, pAddrRsp );

          osal_mem_free( pAddrRsp );
        }
      }
      break;
#endif

#if defined (ZDO_IEEEADDR_REQUEST)
    case IEEE_addr_rsp:
      {
        ZDO_NwkIEEEAddrResp_t *pAddrRsp = ZDO_ParseAddrRsp( inMsg );
        if ( pAddrRsp )
        {
          sendIeeeAddrRsp( inMsg->srcAddr.addr.shortAddr, pAddrRsp );

          osal_mem_free( pAddrRsp );
        }
      }
      break;
#endif

#if defined (ZDO_NODEDESC_REQUEST)
    case Node_Desc_rsp:
      {
        ZDO_NodeDescRsp_t ndRsp;

        osal_memset( &ndRsp, 0, sizeof(ZDO_NodeDescRsp_t) );

        ZDO_ParseNodeDescRsp( inMsg, &ndRsp );
        sendNodeDescRsp( inMsg->srcAddr.addr.shortAddr, &ndRsp );
      }
      break;
#endif

#if defined (ZDO_POWERDESC_REQUEST)
    case Power_Desc_rsp:
      {
        ZDO_PowerRsp_t powerRsp;

        osal_memset( &powerRsp, 0, sizeof(ZDO_PowerRsp_t) );

        ZDO_ParsePowerDescRsp( inMsg, &powerRsp );
        sendPowerDescRsp( inMsg->srcAddr.addr.shortAddr, &powerRsp );
      }
      break;
#endif

#if defined (ZDO_SIMPLEDESC_REQUEST)
    case Simple_Desc_rsp:
      {
        ZDO_SimpleDescRsp_t simpleRsp;

        osal_memset( &simpleRsp, 0, sizeof(ZDO_SimpleDescRsp_t) );

        ZDO_ParseSimpleDescRsp( inMsg, &simpleRsp );
        sendSimpleDescRsp( inMsg->srcAddr.addr.shortAddr, &simpleRsp );
      }
      break;
#endif

#if defined (ZDO_ACTIVEEP_REQUEST)
    case Active_EP_rsp:
      {
        ZDO_ActiveEndpointRsp_t *epRsp = ZDO_ParseEPListRsp( inMsg );
        if ( epRsp )
        {
          sendActiveEPRsp( inMsg->srcAddr.addr.shortAddr, epRsp );

          osal_mem_free( epRsp );
        }
      }
      break;
#endif

#if defined (ZDO_MATCH_REQUEST)
    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *epRsp = ZDO_ParseEPListRsp( inMsg );
        if ( epRsp )
        {
          sendMatchDescRsp( inMsg->srcAddr.addr.shortAddr, epRsp );

          osal_mem_free( epRsp );
        }
      }
      break;
#endif

#if defined (ZDO_USERDESC_REQUEST)
    case User_Desc_rsp:
      {
        ZDO_UserDescRsp_t *udRsp = ZDO_ParseUserDescRsp( inMsg );
        if ( udRsp )
        {
          sendUserDescRsp( inMsg->srcAddr.addr.shortAddr, udRsp );

          osal_mem_free( udRsp );
        }
      }
      break;
#endif

#if defined (ZDO_SERVERDISC_REQUEST)
    case Server_Discovery_rsp:
      {
        ZDO_ServerDiscRsp_t sdRsp;

        osal_memset( &sdRsp, 0, sizeof(ZDO_ServerDiscRsp_t) );

        ZDO_ParseServerDiscRsp( inMsg, &sdRsp );
        sendServerDiscRsp( inMsg->srcAddr.addr.shortAddr, &sdRsp );
      }
      break;
#endif

#if defined (ZDO_BIND_UNBIND_REQUEST)
    case Bind_rsp:
      {
        uint8 result = ZDO_ParseBindRsp( inMsg );
        sendBindRsp( inMsg->srcAddr.addr.shortAddr, &result );
      }
      break;
#endif

#if defined (ZDO_ENDDEVICEBIND_REQUEST)
    case End_Device_Bind_rsp:
      {
        uint8 result = ZDO_ParseBindRsp( inMsg );
        sendEndDeviceBindRsp( inMsg->srcAddr.addr.shortAddr, &result );
      }
      break;
#endif

#if defined (ZDO_BIND_UNBIND_REQUEST)
    case Unbind_rsp:
      {
        uint8 result = ZDO_ParseBindRsp( inMsg );
        sendUnbindRsp( inMsg->srcAddr.addr.shortAddr, &result );
      }
      break;
#endif

#if defined (ZDO_MGMT_NWKDISC_REQUEST)
    case Mgmt_NWK_Disc_rsp:
      {
        ZDO_MgmNwkDiscRsp_t *pNwkDiscRsp = ZDO_ParseMgmNwkDiscRsp( inMsg );
        if ( pNwkDiscRsp )
        {
          sendMgmtNwkDiscRsp( inMsg->srcAddr.addr.shortAddr, pNwkDiscRsp );

          osal_mem_free( pNwkDiscRsp );
        }
      }
      break;
#endif

#if defined (ZDO_MGMT_LQI_REQUEST)
    case Mgmt_Lqi_rsp:
      {
        ZDO_MgmtLqiRsp_t *pLqiRsp = ZDO_ParseMgmtLqiRsp( inMsg );
        if ( pLqiRsp )
        {
          sendMgmtLqiRsp( inMsg->srcAddr.addr.shortAddr, pLqiRsp );

          osal_mem_free( pLqiRsp );
        }
      }
      break;
#endif

#if defined (ZDO_MGMT_RTG_REQUEST)
    case Mgmt_Rtg_rsp:
      {
        ZDO_MgmtRtgRsp_t *pRtgRsp = ZDO_ParseMgmtRtgRsp( inMsg );
        if ( pRtgRsp )
        {
          sendMgmtRtgRsp( inMsg->srcAddr.addr.shortAddr, pRtgRsp );

          osal_mem_free( pRtgRsp );
        }
      }
      break;
#endif

#if defined (ZDO_MGMT_BIND_REQUEST)
    case Mgmt_Bind_rsp:
      {
        ZDO_MgmtBindRsp_t *pBindRsp = ZDO_ParseMgmtBindRsp( inMsg );
        if ( pBindRsp )
        {
          sendMgmtBindRsp( inMsg->srcAddr.addr.shortAddr, pBindRsp );

          osal_mem_free( pBindRsp );
        }
      }
      break;
#endif

#if defined (ZDO_MGMT_LEAVE_REQUEST)
    case Mgmt_Leave_rsp:
      {
        uint8 result = ZDO_ParseMgmtLeaveRsp( inMsg );
        sendMgmtLeaveRsp( inMsg->srcAddr.addr.shortAddr, &result );
      }
      break;
#endif

#if defined (ZDO_MGMT_JOINDIRECT_REQUEST)
    case Mgmt_Direct_Join_rsp:
      {
        uint8 result = ZDO_ParseMgmtDirectJoinRsp( inMsg );
        sendMgmtDirectJoinRsp( inMsg->srcAddr.addr.shortAddr, &result );
      }
      break;
#endif

#if defined (ZDO_MGMT_PERMIT_JOIN_REQUEST)
    case Mgmt_Permit_Join_rsp:
      {
        uint8 result = ZDO_ParseMgmtPermitJoinRsp( inMsg );
        sendMgmtPermitJoinRsp( inMsg->srcAddr.addr.shortAddr, &result );
      }
      break;
#endif

#if defined (ZDO_MGMT_NWKUPDATE_REQUEST)
    case Mgmt_NWK_Update_notify:
      {
        ZDO_MgmtNwkUpdateNotify_t *pNwkUpdateNotifyRsp =
            ZDO_ParseMgmtNwkUpdateNotify( inMsg );
        if ( pNwkUpdateNotifyRsp )
        {
          sendMgmtNwkUpdateNotify( inMsg->srcAddr.addr.shortAddr,
              pNwkUpdateNotifyRsp );

          osal_mem_free( pNwkUpdateNotifyRsp );
        }
      }
      break;
#endif

    // TBD: Not implemented yet
    case Complex_Desc_rsp:
    case Discovery_Cache_rsp:
    default:
      break;
  }
}

/**************************************************************************************************
 * @fn          epTableAddNewEntry
 *
 * @brief       Add entry to the list.  It will add to the end of the list.
 *
 * @param       newEntry - pointer to the entry
 *
 * @return      TRUE if added, FALSE if MAX reached
 */
static uint8 epTableAddNewEntry( epItem_t *newEntry )
{
  epItem_t *srch;

  if ( epTableNumEntries( ) > maxSupportedEndpoints )
  {
    return (FALSE);
  }

  srch = pEpTableHdr;

  while ( srch && srch->next )
  {
    srch = srch->next;
  }

  newEntry->next = (epItem_t *)NULL;

  if ( srch )
  {
    srch->next = newEntry;
  }
  else
  {
    pEpTableHdr = newEntry;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          epTableFindEntryEP
 *
 * @brief       Search list for endpoint and return pointer to record
 *
 * @param       ep - endpoint to find
 *
 * @return      pointer to entry record, NULL if not found
 */
static epItem_t *epTableFindEntryEP( uint8 ep )
{
  epItem_t *srch;

  srch = pEpTableHdr;

  while ( srch )
  {
    if ( srch->epDesc.endPoint == ep )
    {
      return (srch);
    }

    srch = srch->next;
  }

  return (NULL);
}

/**************************************************************************************************
 * @fn          epTableFindEntryConnection
 *
 * @brief       Search list for connection and return pointer to record
 *
 * @param       connection - connection to find
 *
 * @return      pointer to entry record, NULL if not found
 */
static epItem_t *epTableFindEntryConnection( int connection )
{
  epItem_t *srch;

  srch = pEpTableHdr;

  while ( srch )
  {
    if ( srch->connection == connection )
    {
      return (srch);
    }

    srch = srch->next;
  }

  return (NULL);
}

/**************************************************************************************************
 * @fn          epTableRemoveEntry
 *
 * @brief       Remove an entry from the list and release its memory.
 *
 * @param       entry - pointer to entry
 *
 * @return      none
 */
static void epTableRemoveEntry( epItem_t *entry )
{
  epItem_t *srch;
  epItem_t *prev;

  // Is this a real entry and is there anything in the list
  if ( entry && pEpTableHdr )
  {
    srch = pEpTableHdr;
    prev = (epItem_t *)NULL;

    while ( srch && srch != entry )
    {
      prev = srch;
      srch = srch->next;
    }

    // Found?
    if ( srch == entry )
    {
      // Update the list
      if ( entry == pEpTableHdr )
      {
        // First entry
        pEpTableHdr = entry->next;
      }
      else
      {
        prev->next = entry->next;
      }

      // Release the entry's memory
      freeEpItem( entry );
    }
  }
}

/**************************************************************************************************
 * @fn          epTableNumEntries
 *
 * @brief       Get the number of entries currently in the list.
 *
 * @param       none
 *
 * @return      number of entries
 */
static uint8 epTableNumEntries( void )
{
  epItem_t *srch;
  uint8 cnt = 0;

  srch = pEpTableHdr;

  while ( srch )
  {
    cnt++;
    srch = srch->next;
  }

  return (cnt);
}

/**************************************************************************************************
 * @fn          freeEpItem
 *
 * @brief       Free the endpoint item
 *
 * @param       none
 *
 * @return      number of entries
 */
static void freeEpItem( epItem_t *pItem )
{
  if ( pItem )
  {
    osal_mem_free( pItem );
  }
}

/***************************************************************************************************
 * @fn          buildMsgCapInfo
 *
 * @brief       Convert from bitmask byte to ZStack Msg capInfo
 *
 * @param       cInfo - source
 * @param       pPBcapInfo - destination
 *
 * @return      None
 */
static void buildMsgCapInfo( uint8 cInfo, zstack_CapabilityInfo_t *pPBcapInfo )
{
  if ( cInfo & CAPABLE_PAN_COORD )
  {
    pPBcapInfo->panCoord = 1;
  }

  if ( cInfo & CAPABLE_FFD )
  {
    pPBcapInfo->ffd = 1;
  }

  if ( cInfo & CAPABLE_MAINS_POWER )
  {
    pPBcapInfo->mainsPower = 1;
  }

  if ( cInfo & CAPABLE_RX_ON_IDLE )
  {
    pPBcapInfo->rxOnWhenIdle = 1;
  }

  if ( cInfo & CAPABLE_SECURITY )
  {
    pPBcapInfo->security = 1;
  }
}

#if defined (ZDO_SERVERDISC_REQUEST) || defined (ZDO_NODEDESC_REQUEST)
/***************************************************************************************************
 * @fn          buildMsgServerCap
 *
 * @brief       Convert from bitmask byte to ZStack Message ServerCapabilities
 *
 * @param       cInfo - source
 * @param       pPBsrvCap - destination
 *
 * @return      None
 */
static void buildMsgServerCap( uint16 sInfo, zstack_ServerCapabilities_t *pPBsrvCap )
{
  if ( sInfo & PRIM_TRUST_CENTER )
  {
    pPBsrvCap->primaryTrustCenter = 1;
  }

  if ( sInfo & BKUP_TRUST_CENTER )
  {
    pPBsrvCap->backupTrustCenter = 1;
  }

  if ( sInfo & PRIM_BIND_TABLE )
  {
    pPBsrvCap->primaryBindingTableCache = 1;
  }

  if ( sInfo & BKUP_BIND_TABLE )
  {
    pPBsrvCap->backupBindingTableCache = 1;
  }

  if ( sInfo & PRIM_DISC_TABLE )
  {
    pPBsrvCap->primaryDiscoveryCache = 1;
  }

  if ( sInfo & BKUP_DISC_TABLE )
  {
    pPBsrvCap->backupDiscoveryCache = 1;
  }

  if ( sInfo & NETWORK_MANAGER )
  {
    pPBsrvCap->networkManager = 1;
  }
}
#endif // ZDO_SERVERDISC_REQUEST || ZDO_NODEDESC_REQUEST

#if defined (ZDO_POWERDESC_REQUEST)
/***************************************************************************************************
 * @fn          buildMsgPowerSource
 *
 * @brief       Convert from bitmask byte to ZStack Message PowerSource
 *
 * @param       cInfo - source
 * @param       pPBsrvCap - destination
 *
 * @return      None
 */
static void buildMsgPowerSource( uint8 pInfo, zstack_PowerSource_t *pPBpwrSrc )
{
  if ( pInfo & NODEAVAILPWR_MAINS )
  {
    pPBpwrSrc->mains = 1;
  }

  if ( pInfo & NODEAVAILPWR_RECHARGE )
  {
    pPBpwrSrc->recharge = 1;
  }

  if ( pInfo & NODEAVAILPWR_DISPOSE )
  {
    pPBpwrSrc->dispose = 1;
  }
}
#endif // ZDO_POWERDESC_REQUEST

/***************************************************************************************************
 * @fn          convertMsgTransOptions
 *
 * @brief       Convert PB TransOptions data type to uint8 txOptions
 *
 * @param       pOptions - TransOptions pointer
 *
 * @return      txOptions
 */
static uint8 convertMsgTransOptions( zstack_TransOptions_t *pOptions )
{
  uint8 options = 0;

  if ( pOptions->wildcardProfileID )
  {
    options |= AF_WILDCARD_PROFILEID;
  }

  if ( pOptions->ackRequest )
  {
    options |= AF_ACK_REQUEST;
  }

  if ( pOptions->limitConcentrator )
  {
    options |= AF_LIMIT_CONCENTRATOR;
  }

  if ( pOptions->suppressRouteDisc )
  {
    options |= AF_SUPRESS_ROUTE_DISC_NETWORK;
  }

  if ( pOptions->apsSecurity )
  {
    options |= AF_EN_SECURITY;
  }

  if ( pOptions->skipRouting )
  {
    options |= AF_SKIP_ROUTING;
  }

  return (options);
}

#if defined (ZDO_SERVERDISC_REQUEST)
/***************************************************************************************************
 * @fn          convertMsgServerCapabilities
 *
 * @brief       Convert Zigbee API Message ServerCapabilities data type to uint16 server
 *              capabilities mask
 *
 * @param       pSrvCap - Server capabilities
 *
 * @return      Server Capabilities mask
 */
static uint16 convertMsgServerCapabilities( zstack_ServerCapabilities_t *pSrvCap )
{
  uint16 mask = 0;

  if ( pSrvCap->primaryTrustCenter )
  {
    mask |= PRIM_TRUST_CENTER;
  }

  if ( pSrvCap->backupTrustCenter )
  {
    mask |= BKUP_TRUST_CENTER;
  }

  if ( pSrvCap->primaryBindingTableCache )
  {
    mask |= PRIM_BIND_TABLE;
  }

  if ( pSrvCap->backupBindingTableCache )
  {
    mask |= BKUP_BIND_TABLE;
  }

  if ( pSrvCap->primaryDiscoveryCache )
  {
    mask |= PRIM_DISC_TABLE;
  }

  if ( pSrvCap->backupDiscoveryCache )
  {
    mask |= BKUP_DISC_TABLE;
  }

  if ( pSrvCap->networkManager )
  {
    mask |= NETWORK_MANAGER;
  }

  return (mask);
}
#endif // ZDO_SERVERDISC_REQUEST

/***************************************************************************************************
 * @fn          convertCapabilityInfo
 *
 * @brief       Convert Zigbee API Message zstack_CapabilityInfo_t data type to uint8 capInfo
 *
 * @param       pMsgcapInfo - CapabilityInfo pointer
 *
 * @return      capInfo
 */
static uint8 convertCapabilityInfo( zstack_CapabilityInfo_t *pMsgcapInfo )
{
  uint8 capInfo = 0;

  if ( pMsgcapInfo->panCoord )
  {
    capInfo |= CAPABLE_PAN_COORD;
  }

  if ( pMsgcapInfo->ffd )
  {
    capInfo |= CAPABLE_FFD;
  }

  if ( pMsgcapInfo->mainsPower )
  {
    capInfo |= CAPABLE_MAINS_POWER;
  }

  if ( pMsgcapInfo->rxOnWhenIdle )
  {
    capInfo |= CAPABLE_RX_ON_IDLE;
  }

  if ( pMsgcapInfo->security )
  {
    capInfo |= CAPABLE_SECURITY;
  }

  return (capInfo);
}

/**************************************************************************************************
 * @fn          sendMsgToAllCBs
 *
 * @brief       Send a message to all that is subscribed to ZDO CB functions
 *
 * @param       cbMask - callback mask
 * @param       pFn - function pointer to call
 *
 * @return      none
 */
static void sendMsgToAllCBs( uint16 cbMask, void *pBuf, pfnZDOCB pFn )
{
  epItem_t *srch;

  srch = pEpTableHdr;

  while ( srch )
  {
    if ( (srch->zdoCBs & cbMask) == cbMask )
    {
      if ( pFn )
      {
        // Send the a subscriber
        pFn( srch->connection, pBuf );
      }
    }

    srch = srch->next;
  }
}

/**************************************************************************************************
 * @fn          sendMsgToAllCBMsgs
 *
 * @brief       Send a message to all that is subscribed to ZDO CB Messages
 *
 * @param       cbMask - callback mask
 * @param       srcAddr - source address of the message
 * @param       pBuf - pointer to incoming message
 *
 * @return      none
 */
static void sendMsgToAllCBMsgs( uint32 cbMask, uint16 srcAddr, void *pBuf, pfnZDOMsgCB pFn )
{
  epItem_t *srch;

  srch = pEpTableHdr;

  while ( srch )
  {
    if ( (srch->zdoRsps & cbMask) == cbMask )
    {
      // Send the a subscriber
      pFn( srch->connection, srcAddr, pBuf );
    }

    srch = srch->next;
  }
}

/**************************************************************************************************
 * @fn          zdoNwkDiscCnfCB
 *
 * @brief       callback function to handle Network Discovery Confirmation
 *
 * @param       pStr - pointer to beacon notification information
 *
 * @return      NULL
 */
static void *zdoNwkDiscCnfCB( void *pStr )
{
  epItem_t *pItem;

  sendMsgToAllCBs( ZS_ZDO_NWK_DISCOVERY_CNF_CBID, pStr, sendNwkDiscCnf );

  // Clear the scan information
  NLME_NwkDiscTerm( );

  // Remove ZDO callbacks for network discovery and beacon notification
  ZDO_DeregisterForZdoCB( ZDO_NWK_DISCOVERY_CNF_CBID );
  ZDO_DeregisterForZdoCB( ZDO_BEACON_NOTIFY_IND_CBID );

  // Remove all callback messages for network discovery and beacon notification
  pItem = pEpTableHdr;

  while ( pItem )
  {
    pItem->zdoCBs &= ~ZS_ZDO_BEACON_NOTIFY_IND_CBID;
    pItem->zdoCBs &= ~ZS_ZDO_NWK_DISCOVERY_CNF_CBID;

    pItem = pItem->next;
  }

  return (NULL);
}

/**************************************************************************************************
 * @fn          sendNwkDiscCnf
 *
 * @brief       Function to send Network Discovery Confirmation
 *
 * @param       dstID - Destination iCall entity ID
 * @param       pStr - pointer to etwork Discovery Confirmation information
 *
 * @return      none
 */
static void sendNwkDiscCnf( uint16 dstID, void *pStr )
{
  zstackmsg_zdoNwkDiscCnf_t *pNwkDiscCnf;
  uint8 nwkDiscStatus = *(uint8 *)pStr;

  pNwkDiscCnf = (zstackmsg_zdoNwkDiscCnf_t *)ICall_allocMsg( sizeof(zstackmsg_zdoNwkDiscCnf_t) );
  if ( pNwkDiscCnf == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pNwkDiscCnf, 0, sizeof(zstackmsg_zdoNwkDiscCnf_t) );

  pNwkDiscCnf->hdr.event = zstackmsg_CmdIDs_ZDO_NWK_DISC_CNF;
  pNwkDiscCnf->hdr.status = zstack_ZStatusValues_ZSuccess;

  pNwkDiscCnf->req.status = nwkDiscStatus;

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP,
        (void *)pNwkDiscCnf );
}

/**************************************************************************************************
 * @fn          zdoBeaconNotifyIndCB
 *
 * @brief       callback function to handle Beacon Notification Indications
 *
 * @param       pStr - pointer to beacon notification information
 *
 * @return      NULL
 */
static void *zdoBeaconNotifyIndCB( void *pStr )
{
  sendMsgToAllCBs( ZS_ZDO_BEACON_NOTIFY_IND_CBID, pStr, sendBeaconNotifyInd );

  return (NULL);
}

/**************************************************************************************************
 * @fn          sendBeaconNotifyInd
 *
 * @brief       Function to send Beacon Notification Indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       pStr - pointer to Beacon Notification information
 *
 * @return      none
 */
static void sendBeaconNotifyInd( uint16 dstID, void *pStr )
{
  zstackmsg_zdoBeaconNotifyInd_t *pBeaconInd;
  zdoBeaconInd_t *pBeacon = (zdoBeaconInd_t *)pStr;

  pBeaconInd =
        (zstackmsg_zdoBeaconNotifyInd_t *)ICall_allocMsg( sizeof(
          zstackmsg_zdoBeaconNotifyInd_t) );
  if ( pBeaconInd == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pBeaconInd, 0, sizeof(zstackmsg_zdoBeaconNotifyInd_t) );

  pBeaconInd->hdr.event = zstackmsg_CmdIDs_ZDO_BEACON_NOTIFY_IND;
  pBeaconInd->hdr.status = zstack_ZStatusValues_ZSuccess;

  pBeaconInd->req.sourceAddr = pBeacon->sourceAddr;
  pBeaconInd->req.panID = pBeacon->panID;
  pBeaconInd->req.logicalChannel = pBeacon->logicalChannel;
  pBeaconInd->req.permitJoining = pBeacon->permitJoining;
  pBeaconInd->req.routerCapacity = pBeacon->routerCapacity;
  pBeaconInd->req.deviceCapacity = pBeacon->deviceCapacity;
  pBeaconInd->req.protocolVersion = pBeacon->protocolVersion;
  pBeaconInd->req.stackProfile = pBeacon->stackProfile;
  pBeaconInd->req.lqi = pBeacon->LQI;
  pBeaconInd->req.depth = pBeacon->depth;
  pBeaconInd->req.updateID = pBeacon->updateID;
  osal_memcpy( &(pBeaconInd->req.extendedPANID), pBeacon->extendedPanID, Z_EXTADDR_LEN );

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP,
        (void *)pBeaconInd );
}

/**************************************************************************************************
 * @fn          zdoSrcRtgCB
 *
 * @brief       callback function to handle Source Route indication
 *
 * @param       pStr - pointer to source route information
 *
 * @return      NULL
 */
static void *zdoSrcRtgCB( void *pStr )
{
  sendMsgToAllCBs( ZS_ZDO_SRC_RTG_IND_CBID, pStr, sendZdoSrcRtgInd );

  return (NULL);
}

/**************************************************************************************************
 * @fn          sendZdoSrcRtgInd
 *
 * @brief       Function to send a ZDO Source Route indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       pStr - pointer to source route information
 *
 * @return      none
 */
static void sendZdoSrcRtgInd( uint16 dstID, void *pStr )
{
  zstackmsg_zdoSrcRtgInd_t *pSrcRtgInd;
  zdoSrcRtg_t *pSrcRtg = (zdoSrcRtg_t *)pStr;

  pSrcRtgInd = (zstackmsg_zdoSrcRtgInd_t *)ICall_allocMsg( sizeof(zstackmsg_zdoSrcRtgInd_t) );
  if ( pSrcRtgInd == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pSrcRtgInd, 0, sizeof(zstackmsg_zdoSrcRtgInd_t) );

  pSrcRtgInd->hdr.event = zstackmsg_CmdIDs_ZDO_SRC_RTG_IND;
  pSrcRtgInd->hdr.status = 0;

  pSrcRtgInd->req.pRelay = ICall_malloc( sizeof(uint16_t) * pSrcRtg->relayCnt );
  if ( pSrcRtgInd->req.pRelay == NULL )
  {
    ICall_freeMsg( pSrcRtgInd );
    return;
  }

  pSrcRtgInd->req.srcAddr = pSrcRtg->srcAddr;
  pSrcRtgInd->req.n_relay = pSrcRtg->relayCnt;
  if ( pSrcRtg->relayCnt )
  {
    int i;
    for ( i = 0; i < pSrcRtg->relayCnt; i++ )
    {
      pSrcRtgInd->req.pRelay[i] = pSrcRtg->pRelayList[i];
    }
  }

  // Send the a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pSrcRtgInd );
}

/**************************************************************************************************
 * @fn          zdoConcentratorIndCB
 *
 * @brief       callback function to handle Concentrator indication
 *
 * @param       pStr - pointer to concentrator information
 *
 * @return      NULL
 */
static void *zdoConcentratorIndCB( void *pStr )
{
  sendMsgToAllCBs( ZS_ZDO_CONCENTRATOR_IND_CBID, pStr, sendZdoConcentratorInd );

  return (NULL);
}

/**************************************************************************************************
 * @fn          sendZdoConcentratorInd
 *
 * @brief       function to send ZDO Concentrator indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       pStr - pointer to concentrator information
 *
 * @return      none
 */
static void sendZdoConcentratorInd( uint16 dstID, void *pStr )
{
  zstackmsg_zdoCncntrtrInd_t *pConInd;
  zdoConcentratorInd_t *pCon = (zdoConcentratorInd_t *)pStr;

  pConInd = (zstackmsg_zdoCncntrtrInd_t *)ICall_allocMsg( sizeof(zstackmsg_zdoCncntrtrInd_t) );
  if ( pConInd == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pConInd, 0, sizeof(zstackmsg_zdoCncntrtrInd_t) );

  pConInd->hdr.event = zstackmsg_CmdIDs_ZDO_CONCENTRATOR_IND;
  pConInd->hdr.status = 0;

  pConInd->req.nwkAddr = pCon->nwkAddr;
  pConInd->req.pktCost = pCon->pktCost;
  osal_memcpy( &(pConInd->req.ieeeAddr), pCon->extAddr, Z_EXTADDR_LEN );

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pConInd );
}

/**************************************************************************************************
 * @fn          zdoJoinCnfCB
 *
 * @brief       callback function to handle Join Confirm
 *
 * @param       pStr - pointer to join confirm information
 *
 * @return      NULL
 */
static void *zdoJoinCnfCB( void *pStr )
{
  sendMsgToAllCBs( ZS_ZDO_JOIN_CNF_CBID, pStr, sendJoinCnfInd );

  if ( manualJoin )
  {
    zdoJoinCnf_t *pJoinCnf = (zdoJoinCnf_t *)pStr;
    if ( pJoinCnf && pJoinCnf->status != ZSuccess )
    {
      // Force ZDApp to not process the join confirm
      ZDApp_ChangeState( DEV_HOLD );
    }
    manualJoin = FALSE;
  }

  return (NULL);
}

/**************************************************************************************************
 * @fn          sendJoinCnfInd
 *
 * @brief       Function to send Join Confirm Indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       pStr - pointer to join confirm information
 *
 * @return      none
 */
static void sendJoinCnfInd( uint16 dstID, void *pStr )
{
  zstackmsg_zdoJoinConf_t *pJoinCnfInd;
  zdoJoinCnf_t *pJoinCnf = (zdoJoinCnf_t *)pStr;

  pJoinCnfInd = (zstackmsg_zdoJoinConf_t *)ICall_allocMsg( sizeof(zstackmsg_zdoJoinConf_t) );
  if ( pJoinCnfInd == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pJoinCnfInd, 0, sizeof(zstackmsg_zdoJoinConf_t) );

  pJoinCnfInd->hdr.event = zstackmsg_CmdIDs_ZDO_JOIN_CNF;
  pJoinCnfInd->hdr.status = pJoinCnf->status;

  pJoinCnfInd->req.devAddr = pJoinCnf->deviceAddr;
  pJoinCnfInd->req.parentAddr = pJoinCnf->parentAddr;

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pJoinCnfInd );
}

/**************************************************************************************************
 * @fn          zdoLeaveIndCB
 *
 * @brief       callback function to handle Leave indication
 *
 * @param       pStr - pointer to leave information
 *
 * @return      NULL
 */
static void *zdoLeaveIndCB( void *pStr )
{
  sendMsgToAllCBs( ZS_ZDO_LEAVE_IND_CBID, pStr, sendLeaveInd );

  return (NULL);
}

/**************************************************************************************************
 * @fn          sendLeaveInd
 *
 * @brief       Function to send Leave Indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       pStr - pointer to leave information
 *
 * @return      none
 */
static void sendLeaveInd( uint16 dstID, void *pStr )
{
  zstackmsg_zdoLeaveInd_t *pLeave;
  NLME_LeaveInd_t *pLeaveInd = (NLME_LeaveInd_t *)pStr;

  pLeave = (zstackmsg_zdoLeaveInd_t *)ICall_allocMsg( sizeof(zstackmsg_zdoLeaveInd_t) );
  if ( pLeave == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pLeave, 0, sizeof(zstackmsg_zdoLeaveInd_t) );

  pLeave->hdr.event = zstackmsg_CmdIDs_ZDO_LEAVE_IND;
  pLeave->hdr.status = 0;

  pLeave->req.srcAddr = pLeaveInd->srcAddr;
  pLeave->req.request = pLeaveInd->request;
  pLeave->req.rejoin = pLeaveInd->rejoin;
  pLeave->req.removeChildren = pLeaveInd->removeChildren;
  osal_memcpy( &pLeave->req.extendedAddr, pLeaveInd->extAddr, Z_EXTADDR_LEN );

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pLeave );
}

/**************************************************************************************************
 * @fn          zdoPermitJoinIndCB
 *
 * @brief       callback function to handle Permit Join CB indication
 *
 * @param       pStr - pointer to permit join information
 *
 * @return      NULL
 */
static void *zdoPermitJoinIndCB( void *pStr )
{
  sendMsgToAllCBMsgs( ZS_DEV_PERMIT_JOIN_IND_CDID, 0, pStr, sendDevPermitJoinInd );

  return (NULL);
}

/**************************************************************************************************
 * @fn          sendDevPermitJoinInd
 *
 * @brief       Send a Device Permit Join Indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pStr - pointer to permit join information
 *
 * @return      none
 */
static void sendDevPermitJoinInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  uint8 duration = *( (uint8 *)pMsg );
  zstackmsg_devPermitJoinInd_t *pDevInd;

  pDevInd =
        (zstackmsg_devPermitJoinInd_t *)ICall_allocMsg( sizeof(zstackmsg_devPermitJoinInd_t) );
  if ( pDevInd == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pDevInd, 0, sizeof(zstackmsg_devPermitJoinInd_t) );

  pDevInd->hdr.event = zstackmsg_CmdIDs_DEV_PERMIT_JOIN_IND;
  pDevInd->hdr.status = 0;
  pDevInd->req.duration = duration;

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pDevInd );
}

/**************************************************************************************************
 * @fn          zdoTcDeviceIndCB
 *
 * @brief       callback function to handle Trust Center Device CB indication
 *
 * @param       pStr - pointer to Device information
 *
 * @return      NULL
 */
static void *zdoTcDeviceIndCB( void *pStr )
{
  sendMsgToAllCBMsgs( ZS_TC_DEVICE_IND_CDID, 0, pStr, sendTcDeviceInd );

  return (NULL);
}

/**************************************************************************************************
 * @fn          sendTcDeviceInd
 *
 * @brief       Send a ZDO TC Device indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to incoming message
 *
 * @return      none
 */
static void sendTcDeviceInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_TC_Device_t *pDev = (ZDO_TC_Device_t *)pMsg;
  zstackmsg_zdoTcDeviceInd_t *pDevInd;

  pDevInd = (zstackmsg_zdoTcDeviceInd_t *)ICall_allocMsg( sizeof(zstackmsg_zdoTcDeviceInd_t) );
  if ( pDevInd == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pDevInd, 0, sizeof(zstackmsg_zdoTcDeviceInd_t) );

  pDevInd->hdr.event = zstackmsg_CmdIDs_ZDO_TC_DEVICE_IND;
  pDevInd->hdr.status = 0;

  pDevInd->req.nwkAddr = pDev->nwkAddr;
  osal_memcpy( &pDevInd->req.extendedAddr, pDev->extAddr, Z_EXTADDR_LEN );
  pDevInd->req.parentAddr = pDev->parentAddr;

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pDevInd );
}

/**************************************************************************************************
 * @fn          sendDeviceAnnounce
 *
 * @brief       function to handle Device Announce
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendDeviceAnnounce( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_DEVICE_ANNOUNCE_CDID, srcAddr, pMsg, sendDeviceAnnounceInd );
}

/**************************************************************************************************
 * @fn          sendDeviceAnnounceInd
 *
 * @brief       Send a ZDO Device Announce indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendDeviceAnnounceInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_DeviceAnnce_t *pDevAnn = (ZDO_DeviceAnnce_t *)pMsg;
  zstackmsg_zdoDeviceAnnounceInd_t *pDevInd;

  pDevInd =
        (zstackmsg_zdoDeviceAnnounceInd_t *)ICall_allocMsg( sizeof(
          zstackmsg_zdoDeviceAnnounceInd_t) );
  if ( pDevInd == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pDevInd, 0, sizeof(zstackmsg_zdoDeviceAnnounceInd_t) );

  pDevInd->hdr.event = zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE;
  pDevInd->hdr.status = 0;

  pDevInd->req.srcAddr = srcAddr;
  pDevInd->req.devAddr = pDevAnn->nwkAddr;

  osal_memcpy( &pDevInd->req.devExtAddr, pDevAnn->extAddr, Z_EXTADDR_LEN );

  buildMsgCapInfo( pDevAnn->capabilities, &pDevInd->req.capInfo );

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pDevInd );
}

#if defined (ZDO_NWKADDR_REQUEST)
/**************************************************************************************************
 * @fn          sendNwkAddrRsp
 *
 * @brief       function to handle a ZDO Network Address Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendNwkAddrRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_NWK_ADDR_RSP_CDID, srcAddr, pMsg, sendNwkAddrRspInd );
}

/**************************************************************************************************
 * @fn          sendNwkAddrRspInd
 *
 * @brief       Send a ZDO Network Address Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendNwkAddrRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_NwkIEEEAddrResp_t *pAddrRsp = (ZDO_NwkIEEEAddrResp_t *)pMsg;
  zstackmsg_zdoNwkAddrRspInd_t *pNwkAddrRsp;

  pNwkAddrRsp =
        (zstackmsg_zdoNwkAddrRspInd_t *)ICall_allocMsg( sizeof(zstackmsg_zdoNwkAddrRspInd_t) );
  if ( pNwkAddrRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pNwkAddrRsp, 0, sizeof(zstackmsg_zdoNwkAddrRspInd_t) );

  pNwkAddrRsp->hdr.event = zstackmsg_CmdIDs_ZDO_NWK_ADDR_RSP;
  pNwkAddrRsp->hdr.status = 0;

  pNwkAddrRsp->rsp.status = (zstack_ZdpStatus)pAddrRsp->status;
  pNwkAddrRsp->rsp.nwkAddr = pAddrRsp->nwkAddr;
  osal_memcpy( &pNwkAddrRsp->rsp.ieeeAddr, pAddrRsp->extAddr, Z_EXTADDR_LEN );

  pNwkAddrRsp->rsp.n_assocDevList = pAddrRsp->numAssocDevs;
  if ( pNwkAddrRsp->rsp.n_assocDevList )
  {
    pNwkAddrRsp->rsp.pAssocDevList =
          (uint16_t *)ICall_malloc( (sizeof(uint16_t) * pAddrRsp->numAssocDevs) );
    if ( pNwkAddrRsp->rsp.pAssocDevList )
    {
      int i;
      for ( i = 0; i < pNwkAddrRsp->rsp.n_assocDevList; i++ )
      {
        pNwkAddrRsp->rsp.pAssocDevList[i] = pAddrRsp->devList[i];
      }
    }
  }

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pNwkAddrRsp );
}
#endif // ZDO_NWKADDR_REQUEST

#if defined (ZDO_IEEEADDR_REQUEST)
/**************************************************************************************************
 * @fn          sendIeeeAddrRsp
 *
 * @brief       function to handle a ZDO IEEE Address Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendIeeeAddrRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_IEEE_ADDR_RSP_CDID, srcAddr, pMsg, sendIeeeAddrRspInd );
}

/**************************************************************************************************
 * @fn          sendIeeeAddrRspInd
 *
 * @brief       Send a ZDO IEEE Address Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendIeeeAddrRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_NwkIEEEAddrResp_t *pAddrRsp = (ZDO_NwkIEEEAddrResp_t *)pMsg;
  zstackmsg_zdoIeeeAddrRspInd_t *pIeeeAddrRsp;

  pIeeeAddrRsp =
        (zstackmsg_zdoIeeeAddrRspInd_t *)ICall_allocMsg( sizeof(zstackmsg_zdoIeeeAddrRspInd_t) );
  if ( pIeeeAddrRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pIeeeAddrRsp, 0, sizeof(zstackmsg_zdoIeeeAddrRspInd_t) );

  pIeeeAddrRsp->hdr.event = zstackmsg_CmdIDs_ZDO_IEEE_ADDR_RSP;
  pIeeeAddrRsp->hdr.status = 0;

  pIeeeAddrRsp->rsp.status = (zstack_ZdpStatus)pAddrRsp->status;
  pIeeeAddrRsp->rsp.nwkAddr = pAddrRsp->nwkAddr;
  osal_memcpy( &pIeeeAddrRsp->rsp.ieeeAddr, pAddrRsp->extAddr, Z_EXTADDR_LEN );

  pIeeeAddrRsp->rsp.n_assocDevList = pAddrRsp->numAssocDevs;
  if ( pIeeeAddrRsp->rsp.n_assocDevList )
  {
    pIeeeAddrRsp->rsp.pAssocDevList =
          (uint16_t *)ICall_malloc( (sizeof(uint16_t) * pAddrRsp->numAssocDevs) );
    if ( pIeeeAddrRsp->rsp.pAssocDevList )
    {
      int i;
      for ( i = 0; i < pIeeeAddrRsp->rsp.n_assocDevList; i++ )
      {
        pIeeeAddrRsp->rsp.pAssocDevList[i] = pAddrRsp->devList[i];
      }
    }
  }

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pIeeeAddrRsp );
}
#endif // ZDO_IEEEADDR_REQUEST

#if defined (ZDO_NODEDESC_REQUEST)
/**************************************************************************************************
 * @fn          sendNodeDescRsp
 *
 * @brief       function to handle a ZDO Node Descriptor Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendNodeDescRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_NODE_DESC_RSP_CDID, srcAddr, pMsg, sendNodeDescRspInd );
}

/**************************************************************************************************
 * @fn          sendNodeDescRspInd
 *
 * @brief       Send a ZDO Node Descriptor Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendNodeDescRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_NodeDescRsp_t *pNdRsp = (ZDO_NodeDescRsp_t *)pMsg;
  zstackmsg_zdoNodeDescRspInd_t *pRsp;

  pRsp = (zstackmsg_zdoNodeDescRspInd_t *)ICall_allocMsg( sizeof(zstackmsg_zdoNodeDescRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoNodeDescRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_NODE_DESC_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)pNdRsp->status;
  pRsp->rsp.nwkAddrOfInterest = pNdRsp->nwkAddr;

  buildMsgCapInfo( pNdRsp->nodeDesc.CapabilityFlags, &pRsp->rsp.nodeDesc.capInfo );
  buildMsgServerCap( pNdRsp->nodeDesc.ServerMask, &pRsp->rsp.nodeDesc.serverMask );
  pRsp->rsp.nodeDesc.logicalType = (zstack_LogicalTypes)pNdRsp->nodeDesc.LogicalType;
  pRsp->rsp.nodeDesc.complexDescAvail = pNdRsp->nodeDesc.ComplexDescAvail;
  pRsp->rsp.nodeDesc.userDescAvail = pNdRsp->nodeDesc.UserDescAvail;
  pRsp->rsp.nodeDesc.apsFlags = pNdRsp->nodeDesc.APSFlags;
  pRsp->rsp.nodeDesc.freqBand = pNdRsp->nodeDesc.FrequencyBand;
  pRsp->rsp.nodeDesc.manufacturerCode =
        BUILD_UINT16( pNdRsp->nodeDesc.ManufacturerCode[0],
        pNdRsp->nodeDesc.ManufacturerCode[1] );
  pRsp->rsp.nodeDesc.maxBufferSize = pNdRsp->nodeDesc.MaxBufferSize;
  pRsp->rsp.nodeDesc.maxInTransferSize =
        BUILD_UINT16( pNdRsp->nodeDesc.MaxInTransferSize[0],
        pNdRsp->nodeDesc.MaxInTransferSize[1] );
  pRsp->rsp.nodeDesc.maxOutTransferSize =
        BUILD_UINT16( pNdRsp->nodeDesc.MaxOutTransferSize[0],
        pNdRsp->nodeDesc.MaxOutTransferSize[1] );
  pRsp->rsp.nodeDesc.descCap = pNdRsp->nodeDesc.DescriptorCapability;

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID,
        ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif

#if defined (ZDO_POWERDESC_REQUEST)
/**************************************************************************************************
 * @fn          sendPowerDescRsp
 *
 * @brief       function to handle a ZDO Power Descriptor Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendPowerDescRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_POWER_DESC_RSP_CDID, srcAddr, pMsg, sendPowerDescRspInd );
}

/**************************************************************************************************
 * @fn          sendPowerDescRspInd
 *
 * @brief       Send a ZDO Power Descriptor Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendPowerDescRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_PowerRsp_t *pPowerRsp = (ZDO_PowerRsp_t *)pMsg;
  zstackmsg_zdoPowerDescRspInd_t *pRsp;

  pRsp =
        (zstackmsg_zdoPowerDescRspInd_t *)ICall_allocMsg( sizeof(
          zstackmsg_zdoPowerDescRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoPowerDescRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_POWER_DESC_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)pPowerRsp->status;
  pRsp->rsp.nwkAddrOfInterest = pPowerRsp->nwkAddr;

  buildMsgPowerSource( pPowerRsp->pwrDesc.AvailablePowerSources,
        &pRsp->rsp.powerDesc.availPowerSource );
  buildMsgPowerSource( pPowerRsp->pwrDesc.CurrentPowerSource,
        &pRsp->rsp.powerDesc.currentPowerSource );

  pRsp->rsp.powerDesc.currentPowerLevel
    = (zstack_PowerLevel)pPowerRsp->pwrDesc.CurrentPowerSourceLevel;
  pRsp->rsp.powerDesc.powerMode = (zstack_PowerModes)pPowerRsp->pwrDesc.PowerMode;

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif

#if defined (ZDO_SIMPLEDESC_REQUEST)
/**************************************************************************************************
 * @fn          sendSimpleDescRsp
 *
 * @brief       function to handle a ZDO Simple Descriptor Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendSimpleDescRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_SIMPLE_DESC_RSP_CDID, srcAddr, pMsg, sendSimpleDescRspInd );
}

/**************************************************************************************************
 * @fn          sendSimpleDescRspInd
 *
 * @brief       Send a ZDO Simple Descriptor Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendSimpleDescRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  int i;
  ZDO_SimpleDescRsp_t *pSimpleRsp = (ZDO_SimpleDescRsp_t *)pMsg;
  zstackmsg_zdoSimpleDescRspInd_t *pRsp;

  pRsp =
        (zstackmsg_zdoSimpleDescRspInd_t *)ICall_allocMsg( sizeof(
          zstackmsg_zdoSimpleDescRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoSimpleDescRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)pSimpleRsp->status;
  pRsp->rsp.nwkAddrOfInterest = pSimpleRsp->nwkAddr;

  pRsp->rsp.simpleDesc.deviceID = pSimpleRsp->simpleDesc.AppDeviceId;
  pRsp->rsp.simpleDesc.deviceVer = pSimpleRsp->simpleDesc.AppDevVer;
  pRsp->rsp.simpleDesc.endpoint = pSimpleRsp->simpleDesc.EndPoint;
  pRsp->rsp.simpleDesc.profileID = pSimpleRsp->simpleDesc.AppProfId;
  pRsp->rsp.simpleDesc.n_inputClusters = pSimpleRsp->simpleDesc.AppNumInClusters;
  pRsp->rsp.simpleDesc.n_outputClusters = pSimpleRsp->simpleDesc.AppNumOutClusters;

  pRsp->rsp.simpleDesc.pInputClusters =
        (uint16_t *)ICall_malloc( (sizeof(uint16_t) * pSimpleRsp->simpleDesc.AppNumInClusters) );
  if ( pRsp->rsp.simpleDesc.pInputClusters )
  {
    for ( i = 0; i < pRsp->rsp.simpleDesc.n_inputClusters; i++ )
    {
      pRsp->rsp.simpleDesc.pInputClusters[i]
        = pSimpleRsp->simpleDesc.pAppInClusterList[i];
    }
  }

  pRsp->rsp.simpleDesc.pOutputClusters =
        (uint16_t *)ICall_malloc( (sizeof(uint16_t) * pSimpleRsp->simpleDesc.AppNumOutClusters) );
  if ( pRsp->rsp.simpleDesc.pOutputClusters )
  {
    for ( i = 0; i < pRsp->rsp.simpleDesc.n_outputClusters; i++ )
    {
      pRsp->rsp.simpleDesc.pOutputClusters[i] =
            pSimpleRsp->simpleDesc.pAppOutClusterList[i];
    }
  }

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif

#if defined (ZDO_ACTIVEEP_REQUEST)
/**************************************************************************************************
 * @fn          sendActiveEPRsp
 *
 * @brief       function to handle a ZDO Active Endpoint Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendActiveEPRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_ACTIVE_EP_RSP_CDID, srcAddr, pMsg, sendActiveEPRspInd );
}

/**************************************************************************************************
 * @fn          sendActiveEPRspInd
 *
 * @brief       Send a ZDO Active Endpoint Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendActiveEPRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_ActiveEndpointRsp_t *pActiveEPRsp = (ZDO_ActiveEndpointRsp_t *)pMsg;
  zstackmsg_zdoActiveEndpointsRspInd_t *pRsp;

  pRsp =
        (zstackmsg_zdoActiveEndpointsRspInd_t *)ICall_allocMsg( sizeof(
          zstackmsg_zdoActiveEndpointsRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoActiveEndpointsRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_ACTIVE_EP_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)pActiveEPRsp->status;
  pRsp->rsp.nwkAddrOfInterest = pActiveEPRsp->nwkAddr;

  pRsp->rsp.n_activeEPList = pActiveEPRsp->cnt;

  pRsp->rsp.pActiveEPList = (uint8_t *)ICall_malloc( (sizeof(uint8_t) * pActiveEPRsp->cnt) );
  if ( pRsp->rsp.pActiveEPList )
  {
    int i;
    for ( i = 0; i < pRsp->rsp.n_activeEPList; i++ )
    {
      pRsp->rsp.pActiveEPList[i] = pActiveEPRsp->epList[i];
    }
  }

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif

#if defined (ZDO_MATCH_REQUEST)
/**************************************************************************************************
 * @fn          sendMatchDescRsp
 *
 * @brief       function to handle a ZDO Match Descriptor Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMatchDescRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_MATCH_DESC_RSP_CDID, srcAddr, pMsg, sendMatchDescRspInd );
}

/**************************************************************************************************
 * @fn          sendMatchDescRspInd
 *
 * @brief       Send a ZDO Match Descriptor Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMatchDescRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_ActiveEndpointRsp_t *pActiveEPRsp = (ZDO_ActiveEndpointRsp_t *)pMsg;
  zstackmsg_zdoMatchDescRspInd_t *pRsp;

  pRsp =
        (zstackmsg_zdoMatchDescRspInd_t *)ICall_allocMsg( sizeof(
          zstackmsg_zdoMatchDescRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoMatchDescRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_MATCH_DESC_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)pActiveEPRsp->status;
  pRsp->rsp.nwkAddrOfInterest = pActiveEPRsp->nwkAddr;

  pRsp->rsp.n_matchList = pActiveEPRsp->cnt;

  pRsp->rsp.pMatchList = (uint8_t *)ICall_malloc( (sizeof(uint8_t) * pActiveEPRsp->cnt) );
  if ( pRsp->rsp.pMatchList )
  {
    int i;
    for ( i = 0; i < pRsp->rsp.n_matchList; i++ )
    {
      pRsp->rsp.pMatchList[i] = pActiveEPRsp->epList[i];
    }
  }

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif

#if defined (ZDO_USERDESC_REQUEST)
/**************************************************************************************************
 * @fn          sendUserDescRsp
 *
 * @brief       function to handle a ZDO User Descriptor Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendUserDescRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_USER_DESC_RSP_CDID, srcAddr, pMsg, sendUserDescRspInd );
}

/**************************************************************************************************
 * @fn          sendUserDescRspInd
 *
 * @brief       Send a ZDO User Descriptor Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendUserDescRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_UserDescRsp_t *pUdRsp = (ZDO_UserDescRsp_t *)pMsg;
  zstackmsg_zdoUserDescRspInd_t *pRsp;

  pRsp = (zstackmsg_zdoUserDescRspInd_t *)ICall_allocMsg( sizeof(zstackmsg_zdoUserDescRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoUserDescRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_USER_DESC_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)pUdRsp->status;
  pRsp->rsp.nwkAddrOfInterest = pUdRsp->nwkAddr;

  pRsp->rsp.n_desc = pUdRsp->length;

  pRsp->rsp.pDesc = ICall_malloc( pUdRsp->length );
  if ( pRsp->rsp.pDesc )
  {
    osal_memcpy( pRsp->rsp.pDesc, pUdRsp->desc, pUdRsp->length );
  }

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif

#if defined (ZDO_SERVERDISC_REQUEST)
/**************************************************************************************************
 * @fn          sendServerDiscRsp
 *
 * @brief       function to handle a ZDO Server Descriptor Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendServerDiscRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_SERVER_DISCOVERY_RSP_CDID, srcAddr,
        pMsg, sendServerDiscRspInd );
}

/**************************************************************************************************
 * @fn          sendServerDiscRspInd
 *
 * @brief       Send a ZDO Server Descriptor Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendServerDiscRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_ServerDiscRsp_t *pSdRsp = (ZDO_ServerDiscRsp_t *)pMsg;
  zstackmsg_zdoServerDiscoveryRspInd_t *pRsp;

  pRsp =
        (zstackmsg_zdoServerDiscoveryRspInd_t *)ICall_allocMsg( sizeof(
          zstackmsg_zdoServerDiscoveryRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoServerDiscoveryRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_SERVER_DISC_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)pSdRsp->status;
  //pRsp->rsp.nwkAddrOfInterest = pSdRsp->nwkAddr;

  buildMsgServerCap( pSdRsp->serverMask, &pRsp->rsp.serverCap );

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif

#if defined (ZDO_BIND_UNBIND_REQUEST)
/**************************************************************************************************
 * @fn          sendBindRsp
 *
 * @brief       function to handle a ZDO Bind Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendBindRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_BIND_RSP_CDID, srcAddr, pMsg, sendBindRspInd );
}

/**************************************************************************************************
 * @fn          sendBindRspInd
 *
 * @brief       Send a ZDO Bind Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendBindRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  uint8 result = *( (uint8 *)pMsg );
  zstackmsg_zdoBindRspInd_t *pRsp;

  pRsp = (zstackmsg_zdoBindRspInd_t *)ICall_allocMsg( sizeof(zstackmsg_zdoBindRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoBindRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_BIND_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)result;

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif

#if defined (ZDO_ENDDEVICEBIND_REQUEST)
/**************************************************************************************************
 * @fn          sendEndDeviceBindRsp
 *
 * @brief       function to handle a ZDO End Device Bind Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendEndDeviceBindRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_END_DEVICE_BIND_RSP_CDID, srcAddr,
        pMsg, sendEndDeviceBindRspInd );
}

/**************************************************************************************************
 * @fn          sendEndDeviceBindRspInd
 *
 * @brief       Send a ZDO End Device Bind Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendEndDeviceBindRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  uint8 result = *( (uint8 *)pMsg );
  zstackmsg_zdoEndDeviceBindRspInd_t *pRsp;

  pRsp =
        (zstackmsg_zdoEndDeviceBindRspInd_t *)ICall_allocMsg( sizeof(
          zstackmsg_zdoEndDeviceBindRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoEndDeviceBindRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)result;

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif

#if defined (ZDO_BIND_UNBIND_REQUEST)
/**************************************************************************************************
 * @fn          sendUnbindRsp
 *
 * @brief       function to handle a ZDO Unbind Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendUnbindRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_UNBIND_RSP_CDID, srcAddr, pMsg, sendUnbindRspInd );
}

/**************************************************************************************************
 * @fn          sendUnbindRspInd
 *
 * @brief       Send a ZDO Unbind Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendUnbindRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  uint8 result = *( (uint8 *)pMsg );
  zstackmsg_zdoUnbindRspInd_t *pRsp;

  pRsp = (zstackmsg_zdoUnbindRspInd_t *)ICall_allocMsg( sizeof(zstackmsg_zdoUnbindRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoUnbindRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_UNBIND_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)result;

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif

#if defined (ZDO_MGMT_NWKDISC_REQUEST)
/**************************************************************************************************
 * @fn          sendMgmtNwkDiscRsp
 *
 * @brief       function to handle a ZDO Management Network Discovery Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtNwkDiscRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_MGMT_NWK_DISC_RSP_CDID, srcAddr,
        pMsg, sendMgmtNwkDiscRspInd );
}

/**************************************************************************************************
 * @fn          sendMgmtNwkDiscRspInd
 *
 * @brief       Send a ZDO Management Network Discovery Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtNwkDiscRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_MgmNwkDiscRsp_t *pNwkDiscRsp = (ZDO_MgmNwkDiscRsp_t *)pMsg;
  zstackmsg_zdoMgmtNwkDiscRspInd_t *pRsp;

  pRsp =
        (zstackmsg_zdoMgmtNwkDiscRspInd_t *)ICall_allocMsg( sizeof(
          zstackmsg_zdoMgmtNwkDiscRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoMgmtNwkDiscRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)pNwkDiscRsp->status;

  pRsp->rsp.networkCount = pNwkDiscRsp->networkCount;
  pRsp->rsp.startIndex = pNwkDiscRsp->startIndex;
  pRsp->rsp.n_netList = pNwkDiscRsp->networkListCount;

  pRsp->rsp.pNetList =
        (zstack_nwkDiscItem_t *)ICall_malloc( (sizeof(zstack_nwkDiscItem_t) *
          pRsp->rsp.n_netList) );
  if ( pRsp->rsp.pNetList )
  {
    int x;
    zstack_nwkDiscItem_t *pDisc = pRsp->rsp.pNetList;
    for ( x = 0; x < pRsp->rsp.n_netList; x++, pDisc++ )
    {
      osal_memset( pDisc, 0, sizeof(zstack_nwkDiscItem_t) );

      osal_memcpy( &(pDisc->extendedPANID), pNwkDiscRsp->list[x].extendedPANID, Z_EXTADDR_LEN );
      pDisc->logicalChan = pNwkDiscRsp->list[x].logicalChannel;
      pDisc->stackProfile = pNwkDiscRsp->list[x].stackProfile;
      pDisc->version = pNwkDiscRsp->list[x].version;
      pDisc->beaconOrder = pNwkDiscRsp->list[x].beaconOrder;
      pDisc->superFrameOrder = pNwkDiscRsp->list[x].superFrameOrder;
      pDisc->permitJoin = pNwkDiscRsp->list[x].permitJoining;
    }
  }

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif // ZDO_MGMT_NWKDISC_REQUEST

#if defined (ZDO_MGMT_LQI_REQUEST)
/**************************************************************************************************
 * @fn          sendMgmtLqiRsp
 *
 * @brief       function to handle a ZDO Management LQI Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtLqiRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_MGMT_LQI_RSP_CDID, srcAddr, pMsg, sendMgmtLqiRspInd );
}

/**************************************************************************************************
 * @fn          sendMgmtLqiRspInd
 *
 * @brief       Send a ZDO Management LQI Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtLqiRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_MgmtLqiRsp_t *pLqiRsp = (ZDO_MgmtLqiRsp_t *)pMsg;
  zstackmsg_zdoMgmtLqiRspInd_t *pRsp;

  pRsp = (zstackmsg_zdoMgmtLqiRspInd_t *)ICall_allocMsg( sizeof(zstackmsg_zdoMgmtLqiRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoMgmtLqiRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_MGMT_LQI_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)pLqiRsp->status;

  pRsp->rsp.neighborLqiEntries = pLqiRsp->neighborLqiEntries;
  pRsp->rsp.startIndex = pLqiRsp->startIndex;

  pRsp->rsp.n_lqiList = pLqiRsp->neighborLqiCount;
  pRsp->rsp.pLqiList =
        (zstack_nwkLqiItem_t *)ICall_malloc( (sizeof(zstack_nwkLqiItem_t) *
          pRsp->rsp.n_lqiList) );
  if ( pRsp->rsp.pLqiList )
  {
    zstack_nwkLqiItem_t *pItem = pRsp->rsp.pLqiList;
    ZDP_MgmtLqiItem_t *pList = (ZDP_MgmtLqiItem_t *)pLqiRsp->list;
    int x;

    osal_memset( pRsp->rsp.pLqiList, 0,
          (sizeof(zstack_nwkLqiItem_t) * pRsp->rsp.n_lqiList) );

    for ( x = 0; x < pRsp->rsp.n_lqiList; x++, pItem++, pList++ )
    {
      osal_memcpy( &(pItem->extendedPANID), pList->extPanID, Z_EXTADDR_LEN );
      osal_memcpy( &(pItem->extendedAddr), pList->extAddr, Z_EXTADDR_LEN );
      pItem->panID = pList->panID;
      pItem->nwkAddr = pList->nwkAddr;
      pItem->deviceType = (zstack_LogicalTypes)pList->devType;
      pItem->rxOnWhenIdle = (zstack_RxOnWhenIdleTypes)pList->rxOnIdle;
      pItem->relationship = (zstack_RelationTypes)pList->relation;
      pItem->permit = (zstack_PermitJoinTypes)pList->permit;
      pItem->depth = pList->depth;
      pItem->rxLqi = pList->lqi;
    }
  }

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif // ZDO_MGMT_LQI_REQUEST

#if defined (ZDO_MGMT_RTG_REQUEST)
/**************************************************************************************************
 * @fn          sendMgmtRtgRsp
 *
 * @brief       function to handle a ZDO Management Routing Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtRtgRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_MGMT_RTG_RSP_CDID, srcAddr, pMsg, sendMgmtRtgRspInd );
}

/**************************************************************************************************
 * @fn          sendMgmtRtgRspInd
 *
 * @brief       Send a ZDO Management Routing Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtRtgRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_MgmtRtgRsp_t *pRtgRsp = (ZDO_MgmtRtgRsp_t *)pMsg;
  zstackmsg_zdoMgmtRtgRspInd_t *pRsp;

  pRsp = (zstackmsg_zdoMgmtRtgRspInd_t *)ICall_allocMsg( sizeof(zstackmsg_zdoMgmtRtgRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoMgmtRtgRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_MGMT_RTG_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)pRtgRsp->status;

  pRsp->rsp.rtgEntries = pRtgRsp->rtgCount;
  pRsp->rsp.startIndex = pRtgRsp->startIndex;
  pRsp->rsp.n_rtgList = pRtgRsp->rtgListCount;

  pRsp->rsp.pRtgList =
        (zstack_routeItem_t *)ICall_malloc( (sizeof(zstack_routeItem_t) * pRsp->rsp.n_rtgList) );
  if ( pRsp->rsp.pRtgList )
  {
    zstack_routeItem_t *pItem = (zstack_routeItem_t *)pRsp->rsp.pRtgList;
    int x;

    osal_memset( pRsp->rsp.pRtgList, 0,
          (sizeof(zstack_routeItem_t) * pRsp->rsp.n_rtgList) );

    for ( x = 0; x < pRsp->rsp.n_rtgList; x++, pItem++ )
    {
      pItem->dstAddr = pRtgRsp->list[x].dstAddress;
      pItem->nextHop = pRtgRsp->list[x].nextHopAddress;
      pItem->status = (zstack_RouteStatus)pRtgRsp->list[x].status;

      if ( pRtgRsp->list[x].options & ZDO_MGMT_RTG_ENTRY_MANYTOONE )
      {
        pItem->manyToOne = TRUE;
      }

      if ( pRtgRsp->list[x].options & ZDO_MGMT_RTG_ENTRY_ROUTE_RECORD_REQUIRED )
      {
        pItem->routeRecordRequired = TRUE;
      }

      if ( pRtgRsp->list[x].options & ZDO_MGMT_RTG_ENTRY_MEMORY_CONSTRAINED )
      {
        pItem->memoryConstrained = TRUE;
      }
    }
  }

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif // ZDO_MGMT_RTG_REQUEST

#if defined (ZDO_MGMT_BIND_REQUEST)
/**************************************************************************************************
 * @fn          sendMgmtBindRsp
 *
 * @brief       function to handle a ZDO Management Bind Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtBindRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_MGMT_BIND_RSP_CDID, srcAddr, pMsg, sendMgmtBindRspInd );
}

/**************************************************************************************************
 * @fn          sendMgmtBindRspInd
 *
 * @brief       Send a ZDO Management Bind Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtBindRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_MgmtBindRsp_t *pBindRsp = (ZDO_MgmtBindRsp_t *)pMsg;
  zstackmsg_zdoMgmtBindRspInd_t *pRsp;

  pRsp = (zstackmsg_zdoMgmtBindRspInd_t *)ICall_allocMsg( sizeof(zstackmsg_zdoMgmtBindRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoMgmtBindRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_MGMT_BIND_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)pBindRsp->status;

  pRsp->rsp.bindEntries = pBindRsp->bindingCount;
  pRsp->rsp.startIndex = pBindRsp->startIndex;
  pRsp->rsp.n_bindList = pBindRsp->bindingListCount;

  pRsp->rsp.pBindList =
        (zstack_bindItem_t *)ICall_malloc( (sizeof(zstack_bindItem_t) * pRsp->rsp.n_bindList) );
  if ( pRsp->rsp.pBindList )
  {
    zstack_bindItem_t *pItem = (zstack_bindItem_t *)pRsp->rsp.pBindList;
    int x;

    osal_memset( pRsp->rsp.pBindList, 0,
          (sizeof(zstack_bindItem_t) * pRsp->rsp.n_bindList) );

    for ( x = 0; x < pRsp->rsp.n_bindList; x++, pItem++ )
    {
      osal_memcpy( &(pItem->srcAddr), pBindRsp->list[x].srcAddr, Z_EXTADDR_LEN );
      pItem->srcEndpoint = pBindRsp->list[x].srcEP;
      pItem->clustedID = pBindRsp->list[x].clusterID;

      pItem->dstAddr.addrMode
        = (zstack_AFAddrMode)pBindRsp->list[x].dstAddr.addrMode;

      if ( pItem->dstAddr.addrMode == zstack_AFAddrMode_EXT )
      {
        osal_memcpy( &(pItem->dstAddr.addr.extAddr),
              pBindRsp->list[x].dstAddr.addr.extAddr, Z_EXTADDR_LEN );
      }
      else
      {
        pItem->dstAddr.addr.shortAddr =
              pBindRsp->list[x].dstAddr.addr.shortAddr;
      }

      pItem->dstAddr.endpoint = pBindRsp->list[x].dstEP;
    }
  }

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif // ZDO_MGMT_BIND_REQUEST

#if defined (ZDO_MGMT_LEAVE_REQUEST)
/**************************************************************************************************
 * @fn          sendMgmtLeaveRsp
 *
 * @brief       function to handle a ZDO Management Leave Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtLeaveRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_MGMT_LEAVE_RSP_CDID, srcAddr, pMsg, sendMgmtLeaveRspInd );
}

/**************************************************************************************************
 * @fn          sendMgmtLeaveRspInd
 *
 * @brief       Send a ZDO Management Leave Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtLeaveRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  uint8 result = *(uint8 *)pMsg;
  zstackmsg_zdoMgmtLeaveRspInd_t *pRsp;

  pRsp =
        (zstackmsg_zdoMgmtLeaveRspInd_t *)ICall_allocMsg( sizeof(
          zstackmsg_zdoMgmtLeaveRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoMgmtLeaveRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)result;

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif // ZDO_MGMT_LEAVE_REQUEST

#if defined (ZDO_MGMT_JOINDIRECT_REQUEST)
/**************************************************************************************************
 * @fn          sendMgmtDirectJoinRsp
 *
 * @brief       function to handle a ZDO Management Direct Join Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtDirectJoinRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_MGMT_DIRECT_JOIN_RSP_CDID, srcAddr,
        pMsg, sendMgmtDirectJoinRspInd );
}

/**************************************************************************************************
 * @fn          sendMgmtDirectJoinRspInd
 *
 * @brief       Send a ZDO Management Direct Join Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtDirectJoinRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  uint8 result = *(uint8 *)pMsg;
  zstackmsg_zdoMgmtDirectJoinRspInd_t *pRsp;

  pRsp =
        (zstackmsg_zdoMgmtDirectJoinRspInd_t *)ICall_allocMsg( sizeof(
          zstackmsg_zdoMgmtDirectJoinRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoMgmtDirectJoinRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)result;

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif // ZDO_MGMT_JOINDIRECT_REQUEST

#if defined (ZDO_MGMT_PERMIT_JOIN_REQUEST)
/**************************************************************************************************
 * @fn          sendMgmtPermitJoinRsp
 *
 * @brief       function to handle a ZDO Management Permit Join Response
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtPermitJoinRsp( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_MGMT_PERMIT_JOIN_RSP_CDID, srcAddr,
        pMsg, sendMgmtPermitJoinRspInd );
}

/**************************************************************************************************
 * @fn          sendMgmtPermitJoinRspInd
 *
 * @brief       Send a ZDO Management Permit Join Response indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtPermitJoinRspInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  uint8 result = *(uint8 *)pMsg;
  zstackmsg_zdoMgmtPermitJoinRspInd_t *pRsp;

  pRsp =
        (zstackmsg_zdoMgmtPermitJoinRspInd_t *)ICall_allocMsg( sizeof(
          zstackmsg_zdoMgmtPermitJoinRspInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoMgmtPermitJoinRspInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_RSP;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)result;

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif // ZDO_MGMT_PERMIT_JOIN_REQUEST

#if defined (ZDO_MGMT_NWKUPDATE_REQUEST)
/**************************************************************************************************
 * @fn          sendMgmtNwkUpdateNotify
 *
 * @brief       function to handle a ZDO Management Network Update Notify
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtNwkUpdateNotify( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_ZDO_MGMT_NWK_UPDATE_NOTIFY_CDID, srcAddr,
        pMsg, sendMgmtNwkUpdateNotifyInd );
}

/**************************************************************************************************
 * @fn          sendMgmtNwkUpdateNotifyInd
 *
 * @brief       Send a ZDO Management Network Update Notify indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendMgmtNwkUpdateNotifyInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  ZDO_MgmtNwkUpdateNotify_t *pNotify = (ZDO_MgmtNwkUpdateNotify_t *)pMsg;
  zstackmsg_zdoMgmtNwkUpdateNotifyInd_t *pRsp;

  pRsp =
        (zstackmsg_zdoMgmtNwkUpdateNotifyInd_t *)ICall_allocMsg( sizeof(
          zstackmsg_zdoMgmtNwkUpdateNotifyInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_zdoMgmtNwkUpdateNotifyInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_NOTIFY;
  pRsp->hdr.status = 0;

  pRsp->rsp.srcAddr = srcAddr;
  pRsp->rsp.status = (zstack_ZdpStatus)pNotify->status;

  pRsp->rsp.scannedChannels = pNotify->scannedChannels;
  pRsp->rsp.totalTrans = pNotify->totalTransmissions;
  pRsp->rsp.transFails = pNotify->transmissionFailures;
  pRsp->rsp.n_energyValuesList = pNotify->listCount;

  pRsp->rsp.pEnergyValuesList = ICall_malloc( pRsp->rsp.n_energyValuesList );

  if ( pRsp->rsp.pEnergyValuesList )
  {
    osal_memcpy( pRsp->rsp.pEnergyValuesList, pNotify->energyValues,
          pRsp->rsp.n_energyValuesList );
  }

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}
#endif // ZDO_MGMT_NWKUPDATE_REQUEST

/**************************************************************************************************
 * @fn          processDevStateChange
 *
 * @brief       Process a Device State Change
 *
 * @param       srcAddr - Network address of source device
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void processDevStateChange( uint16 srcAddr, void *pMsg )
{
  sendMsgToAllCBMsgs( ZS_DEV_STATE_CHANGE_CDID, srcAddr, pMsg,
        sendDevStateChangeInd );
}

/**************************************************************************************************
 * @fn          sendDevStateChangeInd
 *
 * @brief       Send a Device State Change indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      none
 */
static void sendDevStateChangeInd( uint16 dstID, uint16 srcAddr, void *pMsg )
{
  uint8 state = *( (uint8 *)pMsg );
  zstackmsg_devStateChangeInd_t *pRsp;

  pRsp = (zstackmsg_devStateChangeInd_t *)ICall_allocMsg( sizeof(zstackmsg_devStateChangeInd_t) );
  if ( pRsp == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pRsp, 0, sizeof(zstackmsg_devStateChangeInd_t) );

  pRsp->hdr.event = zstackmsg_CmdIDs_DEV_STATE_CHANGE_IND;
  pRsp->req.state = (zstack_DevState)state;

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID, ICALL_MSG_FORMAT_KEEP, (void *)pRsp );
}

/**************************************************************************************************
 * @fn          processSysVersionReq
 *
 * @brief       Process the System Version Request by filling out
 *              the response section of the message
 *
 * @param       dstID - Destination iCall entity ID
 * @param       srcAddr - source address of message
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processSysVersionReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_sysVersionReq_t *pReq = (zstackmsg_sysVersionReq_t *)pMsg;

  if ( pReq->pRsp )
  {
    pReq->hdr.status = zstack_ZStatusValues_ZSuccess;

    pReq->pRsp->transportRev = 1;

    if ( ZG_BUILD_COORDINATOR_TYPE && ZG_DEVICE_COORDINATOR_TYPE )
    {
      pReq->pRsp->product = zstack_BuildTypes_COORDINATOR;
    }
    else if ( ZG_BUILD_RTR_TYPE )
    {
      pReq->pRsp->product = zstack_BuildTypes_ROUTER;
    }
    else
    {
      pReq->pRsp->product = zstack_BuildTypes_ENDDEVICE;
    }

    pReq->pRsp->majorRel = ZSTACK_SERVER_VERSION_MAJOR;
    pReq->pRsp->minorRel = ZSTACK_SERVER_VERSION_MINOR;
    pReq->pRsp->maintRel = ZSTACK_SERVER_VERSION_MAINTENANCE;
  }
  else
  {
    pReq->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processDevStartReq
 *
 * @brief       Process the Device Start Request by filling out
 *              the response section of the message
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processDevStartReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_devStartReq_t *pReq = (zstackmsg_devStartReq_t *)pMsg;

  if ( pReq->pReq )
  {
#if defined (HOLD_AUTO_START)
    pReq->hdr.status = zstack_ZStatusValues_ZSuccess;
    ZDOInitDevice( pReq->pReq->startDelay );
#else
    pReq->hdr.status = zstack_ZStatusValues_ZUnsupportedMode;
#endif
  }
  else
  {
    pReq->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processSysSetTxPowerReq
 *
 * @brief       Process the System Tx Power Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processSysSetTxPowerReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_sysSetTxPowerReq_t *pReq = (zstackmsg_sysSetTxPowerReq_t *)pMsg;

  if ( pReq->pReq )
  {
    int8 txPower = pReq->pReq->requestedTxPower;

    (void)ZMacSetTransmitPower( (ZMacTransmitPower_t)txPower );

    if ( pReq->pRsp )
    {
      // We don't have a corrected value, so use the requested
      pReq->pRsp->txPower = txPower;
    }
    pReq->hdr.status = zstack_ZStatusValues_ZSuccess;
  }
  else
  {
    pReq->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processDevJoinReq
 *
 * @brief       Process the Device Join Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processDevJoinReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_devJoinReq_t *pReq = (zstackmsg_devJoinReq_t *)pMsg;

#if !defined (ZDO_COORDINATOR)
  if ( pReq->pReq )
  {
    uint8 extendedPANID[Z_EXTADDR_LEN];

    osal_memcpy( extendedPANID, &(pReq->pReq->extendedPANID), Z_EXTADDR_LEN );

    // Set the ZDOApp state variable to finish the Device start
    ZDApp_ChangeState( DEV_NWK_JOINING );
    manualJoin = TRUE;

    ZDApp_NodeProfileSync( pReq->pReq->stackProfile );

    pReq->hdr.status = NLME_JoinRequest( extendedPANID, pReq->pReq->panID,
          pReq->pReq->logicalChannel,
          ZDO_Config_Node_Descriptor.CapabilityFlags,
          pReq->pReq->chosenParent,
          pReq->pReq->parentDepth );
  }
  else
  {
    pReq->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }
#else
  pReq->hdr.status = zstack_ZStatusValues_ZUnsupportedMode;
#endif

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processDevForceNetworkSettingsReq
 *
 * @brief       Process the Device Force Network Settings Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processDevForceNetworkSettingsReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_devForceNetworkSettingsReq_t *pReq =
        (zstackmsg_devForceNetworkSettingsReq_t *)pMsg;

  if ( pReq->pReq )
  {
    // Copy the new network parameters to NIB
    _NIB.nwkState = (nwk_states_t)pReq->pReq->state;
    _NIB.nwkDevAddress = pReq->pReq->nwkAddr;
    _NIB.nwkLogicalChannel = pReq->pReq->channelList;
    _NIB.nwkCoordAddress = pReq->pReq->parentNwkAddr;
    _NIB.channelList = pReq->pReq->channelList;
    _NIB.nwkPanId = pReq->pReq->panID;
    _NIB.nodeDepth = pReq->pReq->depth;
    _NIB.MaxRouters = pReq->pReq->maxRouters;
    _NIB.MaxChildren = pReq->pReq->maxChildren;
    _NIB.allocatedRouterAddresses = pReq->pReq->allocatedRouterAddresses;
    _NIB.allocatedEndDeviceAddresses = pReq->pReq->allocatedEndDeviceAddresses;

    if ( _NIB.nwkUpdateId != pReq->pReq->updateID )
    {
      NLME_SetUpdateID( pReq->pReq->updateID );
    }

    osal_cpyExtAddr( _NIB.extendedPANID, &(pReq->pReq->extendedPANID) );

    // Save the NIB
    if ( _NIB.nwkState == NWK_ROUTER )
    {
      // Update NIB in NV
      osal_nv_write( ZCD_NV_NIB, 0, sizeof(nwkIB_t), &_NIB );

      // Reset the NV startup option to resume from NV by clearing
      // the "New" join option.
      zgWriteStartupOptions( ZG_STARTUP_CLEAR,
            ZCD_STARTOPT_DEFAULT_NETWORK_STATE );
    }

    pReq->hdr.status = zstack_ZStatusValues_ZSuccess;
  }
  else
  {
    pReq->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processDevForceNetworkUpdateReq
 *
 * @brief       Process the Device Force Network Update Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processDevForceNetworkUpdateReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_devForceNetworkUpdateReq_t *pReq =
        (zstackmsg_devForceNetworkUpdateReq_t *)pMsg;

  if ( pReq->pReq )
  {
    // Update the network update id
    NLME_SetUpdateID( pReq->pReq->updateID );

    // Switch channel
    if ( _NIB.nwkLogicalChannel != pReq->pReq->logicalChannel )
    {
      uint8 curChannel;

      _NIB.nwkLogicalChannel = pReq->pReq->logicalChannel;

      // Try to change channel
      ZMacGetReq( ZMacChannel, &curChannel );

      if ( curChannel != pReq->pReq->logicalChannel )
      {
        curChannel = pReq->pReq->logicalChannel;
        // Set the new channel
        ZMacSetReq( ZMacChannel, &curChannel );
      }
    }

    // Update channel list
    _NIB.channelList = (uint32)1 << pReq->pReq->logicalChannel;

    // Our Channel has been changed -- notify to save info into NV
    ZDApp_NwkStateUpdateCB( );

    // Reset the total transmit count and the transmit failure counters
    _NIB.nwkTotalTransmissions = 0;
    nwkTransmissionFailures( TRUE );

    pReq->hdr.status = zstack_ZStatusValues_ZSuccess;
  }
  else
  {
    pReq->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processDevForceMacParamsReq
 *
 * @brief       Process the Device Force MAC Parameters Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processDevForceMacParamsReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_devForceMacParamsReq_t *pReq =
        (zstackmsg_devForceMacParamsReq_t *)pMsg;

  if ( pReq->pReq )
  {
    uint8 curChannel;
    uint16 nwkAddr = pReq->pReq->nwkAddr;
    uint16 panId = pReq->pReq->panID;

    // Set our short address
    ZMacSetReq( ZMacShortAddress, (uint8 *)&nwkAddr );

    // Set our PAN ID
    ZMacSetReq( ZMacPanId, (uint8 *)&panId );

    // Try to change channel
    ZMacGetReq( ZMacChannel, &curChannel );

    if ( curChannel != pReq->pReq->logicalChannel )
    {
      curChannel = pReq->pReq->logicalChannel;
      // Set the new channel
      ZMacSetReq( ZMacChannel, &curChannel );
    }

    pReq->hdr.status = zstack_ZStatusValues_ZSuccess;
  }
  else
  {
    pReq->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processDevUpdateNeighborTxCostReq
 *
 * @brief       Process the Device Update Neighbor TxCost Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processDevUpdateNeighborTxCostReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_devUpdateNeighborTxCostReq_t *pReq =
        (zstackmsg_devUpdateNeighborTxCostReq_t *)pMsg;

#if defined (RTR_NWK)
  if ( pReq->pReq )
  {
    linkInfo_t *linkInfo;

    // check if entry exists
    linkInfo = nwkNeighborGetLinkInfo( pReq->pReq->nwkAddr, pReq->pReq->panID );

    // if not, look for a vacant entry to add this node...
    if ( linkInfo == NULL )
    {
      nwkNeighborAdd( pReq->pReq->nwkAddr, pReq->pReq->panID, 1 );

      linkInfo = nwkNeighborGetLinkInfo( pReq->pReq->nwkAddr, pReq->pReq->panID );
    }

    if ( linkInfo )
    {
      linkInfo->txCost = pReq->pReq->txCost;
      pReq->hdr.status = zstack_ZStatusValues_ZSuccess;
    }
    else
    {
      pReq->hdr.status = zstack_ZStatusValues_ZNwkUnknownDevice;
    }
  }
  else
  {
    pReq->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }
#else
  pReq->hdr.status = zstack_ZStatusValues_ZUnsupportedMode;
#endif

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processDevNwkDiscoveryReq
 *
 * @brief       Process the Device Network Discovery Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processDevNwkDiscoveryReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_devNwkDiscReq_t *pReq = (zstackmsg_devNwkDiscReq_t *)pMsg;

  // Do checks to see if you can do this?
  if ( pReq->pReq == NULL )
  {
    pReq->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }
  else
  {
    epItem_t *pItem;

    // Find the endpoint connection relationship
    pItem = epTableFindEntryConnection( srcEntityID );
    if ( pItem == NULL )
    {
      // EP entry wasn't found, so create one with empty EP descriptor
      pItem = (epItem_t *)osal_mem_alloc( sizeof(epItem_t) );
      if ( pItem )
      {
        osal_memset( pItem, 0, sizeof(epItem_t) );
        pItem->connection = srcEntityID;
        if ( epTableAddNewEntry( pItem ) == FALSE )
        {
          osal_mem_free( pItem );
          pItem = NULL;
          pReq->hdr.status = zstack_ZStatusValues_ZAfEndpointMax;
        }
      }
    }

    if ( pItem != NULL )
    {
      NLME_ScanFields_t scanFields;

      pItem->zdoCBs |= ZS_ZDO_BEACON_NOTIFY_IND_CBID;
      pItem->zdoCBs |= ZS_ZDO_NWK_DISCOVERY_CNF_CBID;

      ZDO_RegisterForZdoCB( ZDO_NWK_DISCOVERY_CNF_CBID, zdoNwkDiscCnfCB );
      ZDO_RegisterForZdoCB( ZDO_BEACON_NOTIFY_IND_CBID, zdoBeaconNotifyIndCB );

      scanFields.channels = pReq->pReq->scanChannels;
      scanFields.duration = pReq->pReq->scanDuration;
      scanFields.scanType = ZMAC_ACTIVE_SCAN;
      scanFields.scanApp = NLME_DISC_SCAN;

      pReq->hdr.status = NLME_NwkDiscReq2( &scanFields );
    }
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processSysForceLinkStatusReq
 *
 * @brief       Process the System Force Link Status Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processSysForceLinkStatusReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_sysForceLinkStatusReq_t *pReq = (zstackmsg_sysForceLinkStatusReq_t *)pMsg;

#if defined (RTR_NWK)
  osal_set_event( NWK_TaskID, NWK_LINK_STATUS_EVT );

  pReq->hdr.status = zstack_ZStatusValues_ZSuccess;
#else
  pReq->hdr.status = zstack_ZStatusValues_ZUnsupportedMode;
#endif

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processApsRemoveGroup
 *
 * @brief       Process the APS Remove Group Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processApsRemoveGroup( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_apsRemoveGroup_t *pReq = (zstackmsg_apsRemoveGroup_t *)pMsg;

  pReq->hdr.status = zstack_ZStatusValues_ZInvalidParameter;

  if ( pReq->pReq )
  {
    if ( aps_RemoveGroup( pReq->pReq->endpoint, pReq->pReq->groupID ) )
    {
      pReq->hdr.status = zstack_ZStatusValues_ZSuccess;
    }
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processApsRemoveAllGroups
 *
 * @brief       Process the APS Remove All Groups Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processApsRemoveAllGroups( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_apsRemoveAllGroups_t *pReq = (zstackmsg_apsRemoveAllGroups_t *)pMsg;

  if ( pReq->pReq )
  {
    pReq->hdr.status = zstack_ZStatusValues_ZSuccess;
    aps_RemoveAllGroup( pReq->pReq->endpoint );
  }
  else
  {
    pReq->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processApsFindAllGroupsReq
 *
 * @brief       Process the APS Find All Groups Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processApsFindAllGroupsReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_apsFindAllGroupsReq_t *pReq =
        (zstackmsg_apsFindAllGroupsReq_t *)pMsg;

  if ( pReq->pReq && pReq->pRsp )
  {
    pReq->hdr.status = zstack_ZStatusValues_ZSuccess;

    pReq->pRsp->numGroups = aps_CountGroups( pReq->pReq->endpoint );

    pReq->pRsp->pGroupList = (uint16_t *)ICall_malloc(
          (sizeof(uint16_t) * pReq->pRsp->numGroups) );

    (void)aps_FindAllGroupsForEndpoint( pReq->pReq->endpoint,
          pReq->pRsp->pGroupList );
  }
  else
  {
    pReq->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processApsFindAllGroupsReq
 *
 * @brief       Process the APS Find a Group Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processApsFindGroupReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_apsFindGroupReq_t *pReq = (zstackmsg_apsFindGroupReq_t *)pMsg;

  if ( pReq->pReq && pReq->pRsp )
  {
    aps_Group_t *pGroup;

    osal_memset( pReq->pRsp, 0, sizeof(zstack_apsFindGroupRsp_t) );

    pGroup = aps_FindGroup( pReq->pReq->endpoint, pReq->pReq->groupID );
    if ( pGroup )
    {
      pReq->hdr.status = zstack_ZStatusValues_ZSuccess;

      pReq->pRsp->groupID = pGroup->ID;
      pReq->pRsp->n_name = pGroup->name[0];
      if ( pReq->pRsp->n_name )
      {
        pReq->pRsp->pName = (uint8_t *)ICall_malloc( pReq->pRsp->n_name );
        if ( pReq->pRsp->pName )
        {
          osal_memcpy( pReq->pRsp->pName, &(pGroup->name[1]),
                pReq->pRsp->n_name );
        }
      }
    }
    else
    {
      pReq->hdr.status = zstack_ZStatusValues_ZApsFail;
    }

  }
  else
  {
    pReq->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processApsAddGroup
 *
 * @brief       Process the APS Add a Group Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processApsAddGroup( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_apsAddGroup_t *pReq = (zstackmsg_apsAddGroup_t *)pMsg;

  if ( pReq->pReq )
  {
    aps_Group_t group = {0};

    // Build local group structure
    group.ID = pReq->pReq->groupID;
    if ( pReq->pReq->n_name && pReq->pReq->pName )
    {
      uint8 len = pReq->pReq->n_name;
      if ( len > (APS_GROUP_NAME_LEN - 1) )
      {
        len = (APS_GROUP_NAME_LEN - 1);
      }
      group.name[0] = len;
      osal_memcpy( &group.name[1], pReq->pReq->pName, len );
    }

    pReq->hdr.status = (zstack_ZStatusValues)aps_AddGroup(
          pReq->pReq->endpoint, &group );
  }
  else
  {
    pReq->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processApsCountAllGroups
 *
 * @brief       Process the APS Count all Groups Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processApsCountAllGroups( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_apsCountAllGroups_t *pReq = (zstackmsg_apsCountAllGroups_t *)pMsg;

  // The number of groups is returned in the status field
  pReq->hdr.status = (zstack_ZStatusValues)aps_CountAllGroups( );

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processSecApsRemoveReq
 *
 * @brief       Process the Security APS Remove Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processSecApsRemoveReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_secApsRemoveReq_t *pPtr = (zstackmsg_secApsRemoveReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    if ( ZG_BUILD_COORDINATOR_TYPE && ZG_DEVICE_COORDINATOR_TYPE )
    {
      pPtr->hdr.status = (zstack_ZStatusValues)ZDSecMgrAPSRemove(
            pPtr->pReq->nwkAddr, pPtr->pReq->extAddr, pPtr->pReq->parentAddr );
    }
    else
    {
      pPtr->hdr.status = zstack_ZStatusValues_ZUnsupportedMode;
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processSecNwkKeyUpdateReq
 *
 * @brief       Process the Security Network Key Update Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processSecNwkKeyUpdateReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_secNwkKeyUpdateReq_t *pPtr = (zstackmsg_secNwkKeyUpdateReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    if ( ZG_BUILD_COORDINATOR_TYPE && ZG_DEVICE_COORDINATOR_TYPE )
    {
      nwkKeyDesc nwkKey = {0};
      uint16 nvID = 0;
      uint8 done = FALSE;

      pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;

      while ( !done )
      {
        if ( nvID == 0 )
        {
          nvID = ZCD_NV_NWK_ALTERN_KEY_INFO;
        }
        else
        {
          nvID = ZCD_NV_NWK_ACTIVE_KEY_INFO;
          done = TRUE;
        }

        osal_nv_read( nvID, 0, sizeof(nwkKeyDesc), &nwkKey );

        if ( nwkKey.keySeqNum == pPtr->pReq->seqNum )
        {
          done = TRUE;
          pPtr->hdr.status = zstack_ZStatusValues_ZSuccess;
        }
      }

      if ( pPtr->hdr.status == zstack_ZStatusValues_ZSuccess )
      {
        pPtr->hdr.status = (zstack_ZStatusValues)ZDSecMgrUpdateNwkKey(
              nwkKey.key, pPtr->pReq->seqNum, pPtr->pReq->dstAddr );
      }
    }
    else
    {
      pPtr->hdr.status = zstack_ZStatusValues_ZUnsupportedMode;
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processSecNwkKeySwitchReq
 *
 * @brief       Process the Security Network Key Switch Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processSecNwkKeySwitchReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_secNwkKeySwitchReq_t *pPtr = (zstackmsg_secNwkKeySwitchReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    if ( ZG_BUILD_COORDINATOR_TYPE && ZG_DEVICE_COORDINATOR_TYPE )
    {
      pPtr->hdr.status = (zstack_ZStatusValues)ZDSecMgrSwitchNwkKey(
            pPtr->pReq->seqNum, pPtr->pReq->dstAddr );
    }
    else
    {
      pPtr->hdr.status = zstack_ZStatusValues_ZUnsupportedMode;
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processSecNwkKeySetReq
 *
 * @brief       Process the Security Network Key Set Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processSecNwkKeySetReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_secNwkKeySetReq_t *pPtr = (zstackmsg_secNwkKeySetReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    nwkKeyDesc nwkKey;
    uint16 nvID;

    if ( pPtr->pReq->activeKey )
    {
      nvID = ZCD_NV_NWK_ACTIVE_KEY_INFO;
    }
    else
    {
      nvID = ZCD_NV_NWK_ALTERN_KEY_INFO;
    }

    nwkKey.keySeqNum = pPtr->pReq->seqNum;
    if ( pPtr->pReq->has_key )
    {
      osal_memcpy( nwkKey.key, pPtr->pReq->key, 16 );
    }
    else
    {
      // optional key isn't included, so generate the key
      int x;
      for ( x = 0; x < 16; x++ )
      {
        nwkKey.key[x] = osal_rand( );
      }
    }

    pPtr->hdr.status = (zstack_ZStatusValues)osal_nv_write( nvID, 0,
          sizeof(nwkKeyDesc), &nwkKey );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processSecNwkKeyGetReq
 *
 * @brief       Process the Security Network Key Get Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processSecNwkKeyGetReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_secNwkKeyGetReq_t *pPtr = (zstackmsg_secNwkKeyGetReq_t *)pMsg;

  if ( pPtr->pReq && pPtr->pRsp )
  {
    nwkKeyDesc nwkKey;
    uint16 nvID;

    if ( pPtr->pReq->activeKey )
    {
      nvID = ZCD_NV_NWK_ACTIVE_KEY_INFO;
    }
    else
    {
      nvID = ZCD_NV_NWK_ALTERN_KEY_INFO;
    }

    pPtr->hdr.status = (zstack_ZStatusValues)osal_nv_read( nvID, 0,
          sizeof(nwkKeyDesc), &nwkKey );

    if ( pPtr->hdr.status == zstack_ZStatusValues_ZSuccess )
    {
      pPtr->pRsp->activeKey = pPtr->pReq->activeKey;
      pPtr->pRsp->seqNum = nwkKey.keySeqNum;
      osal_memcpy( pPtr->pRsp->key, nwkKey.key, 16 );
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processSecApsLinkKeyGetReq
 *
 * @brief       Process the Security APS Link Key Get Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processSecApsLinkKeyGetReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_secApsLinkKeyGetReq_t *pPtr =
        (zstackmsg_secApsLinkKeyGetReq_t *)pMsg;

  if ( pPtr->pReq && pPtr->pRsp )
  {
    pPtr->pRsp->tcLinkKey = pPtr->pReq->tcLinkKey;
    osal_memcpy( pPtr->pRsp->ieeeAddr, pPtr->pReq->ieeeAddr, Z_EXTADDR_LEN );

    if ( pPtr->pReq->tcLinkKey )
    {
      int x;
      APSME_TCLinkKey_t tcLinkKey;

      for ( x = 0; x < ZDSECMGR_TC_DEVICE_MAX; x++ )
      {
        pPtr->hdr.status = (zstack_ZStatusValues)osal_nv_read(
              (ZCD_NV_TCLK_TABLE_START + x), 0, sizeof(APSME_TCLinkKey_t),
              &tcLinkKey );
        if ( pPtr->hdr.status == zstack_ZStatusValues_ZSuccess )
        {
          if ( osal_memcmp( pPtr->pReq->ieeeAddr, tcLinkKey.extAddr, Z_EXTADDR_LEN ) == 0 )
          {
            pPtr->pRsp->rxFrmCntr = tcLinkKey.rxFrmCntr;
            pPtr->pRsp->txFrmCntr = tcLinkKey.txFrmCntr;
            osal_memcpy( pPtr->pRsp->key, tcLinkKey.key, 16 );
            break;
          }
        }
      }

      if ( x == ZDSECMGR_TC_DEVICE_MAX )
      {
        pPtr->hdr.status = zstack_ZStatusValues_ZSuccess;
      }
    }
    else
    {
      ZDSecMgrEntry_t *pEntry;

      pPtr->hdr.status = (zstack_ZStatusValues)ZDSecMgrEntryLookupExt(
            pPtr->pReq->ieeeAddr, &pEntry );
      if ( pPtr->hdr.status == zstack_ZStatusValues_ZSuccess )
      {
        APSME_LinkKeyData_t linkKey;

        pPtr->hdr.status = (zstack_ZStatusValues)osal_nv_read( pEntry->keyNvId,
              0, sizeof(APSME_LinkKeyData_t),
              &linkKey );
        if ( pPtr->hdr.status == zstack_ZStatusValues_ZSuccess )
        {
          pPtr->pRsp->rxFrmCntr = linkKey.rxFrmCntr;
          pPtr->pRsp->txFrmCntr = linkKey.txFrmCntr;
          osal_memcpy( pPtr->pRsp->key, linkKey.key, 16 );
        }
      }
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processSecApsLinkKeySetReq
 *
 * @brief       Process the Security APS Link Key Set Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processSecApsLinkKeySetReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_secApsLinkKeySetReq_t *pPtr =
        (zstackmsg_secApsLinkKeySetReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    if ( pPtr->pReq->tcLinkKey )
    {
      int y = 0;
      int x;
      APSME_TCLinkKey_t tcLinkKey;
      uint8 dummyKey[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

      for ( x = 0; x < ZDSECMGR_TC_DEVICE_MAX; x++ )
      {
        pPtr->hdr.status = (zstack_ZStatusValues)osal_nv_read(
              (ZCD_NV_TCLK_TABLE_START + x), 0, sizeof(APSME_TCLinkKey_t),
              &tcLinkKey );
        if ( pPtr->hdr.status == zstack_ZStatusValues_ZSuccess )
        {
          if ( osal_memcmp( pPtr->pReq->ieeeAddr, tcLinkKey.extAddr, Z_EXTADDR_LEN ) == 0 )
          {
            y = x;
            break;
          }
          else if ( (y == 0)
                && (osal_memcmp( pPtr->pReq->ieeeAddr, dummyKey, Z_EXTADDR_LEN ) == 0) )
          {
            // First empty spot
            y = x;
          }
        }
      }

      if ( y > 0 )
      {
        osal_memcpy( tcLinkKey.extAddr, pPtr->pReq->ieeeAddr, Z_EXTADDR_LEN );
        osal_memcpy( tcLinkKey.key, pPtr->pReq->key, SEC_KEY_LEN );
        tcLinkKey.txFrmCntr = pPtr->pReq->txFrmCntr;
        tcLinkKey.rxFrmCntr = pPtr->pReq->rxFrmCntr;

        pPtr->hdr.status = (zstack_ZStatusValues)osal_nv_write(
              (ZCD_NV_TCLK_TABLE_START + x), 0, sizeof(APSME_TCLinkKey_t),
              &tcLinkKey );
      }
    }
    else
    {
      ZDSecMgrEntry_t *pEntry;
      APSME_LinkKeyData_t linkKey;

      osal_memcpy( linkKey.key, pPtr->pReq->key, SEC_KEY_LEN );
      linkKey.txFrmCntr = pPtr->pReq->txFrmCntr;
      linkKey.rxFrmCntr = pPtr->pReq->rxFrmCntr;

      pPtr->hdr.status = (zstack_ZStatusValues)ZDSecMgrEntryLookupExt(
            pPtr->pReq->ieeeAddr, &pEntry );
      if ( pPtr->hdr.status == zstack_ZStatusValues_ZSuccess )
      {
        pPtr->hdr.status = (zstack_ZStatusValues)osal_nv_write(
              pEntry->keyNvId, 0, sizeof(APSME_LinkKeyData_t), &linkKey );
      }
      else
      {
        pPtr->hdr.status = (zstack_ZStatusValues)ZDSecMgrAddLinkKey(
              pPtr->pReq->shortAddr, pPtr->pReq->ieeeAddr, linkKey.key );
      }
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processSecApsLinkKeyRemoveReq
 *
 * @brief       Process the Security APS Link Key Remove Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processSecApsLinkKeyRemoveReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_secApsLinkKeyRemoveReq_t *pPtr =
        (zstackmsg_secApsLinkKeyRemoveReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    if ( pPtr->pReq->tcLinkKey )
    {
      int x;
      APSME_TCLinkKey_t tcLinkKey;

      for ( x = 0; x < ZDSECMGR_TC_DEVICE_MAX; x++ )
      {
        pPtr->hdr.status = (zstack_ZStatusValues)osal_nv_read(
              (ZCD_NV_TCLK_TABLE_START + x), 0, sizeof(APSME_TCLinkKey_t),
              &tcLinkKey );
        if ( pPtr->hdr.status == zstack_ZStatusValues_ZSuccess )
        {
          if ( osal_memcmp( pPtr->pReq->ieeeAddr, tcLinkKey.extAddr, Z_EXTADDR_LEN ) == 0 )
          {
            osal_memset( &tcLinkKey, 0, sizeof(APSME_TCLinkKey_t) );

            pPtr->hdr.status = (zstack_ZStatusValues)osal_nv_write(
                  (ZCD_NV_TCLK_TABLE_START + x), 0, sizeof(APSME_TCLinkKey_t),
                  &tcLinkKey );
            break;
          }
        }
      }
    }
    else
    {
      pPtr->hdr.status = (zstack_ZStatusValues)ZDSecMgrDeviceRemoveByExtAddr(
            pPtr->pReq->ieeeAddr );
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

#if defined (RTR_NWK)
/**************************************************************************************************
 * @fn          processDevNwkRouteReq
 *
 * @brief       Process the Device Network Route Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processDevNwkRouteReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_devNwkRouteReq_t *pPtr = (zstackmsg_devNwkRouteReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    uint8 options = 0;

    if ( pPtr->pReq->mtoRoute )
    {
      options |= MTO_ROUTE;

      if ( pPtr->pReq->mtoNoCache )
      {
        options |= NO_ROUTE_CACHE;
      }
    }

    if ( pPtr->pReq->multicast )
    {
      options |= MULTICAST_ROUTE;
    }

    pPtr->hdr.status = (zstack_ZStatusValues)NLME_RouteDiscoveryRequest(
          pPtr->pReq->dstAddr, options, pPtr->pReq->radius );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // RTR_NWK

#if defined (RTR_NWK)
/**************************************************************************************************
 * @fn          processDevNwkCheckRouteReq
 *
 * @brief       Process the Device Network Check Route Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processDevNwkCheckRouteReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_devNwkCheckRouteReq_t *pPtr = (zstackmsg_devNwkCheckRouteReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    uint8 options = 0;

    if ( pPtr->pReq->mtoRoute )
    {
      options |= MTO_ROUTE;
    }

    pPtr->hdr.status = (zstack_ZStatusValues)RTG_CheckRtStatus(
          pPtr->pReq->dstAddr, RT_ACTIVE, options );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // RTR_NWK

/**************************************************************************************************
 * @fn          processSysConfigReadReq
 *
 * @brief       Process the System Config Read Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processSysConfigReadReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_sysConfigReadReq_t *pPtr = (zstackmsg_sysConfigReadReq_t *)pMsg;

  if ( pPtr->pReq && pPtr->pRsp )
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZSuccess;

    if ( pPtr->pReq->preConfigKeyEnable )
    {
      pPtr->pRsp->has_preConfigKeyEnable = TRUE;
      pPtr->pRsp->preConfigKeyEnable = zgPreConfigKeys;
    }

    if ( pPtr->pReq->securityModeEnable )
    {
      pPtr->pRsp->has_securityModeEnable = TRUE;
      pPtr->pRsp->securityModeEnable = zgSecurityMode;
    }

    if ( pPtr->pReq->useDefaultTCLK )
    {
      pPtr->pRsp->has_useDefaultTCLK = TRUE;
      pPtr->pRsp->useDefaultTCLK = zgUseDefaultTCLK;
    }

    if ( pPtr->pReq->pollRate )
    {
      pPtr->pRsp->has_pollRate = TRUE;
      pPtr->pRsp->pollRate = zgPollRate;
    }

    if ( pPtr->pReq->queuedPollRate )
    {
      pPtr->pRsp->has_queuedPollRate = TRUE;
      pPtr->pRsp->queuedPollRate = zgQueuedPollRate;
    }

    if ( pPtr->pReq->responsePollRate )
    {
      pPtr->pRsp->has_responsePollRate = TRUE;
      pPtr->pRsp->responsePollRate = zgResponsePollRate;
    }

    if ( pPtr->pReq->apsAckWaitDuration )
    {
      pPtr->pRsp->has_apsAckWaitDuration = TRUE;
      pPtr->pRsp->apsAckWaitDuration = zgApscAckWaitDurationPolled;
    }

    if ( pPtr->pReq->bindingTime )
    {
      pPtr->pRsp->has_bindingTime = TRUE;
      pPtr->pRsp->bindingTime = zgApsDefaultMaxBindingTime;
    }

    if ( pPtr->pReq->panID )
    {
      pPtr->pRsp->has_panID = TRUE;
      pPtr->pRsp->panID = zgConfigPANID;
    }

    if ( pPtr->pReq->pollFailureRetries )
    {
      pPtr->pRsp->has_pollFailureRetries = TRUE;
      pPtr->pRsp->pollFailureRetries = zgMaxPollFailureRetries;
    }

    if ( pPtr->pReq->indirectMsgTimeout )
    {
      pPtr->pRsp->has_indirectMsgTimeout = TRUE;
      pPtr->pRsp->indirectMsgTimeout = zgIndirectMsgTimeout;
    }

    if ( pPtr->pReq->apsFrameRetries )
    {
      pPtr->pRsp->has_apsFrameRetries = TRUE;
      pPtr->pRsp->apsFrameRetries = zgApscMaxFrameRetries;
    }

    if ( pPtr->pReq->bcastRetries )
    {
      pPtr->pRsp->has_bcastRetries = TRUE;
      pPtr->pRsp->bcastRetries = zgMaxBcastRetires;
    }

    if ( pPtr->pReq->passiveAckTimeout )
    {
      pPtr->pRsp->has_passiveAckTimeout = TRUE;
      pPtr->pRsp->passiveAckTimeout = zgPassiveAckTimeout;
    }

    if ( pPtr->pReq->bcastDeliveryTime )
    {
      pPtr->pRsp->has_bcastDeliveryTime = TRUE;
      pPtr->pRsp->bcastDeliveryTime = zgBcastDeliveryTime;
    }

    if ( pPtr->pReq->routeExpiryTime )
    {
      pPtr->pRsp->has_routeExpiryTime = TRUE;
      pPtr->pRsp->routeExpiryTime = zgRouteExpiryTime;
    }

    if ( pPtr->pReq->userDesc )
    {
      UserDescriptorFormat_t userDesc;

      pPtr->pRsp->has_userDesc = TRUE;

      osal_nv_read( ZCD_NV_USERDESC, 0, sizeof(UserDescriptorFormat_t),
            &userDesc );

      pPtr->pRsp->n_userDesc = userDesc.len;
      if ( pPtr->pRsp->n_userDesc )
      {
        pPtr->pRsp->pUserDesc = ICall_malloc( userDesc.len );
        if ( pPtr->pRsp->pUserDesc )
        {
          osal_memcpy( pPtr->pRsp->pUserDesc, userDesc.desc, userDesc.len );
        }
        else
        {
          pPtr->pRsp->n_userDesc = 0;
          pPtr->pRsp->pUserDesc = NULL;
        }
      }
      else
      {
        pPtr->pRsp->pUserDesc = NULL;
      }
    }

    if ( pPtr->pReq->preConfigKey )
    {
      pPtr->pRsp->pPreConfigKey = ICall_malloc( 16 );
      if ( pPtr->pRsp->pPreConfigKey )
      {
        pPtr->pRsp->has_preConfigKey = TRUE;

        osal_nv_read( ZCD_NV_PRECFGKEY, 0, 16, pPtr->pRsp->pPreConfigKey );
      }
    }

    if ( pPtr->pReq->chanList )
    {
      pPtr->pRsp->has_chanList = TRUE;
      pPtr->pRsp->chanList = zgDefaultChannelList;
    }

    if ( pPtr->pReq->multicastRadius )
    {
      pPtr->pRsp->has_multicastRadius = TRUE;
      pPtr->pRsp->multicastRadius = zgApsNonMemberRadius;
    }

    if ( pPtr->pReq->extendedPANID )
    {
      pPtr->pRsp->has_extendedPANID = TRUE;
      osal_memcpy( &pPtr->pRsp->extendedPANID, zgApsUseExtendedPANID, Z_EXTADDR_LEN );
    }

    if ( pPtr->pReq->ieeeAddr )
    {
      pPtr->pRsp->has_ieeeAddr = TRUE;
      osal_memcpy( &pPtr->pRsp->ieeeAddr, NLME_GetExtAddr( ), Z_EXTADDR_LEN );
    }

    if ( pPtr->pReq->macRxOnIdle )
    {
      uint8 x;
      ZMacGetReq( ZMacRxOnIdle, &x );

      pPtr->pRsp->has_macRxOnIdle = TRUE;
      if ( x )
      {
        pPtr->pRsp->macRxOnIdle = TRUE;
      }
      else
      {
        pPtr->pRsp->macRxOnIdle = FALSE;
      }
    }

    if ( pPtr->pReq->snifferFeature )
    {
      pPtr->pRsp->has_snifferFeature = TRUE;
      // TBD
      pPtr->pRsp->snifferFeature = FALSE;
    }

    if ( pPtr->pReq->concentratorEnable )
    {
      pPtr->pRsp->has_concentratorEnable = TRUE;
      pPtr->pRsp->concentratorEnable = zgConcentratorEnable;
    }

    if ( pPtr->pReq->concentratorDiscTime )
    {
      pPtr->pRsp->has_concentratorDiscTime = TRUE;
      pPtr->pRsp->concentratorDiscTime = zgConcentratorDiscoveryTime;
    }

    if ( pPtr->pReq->nwkUseMultiCast )
    {
      pPtr->pRsp->has_nwkUseMultiCast = TRUE;
      pPtr->pRsp->nwkUseMultiCast = nwkUseMultiCast;
    }

    if ( pPtr->pReq->devPartOfNetwork )
    {
      pPtr->pRsp->has_devPartOfNetwork = TRUE;
      pPtr->pRsp->devPartOfNetwork = isDevicePartOfNetwork( );
    }

    if ( pPtr->pReq->rejoinBackoffDuration )
    {
      pPtr->pRsp->has_rejoinBackoffDuration = TRUE;
      pPtr->pRsp->rejoinBackoffDuration = zgDefaultRejoinBackoff;
    }

    if ( pPtr->pReq->rejoinScanDuration )
    {
      pPtr->pRsp->has_rejoinScanDuration = TRUE;
      pPtr->pRsp->rejoinScanDuration = zgDefaultRejoinScan;
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processSysConfigWriteReq
 *
 * @brief       Process the System Config Write Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processSysConfigWriteReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_sysConfigWriteReq_t *pPtr = (zstackmsg_sysConfigWriteReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZSuccess;

    if ( pPtr->pReq->has_preConfigKeyEnable )
    {
      if ( pPtr->pReq->preConfigKeyEnable )
      {
        zgPreConfigKeys = TRUE;
      }
      else
      {
        zgPreConfigKeys = FALSE;
      }
      osal_nv_write( ZCD_NV_PRECFGKEYS_ENABLE, 0, sizeof(zgPreConfigKeys),
            &zgPreConfigKeys );
    }

    if ( pPtr->pReq->has_securityModeEnable )
    {
      if ( pPtr->pReq->securityModeEnable )
      {
        zgSecurityMode = TRUE;
      }
      else
      {
        zgSecurityMode = FALSE;
      }
      osal_nv_write( ZCD_NV_SECURITY_MODE, 0, sizeof(zgSecurityMode),
            &zgSecurityMode );
    }

    if ( pPtr->pReq->has_useDefaultTCLK )
    {
      if ( pPtr->pReq->useDefaultTCLK )
      {
        zgUseDefaultTCLK = TRUE;
      }
      else
      {
        zgUseDefaultTCLK = FALSE;
      }
      osal_nv_write( ZCD_NV_USE_DEFAULT_TCLK, 0, sizeof(zgUseDefaultTCLK),
            &zgUseDefaultTCLK );
    }

    if ( pPtr->pReq->has_pollRate )
    {
      zgPollRate = (uint16)pPtr->pReq->pollRate;
      osal_nv_write( ZCD_NV_POLL_RATE, 0, sizeof(zgPollRate), &zgPollRate );
    }

    if ( pPtr->pReq->has_queuedPollRate )
    {
      zgQueuedPollRate = (uint16)pPtr->pReq->queuedPollRate;
      osal_nv_write( ZCD_NV_QUEUED_POLL_RATE, 0, sizeof(zgQueuedPollRate),
            &zgQueuedPollRate );
    }

    if ( pPtr->pReq->has_responsePollRate )
    {
      zgResponsePollRate = (uint16)pPtr->pReq->responsePollRate;
      osal_nv_write( ZCD_NV_RESPONSE_POLL_RATE, 0, sizeof(zgResponsePollRate),
            &zgResponsePollRate );
    }

    if ( pPtr->pReq->has_apsAckWaitDuration )
    {
      zgApscAckWaitDurationPolled = (uint16)pPtr->pReq->apsAckWaitDuration;
      osal_nv_write( ZCD_NV_APS_ACK_WAIT_DURATION, 0,
            sizeof(zgApscAckWaitDurationPolled), &zgApscAckWaitDurationPolled );
    }

    if ( pPtr->pReq->has_bindingTime )
    {
      zgApsDefaultMaxBindingTime = (uint16)pPtr->pReq->bindingTime;
      osal_nv_write( ZCD_NV_BINDING_TIME, 0, sizeof(zgApsDefaultMaxBindingTime),
            &zgApsDefaultMaxBindingTime );
    }

    if ( pPtr->pReq->has_panID )
    {
      zgConfigPANID = (uint16)pPtr->pReq->panID;
      osal_nv_write( ZCD_NV_PANID, 0, sizeof(zgConfigPANID), &zgConfigPANID );
    }

    if ( pPtr->pReq->has_pollFailureRetries )
    {
      zgMaxPollFailureRetries = (uint8)pPtr->pReq->pollFailureRetries;
      osal_nv_write( ZCD_NV_POLL_FAILURE_RETRIES, 0,
            sizeof(zgMaxPollFailureRetries), &zgMaxPollFailureRetries );
    }

    if ( pPtr->pReq->has_indirectMsgTimeout )
    {
      zgIndirectMsgTimeout = (uint8)pPtr->pReq->indirectMsgTimeout;
      osal_nv_write( ZCD_NV_INDIRECT_MSG_TIMEOUT, 0,
            sizeof(zgIndirectMsgTimeout), &zgIndirectMsgTimeout );
    }

    if ( pPtr->pReq->has_apsFrameRetries )
    {
      zgApscMaxFrameRetries = (uint8)pPtr->pReq->apsFrameRetries;
      osal_nv_write( ZCD_NV_APS_FRAME_RETRIES, 0, sizeof(zgApscMaxFrameRetries),
            &zgApscMaxFrameRetries );
    }

    if ( pPtr->pReq->has_bcastRetries )
    {
      zgMaxBcastRetires = (uint8)pPtr->pReq->bcastRetries;
      osal_nv_write( ZCD_NV_BCAST_RETRIES, 0, sizeof(zgMaxBcastRetires),
            &zgMaxBcastRetires );
    }

    if ( pPtr->pReq->has_passiveAckTimeout )
    {
      zgPassiveAckTimeout = (uint8)pPtr->pReq->passiveAckTimeout;
      osal_nv_write( ZCD_NV_PASSIVE_ACK_TIMEOUT, 0, sizeof(zgPassiveAckTimeout),
            &zgPassiveAckTimeout );
    }

    if ( pPtr->pReq->has_bcastDeliveryTime )
    {
      zgBcastDeliveryTime = (uint8)pPtr->pReq->bcastDeliveryTime;
      osal_nv_write( ZCD_NV_BCAST_DELIVERY_TIME, 0, sizeof(zgBcastDeliveryTime),
            &zgBcastDeliveryTime );
    }

    if ( pPtr->pReq->has_routeExpiryTime )
    {
      zgRouteExpiryTime = (uint8)pPtr->pReq->routeExpiryTime;
      osal_nv_write( ZCD_NV_ROUTE_EXPIRY_TIME, 0, sizeof(zgRouteExpiryTime),
            &zgRouteExpiryTime );
    }

    if ( pPtr->pReq->has_userDesc )
    {
      UserDescriptorFormat_t userDesc;

      userDesc.len = pPtr->pReq->n_userDesc;
      if ( userDesc.len > AF_MAX_USER_DESCRIPTOR_LEN )
      {
        userDesc.len = AF_MAX_USER_DESCRIPTOR_LEN;
      }

      osal_memcpy( userDesc.desc, pPtr->pReq->pUserDesc, userDesc.len );
      osal_nv_write( ZCD_NV_USERDESC, 0, sizeof(UserDescriptorFormat_t),
            &userDesc );
    }

    if ( pPtr->pReq->has_preConfigKey )
    {
      int len;

      len = pPtr->pReq->n_preConfigKey;
      if ( len > 16 )
      {
        len = 16;
      }

      osal_nv_write( ZCD_NV_PRECFGKEY, 0, len, pPtr->pReq->pPreConfigKey );
    }

    if ( pPtr->pReq->has_chanList )
    {
      zgDefaultChannelList = (uint32)pPtr->pReq->chanList;
      osal_nv_write( ZCD_NV_CHANLIST, 0, sizeof(zgDefaultChannelList),
            &zgDefaultChannelList );
    }

    if ( pPtr->pReq->has_multicastRadius )
    {
      zgApsNonMemberRadius = (uint8)pPtr->pReq->multicastRadius;
      osal_nv_write( ZCD_NV_APS_NONMEMBER_RADIUS, 0,
            sizeof(zgApsNonMemberRadius), &zgApsNonMemberRadius );
    }

    if ( pPtr->pReq->has_extendedPANID )
    {
      osal_memcpy( zgApsUseExtendedPANID, &(pPtr->pReq->extendedPANID), Z_EXTADDR_LEN );
      osal_nv_write( ZCD_NV_APS_USE_EXT_PANID, 0, Z_EXTADDR_LEN, zgApsUseExtendedPANID );
    }

    if ( pPtr->pReq->has_ieeeAddr )
    {
      osal_nv_write( ZCD_NV_EXTADDR, 0, Z_EXTADDR_LEN, pPtr->pReq->ieeeAddr );
    }

    if ( pPtr->pReq->has_macRxOnIdle )
    {
      uint8 x = pPtr->pReq->macRxOnIdle;
      ZMacSetReq( ZMacRxOnIdle, &x );
    }

    if ( pPtr->pReq->has_snifferFeature )
    {
      // TBD - Added enable sniffer
    }

    if ( pPtr->pReq->has_concentratorDiscTime )
    {
      zgConcentratorDiscoveryTime = (uint8)pPtr->pReq->concentratorDiscTime;
      osal_nv_write( ZCD_NV_CONCENTRATOR_DISCOVERY, 0,
            sizeof(zgConcentratorDiscoveryTime), &zgConcentratorDiscoveryTime );
    }

    if ( pPtr->pReq->has_concentratorEnable )
    {
      zgConcentratorEnable = (uint8)pPtr->pReq->concentratorEnable;
      osal_nv_write( ZCD_NV_CONCENTRATOR_ENABLE, 0,
            sizeof(zgConcentratorEnable), &zgConcentratorEnable );

      ZDApp_ForceConcentratorChange( );
    }

    if ( pPtr->pReq->has_nwkUseMultiCast )
    {
      nwkUseMultiCast = pPtr->pReq->nwkUseMultiCast;
      _NIB.nwkUseMultiCast = nwkUseMultiCast;
    }

    if  ( pPtr->pReq->has_rejoinBackoffDuration )
    {
      ZDApp_SetRejoinBackoffDuration( pPtr->pReq->rejoinBackoffDuration );
    }

    if ( pPtr->pReq->has_rejoinScanDuration )
    {
      ZDApp_SetRejoinScanDuration( pPtr->pReq->rejoinScanDuration );
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processSysNwkInfoReadReq
 *
 * @brief       Process the System Network Info Read Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processSysNwkInfoReadReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_sysNwkInfoReadReq_t *pPtr = (zstackmsg_sysNwkInfoReadReq_t *)pMsg;

  if ( pPtr->pRsp )
  {
    uint8 deviceType = ZSTACK_DEVICE_BUILD;

    pPtr->hdr.status = zstack_ZStatusValues_ZSuccess;

    osal_memset( pPtr->pRsp, 0, sizeof(zstack_sysNwkInfoReadRsp_t) );

    pPtr->pRsp->nwkAddr = _NIB.nwkDevAddress;
    pPtr->pRsp->devState = (zstack_DevState)devState;
    pPtr->pRsp->panId = _NIB.nwkPanId;
    pPtr->pRsp->parentNwkAddr = _NIB.nwkCoordAddress;
    osal_memcpy( pPtr->pRsp->extendedPanId, _NIB.extendedPANID, Z_EXTADDR_LEN );
    osal_memcpy( pPtr->pRsp->ieeeAddr, NLME_GetExtAddr( ), Z_EXTADDR_LEN );
    osal_memcpy( pPtr->pRsp->parentExtAddr, _NIB.nwkCoordExtAddress, Z_EXTADDR_LEN );
    pPtr->pRsp->logicalChannel = _NIB.nwkLogicalChannel;

    if ( deviceType & DEVICE_BUILD_COORDINATOR )
    {
      pPtr->pRsp->devTypes.coodinator = TRUE;
    }
    if ( deviceType & DEVICE_BUILD_ROUTER )
    {
      pPtr->pRsp->devTypes.router = TRUE;
    }
    if ( deviceType & DEVICE_BUILD_ENDDEVICE )
    {
      pPtr->pRsp->devTypes.enddevice = TRUE;
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processDevZDOCBReq
 *
 * @brief       Process the Device ZDO Callback Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processDevZDOCBReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_devZDOCBReq_t *pPtr = (zstackmsg_devZDOCBReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    epItem_t *pItem;

    pPtr->hdr.status = zstack_ZStatusValues_ZSuccess;

    pItem = epTableFindEntryConnection( srcEntityID );
    if ( pItem == NULL )
    {
      // EP entry wasn't found, so create one with empty EP descriptor
      pItem = (epItem_t *)osal_mem_alloc( sizeof(epItem_t) );
      if ( pItem )
      {
        osal_memset( pItem, 0, sizeof(epItem_t) );
        pItem->connection = srcEntityID;
        if ( epTableAddNewEntry( pItem ) == FALSE )
        {
          osal_mem_free( pItem );
          pItem = NULL;
          pPtr->hdr.status = zstack_ZStatusValues_ZAfEndpointMax;
        }
      }
    }

    if ( pItem != NULL )
    {
      if ( pPtr->pReq->has_srcRtgIndCB )
      {
        if ( pPtr->pReq->srcRtgIndCB )
        {
          pItem->zdoCBs |= ZS_ZDO_SRC_RTG_IND_CBID;
        }
        else
        {
          pItem->zdoCBs &= ~ZS_ZDO_SRC_RTG_IND_CBID;
        }
      }

      if ( pPtr->pReq->has_concentratorIndCb )
      {
        if ( pPtr->pReq->concentratorIndCb )
        {
          pItem->zdoCBs |= ZS_ZDO_CONCENTRATOR_IND_CBID;
        }
        else
        {
          pItem->zdoCBs &= ~ZS_ZDO_CONCENTRATOR_IND_CBID;
        }
      }

      if ( pPtr->pReq->has_nwkDiscCnfCB )
      {
        if ( pPtr->pReq->nwkDiscCnfCB )
        {
          pItem->zdoCBs |= ZS_ZDO_NWK_DISCOVERY_CNF_CBID;
        }
        else
        {
          pItem->zdoCBs &= ~ZS_ZDO_NWK_DISCOVERY_CNF_CBID;
        }
      }

      if ( pPtr->pReq->has_beaconNotIndCB )
      {
        if ( pPtr->pReq->beaconNotIndCB )
        {
          pItem->zdoCBs |= ZS_ZDO_BEACON_NOTIFY_IND_CBID;
        }
        else
        {
          pItem->zdoCBs &= ~ZS_ZDO_BEACON_NOTIFY_IND_CBID;
        }
      }

      if ( pPtr->pReq->has_joinCnfCB )
      {
        if ( pPtr->pReq->joinCnfCB )
        {
          pItem->zdoCBs |= ZS_ZDO_JOIN_CNF_CBID;
        }
        else
        {
          pItem->zdoCBs &= ~ZS_ZDO_JOIN_CNF_CBID;
        }
      }

      if ( pPtr->pReq->has_leaveCnfCB )
      {
        if ( pPtr->pReq->leaveCnfCB )
        {
          pItem->zdoCBs |= ZS_ZDO_LEAVE_CNF_CBID;
        }
        else
        {
          pItem->zdoCBs &= ~ZS_ZDO_LEAVE_CNF_CBID;
        }
      }

      if ( pPtr->pReq->has_leaveIndCB )
      {
        if ( pPtr->pReq->leaveIndCB )
        {
          pItem->zdoCBs |= ZS_ZDO_LEAVE_IND_CBID;
        }
        else
        {
          pItem->zdoCBs &= ~ZS_ZDO_LEAVE_IND_CBID;
        }
      }

#if defined (ZDO_NWKADDR_REQUEST)
      if ( pPtr->pReq->has_nwkAddrRsp )
      {
        if ( pPtr->pReq->nwkAddrRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_NWK_ADDR_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_NWK_ADDR_RSP_CDID;
        }
      }
#endif // ZDO_NWKADDR_REQUEST

#if defined (ZDO_IEEEADDR_REQUEST)
      if ( pPtr->pReq->has_ieeeAddrRsp )
      {
        if ( pPtr->pReq->ieeeAddrRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_IEEE_ADDR_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_IEEE_ADDR_RSP_CDID;
        }
      }
#endif // ZDO_IEEEADDR_REQUEST

#if defined (ZDO_NODEDESC_REQUEST)
      if ( pPtr->pReq->has_nodeDescRsp )
      {
        if ( pPtr->pReq->nodeDescRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_NODE_DESC_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_NODE_DESC_RSP_CDID;
        }
      }
#endif // ZDO_NODEDESC_REQUEST

#if defined (ZDO_POWERDESC_REQUEST)
      if ( pPtr->pReq->has_powerDescRsp )
      {
        if ( pPtr->pReq->powerDescRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_POWER_DESC_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_POWER_DESC_RSP_CDID;
        }
      }
#endif // ZDO_POWERDESC_REQUEST

#if defined (ZDO_SIMPLEDESC_REQUEST)
      if ( pPtr->pReq->has_simpleDescRsp )
      {
        if ( pPtr->pReq->simpleDescRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_SIMPLE_DESC_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_SIMPLE_DESC_RSP_CDID;
        }
      }
#endif // ZDO_SIMPLEDESC_REQUEST

#if defined (ZDO_ACTIVEEP_REQUEST)
      if ( pPtr->pReq->has_activeEndpointRsp )
      {
        if ( pPtr->pReq->activeEndpointRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_ACTIVE_EP_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_ACTIVE_EP_RSP_CDID;
        }
      }
#endif // ZDO_ACTIVEEP_REQUEST

#if defined (ZDO_MATCH_REQUEST)
      if ( pPtr->pReq->has_matchDescRsp )
      {
        if ( pPtr->pReq->matchDescRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_MATCH_DESC_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_MATCH_DESC_RSP_CDID;
        }
      }
#endif // ZDO_MATCH_REQUEST

#if defined (ZDO_COMPLEXDESC_REQUEST)
      if ( pPtr->pReq->has_complexDescRsp )
      {
        if ( pPtr->pReq->complexDescRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_COMPLEX_DESC_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_COMPLEX_DESC_RSP_CDID;
        }
      }
#endif // ZDO_COMPLEXDESC_REQUEST

#if defined (ZDO_USERDESC_REQUEST)
      if ( pPtr->pReq->has_userDescRsp )
      {
        if ( pPtr->pReq->userDescRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_USER_DESC_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_USER_DESC_RSP_CDID;
        }
      }
#endif // ZDO_USERDESC_REQUEST

#if 0
      if ( pPtr->pReq->has_discoveryCacheRsp )
      {
        if ( pPtr->pReq->discoveryCacheRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_DISCOVERY_CACHE_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_DISCOVERY_CACHE_RSP_CDID;
        }
      }
#endif

      if ( pPtr->pReq->has_userDescCnf )
      {
        if ( pPtr->pReq->userDescCnf )
        {
          pItem->zdoRsps |= ZS_ZDO_USER_DESC_CONF_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_USER_DESC_CONF_CDID;
        }
      }

#if defined (ZDO_SERVERDISC_REQUEST)
      if ( pPtr->pReq->has_serverDiscoveryRsp )
      {
        if ( pPtr->pReq->serverDiscoveryRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_SERVER_DISCOVERY_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_SERVER_DISCOVERY_RSP_CDID;
        }
      }
#endif // ZDO_SERVERDISC_REQUEST

#if defined (ZDO_BIND_UNBIND_REQUEST)
      if ( pPtr->pReq->has_bindRsp )
      {
        if ( pPtr->pReq->bindRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_BIND_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_BIND_RSP_CDID;
        }
      }
#endif // ZDO_BIND_UNBIND_REQUEST

#if defined (ZDO_ENDDEVICEBIND_REQUEST)
      if ( pPtr->pReq->has_endDeviceBindRsp )
      {
        if ( pPtr->pReq->endDeviceBindRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_END_DEVICE_BIND_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_END_DEVICE_BIND_RSP_CDID;
        }
      }
#endif // ZDO_ENDDEVICEBIND_REQUEST

#if defined (ZDO_BIND_UNBIND_REQUEST)
      if ( pPtr->pReq->has_unbindRsp )
      {
        if ( pPtr->pReq->unbindRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_UNBIND_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_UNBIND_RSP_CDID;
        }
      }
#endif // ZDO_BIND_UNBIND_REQUEST

#if defined (ZDO_MGMT_NWKDISC_REQUEST)
      if ( pPtr->pReq->has_mgmtNwkDiscRsp )
      {
        if ( pPtr->pReq->mgmtNwkDiscRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_MGMT_NWK_DISC_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_MGMT_NWK_DISC_RSP_CDID;
        }
      }
#endif // ZDO_MGMT_NWKDISC_REQUEST

#if defined (ZDO_MGMT_LQI_REQUEST)
      if ( pPtr->pReq->has_mgmtLqiRsp )
      {
        if ( pPtr->pReq->mgmtLqiRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_MGMT_LQI_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_MGMT_LQI_RSP_CDID;
        }
      }
#endif // ZDO_MGMT_LQI_REQUEST

#if defined (ZDO_MGMT_RTG_REQUEST)
      if ( pPtr->pReq->has_mgmtRtgRsp )
      {
        if ( pPtr->pReq->mgmtRtgRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_MGMT_RTG_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_MGMT_RTG_RSP_CDID;
        }
      }
#endif // ZDO_MGMT_RTG_REQUEST

#if defined (ZDO_MGMT_BIND_REQUEST)
      if ( pPtr->pReq->has_mgmtBindRsp )
      {
        if ( pPtr->pReq->mgmtBindRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_MGMT_BIND_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_MGMT_BIND_RSP_CDID;
        }
      }
#endif // ZDO_MGMT_BIND_REQUEST

#if defined (ZDO_MGMT_LEAVE_REQUEST)
      if ( pPtr->pReq->has_mgmtLeaveRsp )
      {
        if ( pPtr->pReq->mgmtLeaveRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_MGMT_LEAVE_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_MGMT_LEAVE_RSP_CDID;
        }
      }
#endif // ZDO_MGMT_LEAVE_REQUEST

#if defined (ZDO_MGMT_JOINDIRECT_REQUEST)
      if ( pPtr->pReq->has_mgmtDirectJoinRsp )
      {
        if ( pPtr->pReq->mgmtDirectJoinRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_MGMT_DIRECT_JOIN_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_MGMT_DIRECT_JOIN_RSP_CDID;
        }
      }
#endif // ZDO_MGMT_JOINDIRECT_REQUEST

#if defined (ZDO_MGMT_PERMIT_JOIN_REQUEST)
      if ( pPtr->pReq->has_mgmtPermitJoinRsp )
      {
        if ( pPtr->pReq->mgmtPermitJoinRsp )
        {
          pItem->zdoRsps |= ZS_ZDO_MGMT_PERMIT_JOIN_RSP_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_MGMT_PERMIT_JOIN_RSP_CDID;
        }
      }
#endif // ZDO_MGMT_PERMIT_JOIN_REQUEST

#if defined (ZDO_MGMT_NWKUPDATE_REQUEST)
      if ( pPtr->pReq->has_mgmtNwkUpdateNotify )
      {
        if ( pPtr->pReq->mgmtNwkUpdateNotify )
        {
          pItem->zdoRsps |= ZS_ZDO_MGMT_NWK_UPDATE_NOTIFY_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_MGMT_NWK_UPDATE_NOTIFY_CDID;
        }
      }
#endif // ZDO_MGMT_NWKUPDATE_REQUEST

      if ( pPtr->pReq->has_deviceAnnounce )
      {
        if ( pPtr->pReq->deviceAnnounce )
        {
          pItem->zdoRsps |= ZS_ZDO_DEVICE_ANNOUNCE_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_ZDO_DEVICE_ANNOUNCE_CDID;
        }
      }

      if ( pPtr->pReq->has_devStateChange )
      {
        if ( pPtr->pReq->devStateChange )
        {
          pItem->zdoRsps |= ZS_DEV_STATE_CHANGE_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_DEV_STATE_CHANGE_CDID;
        }
      }

      if ( pPtr->pReq->has_devJammerInd )
      {
        if ( pPtr->pReq->devJammerInd )
        {
          pItem->zdoRsps |= ZS_DEV_JAMMER_IND_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_DEV_JAMMER_IND_CDID;
        }
      }

      if ( pPtr->pReq->has_tcDeviceInd )
      {
        if ( pPtr->pReq->tcDeviceInd )
        {
          pItem->zdoRsps |= ZS_TC_DEVICE_IND_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_TC_DEVICE_IND_CDID;
        }
      }

      if ( pPtr->pReq->has_devPermitJoinInd )
      {
        if ( pPtr->pReq->devPermitJoinInd )
        {
          pItem->zdoRsps |= ZS_DEV_PERMIT_JOIN_IND_CDID;
        }
        else
        {
          pItem->zdoRsps &= ~ZS_DEV_PERMIT_JOIN_IND_CDID;
        }
      }
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processAfRegisterReq
 *
 * @brief       Process the AF Register Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processAfRegisterReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_afRegisterReq_t *pPtr = (zstackmsg_afRegisterReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    epItem_t *pItem;

    if ( epTableFindEntryEP( (uint8)pPtr->pReq->endpoint ) )
    {
      pPtr->hdr.status = zstack_ZStatusValues_ZAfDuplicateEndpoint;
    }
    else if ( epTableNumEntries( ) >= maxSupportedEndpoints )
    {
      pPtr->hdr.status = zstack_ZStatusValues_ZAfEndpointMax;
    }
    else
    {
      uint8 allocated = FALSE;

      pPtr->hdr.status = zstack_ZStatusValues_ZSuccess;

      // See if there is an EP item that hasn't defined an endpointl
      pItem = epTableFindEntryConnection( srcEntityID );
      if ( (pItem == NULL) || (pItem->epDesc.endPoint != 0) )
      {
        // Allocate a new epItem_t if one doesn't existed or an endpoint has
        // already been
        // defined.
        pItem = (epItem_t *)osal_mem_alloc( sizeof(epItem_t) );
        if ( pItem )
        {
          allocated = TRUE;
          osal_memset( pItem, 0, sizeof(epItem_t) );
          pItem->connection = srcEntityID;
        }
      }

      if ( pItem )
      {
        uint8 status;

        pItem->epDesc.endPoint = pPtr->pReq->endpoint;
        pItem->epDesc.latencyReq =
              (afNetworkLatencyReq_t)pPtr->pReq->latencyReq;
        pItem->epDesc.task_id = &ZStackTaskId;

        pItem->epDesc.simpleDesc = (SimpleDescriptionFormat_t *)ICall_malloc(
              sizeof(SimpleDescriptionFormat_t) );
        if ( pItem->epDesc.simpleDesc )
        {
          int i;

          osal_memset( pItem->epDesc.simpleDesc, 0,
                sizeof(SimpleDescriptionFormat_t) );

          pItem->epDesc.simpleDesc->AppDevVer =
                pPtr->pReq->pSimpleDesc->deviceVer;
          pItem->epDesc.simpleDesc->AppDeviceId =
                pPtr->pReq->pSimpleDesc->deviceID;
          pItem->epDesc.simpleDesc->EndPoint =
                pPtr->pReq->pSimpleDesc->endpoint;
          pItem->epDesc.simpleDesc->AppProfId =
                pPtr->pReq->pSimpleDesc->profileID;
          pItem->epDesc.simpleDesc->AppNumInClusters =
                pPtr->pReq->pSimpleDesc->n_inputClusters;
          pItem->epDesc.simpleDesc->AppNumOutClusters =
                pPtr->pReq->pSimpleDesc->n_outputClusters;

          pItem->epDesc.simpleDesc->pAppInClusterList =
                (cId_t *)osal_mem_alloc(
                sizeof(cId_t) * pItem->epDesc.simpleDesc->AppNumInClusters );
          if ( pItem->epDesc.simpleDesc->pAppInClusterList )
          {
            for ( i = 0; i < pItem->epDesc.simpleDesc->AppNumInClusters; i++ )
            {
              pItem->epDesc.simpleDesc->pAppInClusterList[i] =
                    pPtr->pReq->pSimpleDesc->pInputClusters[i];
            }
          }

          pItem->epDesc.simpleDesc->pAppOutClusterList =
                (cId_t *)osal_mem_alloc(
                sizeof(cId_t) * pItem->epDesc.simpleDesc->AppNumOutClusters );
          if ( pItem->epDesc.simpleDesc->pAppOutClusterList )
          {
            for ( i = 0; i < pItem->epDesc.simpleDesc->AppNumOutClusters; i++ )
            {
              pItem->epDesc.simpleDesc->pAppOutClusterList[i] =
                    pPtr->pReq->pSimpleDesc->pOutputClusters[i];
            }
          }
        }

        status = afRegister( &(pItem->epDesc) );

        if ( (status == afStatus_INVALID_PARAMETER)
              || (status == ZApsDuplicateEntry) )
        {
          afDelete( pItem->epDesc.endPoint );
          status = (zstack_ZStatusValues)afRegister( &(pItem->epDesc) );
        }

        if ( status == ZSuccess )
        {
          if ( allocated == TRUE )
          {
            if ( epTableAddNewEntry( pItem ) == FALSE )
            {
              freeEpItem( pItem );
            }
          }
        }
        else
        {
          if ( allocated == TRUE )
          {
            freeEpItem( pItem );
          }
          else
          {
            // AF registration failed clear the end point descriptor
            if ( pItem->epDesc.simpleDesc )
            {
              if ( pItem->epDesc.simpleDesc->pAppInClusterList )
              {
                ICall_free( pItem->epDesc.simpleDesc->pAppInClusterList );
              }
              if ( pItem->epDesc.simpleDesc->pAppInClusterList )
              {
                ICall_free( pItem->epDesc.simpleDesc->pAppInClusterList );
              }
              ICall_free( pItem->epDesc.simpleDesc );
            }

            osal_memset( &(pItem->epDesc), 0, sizeof(endPointDesc_t) );
          }
        }

        // If status is still bad
        if ( (status == afStatus_INVALID_PARAMETER)
              || (status == ZApsDuplicateEntry) )
        {
          pPtr->hdr.status = zstack_ZStatusValues_ZAfDuplicateEndpoint;
        }
      }
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processAfUnRegisterReq
 *
 * @brief       Process the AF Unregister Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processAfUnRegisterReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_afUnRegisterReq_t *pPtr = (zstackmsg_afUnRegisterReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    epItem_t *pItem;

    pItem = epTableFindEntryEP( (uint8)pPtr->pReq->endpoint );
    if ( pItem == NULL )
    {
      pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
    }
    else
    {
      pPtr->hdr.status = (zstack_ZStatusValues)afDelete(
            pItem->epDesc.endPoint );

      if ( pPtr->hdr.status == zstack_ZStatusValues_ZSuccess )
      {
        epTableRemoveEntry( pItem );
      }
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processAfConfigGetReq
 *
 * @brief       Process the AF Config Get Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processAfConfigGetReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_afConfigGetReq_t *pPtr = (zstackmsg_afConfigGetReq_t *)pMsg;

  if ( pPtr->pReq && pPtr->pRsp )
  {
    afAPSF_Config_t config;

    afAPSF_ConfigGet( pPtr->pReq->endpoint, &config );

    pPtr->pRsp->endpoint = pPtr->pReq->endpoint;
    pPtr->pRsp->frameDelay = config.frameDelay;
    pPtr->pRsp->windowSize = config.windowSize;

    pPtr->hdr.status = zstack_ZStatusValues_ZSuccess;
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processAfConfigSetReq
 *
 * @brief       Process the AF Config Set Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processAfConfigSetReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_afConfigSetReq_t *pPtr = (zstackmsg_afConfigSetReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    afAPSF_Config_t config;

    config.frameDelay = (uint8)pPtr->pReq->frameDelay;
    config.windowSize = (uint8)pPtr->pReq->windowSize;

    pPtr->hdr.status = (zstack_ZStatusValues)afAPSF_ConfigSet(
          pPtr->pReq->endpoint, &config );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

/**************************************************************************************************
 * @fn          processAfDataReq
 *
 * @brief       Process the AF Data Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processAfDataReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_afDataReq_t *pPtr = (zstackmsg_afDataReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    if ( epTableFindEntryEP( pPtr->pReq->srcEndpoint ) == NULL )
    {
      pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
    }
    else
    {
      endPointDesc_t *pEPDesc = NULL;
      afAddrType_t dstAddr;
      uint8 transId = pPtr->pReq->transID;
      uint8 txOptions = convertMsgTransOptions( &pPtr->pReq->options );

      pEPDesc = afFindEndPointDesc( pPtr->pReq->srcEndpoint );
      if ( pEPDesc )
      {
        dstAddr.endPoint = pPtr->pReq->dstAddr.endpoint;
        dstAddr.panId = pPtr->pReq->dstAddr.panID;
        dstAddr.addrMode = (afAddrMode_t)pPtr->pReq->dstAddr.addrMode;
        if ( (dstAddr.addrMode == afAddr16Bit)
              || (dstAddr.addrMode == afAddrGroup)
              || (dstAddr.addrMode == afAddrBroadcast) )
        {
          dstAddr.addr.shortAddr = pPtr->pReq->dstAddr.addr.shortAddr;
        }
        else if ( dstAddr.addrMode == afAddr64Bit )
        {
          osal_memcpy( dstAddr.addr.extAddr,
                &(pPtr->pReq->dstAddr.addr.extAddr), Z_EXTADDR_LEN );
        }

        if ( pPtr->pReq->n_relayList )
        {
          uint16 *pList = osal_mem_alloc(
                sizeof(uint16) * pPtr->pReq->n_relayList );
          if ( pList )
          {
            int i;
            for ( i = 0; i < pPtr->pReq->n_relayList; i++ )
            {
              pList[i] = (uint16)pPtr->pReq->pRelayList[i];
            }

            pPtr->hdr.status = (zstack_ZStatusValues)AF_DataRequestSrcRtg(
                  &dstAddr, pEPDesc, pPtr->pReq->clusterID, pPtr->pReq->n_payload,
                  pPtr->pReq->pPayload, &transId, txOptions, pPtr->pReq->radius,
                  pPtr->pReq->n_relayList, pList );

            osal_mem_free( pList );
          }
          else
          {
            pPtr->hdr.status = zstack_ZStatusValues_ZMemError;
          }
        }
        else
        {
          pPtr->hdr.status = (zstack_ZStatusValues)AF_DataRequest( &dstAddr,
                pEPDesc, pPtr->pReq->clusterID,
                pPtr->pReq->n_payload,
                pPtr->pReq->pPayload, &transId,
                txOptions,
                pPtr->pReq->radius );
        }
      }
      else
      {
        pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
      }
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

#if defined (ZDO_NWKADDR_REQUEST)
/**************************************************************************************************
 * @fn          processZdoNwkAddrReq
 *
 * @brief       Process the ZDO Network Address Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoNwkAddrReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoNwkAddrReq_t *pPtr = (zstackmsg_zdoNwkAddrReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_NwkAddrReq(
          pPtr->pReq->ieeeAddr, pPtr->pReq->type,
          pPtr->pReq->startIndex, FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_NWKADDR_REQUEST

#if defined (ZDO_IEEEADDR_REQUEST)
/**************************************************************************************************
 * @fn          processZdoIeeeAddrReq
 *
 * @brief       Process the ZDO IEEE Address Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoIeeeAddrReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoIeeeAddrReq_t *pPtr = (zstackmsg_zdoIeeeAddrReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_IEEEAddrReq(
          pPtr->pReq->nwkAddr, pPtr->pReq->type,
          pPtr->pReq->startIndex, FALSE );

  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_IEEEADDR_REQUEST

#if defined (ZDO_NODEDESC_REQUEST)
/**************************************************************************************************
 * @fn          processZdoNodeDescReq
 *
 * @brief       Process the ZDO Node Descriptor Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoNodeDescReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoNodeDescReq_t *pPtr = (zstackmsg_zdoNodeDescReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->dstAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_NodeDescReq( &dstAddr,
          pPtr->pReq->nwkAddrOfInterest, FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif

#if defined (ZDO_POWERDESC_REQUEST)
/**************************************************************************************************
 * @fn          processZdoPowerDescReq
 *
 * @brief       Process the ZDO Power Descriptor Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoPowerDescReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoPowerDescReq_t *pPtr = (zstackmsg_zdoPowerDescReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->dstAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_PowerDescReq( &dstAddr,
          pPtr->pReq->nwkAddrOfInterest, FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif

#if defined (ZDO_SIMPLEDESC_REQUEST)
/**************************************************************************************************
 * @fn          processZdoSimpleDescReq
 *
 * @brief       Process the ZDO Simple Descriptor Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoSimpleDescReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoSimpleDescReq_t *pPtr = (zstackmsg_zdoSimpleDescReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->dstAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_SimpleDescReq( &dstAddr,
          pPtr->pReq->nwkAddrOfInterest,
          pPtr->pReq->endpoint, FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif

#if defined (ZDO_ACTIVEEP_REQUEST)
/**************************************************************************************************
 * @fn          processZdoActiveEndpointsReq
 *
 * @brief       Process the ZDO Active Endpoints Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoActiveEndpointsReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoActiveEndpointReq_t *pPtr = (zstackmsg_zdoActiveEndpointReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->dstAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_ActiveEPReq( &dstAddr,
          pPtr->pReq->nwkAddrOfInterest, FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_ACTIVEEP_REQUEST

#if defined (ZDO_MATCH_REQUEST)
/**************************************************************************************************
 * @fn          processZdoMatchDescReq
 *
 * @brief       Process the ZDO Match Descriptor Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoMatchDescReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoMatchDescReq_t *pPtr = (zstackmsg_zdoMatchDescReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->dstAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_MatchDescReq( &dstAddr,
          pPtr->pReq->nwkAddrOfInterest,
          pPtr->pReq->profileID,
          pPtr->pReq->n_inputClusters,
          pPtr->pReq->pInputClusters,
          pPtr->pReq->n_outputClusters,
          pPtr->pReq->pOutputClusters,
          FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_MATCH_REQUEST

#if defined (ZDO_COMPLEXDESC_REQUEST)
/**************************************************************************************************
 * @fn          processZdoComplexDescReq
 *
 * @brief       Process the ZDO Complex Descriptor Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoComplexDescReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoComplexDescReq_t *pPtr = (zstackmsg_zdoComplexDescReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->dstAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_ComplexDescReq( &dstAddr,
          pPtr->pReq->nwkAddrOfInterest,
          FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_COMPLEXDESC_REQUEST

#if defined (ZDO_SERVERDISC_REQUEST)
/**************************************************************************************************
 * @fn          processZdoServerDiscReq
 *
 * @brief       Process the ZDO Server Discovery Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoServerDiscReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoServerDiscReq_t *pPtr = (zstackmsg_zdoServerDiscReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    uint16 serverMask;

    serverMask = convertMsgServerCapabilities( &pPtr->pReq->serverMask );

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_ServerDiscReq( serverMask, FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_SERVERDISC_REQUEST

#if defined (ZDO_ENDDEVICEBIND_REQUEST)
/**************************************************************************************************
 * @fn          processZdoEndDeviceBindReq
 *
 * @brief       Process the ZDO End Device Bind Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoEndDeviceBindReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoEndDeviceBindReq_t *pPtr = (zstackmsg_zdoEndDeviceBindReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->dstAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_EndDeviceBindReq( &dstAddr,
          pPtr->pReq->bindingTarget,
          pPtr->pReq->endpoint,
          pPtr->pReq->profileID,
          pPtr->pReq->n_inputClusters,
          pPtr->pReq->pInputClusters,
          pPtr->pReq->n_outputClusters,
          pPtr->pReq->pOutputClusters,
          FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_ENDDEVICEBIND_REQUEST

#if defined (ZDO_BIND_UNBIND_REQUEST)
/**************************************************************************************************
 * @fn          processZdoBindReq
 *
 * @brief       Process the ZDO Bind Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoBindReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoBindReq_t *pPtr = (zstackmsg_zdoBindReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;
    zAddrType_t destinationAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->nwkAddr;

    destinationAddr.addrMode = pPtr->pReq->bindInfo.dstAddr.addrMode;
    if ( (destinationAddr.addrMode == afAddr16Bit)
          || (destinationAddr.addrMode == afAddrGroup)
          || (destinationAddr.addrMode == afAddrBroadcast) )
    {
      destinationAddr.addr.shortAddr = pPtr->pReq->bindInfo.dstAddr.addr.shortAddr;
    }
    else if ( destinationAddr.addrMode == afAddr64Bit )
    {
      osal_memcpy( destinationAddr.addr.extAddr,
            pPtr->pReq->bindInfo.dstAddr.addr.extAddr, Z_EXTADDR_LEN );
    }

    if ( (dstAddr.addr.shortAddr == _NIB.nwkDevAddress)
          && (destinationAddr.addrMode == afAddr16Bit) )
    {
      if ( bindAddEntry( pPtr->pReq->bindInfo.srcEndpoint,
              &destinationAddr,
              pPtr->pReq->bindInfo.dstAddr.endpoint,
              1,
              (uint16 *)&(pPtr->pReq->bindInfo.clusterID) ) )
      {
        // Request the IEEE Address of the destination device.
        ZDP_IEEEAddrReq( destinationAddr.addr.shortAddr, 0, 0, FALSE );
        pPtr->hdr.status = zstack_ZStatusValues_ZSuccess;
      }
      else
      {
        pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
      }
    }
    else
    {
      pPtr->hdr.status = (zstack_ZStatusValues)ZDP_BindUnbindReq(
            Bind_req,
            &dstAddr,
            pPtr->pReq->bindInfo.srcAddr,
            pPtr->pReq->bindInfo.srcEndpoint,
            (cId_t)pPtr->pReq->bindInfo.
            clusterID,
            &destinationAddr,
            pPtr->pReq->bindInfo.dstAddr.
            endpoint,
            FALSE );
    }
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_BIND_UNBIND_REQUEST

#if defined (ZDO_BIND_UNBIND_REQUEST)
/**************************************************************************************************
 * @fn          processZdoUnbindReq
 *
 * @brief       Process the ZDO Unbind Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoUnbindReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoUnbindReq_t *pPtr = (zstackmsg_zdoUnbindReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;
    zAddrType_t destinationAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->nwkAddr;

    destinationAddr.addrMode = pPtr->pReq->bindInfo.dstAddr.addrMode;
    if ( (destinationAddr.addrMode == afAddr16Bit)
          || (destinationAddr.addrMode == afAddrGroup)
          || (destinationAddr.addrMode == afAddrBroadcast) )
    {
      destinationAddr.addr.shortAddr = pPtr->pReq->bindInfo.dstAddr.addr.shortAddr;
    }
    else if ( destinationAddr.addrMode == afAddr64Bit )
    {
      osal_memcpy( destinationAddr.addr.extAddr,
            pPtr->pReq->bindInfo.dstAddr.addr.extAddr, Z_EXTADDR_LEN );
    }

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_BindUnbindReq(
          Unbind_req,
          &dstAddr,
          pPtr->pReq->bindInfo.srcAddr,
          pPtr->pReq->bindInfo.srcEndpoint,
          (cId_t)pPtr->pReq->bindInfo.clusterID,
          &destinationAddr,
          pPtr->pReq->bindInfo.dstAddr.endpoint, FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_BIND_UNBIND_REQUEST

#if defined (ZDO_MGMT_NWKDISC_REQUEST)
/**************************************************************************************************
 * @fn          processZdoMgmtNwkDiscReq
 *
 * @brief       Process the ZDO Management Network Discovery Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoMgmtNwkDiscReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoMgmtNwkDiscReq_t *pPtr = (zstackmsg_zdoMgmtNwkDiscReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->nwkAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_MgmtNwkDiscReq(
          &dstAddr, pPtr->pReq->scanChannels,
          pPtr->pReq->scanDuration, pPtr->pReq->startIndex, FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_MGMT_NWKDISC_REQUEST

#if defined (ZDO_MGMT_LQI_REQUEST)
/**************************************************************************************************
 * @fn          processZdoMgmtLqiReq
 *
 * @brief       Process the ZDO Management LQI Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoMgmtLqiReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoMgmtLqiReq_t *pPtr = (zstackmsg_zdoMgmtLqiReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->nwkAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_MgmtLqiReq(
          &dstAddr, pPtr->pReq->startIndex, FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_MGMT_LQI_REQUEST

#if defined (ZDO_MGMT_RTG_REQUEST)
/**************************************************************************************************
 * @fn          processZdoMgmtRtgReq
 *
 * @brief       Process the ZDO Management Routing Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoMgmtRtgReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoMgmtRtgReq_t *pPtr = (zstackmsg_zdoMgmtRtgReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->nwkAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_MgmtRtgReq(
          &dstAddr, pPtr->pReq->startIndex, FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_MGMT_RTG_REQUEST

#if defined (ZDO_MGMT_BIND_REQUEST)
/**************************************************************************************************
 * @fn          processZdoMgmtBindReq
 *
 * @brief       Process the ZDO Management Binding Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoMgmtBindReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoMgmtBindReq_t *pPtr = (zstackmsg_zdoMgmtBindReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->nwkAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_MgmtBindReq(
          &dstAddr, pPtr->pReq->startIndex, FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_MGMT_BIND_REQUEST

#if defined (ZDO_MGMT_LEAVE_REQUEST)
/**************************************************************************************************
 * @fn          processZdoMgmtLeaveReq
 *
 * @brief       Process the ZDO Management Leave Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoMgmtLeaveReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoMgmtLeaveReq_t *pPtr = (zstackmsg_zdoMgmtLeaveReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;

    // Check for broadcast message
    if ( (pPtr->pReq->nwkAddr == 0xFFFF) || (pPtr->pReq->nwkAddr == 0xFFFD)
          || (pPtr->pReq->nwkAddr == 0xFFFC) )
    {
      // Send to self first
      dstAddr.addr.shortAddr = NLME_GetShortAddr( );
      pPtr->hdr.status = (zstack_ZStatusValues)ZDP_MgmtLeaveReq( &dstAddr,
            pPtr->pReq->deviceAddress,
            pPtr->pReq->options.removeChildren,
            pPtr->pReq->options.rejoin,
            FALSE );
    }

    dstAddr.addr.shortAddr = pPtr->pReq->nwkAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_MgmtLeaveReq( &dstAddr,
          pPtr->pReq->deviceAddress,
          pPtr->pReq->options.removeChildren,
          pPtr->pReq->options.rejoin,
          FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_MGMT_LEAVE_REQUEST

#if defined (ZDO_MGMT_JOINDIRECT_REQUEST)
/**************************************************************************************************
 * @fn          processZdoMgmtDirectJoinReq
 *
 * @brief       Process the ZDO Management Direct Join Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoMgmtDirectJoinReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoMgmtDirectJoinReq_t *pPtr = (zstackmsg_zdoMgmtDirectJoinReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;
    uint8 cInfo;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->nwkAddr;

    cInfo = convertCapabilityInfo( &pPtr->pReq->capInfo );

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_MgmtDirectJoinReq( &dstAddr,
          pPtr->pReq->deviceAddress,
          cInfo,
          FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_MGMT_JOINDIRECT_REQUEST

#if defined (ZDO_MGMT_PERMIT_JOIN_REQUEST)
/**************************************************************************************************
 * @fn          processZdoMgmtPermitJoinReq
 *
 * @brief       Process the ZDO Management Permit Join Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoMgmtPermitJoinReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoMgmtPermitJoinReq_t *pPtr = (zstackmsg_zdoMgmtPermitJoinReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;

    dstAddr.addr.shortAddr = pPtr->pReq->nwkAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_MgmtPermitJoinReq( &dstAddr,
          pPtr->pReq->duration,
          pPtr->pReq->tcSignificance,
          FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

#endif // ZDO_MGMT_PERMIT_JOIN_REQUEST

#if defined (ZDO_MGMT_NWKUPDATE_REQUEST)
/**************************************************************************************************
 * @fn          processZdoMgmtNwkUpdateReq
 *
 * @brief       Process the ZDO Management Network Update Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoMgmtNwkUpdateReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoMgmtNwkUpdateReq_t *pPtr = (zstackmsg_zdoMgmtNwkUpdateReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->dstAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_MgmtNwkUpdateReq( &dstAddr,
          pPtr->pReq->channelMask,
          pPtr->pReq->scanDuration,
          pPtr->pReq->scanCount,
          pPtr->pReq->nwkUpdateId,
          pPtr->pReq->nwkMgrAddr );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_MGMT_NWKUPDATE_REQUEST

/**************************************************************************************************
 * @fn          processZdoDeviceAnnounceReq
 *
 * @brief       Process the ZDO Device Announce Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoDeviceAnnounceReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoDeviceAnnounceReq_t *pPtr =
        (zstackmsg_zdoDeviceAnnounceReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    uint8 cInfo;

    cInfo = convertCapabilityInfo( &pPtr->pReq->capabilities );

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_DeviceAnnce(
          pPtr->pReq->nwkAddr, pPtr->pReq->ieeeAddr, cInfo, FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}

#if defined (ZDO_USERDESCSET_REQUEST)
/**************************************************************************************************
 * @fn          processZdoUserDescSetReq
 *
 * @brief       Process the ZDO Management User Descriptor Set Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoUserDescSetReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoUserDescSetReq_t *pPtr = (zstackmsg_zdoUserDescSetReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;
    UserDescriptorFormat_t userDesc;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->dstAddr;

    userDesc.len = pPtr->pReq->n_userDescriptor;
    if ( userDesc.len > AF_MAX_USER_DESCRIPTOR_LEN )
    {
      userDesc.len = AF_MAX_USER_DESCRIPTOR_LEN;
    }
    osal_memcpy( userDesc.desc, pPtr->pReq->pUserDescriptor, userDesc.len );

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_UserDescSet( &dstAddr,
          pPtr->pReq->nwkAddrOfInterest,
          &userDesc,
          FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_USERDESCSET_REQUEST

#if defined (ZDO_USERDESC_REQUEST)
/**************************************************************************************************
 * @fn          processZdoUserDescReq
 *
 * @brief       Process the ZDO User Descriptor Request
 *
 * @param       srcEntityID - Source iCall entity ID
 * @param       pMsg - pointer to message
 *
 * @return      TRUE to send the response back
 */
static bool processZdoUserDescReq( ICall_EntityID srcEntityID, void *pMsg )
{
  zstackmsg_zdoUserDescReq_t *pPtr = (zstackmsg_zdoUserDescReq_t *)pMsg;

  if ( pPtr->pReq )
  {
    zAddrType_t dstAddr;

    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = pPtr->pReq->dstAddr;

    pPtr->hdr.status = (zstack_ZStatusValues)ZDP_UserDescReq( &dstAddr,
          pPtr->pReq->nwkAddrOfInterest, FALSE );
  }
  else
  {
    pPtr->hdr.status = zstack_ZStatusValues_ZInvalidParameter;
  }

  return (TRUE);
}
#endif // ZDO_USERDESC_REQUEST

/**************************************************************************************************
 * @fn          isDevicePartOfNetwork
 *
 * @brief       Checks to see if the device has already joined a network
 *              by looking at _NIB.nwkState === NWK_INIT, then checks
 *              what is stored in NV's NIB.
 *
 * @param       none
 *
 * @return      TRUE if the device is already part of a network, FALSE if not
 */
static bool isDevicePartOfNetwork( void )
{
  bool ret = FALSE;
  if ( (_NIB.nwkState == NWK_ENDDEVICE) || (_NIB.nwkState == NWK_ROUTER) )
  {
    ret = TRUE;
  }
  else if ( _NIB.nwkState == NWK_INIT )
  {
    // Could be that the device hasn't started yet, so check in NV for a NIB
    if ( osal_nv_item_len( ZCD_NV_NIB ) == sizeof(nwkIB_t) )
    {
      nwkIB_t tempNIB;
      if ( osal_nv_read( ZCD_NV_NIB, 0, sizeof(nwkIB_t), &tempNIB )
            == ZSUCCESS )
      {
        if ( (tempNIB.nwkState == NWK_ENDDEVICE)
              || (tempNIB.nwkState == NWK_ROUTER) )
        {
          ret = TRUE;
        }
      }
    }
  }

  return (ret);
}
