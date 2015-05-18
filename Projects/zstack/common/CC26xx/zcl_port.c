/******************************************************************************
   Filename:       zcl_port.c
   Revised:        $Date: 2015-02-12 12:55:11 -0800 (Thu, 12 Feb 2015) $
   Revision:       $Revision: 42532 $

   Description:    This file contains the ZCL porting layer.


   Copyright 2014 - 2015 Texas Instruments Incorporated. All rights reserved.

   IMPORTANT: Your use of this Software is limited to those specific rights
   granted under the terms of a software license agreement between the user
   who downloaded the software, his/her employer (which must be your employer)
   and Texas Instruments Incorporated (the "License").  You may not use this
   Software unless you agree to abide by the terms of the License. The License
   limits your use, and you acknowledge, that the Software may not be modified,
   copied or distributed unless used solely and exclusively in conjunction with
   a Texas Instruments radio frequency transceiver, which is integrated into
   your product.  Other than for the foregoing purpose, you may not use,
   reproduce, copy, prepare derivative works of, modify, distribute, perform,
   display or sell this Software and/or its documentation for any purpose.

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
******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <string.h>
#include <inc/hw_ints.h>
#include "ICall.h"

#include "zstackapi.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_port.h"

#if defined (ZSTACK_MANUAL_START)
#include "zstart.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#if !defined (ZCL_PORT_MAX_ENDPOINTS)
#define ZCL_PORT_MAX_ENDPOINTS 5
#endif

#define EXT_ADDR_LEN 8

/*********************************************************************
 * TYPEDEFS
 */
typedef struct
{
    ICall_EntityID entity;
    endPointDesc_t *pEpDesc;
#if defined (ZCL_SCENES)
    zclGeneral_Scene_t scene;
#endif
} zclPort_entityEPDesc_t;

#if defined (ZCL_SCENES)
// Scene NV types
typedef struct zclGenSceneNVItem
{
    uint8 endpoint;
    zclGeneral_Scene_t scene;
}zclGenSceneNVItem_t;
#endif

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static NVINTF_nvFuncts_t *pfnZclPortNV = NULL;
#if defined (ZCL_SCENES)
static uint16_t zclSceneNVID = ZCL_PORT_SCENE_TABLE_NV_ID;
#endif

#if defined (ZCL_GROUPS)
static aps_Group_t foundGrp;
#endif

static zclPort_entityEPDesc_t entityEPDescs[ZCL_PORT_MAX_ENDPOINTS] = {0};

static zstack_sysNwkInfoReadRsp_t nwkInfo =
{
    0xFFFE,
    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    zstack_DevState_HOLD,
    0xFFFF,
    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    0xFFFE,
    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
    { 0, 0, 0 },
    0
};

#if defined (ZCL_EZMODE)
static pfnEZModeTimerCB ezModeTimerCB = NULL;
#endif

#if defined (ZCL_SCENES)
static uint8_t lastFindSceneEndpoint = 0xFF;
#endif

// Function pointer for applications to ZCL Handle External
static zclport_pFnZclHandleExternal pfnZclHandleExternal = NULL;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void convertTxOptions(zstack_TransOptions_t *pOptions, uint8 options);
endPointDesc_t *afFindEndPointDesc(uint8 EndPoint);

/*********************************************************************
* PUBLIC FUNCTIONS
*********************************************************************/

/*********************************************************************
* API Functions
*********************************************************************/

/**
 * Register an AF Endpoint.  This is needed by the ZCL code
 * to find an AF endpoint descriptor.
 *
 * Public function defined in zcl_port.h
 */
bool zclport_registerEndpoint(ICall_EntityID entity, endPointDesc_t *pEpDesc)
{
    int x;

    for(x = 0; x < ZCL_PORT_MAX_ENDPOINTS; x++)
    {
        if(entityEPDescs[x].pEpDesc == NULL)
        {
            zstack_afRegisterReq_t regReq = {0};
            zstack_SimpleDescriptor_t simpleDesc;

            // Save information to local table
            entityEPDescs[x].entity = entity;
            entityEPDescs[x].pEpDesc = pEpDesc;

            // Register an endpoint with the stack thread
            simpleDesc.endpoint = pEpDesc->endPoint;
            simpleDesc.profileID = pEpDesc->simpleDesc->AppProfId;
            simpleDesc.deviceID = pEpDesc->simpleDesc->AppDeviceId;
            simpleDesc.deviceVer = pEpDesc->simpleDesc->AppDevVer;
            simpleDesc.n_inputClusters = pEpDesc->simpleDesc->AppNumInClusters;
            simpleDesc.pInputClusters = pEpDesc->simpleDesc->pAppInClusterList;
            simpleDesc.n_outputClusters =
                pEpDesc->simpleDesc->AppNumOutClusters;
            simpleDesc.pOutputClusters =
                pEpDesc->simpleDesc->pAppOutClusterList;
            regReq.endpoint = pEpDesc->endPoint;
            regReq.pSimpleDesc = &simpleDesc;
            regReq.latencyReq = zstack_NetworkLatency_NO_LATENCY_REQS;
            (void)Zstackapi_AfRegisterReq(entity, &regReq);

            return(true);
        }
    }

    return(false);
}

/**
 * Call to register the functions pointers for the NV driver.
 *
 * Public function defined in zcl_port.h
 */
void zclport_registerNV(NVINTF_nvFuncts_t *pfnNV, uint16_t sceneNVID)
{
    pfnZclPortNV = pfnNV;
#if defined (ZCL_SCENES)
    zclSceneNVID = sceneNVID;
#endif
}

/**
 * Call to register a function pointer to handle zcl_HandleExternal() messages.
 *
 * Public function defined in zcl_port.h
 */
void zclport_registerZclHandleExternal(zclport_pFnZclHandleExternal pfn)
{
    pfnZclHandleExternal = pfn;
}


/**
 * Call to get Device Information.
 *
 * Public function defined in zcl_port.h
 */
zstack_sysNwkInfoReadRsp_t *zclport_getDeviceInfo(ICall_EntityID entity)
{
    Zstackapi_sysNwkInfoReadReq(entity, &nwkInfo);
    return(&nwkInfo);
}

/**
 * Determines if the device is already part of a network by asking the
 * stack thread.
 *
 * Public function defined in zcl_port.h
 */
bool zclport_isAlreadyPartOfNetwork(ICall_EntityID entity)
{
    zstack_sysConfigReadReq_t readReq = {0};
    zstack_sysConfigReadRsp_t readRsp = {0};

    // Ask if the device is already part of a network
    readReq.devPartOfNetwork = true;

    (void)Zstackapi_sysConfigReadReq(entity, &readReq, &readRsp);

    return(readRsp.devPartOfNetwork);
}

#if defined (ZCL_EZMODE)
/**
 * Call to register a function for the EZMode commissioning
 * timeout callback function
 *
 * Public function defined in zcl_port.h
 */
void zclport_registerEZModeTimerCB(pfnEZModeTimerCB pFn)
{
    ezModeTimerCB = pFn;
}
#endif // ZCL_EZMODE

/**
 * If the NV item does not already exist, it is created and
 * initialized with the data passed to the function, if any.
 *
 * Public function defined in zcl_port.h
 */
uint8_t zclport_initializeNVItem(uint16_t id, uint16_t subId, uint16_t len,
                                 void *buf)
{
    if(pfnZclPortNV && pfnZclPortNV->createItem)
    {
        uint32_t nvLen = 0;
        NVINTF_itemID_t nvId;

        nvId.systemID = NVINTF_SYSID_APP;
        nvId.itemID = (uint16_t)id;
        nvId.subID = (uint16_t)subId;

        if(pfnZclPortNV->getItemLen)
        {
            nvLen = pfnZclPortNV->getItemLen(nvId);
        }

        if(nvLen == len)
        {
            // Already exists and length is good
            return(SUCCESS);
        }

        if(pfnZclPortNV->createItem(nvId, len, buf) == NVINTF_FAILURE)
        {
            // Operation failed
            return(NV_OPER_FAILED);
        }
    }

    // NV was created
    return(NV_ITEM_UNINIT);
}

/**
 * Write a data item to NV. Function can write an entire item to NV or
 * an element of an item by indexing into the item with an offset.
 *
 * Public function defined in zcl_port.h
 */
uint8_t zclport_writeNV(uint16_t id, uint16_t subId, uint16_t ndx,
                        uint16_t len,
                        void *buf)
{
    uint8 rtrn = SUCCESS;

    if(pfnZclPortNV && pfnZclPortNV->writeItem)
    {
        uint32 nvLen = 0;
        NVINTF_itemID_t nvId;

        nvId.systemID = NVINTF_SYSID_APP;
        nvId.itemID = (uint16_t)id;
        nvId.subID = (uint16_t)subId;

        if(pfnZclPortNV->getItemLen)
        {
            nvLen = pfnZclPortNV->getItemLen(nvId);
        }

        if(nvLen > 0)
        {
            if(pfnZclPortNV->writeItemEx(nvId, ndx, len, buf)
               == NVINTF_FAILURE)
            {
                rtrn = NV_OPER_FAILED;
            }
        }
        else
        {
            rtrn = NV_ITEM_UNINIT;
        }
    }

    return rtrn;
}

/**
 * Read data from NV. This function can be used to read an entire item
 * from NV or an element of an item by indexing into the item with an
 * offset. Read data is copied into buf.
 *
 * Public function defined in zcl_port.h
 */
uint8_t zclport_readNV(uint16_t id, uint16_t subId, uint16_t ndx, uint16_t len,
                       void *buf)
{
    uint8 ret = SUCCESS;

    if(pfnZclPortNV && pfnZclPortNV->readItem)
    {
        NVINTF_itemID_t nvId;

        nvId.systemID = NVINTF_SYSID_APP;
        nvId.itemID = (uint16_t)id;
        nvId.subID = (uint16_t)subId;

        if(pfnZclPortNV->readItem(nvId, ndx, len, buf) == NVINTF_FAILURE)
        {
            ret = NV_OPER_FAILED;
        }
    }

    return(ret);
}

/*********************************************************************
 * @fn      zcl_HandleExternal
 *
 * @brief   Callback function to handle messages externally
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  TRUE
 */
uint8 zcl_HandleExternal(zclIncoming_t *pInMsg)
{
    // Did the application register to handle this message
    if(pfnZclHandleExternal)
    {
        // Let the application handle it
        return(pfnZclHandleExternal(pInMsg));
    }
    else
    {
        return(TRUE);
    }
}

/*********************************************************************
 * @fn          zcl_mem_alloc
 *
 * @brief       Abstraction function to allocate memory
 *
 * @param       size - size in bytes needed
 *
 * @return      pointer to allocated buffer, NULL if nothing allocated.
 */
void *zcl_mem_alloc(uint16 size)
{
    return( (void *)ICall_malloc(size) );
}

/*********************************************************************
 * @fn      zcl_memset
 *
 * @brief   Abstract function to memset
 *
 * @param   dest - buffer to set
 * @param   value - value to set into memory
 * @param   len - length to set in memory
 *
 * @return  pointer to buffer after set
 */
void *zcl_memset(void *dest, uint8 value, int len)
{
    return( (void *)memset(dest, value, len) );
}

/*********************************************************************
 * @fn      zcl_memcpy
 *
 * @brief   Generic memory copy.
 *
 *   Note: This function differs from the standard memcpy(), since
 *         it returns the pointer to the next destination uint8. The
 *         standard memcpy() returns the original destination address.
 *
 * @param   dst - pointer to destination memory
 * @param   src - pointer to source memroy
 * @param   len - length to copy
 *
 * @return  pointer to buffer after set
 */
void *zcl_memcpy(void *dst, void *src, unsigned int len)
{
    uint8 *pDst = dst;
    uint8 *pSrc = src;

    while(len--)
    {
        *pDst++ = *pSrc++;
    }

    return(pDst);
}

/*********************************************************************
 * @fn      zcl_cpyExtAddr
 *
 * @brief   Copy an extended address.
 *
 * @param   dst - pointer to destination memory
 * @param   src - pointer to source memory
 *
 * @return  pointer to buffer after set
 */
void *zcl_cpyExtAddr(uint8 *pDest, const uint8 *pSrc)
{
    return zcl_memcpy( (void *)pDest, (void *)pSrc, EXT_ADDR_LEN );
}

/*********************************************************************
 * @fn      zcl_mem_free
 *
 * @brief   Abstract function to free allocated memory
 *
 * @param   ptr - pointer to allocated memory
 */
void zcl_mem_free(void *ptr)
{
    ICall_free(ptr);
}

/*********************************************************************
 * @fn      zcl_buffer_uint32
 *
 * @brief   Abstract function to break a uin32 into a buffer
 *
 * @param   buf - pointer to destination memory
 * @param   val - value to break
 *
 * @return  pointer to buffer after set
 */
uint8 *zcl_buffer_uint32(uint8 *buf, uint32 val)
{
    *buf++ = BREAK_UINT32(val, 0);
    *buf++ = BREAK_UINT32(val, 1);
    *buf++ = BREAK_UINT32(val, 2);
    *buf++ = BREAK_UINT32(val, 3);

    return buf;
}

/*********************************************************************
 * @fn      zcl_build_uint32
 *
 * @brief   Abstract function to build a uint32 from an array of bytes
 *
 * @param   swapped - array of bytes
 * @param   len - length of array
 *
 * @return  uint32 value
 */
uint32 zcl_build_uint32(uint8 *swapped, uint8 len)
{
    if(len == 2)
    {
        return( BUILD_UINT32(swapped[0], swapped[1], 0L, 0L) );
    }
    else if(len == 3)
    {
        return( BUILD_UINT32(swapped[0], swapped[1], swapped[2], 0L) );
    }
    else if(len == 4)
    {
        return( BUILD_UINT32(swapped[0], swapped[1], swapped[2], swapped[3]) );
    }
    else
    {
        return( (uint32)swapped[0] );
    }
}

/*********************************************************************
 * @fn      zclPortFind
 *
 * @brief   Find the endpoint descriptor from endpoint
 *
 * @param   EndPoint - endpoint
 *
 * @return  pointer to found endpoint descriptor, NULL if not found
 */
zclPort_entityEPDesc_t *zclPortFind(uint8 EndPoint)
{
    int x;

    for(x = 0; x < ZCL_PORT_MAX_ENDPOINTS; x++)
    {
        if( entityEPDescs[x].pEpDesc
            && (EndPoint == entityEPDescs[x].pEpDesc->endPoint) )
        {
            return(&entityEPDescs[x]);
        }
    }

    return( (zclPort_entityEPDesc_t *)NULL );
}

/*********************************************************************
 * @fn      zclPortFindEntity
 *
 * @brief   Find the ICall entity ID from endpoint
 *
 * @param   EndPoint - endpoint
 *
 * @return  ICall entity ID
 */
ICall_EntityID zclPortFindEntity(uint8 EndPoint)
{
    int x;

    for(x = 0; x < ZCL_PORT_MAX_ENDPOINTS; x++)
    {
        if( (entityEPDescs[x].pEpDesc)
            && (EndPoint == entityEPDescs[x].pEpDesc->endPoint) )
        {
            return(entityEPDescs[x].entity);
        }
    }

    return( (ICall_EntityID)NULL );
}

/*********************************************************************
 * @fn      afFindEndPointDesc
 *
 * @brief   Find the endpoint descriptor from endpoint
 *
 * @param   EndPoint - endpoint
 *
 * @return  pointer to found endpoint descriptor, NULL if not found
 */
endPointDesc_t *afFindEndPointDesc(uint8 EndPoint)
{
    int x;

    for(x = 0; x < ZCL_PORT_MAX_ENDPOINTS; x++)
    {
        if( (entityEPDescs[x].pEpDesc)
            && (EndPoint == entityEPDescs[x].pEpDesc->endPoint) )
        {
            return(entityEPDescs[x].pEpDesc);
        }
    }

    return( (endPointDesc_t *)NULL );
}

/*********************************************************************
 * @fn      AF_DataRequest
 *
 * @brief   Common functionality for invoking APSDE_DataReq() for both
 *          SendMulti and MSG-Send.
 *
 *          NOTE: this is a conversion function
 *
 * input parameters
 *
 * @param  *dstAddr - Full ZB destination address: Nwk Addr + End Point.
 * @param  *srcEP - Origination (i.e. respond to or ack to) End Point Descr.
 * @param   cID - A valid cluster ID as specified by the Profile.
 * @param   bufLen - Number of bytes of data pointed to by next param.
 * @param  *buf - A pointer to the data bytes to send.
 * @param  *transID - A pointer to a byte which can be modified and which will
 *                    be used as the transaction sequence number of the msg.
 * @param   options - Valid bit mask of Tx options.
 * @param   radius - Normally set to AF_DEFAULT_RADIUS.
 *
 * output parameters
 *
 * @param  *transID - Incremented by one if the return value is success.
 *
 * @return  afStatus_t - See previous definition of afStatus_... types.
 */
afStatus_t AF_DataRequest(afAddrType_t *dstAddr, endPointDesc_t *srcEP,
                          uint16 cID, uint16 bufLen, uint8 *buf,
                          uint8 *transID, uint8 options,
                          uint8 radius)
{
    afStatus_t status;
    zstack_afDataReq_t req;

    memset(&req, 0, sizeof(zstack_afDataReq_t));

    req.dstAddr.addrMode = (zstack_AFAddrMode)dstAddr->addrMode;
    if(req.dstAddr.addrMode == zstack_AFAddrMode_EXT)
    {
        memcpy(req.dstAddr.addr.extAddr, dstAddr->addr.extAddr, EXT_ADDR_LEN);
    }
    else if(req.dstAddr.addrMode != zstack_AFAddrMode_NONE)
    {
        req.dstAddr.addr.shortAddr = dstAddr->addr.shortAddr;
    }
    req.dstAddr.endpoint = dstAddr->endPoint;
    req.dstAddr.panID = dstAddr->panId;

    convertTxOptions(&(req.options), options);

    req.srcEndpoint = srcEP->endPoint;

    req.clusterID = cID;

    req.transID = *transID;
    *transID++;

    req.radius = radius;

    req.n_payload = bufLen;
    req.pPayload = buf;

    status = Zstackapi_AfDataReq(zclPortFindEntity(srcEP->endPoint), &req);

    return(status);
}

/******************************************************************************
* @fn          convertTxOptions
*
* @brief       Convert uint8 txOptions into PB TransOptions data type
*
* @param       pOptions - TransOptions pointer
* @param       options - txOptions
*
* @return      none
******************************************************************************/
static void convertTxOptions(zstack_TransOptions_t *pOptions, uint8 options)
{
    if(options & AF_WILDCARD_PROFILEID)
    {
        pOptions->wildcardProfileID = TRUE;
    }

    if(options & AF_ACK_REQUEST)
    {
        pOptions->ackRequest = TRUE;
    }

    if(options & AF_LIMIT_CONCENTRATOR)
    {
        pOptions->limitConcentrator = TRUE;
    }

    if(options & AF_SUPRESS_ROUTE_DISC_NETWORK)
    {
        pOptions->suppressRouteDisc = TRUE;
    }

    if(options & AF_EN_SECURITY)
    {
        pOptions->apsSecurity = TRUE;
    }

    if(options & AF_SKIP_ROUTING)
    {
        pOptions->skipRouting = TRUE;
    }
}

#if defined (ZCL_GROUPS)
/*********************************************************************
* APS Interface messages
*********************************************************************/

/*********************************************************************
 * @fn      aps_RemoveGroup
 *
 * @brief   Remove a group with endpoint and groupID
 *
 * @param   endpoint -
 * @param   groupID - ID to look forw group
 *
 * @return  TRUE if removed, FALSE if not found
 */
uint8 aps_RemoveGroup(uint8 endpoint, uint16 groupID)
{
    uint8 status;
    zstack_apsRemoveGroup_t req;

    req.endpoint = endpoint;
    req.groupID = groupID;

    status = Zstackapi_ApsRemoveGroupReq(zclPortFindEntity(endpoint), &req);

    return(status);
}

/*********************************************************************
 * @fn      aps_RemoveAllGroup
 *
 * @brief   Remove a groups with an endpoint
 *
 * @param   endpoint -
 * @param   groupID - ID to look for group
 *
 * @return  none
 */
void aps_RemoveAllGroup(uint8 endpoint)
{
    zstack_apsRemoveAllGroups_t req;

    req.endpoint = endpoint;

    Zstackapi_ApsRemoveAllGroupsReq(zclPortFindEntity(endpoint), &req);
}

/*********************************************************************
 * @fn      aps_FindAllGroupsForEndpoint
 *
 * @brief   Find all the groups with endpoint
 *
 * @param   endpoint - endpoint to look for
 * @param   groupList - List to hold group IDs (should hold
 *                      APS_MAX_GROUPS entries)
 *
 * @return  number of groups copied to groupList
 */
uint8 aps_FindAllGroupsForEndpoint(uint8 endpoint, uint16 *groupList)
{
    zstack_apsFindAllGroupsReq_t req;
    zstack_apsFindAllGroupsRsp_t rsp = {0};

    req.endpoint = endpoint;

    if( (groupList)
        && (Zstackapi_ApsFindAllGroupsReq(
                zclPortFindEntity(endpoint),
                &req, &rsp) == zstack_ZStatusValues_ZSuccess) )
    {
        uint8_t x;
        for(x = 0; x < rsp.numGroups; x++)
        {
            groupList[x] = rsp.pGroupList[x];
        }

        if(rsp.pGroupList)
        {
            ICall_free(rsp.pGroupList);
        }
    }

    return( (uint8)rsp.numGroups );
}

/*********************************************************************
 * @fn      aps_FindGroup
 *
 * @brief   Find a group with endpoint and groupID
 *
 * @param   endpoint -
 * @param   groupID - ID to look forw group
 *
 * @return  a pointer to the group information, NULL if not found
 */
aps_Group_t *aps_FindGroup(uint8 endpoint, uint16 groupID)
{
    aps_Group_t *pFound = (aps_Group_t *)NULL;
    zstack_apsFindGroupReq_t req;
    zstack_apsFindGroupRsp_t rsp;

    req.endpoint = endpoint;
    req.groupID = groupID;

    if(Zstackapi_ApsFindGroupReq(zclPortFindEntity(endpoint),
                                 &req, &rsp) == zstack_ZStatusValues_ZSuccess)
    {
        memset( &foundGrp, 0, sizeof(aps_Group_t) );
        foundGrp.ID = rsp.groupID;
        if(rsp.pName)
        {
            if(rsp.n_name <= APS_GROUP_NAME_LEN)
            {
                memcpy(foundGrp.name, rsp.pName, rsp.n_name);
            }
            ICall_free(rsp.pName);
        }

        pFound = &foundGrp;
    }

    return(pFound);
}

/*********************************************************************
 * @fn      aps_AddGroup
 *
 * @brief   Add a group for an endpoint
 *
 * @param   endpoint -
 * @param   group - new group
 *
 * @return  ZStatus_t
 */
ZStatus_t aps_AddGroup(uint8 endpoint, aps_Group_t *group)
{
    uint8 status = zstack_ZStatusValues_ZFailure;
    zstack_apsAddGroup_t req;

    memset( &req, 0, sizeof(zstack_apsAddGroup_t) );

    req.endpoint = endpoint;

    if(group)
    {
        uint8_t len;

        req.groupID = group->ID;
        len = strlen( (const char *)group->name );
        if(len)
        {
            req.n_name = len;
            req.pName = group->name;
        }

        status = Zstackapi_ApsAddGroupReq(zclPortFindEntity(endpoint), &req);
    }

    return(status);
}

/*********************************************************************
 * @fn      aps_CountAllGroups
 *
 * @brief   Count the total number of groups
 *
 * @param   none
 *
 * @return  number of groups
 */
uint8 aps_CountAllGroups(void)
{
    // Slight cheat, don't know the endpoint - use first entity
    return( Zstackapi_ApsCountAllGroupsReq(entityEPDescs[0].entity) );
}
#endif // defined(ZCL_GROUPS)

#if defined (ZCL_EZMODE)
/*********************************************************************
 * @fn      zcl_EZModeSendMatchDescReq
 *
 * @brief   Call to send a ZDP Match Descriptor Request
 *
 * @param   srcID - source endpoint
 * @param   dstAddr - destination address
 * @param   nwkAddr - network address of interest
 * @param   NumInClusters - number of input clusters
 * @param   InClusterList - input cluster ID list
 * @param   NumOutClusters - number of output clusters
 * @param   OutClusterList - output cluster ID list
 *
 * @return      afStatus_t
 */
afStatus_t zcl_EZModeSendMatchDescReq(uint8 srcID, zAddrType_t *dstAddr,
                                      uint16 nwkAddr, uint8 NumInClusters,
                                      uint16 *InClusterList,
                                      uint8 NumOutClusters,
                                      uint16 *OutClusterList)
{
    zstack_zdoMatchDescReq_t req;
    endPointDesc_t *pEpDesc = afFindEndPointDesc(srcID);

    if( (pEpDesc == NULL) || (pEpDesc->simpleDesc == NULL) || (dstAddr == NULL) )
    {
        return afStatus_INVALID_PARAMETER;
    }

    req.dstAddr = dstAddr->addr.shortAddr;
    req.nwkAddrOfInterest = nwkAddr;
    req.profileID = pEpDesc->simpleDesc->AppProfId;
    req.n_inputClusters = NumInClusters;
    req.pInputClusters = InClusterList;
    req.n_outputClusters = NumOutClusters;
    req.pOutputClusters = OutClusterList;

    return Zstackapi_ZdoMatchDescReq(zclPortFindEntity(srcID), &req);
}

/*********************************************************************
 * @fn      zcl_EZModeStartTimer
 *
 * @brief   This function is called to start a timer to expire in n mSecs.
 *          When the timer expires, call zcl_EZModeAction().
 *
 * @param   uint8 taskID - task id to set timer for
 * @param   uint16 event_id - event to be notified with
 * @param   uint32 timeout_value - in milliseconds.
 *
 * @return  SUCCESS, or NO_TIMER_AVAIL.
 */
afStatus_t zcl_EZModeStartTimer(uint8 task_id, uint16 event_id,
                                uint32 timeout_value)
{
    if(ezModeTimerCB)
    {
        ezModeTimerCB(true, event_id, timeout_value);
        return(SUCCESS);
    }
    else
    {
        return(NO_TIMER_AVAIL);
    }
}

/*********************************************************************
 * @fn      zcl_EZModeStopTimer
 *
 * @brief   This function is called to cancel a timer.
 *
 * @param   uint8 taskID - task id to set timer for
 * @param   uint16 event_id - event to be notified with
 *
 * @return  SUCCESS, or NO_TIMER_AVAIL.
 */
afStatus_t zcl_EZModeStopTimer(uint8 task_id, uint16 event_id)
{
    if(ezModeTimerCB)
    {
        ezModeTimerCB(false, event_id, 0);
        return(SUCCESS);
    }
    else
    {
        return(NO_TIMER_AVAIL);
    }
}

/*********************************************************************
 * @fn      zcl_EZModePermitJoiningRequest
 *
 * @brief   Call to set the permit joining for device and network.
 *
 * @param   srcID - source endpoint
 * @param   dstAddr - destination address of the message (short addr only),
 *                    NULL to send to local device only.
 * @param   duration - Permit duration
 *
 * @return  none
 */
void zcl_EZModePermitJoiningRequest(uint8 srcID, zAddrType_t *dstAddr,
                                    uint8 duration)
{
    zstack_zdoMgmtPermitJoinReq_t req;

    req.duration = duration;
    req.tcSignificance = true;

    if(dstAddr == NULL)
    {
        req.nwkAddr = nwkInfo.nwkAddr;
    }
    else
    {
        req.nwkAddr = dstAddr->addr.shortAddr;
    }

    Zstackapi_ZdoMgmtPermitJoinReq(zclPortFindEntity(srcID), &req);
}

/*********************************************************************
 * @fn      zcl_EZModeStartDevice
 *
 * @brief   Call to start a device joining.
 *
 * @param   srcID - source endpoint
 * @param   startDelay - timeDelay to start device (in milliseconds)
 *
 * @return  none
 */
void zcl_EZModeStartDevice(uint8 srcID, uint16 startDelay)
{
#if defined (ZSTACK_MANUAL_START)
    Zstart_discovery();
#else
    {
        zstack_devStartReq_t startReq = {0};

        // Start the ZStack Thread
        startReq.startDelay = 0;
        (void)Zstackapi_DevStartReq(zclPortFindEntity(srcID), &startReq);
    }
#endif
}

/*********************************************************************
 * @fn      zcl_EZModeBindAddEntry()
 *
 * @brief   This function is used to Add an entry to the binding table
 *
 * @param       srcEpInt - source endpoint
 * @param       dstAddr - destination Address
 * @param       dstEpInt - destination endpoint
 * @param       numClusterIds - number of cluster Ids in the list
 * @param       clusterIds - pointer to the Object ID list
 *
 * @return  pointer to binding table entry, NULL if not added
 */
void zcl_EZModeBindAddEntry(uint8 srcEpInt, zAddrType_t *dstAddr,
                            uint8 dstEpInt, uint8 numClusterIds,
                            uint16 *clusterIds)
{
    ICall_EntityID entityID;
    int x;
    zstack_zdoBindReq_t req;

    entityID = zclPortFindEntity(srcEpInt);

    req.nwkAddr = nwkInfo.nwkAddr;
    memcpy(req.bindInfo.srcAddr, nwkInfo.ieeeAddr, EXT_ADDR_LEN);
    req.bindInfo.srcEndpoint = srcEpInt;

    req.bindInfo.dstAddr.addrMode = (zstack_AFAddrMode)dstAddr->addrMode;
    req.bindInfo.dstAddr.addr.shortAddr = dstAddr->addr.shortAddr;
    req.bindInfo.dstAddr.endpoint = dstEpInt;
    req.bindInfo.dstAddr.panID = nwkInfo.panId;

    for(x = 0; x < numClusterIds; x++) ;
    {
        req.bindInfo.clusterID = *clusterIds;
        Zstackapi_ZdoBindReq(entityID, &req);
        clusterIds++;
    }
}

/*********************************************************************
 * @fn      zcl_EZModeGetNwkAddr()
 *
 * @brief   This function is used to return the network address.
 *
 *   NOTE: Do not call this function outside of zcl_ezmode.c.
 *   NOTE2:  For ZCL_STANDALONE, this function needs to be implemented
 *           elsewhere.
 *
 * @param       none
 *
 * @return  16 bit network address
 */
uint16 zcl_EZModeGetNwkAddr(void)
{
    return(nwkInfo.nwkAddr);
}
#endif // ZCL_EZMODE

#if defined (ZCL_SCENES)

/*********************************************************************
 * @fn      sceneRecEmpty
 *
 * @brief   Checks for an empty record
 *
 * @param   pNvItem - pointer to scene NV record
 *
 * @return  true if all bytes where 0xFF, false otherwise
 */
static bool sceneRecEmpty(zclGenSceneNVItem_t *pNvItem)
{
    uint8_t *pBuf = (uint8_t *)pNvItem;
    uint16_t x;

    for(x = 0; x < sizeof(zclGenSceneNVItem_t); x++)
    {
        if(*pBuf++ != 0xFF)
        {
            return(false);
        }
    }
    return(true);
}

/*********************************************************************
 * @fn      zclGeneral_ScenesInit
 *
 * @brief   Initialize the Scenes Table
 */
void zclGeneral_ScenesInit(void)
{
    uint16_t x;

    for(x = 0; x < ZCL_GEN_MAX_SCENES; x++)
    {
        zclport_initializeNVItem(zclSceneNVID, x,
                                 sizeof(zclGenSceneNVItem_t), NULL);
    }
}

/*********************************************************************
 * @fn      zclGeneral_RemoveAllScenes
 *
 * @brief   Remove all scenes for an endpoint and groupID
 *
 * @param   endpoint - endpoint to filter with
 * @param   groupID - group ID looking for
 */
void zclGeneral_RemoveAllScenes(uint8 endpoint, uint16 groupID)
{
    uint16_t x;
    zclGenSceneNVItem_t nvItem;

    for(x = 0; x < ZCL_GEN_MAX_SCENES; x++)
    {
        if(zclport_readNV(zclSceneNVID, x, 0,
                          sizeof(zclGenSceneNVItem_t), &nvItem) == SUCCESS)
        {
            if( (sceneRecEmpty(&nvItem) == false)
                && ( (nvItem.endpoint == endpoint) || (endpoint == 0xFF) )
                && (nvItem.scene.groupID == groupID) )
            {
                // Remove the item by setting it all to 0xFF
                memset( &nvItem, 0xFF, sizeof(zclGenSceneNVItem_t) );
                zclport_writeNV(zclSceneNVID, x, 0,
                                sizeof(zclGenSceneNVItem_t), &nvItem);
            }
        }
    }
}

/*********************************************************************
 * @fn      zclGeneral_RemoveScene
 *
 * @brief   Remove a scene
 *
 * @param   endpoint - endpoint to filter with
 * @param   groupID - group ID looking for
 * @param   sceneID - scene ID
 *
 * @return  TRUE if removed, FALSE if not found
 */
uint8 zclGeneral_RemoveScene(uint8 endpoint, uint16 groupID, uint8 sceneID)
{
    uint16_t x;
    zclGenSceneNVItem_t nvItem;

    for(x = 0; x < ZCL_GEN_MAX_SCENES; x++)
    {
        if(zclport_readNV(zclSceneNVID, x, 0,
                          sizeof(zclGenSceneNVItem_t), &nvItem) == SUCCESS)
        {
            if( (sceneRecEmpty(&nvItem) == false)
                && ( (nvItem.endpoint == endpoint) || (endpoint == 0xFF) )
                && (nvItem.scene.groupID == groupID)
                && (nvItem.scene.ID == sceneID) )
            {
                // Remove the item by setting it all to 0xFF
                memset( &nvItem, 0xFF, sizeof(zclGenSceneNVItem_t) );
                if(zclport_writeNV(zclSceneNVID, x, 0,
                                   sizeof(zclGenSceneNVItem_t),
                                   &nvItem) == SUCCESS)
                {
                    return(TRUE);
                }
                else
                {
                    return(FALSE);
                }
            }
        }
    }
    return(FALSE);
}

/*********************************************************************
 * @fn      zclGeneral_FindScene
 *
 * @brief   Find a scene with endpoint and sceneID
 *
 * @param   endpoint - endpoint filter to find scene
 * @param   groupID - what group the scene belongs to
 * @param   sceneID - ID to look for scene
 *
 * @return  a pointer to the scene information, NULL if not found
 */
zclGeneral_Scene_t *zclGeneral_FindScene(uint8 endpoint, uint16 groupID,
                                         uint8 sceneID)
{
    uint16_t x;
    zclGenSceneNVItem_t nvItem;
    zclPort_entityEPDesc_t *pEPDesc = zclPortFind(endpoint);

    if(pEPDesc == NULL)
    {
        for(x = 0; x < ZCL_GEN_MAX_SCENES; x++)
        {
            if(zclport_readNV(zclSceneNVID, x, 0,
                              sizeof(zclGenSceneNVItem_t),
                              &nvItem) == SUCCESS)
            {
                if( (sceneRecEmpty(&nvItem) == false)
                    && ( (nvItem.endpoint == endpoint) || (endpoint == 0xFF) )
                    && (nvItem.scene.groupID == groupID)
                    && (nvItem.scene.ID == sceneID) )
                {
                    lastFindSceneEndpoint = endpoint;

                    // Copy to a temp area
                    memcpy( &(pEPDesc->scene), &(nvItem.scene),
                            sizeof(zclGeneral_Scene_t) );

                    return( &(pEPDesc->scene) );
                }
            }
        }
    }

    return( (zclGeneral_Scene_t *)NULL );
}

/*********************************************************************
 * @fn      zclGeneral_AddScene
 *
 * @brief   Add a scene for an endpoint
 *
 * @param   endpoint -
 * @param   scene - new scene item
 *
 * @return  ZStatus_t
 */
ZStatus_t zclGeneral_AddScene(uint8 endpoint, zclGeneral_Scene_t *scene)
{
    uint16_t x;
    zclGenSceneNVItem_t nvItem;

    // See if the item exists already
    for(x = 0; x < ZCL_GEN_MAX_SCENES; x++)
    {
        if(zclport_readNV(zclSceneNVID, x, 0,
                          sizeof(zclGenSceneNVItem_t), &nvItem) == SUCCESS)
        {
            if( (sceneRecEmpty(&nvItem) == false)
                && (nvItem.endpoint == endpoint)
                && (nvItem.scene.groupID == scene->groupID)
                && (nvItem.scene.ID == scene->ID) )
            {
                break;
            }
        }
    }

    // Find an empty slot
    if(x == ZCL_GEN_MAX_SCENES)
    {
        for(x = 0; x < ZCL_GEN_MAX_SCENES; x++)
        {
            if(zclport_readNV(zclSceneNVID, x, 0,
                              sizeof(zclGenSceneNVItem_t),
                              &nvItem) == SUCCESS)
            {
                if( sceneRecEmpty(&nvItem) )
                {
                    break;
                }
            }
        }
    }

    if(x == ZCL_GEN_MAX_SCENES)
    {
        return(ZFailure);
    }

    // Item found or empty slot found
    nvItem.endpoint = endpoint;
    memcpy( &(nvItem.scene), scene, sizeof(zclGeneral_Scene_t) );

    if(zclport_writeNV(zclSceneNVID, x, 0,
                       sizeof(zclGenSceneNVItem_t), &nvItem) == SUCCESS)
    {
        return(ZSuccess);
    }
    else
    {
        return(ZFailure);
    }
}

/*********************************************************************
 * @fn      zclGeneral_CountAllScenes
 *
 * @brief   Count the number of scenes
 *
 * @return  number of scenes found
 */
uint8 zclGeneral_CountAllScenes(void)
{
    uint16_t x;
    zclGenSceneNVItem_t nvItem;
    uint8 cnt = 0;

    for(x = 0; x < ZCL_GEN_MAX_SCENES; x++)
    {
        if(zclport_readNV(zclSceneNVID, x, 0,
                          sizeof(zclGenSceneNVItem_t), &nvItem) == SUCCESS)
        {
            if(sceneRecEmpty(&nvItem) == false)
            {
                cnt++;
            }
        }
    }

    return(cnt);
}

/*********************************************************************
 * @fn      zclGeneral_FindAllScenesForGroup
 *
 * @brief   Get all the scenes with groupID
 *
 * @param   endpoint - endpoint to filter with
 * @param   groupID - group ID looking for
 *
 * @return  number of scenes found
 */
uint8 zclGeneral_FindAllScenesForGroup(uint8 endpoint, uint16 groupID,
                                       uint8 *sceneList)
{
    uint16_t x;
    zclGenSceneNVItem_t nvItem;
    uint8 cnt = 0;

    for(x = 0; x < ZCL_GEN_MAX_SCENES; x++)
    {
        if(zclport_readNV(zclSceneNVID, x, 0,
                          sizeof(zclGenSceneNVItem_t), &nvItem) == SUCCESS)
        {
            if( (sceneRecEmpty(&nvItem) == false)
                && (nvItem.endpoint == endpoint) &&
                (nvItem.scene.groupID == groupID) )
            {
                sceneList[cnt++] = nvItem.scene.ID;
            }
        }
    }
    return(cnt);
}

/*********************************************************************
 * @fn      zclGeneral_ScenesSave
 *
 * @brief   Save the Scenes Table - Something has changed.
 *          This function is only called if zclGeneral_FindScene()
 *          was called and the found information was changed.
 */
void zclGeneral_ScenesSave(void)
{
    if(lastFindSceneEndpoint != 0xFF)
    {
        zclPort_entityEPDesc_t *pEPDesc = zclPortFind(lastFindSceneEndpoint);
        if(pEPDesc)
        {
            zclGeneral_AddScene( lastFindSceneEndpoint, &(pEPDesc->scene) );
            lastFindSceneEndpoint = 0xFF;
        }
    }
}
#endif // ZCL_SCENES

/*********************************************************************
*********************************************************************/

