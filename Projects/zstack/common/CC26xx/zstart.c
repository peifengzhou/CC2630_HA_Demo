/******************************************************************************
 Filename:       zstart.c
 Revised:        $Date: 2015-02-04 16:34:14 -0800 (Wed, 04 Feb 2015) $
 Revision:       $Revision: 42328 $

 Description:    This file contains the manual start algorithms controlling
 ZStack starting.  These routines will contain the basic
 network discovery and joining functions. The user
 is encouraged to make custom changes for their own needs.


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
 *****************************************************************************/

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

#include "zstackapi.h"
#include "util.h"
#include "znwk_config.h"
#include "nvintf.h"

#include "zstart.h"
#include "zstart_config.h"

#if defined (ZSTACK_MANUAL_START)

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define BROADCAST_ROUTER 0xFFFF
#define MIN_LQI 6
#define EXPECTED_PROTOCOL_VERSION 2   // Network Protocol v1.1
#define ZIGBEEPRO_PROFILE 2

#define EXT_PANID_LEN 8

#define BLACKLIST_MAX 10

/*********************************************************************
 * TYPEDEFS
 */

typedef struct
{
    uint8_t extendedPANID[EXT_PANID_LEN];
    uint16_t router;
    void *next;
}zstart_bl_item;

typedef struct
{
    uint8_t extendedPANID[EXT_PANID_LEN];
    uint16_t router;
}zstart_bl_NVitem;


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
static NVINTF_nvFuncts_t *pfnZstartNV = NULL;

static zstart_scanParams scanParams = {0};
static zstart_params params;

static zstart_NwkDiscItem *nwkList = (zstart_NwkDiscItem *)NULL;
static zstart_bl_item *pBlItems = (zstart_bl_item *)NULL;

// Clock structures
static Clock_Struct zstartClkStruct;
static Clock_Handle zstartClkHandle = (Clock_Handle)NULL;

static const uint8_t emptyPANID[] =
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t dummyPANID [] =
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t configExtendedPANDID[] = ZNWK_CONFIG_EXTENDED_PAN_ID;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zstart_initializeBlackList(void);
static void zstart_freeBlackList(void);
static bool zstart_inBlackList(uint8_t *pExtendedPANID, uint16_t router);
static bool zstart_existBlackList(uint8_t *pExtendedPANID, uint16_t router);

static void zstart_initBlackListNV(void);
static uint16_t zstart_findBlackListNVItem(uint8_t *pExtendedPANID,
                                           uint16_t router);
static void zstart_addBlackListNV(uint8_t *pExtendedPANID, uint16_t router);
static void zstart_deleteBlackListNV(void);
static void zstart_restoreFromBlackListNV(void);

static void zstart_clockCB(UArg a0);
static void zstart_deleteNwkList(void);
static void zstart_updateNwkList(zstack_zdoBeaconNotifyInd_t *pBeacon);
static zstart_NwkDiscItem *zstart_findNwk(zstack_zdoBeaconNotifyInd_t *pBeacon);
static zstack_routerInfo *zstart_findRouter(
    zstack_zdoBeaconNotifyInd_t *pBeacon, zstart_NwkDiscItem *pNwk);
static void zstart_addRouter(zstack_zdoBeaconNotifyInd_t *pBeacon,
                             zstart_NwkDiscItem *         pNwk);
static void zstart_addNetwork(zstack_zdoBeaconNotifyInd_t *pBeacon);
static zstack_ZStatusValues zstart_sendScanReq(void);
static void zstart_writeParameters(uint32_t chanList);

/*********************************************************************
 * PUBLIC FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * API Functions
 *********************************************************************/

/**
 * Called to initialize the manual start process.
 *
 * Public function defined in zstart.h
 */
void Zstart_Initialize(ICall_EntityID      entityID,
                       zstart_pfnClkCB     pfnzstart_clockCB,
                       zstack_LogicalTypes deviceType,
                       NVINTF_nvFuncts_t *pfnNV)
{
    // Initialize variables
    scanParams.scanChannels = ZNWK_DEFAULT_CHANLIST;
    scanParams.scanDuration = ZSTART_SCAN_DURATION;
    scanParams.betweenScans = ZSTART_SCAN_OFF_PERIOD;
    scanParams.numScans = ZSTART_SCAN_ATTEMPTS;
    memcpy(&scanParams.findExtendedPANID, configExtendedPANDID,
           EXT_PANID_LEN);
    scanParams.findPANID = ZNWK_CONFIG_PAN_ID;

    params.state = ZSTART_STATE_INIT;
    params.entityID = entityID;
    params.scans = 0;
    params.deviceType = deviceType;
    params.rejoining = false;
    params.pfnClockCB = pfnzstart_clockCB;
    params.chosenNetwork = 0;
    params.chosenRouter = 0;

    pfnZstartNV = pfnNV;

    // Initialize the clock
    if(zstartClkHandle == NULL)
    {
        zstartClkHandle = Util_constructClock(
            &zstartClkStruct,
            zstart_clockCB,
            (scanParams.betweenScans * 100),
            0,           // no period
            false,    // Don't start now
            (UArg) & params  // pass in pointer to params
            );
    }

    // Clear the network list
    zstart_deleteNwkList();

    // Initialize the black list
    zstart_initializeBlackList();
}

/**
 * Start the Discovery process
 *
 * Public function defined in zstart.h
 */
zstack_ZStatusValues Zstart_discovery(void)
{
    zstack_ZStatusValues ret;

    if(params.state != ZSTART_STATE_INIT)
    {
        return(zstack_ZStatusValues_ZFailure);
    }

    params.state = ZSTART_STATE_SCANNING;
    params.scans = scanParams.numScans;
    params.chosenNetwork = 0;
    params.chosenRouter = 0;
    params.rejoining = false;

    ret = zstart_sendScanReq();
    if(ret != zstack_ZStatusValues_ZSuccess)
    {
        params.state = ZSTART_STATE_HOLD;
    }

    return(ret);
}

/**
 * Called to join a device found in the discovery process.
 *
 * Public function defined in zstart.h
 */
zstack_ZStatusValues Zstart_join(void)
{
    zstart_NwkDiscItem *pNwk;

    pNwk = Zstart_getNwk(params.chosenNetwork);
    if(pNwk)
    {
        zstack_routerInfo *pRouter = Zstart_getRouter(pNwk,
                                                      params.chosenRouter);
        if(pRouter)
        {
            zstack_ZStatusValues ret;
            zstack_devJoinReq_t join;

            join.logicalChannel = pNwk->logicalChannel;
            join.panID = pNwk->panID;
            memcpy(&(join.extendedPANID), &(pNwk->extendedPANID),
                   EXT_PANID_LEN);
            join.chosenParent = pRouter->sourceAddr;
            join.parentDepth = pRouter->depth;
            join.stackProfile = pNwk->stackProfile;

            ret = Zstackapi_DevJoinReq(params.entityID, &join);
            if(ret == zstack_ZStatusValues_ZSuccess)
            {
                params.state = ZSTART_STATE_JOINING;
            }
            else
            {
                params.state = ZSTART_STATE_HOLD;
            }

            return(ret);
        }
    }

    params.state = ZSTART_STATE_HOLD;

    return(zstack_ZStatusValues_ZNwkNoNetworks);
}

/**
 * Called to process a received Beacon Notification indication.
 *
 * Public function defined in zstart.h
 */
void Zstart_processBeacon(zstack_zdoBeaconNotifyInd_t *pBeacon)
{
    // Must be scanning to receive beacon indications
    if(params.state == ZSTART_STATE_SCANNING)
    {
        // Make sure the device has the correct network protocol, minimum LQI,
        // and stack profile (ZigbeePro).
        if( (pBeacon->protocolVersion == EXPECTED_PROTOCOL_VERSION)
            && (pBeacon->lqi > MIN_LQI)
            && (pBeacon->stackProfile == ZIGBEEPRO_PROFILE) )
        {
            // Is this device in the black list
            if(zstart_inBlackList( (uint8_t *)&(pBeacon->extendedPANID),
                                   pBeacon->sourceAddr ) == false)
            {
                /*
                 * The scanParams.findExtendedPANID parameter must be set
                 * to all 0xFFs or equal the extended PAN ID in the beacon.
                 */
                if( (memcmp(&(scanParams.findExtendedPANID), dummyPANID,
                            EXT_PANID_LEN) == 0)
                    || (memcmp(&(scanParams.findExtendedPANID),
                               &(pBeacon->extendedPANID),
                               EXT_PANID_LEN) == 0) )
                {
                    /**
                     * The scanParams.findPANID parameter must be equal to
                     * 0xFFFF or the PAN ID in the beacon.
                     */
                    if( (scanParams.findPANID == 0xFFFF) ||
                        (scanParams.findPANID == pBeacon->panID) )
                    {
                        // Update the network discovery list
                        zstart_updateNwkList(pBeacon);
                    }
                }
            }
        }
    }
}

/**
 * Called to process a received Network Discovery Confirmation.
 *
 * Public function defined in zstart.h
 */
bool Zstart_processNwkDiscCnf(uint8_t status)
{
    // Must be scanning to receive scan confirm
    if(params.state == ZSTART_STATE_SCANNING)
    {
        if(params.scans)
        {
            params.scans--;

            if(scanParams.betweenScans > 0)
            {
                params.state = ZSTART_STATE_SCAN_WAIT;
                if( Util_isClockActive(&zstartClkStruct) )
                {
                    Util_stopClock(&zstartClkStruct);
                }
                Util_startClock(&zstartClkStruct);
            }
            else
            {
                // scan again
                zstack_ZStatusValues ret = zstart_sendScanReq();
                if(ret != zstack_ZStatusValues_ZSuccess)
                {
                    params.state = ZSTART_STATE_HOLD;
                }
            }
        }
        else
        {
            // scans are done, select a device to join
            if(nwkList)
            {
                uint16_t routerCounter = 0;
                uint16_t bestRouterIdx = 0;
                uint8_t bestLqi = 0;
                uint8_t bestDepth = 255;
                // Assume the first network in the list
                zstack_routerInfo *pItem = nwkList->routers;

                // Find the end of the list
                while(pItem)
                {
                    if( (pItem->lqi > bestLqi)
                        || ( (pItem->lqi == bestLqi)
                             && (pItem->depth < bestDepth) ) )
                    {
                        bestRouterIdx = routerCounter;
                        bestLqi = pItem->lqi;
                        bestDepth = pItem->depth;
                    }
                    pItem = pItem->next;
                    routerCounter++;
                }
                params.chosenRouter = bestRouterIdx;
                params.state = ZSTART_STATE_SCAN_COMPLETE;
            }
            else
            {
                params.state = ZSTART_STATE_SCAN_COMPLETE_NO_RESULTS;
            }
            return(true);
        }
    }
    return(false);
}

/**
 * Called to process a received Join Confirmation.
 *
 * Public function defined in zstart.h
 */
void Zstart_processJoinCnf(zstack_ZStatusValues  status,
                           zstack_zdoJoinConf_t *pJoinCnf)
{
    if(params.state == ZSTART_STATE_JOINING)
    {
        if(status == zstack_ZStatusValues_ZSuccess)
        {
            params.state = ZSTART_STATE_JOINED_PREAUTH;
        }
    }
}

/**
 * Called to process a state change indication.
 *
 * Public function defined in zstart.h
 */
zstart_params *Zstart_processStateChange(zstack_DevState state)
{
    if( (state == zstack_DevState_DEV_ROUTER)
        || (state == zstack_DevState_DEV_END_DEVICE) )
    {
        if(params.state == ZSTART_STATE_JOINED_PREAUTH)
        {
            params.state = ZSTART_STATE_JOINED;
        }
        else if(params.state == ZSTART_STATE_REJOINING)
        {
            params.state = ZSTART_STATE_REJOINED;
        }
        else
        {
            // Unknown state
            params.state = ZSTART_STATE_HOLD;
        }

        // Save off default channel list used, for rejoining later
        zstart_writeParameters(ZNWK_DEFAULT_CHANLIST);

        // Free the allocated memory
        params.chosenNetwork = 0;
        params.chosenRouter = 0;
        zstart_deleteNwkList();
        zstart_freeBlackList();

        // Stop clock
        if( Util_isClockActive(&zstartClkStruct) )
        {
            Util_stopClock(&zstartClkStruct);
        }
    }
    else if(state == zstack_DevState_NWK_ORPHAN)
    {
        params.state = ZSTART_STATE_REJOINING;
    }
    else
    {
        // Unknown state
        params.state = ZSTART_STATE_HOLD;
    }
    return(&params);
}

/**
 * Called to process a clock event.
 *
 * Public function defined in zstart.h
 */
void Zstart_processClockEvt(void)
{
    if(params.state == ZSTART_STATE_SCAN_WAIT)
    {
        // scan again
        zstack_ZStatusValues ret = zstart_sendScanReq();
        if(ret != zstack_ZStatusValues_ZSuccess)
        {
            params.state = ZSTART_STATE_HOLD;
        }
        else
        {
            params.state = ZSTART_STATE_SCANNING;
        }
    }
}

/**
 * Get the parameters.
 *
 * Public function defined in zstart.h
 */
zstart_params *Zstart_getParameters(void)
{
    return(&params);
}

/**
 * Get the scan parameters.
 *
 * Public function defined in zstart.h
 */
zstart_scanParams *Zstart_getScanParameters(void)
{
    return(&scanParams);
}

/**
 * Get the Discovered Network List
 *
 * Public function defined in zstart.h
 */
zstart_NwkDiscItem *Zstart_getNetworkList(void)
{
    return(nwkList);
}

/**
 * Called to clear the blacklist.  This will cause all NV items to be deleted
 * and the blacklist cleared.
 *
 * Public function defined in zstart.h
 */
void Zstart_clearBlackList(void)
{
    zstart_freeBlackList();

    zstart_deleteBlackListNV();
}

/**
 * Called to add an entry to the black list.  This will also save the item
 * into NV.
 *
 * Public function defined in zstart.h
 */
bool Zstart_addToBlackList(uint8_t *pExtendedPANID, uint16_t router)
{
    bool ret = false;

    // Validity check
    if( pExtendedPANID && (memcmp(pExtendedPANID,
                                  dummyPANID, EXT_PANID_LEN) != 0) )
    {
        if(zstart_existBlackList(pExtendedPANID, router) == false)
        {
            zstart_bl_item *pAddItem;
            pAddItem = ICall_malloc( sizeof(zstart_bl_item) );
            if(pAddItem)
            {
                zstart_bl_item *pItem = pBlItems;

                // Build new entry
                memcpy(pAddItem->extendedPANID, pExtendedPANID,
                       EXT_PANID_LEN);
                pAddItem->router = router;
                pAddItem->next = NULL;

                // Look for the end of the list
                while(pItem->next)
                {
                    pItem = pItem->next;
                }
                if(pItem)
                {
                    pItem->next = pAddItem;
                }
                else
                {
                    pBlItems = pAddItem;
                }

                // Add entry to NV
                zstart_addBlackListNV(pExtendedPANID, router);
                ret = true;
            }
        }
    }
    return(ret);
}

/**
 * Get the network in the network list.
 *
 * Public function defined in zstart.h
 */
zstart_NwkDiscItem *Zstart_getNwk(uint16_t nwkIdx)
{
    uint16_t counter = 0;
    zstart_NwkDiscItem *pItem = nwkList;

    while(pItem)
    {
        if(nwkIdx == counter)
        {
            break;
        }
        counter++;
        pItem = pItem->next;
    }

    return( (zstart_NwkDiscItem *)pItem );
}

/**
 * @brief       Get the router in the router list
 *
 * @param       pNwk - pointer to the network desc
 * @param       rtrIdx - index into router list
 *
 * @return      pointer to found router or NULL if not found
 */
zstack_routerInfo *Zstart_getRouter(zstart_NwkDiscItem *pNwk, uint16_t rtrIdx)
{
    uint16_t counter = 0;
    zstack_routerInfo *pItem = pNwk->routers;

    while(pItem)
    {
        if(rtrIdx == counter)
        {
            break;
        }
        counter++;
        pItem = pItem->next;
    }

    return( (zstack_routerInfo *)pItem );
}

/*********************************************************************
 * Local Functions
 *********************************************************************/

/**
 * @brief       Call to initialize the blacklist
 *
 * @param       none
 *
 * @return      none
 */
static void zstart_initializeBlackList(void)
{
    zstart_freeBlackList();

    // Initialize the NV needed for the blacklist
    zstart_initBlackListNV();

    // Read black list from NV
    zstart_restoreFromBlackListNV();
}

/**
 * @brief       Call to free the blacklist stored in RAM
 *
 * @param       none
 *
 * @return      none
 */
static void zstart_freeBlackList(void)
{
    zstart_bl_item *pItem = pBlItems;
    while(pItem)
    {
        zstart_bl_item *pTempItem = pItem;
        pItem = pItem->next;
        ICall_free(pTempItem);
    }
    pBlItems = (zstart_bl_item *)NULL;
}

/**
 * @brief       Call to check if the device is in the black list
 *
 * @param       pExtendedPANID - pointer to extended PAN ID, can't be
 *              all 0x00 or all 0xFF
 * @param       router - router network address, can't be 0xFFFF
 *
 * @return      true if item exists, false if it doesn't
 */
static bool zstart_inBlackList(uint8_t *pExtendedPANID, uint16_t router)
{
    zstart_bl_item *pItem = pBlItems;

    while(pItem)
    {
        if( (memcmp(pExtendedPANID, pItem->extendedPANID,
                    EXT_PANID_LEN) == 0)
            && ( (pItem->router == BROADCAST_ROUTER)
                 || (router == pItem->router) ) )
        {
            return(true);
        }
        pItem = pItem->next;
    }

    return(false);
}

/**
 * @brief       Call to check if the exact parameters exist in the list
 *
 * @param       pExtendedPANID - pointer to extended PAN ID
 * @param       router - router network address
 *
 * @return      true if item exists, false if it doesn't
 */
static bool zstart_existBlackList(uint8_t *pExtendedPANID, uint16_t router)
{
    zstart_bl_item *pItem = pBlItems;

    while(pItem)
    {
        if( (memcmp(pExtendedPANID, pItem->extendedPANID,
                    EXT_PANID_LEN) == 0)
            && (router == pItem->router) )
        {
            return(true);
        }
        pItem = pItem->next;
    }

    return(false);
}

/**
 * @brief       Call to initialize the NV Black List
 *
 * @param       none
 *
 * @return      none
 */
static void zstart_initBlackListNV(void)
{
    if(pfnZstartNV && pfnZstartNV->createItem)
    {
        /*
         * Save NV space for the defined number
         * of items in the blacklist
         */
        uint16_t x;
        for(x = 0; x < BLACKLIST_MAX; x++)
        {
            uint32_t nvLen;
            uint32_t len = sizeof(zstart_bl_NVitem);
            NVINTF_itemID_t nvId;

            // Load the NV ID header
            nvId.systemID = NVINTF_SYSID_APP;
            nvId.itemID = (uint16_t)ZSTART_BLACKLIST_NV_ID;
            nvId.subID = (uint16_t)x;

            // Check if item already exists
            nvLen = pfnZstartNV->getItemLen(nvId);
            if(nvLen != len)
            {
                // Create new item
                pfnZstartNV->createItem(nvId, len, NULL);
            }
        }
    }
}

/**
 * @brief       Call to find a black list item in NV
 *
 * @param       pExtendedPANID - pointer to the extended PAN ID
 * @param       router - router address to black list
 *
 * @return      NV index (subID) or BLACKLIST_MAX if not found
 */
static uint16_t zstart_findBlackListNVItem(uint8_t *pExtendedPANID,
                                           uint16_t router)
{
    // Default to not found
    uint16_t x = BLACKLIST_MAX;
    if(pfnZstartNV && pfnZstartNV->readItem)
    {
        // Loop through the list in NV to find item
        for(x = 0; x < BLACKLIST_MAX; x++)
        {
            zstart_bl_NVitem item;
            NVINTF_itemID_t nvId;

            // Fill in the NV ID header
            nvId.systemID = NVINTF_SYSID_APP;
            nvId.itemID = (uint16_t)ZSTART_BLACKLIST_NV_ID;
            nvId.subID = (uint16_t)x;

            // Read NV Item
            if(pfnZstartNV->readItem(nvId, 0, sizeof(zstart_bl_NVitem),
                                     &item) == NVINTF_SUCCESS)
            {
                // Compare to values passed in to find
                if( (memcmp(item.extendedPANID,
                            pExtendedPANID, EXT_PANID_LEN) == 0)
                    && (item.router == router) )
                {
                    // Item is found, break out of the loop to report
                    break;
                }
            }
        }
    }

    // Report the index found
    return(x);
}

/**
 * @brief       Call to add a black list item to NV
 *
 * @param       pExtendedPANID - pointer to the extended PAN ID
 * @param       router - router address to black list
 *
 * @return      none
 */
static void zstart_addBlackListNV(uint8_t *pExtendedPANID, uint16_t router)
{
    // Check if driver is available
    if(pfnZstartNV && pfnZstartNV->writeItem)
    {
        uint16_t addIdx;

        // Look to see if item already exists
        addIdx = zstart_findBlackListNVItem(pExtendedPANID, router);
        if(addIdx == BLACKLIST_MAX)
        {
            // Find an empty slot in the NV
            addIdx = zstart_findBlackListNVItem( (uint8_t *)emptyPANID, 0xFFFF );
            if(addIdx < BLACKLIST_MAX)
            {
                zstart_bl_NVitem item;
                NVINTF_itemID_t nvId;

                // Make a NV structure
                memcpy(item.extendedPANID, pExtendedPANID, EXT_PANID_LEN);
                item.router = router;

                // Update the NV ID header
                nvId.systemID = NVINTF_SYSID_APP;
                nvId.itemID = (uint16_t)ZSTART_BLACKLIST_NV_ID;
                nvId.subID = (uint16_t)addIdx;

                // Write out to NV
                pfnZstartNV->writeItemEx(nvId, 0,
                                         sizeof(zstart_bl_NVitem), &item);
            }
        }
    }
}

/**
 * @brief       Call to delete all black list items stored in NV
 *
 * @param       none
 *
 * @return      none
 */
static void zstart_deleteBlackListNV(void)
{
    // Check if driver is available
    if(pfnZstartNV && pfnZstartNV->writeItem)
    {
        uint16_t x;
        zstart_bl_NVitem item;
        NVINTF_itemID_t nvId;

        // Fill in the NV ID header
        nvId.systemID = NVINTF_SYSID_APP;
        nvId.itemID = (uint16_t)ZSTART_BLACKLIST_NV_ID;

        // Default the information to erased values
        memset(item.extendedPANID, 0xFF, EXT_PANID_LEN);
        item.router = 0xFFFF;

        // Loop through all items and write erased values
        for(x = 0; x < BLACKLIST_MAX; x++)
        {
            // Set the record number
            nvId.subID = (uint16_t)x;

            // Write out to NV
            pfnZstartNV->writeItemEx(nvId, 0, sizeof(zstart_bl_NVitem), &item);
        }
    }
}

/**
 * @brief       Call to restore the black list from what is stored in NV.
 *
 * @param       none
 *
 * @return      none
 */
static void zstart_restoreFromBlackListNV(void)
{
    // Check if driver is available
    if(pfnZstartNV && pfnZstartNV->readItem)
    {
        uint16_t x;

        // Loop through all entries, read and write to local table
        for(x = 0; x < BLACKLIST_MAX; x++)
        {
            zstart_bl_NVitem item;
            NVINTF_itemID_t nvId;

            // Fill in the NV ID header
            nvId.systemID = NVINTF_SYSID_APP;
            nvId.itemID = (uint16_t)ZSTART_BLACKLIST_NV_ID;
            nvId.subID = (uint16_t)x;

            // Read the record from NV
            if(pfnZstartNV->readItem(nvId, 0, sizeof(zstart_bl_NVitem),
                                     &item) == NVINTF_SUCCESS)
            {
                // If the item read isn't an empty slot, save it to list
                if(memcmp(item.extendedPANID,
                          emptyPANID, EXT_PANID_LEN) != 0)
                {
                    // Save to local RAM table
                    Zstart_addToBlackList(item.extendedPANID, item.router);
                }
            }
        }
    }
}

/**
 * @brief       Clock callback function
 *
 * @param       a0 - passed in variables
 *
 * @return      none
 */
static void zstart_clockCB(UArg a0)
{
    zstart_params *pParams = (zstart_params *)a0;

    if(pParams && pParams->pfnClockCB)
    {
        pParams->pfnClockCB();
    }
}

/**
 * @brief       Release all the allocated memory in the network list
 *
 * @param       none
 *
 * @return      none
 */
static void zstart_deleteNwkList(void)
{
    zstart_NwkDiscItem *pItem = nwkList;

    while(pItem)
    {
        zstack_routerInfo *pRtrItem = pItem->routers;
        zstart_NwkDiscItem *pTempItem = pItem;
        pItem = pItem->next;

        while(pRtrItem)
        {
            zstack_routerInfo *pTempRtrItem = pRtrItem;
            pRtrItem = pRtrItem->next;
            ICall_free(pTempRtrItem);
        }
        ICall_free(pTempItem);
    }
    nwkList = (zstart_NwkDiscItem *)NULL;
}

/**
 * @brief       update the network list
 *
 * @param       pBeacon - pointer to a beacon notification
 *
 * @return      none
 */
static void zstart_updateNwkList(zstack_zdoBeaconNotifyInd_t *pBeacon)
{
    // Check capacity
    if( (params.rejoining || pBeacon->permitJoining)
        && ( ( (params.deviceType == zstack_LogicalTypes_ROUTER)
               && (pBeacon->routerCapacity) )
             || ( (params.deviceType == zstack_LogicalTypes_ENDDEVICE)
                  && (pBeacon->deviceCapacity) ) ) )
    {
        static zstart_NwkDiscItem *pNwkItem;
        pNwkItem = zstart_findNwk(pBeacon);
        if(pNwkItem)
        {
            zstack_routerInfo *pRtrItem;
            pRtrItem = zstart_findRouter(pBeacon, pNwkItem);
            if(pRtrItem)
            {
                // Update record
                pRtrItem->lqi = pBeacon->lqi;
            }
            else
            {
                // Add new router
                zstart_addRouter(pBeacon, pNwkItem);
            }
        }
        else
        {
            // Add new network
            zstart_addNetwork(pBeacon);
        }
    }
}

/**
 * @brief       Find the network in the network list
 *
 * @param       pBeacon - pointer to a beacon notification
 *
 * @return      pointer to found network or NULL if not found
 */
static zstart_NwkDiscItem *zstart_findNwk(
    zstack_zdoBeaconNotifyInd_t *pBeacon)
{
    zstart_NwkDiscItem *pItem = nwkList;

    while(pItem)
    {
        if( (memcmp(&(pBeacon->extendedPANID),
                    &(pItem->extendedPANID), EXT_PANID_LEN) == 0)
            && (pBeacon->logicalChannel = pItem->logicalChannel) )
        {
            break;
        }
        pItem = pItem->next;
    }

    return( (zstart_NwkDiscItem *)pItem );
}

/**
 * @brief       Find the router in the router list
 *
 * @param       pBeacon - pointer to a beacon notification
 * @param       pNwk - pointer to the network desc
 *
 * @return      pointer to found router or NULL if not found
 */
static zstack_routerInfo *zstart_findRouter(
    zstack_zdoBeaconNotifyInd_t *pBeacon, zstart_NwkDiscItem *pNwk)
{
    zstack_routerInfo *pItem = pNwk->routers;

    while(pItem)
    {
        if(pItem->sourceAddr == pBeacon->sourceAddr)
        {
            break;
        }
        pItem = pItem->next;
    }

    return( (zstack_routerInfo *)pItem );
}

/**
 * @brief       Add a new router to the router list
 *
 * @param       pBeacon - pointer to a beacon notification
 * @param       pNwk - pointer to the network desc
 *
 * @return      none
 */
static void zstart_addRouter(zstack_zdoBeaconNotifyInd_t *pBeacon,
                             zstart_NwkDiscItem *         pNwk)
{
    zstack_routerInfo *pNewItem;

    pNewItem = (zstack_routerInfo *)ICall_malloc( sizeof(zstack_routerInfo) );
    if(pNewItem)
    {
        zstack_routerInfo *pItem = pNwk->routers;

        pNewItem->sourceAddr = pBeacon->sourceAddr;
        pNewItem->lqi = pBeacon->lqi;
        pNewItem->depth = pBeacon->depth;
        pNewItem->next = NULL;

        // Find the end of the list
        while(pItem && pItem->next)
        {
            pItem = pItem->next;
        }

        // Was there a list to begin with
        if(pItem)
        {
            // Add to the end of the list
            pItem->next = pNewItem;
        }
        else
        {
            // First item in the list
            pNwk->routers = pNewItem;
        }
    }
}

/**
 * @brief       Add a new network to the network list
 *
 * @param       pBeacon - pointer to a beacon notification
 *
 * @return      none
 */
static void zstart_addNetwork(zstack_zdoBeaconNotifyInd_t *pBeacon)
{
    zstart_NwkDiscItem *pNewItem;

    pNewItem = (zstart_NwkDiscItem *)ICall_malloc( sizeof(zstart_NwkDiscItem) );
    if(pNewItem)
    {
        zstart_NwkDiscItem *pItem = nwkList;

        pNewItem->panID = pBeacon->panID;
        pNewItem->logicalChannel = pBeacon->logicalChannel;
        pNewItem->protocolVersion = pBeacon->protocolVersion;
        pNewItem->stackProfile = pBeacon->stackProfile;
        memcpy(&(pNewItem->extendedPANID), &(pBeacon->extendedPANID),
               EXT_PANID_LEN);
        pNewItem->routers = (zstack_routerInfo *)NULL;
        pNewItem->next = NULL;

        // Since this is a new network, add its first router
        zstart_addRouter(pBeacon, pNewItem);

        // Find the end of the list
        while(pItem && pItem->next)
        {
            pItem = pItem->next;
        }

        // Was there a list to begin with
        if(pItem)
        {
            // Add to the end of the list
            pItem->next = pNewItem;
        }
        else
        {
            // First item in the list
            nwkList = pNewItem;
        }
    }
}

/**
 * @brief       Send the Network Discovery request
 *
 * @param       none
 *
 * @return      zstack_ZStatusValues
 */
static zstack_ZStatusValues zstart_sendScanReq(void)
{
    zstack_devNwkDiscReq_t discReq;

    discReq.scanChannels = scanParams.scanChannels;
    discReq.scanDuration = scanParams.scanDuration;

    return( Zstackapi_DevNwkDiscReq(
                params.entityID,
                &discReq) );
}

/**
 * @brief   Write the parameters to zstack
 *
 * @param   chanList - channel list to change to
 *
 * @return  none
 */
static void zstart_writeParameters(uint32_t chanList)
{
    zstack_sysConfigWriteReq_t writeReq = {0};

    writeReq.has_chanList = true;
    writeReq.chanList = chanList;

    (void)Zstackapi_sysConfigWriteReq(params.entityID, &writeReq);
}

#endif // ZSTACK_MANUAL_START
/*********************************************************************
 *********************************************************************/

