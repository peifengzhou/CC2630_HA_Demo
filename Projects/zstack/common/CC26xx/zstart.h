/******************************************************************************
 @headerfile zstart.h
 $Date: 2015-02-08 12:04:59 -0800 (Sun, 08 Feb 2015) $
 $Revision: 42426 $

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
 *****************************************************************************/

#ifndef ZSTART_H
#define ZSTART_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 \defgroup ZStart Manual Device Startup (ZStart), Optional
 <BR>
 IMPORTANT: This module should only be included for Zigbee Pro (ZIGBEEPRO)
 Routers and End Devices.
 <BR><BR>
 This module is an example of how your application can manually control
 the initial joining process, use it as a starting point for your custom
 joining procedure.
 <BR><BR>
 The ZStart module controls the "manual" start up of a device.  The ZStack
 Thread has 2 ways to handle the startup of a device: automatic and manual.
 <BR><BR>
 An application can use the ZStack thread to automatically scan and join,
 by setting the joining parameters (PAD ID, channel list, ...), then calling
 Zstackapi_DevStartReq().
 <BR><BR>
 Or, it can control the joining process by using this module or your version
 of this module.
 <BR><BR>
 First, lets define start up of a device.  To join a network, a device needs
 to know the following:
 - Channel List: What channels to look for networks?
 - PAN ID: Extended PAN ID and/or short PAN ID
 <BR>
 Then, it will scan the channels in the channel list, looking for networks to
 join.  Determine which network to join, then join the network.
 <BR><BR>
 This module is part of the application and is controlled by calling commands
 to start states (Scanning and Joining) and passing events (beacon notify,
 scan confirm, join confirm, state change) to continue processes.  The
 following diagram shows the commands and events passing from the app to
 ZStart and to the ZStack Thread (from ZStart).
 <BR>
 \image html ZStartJoin.PNG
 <BR>
 */

/******************************************************************************
 * INCLUDES
 */

/******************************************************************************
 *  User's  Defines
 */

/******************************************************************************
 * CONSTANTS
 */

/** Blacklist NV ID */
#define ZSTART_BLACKLIST_NV_ID 0x0010

/******************************************************************************
 * TYPEDEFS
 */

/**
 * \ingroup ZStart
 * @{
 */

/**
 * ZStart states
 */
typedef enum
{
    //! Invalid state
    ZSTART_STATE_INVALID,
    //! Initialized, not doing anything
    ZSTART_STATE_INIT,
    //! Currently scanning, waiting for scan confirm, expecting beacons
    ZSTART_STATE_SCANNING,
    //! In between scans, off period
    ZSTART_STATE_SCAN_WAIT,
    //! Scan is complete, results are ready
    ZSTART_STATE_SCAN_COMPLETE,
    //! Scan is complete, no results
    ZSTART_STATE_SCAN_COMPLETE_NO_RESULTS,
    //! Joining, waiting for join confirm
    ZSTART_STATE_JOINING,
    //! Joined, waiting for state change (indicates that security is enabled)
    ZSTART_STATE_JOINED_PREAUTH,
    //! Joined, not doing anything
    ZSTART_STATE_JOINED,
    /**
     * Joining/Rejoining failed, waiting to do something, Zstart_Initialize()
     * needs to be called again.
     */
    ZSTART_STATE_HOLD,
    //! Rejoining, waiting for rejoin confirm
    ZSTART_STATE_REJOINING,
    //! Rejoined, not doing anything
    ZSTART_STATE_REJOINED
} zstart_states;

/**
 * Default Scan Parameters
 */
typedef struct
{
    //! Bit Mask of channels to scan
    uint32_t scanChannels;
    /**
     *  Scanning Time:
     *     0 is 30 milliseconds,
     *     1 is 60 milliseconds,
     *     2 is 120 milliseconds,
     *     3 is 240 milliseconds,
     *     4 is 480 milliseconds,
     *     5 is 960 milliseconds,
     *     6 is 1920 milliseconds,
     *     7 is 3840 milliseconds,
     *     8 is 7680 milliseconds,
     *     9 is 15360 milliseconds,
     *     10 is 30720 milliseconds,
     *     11 is 61440 milliseconds,
     *     12 is 122880 milliseconds,
     *     13 is 245760 milliseconds,
     *     14 is 491520 milliseconds
     */
    uint8_t scanDuration;
    //! Time between scans in 100 millisecond increments
    uint16_t betweenScans;
    //! Number of scans to perform
    uint8_t numScans;
    /**
     * Fixed Extended PANID - anything other than all 0xFF, will filter all
     * other extended PANIDs out
     */
    zstack_LongAddr_t findExtendedPANID;
    /**
     * Fixed PANID - anything other than 0xFFFF, will filter all
     * other extended PANIDs out
     */
    uint16_t findPANID;
} zstart_scanParams;

/**
 * Router information for the Network Discovery List
 */
typedef struct
{
    //! Source address
    uint16_t sourceAddr;
    //! LQI
    uint8_t lqi;
    //! Network depth
    uint8_t depth;
    //! pointer to the next item in the list
    void *next;
} zstack_routerInfo;

/**
 * Network Discovery List - This is built using collected beacon indications
 */
typedef struct
{
    //! PAN ID
    uint16_t panID;
    //! Logical channel
    uint8_t logicalChannel;
    //! Protocol version
    uint8_t protocolVersion;
    //! Stack profile
    uint8_t stackProfile;
    //! 64 bit extended PAN ID
    zstack_LongAddr_t extendedPANID;
    //! Router List
    zstack_routerInfo *routers;
    //! pointer to the next item in the list
    void *next;
} zstart_NwkDiscItem;

/**
 * Function pointer to callback function to set a Clock Event in the app.
 */
typedef void (*zstart_pfnClkCB)(void);

/**
 * ZStart State Parameters
 */
typedef struct
{
    //! Current state
    zstart_states state;
    //! App's ICall entity ID
    ICall_EntityID entityID;
    //! App's semaphore pointer
    ICall_Semaphore *sem;
    //! Apps events pointer
    uint16_t *events;
    //! A place for the app to store event values
    uint16_t eventVal;
    //! Number of scan attempts performed
    uint8_t scans;
    //! The type of this device
    zstack_LogicalTypes deviceType;
    //! true if rejoining, false if joining
    bool rejoining;
    //! App's clock callback
    zstart_pfnClkCB pfnClockCB;
    //! Chosen network index
    uint16_t chosenNetwork;
    //! Chosen router index (within the network)
    uint16_t chosenRouter;
} zstart_params;

/*********************************************************************
 * GLOBALS
 */

/*********************************************************************
 * FUNCTIONS
 */

/**
 * @brief       Called to initialize the manual start process
 *
 * @param       entityID - the calling Apps ICall Entity ID
 * @param       pfnzstart_clockCB - clock callback function pointer
 * @param       deviceType - the device type of this device
 * @param       pfnNV - function pointers for the NV driver
 */
extern void Zstart_Initialize(ICall_EntityID      entityID,
                              zstart_pfnClkCB     pfnzstart_clockCB,
                              zstack_LogicalTypes deviceType,
                              NVINTF_nvFuncts_t *pfnNV);

/**
 * @brief       Called to start the discovery process, which is to
 *              scan for networks
 *
 * @return      zstack_ZStatusValues_ZSuccess if starting
 *              or zstack_ZStatusValues_ZFailure in Zstart_Initialize()
 *              hasn't been called.
 */
extern zstack_ZStatusValues Zstart_discovery(void);

/**
 * @brief       Called to join a device found in the discovery process.
 *              The parameters used to join were discovered by calling
 *              Zstart_discovery() and contained in the Network List
 *              [Zstart_getNetworkList()] and the selected device is in
 *              the chosenNetwork and chosenRouter in the ZStart State
 8              Parameters [Zstart_getParameters()].
 *
 * @return      zstack_ZStatusValues_ZSuccess if starting
 *              or zstack_ZStatusValues_ZFailure if there is nothing to join.
 */
extern zstack_ZStatusValues Zstart_join(void);

/**
 * @brief       Get the parameters.
 *
 * @return      pointer to the zstart parameters
 */
extern zstart_params *Zstart_getParameters(void);

/**
 * @brief       Get the scan parameters.  The user can change these before
 *              calling Zstart_discovery().
 *
 * @return      pointer to the scan parameters
 */
extern zstart_scanParams *Zstart_getScanParameters(void);

/**
 * @brief       Get the Discovered Network List.  This list is
 *              a linked list for discovered networks.  Each
 *              network contains a linked list of routers.
 *              Each item in the network and router lists are allocated,
 *              so don't add or delete from these lists.
 *
 * @return      pointer to the discovered network list
 */
extern zstart_NwkDiscItem *Zstart_getNetworkList(void);

/**
 * @brief       Called to clear the blacklist.  This will cause all NV items
 *              to be deleted and the blacklist cleared.
 */
extern void Zstart_clearBlackList(void);

/**
 * @brief       Called to add an entry to the black list.
 *              This will also save the item into NV.
 *
 * @param       pExtendedPANID - pointer to extended PANID.  The value can't
 *                               be all 0x00 or all 0xFF.
 * @param       router - router value to not join.
 *                       0xFFFF if only pExtendedPANID.
 *
 * @return      true if added, false if not
 */
extern bool Zstart_addToBlackList(uint8_t *pExtendedPANID, uint16_t router);

/**
 * @brief       Get the network in the network list from an index
                into the list.
 *              Example: chosenNetwork in zstart_params.
 *
 * @param       nwkIdx - index into network list
 *
 * @return      pointer to found network or NULL if not found
 */
extern zstart_NwkDiscItem *Zstart_getNwk(uint16_t nwkIdx);

/**
 * @brief       Get the router in the router list from an index into the list.
 *              Example: chosenRouter in zstart_params.
 *
 * @param       pNwk - pointer to the network desc
 * @param       rtrIdx - index into router list
 *
 * @return      pointer to found router or NULL if not found
 */
extern zstack_routerInfo *Zstart_getRouter(zstart_NwkDiscItem *pNwk,
                                           uint16_t            rtrIdx);
/**
 * @brief       Called to process a received Beacon Notification
 *              indication.
 *
 * @param       pBeacon - pointer to beacon notification indication
 */
extern void Zstart_processBeacon(zstack_zdoBeaconNotifyInd_t *pBeacon);

/**
 * @brief       Called to process a received Network Discovery Confirmation.
 *
 * @param       status - MAC Scan status
 *
 * @return      true if discovery process is complete, false if discovery is
 *              continuing
 */
extern bool Zstart_processNwkDiscCnf(uint8_t status);

/**
 * @brief       Called to process a received Join Confirmation.
 *
 * @param       status - join status
 * @param       pJoinCnf - pointer to Join Confirm
 */
extern void Zstart_processJoinCnf(zstack_ZStatusValues  status,
                                  zstack_zdoJoinConf_t *pJoinCnf);

/**
 * @brief       Called to process a state change indication.
 *
 * @param       state - ZStack state
 */
extern zstart_params *Zstart_processStateChange(zstack_DevState state);

/**
 * @brief       Called to process a a clock event.
 */
extern void Zstart_processClockEvt(void);

/** @} end group ZStart */

/*********************************************************************
 *********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZSTART_H */
