/**
@file  switch.c
@brief TI RTOS ZCL Home Automation Switch sample application
interfacing with ZStack.

The application interacts with the ZStack Thread
via both messaging interface and C function interface.

<!--
Copyright 2015 Texas Instruments Incorporated. All rights reserved.

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

/**

Description:

KEY UP (or S1)
- Toggle remote light
|
KEY LEFT (or S4)                |      KEY RIGHT(or S2)
- Enable/Disable Permit Join --+-----  - Invoke EZMode or ZStart
|
|
KEY DOWN

KEY_SELECT (or S5)
-  Go to Help screen


*/

//*****************************************************************************
// Includes
//*****************************************************************************
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

#include <ioc.h>

#include "Board.h"

#include "board_key.h"
#include "board_lcd.h"
#include "board_led.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"

#if defined (ZCL_EZMODE)
#include "zcl_ezmode.h"
#endif

#include "util.h"
#include "zcl_port.h"
#include "znwk_config.h"

#if defined (ZSTACK_MANUAL_START)
#include "zstart.h"
#endif

#include "switch.h"

//*****************************************************************************
// Constants
//*****************************************************************************

/* Event IDs */
#define SWITCH_IDENTIFY_TIMEOUT_EVT       0x0001
#define SWITCH_POLL_CONTROL_TIMEOUT_EVT   0x0002
#define SWITCH_EZMODE_TIMEOUT_EVT         0x0004
#define SWITCH_EZMODE_NEXTSTATE_EVT       0x0008
#define SWITCH_MAIN_SCREEN_EVT            0x0010
#define SWITCH_KEY_EVENT                  0x0020
#if defined (ZSTACK_MANUAL_START)
#define SWITCH_MANUAL_START_CLK_EVT       0x0040
#endif

/* Debounce timeout in ticks */
#define SWITCH_KEY_DEBOUNCE_TIMEOUT       200

#define SWITCH_MAX_ATTRIBUTES             15

#define SWITCH_EP                         6

#define SWITCH_DEVICE_VERSION             0
#define SWITCH_FLAGS                      0

#define SWITCH_HWVERSION                  0
#define SWITCH_ZCLVERSION                 0

#define SWITCH_LIGHT_OFF                  0x00
#define SWITCH_LIGHT_ON                   0x01

#define SWITCH_MAX_INCLUSTERS             3
#define SWITCH_MAX_OUTCLUSTERS            2

// Application Display Modes
#define SWITCH_MAINMODE                   0x00
#define SWITCH_HELPMODE                   0x01

#define SWITCH_MAINSCREEN_TIMEOUT         3000             // 3 Seconds

#define SWITCH_INIT_TIMEOUT_VALUE         100

#define SWITCH_MAX_LCD_LINE               20

#define SWITCH_1SEC_MSEC                  1000

#define SWITCH_CONVERT_TO_SECONDS(a)      ((a)/SWITCH_1SEC_MSEC)

//*****************************************************************************
// Global Variables
//*****************************************************************************

//*****************************************************************************
// Local Variables
//*****************************************************************************

// Semaphore used to post events to the application thread
static ICall_Semaphore sem;
static ICall_EntityID zswEntity;

// Passed in function pointers to the NV driver
static NVINTF_nvFuncts_t *pfnZswNV = NULL;

// Hold the device's Zstack state when recieved
static zstack_DevState savedState = zstack_DevState_HOLD;

// Task pending events
static uint16_t events = 0;

// Destination address to send the On/Off toggle
static zstack_AFAddr_t zswDstAddr;

#if defined (ZCL_EZMODE)
// Transaction ID used to send data messages
static uint8_t zswTransID = 0;
#endif

// ZStack Thread network information
static zstack_sysNwkInfoReadRsp_t *pNwkInfo = NULL;

// Key press parameters
static uint8_t keys;

// Clock/timer resources
static Clock_Struct identifyClkStruct;
static Clock_Handle identifyClkHandle;
static Clock_Struct mainScreenClkStruct;
#if defined (ZCL_EZMODE)
static Clock_Handle mainScreenClkHandle;
#endif

#if defined (ZCL_EZMODE)
// Clock resources
static Clock_Struct ezmodeTimeoutClkStruct;
static Clock_Struct ezmodeStateClkStruct;

// Clock object for Ezmode timeout
static Clock_Handle ezmodeTimeoutClkHandle;
static Clock_Handle ezmodeStateClkHandle;
#endif // ZCL_EZMODE

// Display the main screen mode first
static uint8_t giSwScreenMode = SWITCH_MAINMODE;

// Permit joining default to disabled
static uint8_t gPermitDuration = 0;

// Cluster lists for the simple descriptor
static uint16_t inputClusters[SWITCH_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_GEN_ON_OFF
};
static uint16_t outputClusters[SWITCH_MAX_OUTCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_GEN_ON_OFF
};

//static uint16_t bindingInClusters[] =
//{
//ZCL_CLUSTER_ID_GEN_ON_OFF
//};

static uint16_t bindingOutClusters[] =
{
  ZCL_CLUSTER_ID_GEN_ON_OFF
};


// Endpoint descriptor
static endPointDesc_t zswEpDesc = {0};
static SimpleDescriptionFormat_t afSimpleDesc = {0};

#if defined (ZCL_EZMODE)
// EZMode Commissioning control structure
static void Switch_ezmodeCB(zlcEZMode_State_t state, zclEZMode_CBData_t *pData);
static const zclEZMode_RegisterData_t ezmodeRegisterData =
{
  &zswEpDesc.endPoint,
  SWITCH_EZMODE_NEXTSTATE_EVT,
  SWITCH_EZMODE_TIMEOUT_EVT,
  &zswTransID,
  Switch_ezmodeCB
};
#endif // ZCL_EZMODE

// Strings used to display on the LCD
static const char sDeviceName[] = "Sample Switch";
static const char sClearLine[] = " ";
static const char sSwLight[] = "Up: ToggleLight";
#if defined (ZCL_EZMODE)
static const char sSwEZMode[] = "Rt: EZ-Mode";
#elif defined (ZSTACK_MANUAL_START)
static const char sSwStart[] = "Rt: Start:";
#endif
static const char sSwHelp[] = "Select: Help";
static const char sSwMain[] = "Select: Main";
//static const char sCmdSent[] = "  COMMAND SENT";
static const char zcStr[] = "ZC: ";
static const char zrStr[] = "ZR: ";
static const char zedStr[] = "Zed:";
static const char unknownStr[] = "Unk:";
static const char *const devInfoStrs[] = { zcStr, zrStr, zedStr, unknownStr };

/**
* @internal A semaphore used to wait on a clock event
*/
extern ti_sysbios_knl_Semaphore_Handle semaphore0;

//*****************************************************************************
// Attribute Variables
//*****************************************************************************

// Attributes that aren't writable
static const uint8_t zswHWRevision = SWITCH_HWVERSION;
static const uint8_t zswZCLVersion = SWITCH_ZCLVERSION;
static const uint8_t zswManufacturerName[] =
{
  16,
  'T', 'e', 'x', 'a', 's', 'I', 'n', 's',
  't', 'r', 'u', 'm', 'e', 'n', 't', 's'
};
static const uint8_t zswModelId[] =
{
  16,
  'T', 'I', '0', '0', '0', '1', ' ', ' ',
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '
};
static const uint8_t zswDateCode[] =
{
  16,
  '2', '0', '0', '6', '0', '8', '3', '1',
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '
};
static const uint8_t zswPowerSource = POWER_SOURCE_MAINS_1_PHASE;

// Device location - updatable over the air
static uint8_t zswLocationDescription[17] =
{
  16,
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '
};
static uint8_t zswPhysicalEnvironment = PHY_UNSPECIFIED_ENV;

// Attribute to control the device
static uint8_t zswDeviceEnable = DEVICE_ENABLED;

// Identify Cluster
static uint16_t zswIdentifyTime = 0;

// On/Off Cluster
static uint8_t zswOnOff = SWITCH_LIGHT_OFF;

// Toggle -> Toggle
static uint8_t zswOnOffSwitchType = ON_OFF_SWITCH_TYPE_TOGGLE;
static uint8_t zswOnOffSwitchActions = ON_OFF_SWITCH_ACTIONS_2;


static bool zswbindflag = 0;
//*****************************************************************************
// Local Function Prototypes
//*****************************************************************************

static void Switch_initialization(void);
static void Switch_initializeClocks(void);
static void Switch_initializeZStack(void);
static void Switch_process(void);
static void Switch_processZStackMsgs(zstackmsg_genericReq_t *pMsg);
static void Switch_processAfIncomingMsgInd(zstack_afIncomingMsgInd_t *pInMsg);
//static void Switch_sendToggle(void);
static void Switch_handleKeys(uint8_t keys);

static void Switch_updateLcdDisplay(void);
static void Switch_updateLcdLine1Status(void);
static void Switch_updateLcdMainScreen(void);
static void Switch_updateLcdHelpScreen(void);

static void Switch_processKeyChangeCallback(uint8_t keysPressed);
static void Switch_processIdentifyTimeChange(void);
static void Switch_processIdentifyQueryResponseCallback(
                                                        zclIdentifyQueryRsp_t *pRsp);
static void Switch_processIdentifyCallback(zclIdentify_t *pCmd);

static void Switch_processOnOffCB( uint8 cmd );


#if defined (ZCL_EZMODE)
static void Switch_setEzmodeTimerCallback(bool start, uint16_t event_id,
                                          uint32_t timeout_value);
static void Switch_processEzmodeTimeoutCallback(UArg a0);
static void Switch_processEzmodeStateChangeCallback(UArg a0);
#endif // ZCL_EZMODE

static void Switch_processIdentifyTimeoutCallback(UArg a0);
static void Switch_processMainScreenTimeoutCallback(UArg a0);
static void Switch_setPollRate(uint32_t newPollRate);

#if defined (ZSTACK_MANUAL_START)
static void Switch_processZstartClockCallback(void);
static void Switch_initializeZstartDiscovery(void);
#endif // ZSTACK_MANUAL_START


static bool Switch_RestoreBindflagFromNV(ICall_EntityID entity);


/*********************************************************************
* ZCL General Profile Callback table
*/
static zclGeneral_AppCallbacks_t cmdCallbacks =
{
  NULL,                   // Basic Cluster Reset command
  Switch_processIdentifyCallback,  // Identify command
#ifdef ZCL_EZMODE
  NULL,                   // Identify EZ-Mode Invoke command
  NULL,                   // Identify Update Commission State command
#endif
  NULL,                   // Identify Trigger Effect command
  Switch_processIdentifyQueryResponseCallback, // Identify Query Response cmd
  Switch_processOnOffCB,//NULL,                   // On/Off cluster commands
  NULL,                   // On/Off cluster enhanced command Off with Effect
  NULL,                   // On/Off cluster enhanced command On with Recall
  // Global Scene
  NULL,                   // On/Off cluster enhanced command On with Timed
  // Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                   // Level Control Move to Level command
  NULL,                   // Level Control Move command
  NULL,                   // Level Control Step command
  NULL,                   // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                   // Scene Store Request command
  NULL,                   // Scene Recall Request command
  NULL,                   // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                   // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                   // Get Event Log command
  NULL,                   // Publish Event Log command
#endif
  NULL,                   // RSSI Location command
  NULL                    // RSSI Location Response command
};

const zclAttrRec_t zswAttrs[SWITCH_MAX_ATTRIBUTES] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC, // Cluster IDs - defined in the foundation
    // (ie. zcl.h)
    {
      // Attribute record
      ATTRID_BASIC_HW_VERSION, // Attribute ID - Found in Cluster Library
      // header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,     // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,    // Variable access control - found in zcl.h
      (void *)&zswHWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zswZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zswManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zswModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zswDateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zswPowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)zswLocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zswPhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {
      // Attribute record
      ATTRID_BASIC_DEVICE_ENABLED,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zswDeviceEnable
    }
  },
  
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    {
      // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zswIdentifyTime
    }
  },
  
  // *** On / Off Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    {
      // Attribute record
      ATTRID_ON_OFF,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,
      (void *)&zswOnOff
    }
  },
  
  // *** On / Off Switch Configuration Cluster *** //
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF_SWITCH_CONFIG,
    {
      // Attribute record
      ATTRID_ON_OFF_SWITCH_TYPE,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ,
      (void *)&zswOnOffSwitchType
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF_SWITCH_CONFIG,
    {
      // Attribute record
      ATTRID_ON_OFF_SWITCH_ACTIONS,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zswOnOffSwitchActions
    }
  },
};

/**
* @internal Clock handler function
* @param a0 ignored
*/
Void tirtosapp_clock(UArg a0)
{
  /* Wake up the application thread when it waits for clock event */
  Semaphore_post(semaphore0);
}

/*******************************************************************************
* @fn          Switch_task
*
* @brief       Application task entry point for the ZStack HA Sample Swtich
*              Appication.
*
* @param       pfnNV - pointer to the NV functions
*
* @return      none
*/
void Switch_task(NVINTF_nvFuncts_t *pfnNV)
{
  // Save and register the function pointers to the NV drivers
  pfnZswNV = pfnNV;
  zclport_registerNV(pfnZswNV, ZCL_PORT_SCENE_TABLE_NV_ID);
  
  // Initialize application
  Switch_initialization();
  
  // No return from task process
  Switch_process();
}

/*******************************************************************************
* @fn          Switch_initialization
*
* @brief       Initialize the application
*
* @param       none
*
* @return      none
*/
static void Switch_initialization(void)
{
  /* Initialize variables */
  zswDstAddr.addrMode = zstack_AFAddrMode_NONE;
  zswDstAddr.addr.shortAddr = 0;
  zswDstAddr.endpoint = 0;
  zswDstAddr.panID = 0;
  
#if defined (ZCL_EZMODE)
  zclport_registerEZModeTimerCB(Switch_setEzmodeTimerCallback);
#endif
  
  Switch_initializeClocks();
  
  /* Initialize keys */
  Board_Key_initialize(Switch_processKeyChangeCallback);
  
  /* Initialize the LCD */
  Board_LCD_open();
  LCD_WRITE_STRING( (char *)sDeviceName, LCD_PAGE1 );
#if defined (ZCL_EZMODE)
  LCD_WRITE_STRING( (char *)sSwEZMode, LCD_PAGE2 );
#elif  defined (ZSTACK_MANUAL_START)
  LCD_WRITE_STRING( (char *)sSwStart, LCD_PAGE2 );
#else
  LCD_WRITE_STRING( (char *)sClearLine, LCD_PAGE2 );
#endif
  LCD_WRITE_STRING( (char *)sSwHelp, LCD_PAGE3 );
  
  /* Initialize the LEDS */
  Board_Led_initialize();
  
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&zswEntity, &sem);
  
  // Initialize the ZStack
  Switch_initializeZStack();
  
  zswbindflag = Switch_RestoreBindflagFromNV(zswEntity);
}

/*******************************************************************************
* @fn      Switch_initializeClocks
*
* @brief   Initialize Clocks
*
* @param   none
*
* @return  none
*/
static void Switch_initializeClocks(void)
{
  // Initialize the timers needed for this application
  identifyClkHandle = Util_constructClock(
                                          &identifyClkStruct,
                                          Switch_processIdentifyTimeoutCallback,
                                          SWITCH_INIT_TIMEOUT_VALUE,
                                          0, false, 0);
#if defined (ZCL_EZMODE)
  mainScreenClkHandle =
#endif
    Util_constructClock(
                        &mainScreenClkStruct,
                        Switch_processMainScreenTimeoutCallback,
                        SWITCH_MAINSCREEN_TIMEOUT,
                        0, false,
                        0);
#if defined (ZCL_EZMODE)
  ezmodeTimeoutClkHandle = Util_constructClock(
                                               &ezmodeTimeoutClkStruct,
                                               Switch_processEzmodeTimeoutCallback,
                                               SWITCH_INIT_TIMEOUT_VALUE,
                                               0,
                                               false,
                                               0);
  ezmodeStateClkHandle = Util_constructClock(
                                             &ezmodeStateClkStruct,
                                             Switch_processEzmodeStateChangeCallback,
                                             SWITCH_INIT_TIMEOUT_VALUE,
                                             0,
                                             false,
                                             0);
#endif // ZCL_EZMODE
}

/*******************************************************************************
* @fn      zswRegEndpoints
*
* @brief   Setup a Zigbee HA Switch Endpoint
*
* @param   none
*
* @return  none
*/
static void zswRegEndpoints(void)
{
  // Initialize the Switch Simple Descriptor
  zswEpDesc.endPoint = SWITCH_EP;
  afSimpleDesc.EndPoint = SWITCH_EP;
  afSimpleDesc.AppProfId = ZCL_HA_PROFILE_ID;
  afSimpleDesc.AppDeviceId = ZCL_HA_DEVICEID_ON_OFF_SWITCH;
  afSimpleDesc.AppDevVer = SWITCH_DEVICE_VERSION;
  afSimpleDesc.AppNumInClusters = sizeof(inputClusters) / sizeof(uint16_t);
  afSimpleDesc.pAppInClusterList = inputClusters;
  afSimpleDesc.AppNumOutClusters = sizeof(outputClusters) / sizeof(uint16_t);
  afSimpleDesc.pAppOutClusterList = outputClusters;
  zswEpDesc.simpleDesc = &afSimpleDesc;
  (void)zclport_registerEndpoint(zswEntity, &zswEpDesc);
}

/*******************************************************************************
* @fn      zswSetupZStackCallbacks
*
* @brief   Setup the Zstack Callbacks wanted
*
* @param   none
*
* @return  none
*/
static void zswSetupZStackCallbacks(void)
{
  zstack_devZDOCBReq_t zdoCBReq = {0};
  
  // Register for Callbacks, turn on:
  //  Device State Change,
  //  ZDO Match Descriptor Response,
  zdoCBReq.has_devStateChange = true;
  zdoCBReq.devStateChange = true;
  
#if defined (ZCL_EZMODE)
  zdoCBReq.has_matchDescRsp = true;
  zdoCBReq.matchDescRsp = true;
#endif
  
#if defined (ZSTACK_MANUAL_START)
  // Join Confirmation Indication
  zdoCBReq.has_joinCnfCB = true;
  zdoCBReq.joinCnfCB = true;
#endif
  
  (void)Zstackapi_DevZDOCBReq(zswEntity, &zdoCBReq);
}

/*******************************************************************************
* @fn      zswWriteParameters
*
* @brief   Initialize ZStack Parameters
*
* @param   none
*
* @return  none
*/
static void zswWriteParameters(void)
{
  zstack_sysConfigWriteReq_t writeReq = {0};
  uint8_t extendedPANID[] = ZNWK_CONFIG_EXTENDED_PAN_ID;
  
  // HA specifies no Multicast, use group broadcast
  writeReq.has_nwkUseMultiCast = true;
  writeReq.nwkUseMultiCast = false;
  
  // Update the Default Channel List, defined in znwk_config.h
  writeReq.has_chanList = true;
  writeReq.chanList = ZNWK_DEFAULT_CHANLIST;
  
  // Update the Extended PAN ID, defined in znwk_config.h
  writeReq.has_extendedPANID = true;
  memcpy(&(writeReq.extendedPANID), extendedPANID, EXTADDR_LEN);
  
  // Update the config PAN ID, defined in znwk_config.h
  writeReq.has_panID = true;
  writeReq.panID = ZNWK_CONFIG_PAN_ID;
  
  (void)Zstackapi_sysConfigWriteReq(zswEntity, &writeReq);
}

/*******************************************************************************
* @fn      Switch_initializeZStack
*
* @brief   Initialize ZStack
*
* @param   none
*
* @return  none
*/
static void Switch_initializeZStack(void)
{
  // Initialize the ZStack Thread
  bool startDev = true;  // default to auto-start
  
  // Setup the endpoints
  zswRegEndpoints();
  
  // Setup indications from ZStack
  zswSetupZStackCallbacks();
  
#if defined (ZSTACK_MANUAL_START) || defined (ZCL_EZMODE)
  // Check to see if the device is already part of a network,
  // to see if we need to invoke EZMode or Manual startup
  startDev = zclport_isAlreadyPartOfNetwork(zswEntity);
#endif
  
#if defined (ZSTACK_MANUAL_START)
  // Setup the Manual Start module
  Switch_initializeZstartDiscovery();
#endif
  
#if defined (ZCL_EZMODE)
  {
    // Register EZ-Mode
    zcl_RegisterEZMode(&ezmodeRegisterData);
    Board_Led_control(board_led_type_LED1, board_led_state_BLINK);
  }
#endif
  
  if(startDev)
  {
    zstack_devStartReq_t startReq = {0};
    
    // Start the ZStack Thread
    startReq.startDelay = 0;
    (void)Zstackapi_DevStartReq(zswEntity, &startReq);
    
#if defined (ZCL_EZMODE)
    // Clear the EZ Mode line
    LCD_WRITE_STRING( (char *)sClearLine, LCD_PAGE2 );
#endif
  }
  
  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks(SWITCH_EP, &cmdCallbacks);
  
  // Register the application's attribute list
  zcl_registerAttrList(SWITCH_EP, SWITCH_MAX_ATTRIBUTES, zswAttrs);
  
  // Update the ZStack Parameters
  zswWriteParameters();
}

/*******************************************************************************
* @fn      Switch_process
*
* @brief   Application task processing start.
*
* @param   none
*
* @return  void
*/
static void Switch_process(void)
{
  /* Forever loop */
  for(;;)
  {
    ICall_ServiceEnum stackid;
    ICall_EntityID dest;
    zstackmsg_genericReq_t *pMsg = NULL;
    
    /* Wait for response message */
    if(ICall_wait(ICALL_TIMEOUT_FOREVER) == ICALL_ERRNO_SUCCESS)
    {
      /* Retrieve the response message */
      if(ICall_fetchServiceMsg(&stackid, &dest, (void **)&pMsg)
         == ICALL_ERRNO_SUCCESS)
      {
        if( (stackid == ICALL_SERVICE_CLASS_ZSTACK)
           && (dest == zswEntity) )
        {
          if(pMsg)
          {
            Switch_processZStackMsgs(pMsg);
            
            // Free any separately allocated memory
            Zstackapi_freeIndMsg(pMsg);
          }
        }
        
        if(pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
      
      if(events & SWITCH_KEY_EVENT)
      {
        // Process Key Presses
        Switch_handleKeys(keys);
        keys = 0;
        events &= ~SWITCH_KEY_EVENT;
      }
      
      if(events & SWITCH_IDENTIFY_TIMEOUT_EVT)
      {
        // Process the Identify timer expiration
        if(zswIdentifyTime > 0)
        {
          zswIdentifyTime--;
        }
        Switch_processIdentifyTimeChange();
        
        events &= ~SWITCH_IDENTIFY_TIMEOUT_EVT;
      }
      
      if(events & SWITCH_MAIN_SCREEN_EVT)
      {
        // Update the display
        giSwScreenMode = SWITCH_MAINMODE;
        Switch_updateLcdMainScreen();
        events &= ~SWITCH_MAIN_SCREEN_EVT;
      }
      
#ifdef ZCL_EZMODE
      if(events & SWITCH_EZMODE_NEXTSTATE_EVT)
      {
        // going on to next state
        zcl_EZModeAction(EZMODE_ACTION_PROCESS, NULL);
        events &= ~SWITCH_EZMODE_NEXTSTATE_EVT;
      }
      
      if(events & SWITCH_EZMODE_TIMEOUT_EVT)
      {
        // EZ-Mode timed out
        zcl_EZModeAction(EZMODE_ACTION_TIMED_OUT, NULL);
        events &= ~SWITCH_EZMODE_TIMEOUT_EVT;
      }
#endif // ZLC_EZMODE
      
#if defined (ZSTACK_MANUAL_START)
      if(events & SWITCH_MANUAL_START_CLK_EVT)
      {
        // Manual start timeout
        Zstart_processClockEvt();
        events &= ~SWITCH_MANUAL_START_CLK_EVT;
      }
#endif
    }
  }
}

/*******************************************************************************
* @fn      Switch_processZStackMsgs
*
* @brief   Process event from Stack
*
* @param   pMsg - pointer to incoming ZStack message to process
*
* @return  void
*/
static void Switch_processZStackMsgs(zstackmsg_genericReq_t *pMsg)
{
  switch(pMsg->hdr.event)
  {
  case zstackmsg_CmdIDs_DEV_STATE_CHANGE_IND:
    {
      // The ZStack Thread is indicating a State change
      zstackmsg_devStateChangeInd_t *pInd =
        (zstackmsg_devStateChangeInd_t *)pMsg;
      
      // Only process the state change if it actually changed.
      if(savedState != pInd->req.state)
      {
        // Save the new state
        savedState = pInd->req.state;
        
        if( (pInd->req.state == zstack_DevState_DEV_ZB_COORD)
           || (pInd->req.state == zstack_DevState_DEV_ROUTER)
             || (pInd->req.state == zstack_DevState_DEV_END_DEVICE) )
        {
          // The device is part of a network, get the device's
          // network parameters.
          pNwkInfo = zclport_getDeviceInfo(zswEntity);
          
          // Update the display with network information
          giSwScreenMode = SWITCH_MAINMODE;
          Switch_updateLcdDisplay();
          Board_Led_control(board_led_type_LED1,
                            board_led_state_OFF);
          
          if(pInd->req.state != zstack_DevState_DEV_END_DEVICE)
          {
            // Don't turn on LED if Power saving end device
            Board_Led_control(board_led_type_LED4,
                              board_led_state_ON);
          }
          else
          {
            // Change the default poll rate from 1 second to
            // the config
            // setting (znwk_config.h)
            Switch_setPollRate(ZNWK_POLL_RATE);
            
            //add by zpf to indicate network status
            Board_Led_control(board_led_type_LED4,board_led_state_ON);
          }
          
#if defined (ZCL_EZMODE)
          zcl_EZModeAction(EZMODE_ACTION_NETWORK_STARTED, NULL);
#endif
        }
#if defined (ZSTACK_MANUAL_START)
        {
          zstart_params *pParams = Zstart_processStateChange(
                                                             pInd->req.state);
          if(pParams->state == ZSTART_STATE_JOINED)
          {
            // Device joined
          }
          else if(pParams->state == ZSTART_STATE_REJOINED)
          {
            // Device has rejoined
          }
          else if(pParams->state == ZSTART_STATE_HOLD)
          {
            // Error, do rescan
          }
        }
#endif
      }
    }
    break;
    
#if defined (ZCL_EZMODE)
  case zstackmsg_CmdIDs_ZDO_MATCH_DESC_RSP:
    {
      zstackmsg_zdoMatchDescRspInd_t *pInd
        = (zstackmsg_zdoMatchDescRspInd_t *)pMsg;
      zclEZMode_ActionData_t actionData;
      ZDO_MatchDescRsp_t *pRsp;
      
      /*
      * Parse the Match Descriptor Response and give it to
      * the ZCL EZMode module to process
      */
      pRsp = (ZDO_MatchDescRsp_t *)ICall_allocMsg(
                                                  sizeof(ZDO_MatchDescRsp_t) + pInd->rsp.n_matchList);
      if(pRsp)
      {
        pRsp->status = pInd->rsp.status;
        pRsp->nwkAddr = pInd->rsp.nwkAddrOfInterest;
        pRsp->cnt = pInd->rsp.n_matchList;
        memcpy(pRsp->epList, pInd->rsp.pMatchList,
               pInd->rsp.n_matchList);
        
        actionData.pMatchDescRsp = pRsp;
        zcl_EZModeAction(EZMODE_ACTION_MATCH_DESC_RSP, &actionData);
        ICall_freeMsg(pRsp);
      }
    }
    break;
#endif // ZCL_EZMODE
    
  case zstackmsg_CmdIDs_AF_INCOMING_MSG_IND:
    {
      // Process incoming data messages
      zstackmsg_afIncomingMsgInd_t *pInd =
        (zstackmsg_afIncomingMsgInd_t *)pMsg;
      Switch_processAfIncomingMsgInd( &(pInd->req) );
    }
    break;
    
#if defined (ZSTACK_MANUAL_START)
  case zstackmsg_CmdIDs_ZDO_BEACON_NOTIFY_IND:
    {
      // ZStart will process this message
      zstackmsg_zdoBeaconNotifyInd_t *pInd
        = (zstackmsg_zdoBeaconNotifyInd_t *)pMsg;
      Zstart_processBeacon( &(pInd->req) );
    }
    break;
    
  case zstackmsg_CmdIDs_ZDO_NWK_DISC_CNF:
    {
      // ZStart will process this message
      zstackmsg_zdoNwkDiscCnf_t *pInd =
        (zstackmsg_zdoNwkDiscCnf_t *)pMsg;
      if(Zstart_processNwkDiscCnf(pInd->req.status) == true)
      {
        // Scan process is over
        zstart_params *pParams = Zstart_getParameters();
        if(pParams->state == ZSTART_STATE_SCAN_COMPLETE)
        {
          // Scan is complete and it's time to join
          // In the zstart_params structure, the zstart has
          // selected
          // a network and device to join, you could change it by
          // looking
          // through the found network list
          // [Zstart_getNetworkList()] and
          // setting your own network(pParams->chosenNetwork) and
          // device (pParams->chosenRouter) to join. Or, you
          // could go with
          // what was selected, like this example.
          Zstart_join();
        }
        else if(pParams->state ==
                ZSTART_STATE_SCAN_COMPLETE_NO_RESULTS)
        {
          // No results means that no networks found (that passed
          // filtering)
          // If you would like to delay the start of scanning,
          // setup a timer
          // to start scanning on timeout. For now, start a new
          // discovery now.
          Switch_initializeZstartDiscovery();
          
          Zstart_discovery();
        }
      }
    }
    break;
    
  case zstackmsg_CmdIDs_ZDO_JOIN_CNF:
    {
      zstackmsg_zdoJoinConf_t *pInd =
        (zstackmsg_zdoJoinConf_t *)pMsg;
      zstack_ZStatusValues status =
        (zstack_ZStatusValues)pInd->hdr.status;
      
      // ZStart will process this message
      Zstart_processJoinCnf( status, &(pInd->req) );
      if(status != zstack_ZStatusValues_ZSuccess)
      {
        zstart_params *pParams = Zstart_getParameters();
        if(pParams && pParams->state == ZSTART_STATE_JOINING)
        {
          // Join didn't go well, let's add this device to the
          // blacklist
          // to skip this device next time, and restart the
          // discovery process
          zstart_NwkDiscItem *pNwk;
          
          pNwk = Zstart_getNwk(pParams->chosenNetwork);
          if(pNwk)
          {
            zstack_routerInfo *pRouter = Zstart_getRouter(
                                                          pNwk,
                                                          pParams
                                                            ->chosenRouter);
            if(pRouter)
            {
              Zstart_addToBlackList( (uint8_t *)&(pNwk->
                                                  extendedPANID),
              pRouter->sourceAddr );
            }
          }
          // You could add delay here -> timer, event
          
          // Restart discovery process
          Switch_initializeZstartDiscovery();
          Zstart_discovery();
        }
      }
    }
    break;
#endif // ZSTACK_MANUAL_START

    case zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_RSP:
    {
      zswbindflag = Switch_RestoreBindflagFromNV(zswEntity);
    }
	break;

    /*
    * These are messages/indications from ZStack that this
    * application doesn't process.  These message can be
    * processed by your application, remove from this list and
    * process them here in this switch statement.
    */
  case zstackmsg_CmdIDs_AF_DATA_CONFIRM_IND:
  case zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE:
  case zstackmsg_CmdIDs_ZDO_NWK_ADDR_RSP:
  case zstackmsg_CmdIDs_ZDO_IEEE_ADDR_RSP:
  case zstackmsg_CmdIDs_ZDO_NODE_DESC_RSP:
  case zstackmsg_CmdIDs_ZDO_POWER_DESC_RSP:
  case zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_RSP:
  case zstackmsg_CmdIDs_ZDO_ACTIVE_EP_RSP:
  case zstackmsg_CmdIDs_ZDO_COMPLEX_DESC_RSP:
  case zstackmsg_CmdIDs_ZDO_USER_DESC_RSP:
  case zstackmsg_CmdIDs_ZDO_USER_DESC_SET_RSP:
  case zstackmsg_CmdIDs_ZDO_SERVER_DISC_RSP:
  
  case zstackmsg_CmdIDs_ZDO_BIND_RSP:
  case zstackmsg_CmdIDs_ZDO_UNBIND_RSP:
  case zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_RSP:
  case zstackmsg_CmdIDs_ZDO_MGMT_LQI_RSP:
  case zstackmsg_CmdIDs_ZDO_MGMT_RTG_RSP:
  case zstackmsg_CmdIDs_ZDO_MGMT_BIND_RSP:
  case zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_RSP:
  case zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_RSP:
  case zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_RSP:
  case zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_NOTIFY:
  case zstackmsg_CmdIDs_ZDO_SRC_RTG_IND:
  case zstackmsg_CmdIDs_ZDO_CONCENTRATOR_IND:
  case zstackmsg_CmdIDs_ZDO_LEAVE_CNF:
  case zstackmsg_CmdIDs_ZDO_LEAVE_IND:
  case zstackmsg_CmdIDs_SYS_RESET_IND:
  case zstackmsg_CmdIDs_AF_REFLECT_ERROR_IND:
  case zstackmsg_CmdIDs_ZDO_TC_DEVICE_IND:
  case zstackmsg_CmdIDs_DEV_PERMIT_JOIN_IND:
    break;
    
  default:
    break;
  }
}

/*******************************************************************************
*
* @fn          Switch_processAfIncomingMsgInd
*
* @brief       Process AF Incoming Message Indication message
*
* @param       pInMsg - pointer to incoming message
*
* @return      none
*
******************************************************************************/
static void Switch_processAfIncomingMsgInd(zstack_afIncomingMsgInd_t *pInMsg)
{
  afIncomingMSGPacket_t afMsg;
  
  /*
  * All incoming messages are passed to the ZCL message processor,
  * first convert to a structure that ZCL can process.
  */
  afMsg.groupId = pInMsg->groupID;
  afMsg.clusterId = pInMsg->clusterId;
  afMsg.srcAddr.endPoint = pInMsg->srcAddr.endpoint;
  afMsg.srcAddr.panId = pInMsg->srcAddr.panID;
  afMsg.srcAddr.addrMode = (afAddrMode_t)pInMsg->srcAddr.addrMode;
  if( (afMsg.srcAddr.addrMode == afAddr16Bit)
     || (afMsg.srcAddr.addrMode == afAddrGroup)
       || (afMsg.srcAddr.addrMode == afAddrBroadcast) )
  {
    afMsg.srcAddr.addr.shortAddr = pInMsg->srcAddr.addr.shortAddr;
  }
  else if(afMsg.srcAddr.addrMode == afAddr64Bit)
  {
    memcpy(afMsg.srcAddr.addr.extAddr, &(pInMsg->srcAddr.addr.extAddr),
           EXTADDR_LEN);
  }
  afMsg.macDestAddr = pInMsg->macDestAddr;
  afMsg.endPoint = pInMsg->endpoint;
  afMsg.wasBroadcast = pInMsg->wasBroadcast;
  afMsg.LinkQuality = pInMsg->linkQuality;
  afMsg.correlation = pInMsg->correlation;
  afMsg.rssi = pInMsg->rssi;
  afMsg.SecurityUse = pInMsg->securityUse;
  afMsg.timestamp = pInMsg->timestamp;
  afMsg.nwkSeqNum = pInMsg->nwkSeqNum;
  afMsg.macSrcAddr = pInMsg->macSrcAddr;
  afMsg.radius = pInMsg->radius;
  afMsg.cmd.TransSeqNumber = pInMsg->transSeqNum;
  afMsg.cmd.DataLength = pInMsg->n_payload;
  afMsg.cmd.Data = pInMsg->pPayload;
  
  zcl_ProcessMessageMSG(&afMsg);
}

/*******************************************************************************
* @fn      Switch_sendToggle
*
* @brief   Send an ON/OFF toggle command
*
* @param   none
*
* @return  none

static void Switch_sendToggle(void)
{
afAddrType_t dstAddr;

dstAddr.addrMode = (afAddrMode_t)zswDstAddr.addrMode;
dstAddr.addr.shortAddr = zswDstAddr.addr.shortAddr;
dstAddr.endPoint = zswDstAddr.endpoint;
dstAddr.panId = zswDstAddr.panID;

// Send a toggle
zclGeneral_SendOnOff_CmdToggle(SWITCH_EP, &dstAddr, false, 0);

LCD_WRITE_STRING( (char *)sCmdSent, LCD_PAGE2 );
}
*/
/*******************************************************************************
* @fn      Switch_handleKeys
*
* @brief   Callback service for keys
*
* @param   keys  - keys that were pressed
*
* @return  void
*/
static void Switch_handleKeys(uint8_t keys)
{
  if(keys == KEY_UP)
  {
    // Send the Toggle command through ZCL
    //Switch_sendToggle();
    
    //add by zpf to identify gateway
    //afAddrType_t dstAddr;
    
    //dstAddr.addrMode = (afAddrMode_t)afAddr16Bit;
    //dstAddr.addr.shortAddr = 0;
    //dstAddr.endPoint = 1;
    //dstAddr.panId = zswDstAddr.panID;
    
    // Send identify
    //zclGeneral_SendIdentifyQuery(SWITCH_EP, &dstAddr, false, 0);

    zstack_zdoEndDeviceBindReq_t pReq;
      
    pReq.dstAddr = 0x0000;
    pReq.bindingTarget = pNwkInfo->nwkAddr;//NLME_GetShortAddr();
    pReq.endpoint = SWITCH_EP;
    pReq.profileID = ZCL_HA_PROFILE_ID;
    pReq.n_inputClusters = sizeof(bindingOutClusters)/sizeof(bindingOutClusters[0]);//0;
    pReq.pInputClusters = bindingOutClusters;//NULL;
    pReq.n_outputClusters = sizeof(bindingOutClusters)/sizeof(bindingOutClusters[0]);
    pReq.pOutputClusters = bindingOutClusters;
      
    // Send end device bind request
    Zstackapi_ZdoEndDeviceBindReq(zswEntity, &pReq);
	
  }
  
  if(keys == KEY_DOWN)
  {
    if(zswbindflag == 1)
    {
      afAddrType_t dstAddr;
      
      dstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
      dstAddr.addr.shortAddr = 0xfffe;
      dstAddr.endPoint = SWITCH_EP;
      //dstAddr.panId = zswDstAddr.panID;
      
      // Send a toggle
      zclGeneral_SendOnOff_CmdToggle(SWITCH_EP, &dstAddr, false, 0);
    }
  }
  
  // toggle permit join
  if(keys == KEY_LEFT)
  {
    giSwScreenMode = SWITCH_MAINMODE;   // remove help screen if there
    
    if( pNwkInfo
       && ( (savedState == zstack_DevState_DEV_ZB_COORD)
           || (savedState == zstack_DevState_DEV_ROUTER) ) )
    {
      zstack_zdoMgmtPermitJoinReq_t req;
      
      // toggle permit join
      gPermitDuration = gPermitDuration ? 0 : 0xff;
      
      req.nwkAddr = pNwkInfo->nwkAddr;
      req.duration = gPermitDuration;
      req.tcSignificance = true;
      
      Zstackapi_ZdoMgmtPermitJoinReq(zswEntity, &req);
    }
  }
  
  if(keys == KEY_SELECT)
  {
    // Switch between Help and Main screens
    if(giSwScreenMode == SWITCH_MAINMODE)
    {
      // Switch from main screen to help screen
      giSwScreenMode = SWITCH_HELPMODE;
    }
    else
    {
      // Switch from help screen to main screen
      giSwScreenMode = SWITCH_MAINMODE;
      LCD_WRITE_STRING( (char *)sClearLine,  LCD_PAGE2 );
    }
    
    //add by zpf to fulfill reset to factory function
    zstack_zdoMgmtLeaveReq_t pReq;
    
    pReq.nwkAddr = pNwkInfo->nwkAddr;
    //*(pReq.deviceAddress) = NULL;
    memcpy( pReq.deviceAddress, pNwkInfo->ieeeAddr, Z_EXTADDR_LEN );
    pReq.options.rejoin = false;
    pReq.options.removeChildren = false;
    
    Zstackapi_ZdoMgmtLeaveReq(zswEntity, &pReq);
  }
  
  if(keys == KEY_RIGHT)
  {
#if defined (ZCL_EZMODE)
    // Start EZMode Commissioning
    {
      zclEZMode_InvokeData_t ezModeData;
      // only bind on the on/off cluster
      static uint16_t clusterIDs[] =
      {   ZCL_CLUSTER_ID_GEN_ON_OFF};
      
      // Invoke EZ-Mode
      ezModeData.endpoint = SWITCH_EP; // endpoint on which to invoke
      // EZ-Mode
      if( (savedState == zstack_DevState_DEV_ZB_COORD)
         || (savedState == zstack_DevState_DEV_ROUTER)
           || (savedState == zstack_DevState_DEV_END_DEVICE) )
      {
        ezModeData.onNetwork = true;   // node is already on the
        // network
      }
      else
      {
        ezModeData.onNetwork = false;  // node is not yet on the
        // network
      }
      ezModeData.initiator = true;        // OnOffSwitch is an initiator
      ezModeData.numActiveOutClusters = 1; // active output cluster
      ezModeData.pActiveOutClusterIDs = clusterIDs;
      ezModeData.numActiveInClusters = 0; // no active input clusters
      ezModeData.pActiveInClusterIDs = NULL;
      zcl_InvokeEZMode(&ezModeData);
      
      LCD_WRITE_STRING("EZMode", LCD_PAGE2);
    }
#elif defined (ZSTACK_MANUAL_START)
    Zstart_discovery();
#endif // ZCL_EZMODE
  }
  
  // update the display
  Switch_updateLcdDisplay();
}

/*********************************************************************
* @fn      Switch_updateLcdDisplay
*
* @brief   Called to update the LCD display.
*
* @param   none
*
* @return  none
*/
static void Switch_updateLcdDisplay(void)
{
  // Update the LCD depending on Screen Mode
  if(giSwScreenMode == SWITCH_HELPMODE)
  {
    Switch_updateLcdHelpScreen();
  }
  else
  {
    Switch_updateLcdMainScreen();
  }
}

/*********************************************************************
* @fn      Switch_updateLcdLine1Status
*
* @brief   Display LCD line 1 with network status
*          only call after on network
*          ZC: PPPP CH ADDR
*          ZR: PPPP CH ADDR
*          ZE: PPPP CH ADDR
*
* @param   none
*
* @return  none
*/
static void Switch_updateLcdLine1Status(void)
{
  int idx;
  char szLine[SWITCH_MAX_LCD_LINE];
  
  // Determine the device type
  if(savedState == zstack_DevState_DEV_ZB_COORD)
  {
    idx = 0;
  }
  else if(savedState == zstack_DevState_DEV_ROUTER)
  {
    idx = 1;
  }
  else if(savedState == zstack_DevState_DEV_END_DEVICE)
  {
    idx = 2;
  }
  else
  {
    idx = 3;
  }
  
  // If we've collected the device info from the ZStack Thread
  // display the device information on the screen
  //  ZE: PANx CH ADDR
  if(pNwkInfo)
  {
    memcpy(szLine, devInfoStrs[idx], 4);
    Util_uint16toa(pNwkInfo->panId, &szLine[4]);
    szLine[8] = ' ';
    Util_ltoa(pNwkInfo->logicalChannel, (void *)(&szLine[9]), 10);
    szLine[11] = ' ';
    Util_uint16toa(pNwkInfo->nwkAddr, &szLine[12]);
    LCD_WRITE_STRING(szLine, LCD_PAGE1);
  }
}

/*********************************************************************
* @fn      Switch_updateLcdMainScreen
*
* @brief   Called to display the main screen on the LCD.
*
* @param   none
*
* @return  none
*/
static void Switch_updateLcdMainScreen(void)
{
  Switch_updateLcdLine1Status();
  
  if( (savedState == zstack_DevState_DEV_ZB_COORD)
     || (savedState == zstack_DevState_DEV_ROUTER) )
  {
    // display help key with permit join status
    if(gPermitDuration)
    {
      // The '*' means that permit join is enabled
      LCD_WRITE_STRING("Select: Help   *", LCD_PAGE3);
    }
    else
    {
      LCD_WRITE_STRING("Select: Help    ", LCD_PAGE3);
    }
  }
  else
  {
    // display help key
    LCD_WRITE_STRING( (char *)sSwHelp, LCD_PAGE3 );
  }
}

/*********************************************************************
* @fn      Switch_updateLcdHelpScreen
*
* @brief   Called to update the LCD display with the help screen.
*
* @param   none
*
* @return  none
*/
static void Switch_updateLcdHelpScreen(void)
{
  LCD_WRITE_STRING( (char *)sSwLight, LCD_PAGE1 );
#if defined (ZCL_EZMODE)
  LCD_WRITE_STRING( (char *)sSwEZMode, LCD_PAGE2 );
#elif  defined (ZSTACK_MANUAL_START)
  LCD_WRITE_STRING( (char *)sSwStart, LCD_PAGE2 );
#else
  LCD_WRITE_STRING( (char *)sClearLine, LCD_PAGE2 );
#endif
  LCD_WRITE_STRING( (char *)sSwMain, LCD_PAGE3 );
}

/*********************************************************************
* @fn      Switch_processKeyChangeCallback
*
* @brief   Key event handler function
*
* @param   keysPressed - keys that are pressed
*
* @return  none
*/
static void Switch_processKeyChangeCallback(uint8_t keysPressed)
{
  keys = keysPressed;
  
  events |= SWITCH_KEY_EVENT;
  
  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

#if defined (ZCL_EZMODE)
/*********************************************************************
* @fn      Switch_ezmodeCB
*
* @brief   The Application is informed of events. This can be used to
*          show on the UI what is going on during EZ-Mode
*          steering/finding/binding.
*
* @param   state - state EZMode is in
* @param   pData - pointer to data for a state
*
* @return  none
*/
static void Switch_ezmodeCB(zlcEZMode_State_t state, zclEZMode_CBData_t *pData)
{
#if defined (TI_DRIVERS_LCD_INCLUDED)
  char szLine[SWITCH_MAX_LCD_LINE];
  char *pStr;
  uint8_t err;
#endif // TI_DRIVERS_LCD_INCLUDED
  
  // time to go into identify mode
  if(state == EZMODE_STATE_IDENTIFYING)
  {
    zswIdentifyTime = SWITCH_CONVERT_TO_SECONDS(EZMODE_TIME);
    Switch_processIdentifyTimeChange();
  }
  
  // autoclosing, show what happened (success, cancelled, etc...)
  if(state == EZMODE_STATE_AUTOCLOSE)
  {
#if defined (TI_DRIVERS_LCD_INCLUDED)
    pStr = NULL;
    err = pData->sAutoClose.err;
    if(err == EZMODE_ERR_SUCCESS)
    {
      pStr = "EZMode: Success";
    }
    else if(err == EZMODE_ERR_NOMATCH)
    {
      pStr = "EZMode: NoMatch"; // not a match made in heaven
    }
    if(pStr)
    {
      if(giSwScreenMode == SWITCH_MAINMODE)
      {
        LCD_WRITE_STRING(pStr, LCD_PAGE2);
      }
    }
#endif // TI_DRIVERS_LCD_INCLUDED
  }
  
  // finished, either show DstAddr/EP, or nothing (depending on success or
  // not)
  if(state == EZMODE_STATE_FINISH)
  {
    // turn off identify mode
    zswIdentifyTime = 0;
    Switch_processIdentifyTimeChange();
    
#if defined (TI_DRIVERS_LCD_INCLUDED)
    // if successful, inform user which nwkaddr/ep we bound to
    pStr = NULL;
    err = pData->sFinish.err;
    if(err == EZMODE_ERR_SUCCESS)
    {
      // "EZDst:1234 EP:34"
      memcpy(szLine, "EZDst:", 6);
      Util_uint16toa(pData->sFinish.nwkaddr, &szLine[6]);
      memcpy(&szLine[10], " EP:", 4);
      // Util_ltoa NULL terminates
      Util_ltoa(pData->sFinish.ep, (void *)(&szLine[14]), 16);
      pStr = szLine;
    }
    else if(err == EZMODE_ERR_BAD_PARAMETER)
    {
      pStr = "EZMode: BadParm";
    }
    else if(err == EZMODE_ERR_CANCELLED)
    {
      pStr = "EZMode: Cancel";
    }
    else
    {
      pStr = "EZMode: TimeOut";
    }
    if(pStr)
    {
      if(giSwScreenMode == SWITCH_MAINMODE)
      {
        LCD_WRITE_STRING(pStr, LCD_PAGE2);
      }
    }
#endif // TI_DRIVERS_LCD_INCLUDED
    
    // show main UI screen 3 seconds after binding
    if(Util_isClockActive(&mainScreenClkStruct) == true)
    {
      Util_stopClock(&mainScreenClkStruct);
    }
    Clock_setTimeout( mainScreenClkHandle,
                     (SWITCH_MAINSCREEN_TIMEOUT * TIMER_MS_ADJUSTMENT) );
    Util_startClock(&mainScreenClkStruct);
  }
}
#endif  // ZCL_EZMODE

/*********************************************************************
* @fn      Switch_processIdentifyTimeChange
*
* @brief   Called to process any change to the IdentifyTime attribute.
*
* @param   none
*
* @return  none
*/
static void Switch_processIdentifyTimeChange(void)
{
  // Stop the Identify timer
  if(Util_isClockActive(&identifyClkStruct) == true)
  {
    Util_stopClock(&identifyClkStruct);
  }
  
  // Are we still identifying?
  if(zswIdentifyTime > 0)
  {
    // Continue with another timer
    Clock_setTimeout( identifyClkHandle,
                     ((zswIdentifyTime * SWITCH_1SEC_MSEC) * TIMER_MS_ADJUSTMENT));
    Util_startClock(&identifyClkStruct);
    
    Board_Led_control(board_led_type_LED4, board_led_state_ON);
  }
  else
  {
    // restore the LED status
    if(zswOnOff)
    {
      Board_Led_control(board_led_type_LED4, board_led_state_ON);
    }
    else
    {
      Board_Led_control(board_led_type_LED4, board_led_state_OFF);
    }
  }
}

/*********************************************************************
* @fn      Switch_processIdentifyQueryResponseCallback
*
* @brief   Callback from the ZCL General Cluster Library when
*          it received an Identity Query Response Command for this application.
*
* @param   pRsp - pointer to the incoming ZCL Response
*
* @return  none
*/
static void Switch_processIdentifyQueryResponseCallback(
                                                        zclIdentifyQueryRsp_t *pRsp)
{
#if defined (ZCL_EZMODE)
  {
    // ZCL EZMode will process this response message
    zclEZMode_ActionData_t data;
    data.pIdentifyQueryRsp = pRsp;
    zcl_EZModeAction(EZMODE_ACTION_IDENTIFY_QUERY_RSP, &data);
  }
#else
  (void)pRsp;
#endif
}

/*********************************************************************
* @fn      Switch_processIdentifyCallback
*
* @brief   Callback from the ZCL General Cluster Library when
*          it received an Identity Command for this application.
*
* @param   pCmd - pointer to Identify command
*
* @return  none
*/
static void Switch_processIdentifyCallback(zclIdentify_t *pCmd)
{
  // Save the incoming time and setup a timer
  zswIdentifyTime = pCmd->identifyTime;
  Switch_processIdentifyTimeChange();
}


static void Switch_processOnOffCB( uint8 cmd )
{
  // Turn on the light
  if ( cmd == COMMAND_ON )
  {
    zswOnOff = SWITCH_LIGHT_ON;
  }
  // Turn off the light
  else if ( cmd == COMMAND_OFF )
  {
    zswOnOff = SWITCH_LIGHT_OFF;
  }
  // Toggle the light
  else if ( cmd == COMMAND_TOGGLE )
  {
    if ( zswOnOff == SWITCH_LIGHT_OFF )
    {
      zswOnOff = SWITCH_LIGHT_ON;
    }
    else
    {
      zswOnOff = SWITCH_LIGHT_OFF;
    }
  }
  
  //add by zpf to indicate network status   board_led_state
  Board_Led_control(board_led_type_LED2,(board_led_state)zswOnOff);
  
}


#if defined (ZCL_EZMODE)
/*********************************************************************
* @fn      Switch_setEzmodeTimerCallback
*
* @param   start - true to start a timeer, false to stop a timer
* @param   event_id - timer ID
* @param   timeout_value - in milliseconds
*
* @return  none
*/
static void Switch_setEzmodeTimerCallback(bool start, uint16_t event_id,
                                          uint32_t timeout_value)
{
  if(event_id == SWITCH_EZMODE_TIMEOUT_EVT)
  {
    // Setup the EZMode Timeout timer
    if(Util_isClockActive(&ezmodeTimeoutClkStruct) == true)
    {
      Util_stopClock(&ezmodeTimeoutClkStruct);
    }
    
    if(start)
    {
      Clock_setTimeout( ezmodeTimeoutClkHandle,
                       (timeout_value * TIMER_MS_ADJUSTMENT) );
      Util_startClock(&ezmodeTimeoutClkStruct);
    }
  }
  else if(event_id == SWITCH_EZMODE_NEXTSTATE_EVT)
  {
    // Setup the EZMode Next State timer
    if(Util_isClockActive(&ezmodeStateClkStruct) == true)
    {
      Util_stopClock(&ezmodeStateClkStruct);
    }
    
    if(start)
    {
      Clock_setTimeout( ezmodeStateClkHandle,
                       (timeout_value * TIMER_MS_ADJUSTMENT) );
      Util_startClock(&ezmodeStateClkStruct);
    }
  }
}

/*******************************************************************************
* @fn      Switch_processEzmodeTimeoutCallback
*
* @brief   Timeout handler function
*
* @param   a0 - ignored
*
* @return  none
*/
static void Switch_processEzmodeTimeoutCallback(UArg a0)
{
  (void)a0; // Parameter is not used
  
  events |= SWITCH_EZMODE_TIMEOUT_EVT;
  
  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*******************************************************************************
* @fn      Switch_processEzmodeStateChangeCallback
*
* @brief   Timeout handler function
*
* @param   a0 - ignored
*
* @return  none
*/
static void Switch_processEzmodeStateChangeCallback(UArg a0)
{
  (void)a0; // Parameter is not used
  
  events |= SWITCH_EZMODE_NEXTSTATE_EVT;
  
  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}
#endif // ZCL_EZMODE

/*******************************************************************************
* @fn      Switch_processIdentifyTimeoutCallback
*
* @brief   Timeout handler function
*
* @param   a0 - ignored
*
* @return  none
*/
static void Switch_processIdentifyTimeoutCallback(UArg a0)
{
  (void)a0; // Parameter is not used
  
  events |= SWITCH_IDENTIFY_TIMEOUT_EVT;
  
  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*******************************************************************************
* @fn      Switch_processMainScreenTimeoutCallback
*
* @brief   Timeout handler function
*
* @param   a0 - ignored
*
* @return  none
*/
static void Switch_processMainScreenTimeoutCallback(UArg a0)
{
  (void)a0; // Parameter is not used
  
  events |= SWITCH_MAIN_SCREEN_EVT;
  
  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*******************************************************************************
* @fn      Switch_setPollRate
*
* @brief   Set the ZStack Thread Poll Rate
*
* @param   newPollRate - new poll rate in milliseconds
*
* @return  none
*/
static void Switch_setPollRate(uint32_t newPollRate)
{
  zstack_sysConfigWriteReq_t writeReq = {0};
  
  // Set the new poll rate
  writeReq.has_pollRate = true;
  writeReq.pollRate = newPollRate;
  
  (void)Zstackapi_sysConfigWriteReq(zswEntity, &writeReq);
}

#if defined (ZSTACK_MANUAL_START)
/**
* Function pointer to callback function to set a Clock Event in the app.
*
* @param   none
*
* @return  none
*/
static void Switch_processZstartClockCallback(void)
{
  events |= SWITCH_MANUAL_START_CLK_EVT;
  
  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}
#endif // ZSTACK_MANUAL_START

#if defined (ZSTACK_MANUAL_START)
/**
* Function to initialize the discovery process.
*
* @param   none
*
* @return  none
*/
static void Switch_initializeZstartDiscovery(void)
{
  zstack_LogicalTypes deviceType = zstack_LogicalTypes_UNKNOWN;
  
  // Get the Device information from the ZStack Thread
  zstack_sysNwkInfoReadRsp_t *pInfo = zclport_getDeviceInfo(zswEntity);
  
  // Determine the device type
  if(pInfo->devTypes.router)
  {
    deviceType = zstack_LogicalTypes_ROUTER;
  }
  else if(pInfo->devTypes.enddevice)
  {
    deviceType = zstack_LogicalTypes_ENDDEVICE;
  }
  
  // Tell ZStart to start network discovery
  Zstart_Initialize(zswEntity, Switch_processZstartClockCallback, deviceType,
                    pfnZswNV);
}
#endif // ZSTACK_MANUAL_START



/*Restore the bindflag from NV items */
static bool Switch_RestoreBindflagFromNV(ICall_EntityID entity)
{
  zstack_sysConfigReadReq_t readReq = {0};
  zstack_sysConfigReadRsp_t readRsp = {0};
  
  // Ask if the device is already part of a network
  readReq.enddevicebindflag = true;
  
  (void)Zstackapi_sysConfigReadReq(entity, &readReq, &readRsp);
  
  return(readRsp.enddevicebindflag);
}



/*******************************************************************************
******************************************************************************/
