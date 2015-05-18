/******************************************************************************
 Filename:       zcl_port.h
 Revised:        $Date: 2015-02-12 12:55:11 -0800 (Thu, 12 Feb 2015) $
 Revision:       $Revision: 42532 $

 Description:    This file contains the the protypes and definitions
                 needed for the ZCL port.

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

#ifndef ZCL_PORT_H
#define ZCL_PORT_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 \defgroup ZclPort ZCL Porting Layer
 <BR>
 This module is a collection of functions used to make the ZStack ZCL module
 portable between OSAL, Linux and TI-RTOS.
 <BR>
 The ZCL parsing/building functions, along with profile code, will call these
 functions and this module is responsible for performing the translation
 or function requested.
 <BR>
 The following APIs are functions that the Application needs to call to pass
 information that this porting layer needs.  The other functions in this module
 are called by ZCL, not by the application.
 <BR>
 */

/**
 * \ingroup ZclPort
 * @{
 */

#if defined (ZCL_EZMODE)
/**
 * @brief   Function pointer definition to callback EZMode Timer function.
 *
 * @param   start - true to start a timer, false to stop a timer
 * @param   event_id - timer ID
 * @param   timeout_value - in milliseconds
 */
typedef void (*pfnEZModeTimerCB)(bool start, uint16_t event_id,
                                 uint32_t timeout_value);
#endif

/**
 * @brief   Function pointer definition to callback EZMode Timer function.
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  TRUE if you've handle it, FALSE if not
 */
typedef uint8_t (*zclport_pFnZclHandleExternal)(zclIncoming_t *pInMsg);

/** @} end group ZclPort */

/**
 * \ingroup ZclPort
 * @{
 */

/** Default ZCL Scene Table NV Item ID */
#define ZCL_PORT_SCENE_TABLE_NV_ID 0x0001

/**
 * @brief   Register an AF Endpoint.  This is needed by the ZCL code
 *          to find an AF endpoint descriptor.
 *
 * @param   entity - The task entity ID.
 * @param   *pEpDesc - Pointer to an endpoint Descriptor.
 *
 * @return  true if added to table, false if table full
 */
extern bool zclport_registerEndpoint(ICall_EntityID  entity,
                                     endPointDesc_t *pEpDesc);

/**
 * @brief   Call to register the function pointers for the NV driver.
 *
 * @param   pfnNV - pointer to the NV driver functions
 * @param   sceneNVID - Scene Table NV ID
 */
extern void zclport_registerNV(NVINTF_nvFuncts_t *pfnNV, uint16_t sceneNVID);

/**
 * @brief   Call to register a function pointer to handle
 *          zcl_HandleExternal() messages.
 *
 * @param   pfn - pointer to the function that handles ZCL External messages.
 */
extern void zclport_registerZclHandleExternal(zclport_pFnZclHandleExternal pfn);

/**
 * @brief   Call to get Device Information.
 *
 * @param  entity - The task entity ID.
 *
 * @return  pointer to structure containing info
 */
extern zstack_sysNwkInfoReadRsp_t *zclport_getDeviceInfo(ICall_EntityID entity);

/**
 * @brief   Determines if the device is already part of a network by
 *          asking the stack thread.
 *          <BR>
 *          "Part of the Network" means that the device's network state is
 *           that of an end device or router/coordinator in a network, and
 *           not initialized or in an intermediate state (like scannning,
 *           joining or rejoining).  A good use of this function would be
 *           to call this function before starting the device, to
 *           "pre"-determine if the device needs to join a network or just
 *           start from the parameters already in NV.
 *
 * @param  entity - The task entity ID.
 *
 * @return  true if already part of a network, false if not
 */
extern bool zclport_isAlreadyPartOfNetwork(ICall_EntityID entity);

#if defined (ZCL_EZMODE)
/**
 * @brief   Call to register a function for the EZMode commissioning
 *          timeout callback function
 *
 * @param   pFn - pointer to function to register
 */
extern void zclport_registerEZModeTimerCB(pfnEZModeTimerCB pFn);
#endif

/**
 * @brief   If the NV item does not already exist, it is created and
 *          initialized with the data passed to the function, if any.
 *
 * @param   id - NV ID
 * @param   subId - NV ID
 * @param   len - length of NV item
 * @param   buf - pointer to NV item
 *
 * @return  status
 */
extern uint8_t zclport_initializeNVItem(uint16_t id, uint16_t subId,
                                        uint16_t len,
                                        void *buf);

/**
 * @brief   Write a data item to NV. Function can write an entire item to NV or
 *          an element of an item by indexing into the item with an offset.
 *
 * @param   id - NV ID
 * @param   subId - NV ID
 * @param   ndx - offset into NV item
 * @param   len - length of NV item to write
 * @param   buf - pointer to NV item
 *
 * @return  status
 */
extern uint8_t zclport_writeNV(uint16_t id, uint16_t subId, uint16_t ndx,
                               uint16_t len,
                               void *buf);

/**
 * @brief   Read data from NV. This function can be used to read an entire item
 *          from NV or an element of an item by indexing into the item with an
 *          offset. Read data is copied into buf.
 *
 * @param   id - NV ID
 * @param   subId - NV ID
 * @param   ndx - offset into NV item
 * @param   len - length of NV item to read
 * @param   buf - pointer to NV item
 *
 * @return  status
 */
extern uint8_t zclport_readNV(uint16_t id, uint16_t subId, uint16_t ndx,
                              uint16_t len,
                              void *buf);

/** @} end group ZclPort */

/*********************************************************************
 *********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_PORT_H */
