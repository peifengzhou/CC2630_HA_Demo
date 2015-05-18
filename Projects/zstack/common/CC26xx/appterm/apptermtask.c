/**
  @file  apptermtask.c
  @brief Application Terminal OSAL implementation

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
#include <ICall.h>
#include "OSAL.h"
#include "ZComDef.h"

#include "hal_led.h"
#include "hal_lcd.h"
#include "hal_key.h"

#include "OnBoard.h"

#include "apptermintf.h"


/* ------------------------------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 * Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 * Global Variables
 * ------------------------------------------------------------------------------------------------
 */
/* Save the AppTermTaskId */
uint8 appTermTaskId;


/* ------------------------------------------------------------------------------------------------
 * Local Variables
 * ------------------------------------------------------------------------------------------------
 */
static ICall_EntityID ZStackEntity;
static ICall_Semaphore ZStackSem;

static ICall_EntityID srcEntityID = 0xFFFE;


/* ------------------------------------------------------------------------------------------------
 * External Function Prototypes
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 * Local Function Prototypes
 * ------------------------------------------------------------------------------------------------
 */
static bool appMsg( ICall_EntityID srcEntityID, void *pMsg );
static void sendLCDReq( uint16 dstID, uint8 lcdStrLen,
                       void *pLcdStr, uint8 lineNumber  );
static void sendLEDReq( uint16 dstID, APPTERM_ledType ledType,
                       APPTERM_ledState ledState  );

/**************************************************************************************************
 * @fn          appTermTaskInit
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
 **************************************************************************************************
 */
void appTermTaskInit( uint8 taskId )
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

  appTermTaskId = taskId;
}

/**************************************************************************************************
 * @fn          appTermTaskProcessEvent
 *
 * @brief       This function is the main event handling function of the this Thread executing
 *              in task context.  This function is called by OSAL when an event or message
 *              is pending for the this Thread.
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
uint16 appTermTaskProcessEvent( uint8 taskId, uint16 events )
{
  AppTerm_InitReq_t *pMsg;

  (void)taskId;  // Intentionally unreferenced parameter

  // Process system messages
  if ( events & SYS_EVENT_MSG )
  {
    if ( (pMsg = (AppTerm_InitReq_t *)osal_msg_receive( appTermTaskId )) != NULL )
    {
      bool send = false;

      switch ( pMsg->hdr.event )
      {
        case ZDO_CB_MSG:
        case AF_DATA_CONFIRM_CMD:
        case AF_REFLECT_ERROR_CMD:
        case AF_INCOMING_MSG_CMD:
        case ZDO_STATE_CHANGE:
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

    // Return unproccessed events
    return ( events ^ SYS_EVENT_MSG );
  }

  // When reaching here, the events are unknown
  // Discard them or make more handlers
  return 0;
}

uint8 HalLedSet( uint8 led, uint8 mode )
{
  if ( srcEntityID != 0xFFFE )
  {
    APPTERM_ledType ledType = APPTERM_ledType_LED1;
    APPTERM_ledState ledState = APPTERM_led_state_OFF;

    if ( led == HAL_LED_2 )
    {
      ledType = APPTERM_ledType_LED2;
    }
    else if ( led == HAL_LED_3 )
    {
      ledType = APPTERM_ledType_LED3;
    }
    else if ( led == HAL_LED_4 )
    {
      ledType = APPTERM_ledType_LED4;
    }

    if ( mode == HAL_LED_MODE_ON )
    {
      ledState = APPTERM_led_state_ON;
    }
    else if ( (mode == HAL_LED_MODE_BLINK) || (mode == HAL_LED_MODE_FLASH) )
    {
      ledState = APPTERM_led_state_BLINK;
    }
    else if ( mode == HAL_LED_MODE_TOGGLE )
    {
      ledState = APPTERM_led_state_TOGGLE;
    }

    sendLEDReq( srcEntityID, ledType, ledState );
  }
  return ( 0 );
}

void HalLedBlink( uint8 leds, uint8 cnt, uint8 duty, uint16 time )
{
  HalLedSet( leds, HAL_LED_MODE_FLASH );
}

/*
 * Write a string to the LCD
 */
void HalLcdWriteString ( char *str, uint8 option )
{
  if ( srcEntityID != 0xFFFE )
  {
    uint8 lineNumber;
    uint8 lcdStrLen = osal_strlen( str );

    if ( option == HAL_LCD_LINE_1 )
    {
      lineNumber = 1;
    }
    else if ( option == HAL_LCD_LINE_2 )
    {
      lineNumber = 2;
    }
    else if ( option == HAL_LCD_LINE_3 )
    {
      lineNumber = 3;
    }
    else if ( option == HAL_LCD_LINE_4 )
    {
      lineNumber = 4;
    }

    sendLCDReq( srcEntityID, lcdStrLen, str, lineNumber );
  }
}

void HalLcdWriteScreen( char *line1, char *line2 )
{
  HalLcdWriteString( line1, HAL_LCD_LINE_1 );
  HalLcdWriteString( line2, HAL_LCD_LINE_2 );
}


#define MAXVALUESTRINGLEN 16
/*
 * Write a string followed by a value to the LCD
 */
void HalLcdWriteStringValue( char *title, uint16 value, uint8 format, uint8 line )
{
  if ( srcEntityID != 0xFFFE )
  {
    uint32 err;
    uint16 titleLen = osal_strlen( title );
    uint8 *pStr = osal_mem_alloc( titleLen + MAXVALUESTRINGLEN );
    if ( pStr )
    {
      osal_memset( pStr, 0, titleLen + MAXVALUESTRINGLEN );

      osal_memcpy( pStr, title, titleLen );

      pStr[titleLen] = ' ';

      err = (uint32)(value);
      _ltoa( err, &pStr[titleLen+1], format );

      HalLcdWriteString( (char*)pStr, line );
      osal_mem_free( pStr );
    }
  }
}

#define HAL_LCD_MAX_BUFF 25

void HalLcdWriteStringValueValue( char *title, uint16 value1, uint8 format1,
                                 uint16 value2, uint8 format2, uint8 line )
{
  if ( srcEntityID != 0xFFFE )
  {
    uint8 tmpLen;
    uint8 buf[HAL_LCD_MAX_BUFF];
    uint32 err;

    tmpLen = (uint8)osal_strlen( (char*)title );
    if ( tmpLen )
    {
      osal_memcpy( buf, title, tmpLen );
      buf[tmpLen++] = ' ';
    }

    err = (uint32)(value1);
    _ltoa( err, &buf[tmpLen], format1 );
    tmpLen = (uint8)osal_strlen( (char*)buf );

    buf[tmpLen++] = ',';
    buf[tmpLen++] = ' ';
    err = (uint32)(value2);
    _ltoa( err, &buf[tmpLen], format2 );

    HalLcdWriteString( (char *)buf, line );
  }
}


/*
 * Write a value to the LCD
 */
void HalLcdWriteValue ( uint32 value, const uint8 radix, uint8 option )
{
  if ( srcEntityID != 0xFFFE )
  {
    uint8 *pStr = osal_mem_alloc( MAXVALUESTRINGLEN );
    if ( pStr )
    {
      osal_memset( pStr, 0, MAXVALUESTRINGLEN );

      _ltoa( value, pStr, radix );

      HalLcdWriteString( (char*)pStr, option );

      osal_mem_free( pStr );
    }
  }
}

void BuzzerControl( uint8 on )
{
  // Put code here to turn a buzzer on/off
  (void)on;
}

/*********************************************************************
 * @fn          appMsg
 *
 * @brief       Process an incoming Application task message.
 *
 * @param       srcEntityID - source thread ID
 * @param       pMsg - pointer to the incoming message
 *
 * @return      true to send the message back to the sender, false if not
 */
static bool appMsg( ICall_EntityID srcEntityID, void *pMsg )
{
  bool resend = false; // default to NOT resend to app task

  // Temp convert to get the event
  AppTerm_InitReq_t *pReq = (AppTerm_InitReq_t *)pMsg;

  switch ( pReq->hdr.event )
  {
    case APPTERM_CmdIDs_INIT_REQ:
      break;

    case APPTERM_CmdIDs_KEYPRESS:
      {
        AppTerm_Keypress_t *pInd = (AppTerm_Keypress_t *)pMsg;
        uint8 keys = 0;

        if ( pInd->keypress & APPTERM_KEY_SELECT )
        {
          keys |= HAL_KEY_SW_5;
        }

        if ( pInd->keypress & APPTERM_KEY_UP )
        {
          keys |= HAL_KEY_SW_1;
        }

        if ( pInd->keypress & APPTERM_KEY_RIGHT )
        {
          keys |= HAL_KEY_SW_2;
        }

        if ( pInd->keypress & APPTERM_KEY_DOWN )
        {
          keys |= HAL_KEY_SW_3;
        }

        if ( pInd->keypress & APPTERM_KEY_LEFT )
        {
          keys |= HAL_KEY_SW_4;
        }

        OnBoard_SendKeys( keys, 0 );
      }
      break;

    default:
      break;
  }

  return ( resend );
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn          sendLCDReq
 *
 * @brief       Function to send Leave Indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       pStr - pointer to join confirm information
 *
 * @return      none
 */
static void sendLCDReq( uint16 dstID, uint8 lcdStrLen,
                       void *pLcdStr, uint8 lineNumber  )
{
  AppTerm_LcdReq_t *pLCDReq;

  pLCDReq = (AppTerm_LcdReq_t *)ICall_allocMsg(
                        sizeof ( AppTerm_LcdReq_t ) );
  if ( pLCDReq == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pLCDReq, 0, sizeof ( AppTerm_LcdReq_t ) );

  pLCDReq->hdr.event = APPTERM_CmdIDs_LCD_REQ;
  pLCDReq->hdr.status = 0;

  pLCDReq->lcdLen = lcdStrLen;
  pLCDReq->pLcdString = ICall_malloc( lcdStrLen+1 );
  if ( pLCDReq->pLcdString )
  {
    osal_memcpy( pLCDReq->pLcdString, pLcdStr, lcdStrLen );
    pLCDReq->pLcdString[lcdStrLen] = 0;
  }
  else
  {
    pLCDReq->lcdLen = 0;
  }

  pLCDReq->lineNumber = lineNumber;

  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID,
                   ICALL_MSG_FORMAT_KEEP, (void *)pLCDReq );
}

/*********************************************************************
 * @fn          sendLCDReq
 *
 * @brief       Function to send Leave Indication
 *
 * @param       dstID - Destination iCall entity ID
 * @param       pStr - pointer to join confirm information
 *
 * @return      none
 */
static void sendLEDReq( uint16 dstID, APPTERM_ledType ledType,
                       APPTERM_ledState ledState  )
{
  AppTerm_LedReq_t *pLEDReq;

  pLEDReq = (AppTerm_LedReq_t *)ICall_allocMsg(
                        sizeof ( AppTerm_LedReq_t ) );
  if ( pLEDReq == NULL )
  {
    // Ignore the message
    return;
  }

  osal_memset( pLEDReq, 0, sizeof ( AppTerm_LedReq_t ) );

  pLEDReq->hdr.event = APPTERM_CmdIDs_LED_REQ;
  pLEDReq->hdr.status = 0;

  pLEDReq->ledType = ledType;
  pLEDReq->ledState = ledState;
  // Send to a subscriber
  (void)ICall_send( ZStackEntity, dstID,
                   ICALL_MSG_FORMAT_KEEP, (void *)pLEDReq );
}
