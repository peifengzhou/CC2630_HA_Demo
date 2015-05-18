/**************************************************************************************************
  Filename:       hal_aes.h
  Revised:        $Date: 2014-11-04 14:27:46 -0800 (Tue, 04 Nov 2014) $
  Revision:       $Revision: 40983 $

  Description:    Support for HW/SW AES encryption.


  Copyright 2007-2014 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
**************************************************************************************************/

#ifndef     HAL_AES_H_
#define     HAL_AES_H_

#include "ZComDef.h"
#include <inc/hw_types.h>
#include "hal_board.h"
#include <driverlib/crypto.h>

/* AES and Keystore functions */
extern void HalAesInit( void );
extern void ssp_HW_KeyInit (uint8 *);
extern void sspAesDecryptHW (uint8 *, uint8 *);
extern void sspAesEncryptHW_keylocation( uint8 *, uint8 *, uint8 );
extern void sspAesDecryptHW_keylocation( uint8 *, uint8 *, uint8 );
extern void (*pSspAesEncrypt)( uint8 *, uint8 * );
extern void sspAesEncryptHW (uint8 *, uint8 *);

#define     STATE_BLENGTH   16      // Number of bytes in State
#define     KEY_BLENGTH     16      // Number of bytes in Key
#define     KEY_EXP_LENGTH  176     // Nb * (Nr+1) * 4

/* AES Engine is default to hardware AES. To turn on software AES, #define one of the followings:
 * #define SOFTWARE_AES TRUE, uses software aes  ( slowest setting )
 * #define SW_AES_AND_KEY_EXP TRUE, enables software aes with key expansion ( improves speed at the cost of 176 bytes of data (RAM) )
 */
#if ((defined SOFTWARE_AES) && (SOFTWARE_AES == TRUE)) && ((defined SW_AES_AND_KEY_EXP) && (SW_AES_AND_KEY_EXP == TRUE))
#error "SOFTWARE_AES and SW_AES_AND_KEY_EXP cannot be both defined."
#endif

extern void HalAesInit( void );
extern uint8 AesLoadKey( uint8 * );

extern void (*pSspAesEncrypt)( uint8 *, uint8 * );
extern void ssp_HW_KeyInit (uint8 *);


extern void sspAesEncryptHW (uint8 *, uint8 *);

#define AES_BUSY    0x08
#define ENCRYPT     0x00
#define DECRYPT     0x01

// _mode_ is one of
#define CBC         0x00000020
#define CTR         0x00000040
#define ECB         0x1FFFFFE0
#define CBC_MAC     0x00008000
#define CCM         0x00040000

// Macro for setting the mode of the AES operation
#define AES_SETMODE_ECB do { HWREG(AES_AES_CTRL) &= ~ECB; } while (0)
#define AES_SETMODE(mode) do { HWREG(AES_AES_CTRL) &= ~mode; HW_REG(AES_AES_CTRL) |= mode} while (0)



#endif  // HAL_AES_H_
