/**
  @file  CryptoBoard.c
  @brief TI-RTOS and *Ware implementation of Crypto board service

  <!--
  Copyright 2013-2015 Texas Instruments Incorporated. All rights reserved.

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
#include <xdc/std.h>
#include "CryptoBoard.h"

#include <hw_types.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <ti/drivers/crypto/CryptoCC26XX.h>
#include "ICall.h"

/** @internal service dispatcher entity ID */
static ICall_EntityID Crypto_entity;
CryptoCC26XX_Handle CryptoCC26XXHandle;

/* service function.
 * See type description of ICall_ServiceFunc for
 * descriptions of arguments. */
static ICall_Errno Crypto_func(ICall_FuncArgsHdr *args)
{
  if(args->func == CRYPTOBOARD_AES_LOAD_KEY)
  {
    CryptoBoard_AesArgs *cryptoArgs = (CryptoBoard_AesArgs *) args;
    return CryptoCC26XX_allocateKey( CryptoCC26XXHandle,
                                     (CryptoCC26XX_KeyLocation)cryptoArgs->keyLocation,
                                     (const uint32_t *)cryptoArgs->keySrc );
  }
  else if(args->func == CRYPTOBOARD_AES_RELEASE_KEY)
  {
    CryptoBoard_AesArgs *cryptoArgs = (CryptoBoard_AesArgs *) args;
    int keyIndex = (int)cryptoArgs->keyLocation;
    return CryptoCC26XX_releaseKey( CryptoCC26XXHandle, &keyIndex );
  }
  else if(args->func == CRYPTOBOARD_AES_ECB)
  {
    CryptoBoard_AesArgs *cryptoArgs = (CryptoBoard_AesArgs *) args;
    CryptoCC26XX_AESECB_Transaction transaction;

    transaction.opType = CRYPTOCC26XX_OP_AES_ECB;
    transaction.keyIndex = cryptoArgs->keyLocation;
    transaction.msgIn = (uint32_t *)cryptoArgs->msgIn;
    transaction.msgOut = (uint32_t *)cryptoArgs->msgOut;
    return CryptoCC26XX_transact( CryptoCC26XXHandle,
                                  (CryptoCC26XX_Transaction *)&transaction );
  }
  else if(args->func == CRYPTOBOARD_AES_CCM)
  {
    CryptoBoard_AesCcmArgs *cryptoArgs = (CryptoBoard_AesCcmArgs *) args;
    CryptoCC26XX_AESCCM_Transaction transaction;

    transaction.opType = CRYPTOCC26XX_OP_AES_CCM;
    transaction.keyIndex = cryptoArgs->AesCcmTransact.keyLocation;
    transaction.authLength = cryptoArgs->AesCcmTransact.authLength;
    transaction.nonce = (char *)cryptoArgs->AesCcmTransact.nonce;
    transaction.msgIn = (char *)cryptoArgs->AesCcmTransact.plainText;
    transaction.header = (char *)cryptoArgs->AesCcmTransact.header;
    transaction.fieldLength = cryptoArgs->AesCcmTransact.fieldLength;
    transaction.msgInLength = cryptoArgs->AesCcmTransact.plainTextLength;
    transaction.headerLength = cryptoArgs->AesCcmTransact.headerLength;
    transaction.msgOut = (uint32_t *)cryptoArgs->msgOut;
    return CryptoCC26XX_transact( CryptoCC26XXHandle,
                                  (CryptoCC26XX_Transaction *)&transaction );
  }
  else if(args->func == CRYPTOBOARD_AES_CCM_INV)
  {
    CryptoBoard_AesCcmArgs *cryptoArgs = (CryptoBoard_AesCcmArgs *) args;
    CryptoCC26XX_AESCCM_Transaction transaction;

    transaction.opType = CRYPTOCC26XX_OP_AES_CCMINV;
    transaction.keyIndex = cryptoArgs->AesCcmTransact.keyLocation;
    transaction.authLength = cryptoArgs->AesCcmTransact.authLength;
    transaction.nonce = (char *)cryptoArgs->AesCcmTransact.nonce;
    transaction.msgIn = (char *)cryptoArgs->AesCcmTransact.plainText;
    transaction.header = (char *)cryptoArgs->AesCcmTransact.header;
    transaction.fieldLength = cryptoArgs->AesCcmTransact.fieldLength;
    transaction.msgInLength = cryptoArgs->AesCcmTransact.plainTextLength;
    transaction.headerLength = cryptoArgs->AesCcmTransact.headerLength;
    transaction.msgOut = (uint32_t *)cryptoArgs->msgOut;
    return CryptoCC26XX_transact( CryptoCC26XXHandle,
                                  (CryptoCC26XX_Transaction *)&transaction );
  }
  else
  {
    /* Unknown function ID */
    return ICALL_ERRNO_INVALID_FUNCTION;
  }
}


/* See header file to find the function description. */
void Crypto_init(void)
{
  ICall_Semaphore sem;

  /* Enroll the service to the dispatcher */
  if (ICall_enrollService(ICALL_SERVICE_CLASS_CRYPTO,
                          Crypto_func, &Crypto_entity, &sem) !=
                          ICALL_ERRNO_SUCCESS)
  {
    ICall_abort();
  }
}
