/******************************************************************************
  Filename:       hal_ccm.c
  Revised:        $Date: 2014-11-04 14:27:46 -0800 (Tue, 04 Nov 2014) $
  Revision:       $Revision: 40983 $

  Description:    Support for Hardware CCM authentication.

  Copyright 2011-2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
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
******************************************************************************/

/******************************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "hal_aes.h"
#include "hal_ccm.h"
#include "hal_assert.h"
#include <driverlib/crypto.h>


/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS
 */

/******************************************************************************
 * TYPEDEFS
 */

/******************************************************************************
 * LOCAL VARIABLES
 */

/******************************************************************************
 * GLOBAL VARIABLES
 */

/******************************************************************************
 * FUNCTION PROTOTYPES
 */

/******************************************************************************
 * @fn      SSP_CCM_Auth_Encrypt
 *
 * @brief   Generates CCM Authentication tag U.
 *
 * input parameters
 * @param encrypt if set to 'true' then run encryption and set to 'flase' for
 * authentication only.
 * @param   Mval    - Length of authentication field in octets [0,2,4,6,8,10,12,14 or 16]
 * @param   N       - Pointer to 13-byte Nonce
 * @param   M       - Pointer to octet string 'm'
 * @param   len_m   - Length of M[] in octets
 * @param   A       - Pointer to octet string 'a'
 * @param   len_a   - Length of A[] in octets
 * @param   AesKey  - Pointer to AES Key or Pointer to Key Expansion buffer.
 * @param   Cstate  - Pointer to output buffer
 * @param   ccmLVal - ccm L Value to be used.
 *
 * output parameters
 *
 * @param   Cstate[]    - The first Mval bytes contain Authentication Tag T
 *
 * @return  ZStatus_t
 *
 */
uint8 SSP_CCM_Auth_Encrypt (bool encrypt, uint8 Mval, uint8 *N, uint8 *M, uint16 len_m, uint8 *A,
                    uint16 len_a, uint8 *AesKey, uint8 *Cstate, uint8 ccmLVal)
{

  unsigned char status;

  if((status = CRYPTOCcmAuthEncrypt(encrypt, Mval, (uint32_t *)N, (uint32_t *)M,  len_m, (uint32_t *)A, len_a, 0,
                                            ccmLVal, false)) != AES_SUCCESS)
  {
    return status;
  }

  do
  {
    ASM_NOP;
  }while((CRYPTOCcmAuthEncryptStatus()));

  if((status = CRYPTOCcmAuthEncryptResultGet(Mval, (uint32_t *)Cstate)) != AES_SUCCESS)
  {
    return status;
  }
  return AES_SUCCESS;
}


/******************************************************************************
 * @fn      SSP_CCM_InvAuth_Decrypt
 *
 * @brief   Verifies CCM authentication.
 *
 * input parameters
 * @param decrypt if set to 'true' then run decryption and set to 'flase' for
 * authentication only.
 * @param   Mval    - Length of authentication field in octets [0,2,4,6,8,10,12,14 or 16]
 * @param   N       - Pointer to 13-byte Nonce
 * @param   C       - Pointer to octet string 'c' = 'm' || auth tag T
 * @param   len_c   - Length of C[] in octets
 * @param   A       - Pointer to octet string 'a'
 * @param   len_a   - Length of A[] in octets
 * @param   AesKey  - Pointer to AES Key or Pointer to Key Expansion buffer.
 * @param   Cstate  - Pointer to AES state buffer (cannot be part of C[])
 * @param   ccmLVal - ccm L Value to be used.
 *
 * output parameters
 *
 * @param   Cstate[]    - The first Mval bytes contain computed Authentication Tag T
 *
 * @return  0 = Success, 1 = Failure
 *
 */
uint8 SSP_CCM_InvAuth_Decrypt (bool decrypt, uint8 Mval, uint8 *N, uint8 *C, uint16 len_c, uint8 *A,
                       uint16 len_a, uint8 *AesKey, uint8 *Cstate, uint8 ccmLVal)
{

  unsigned char status;
  if((status = CRYPTOCcmInvAuthDecrypt(decrypt, Mval, (uint32_t *)N, (uint32_t *)C, len_c, (uint32_t *)A, len_a, 0,
                                         ccmLVal, false))!= AES_SUCCESS )
  {
    return status;
  }

  /* wait for completion of the operation */
  do
  {
    ASM_NOP;
  }while((CRYPTOCcmInvAuthDecryptStatus()));


  if((status = CRYPTOCcmInvAuthDecryptResultGet(Mval, (uint32_t *)C, len_c, (uint32_t *)Cstate)) != AES_SUCCESS)
  {
    return status;
  }

  return AES_SUCCESS;
}







