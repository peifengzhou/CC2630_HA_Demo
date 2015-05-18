/**************************************************************************************************
  Filename:       ssp_hash.c
  Revised:        $Date: 2013-05-07 15:00:24 -0700 (Tue, 07 May 2013) $
  Revision:       $Revision: 34180 $

  Description:    Provides the keyed hash functions for message authentication.


  Copyright 2004-2013 Texas Instruments Incorporated. All rights reserved.

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
**************************************************************************************************/

/******************************************************************************
 * INCLUDES
 */

#include "nwk_globals.h"
#include "hal_ccm.h"
#include "hal_aes.h"
#include "OSAL.h"
#include "ssp_hash.h"

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
void sspMMOHash (uint8 *Pb, uint8 prefix, uint8 *Mb, uint16 bitlen, uint8 *Cstate);

/******************************************************************************
 * @fn      SSP_KeyedHash
 *
 * @brief   Performs the Keyed-Hash Message Authentication Code (HMAC) as
 *          defined in FIPS-198.  Assumes block size = key size = 128 bits.
 *
 * input parameters
 *
 * @param   Mb          - Pointer to message block to hash
 * @param   bitlen      - Length of M[] in bits
 * @param   Aeskey      - Pointer to AES Key
 * @param   Cstate      - Pointer to Hash output buffer
 *
 * output parameters
 *
 * @param   Cstate[]    - HMAC output
 *
 * @return  None
 */
void SSP_KeyedHash (uint8 *Mb, uint16 bitlen, uint8 *AesKey, uint8 *Cstate)
{
  uint8   Kx[16], T[16], i;

  //
  // Since block_size = key_size = 128 bits, no additional key
  // processing is needed, so go directly to HMAC step 4 (XORing
  // with ipad).
  //

  for (i=0; i < 16; i++)
  {
      Kx[i] = AesKey[i] ^ 0x36;           // Kx[] = Key1 = AesKey ^ ipad
  }

  sspMMOHash (Kx, 1, Mb, bitlen, Cstate);  // Cstate[] = Hash1 = Hash(Key1,M)

  osal_memcpy (T, Cstate, 16);            // T = Hash1

  for (i=0; i < 16; i++)
  {
      Kx[i] = AesKey[i] ^ 0x5c;           // Kx[] = Key2 = AesKey ^ opad
  }

  sspMMOHash (Kx, 1, T, 128, Cstate);     // Cstate[] = Hash2 = Hash(Key2,Hash1)
}

/******************************************************************************
 * @fn      sspMMOHash
 *
 * @brief   Performs the Matyas-Meyer-Oseas hash function on 1 or 2 blocks
 *          of data using AES-128.
 *
 * input parameters
 *
 * @param   Pb      - Pointer to prefix block (if any) to hash
 * @param   prefix  - 1 if 16-byte prefix block exists, 0 otherwise
 * @param   Mb      - Pointer to message block to hash
 * @param   bitlen  - Length of M[] in bits
 * @param   Cstate  - Pointer to Hash output buffer
 *
 * output parameters
 *
 * @param   Cstate[]    - Hash output
 *
 * @return  None
 */
void sspMMOHash (uint8 *Pb, uint8 prefix, uint8 *Mb, uint16 bitlen, uint8 *Cstate)
{
  uint16  blocks, blength;
  uint8   T[16], i, remainder, *dptr;
  uint8   AesKey[16];

  blength = bitlen / 8;
  blocks = blength / 16;
  remainder = blength % 16;

  osal_memset (Cstate, 0, 16);        // Cstate == Hash0

  //
  // If prefix is true, then the message to hash comes in two pieces.
  // The first piece is a 16-byte octet stored in P[], and the second
  // piece is the 'bitlen'-bit message stored in M[].
  //
  if (prefix)     // Process first 16-bytes in P[]
  {
    osal_memcpy (AesKey, Cstate, 16);   // Key = Hash0
    osal_memcpy (Cstate, Pb, 16);
    sspAesEncrypt (AesKey, Cstate);     // Cstate[] = E(Hash0,P)
    for (i=0; i < 16; i++)
    {
      Cstate[i] ^= Pb[i];  // Cstate[] = Hash1 = E(Hash0,P) ^ P
    }
    bitlen += 128;      // adjust bitlen because of the extra 16-bytes of prefix
  }

  dptr = Mb;
  while (blocks--)
  {
    osal_memcpy (AesKey, Cstate, 16);   // Key = Hash(i)
    osal_memcpy (Cstate, dptr, 16);
    sspAesEncrypt (AesKey, Cstate);     // Cstate[] = E(Hash(i-1), M(i))
    for (i=0; i < 16; i++)
    {
      Cstate[i] ^= *dptr++;  // Cstate[] = Hash(i) = E(Hash(i-1),M(i)) ^ M(i)
    }
  }

  if (remainder == 0)
  {
    //
    // Blocks have been completely filled, so we need to create a final
    // 16-byte fill block.
    //
    osal_memcpy (AesKey, Cstate, 16);   // Key = Hash(i)
    osal_memset (Cstate, 0, 14);
    Cstate[0] = 0x80;
    Cstate[14] = (uint8) (bitlen >> 8);
    Cstate[15] = (uint8) (bitlen);
    osal_memcpy (T, Cstate, 16);
    sspAesEncrypt (AesKey, Cstate);     // Cstate[] = E(Hash(i-1), T)
    for (i=0; i < 16; i++)
    {
      Cstate[i] ^= T[i];  // Cstate[] = Hash(i) = E(Hash(i-1),T) ^ T
    }
  }
  else if (remainder > 13)        // remainder == (14 or 15)
  {
    // Cannot fit the minimum 3 fill bytes into current block, so we need to fill out the
    // current block and then create another 16-byte fill block.
    //
    osal_memcpy (AesKey, Cstate, 16);   // Key = Hash(i)
    osal_memset (Cstate, 0, 16);
    osal_memcpy (Cstate, dptr, remainder);
    Cstate[remainder] = 0x80;
    osal_memcpy (T, Cstate, 16);
    sspAesEncrypt (AesKey, Cstate);     // Cstate[] = E(Hash(i-1), T)
    for (i=0; i < 16; i++)
    {
      Cstate[i] ^= T[i];  // Cstate[] = Hash(i) = E(Hash(i-1),T) ^ T
    }

    //
    // Create final 16-byte fill block.
    //
    osal_memcpy (AesKey, Cstate, 16);   // Key = Hash(i)
    osal_memset (Cstate, 0, 14);
    Cstate[14] = (uint8) (bitlen >> 8);
    Cstate[15] = (uint8) (bitlen);
    osal_memcpy (T, Cstate, 16);
    sspAesEncrypt (AesKey, Cstate);     // Cstate[] = E(Hash(i-1), T)
    for (i=0; i < 16; i++)
    {
      Cstate[i] ^= T[i];  // Cstate[] = Hash(i) = E(Hash(i-1),T) ^ T
    }
  }
  else  //  0 < remainder < 14
  {
    //
    // There is room left in the final block to fit the 3 bytes of fill data.
    // 0x80 goes in the next available byte.  The bit-length goes in the last
    // two bytes.
    //
    osal_memcpy (AesKey, Cstate, 16);   // Key = Hash(i)
    osal_memset(Cstate, 0, 14);
    osal_memcpy (Cstate, dptr, remainder);
    Cstate[remainder] = 0x80;
    Cstate[14] = (uint8) (bitlen >> 8);
    Cstate[15] = (uint8) (bitlen);
    osal_memcpy (T, Cstate, 16);
    sspAesEncrypt (AesKey, Cstate);     // Cstate[] = E(Hash(i-1), T)
    for (i=0; i < 16; i++)
    {
      Cstate[i] ^= T[i];  // Cstate[] = Hash(i) = E(Hash(i-1),T) ^ T
    }
  }
}

/******************************************************************************
 * @fn      sspAesEncrypt
 *
 * @brief   Performs the AES function
 *
 * input parameters
 *
 * @param
 * @param
 *
 * output parameters
 *
 * @param
 *
 * @return  None
 */
void sspAesEncrypt( uint8 *key, uint8 *buf )
{

  ssp_HW_KeyInit( key );
  sspAesEncryptHW( key, buf );
}


/******************************************************************************
 * @fn      sspAesDecrypt
 *
 * @brief   Performs the AES function
 *
 * input parameters
 *
 * @param
 * @param
 *
 * output parameters
 *
 * @param
 *
 * @return  None
 */
void sspAesDecrypt( uint8 *key, uint8 *buf )
{

  ssp_HW_KeyInit( key );
  sspAesDecryptHW( key, buf );
}


/*********************************************************************
*********************************************************************/
