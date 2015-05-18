/******************************************************************************
  Filename:       ssp.c
  Revised:        $Date: 2015-02-15 21:33:19 -0800 (Sun, 15 Feb 2015) $
  Revision:       $Revision: 42620 $

  Description:    Security Service Provider (SSP) interface.


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
******************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "ZComDef.h"
#include "OSAL.h"
#include "nwk_globals.h"
#include "nwk_util.h"
#include "ssp_hash.h"
#include "ssp.h"
#include "hal_ccm.h"
#include "hal_aes.h"
#include "ZGlobals.h"
#include "OSAL_Nv.h"
#include "ZDSecMgr.h"
#include "hal_mcu.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define SSP_MMO_MB_LEN     (4)
#define SSP_MMO_MB_BIT_LEN (SSP_MMO_MB_LEN * 8)

#define SSP_MACDATA_LEN_BASE  (uint8)           \
                              (Z_EXTADDR_LEN +  \
                               Z_EXTADDR_LEN +  \
                               SEC_KEY_LEN   +  \
                               SEC_KEY_LEN       )

#define SSP_MACDATA_LEN       (uint8)(SSP_MACDATA_LEN_BASE + 1)

#define SSP_MACDATA_LEN_WTEXT (uint8)            \
                              (SSP_MACDATA_LEN + \
                               SSP_TEXT_LEN       )

#define SSP_MACDATA_BASE_BIT_LEN  (SSP_MACDATA_LEN_BASE*8)
#define SSP_MACDATA_BIT_LEN       (SSP_MACDATA_LEN*8)
#define SSP_MACDATA_BIT_LEN_WTEXT (SSP_MACDATA_LEN_WTEXT*8)

/*********************************************************************
 * TYPEDEFS
 */
typedef ZStatus_t (*SSP_NwkSecurityHook_t)(uint8 ed_flag, uint8 *msg, uint8 hdrLen, uint8 nsduLen);
typedef uint8 (*SSP_GetMicLenHook_t)( uint8 securityLevel );

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint32 nwkFrameCounter;

uint16 nwkFrameCounterChanges = 0;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 storedRand;

static SSP_NwkSecurityHook_t SSP_NwkSecurityHook = NULL;
static SSP_GetMicLenHook_t   SSP_GetMicLenHook   = NULL;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
ZStatus_t SSP_NwkSecurityProc(uint8 ed_flag, uint8 *msg, uint8 hdrLen, uint8 nsduLen);

uint8 SSP_GetMicLenProc( uint8 securityLevel );

void SSP_BuildAuxHdr( uint8 secLevel, uint8 keyId, uint8 extNonce,
                      uint32 frameCntr, uint8 *auxHdr );

void SSP_BuildAuxHdr2( SSP_Info_t* si );

ZStatus_t ccmStar( uint8 ed_flag, ssp_ctx *stx,
                   uint8 *nonce, uint8 *secMsg );

/*********************************************************************
 * @fn      SSP_Init
 *
 * @brief   This function initializes the SSP
 *
 * @param   none
 *
 * @return  none
 */
void SSP_Init( void )
{
  SSP_NwkSecurityHook = SSP_NwkSecurityProc;
  SSP_GetMicLenHook   = SSP_GetMicLenProc;
}

/*********************************************************************
 * @fn      SSP_MemCpyReverse
 *
 * @brief   Duplicate osal_memcpy functionality, but reverse copy
 *
 * @param   dst - destination buffer
 * @param   src - source buffer copied in reverse
 * @param   len - length
 *
 * @return  ( dst + len )
 */
uint8* SSP_MemCpyReverse( uint8* dst, uint8* src, unsigned int len )
{
  uint8* rev;

  rev = src + (len-1);

  while( len-- )
  {
    *dst++ = *rev--;
  }

  return dst;
}

/*********************************************************************
 * @fn      SSP_ParseAuxHdr
 *
 * @brief   Parse auxilliary header.
 *
 * @param   si - SSP_Info_t
 *
 * @return  void
 */
void SSP_ParseAuxHdr(SSP_Info_t* si)
{
  uint8* auxHdr;
  uint8  extNonce;

  //get start of auxilliary header
  auxHdr = si->pdu + si->hdrLen;

  //parse security control
  si->secLevel = auxHdr[SSP_AUXHDR_CTRL] & SSP_AUXHDR_LEVEL_MASK;

  si->keyID = (((auxHdr[SSP_AUXHDR_CTRL])>>
                (SSP_AUXHDR_KEYID_SHIFT )  )&
               (SSP_AUXHDR_KEYID_MASK      ) );

  extNonce = (((auxHdr[SSP_AUXHDR_CTRL]  )>>
               (SSP_AUXHDR_EXTNONCE_SHIFT)  )&
              (SSP_AUXHDR_EXTNONCE_BIT      )  );

  //add frame counter
  si->frmCntr = osal_build_uint32( &auxHdr[SSP_AUXHDR_FRAMECNTR], 4 );

  //set default auxilliary length
  si->auxLen = SSP_AUXHDR_MIN_LEN;

  //check for extended nonce
  if ( extNonce == SSP_AUXHDR_EXTNONCE_BIT )
  {
    //set extended address
    osal_cpyExtAddr( si->extAddr, &auxHdr[si->auxLen] );

    //add to auxilliary length
    si->auxLen += Z_EXTADDR_LEN;
  }
  else
  {
    //set extended address
    osal_memset( si->extAddr, 0x00, Z_EXTADDR_LEN );
  }

  //check for NWK key
  if ( si->keyID == SEC_KEYID_NWK )
  {
    si->keySeqNum = auxHdr[si->auxLen];

    //add to auxilliary length
    si->auxLen += SSP_AUXHDR_SEQNUM_LEN;
  }
}

/*********************************************************************
 * @fn      SSP_BuildAuxHdr2
 *
 * @brief   Build auxilliary header.
 *
 * @param   si - SSP_Info_t
 *
 * @return  void
 */
void SSP_BuildAuxHdr2(SSP_Info_t* si)
{
  uint8* auxHdr;
  uint8  extNonce;

  //set extNonce bit value
  if(si->auxLen==SSP_AUXHDR_EXT_LEN)
  {
    extNonce = SSP_AUXHDR_EXTNONCE_BIT;
  }
  else
  {
    extNonce = 0;
  }

  //get start of auxilliary header
  auxHdr = si->pdu + si->hdrLen;

  //add security control
  *auxHdr++ = ((si->secLevel                          )|
               (si->keyID << SSP_AUXHDR_KEYID_SHIFT   )|
               (extNonce  << SSP_AUXHDR_EXTNONCE_SHIFT) );

  //add frame counter
  auxHdr = osal_buffer_uint32( auxHdr, si->frmCntr );

  //check for extended nonce bit
  if ( extNonce == 1 )
  {
    //add extended address
    osal_cpyExtAddr( auxHdr, si->extAddr );

    auxHdr += Z_EXTADDR_LEN;
  }

  //check for NWK key
  if ( si->keyID == SEC_KEYID_NWK )
  {
    //add NWK key sequence number
    *auxHdr = si->keySeqNum;
  }
}

/*********************************************************************
 * @fn      SSP_Process
 *
 * @brief   Process security information.
 *
 * @param   si - SSP_Info_t
 *
 * @return  ZStatus_t
 */
ZStatus_t SSP_Process( SSP_Info_t* si )
{
  ZStatus_t status;
  uint8     nonce[SSP_NONCE_LEN];
  uint8*    auxHdr;
  ssp_ctx   stx;
  uint8     tmp;
  uint8     tmpKey[SEC_KEY_LEN];
  uint8     keyFromNv[SEC_KEY_LEN];

  //get auxilliary header for security control
  auxHdr = si->pdu + si->hdrLen;

  if ( si->dir == SSP_APPLY )
  {
    //build auxillary header
    SSP_BuildAuxHdr2( si );
  }
  else
  {
    //restore level bits to security control
    auxHdr[SSP_AUXHDR_CTRL] |= si->secLevel;
  }

  //build nonce
  SSP_BuildNonce ( si->extAddr,
                   si->frmCntr,
                   auxHdr[SSP_AUXHDR_CTRL],
                   nonce );

  //process protocol data unit
  stx.hdrLen   = si->hdrLen;
  stx.auxLen   = si->auxLen;
  stx.msgLen   = si->sduLen;
  stx.secLevel = si->secLevel;

  // get key from NV into local variable
  ZDSecMgrReadKeyFromNv(si->keyNvId, keyFromNv);
  stx.key      = keyFromNv;

  tmp = 0xFF;

  //devtag.pro.security.todo - test size on target
  if ( si->keyID == SEC_KEYID_TRANSPORT )
  {
    tmp = 0x00;
  }
  else if ( si->keyID == SEC_KEYID_LOAD )
  {
    tmp = 0x02;
  }

  if ( tmp != 0xFF )
  {
    // pass the real key to hash function, both copies will be cleared before return
    SSP_KeyedHash ( &tmp, 8, keyFromNv, tmpKey );

    stx.key = tmpKey;
  }

  status = ccmStar( si->dir,
                    &stx,
                    nonce,
                    si->pdu );

  //clear level bits in security control
  auxHdr[SSP_AUXHDR_CTRL] &= ~SSP_AUXHDR_LEVEL_MASK;

  // Clear copy in RAM before return
  osal_memset( tmpKey, 0x00, SEC_KEY_LEN );
  osal_memset( keyFromNv, 0x00, SEC_KEY_LEN );

  return status;
}

/*
 * Process MAC TAG Data - Generate Tags
 */
/*********************************************************************
 * @fn      SSP_GetMacTags
 *
 * @brief   Process Mac tag data and generate tags.
 *
 * @param   data - SSP_MacTagData_t
 *
 * @return  ZStatus_t
 */
ZStatus_t SSP_GetMacTags( SSP_MacTagData_t* data )
{
  uint8* buf;
  uint8* pBuf;
  uint8  macTag[SEC_KEY_LEN];
  uint8  mmoMb[SSP_MMO_MB_LEN] = {0x00, 0x00, 0x00, 0x01};
  uint8  macKeyOut[SEC_KEY_LEN];
  uint8* macKey;
  uint16 macDataBitLen;

  buf = osal_mem_alloc(SSP_MACDATA_LEN_WTEXT);

  if(buf)
  {
    // construct macData
    pBuf = buf + 1;   // leave space for one byte for later use
    pBuf = SSP_MemCpyReverse(pBuf, data->initExtAddr, Z_EXTADDR_LEN);
    pBuf = SSP_MemCpyReverse(pBuf, data->rspExtAddr, Z_EXTADDR_LEN);
    pBuf = osal_memcpy(pBuf, data->qeu, SEC_KEY_LEN);
    pBuf = osal_memcpy(pBuf, data->qev, SEC_KEY_LEN);
    pBuf = SSP_MemCpyReverse(pBuf, data->text2, SSP_TEXT_LEN);

    // get macTag
    SSP_KeyedHash(buf+1, SSP_MACDATA_BASE_BIT_LEN, data->key, macTag);

    // get macKey
    sspMMOHash(macTag, 1, mmoMb, SSP_MMO_MB_BIT_LEN, macKeyOut);

    // get linkKey
    mmoMb[3] = 0x02;
    sspMMOHash(macTag, 1, mmoMb, SSP_MMO_MB_BIT_LEN, data->linkKey);

    // setup values for macKey and macData length based on tag type
    if(data->type == SSP_MAC_TAGS_SKKE)
    {
      macKey        = macKeyOut;
      macDataBitLen = SSP_MACDATA_BIT_LEN;
    }
    else
    {
      macKey        = data->key;
      macDataBitLen = SSP_MACDATA_BIT_LEN_WTEXT;
    }

    // generate macData2 first
    // tag2...
    // buf should be set to macData2 = 03 || u || v || qeu || qev || OPT(text2)
    buf[0] = 0x03;
    SSP_KeyedHash(buf, macDataBitLen, macKey, data->tag2); // result contains macTag2

    // generate macData1 next
    // tag1...
    //reset pBuf
    pBuf = buf + 1;
    // buf should be set to macData1 = 02|| v || u || qev || qeu || OPT(text1)
    buf[0] = 0x02;
    pBuf = SSP_MemCpyReverse(pBuf, data->rspExtAddr, Z_EXTADDR_LEN);
    pBuf = SSP_MemCpyReverse(pBuf, data->initExtAddr, Z_EXTADDR_LEN);
    pBuf = osal_memcpy(pBuf, data->qev, SEC_KEY_LEN);
    pBuf = osal_memcpy(pBuf, data->qeu, SEC_KEY_LEN);
    pBuf = SSP_MemCpyReverse(pBuf, data->text1, SSP_TEXT_LEN);

    SSP_KeyedHash(buf, macDataBitLen, macKey, data->tag1); // result contains macTag1

    osal_mem_free(buf);

    return ZSuccess;
  }
  else
  {
    return ZMemError;
  }
}

/*********************************************************************
 * @fn      SSP_GetTrueRand
 *
 * @brief   returns 8*len random bits ( currently set to 128 bits )
 *
 * @param   len -
 * @param   rand -
 *
 * @return  none
 */
void SSP_GetTrueRand( uint8 len, uint8 *rand )
{
  uint8 inputData[8];

  if ( len != 16 )   // input to sspMmoHash assumes rand is a 16-byte buffer
  {
    return;
  }

  // use mix of system variables....
  // ...plus a byte leftover from previous hash for the input data

  inputData[0] = storedRand;

  inputData[1] = LO_UINT16( ( uint16 ) osal_GetSystemClock() );
  inputData[2] = HI_UINT16( ( uint16 ) osal_GetSystemClock() );

  osal_memcpy( &(inputData[3]), NLME_GetExtAddr(), 2 );

  // try adding data from arbitrary ram/stack memory locations ??
  // update to use h/w input..
  sspMMOHash( (uint8 *)NULL, 0, inputData, 64, rand );

  storedRand = rand[0];
}

/*********************************************************************
 * @fn      SSP_StoreRandomSeedNV
 *
 * @brief   Store the 16 byte random seed in NV
 *
 * @param   pSeed - pointer to the random seed
 *
 * @return  none
 */
void SSP_StoreRandomSeedNV( uint8 *pSeed )
{
  // Initialize the nv item and write the random seed to NV
  (void)osal_nv_item_init( ZCD_NV_RANDOM_SEED, SEC_KEY_LEN , NULL );
  (void)osal_nv_write( ZCD_NV_RANDOM_SEED, 0, SEC_KEY_LEN , pSeed );
}

/*********************************************************************
 * @fn      SSP_GetTrueRandAES
 *
 * @brief   returns 8*len random bits ( currently less than 128 bits )
 *
 * @param   len -
 * @param   rand -
 *
 * @return  ZStatus_t
 */
ZStatus_t SSP_GetTrueRandAES( uint8 len, uint8 *rand )
{
  uint8 *pBuf;
  uint8 *pRandom;
  uint8 *pSeed;

  if ( (len > 16) || (len == 0) || (rand == NULL) )  // Assume rand <= 16 byte buffer
  {
    return ZInvalidParameter;
  }

  // Allocate memory for two 16 bytes temporary buffer on the heap to avoid
  // using the stack, since ECC lib uses huge stack space.
  if( ( pSeed = osal_mem_alloc( SEC_KEY_LEN *2 )) == NULL )
  {
    uint8 i;
    uint16 rand16;

    // If memory allocation fails, call osal_rand to generate the random number;
    for( i = 0; i < 8; i++ )
    {
      rand16 = osal_rand();
      *rand++ = LO_UINT16( rand16 );
      *rand++ = HI_UINT16( rand16 );
    }
  }
  else
  {
    uint32 rngCntr;  // Purposely uninitialized to add fuzzy randomness to its initial value.

    (void)osal_nv_item_init(ZCD_NV_RNG_COUNTER, sizeof(rngCntr), &rngCntr);
    (void)osal_nv_read(ZCD_NV_RNG_COUNTER, 0, sizeof(rngCntr), &rngCntr);
    pRandom = pSeed + SEC_KEY_LEN;

    // Generate 128 random bits by encrypting a 128-bit random seed
    // with 128-bit prefix, constructed by a mix of system variables:
    // IEEE addrress || 0x0000 || System Clock || Counter.
    // Notice that the coutner will wrap after 2^32 times.
    pBuf = osal_buffer_uint32(pRandom, rngCntr++);
    (void)osal_nv_write(ZCD_NV_RNG_COUNTER, 0, sizeof(rngCntr), &rngCntr);
    *pBuf++ = LO_UINT16( ( uint16 ) osal_GetSystemClock() );
    *pBuf++ = HI_UINT16( ( uint16 ) osal_GetSystemClock() );
    *pBuf++ = 0;
    *pBuf++ = 0;
    (void)osal_memcpy(pBuf, NLME_GetExtAddr(), 8);

    // Read the previously generated seed from nv.
    (void)osal_nv_read(ZCD_NV_RANDOM_SEED, 0, SEC_KEY_LEN , pSeed);

    // Encrypt the buffer with the seed as the AES key.
    sspAesEncrypt( pSeed, pRandom );

    // Copy the random bits to the output buffer
    (void)osal_memcpy(rand, pRandom, len);

    (void)osal_mem_free(pSeed);
  }

  return ZSuccess;
}

/*********************************************************************
 * @fn      SSP_ReadNwkActiveKey
 *
 * @brief   Reads the active key and framecounter.
 *
 * @param   *items - pointer to the Active Key data
 *
 * @return  none
 */
void SSP_ReadNwkActiveKey( nwkActiveKeyItems *items )
{
  // get Active Key directly from NV
  if ( osal_nv_read( ZCD_NV_NWK_ACTIVE_KEY_INFO, 0,
                     sizeof(nwkKeyDesc), &(items->active) ) == SUCCESS )
  {
    items->frameCounter = nwkFrameCounter;
  }
}

/*********************************************************************
 * @fn      SSP_UpdateNwkKey
 *
 * @brief   This function sets the alternate network key.
 *
 * @param   uint8 *key - pointer to key data
 *          uint8 keySeqNum - sequence number of provided key
 *
 * @return  void
 */
void SSP_UpdateNwkKey( uint8 *key, uint8 keySeqNum )
{
  nwkKeyDesc tmpAltKey;

  tmpAltKey.keySeqNum = keySeqNum;
  osal_memcpy( &(tmpAltKey.key), key, SEC_KEY_LEN );

  // Write nwkAlternateKey directly to NV
  (void)osal_nv_write( ZCD_NV_NWK_ALTERN_KEY_INFO, 0,
                       sizeof( nwkKeyDesc ), &tmpAltKey );

  // Clear copy in RAM before return
  osal_memset( &(tmpAltKey), 0x00, sizeof(nwkKeyDesc) );

}   /* SSP_UpdateNwkKey */

/*********************************************************************
 * @fn      SSP_SwitchNwkKey
 *
 * @brief   Makes key identified by seqNum as the active key
 *
 * @param   uint8 seqNum - sequence number to compare with stored sequence number
 *
 * @return  void
 */
void SSP_SwitchNwkKey( uint8 seqNum )
{
  nwkKeyDesc *pActiveKeyInfo;
  nwkKeyDesc *pAlternateKeyInfo;
  nwkActiveKeyItems keyItems;

  pAlternateKeyInfo = (nwkKeyDesc *)osal_mem_alloc(sizeof(nwkKeyDesc));
  if (pAlternateKeyInfo != NULL)
  {
    // Get Alternate Key from NV
    if (osal_nv_read(ZCD_NV_NWK_ALTERN_KEY_INFO, 0,
                     sizeof(nwkKeyDesc), pAlternateKeyInfo) == SUCCESS)
    {
      if (pAlternateKeyInfo->keySeqNum == seqNum)
      {
        // Sequence numbers match
        pActiveKeyInfo = (nwkKeyDesc *)osal_mem_alloc(sizeof(nwkKeyDesc));
        if (pActiveKeyInfo != NULL)
        {
          // Get Active Key from NV
          if (osal_nv_read(ZCD_NV_NWK_ACTIVE_KEY_INFO, 0,
                           sizeof(nwkKeyDesc), pActiveKeyInfo) == SUCCESS)
          {
            // Swap active and alternate keys, writing them directly to NV
            (void)osal_nv_write(ZCD_NV_NWK_ACTIVE_KEY_INFO, 0,
                                sizeof(nwkKeyDesc), pAlternateKeyInfo);

            (void)osal_nv_write(ZCD_NV_NWK_ALTERN_KEY_INFO, 0,
                                sizeof(nwkKeyDesc), pActiveKeyInfo);

            _NIB.nwkKeyLoaded = TRUE;
            nwkFrameCounter = 0;

            osal_memcpy( &keyItems.active, pAlternateKeyInfo, sizeof(nwkKeyDesc) );
            keyItems.frameCounter = nwkFrameCounter;

            // Write the NWK key to NV
            (void)osal_nv_write( ZCD_NV_NWKKEY, 0, sizeof( nwkActiveKeyItems ),
                                 (void *)&keyItems );
          }

          // Clear Active Key Information
          osal_memset(pActiveKeyInfo, 0x00, sizeof(nwkKeyDesc));
          osal_mem_free(pActiveKeyInfo);
        }
      }
    }

    // Clear Alternate Key Information
    osal_memset(pAlternateKeyInfo, 0x00, sizeof(nwkKeyDesc));
    osal_mem_free(pAlternateKeyInfo);
  }
}

/*********************************************************************
 * @fn      SSP_BuildAuxHdr
 *
 * @brief   This function builds the auxillary header.
 *
 * @param   uint8 secLevel - security level
 *          uint8 keyId - security key Id
 *          uint8 extNonce - nonce
 *          uint32 frameCntr - frame counter for the key
 *          uint8 *auxHdr - pointer to auxillary header
 *
 * @return  void
 */
void SSP_BuildAuxHdr( uint8 secLevel, uint8 keyId, uint8 extNonce,
                      uint32 frameCntr, uint8 *auxHdr )
{
  uint8   tmpKeySeqNum;

  *auxHdr++ = secLevel |
              ( keyId << 3) |
              ( extNonce << 5 );

  auxHdr = osal_buffer_uint32( auxHdr, frameCntr );

  if ( extNonce )
  {
    osal_cpyExtAddr( auxHdr, NLME_GetExtAddr() );
    auxHdr += Z_EXTADDR_LEN;
  }

  if ( keyId == SEC_KEYID_NWK )
  {
    // get only the Sequence Number into a local variable
    if( NLME_ReadNwkKeyInfo( osal_offsetof(nwkKeyDesc, keySeqNum),
                             sizeof(tmpKeySeqNum), &tmpKeySeqNum,
                             ZCD_NV_NWK_ACTIVE_KEY_INFO ) == SUCCESS )
    {
      *auxHdr = tmpKeySeqNum;
    }
    else
    {
      *auxHdr = 0;
    }
  }
}

/*********************************************************************
 * @fn      SSP_BuildNonce
 *
 * @brief   This function builds the nonce string
 *
 * @param
 *
 * @return  void
 */
void SSP_BuildNonce( uint8 *addr, uint32 frameCntr, uint8 secCtrl, uint8 *nonce )
{
  uint8 *pBuf;

  pBuf = nonce;

  // Start with a 64-bit address
  osal_cpyExtAddr( pBuf, addr );
  pBuf += Z_EXTADDR_LEN;

  // Follow with frame counter
  pBuf = osal_buffer_uint32( pBuf, frameCntr );

  // Finish with security control
  *pBuf = secCtrl;
}

/*********************************************************************
 * @fn      SSP_GetMicLenProc
 *
 * @brief   This function returns length of mic string
 *
 * @param   uint8 securityLevel - security level
 *
 * @return  uint8 - length of mic string
 */
uint8 SSP_GetMicLenProc( uint8 securityLevel )
{
  return( ( 0x02 << ( securityLevel & 0x03 ) ) & 0x1C );
}

/*********************************************************************
 * @fn      SSP_GetNwkKey
 *
 * @brief   Indentifies the NWK key based on keySecNum value. Reads
 *          only the sequence numbers from NV and decides what key
 *          type the caller should use.
 *
 * @param   uint8 seqNum - sequence number
 *
 * @return  uint16 - NV ID of the key to use
 */
uint16 SSP_GetNwkKey( uint8 seqNum )
{
  uint16 keyNvId;
  uint8 tmpActiveKeySeqNum;
  uint8 tmpAlternKeySeqNum;

  if (NLME_ReadNwkKeyInfo(osal_offsetof(nwkKeyDesc, keySeqNum),
                                 sizeof(tmpActiveKeySeqNum), &tmpActiveKeySeqNum,
                                 ZCD_NV_NWK_ACTIVE_KEY_INFO) == SUCCESS)
  {
    if ( seqNum == tmpActiveKeySeqNum )
    {
      keyNvId = ZCD_NV_NWK_ACTIVE_KEY_INFO;
    }
    else if (NLME_ReadNwkKeyInfo(osal_offsetof(nwkKeyDesc, keySeqNum),
                                 sizeof(tmpAlternKeySeqNum), &tmpAlternKeySeqNum,
                                 ZCD_NV_NWK_ALTERN_KEY_INFO) == SUCCESS)
    {
      if (seqNum == tmpAlternKeySeqNum)
      {
        // switch keys if this is newer
        if (tmpAlternKeySeqNum > tmpActiveKeySeqNum)
        {
          SSP_SwitchNwkKey(tmpAlternKeySeqNum);
          keyNvId = ZCD_NV_NWK_ACTIVE_KEY_INFO;
        }
        else
        {
          keyNvId = ZCD_NV_NWK_ALTERN_KEY_INFO;
        }
      }
      else
      {
        keyNvId = SEC_NO_KEY_NV_ID;
        // if seqNum is greater than either, give
        // indication to higher layer to request latest
        // nwk key...not in spec
      }
    }
  }
  else
  {
    // if NV read fails return No key index
    keyNvId = SEC_NO_KEY_NV_ID;
  }

  return(keyNvId);
}

/*********************************************************************
 * @fn      SSP_NwkSecurityProc
 *
 * @brief   secure/unsecure the nwk layer packet
 *
 * @param   uint8 ed_flag - action flag
 *          uint8 *msg - message to process
 *          uint8 hdrLen - header length
 *          uint8 nsduLen - nsdu length
 *
 * @return  ZStatus_t
 */
ZStatus_t SSP_NwkSecurityProc(uint8 ed_flag, uint8 *msg, uint8 hdrLen, uint8 nsduLen)
{
  uint8 nonce[SSP_NONCE_LEN];
  ssp_ctx stx;
  ZStatus_t ret;
  nwkKeyDesc tmpKey;
  uint16 keyNvId;

  stx.auxLen = NWK_AUX_HDR_LEN;
  stx.hdrLen = hdrLen;
  stx.msgLen = nsduLen;
  stx.secLevel = _NIB.SecurityLevel;
  stx.keyId = SEC_KEYID_NWK;

  if ( ed_flag == SSP_APPLY )
  {
    // get Network Active key directly from NV
    if( NLME_ReadNwkKeyInfo( 0, sizeof(tmpKey), &tmpKey,
                             ZCD_NV_NWK_ACTIVE_KEY_INFO ) == SUCCESS )
    {
      stx.key = tmpKey.key;

      SSP_BuildAuxHdr( stx.secLevel, SEC_KEYID_NWK, 1, nwkFrameCounter++,
                       msg + hdrLen );
      // Create nonce...8 byte of extAddr, 4 byte frmCntr, 1 byte control field
      osal_cpyExtAddr( nonce, NLME_GetExtAddr() );

      nwkFrameCounterChanges++;

      if ( nwkFrameCounterChanges > gMAX_NWK_FRAMECOUNTER_CHANGES )
      {
        // Notify the ZDApp that the frame counter has changed.
        osal_set_event( ZDAppTaskID, ZDO_FRAMECOUNTER_CHANGE );
      }
    }
    else
    {
      stx.key = NULL;
    }
  }
  else
  {
    keyNvId = SSP_GetNwkKey( *(msg + hdrLen + NWK_AUX_HDR_LEN - 1) );

    if (keyNvId == ZCD_NV_NWK_ACTIVE_KEY_INFO)
    {
      // get Network Active key directly from NV
      if( NLME_ReadNwkKeyInfo( 0, sizeof(tmpKey), &tmpKey,
                               ZCD_NV_NWK_ACTIVE_KEY_INFO ) == SUCCESS )
      {
        stx.key = tmpKey.key;
      }
      else
      {
        stx.key = NULL;
      }
    }
    else if (keyNvId == ZCD_NV_NWK_ALTERN_KEY_INFO)
    {
      // get Network Alternate key directly from NV
      if( NLME_ReadNwkKeyInfo( 0, sizeof(tmpKey), &tmpKey,
                               ZCD_NV_NWK_ALTERN_KEY_INFO ) == SUCCESS )
      {
        stx.key = tmpKey.key;
      }
      else
      {
        stx.key = NULL;
      }
    }
    else
    {
      stx.key = NULL;
    }

    // Overwrite the security control field
    *(msg+hdrLen) |= stx.secLevel;

    // Create nonce... 8 byte of extAddr, 4 byte frmCntr, 1 byte control field
    osal_cpyExtAddr( nonce, msg+hdrLen+5 );
  }

  //...rest of the nonce
  osal_memcpy( nonce+Z_EXTADDR_LEN, msg+hdrLen+1, 4);
  *(nonce + 12) = *(msg + hdrLen);

  if ( stx.key == NULL )
  {
    return ZSecNoKey;
  }

  ret = ccmStar( ed_flag, &stx, nonce, msg );

  // zero-out bits in security control field
  *(msg+hdrLen) &= ~SEC_MASK;

  // clear copy of key in RAM before return
  osal_memset( tmpKey.key, 0x00, SEC_KEY_LEN );

  return( ret );
}

/*********************************************************************
 * @fn      ccmStar
 *
 * @brief   processing for the ccm* protocol
 *
 * @param   void
 *
 * @return  void
 */
ZStatus_t ccmStar( uint8 ed_flag, ssp_ctx *stx,
                   uint8 *nonce, uint8 *secMsg )
{
  ZStatus_t ret;
  uint8 micLen;
  uint8 authLen;
  uint8 encLen;
  uint8 i;
  uint8 *keyptr, *micptr;
  uint8 cstate[16];
  bool decrypt = false;
  bool encrypt = false;
  HAL_AES_ENTER_WORKAROUND();

  keyptr = stx->key;
  ssp_HW_KeyInit( keyptr );
  pSspAesEncrypt = sspAesEncryptHW;

  authLen = stx->hdrLen + stx->auxLen;
  if (stx->secLevel < 4) // check for Auth Only
  {
    authLen += stx->msgLen; // a = Hdr+Aux+Msg
    encLen = 0; // m = 0
  }
  else // auth and encrypt
  {    // a = Hdr+Aux
    encLen = stx->msgLen; // m = Msg
  }

  micLen = SSP_GetMicLen (stx->secLevel);
  micptr = secMsg + authLen + encLen;

  if (ed_flag == SSP_APPLY) // Apply Auth & Encryption
  {
    // Apply encryption only if needed
    if (stx->secLevel >= 4)
    {
      encrypt = true;
    }

    // Assume secMsg[] is sized large enough to insert MIC-T or encrypted MIC-U
    // after payload, but cannot assume that we have enough space (16 bytes)
    // to use it as Cstate buffer. We pass ccmLVal = 2
    ret = SSP_CCM_Auth_Encrypt (encrypt, micLen, nonce, secMsg+authLen, encLen, secMsg,
                                authLen, keyptr, cstate, 2);
    if (ret == ZSuccess)
    {
      // Append MIC to end of message
      for (i=0; i < micLen; i++)
      {
        micptr[i] = cstate[i];
      }
    }
  }
  else // Apply Decryption and Check Auth
  {
    // Apply decryption only if needed
    if (stx->secLevel >= 4)
    {
      decrypt = true;
    }

    ret = SSP_CCM_InvAuth_Decrypt (decrypt, micLen, nonce, secMsg+authLen,
                                   (uint16)(encLen+micLen), secMsg, authLen, keyptr, cstate, 2);
  }

  HAL_AES_EXIT_WORKAROUND();
  return ret;

} /* ccmStar() */

/*********************************************************************
 * @fn      SSP_NwkSecurity
 *
 * @brief   secure/unsecure the nwk layer packet
 *
 * @param   uint8 ed_flag - action flag
 *          uint8 *msg - message to process
 *          uint8 hdrLen - header length
 *          uint8 nsduLen - nsdu length
 *
 * @return  ZStatus_t
 */
ZStatus_t SSP_NwkSecurity(uint8 ed_flag, uint8 *msg, uint8 hdrLen, uint8 nsduLen)
{
  if ( SSP_NwkSecurityHook )
  {
    return SSP_NwkSecurityHook(ed_flag, msg, hdrLen, nsduLen);
  }
  else
  {
    return ZFailure;
  }
}

/*********************************************************************
 * @fn      SSP_GetMicLen
 *
 * @brief   This function returns length of mic string
 *
 * @param
 *
 * @return  void
 */
uint8 SSP_GetMicLen( uint8 securityLevel )
{
  if ( SSP_GetMicLenHook )
  {
    return SSP_GetMicLenHook( securityLevel );
  }
  else
  {
    return 0;
  }
}

/*********************************************************************
*********************************************************************/
