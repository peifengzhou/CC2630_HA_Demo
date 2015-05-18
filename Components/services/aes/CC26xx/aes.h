#ifndef     _AES_H_
#define     _AES_H_

#define     STATE_BLENGTH   16      // Number of bytes in State
#define     KEY_BLENGTH     16      // Number of bytes in Key
#define     KEY_EXP_LENGTH  176     // Nb * (Nr+1) * 4

//
// Key Functions
//
extern void ssp_KeyInit_Sw( uint8_t * );
extern void sspKeyExpansion_Sw( uint8_t *, uint8_t * );
extern void RoundKey_Sw( uint8_t *, uint8_t );

//
// Encryption Functions
//
extern void sspAesEncryptKeyExp_Sw( uint8_t *, uint8_t * );
extern void sspAesEncryptBasic_Sw( uint8_t *, uint8_t * );

extern void AddRoundKeySubBytes_Sw( uint8_t *, uint8_t * );
extern void ShiftRows_Sw( uint8_t * );
extern void MixColumns_Sw( uint8_t * );

extern void (*pSspAesEncrypt_Sw)( uint8_t *, uint8_t * );
extern void sspKeyExpansion_Sw (uint8_t *, uint8_t *);

extern void sspAesEncrypt_Sw (uint8_t *, uint8_t *);
extern void sspAesEncryptKeyExp_Sw (uint8_t *, uint8_t *);
extern void sspAesEncryptBasic_Sw (uint8_t *, uint8_t *);

//
// Decryption Functions
//
#ifdef INCLUDE_AES_DECRYPT
  extern uint8_t FFMult_Sw( uint8_t, uint8_t );
  extern void sspAesDecrypt_Sw( uint8_t *, uint8_t * );
  extern void InvSubBytes_Sw( uint8_t * );
  extern void InvShiftRows_Sw( uint8_t * );
  extern void InvMixColumns_Sw( uint8_t * );
#endif  // INCLUDE_AES_DECRYPT

#ifdef INCLUDE_AES_DECRYPT
#ifdef USE_KEY_EXPANSION
  extern void InvAddRoundKey_Sw( uint8_t, uint8_t *, uint8_t * );
#else
  extern void InvAddRoundKey_Sw( uint8_t, uint8_t *, uint8_t *, uint8_t * );
  extern void InvRoundKey_Sw( uint8_t *, uint8_t, uint8_t * );
#endif  // USE_KEY_EXPANSION
#endif  // INCLUDE_AES_DECRYPT
  

// Following are definitions needed for CC2430 hardware AES engine


#define AES_BUSY    0x08
#define ENCRYPT     0x00
#define DECRYPT     0x01

// Macro for setting the mode of the AES operation
#define AES_SETMODE(mode) do { ENCCS &= ~0x70; ENCCS |= mode; } while (0)

// _mode_ is one of
#define CBC         0x00
#define CFB         0x10
#define OFB         0x20
#define CTR         0x30
#define ECB         0x40
#define CBC_MAC     0x50

// Macro for starting or stopping encryption or decryption
#define AES_SET_ENCR_DECR_KEY_IV(mode) \
   do {                                \
    ENCCS = (ENCCS & ~0x07) | mode     \
   } while(0)

// Where _mode_ is one of
#define AES_ENCRYPT     0x00;
#define AES_DECRYPT     0x02;
#define AES_LOAD_KEY    0x04;
#define AES_LOAD_IV     0x06;

// Macro for starting the AES module for either encryption, decryption,
// key or initialisation vector loading.
#define AES_START()     ENCCS |= 0x01

// End of CC2430 hardware AES engine definitions

#endif  // _AES_H_

