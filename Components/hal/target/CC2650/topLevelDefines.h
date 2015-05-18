/**************************************************************************************************
  Filename:       topLevelDefines.h
  Revised:        $Date: 2010-02-25 15:22:29 -0800 (Thu, 25 Feb 2010) $
  Revision:       $Revision: 21800 $

  Description:    This file used to temporarily resolve all references from 8051.


  Copyright 2005-2010 Texas Instruments Incorporated. All rights reserved.

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

#ifndef TLD_H
#define TLD_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 * INCLUDES
 **************************************************************************************************/

/**************************************************************************************************
 * CONSTANTS
 **************************************************************************************************/
  
#define RSSI *((char *)0x0000)
#define RFD *((char *)0x0000)
#define RFIRQM1 *((char *)0x0000)
#define RFST *((char *)0x0000)
#define MDMCTRL0 *((char *)0x0000)
#define RFIRQF1 *((char *)0x0000)
#define BSP_W *((char *)0x0000)
#define FREQTUNE *((char *)0x0000)
#define PMUX *((char *)0x0000)
#define APCFG *((char *)0x0000)
#define P0SEL *((char *)0x0000)
#define P0DIR *((char *)0x0000)
#define P0IEN *((char *)0x0000)
#define OBSSEL0 *((char *)0x0000)
#define OBSSEL1 *((char *)0x0000)
#define OBSSEL2 *((char *)0x0000)
#define OBSSEL3 *((char *)0x0000)
#define OBSSEL4 *((char *)0x0000)
#define OBSSEL5 *((char *)0x0000)
#define P1SEL *((char *)0x0000)
#define P1DIR *((char *)0x0000)
#define P1IEN *((char *)0x0000)
#define CLKCONCMD *((char *)0x0000)
#define P2SEL *((char *)0x0000)
#define P2DIR *((char *)0x0000)
#define P2IEN *((char *)0x0000)
#define CLKCONCMD_16MHZ *((char *)0x0000)
#define CLKCONCMD_32MHZ *((char *)0x0000)
#define CLKCONSTA *((char *)0x0000)
#define P0 *((char *)0x0000)
#define P1 *((char *)0x0000)
#define P2 *((char *)0x0000)
#define MDMCTRL1 *((char *)0x0000)
#define FRMCTRL0 *((char *)0x0000)
#define TXFILTCFG *((char *)0x0000)
#define TXCTRL *((char *)0x0000)
#define ACOMPOS *((char *)0x0000)
#define IEN0 *((char *)0x0000)
#define IEN1 *((char *)0x0000)
#define IEN2 *((char *)0x0000)
#define TCON *((char *)0x0000)
#define S1CON *((char *)0x0000)
#define IRCON *((char *)0x0000)
#define IP0 *((char *)0x0000)
#define IP1 *((char *)0x0000)
#define RFIRQM0 *((char *)0x0000)
#define RFIRQF0 *((char *)0x0000)
#define RFERRM *((char *)0x0000)
#define RFERRF *((char *)0x0000)
#define ACOMPQS *((char *)0x0000)
#define RXCTRL *((char *)0x0000)
#define FSCTRL *((char *)0x0000)
#define LNAGAIN *((char *)0x0000)
#define TXPOWER *((char *)0x0000)
#define T1CCTL0 *((char *)0x0000)
#define T1CC0L *((char *)0x0000)
#define T1CC0H *((char *)0x0000)
#define T1STAT *((char *)0x0000)
#define T1CTL *((char *)0x0000)
#define T1CNTL *((char *)0x0000)
#define T1CNTH *((char *)0x0000)
#define T1CNTH *((char *)0x0000)
#define RFTXFLEN *((char *)0x0000)
#define RFPSRND *((char *)0x0000)
#define ENCCS *((char *)0x0000)
#define T2IRQM *((char *)0x0000)
#define T2IRQF *((char *)0x0000)
#define T2CSPCFG *((char *)0x0000)
#define T2MSEL *((char *)0x0000)
#define T2M0 *((char *)0x0000)
#define T2M1 *((char *)0x0000)
#define T2MOVF0 *((char *)0x0000)
#define T2MOVF1 *((char *)0x0000)
#define T2MOVF2 *((char *)0x0000)
#define T2CTRL *((char *)0x0000)
#define ST0 *((char *)0x0000)
#define ST1 *((char *)0x0000)
#define ST2 *((char *)0x0000)
#define RFTXFSRP *((char *)0x0000)
#define RFTXFRP *((char *)0x0000)
#define RFFSTATUS *((char *)0x0000)
#define RFTXFRD *((char *)0x0000)
#define RFRXFLEN *((char *)0x0000)
#define RFRXFSWP *((char *)0x0000)
#define RFTXFSWP *((char *)0x0000)
#define RFRXFSRP *((char *)0x0000)
#define RFRXFRP *((char *)0x0000)
#define RFRXFRD *((char *)0x0000)
#define RFTXFWR *((char *)0x0000)
#define RFRXFWR *((char *)0x0000)
#define SW0 *((char *)0x0000)
#define SW1 *((char *)0x0000)
#define SW2 *((char *)0x0000)
#define SW3 *((char *)0x0000)
#define RFFCFG *((char *)0x0000)
#define RFFCFG *((char *)0x0000)
#define RFFCFG *((char *)0x0000)
#define RFFCFG *((char *)0x0000)
#define RFFCFG *((char *)0x0000)
//
#define RFCORE_RAM_PAGE_SZ 10
#define RFCORE_RAM_PAGE    10
  
/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
**************************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif // TLD.h
