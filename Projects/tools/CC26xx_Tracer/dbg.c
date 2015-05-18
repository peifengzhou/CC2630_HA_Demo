//------------------------------------------------------------------------------
// TI Confidential - NDA Restrictions
//
// Copyright (c) 2011 Texas Instruments, Inc.
//
//    This is an unpublished work created in the year stated above.
//    Texas Instruments owns all rights in and to this work and
//    intends to maintain and protect it as an unpublished copyright.
//    In the event of either inadvertent or deliberate publication,
//    the above stated date shall be treated as the year of first
//    publication. In the event of such publication, Texas Instruments
//    intends to enforce its rights in the work under the copyright
//    laws as a published work.
//
//------------------------------------------------------------------------------
///
/// \file            dbg.c
/// \brief           Handles debug trace operations
///
/// \author          Low Power RF Wireless Business Unit
///                  Helge Coward (h.coward@ti.com)
///
/// \date            Tue Nov 15 10:20:07 CET 2011
///
//-----------------------------------------------------------------------------

/// \addtogroup module_dbg
//@{
#include "dbg.h"
#include "rfctrc_regs.h"
//#include "interrupt.h"
#include <stdint.h>
#include "cpu.h"

#ifndef DBG_GLOBAL_DISABLE

typedef char cs_t;

#define ENTER_CRITICAL_SECTION(x)                                              \
  do { (x) = !CPUcpsid(); } while (0)

#define EXIT_CRITICAL_SECTION(x)                                               \
  do { if (x) { (void) CPUcpsie(); } } while (0)

#define __enable_irq()    asm("cpsie i");
#define __disable_irq()   asm("cpsid i");

/** \brief Function used to implement DBG_PRINTn
 *
 * Waits for the LVDS channel to be ready before storing the printf() parameters and transmitting the
 * LVDS packet.
 *
 * \param[in]  x
 *     Contains the following values:
 *     - [17:16] The channel number to use (1, 2 or 3)
 *     - [15:8] Trace packet header byte
 *     - [2:0] Number of 16-bit printf() arguments, 0-4
 * \param[in]  y
 *     Contains printf() argument 0 in LSW and argument 1 in MSW
 * \param[in]  z
 *     Contains printf() argument 2 in LSW and argument 3 in MSW
 */
void dbgPrintf(uint32_t x, uint32_t y, uint32_t z)
{
   int  ch = (x >> 16) - 1;
   cs_t cs;

   // Wait for previous packet to be transmitted
   while (1) {
      //__disable_irq();
      ENTER_CRITICAL_SECTION(cs);
      if ((SP_TRCCH1CMD[ch] & TRCCH1CMD_CH1PKTHDR_BM) == 0) {
         // Channel ready, transmit packet
         SP_TRCCH1PAR01[ch] = y;
         SP_TRCCH1PAR23[ch] = z;
         SP_TRCCH1CMD[ch] = x;
         //__enable_irq();
         EXIT_CRITICAL_SECTION(cs);
         break;
      }
      else {
         //__enable_irq();
         EXIT_CRITICAL_SECTION(cs);
      }
   }
} // dbgPrintf



/** \brief Function used to implement DBG_PRINTn, n=1,2
 *
 * Waits for the LVDS channel to be ready before storing the printf() parameters and transmitting the
 * LVDS packet.
 *
 * \param[in]  x
 *     Contains the following values:
 *     - [17:16] The channel number to use (1, 2 or 3)
 *     - [15:8] Trace packet header byte
 *     - [2:0] Number of 16-bit printf() arguments, 0-2
 * \param[in]  y
 *     Contains printf() argument 0 in LSW and argument 1 in MSW
 */
void dbgPrintf2(uint32_t x, uint32_t y)
{
   int  ch = (x >> 16) - 1;
   cs_t cs;

   // Wait for previous packet to be transmitted
   while (1) {
      //__disable_irq();
      ENTER_CRITICAL_SECTION(cs);
      if ((SP_TRCCH1CMD[ch] & TRCCH1CMD_CH1PKTHDR_BM) == 0) {
         // Channel ready, transmit packet
         SP_TRCCH1PAR01[ch] = y;
         SP_TRCCH1CMD[ch] = x;
         //__enable_irq();
         EXIT_CRITICAL_SECTION(cs);
         break;
      }
      else {
         //__enable_irq();
         EXIT_CRITICAL_SECTION(cs);
      }
   }
} // dbgPrintf2



/** \brief Function used to implement DBG_PRINT0
 *
 * Waits for the LVDS channel to be ready before storing the printf() parameters and transmitting the
 * LVDS packet.
 *
 * \param[in]  x
 *     Contains the following values:
 *     - [17:16] The channel number to use (1, 2 or 3)
 *     - [15:8] Trace packet header byte
 *     - [2:0] Number of 16-bit printf() arguments, 0
 */
void dbgPrintf0(uint32_t x)
{
   int  ch = (x >> 16) - 1;
   cs_t cs;

   // Wait for previous packet to be transmitted
   while (1) {
      //__disable_irq();
      ENTER_CRITICAL_SECTION(cs);
      if ((SP_TRCCH1CMD[ch] & TRCCH1CMD_CH1PKTHDR_BM) == 0) {
         // Channel ready, transmit packet
         SP_TRCCH1CMD[ch] = x;
         //__enable_irq();
         EXIT_CRITICAL_SECTION(cs);
         break;
      }
      else {
         //__enable_irq();
         EXIT_CRITICAL_SECTION(cs);
      }
   }
} // dbgPrintf0
#endif // !DBG_GLOBAL_DISABLE


//@}
