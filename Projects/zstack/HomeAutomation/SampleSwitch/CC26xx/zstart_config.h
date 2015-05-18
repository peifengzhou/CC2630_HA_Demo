/**************************************************************************************************
 Filename:       zstart_config.h
 Revised:        $Date: 2015-01-20 08:50:11 -0800 (Tue, 20 Jan 2015) $
 Revision:       $Revision: 41917 $

 Description:    This file contains the manual start configuration
 settings.


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
 **************************************************************************************************/

#ifndef ZSTART_CONFIG_H
#define ZSTART_CONFIG_H

#if !defined( ZSTART_SCAN_DURATION )
// Scan duration (Beacon Order) values - per channel in ZSTART_DEFAULT_CHANLIST
//#define ZSTART_SCAN_DURATION      14  // 8 minutes
//#define ZSTART_SCAN_DURATION      13  // 4 minutes
//#define ZSTART_SCAN_DURATION      12  // 2 minutes
//#define ZSTART_SCAN_DURATION      11  // 62 seconds
//#define ZSTART_SCAN_DURATION      10  // 30 seconds
//#define ZSTART_SCAN_DURATION       9  // 15 seconds
//#define ZSTART_SCAN_DURATION       8  // 8 seconds
//#define ZSTART_SCAN_DURATION       7  // 4 seconds
//#define ZSTART_SCAN_DURATION       6  // 2 seconds
//#define ZSTART_SCAN_DURATION       5  // 960 milliseconds
//#define ZSTART_SCAN_DURATION       4  // 480 milliseconds
//#define ZSTART_SCAN_DURATION       3  // 240 milliseconds
#define ZSTART_SCAN_DURATION       2  // 120 milliseconds
//#define ZSTART_SCAN_DURATION       1  // 60 milliseconds
//#define ZSTART_SCAN_DURATION       0  // 30 milliseconds
#endif // ZSTART_SCAN_DURATION

/* Define the default time between scans.
 *
 * This values represents the time to wait between scans in 100 millisecond
 * increments.  A value of 0 says that there is no wait.
 * Range: 0 - 0xFFFF
 */
#if !defined( ZSTART_SCAN_OFF_PERIOD )
#define ZSTART_SCAN_OFF_PERIOD 0
#endif // ZSTART_SCAN_OFF_PERIOD

/* Define the default number of scans to perform before joining/rejoining.
 *
 * Range: 1 - 256
 */
#if !defined( ZSTART_SCAN_ATTEMPTS )
#define ZSTART_SCAN_ATTEMPTS  2
#endif // ZSTART_SCAN_ATTEMPTS

#endif // ZSTART_CONFIG_H
