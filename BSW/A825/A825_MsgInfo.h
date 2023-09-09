/******************************************************************************
*  Copyright (c) 2020 Eaton Corporation
*
*  Purpose: Types and definitions to support ARINC825 Communication
*
*
*  Copyright Notice:
*  All source code and data contained in this file is Proprietary and
*  Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
*  disclosed; in whole or in part, without the express written permission of
*  Eaton Aerospace.
*
*  Copyright (c) 2022 - Eaton Aerospace Group, All Rights Reserved.
*
 * Author            Date       CR#              Description
 * ------          ---------    ------        -------------------------------------
 * Adam Bouwens     12/07/2022  NA             Port to MCU
******************************************************************************/
#ifndef A825_MSGINFO_H
#define A825_MSGINFO_H
#include "typedefs.h"
#include "hw_types.h"
#include "F2837xD_device.h"

/* Definition of Type of Bus. */
typedef enum
{
    A825_CONTROL_BUS = 0,
    A825_MAINTENANCE_BUS,
    NUM_A825_BUS_TYPES
} A825_BUS_TYPE_T;

/*Justification for Coding Standard Deviation:
 *    Unions are required for efficient data packing.  MISRA documents this as an
 *    acceptable deviation.
 *    An exception to MISRA Rule 18.4 is required.
 */
/* ARINC825 ANYONE TO MANY (GENERIC DEFINITION) MESSAGE ID TYPE */
typedef struct
{                          /*  Bits  Description                     */
    uint16_t RCI     : 2;  /*  0: 1  Redundancy Channel Identifier   */
    uint16_t DOC     : 14; /*  2:15  Data Object Code                */
    uint16_t PVT     : 1;  /*    16  Private Status                  */
    uint16_t LCL     : 1;  /*    17  Local Status                    */
    uint16_t RSD     : 1;  /*    18  Reserved                        */
    uint16_t SRCFID  : 7;  /* 19:25  Source Function Code Identifier */
    uint16_t LCC     : 3;  /* 26:28  Logical Communication Channel   */
    uint16_t NotUsed : 3;  /* 29:31  Not Used                        */
} MSGID_A825_A2M_MGR;

/* Allow access to the bit fields or entire register */

/*Justification for Coding Standard Deviation:
 *    Unions are required for efficient data packing.  MISRA documents this as an
 *    acceptable deviation.
 *    An exception to MISRA Rule 18.4 is required.
 */
/* ARINC825 ONE TO MANY MESSAGE ID TYPE */
typedef struct
{                         /*  Bits  Description                      */
    uint16_t RCI     : 1; /*     0  Redundancy Channel Identifier    */
    uint16_t ACTID   : 1; /*     1  Actuator Identifier (Left/Right) */
    uint16_t MSGID   : 6; /*  2: 7  Message ID                       */
    uint16_t DSTID   : 4; /*  8:11  Destination LRU Identification   */
    uint16_t SRCID   : 4; /* 12:15  Source LRU Identification        */
    uint16_t PVT     : 1; /*    16  Private Status                   */
    uint16_t LCL     : 1; /*    17  Local Status                     */
    uint16_t RSD     : 1; /*    18  Reserved                         */
    uint16_t SRCFID  : 7; /* 19:25  Source Function Code Identifier  */
    uint16_t LCC     : 3; /* 26:28  Logical Communication Channel    */
    uint16_t NotUsed : 3; /* 29:31  Not Used                         */
} MSGID_A825_O2M_MGR;

/*Justification for Coding Standard Deviation:
 *    Unions are required for efficient data packing.  MISRA documents this as an
 *    acceptable deviation.
 *    An exception to MISRA Rule 18.4 is required.
 */
/* ARINC825 PEER TO PEER MESSAGE ID TYPE */
typedef struct
{                         /*  Bits  Description                     */
    uint16_t RCI     : 2; /*  0: 1  Redundancy Channel Identifier   */
    uint16_t SID     : 7; /*  2: 8  SID                             */
    uint16_t DFID    : 7; /*  9:15  Destination ID                  */
    uint16_t PVT     : 1; /*    16  Private Status                  */
    uint16_t LCL     : 1; /*    17  Local Status                    */
    uint16_t RSD     : 1; /*    18  Reserved                        */
    uint16_t SRCFID  : 7; /* 19:25  Source Function Code Identifier */
    uint16_t LCC     : 3; /* 26:28  Logical Communication Channel   */
    uint16_t NotUsed : 3; /* 29:31  Not Used                        */
} MSGID_A825_P2P_MGR;

/*Justification for Coding Standard Deviation:
 *    Unions are required for efficient data packing.  MISRA documents this as an
 *    acceptable deviation.
 *    An exception to MISRA Rule 18.4 is required.
 */
/* ARINC825 MESSAGE ID TYPES UNION */
typedef union
{
    uint32_t           all;
    MSGID_A825_A2M_MGR a825_a2m;
    MSGID_A825_O2M_MGR a825_o2m;
    MSGID_A825_P2P_MGR a825_p2p;
} MSGID_REG_MGR;

typedef enum
{
    LCC_EEC = 0,
    LCC_NOC = 2,
    LCC_NSC = 4,
    LCC_UDC = 5,
    LCC_TMC = 6,
    LCC_FMC = 7
} eA825Lcc_t;

typedef enum
{
   ONESHOT,     // Send ARINC 825 message exactly once
   CONTINUOUS   // Send ARINC 825 message continuously per scheduled rate until stopped
} A825SNDMODE;  // ARINC 825 message transmit send mode  //>>>SDD done

typedef enum
{
   SIGNAL_NODATA = 0,
   SIGNAL_ENUM,
   SIGNAL_CHAR,
   SIGNAL_UCHAR,
   SIGNAL_ACHAR,
   SIGNAL_SHORT,
   SIGNAL_USHORT,
   SIGNAL_LONG,
   SIGNAL_ULONG,
   SIGNAL_LONG64,
   SIGNAL_ULONG64,
   SIGNAL_FLOAT,
   SIGNAL_DOUBLE,
   SIGNAL_OPAQUE,
   SIGNAL_BOOL,
   SIGNAL_BCD,
   SIGNAL_UBNR,
   SIGNAL_BNR,
   SIGNAL_NUM_SIGNALS
} SIGNAL_SUBFIELD_TYPE;

/* ARINC825 Control Bus Transmit Message Types */
typedef enum 
{
    A825_CNTRL_BUS_TX_MCU_PSN_STATUS_FLT_MAINT,
    A825_CNTRL_BUS_TX_MCU_SENSOR_DATA,
    A825_CNTRL_BUS_TX_NUM_MSGS
} eA825CntrlBusTxMsgIdList_t;

/* ARINC825 Maintenance Bus Transmit Message Types */
typedef enum
{
    A825_MAINT_BUS_TX_MCU_PSN_STATUS_FLT_MAINT,
    A825_MAINT_BUS_TX_MCU_SENSOR_DATA,
    A825_MAINT_BUS_TX_NUM_MSGS
} eA825MaintBusTxMsgIdList_t;

/* ARINC825 Control Bus Receive Message Types */
typedef enum
{
    A825_CNTRL_BUS_RX_FIRST_MSG,
    A825_CNTRL_BUS_RX_FLAP_CMD,
    A825_CNTRL_BUS_RX_NUM_MSGS
} eA825CntrlBusRxMsgIdList_t;

/* ARINC825 Maintenance Bus Receive Message Types */
typedef enum
{
    A825_MAINT_BUS_RX_FIRST_MSG,
    A825_MAINT_BUS_RX_RIG_CMD,
    A825_MAINT_BUS_RX_NUM_MSGS
} eA825MaintBusRxMsgIdList_t;

#define UNSIGNED_32BIT_MASK (0xFFFFFFFFUL)
#define UNSIGNED_16BIT_MASK (0xFFFFUL)
#define UNSIGNED_8BIT_MASK  (0xFFUL)
#define UNSIGNED_8BIT_ASCII_MASK (0x7FUL)
#define UNSIGNED_64BIT_MASK (0xFFFFFFFFFFFFFFFFULL)
#define LONG_LONG_SIZE (64U)
#define UNSIGNED_32BIT_WIDTH (32U)

typedef enum
{
   SIGNAL1,
   SIGNAL2,
   SIGNAL3,
   SIGNAL4,
   SIGNAL5,
   SIGNAL6,
   SIGNAL7,
   SIGNAL8,
   SIGNAL9,
   SIGNAL10,
   SIGNAL11,
   SIGNAL12,
   SIGNAL13,
   SIGNAL14,
   SIGNAL15,
   SIGNAL16,
   SIGNAL17,
   SIGNAL18,
   SIGNAL19,
   SIGNAL20,
   SIGNAL21,
   SIGNAL22,
   SIGNAL23,
   SIGNAL24,
   SIGNAL25,
   SIGNAL26,
   SIGNAL27,
   SIGNAL28,
   SIGNAL29,
   SIGNAL30,
   SIGNAL31,
   SIGNAL32,
   SIGNAL33,
   SIGNAL34,
   SIGNAL35,
   SIGNAL36,
   SIGNAL37,
   SIGNAL38,
   SIGNAL39,
   SIGNAL40,
   SIGNAL41,
   SIGNAL42,
   SIGNAL43,
   SIGNAL44,
   SIGNAL45,
   SIGNAL46,
   SIGNAL47,
   INVALID_SIGNAL
} SIGNAL_ENTRY_INDEX;

typedef struct
{
   SIGNAL_ENTRY_INDEX signalEntryIndex;
   uint16_t startPos;
   uint16_t sizeOfSignal;
   SIGNAL_SUBFIELD_TYPE signalType;
   float32_t f32Resolution;
   float32_t f32InverseResolution;
} Signal_Info; //signal-> subfield within the A825 message payload.

typedef union
{
    uint16_t tx_period_ms;
    uint16_t rx_stale_time_ms;
} Rate_t;

#endif
