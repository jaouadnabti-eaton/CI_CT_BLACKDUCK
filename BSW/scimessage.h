/****************************************************************************************************
*  File name: scimessage.h
*
*  Purpose: Message definition data types
*  This file describes the data types for the Serial Communications message data types.
*
*  Copyright Notice:
*  All source code and data contained in this file is Proprietary and
*  Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
*  disclosed; in whole or in part, without the express written permission of
*  Eaton Aerospace.
*
*  Copyright (c) 2022 - Eaton Aerospace Group, All Rights Reserved.
*
*  Author            Date       CR#          Description
*  ------           ---------   ------     ---------------------------------------
*  Adam Bouwens     12/07/2022  N/A        Ported to MCU
*
****************************************************************************************************/

#ifndef SCIMESSAGE_H__
#define SCIMESSAGE_H__

/*      Include Files
*/
//#include "dsp281x_regs.h"

/*      Public Type Definitions
*/
/*    brief Enumerated status values for SCI operations
 */
typedef enum
{
    TX_DONE,        /* status indicating transmission of a message is complete (i.e. data link escape followed by end of transmission sent)*/
    TX_IN_PROGRESS, /*status indicating transmission of a message is in progress*/
    RX_IN_PROGRESS, /* status indicating a message is currently being received*/
    VALID_RX_MSG,   /* status indicating a message has been received, (i.e. data link escape followed by end of transmission)*/
    INVALID_RX_MSG  /* status indicating a message has been received but incorrectly formatted */
} SCI_STATUS;

/*
    brief Bitfield for FAS LRU status
 */
struct tFasLruStatus
{
    uint16_t bMcuFail:1;          /* MCU fail */
    uint16_t bScuFail:1;          /* SCU fail */
    uint16_t bFlaFail:1;          /* FLA fail */
    uint16_t bFpsuFail:1;         /* FPSU fail */
    uint16_t bMotorFail:1;        /* Motor fail */
    uint16_t bFlapJam:1;          /* Flap jam */
    uint16_t available:10;        /* Available */
};

/*
    brief Bitfield for Motor current reporting
 */
struct motor_current
{
    Uint16 tenths:4;    /* tenths of an ampere */
    Uint16 amps:4;      /* amperes */
    Uint16 reserved:8;  /* reserved */
};

/*
    brief  FAS_LRUStatus is for sending fault notification bits
 */
typedef union
{
    uint16_t                all;
    struct tFasLruStatus    bit;
} tFasLruStatus_t;


/*
    brief  MOTOR_CURRENT is for sending motor current
 */
typedef union
{
    Uint16               all;
    struct motor_current bit;
} MOTOR_CURRENT;

struct tMotorCurrents
{
    uint16_t o_tenths:4;    /* onside tenths of an ampere */
    uint16_t o_amps:4;      /* onside amperes */
    uint16_t x_tenths:4;    /* xside tenths of an ampere */
    uint16_t x_amps:4;      /* xside amperes */
};

/* Motor Current structure as a full 16-bit word for DFT */
typedef union
{
    uint16_t all;
    struct tMotorCurrents bit;
} tMotorCurrents_t;

/*      Public Variable Declarations
*/

/*      Public ROM Constants
*/

/*      Public Defines
*/
#define DLE_CHAR  0x10   /* data link escape character */
#define EOTX_CHAR  0x03  /* end of transmission character */

#define LEFT_CHAN_ID  0x01  /* value representing the left channel for the MCU Channel ID data word */
#define RIGHT_CHAN_ID 0x02  /* value representing the right channel for the MCU Channel ID data word */
#define BOTH_CHANNELS 0x04  /* value representing both channels for the MCU Channel ID data word */

#define MAX_FIFO_COUNT 16 /*maximum capacity of the FIFO */
#define MAX_STUFFED_FIFO_COUNT 14  /* maximum number of bytes the FIFO can contain and still accept a potentially stuffed data word */

/*      Public Interface Function Prototypes
*/

#endif

/* end scimessage.h */


