/****************************************************************************************************
*  File name: A825Mgr.h
*
*  Purpose : Header for A825Mgr.c
*
*  Copyright Notice:
*  All source code and data contained in this file is Proprietary and
*  Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
*  disclosed; in whole or in part, without the express written permission of
*  Eaton Aerospace.
*
*  Copyright (c) 2022 - Eaton Aerospace Group, All Rights Reserved.
*
*  Author            Date       CR#              Description
*  ------          ---------    ------        -------------------------------------
*  Adam Bouwens     12/07/2022  NA             Port to MCU
****************************************************************************************************/
#ifndef A825MGR_H_
#define A825MGR_H_

/******************** INCLUDES ********************/
#include "parameter.h"
#include "A825_MsgInfo.h"
#include "A825Drvr.h"
#include "stdbool.h"
#include "F2837xD_device.h"

/******************** DEFINES ********************/
#define ARINC825_CNTRL_BUS                  (CAN_CHANNEL_A)
#define ARINC825_MAINT_BUS                  (CAN_CHANNEL_B)
#define A825_SRVC_TIME_MILLISEC             (0.125F)    /* Call rate of A825 service in ms */
#define A825_SRVC_COUNTER_INIT              ((uint32_t) (A825_SRVC_TIME_MILLISEC * 8000.0F)) /* Init to start transmission immediately (1sec message is longest period of TX) */
#define ROUND_UP_LIMIT                      (0.5F)      /* Used for rounding in calculations */
#define MSG_NOT_RECVD_ONE_SEC_CNT           ((uint16_t)NUM_MINOR_FRAME_PER_MAJOR)
#define TX_COUNTER_INC_DURATION_MILLISEC    (NORMAL_MINOR_FRAME_TIME_MILLISEC) //Func duration counter
#define A825_MSG_ID_CHECK_FILTER            (0x1FFFF0FCU) /* RCI and DSTID masked out */

/* 10% is max tolerance of SYNC period that messages will be aligned to. 10ms * 1.10 = 10.1ms =
 * 11.0 * 8 = 88 1/8th millisecond counts (A825_SRVC_TIME_MILLISEC). Rationale:  SYNC period should not be outside
 * of a 10% tolerance. The timeslot alignment and adjustment is designed to occur when the SYNC controller comes
 * online AND to address possible drift of CPU timing over long periods of time. The 10% will cover this along with a test
 * for Stale messages which is the case */
/* ONE TIMESLOT = 0.5ms */
#define A825_ALIGN_TIMESLOT_TO_SCU_MSG_PERIOD_MAX      (88UL)   /* A825_COUNTS_PER_SCH_A_PERIOD * 1.1 */
#define A825_ALIGN_TIMESLOT_TO_SCU_MSG_PERIOD_MIN      (72UL)   /* A825_COUNTS_PER_SCH_A_PERIOD * 0.9 */
#define A825_ALIGN_TIMESLOT_IDEAL_TOLERANCE             (2UL)    /* 2, 1/8th ms Counts is 0.25ms */
#define A825_COUNTS_PER_PERIOD                         (80UL)   /* Schedule A period is 10ms. Service is at a 1/8th ms schedule (A825_SRVC_TIME_MILLISEC). 10*8 = 80 */
#define A825_COUNTS_PER_TIMESLOT                        (4UL)   /* Number of 1/8th ms counts in one timeslot (timeslot = 0.5ms) */

/* ARINC825 Manager Types */
typedef enum
{
   A825_TX_SCHEDULE_A,   /* Schedule A for Tx Transmissions (Highest Rate Schedule) */
   A825_TX_SCHEDULE_B,    /* Schedule B for Tx Transmissions (Lowest Rate Schedule) */
   A825_NUM_TX_SCHEDULES /* Number of Schedules */
} A825_TX_SCHEDULE_FREQ;    /* Source of ARINC 825 message */

/* ARINC825 Single Message Definition */
typedef struct
{
   const uint16_t lookUp_Id;
   const MSGID_REG_MGR MsgId;
   const uint16_t num_signals;
   const uint16_t data_length_in_bytes;
   const Rate_t tRate;
   const Signal_Info* const ptr_signalInfoTable;
} A825TableEntry;

typedef struct
{
    const A825TableEntry tMsg;
    const A825_TX_SCHEDULE_FREQ tSchedule;
    t64Types_t tData;                        /* Data (8 byte data field) associated with the A825 Message */
    uint32_t u32LastCount;                   /* Last count associated with Transmission or Reception of Message */
} A825TxTableEntry;

typedef struct
{
    const A825TableEntry tMsg;
    t64Types_t tData;                        /* Data (8 byte data field) associated with the A825 Message */
    uint32_t u32LastCount;                   /* Last count associated with Transmission or Reception of Message */
    bool_t bMsgStaleStatus;                  /* Message stale-status */
} A825RxTableEntry;

typedef struct
{
    const uint16_t u16NumMsgs;
    A825TxTableEntry* const ptMsgList;
    A825_TX_SCHEDULE_FREQ tSchedule;
    uint16_t  u16TxMsgIdx;
    uint16_t  u16TxMsgIdx_A;
    uint16_t  u16TxMsgIdx_B;
} A825_Bus_Tx_t;

typedef struct
{
    const uint16_t u16NumMsgs;
    A825RxTableEntry* const ptMsgList;
} A825_Bus_Rx_t;

typedef struct
{
    A825_BUS_TYPE_T tBusType;               /* A825 Bus Type */
    A825_Bus_Tx_t   tTx;
    A825_Bus_Rx_t   tRx;
    uint32_t  u32A825ServiceCounter;        /* Init to start transmission immediately (1sec message is longest period of TX) */
    uint32_t  u32PositionCmdCntPrev;        /* Copy of the counter of the Last Received Command used to adjust timeslot for synchronization */
    uint32_t  u32EighthMillisecondTarget;   /* */
    bool_t    bCommsEstablished;            /* Flag to indicate ARINC825 Commands have been received on this channel */
    A825Drvr_Data_t* const ptDrvrData;      /* Handle for the tA825Drvr_Data for the channel */
} A825_Bus_t;

/******************** FUNCTION PROTOTYPES ********************/
void A825Mgr_SetSignal(A825_Bus_Tx_t* const ptA825Bus, const uint16_t lookup_id, const SIGNAL_ENTRY_INDEX signal_index, ...);
bool A825Mgr_IsRxMsgStale(A825_Bus_Rx_t* const ptA825Rx, const uint16_t msg_id);
ERRORCODES A825Mgr_Init( void );
void A825Mgr_TxSrvc(A825_Bus_t* const ptA825Bus);
void A825Mgr_RxSrvc(A825_Bus_t* const ptA825Bus);
void A825Mgr_GetSignal_BNR(A825_Bus_Rx_t* const ptA825Rx, const uint16_t lookup_id, const SIGNAL_ENTRY_INDEX signal_index, float32_t *const ptr_val);
void A825Mgr_GetSignal_BOOL(A825_Bus_Rx_t* const ptA825Rx, const uint16_t lookup_id, const SIGNAL_ENTRY_INDEX signal_index, bool_t *const ptr_is_set);
void A825Mgr_GetSignal_OPAQUE(A825_Bus_Rx_t* const ptA825Rx, const uint16_t lookup_id, const SIGNAL_ENTRY_INDEX signal_index, t64Types_t *const pt64Val);

extern A825_Bus_t tA825_Bus[NUM_CAN_CHANNELS];    /* ARINC825 BUS (Primary/A and Secondary/B) Array  */
extern A825TxTableEntry tA825CntrlBusTxMsgList[A825_CNTRL_BUS_TX_NUM_MSGS]; /* ARINC825 CONTROL BUS TRANSMIT MESSAGE LIST */
extern A825RxTableEntry tA825CntrlBusRxMsgList[A825_CNTRL_BUS_RX_NUM_MSGS]; /* ARINC825 CONTROL BUS RECEIVE MESSAGE LIST */
extern A825TxTableEntry tA825MaintBusTxMsgList[A825_MAINT_BUS_TX_NUM_MSGS]; /* ARINC825 MAINTENANCE BUS TRANSMIT MESSAGE LIST */
extern A825RxTableEntry tA825MaintBusRxMsgList[A825_MAINT_BUS_RX_NUM_MSGS]; /* ARINC825 MAINTENANCE BUS RECEIVE MESSAGE LIST */

#endif /*A825MGR_H_*/
