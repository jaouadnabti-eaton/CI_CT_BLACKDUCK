/***************************************************************************************************
 * File name: A825Drvr.h
 *
 * Purpose : Data types and function prototypes for Low-Level Interface to CAN peripheral on
 *        TMS320F28379D Microcontroller
 *
 * Copyright Notice:
 * All source code and data contained in this file is Proprietary and
 * Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
 * disclosed; in whole or in part, without the express written permission of
 * Eaton Aerospace.
 *
 * Copyright (c) 2020 - Eaton Aerospace Group, All Rights Reserved.
 *
 * Author            Date       CR#              Description
 * ------          ---------    ------        -------------------------------------
 * Adam Bouwens     12/07/2022  NA             Port to MCU
 ****************************************************************************************************/

#ifndef A825DRVR_H_
#define A825DRVR_H_
//#include "sys_common.h"
//#include "ECan.h"
#include "A825_MsgInfo.h"
#include "stdbool.h"
#include "Gpio.h"
#include "F2837xD_can.h"

/* Mailbox Defines. */
/* Receive mailboxes start at Mailbox 1 and are consecutive to simplify routine for receiving
 * data service to a for loop starting at mailbox 1. */
#define A825_RX_NUMBER_OF_MAILBOXES   (2U)           /* Number of ARINC 825 receive mailboxes */
/* Mailbox 32 is used for Transmitting data.
 * Only a single Transmit mailbox is used in this design as the rate of Tx service is faster
 * than the rate at which new messages are added to the buffer to send. */
#define A825_TX_MAILBOX_NUMBER        (32U)          /* Mailbox number used for A825 Transmissions */

/*
 * Count of number of times to wait for CAN Transmit Acknowledge (TA) bit to be cleared before
 * declaring ARINC 825 message send failure.
 */
#define A825_XMIT_MAX_RETRY           (100U)
#define A825_WAIT_COUNTER             (100U)

/*
 * Used as a mask to derive the 29 bit ARINC 825 message ID
 */
#define MSGID_29BIT_MASK              (0x1FFFFFFFUL) /* Lowest 29 bits form the msgid */
#define MAX_A825DRVR_RCV_FIFO_SIZE    (10U)          /* Max number of unserviced ARINC 825 messages. Assumes service is 1ms and max baud rate of 500kbps. Shouldn't 6 ever (5.31 msgs with zero DLC). */
#define BYTE_SIZE               (8U)
#define BYTE0_MASK              (0x000000FFUL)
#define BYTE1_MASK              (0x0000FF00UL)
#define BYTE2_MASK              (0x00FF0000UL)
#define BYTE3_MASK              (0xFF000000UL)

/* DCAN BAUD Rate Calculation Defines */
#define CAN_MAX_BIT_DIVISOR     (13)   /* The maximum CAN bit timing divisor */
#define CAN_MIN_BIT_DIVISOR     (5)    /* The minimum CAN bit timing divisor */
#define CAN_MAX_PRE_DIVISOR     (1024) /* The maximum CAN pre-divisor */
#define CAN_MIN_PRE_DIVISOR     (1)    /* The minimum CAN pre-divisor */
#define CAN_BTR_BRP_M           (0x3F)
#define CAN_BTR_BRPE_M          (0xF0000)

typedef enum
{
    CAN_CHANNEL_A = 0,
    CAN_CHANNEL_B,
    NUM_CAN_CHANNELS
} CAN_CHANNEL_T;

typedef enum
{
    STAT_SUCCESS,
    FAILURE,            /* General failure */
    TIMEOUT,            /* CAN Peripheral Busy and Timed out */
    NULLPTR,            /* Null Pointer Error */
    TX_FAILURE,         /* Previous TX message failed to physically transmit to bus. */
    RX_NO_NEW_MESSAGE,  /* No new message to receive for CAN RX driver */
    INVALID_CAN_CHNL,   /* Invalid CAN Channel */
} ERRORCODES;

typedef enum
{
    INVALID_MBOX = 0,
    MBOX01, MBOX02, MBOX03, MBOX04, MBOX05, MBOX06, MBOX07, MBOX08,
    MBOX09, MBOX10, MBOX11, MBOX12, MBOX13, MBOX14, MBOX15, MBOX16,
    MBOX17, MBOX18, MBOX19, MBOX20, MBOX21, MBOX22, MBOX23, MBOX24,
    MBOX25, MBOX26, MBOX27, MBOX28, MBOX29, MBOX30, MBOX31, MBOX32,
    NUM_MBOXES = MBOX32
} CANMBOXESNUMBERS;

/* Mask definition for each Mailbox */

/* Mask 1:  Extended ID, Direction, LCC, SRCFID, RSD, LCL, PVT, SRCID, DSTID, MSGID, and RCI used for filtering.
 * CNTRL Bus:  RCI used for filtering and set here.
 * MAINT Bus:  RCI used for filtering and set here.
 * ACTID Field is not used for filtering.
 * Purpose is to capture only the A825_CNTRL_BUS_RX_FLAP_CMD (LCC = NOC, MSGID = 0) or A825_RX_MAINT_RIG_CMD (LCC = TMC, MSGID = 0) messages on the CNTRL and MAINT busses respectively.*/
#define CANMBOX01MASK    (0xDFFFFFFDUL)
/* Mask 2:  Extended ID, Direction, LCC, SRCFID, RSD, LCL, PVT, SRCID, DSTID, and RCI used for filtering.
 * CNTRL Bus:  SRCID is SCU. RCI is used for filtering and set here.
 * MAINT Bus:  SRCID is GSE. RCI is used for filtering and set here.
 * ACTID Field is not used for filtering.
 * MSGID Field is not used for filtering.
 * Purpose is to capture all MSGIDs intended for this LRU from the SCU or GSE depending upon the bus type CNTRL or MAINT respectively. */
#define CANMBOX02MASK    (0xDFFFFF01UL)
#define CANMBOX03MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX04MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX05MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX06MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX07MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX08MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX09MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX10MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX11MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX12MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX13MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX14MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX15MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX16MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX17MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX18MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX19MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX20MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX21MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX22MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX23MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX24MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX25MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX26MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX27MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX28MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX29MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX30MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX31MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */
#define CANMBOX32MASK    (0xC0000000UL) /* Extended ID and Direction used for filtering. Open ID */

/* Enable/Disable, ID type, Tx/Rx Type, and Filter for each Mailbox */
/* Mailbox 1:  Enable, Extended ID, Receive Type, Filter for A825_CNTRL_BUS_RX_FLAP_CMD (LCC = NOC, MSGID = 0) or A825_RX_MAINT_RIG_CMD (LCC = TMC, MSGID = 0) messages on the CNTRL and MAINT busses respectively.
 * SRCFID must be 10 for Flight Controls. PVT and LCL set to 1, and RSD set to 0.
 * CNTRL Bus:  SRCID must be  0 for SCU address. LCC set to NOC (2). (modified in driver setup)
 * MAINT Bus:  SRCID must be 15 for GSE address. LCC set to TMC (6). (modified in driver setup)
 * DSTID set to LRU address in driver setup.
 * RCI set to LRU Channel ID (A/B) in driver setup. */
#define CANMBOX01ARB     (0xC0530000UL)
/* Mailbox 2:  Enable, Extended ID, Receive Type, Filter for all messages of type O2M destined for this LRU on CNTRL or MAINT busses respectively:  MSGID is open per mask
 * SRCFID must be 10 for Flight Controls. PVT and LCL set to 1, and RSD set to 0.
 * CNTRL Bus:  SRCID must be  0 for SCU address. LCC set to NOC (2). (modified in driver setup)
 * MAINT Bus:  SRCID must be 15 for GSE address. LCC set to TMC (6). (modified in driver setup)
 * DSTID set to LRU address in driver setup.
 * RCI set to LRU Channel ID (A/B) in driver setup. (RCI open for MAINT bus per mask) */
#define CANMBOX02ARB     (0xC0530000UL)
#define CANMBOX03ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX04ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX05ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX06ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX07ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX08ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX09ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX10ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX11ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX12ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX13ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX14ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX15ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX16ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX17ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX18ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX19ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX20ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX21ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX22ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX23ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX24ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX25ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX26ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX27ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX28ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX29ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX30ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX31ARB     (0x00000000UL) /* Disable Mailbox */
#define CANMBOX32ARB     (0xE0000000UL) /* Enable Mailbox, Extended ID, Transmit Type, No Filter */

/* RCI field permits for 4 Channel Designations. This design uses bit 0 to designate between
 * Channel A and B ARINC825 Bus that is being communicated on while bit 1 is used to
 * designate between the Left and Right Actuator MCU that is communicating. The idea is that
 * bit 1 will not be used once the design goes from partially to fully distributed.
 * Channels A and B are the only valid Channels in this architecture.
 * NOTE:  Cannot exceed a value of 3 as this is limited to 2-bit value field in order
 * to save directly to the 2 bit field of the ID */
typedef enum
{
    RCI_CH_A = 0,   /* Channel A  */
    RCI_CH_B,       /* Channel B  */
    RCI_CH_C,       /* Channel C  */
    RCI_CH_D        /* Channel D  */
} REDUNDANCY_CHANNEL_IDENTIFIER;

/* Typedef structure for the bits of the CAN_ES_REG */
typedef struct
{
    uint16_t LEC:3;                       // 2:0 Last Error Code
    uint16_t TxOk:1;                      // 3 Transmission status
    uint16_t RxOk:1;                      // 4 Reception status
    uint16_t EPass:1;                     // 5 Error Passive State
    uint16_t EWarn:1;                     // 6 Warning State
    uint16_t BOff:1;                      // 7 Bus-Off State
    uint16_t PER:1;                       // 8 Parity Error Detected
    uint16_t rsvd1:1;                     // 9 Reserved
    uint16_t rsvd2:1;                     // 10 Reserved
    uint16_t rsvd3:5;                     // 15:11 Reserved
    uint32_t rsvd4:16;                    // 31:16 Reserved
} tCAN_ES_BITS_t;

/* typedef union for the CAN_ES_REG */
typedef union
{
    uint32_t  all;
    tCAN_ES_BITS_t  bit;
} tCAN_ES_REG_t;

/* DCAN IFX Data A Register word definitions */
typedef struct
{
    uint16_t u16_0;    /*  0:15 */
    uint16_t u16_1;    /* 31:16 */
} CAN_IFXDATA_WORDS;

/* DCAN IFX Data A Register byte definitions */
typedef struct
{
    uint16_t u8_0 : 8U;       /* DCAN DATA BYTE 0 */
    uint16_t u8_1 : 8U;       /* DCAN DATA BYTE 1 */
    uint16_t u8_2 : 8U;       /* DCAN DATA BYTE 2 */
    uint16_t u8_3 : 8U;       /* DCAN DATA BYTE 3 */
} CAN_IFXDATA_BYTES;

/*Justification for Coding Standard Deviation:
 *    Unions are required for efficient data packing.  MISRA documents this as an
 *    acceptable deviation.
 *    An exception to MISRA Rule 18.4 is required.
 */
/* DCAN IFX Data A Register type definition */
typedef union
{
    uint32_t     all;
    CAN_IFXDATA_WORDS word;
    CAN_IFXDATA_BYTES byte;
} CAN_IFXDATA;

/* DCAN IFX Data B Register word definitions */
typedef struct
{
    uint16_t u16_2;    /* 32:47 */
    uint16_t u16_3;    /* 48:63 */
} CAN_IFXDATB_WORDS;

/* DCAN IFX Data B Register byte definitions */
typedef struct
{
    uint16_t u8_4 : 8U;       /* DCAN DATA BYTE 4 */
    uint16_t u8_5 : 8U;       /* DCAN DATA BYTE 5 */
    uint16_t u8_6 : 8U;       /* DCAN DATA BYTE 6 */
    uint16_t u8_7 : 8U;       /* DCAN DATA BYTE 7 */
} CAN_IFXDATB_BYTES;

/*Justification for Coding Standard Deviation:
 *    Unions are required for efficient data packing.  MISRA documents this as an
 *    acceptable deviation.
 *    An exception to MISRA Rule 18.4 is required.
 */
/* DCAN IFX Data B Register type definition */
typedef union
{
    uint32_t     all;
    CAN_IFXDATB_WORDS word;
    CAN_IFXDATB_BYTES byte;
} CAN_IFXDATB;

/*Justification for Coding Standard Deviation:
 *    Unions are required for efficient data packing.  MISRA documents this as an
 *    acceptable deviation.
 *    An exception to MISRA Rule 18.4 is required.
 */
/* ARINC825 DCAN Message Manager Message Type Definition */
typedef struct
{
    /*lint -e{960} # intended as is */
    MSGID_REG_MGR   MSGID;      /* ARINC 825 message ID */
    CAN_IFXDATA     DATA_A;     /* DATA A Register:  CAN Bus message data bytes 0, 1, 2, 3 (lower data bytes) */
    CAN_IFXDATB     DATA_B;     /* DATA B Register:  CAN Bus message data bytes 4, 5, 6, 7 (upper data bytes) */
    uint16_t        DLC;        /* Data Length Code of CAN Bus message */
} MSG_MGR_t;             /* ARINC 825 message with contained data bytes */

/* ARINC825 Receive Message FIFO Element Type */
typedef struct
{
    MSG_MGR_t MSG[MAX_A825DRVR_RCV_FIFO_SIZE];  /* A825 message FIFO */
    uint16_t  fifo_avl;                         /* Available FIFO entries */
    uint16_t  fifo_in;                          /* A825 FIFO bottom */
    uint16_t  fifo_out;                         /* A825 FIFO top */
    bool fifo_overflow;                         /* A825 FIFO overflow indicator */
} A825Drvr_DataRxFifo_t;                        /* A825 receive FIFO */

typedef struct
{
    A825Drvr_DataRxFifo_t tFifo;
} A825Drvr_Rx_Data_t;

typedef struct
{
    uint32_t    u32ErrCnt;
    uint16_t    u16TxFailureCnt;
    uint16_t    u16TimeoutCnt;
    uint16_t    u16NullPtrCnt;
} A825Drvr_Tx_Data_t;

typedef struct
{
    volatile struct CAN_REGS* const ptCanRegs;
    A825Drvr_Tx_Data_t tTx;
    A825Drvr_Rx_Data_t tRx;
    tCAN_ES_REG_t tCanEsReg;
    bool_t bBusPassive;
    bool_t bBusWarning;
    bool_t bBusOff;
} A825Drvr_Data_t;

extern A825Drvr_Data_t tA825Drvr_Data[NUM_CAN_CHANNELS];

ERRORCODES A825Drvr_Init(A825Drvr_Data_t* const ptDrvrData, const A825_BUS_TYPE_T eBusType);
void A825Drvr_ReceiveRxDataService(A825Drvr_Data_t* const ptDrvrData);
ERRORCODES A825Drvr_SendTxData(A825Drvr_Data_t* const ptDrvrData, const MSG_MGR_t *const ptr_a825_msg_to_send);
void A825Drvr_ReadRxFifo(A825Drvr_Data_t* const ptDrvrData, MSG_MGR_t *const ptr_msg);

#endif
