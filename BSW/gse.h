/****************************************************************************************************
*  File name: gse.h
*
*  Purpose: Public Interface for sending messages to GSE.
*      This file describes the Public Interface for the GSE (Ground Support
*      Equipment) serial channel.  The interface provides the
*      routines necessary to initialize the SCI port and send a message on the channel.
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

#ifndef GSE_H__
#define GSE_H__

/*      Include Files
*/
//#include "dsp281x_regs.h"
#include "scimessage.h"
#include "hallsensor.h"
#include "timer.h"
#include "parameter.h"

#if 0
/*
  gse GSE SCI Communications
  
  brief The GSE module provides services for communicating with the Ground Support Equipment 
    interface (SCI port B).  Garmin protocol data link escape stuffing is utilized.  All 
    data values necessary for each message sent are pulled by the GSE module from other 
    software modules.
    \n\n
    The behavior of the GSE module depends on the overall state of the MCU.
    When the MCU has not detected the presence of any GSE during powerup, the
    GSE module will default to broadcast mode.  In this mode, the MCU will
    transmit the broadcast message (as defined in the Interface Control Document)
    at a 10Hz rate.
    \n\n
    If GSE has been detected during powerup, then the MCU will begin polling
    the SCI receive register at a 1KHz rate, waiting for the mode select message.
    The mode select message, once received, will put the MCU into either interactive 
    Test mode, or interactive Rig mode.  The MCU will then fall back to polling of the
    SCI receive register at the 1KHz rate, waiting for transmitted commands to parse
    and act on accordingly.  
*/

/*      Public Defines
*/
/* Bit definitions for serial port (SCI-A) initialization */
#define SCIBCHAR_VALUE    7U                 /* 8 bits per char */

/* SYSCLKOUT = 200MHz; @LSPCLK = 100.0MHz) SCI Baud rate = LSPCLK / ((BRR + 1) * 8) */
/* BRR = 0x001A */
/* Baud rate = 100.0MHz / ((650 + 1) * 8) = 100Mhz/(651 * 8) = 19201.229 = ~19200 = 0.1% error */
#define SCIBHBAUD_VALUE    0x02U    /* BRR = 650, High = 0x02 */
#define SCIBLBAUD_VALUE    0x8AU    /* BRR = 650, Low =  0x8A */
#define GSE_RX_BUF_SIZE         32        /* maximum size needed for the global GSE interface serial receive buffer (unstuffed) */
#define GSE_TX_BUF_SIZE         342U      /* maximum size needed for the global transmit buffer to hold the largest defined transmit message */

#define GSE_RX_CMD_LFT_ID       0x80 /* left channel value for the Channel Data Identifier data word in the command message */
#define GSE_RX_CMD_RT_ID        0x81 /* right channel value for the Channel Data Identifier data word in the command message */
#define GSE_RX_CMD_BOTH_ID      0x84 /* both channel value for the Channel Data Identifier data word in the command message */
#define COMMAND_PKT_SIZE        0x07    /* value for the Data Packet Size data word in the command message */

#define GSE_TEST_LFT_CHN_ID     0xA0 /* left channel value for the Channel Data Identifier data word in the test response message */
#define GSE_TEST_RT_CHN_ID      0xA1 /* right channel value for the Channel Data Identifier data word in the test response message */

#define GSE_IBIT_LFT_CHN_ID     0xC0
#define GSE_IBIT_RT_CHN_ID      0xC1
#define IBIT_PKT_SIZE           0x2B

#define SELECT_MSG_SIZE         7       /* the unstuffed maximum size of a select maintenance mode message */
#define COMMAND_MSG_SIZE        13      /* the unstuffed maximum size of a received interactive mode command message */

/*    brief Message definition for GSE Rig interactive mode command response */
typedef struct
{
    Uint16 BeginDLE;    /* Word No. 00, the beginning data link escape character */
    Uint16 ChanDataID;  /* Word No. 01, channel data identifier */
    Uint16 DataPktSize; /* Word No. 02, data packet size for message */
    Uint16 u16ChId;     /* Word No. 03, left or right channel/rig pin status */
    Uint16 RigStatus;   /* Word No. 04, rig status */
    Uint16 hall1_LSW;   /* Word No. 05, onside position quadrature count #1 */
    Uint16 hall1_MSW;   /* Word No. 06, onside position quadrature count #2 */
    Uint16 rvdt_P_POS_LSW;   /* Word No. 07, onside RVDT P_POS count LSW */
    Uint16 rvdt_P_POS_MSW;   /* Word No. 08, onside RVDT P_POS count MSW */
    Uint16 RigFault;    /* Word No. 09, rig fault code data */
    uint16_t RigCmd;    /* Word No. 10, rig command that the node is responding to (echo) */
    uint16_t SpecialRequestResponse1; /* Word No. 11, Special Response Message Word 1 Reserved for Special Request Message Commands */
    uint16_t SpecialRequestResponse2; /* Word No. 12, Special Response Message Word 2 Reserved for Special Request Message Commands */
    uint16_t SpecialRequestResponse3; /* Word No. 13, Special Response Message Word 3 Reserved for Special Request Message Commands */
    uint16_t SpecialRequestResponse4; /* Word No. 14, Special Response Message Word 4 Reserved for Special Request Message Commands */
} GSE_TX_RIG_MSG;

typedef struct
{
    Uint16 BeginDLE;
    Uint16 ChanDataID;
    Uint16 DataPktSize;
    Uint16 SupportDataID;
    Uint16 reserved;
    Uint16 SupportDW00;
    Uint16 SupportDW01;
    Uint16 SupportDW02;
    Uint16 SupportDW03;
    Uint16 SupportDW04;
    Uint16 SupportDW05;
    Uint16 SupportDW06;
    Uint16 SupportDW07;
    Uint16 SupportDW08;
    Uint16 SupportDW09;
    Uint16 SupportDW10;
    Uint16 SupportDW11;
    Uint16 SupportDW12;
    Uint16 SupportDW13;
    Uint16 SupportDW14;
    Uint16 SupportDW15;
    Uint16 SupportDW16;
    Uint16 SupportDW17;
    Uint16 SupportDW18;
    Uint16 SupportDW19;
    Uint16 SupportDW20;
    Uint16 SupportDW21;
    Uint16 SupportDW22;
    Uint16 SupportDW23;
    Uint16 SupportDW24;
    Uint16 SupportDW25;
    Uint16 SupportDW26;
    Uint16 SupportDW27;
    Uint16 SupportDW28;
    Uint16 SupportDW29;
    Uint16 SupportDW30;
    Uint16 SupportDW31;
    Uint16 SupportDW32;
    Uint16 SupportDW33;
    Uint16 SupportDW34;
    Uint16 SupportDW35;
    Uint16 SupportDW36;
    Uint16 SupportDW37;
    Uint16 SupportDW38;
    Uint16 SupportDW39;
    Uint16 SupportDW40;
} GSE_TX_IBIT_MSG;




#define RIG_RSP_MSG_SIZE        sizeof(GSE_TX_RIG_MSG)
#define RIG_data_Size            sizeof(GSE_TX_RIGData_MSG)
#define IBIT_RSP_MSG_SIZE       sizeof(GSE_TX_IBIT_MSG)
#define SIPT_RSP_MSG_SIZE       sizeof(GSE_TX_SIPT_MSG)
#endif

/*      Public Type Definitions
*/

/* Rig commands */
typedef enum
{
    RIG_CMD_NONE =             0x00,
    STOP =                     0x3E, /* stop */
    RLS_BRAKE =                0x3F, /* release brake */
    APPLY_BRAKE =              0x4F, /* apply brake */
    RTN_TO_RSL =               0x50, /* return to retract software limit */
    RTN_TO_N1 =                0x51, /* return to position N1 */
    RTN_TO_ZERO =              0x52, /* return to position ZERO (flap neutral [0 degrees]) */
    RTN_TO_P1 =                0x53, /* return to position P1 */
    RTN_TO_P2 =                0x54, /* return to position P2 */
    RTN_TO_P3 =                0x55, /* return to position P3 */
    RTN_TO_P4 =                0x56, /* return to position P4 */
    RTN_TO_P5 =                0x57, /* return to position P5 */
    RTN_TO_P6 =                0x58, /* return to position P6 */
    RTN_TO_P7 =                0x59, /* return to position P7 */
    RTN_TO_P8 =                0x5A, /* return to position P8 */
    RTN_TO_P9 =                0x5B, /* return to position P9 */
    RTN_TO_P10 =               0x5C, /* return to position P10 */
    RTN_TO_P11 =               0x5D, /* return to position P11 */
    RTN_TO_ESL =               0x5E, /* return to extend software limit */
    EXTEND_QUAD_250 =          0x08, /* jog 100 Quad counts extend */
    RETRACT_QUAD_250 =         0x09, /* jog 100 Quad counts retract */
    EXTEND_QUAD_25 =           0x0A, /* jog 25 Quad counts extend */
    RETRACT_QUAD_25 =          0x0B, /* jog 10 Quad counts retract */
    EXTEND_QUAD_1 =            0x0C, /* jog 1 Quad count extend */
    RETRACT_QUAD_1 =           0x0D, /* jog 1 Quad count retract */
    JOG_EXTEND =               0x0E, /* jog extend at commanded speed */
    JOG_RETRACT =              0x0F, /* jog retract at commanded speed */
    RIG_POSITION_RSL =         0x60, /* rig position retract software limit */
    RIG_POSITION_N1 =          0x61, /* rig position N1 */
    RIG_POSITION_ZERO =        0x62, /* rig position ZERO (flap neutral [0 degrees]) */
    RIG_POSITION_P1 =          0x63, /* rig position P1 */
    RIG_POSITION_P2 =          0x64, /* rig position P2 */
    RIG_POSITION_P3 =          0x65, /* rig position P3 */
    RIG_POSITION_P4 =          0x66, /* rig position P4 */
    RIG_POSITION_P5 =          0x67, /* rig position P5 */
    RIG_POSITION_P6 =          0x68, /* rig position P6 */
    RIG_POSITION_P7 =          0x69, /* rig position P7 */
    RIG_POSITION_P8 =          0x6A, /* rig position P8 */
    RIG_POSITION_P9 =          0x6B, /* rig position P9 */
    RIG_POSITION_P10 =         0x6C, /* rig position P10 */
    RIG_POSITION_P11 =         0x6D, /* rig position P11 */
    RIG_POSITION_ESL =         0x6E, /* rig position extend software limit */
    RIG_START_RIG =            0x6F, /* rig Start Rig Process */
    TX_QUAD_RSL =              0x80, /* transmit quad count for RSL */
    TX_QUAD_N1 =               0x81, /* transmit quad count for position N1 */
    TX_QUAD_ZERO =             0x82, /* transmit quad count for position ZERO (flap neutral [0 degrees]) */
    TX_QUAD_P1 =               0x83, /* transmit quad count for position P1 */
    TX_QUAD_P2 =               0x84, /* transmit quad count for position P2 */
    TX_QUAD_P3 =               0x85, /* transmit quad count for position P3 */
    TX_QUAD_P4 =               0x86, /* transmit quad count for position P4 */
    TX_QUAD_P5 =               0x87, /* transmit quad count for position P5 */
    TX_QUAD_P6 =               0x88, /* transmit quad count for position P6 */
    TX_QUAD_P7 =               0x89, /* transmit quad count for position P7 */
    TX_QUAD_P8 =               0x8A, /* transmit quad count for position P8 */
    TX_QUAD_P9 =               0x8B, /* transmit quad count for position P9 */
    TX_QUAD_P10 =              0x8C, /* transmit quad count for position P10 */
    TX_QUAD_P11 =              0x8D, /* transmit quad count for position P11 */
    TX_QUAD_ESL =              0x8E, /* transmit quad count for extend software limit */
    RIG_ENTER_RIG_MODE =       0xFE, /* Enter Rig Mode */
    RIG_NO_ACTION =            0xFF, /* do nothing */
    MAX_RIG_COMMAND_VAL =      0xFF  /* 8-bit max signal to cover all rig commands */
} eGseRigCmds_t;

/* GSE Rig Command Interface */
typedef struct
{
    eGseRigCmds_t   eCmdL;                      /* Rig Command Left Actuator  */
    eGseRigCmds_t   eCmdR;                      /* Rig Command Right Actuator */
    bool_t          bVcModeCmdL;                /* Variable Camber Mode Command for Left Actuator from GSE:  True = VC, False = HL */
    bool_t          bVcModeCmdR;                /* Variable Camber Mode Command for Right Actuator from GSE:  True = VC, False = HL */
    bool_t          bSensorFusionEnableL;       /* Sensor Fusion Enable Command for Left Actuator from GSE.*/
    bool_t          bSensorFusionEnableR;       /* Sensor Fusion Enable Command for Right Actuator from GSE.*/
    float32_t       f32PositionCmdL;            /* Position Command for Left Actuator from GSE in Inches*/
    float32_t       f32PositionCmdR;            /* Position Command for Right Actuator from GSE in Inches*/
    float32_t       f32SpeedCmdL;               /* Speed Command for Left Actuator from GSE in Percent of max speed [0-1] */
    float32_t       f32SpeedCmdR;               /* Speed Command for Right Actuator from GSE in Percent of max speed [0-1] */
} tGseRigInterface_t;

typedef enum
{
    RSL = 0,
    N1,
    ZERO,
    P1,
    P2,
    P3,
    P4,
    P5,
    P6,
    P7,
    P8,
    P9,
    P10,
    P11,
    ESL,
    NUM_RIG_POSITIONS,
    INVALID_RIG_POSITION = NUM_RIG_POSITIONS
} RIG_POSITIONS_T;

#if 0
typedef enum
{
    START_OF_MODE_ENUM,  /* placeholder for the beginning of the enum */
    INTERACTIVE_SELECT,  /* interactive mode selection */
    INTERACTIVE_RESPOND, /* interactive mode query/response */
//    BROADCAST,           /* broadcast messages only in rig or test mode */
    INVALD_MODE          /* placeholder for the end of the enum */
} GSE_MODE;
#endif

typedef enum
{
    GSE_NO_CONNECTION,   /* GSE is not connected */
    GSE_RIG_MODE,        /* Rig mode */
    GSE_TEST_MODE,       /* Test mode */
    GSE_NUMBER_OF_MODES  /* Total number of GSE Modes of Operation */
} GSE_MODE_T;

#if 0
/*    brief Message definition for GSE interactive mode command */
typedef struct
{
    Uint16 DLEbegin;        /* Word No. 00, the starting data link escape character (0x10) */
    Uint16 ChanDataID;      /* Word No. 01, channel data identifier */
    Uint16 DataPktSize;     /* Word No. 02, data packet size for message */
    Uint16 u16ChId;         /* Word No. 03, left, right, or both channels as well as rig or test mode */
    Uint16 RigCommand;      /* Word No. 04, rig mode command */
    Uint16 TestCommand;     /* Word No. 05, test mode command */
    Uint16 SerialNumLSW;    /* Word No. 06, PWA serial number LSW */
    Uint16 SerialNumMSW;    /* Word No. 07, PWA serial number MSW */
    Uint16 reserved1;       /* Word No. 08, reserved */
    Uint16 reserved2;       /* Word No. 09, reserved */
    int16 checksum;         /* Word No. 10, the message checksum */
    Uint16 DLEend;          /* Word No. 11, the ending data link escape character (0x10) */
    Uint16 endofTx;         /* Word No. 12, the end of transmission character (0x03) */
} GSE_RX_CMD_MSG;

typedef struct
{
    bool SIPT_Message;
    bool Rig_Message;
    bool NVM_Data;
    bool IBIT_Message;
} TRANSMITTING_MSG;

#define GSE_SIPT_DEFAULTS {              \
                        0,0,0,0,0,0,0,0, \
                        0,0,0,0,0,0,0,0, \
                        0,0,0,0,0,0,0,0, \
                        0,0,0,0,0,0,0,0, \
                        0,0,0,0,0,0,0,0, \
                        0,0,0,0,0,0      }
#endif

/*      Public Variable Declarations
*/
extern GSE_MODE_T tGseMode; /* Maintenance Mode State */
extern bool FormatNvm;
extern bool ClearState;
extern bool ClearRigging;
#if 0
extern bool BroadcastNow;
extern Uint16 GseTxBuf[GSE_TX_BUF_SIZE]; /* the global transmit buffer for the GSE interface */
extern Uint16 GseTxIndex;                  /* index into the global transmit buffer */
extern GSE_TX_IBIT_MSG Gse_SIPT_Default;
extern bool IBIT_Test;
extern bool TxRigPage1;
extern bool TxRigAll;
#endif
extern Uint16 ReportedFaultCode;
extern Uint16 ReportedFaultData;
extern const int16_t s16RigPositionQuadCount[NUM_ACTUATOR_TYPES][NUM_RIG_POSITIONS];
extern const float32_t f32RigPositionStroke[NUM_ACTUATOR_TYPES][NUM_RIG_POSITIONS];
extern tGseRigInterface_t tGseRigInterface;
/*      Public ROM Constants */

/*      Public Interface Function Prototypes */
void Gse_Init(void);
void Gse_CheckRigTest(void);
void Gse_EstablishConnection(void);
#if 0
SCI_STATUS SendTestResponse(void);
Uint16 GseTest_ExecuteCommand(Uint16 command, GSE_RX_CMD_MSG *msg);
void IBITTest_Result(GSE_TX_IBIT_MSG *);
#endif
void GseRig_Command(void);
void GseRig_SetFeedback(void);
void GseRig_GoToPosition(int16_t s16PositionCmdQuad, float32_t f32Speed);

#endif
/* end gse.h */

