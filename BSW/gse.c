/****************************************************************************************************
*  File name: gse.c
*
*  Purpose: Send messages to GSE.
*  This file implements the interface for the GSE (Ground Support Equipment)
*  serial channel.  The interface provides routines necessary to intialize the
*  driver and send/receive messages on the channel.
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

/*      Include Files */

#include "DFT.h"
#include "F2837xD_device.h"
#include "mcu.h"
#include "events.h"
#include "timer.h"
#include "gse.h"
#include "bit.h"
#include "actuation.h"
#include "scimessage.h"
#include "adc.h"
#include "panel.h"
#include "gpio.h"
#include "brake.h"
#include "nvm.h"
#include "parameter.h"
#include "motor.h"
#include "rvdt.h"
#include "spi.h"
#include "bitmonitors.h"
#include "PDI_Mgr.h"
#include "ASW_BSW_I.h"

/*      Local Defines */

/* Definition of SCI register based on hardware */
#if defined(DRV8312_DEV_KIT)
    /* Development kit is on SCI_A */
    #define GseSciRegs  ScibRegs
#else
    /* MCU Hardware is on SCI_B */
    #define GseSciRegs  ScibRegs
#endif

//#define SIPT_PKT_SIZE           0x2B
//
//#define SELECT_RIG_MODE_R       0x52 /* ASCII data value of 'R', notifying the MCU to enter Rig mode */
//#define SELECT_RIG_MODE_r       0x72 /* ASCII data value of 'r', notifying the MCU to enter Rig mode */
//#define SELECT_TEST_MODE_T      0x54 /* ASCII data value of 'T', notifying the MCU to enter Test mode */
//#define SELECT_TEST_MODE_t      0x74 /* ASCII data value of 't', notifying the MCU to enter Test mode */
//
//#define GSE_RX_SELECT_LFT_ID    0x70 /* left channel value for the Channel Data Identifier data word in the mode select message */
//#define GSE_RX_SELECT_RT_ID     0x71 /* right channel value for the Channel Data Identifier data word in the mode select message */
//#define GSE_RX_SELECT_BOTH_ID   0x72 /* right channel value for the Channel Data Identifier data word in the mode select message */
//
//#define GSE_RIG_LFT_CHN_ID      0x90 /* left channel value for the Channel Data Identifier data word in the rig response message */
//#define GSE_RIG_RT_CHN_ID       0x91 /* right channel value for the Channel Data Identifier data word in the rig response message */
//#define GSE_RIG_BOTH_CHN_ID     0x92 /* both channel value for the Channel Data Identifier data word in the rig response message */
//
//
//#define GSE_SIPT_LFT_CHN_ID     0xB0
//#define GSE_SIPT_RT_CHN_ID      0xB1

/* Command data word values */
 
#define RIGDATA_PAGE1           0xC3 /* transmit page 1 of the rig data */
#define RIGDATA_PAGE2           0xC4 /* transmit page 2 of the rig data */
#define RIGDATA_PAGE3           0xC5 /* transmit page 3 of the rig data */
#define RIGDATA_PAGE4           0xC6 /* transmit page 4 of the rig data */
#define RIGDATA_PAGE5           0xC7 /* transmit page 5 of the rig data */
#define RIGDATA_PAGE6           0xC8 /* transmit page 6 of the rig data */
#define RIGDATA_PAGE7           0xC9 /* transmit page 7 of the rig data */
#define RIGDATA_PAGE8           0xCA /* transmit page 8 of the rig data */
#define RIGDATA_PAGE9           0xCB /* transmit page 9 of the rig data */
#define RIGDATA_PAGE10          0xCC /* transmit page 10 of the rig data */
#define RIGDATA_PAGE11          0xCD /* transmit page 11 of the rig data */
#define RIGDATA_PAGE12          0xCE /* transmit page 12 of the rig data */
#define RIGDATA_PAGE13          0xCF /* transmit page 13 of the rig data */
#define RIGDATA_PAGE14          0xD0 /* transmit page 14 of the rig data */
#define RIGDATA_PAGE15          0xD1 /* transmit page 15 of the rig data */
#define RIGDATA_PAGE16          0xD2 /* transmit page 16 of the rig data */
#define RIGDATA_PAGE17          0xD3 /* transmit page 17 of the rig data */
#define RIGDATA_PAGE18          0xD4 /* transmit page 18 of the rig data */
#define RIGDATA_PAGE19          0xD5 /* transmit page 19 of the rig data */
#define RIGDATA_PAGE20          0xD6 /* transmit page 20 of the rig data */
#define RIGDATA_PAGE21          0xD7 /* transmit page 21 of the rig data */
#define RIGDATA_PAGE22          0xD8 /* transmit page 22 of the rig data */
#define RIGDATA_PAGE23          0xD9 /* transmit page 23 of the rig data */
#define RIGDATA_PAGE24          0xDA /* transmit page 24 of the rig data */
#define RIGDATA_PAGE25          0xDB /* transmit page 25 of the rig data */
#define RIGDATA_PAGE26          0xDC /* transmit page 26 of the rig data */
#define RIGDATA_PAGE27          0xDD /* transmit page 27 of the rig data */
#define RIGDATA_PAGE28          0xDE /* transmit page 28 of the rig data */
#define RIGDATA_PAGE29          0xDF /* transmit page 29 of the rig data */
#define RIGDATA_PAGE30          0xE0 /* transmit page 30 of the rig data */
#define RIGDATA_PAGE31          0xE1 /* transmit page 31 of the rig data */
#define RIGDATA_PAGE32          0xE2 /* transmit page 32 of the rig data */
#define RIGDATA_PAGE33          0xE3 /* transmit page 33 of the rig data */
#define RIGDATA_PAGE34          0xE4 /* transmit page 34 of the rig data */
#define RIGDATA_PAGE35          0xE5 /* transmit page 35 of the rig data */
#define RIGDATA_PAGE36          0xE6 /* transmit page 36 of the rig data */
#define RIGDATA_PAGE37          0xE7 /* transmit page 37 of the rig data */
#define RIGDATA_PAGE38          0xE8 /* transmit page 38 of the rig data */
#define RIGDATA_PAGE39          0xE9 /* transmit page 39 of the rig data */
#define RIGDATA_PAGE40          0xEA /* transmit page 40 of the rig data */
#define RIGDATA_PAGE41          0xEB /* transmit page 41 of the rig data */
#define RIGDATA_PAGE42          0xEC /* transmit page 42 of the rig data */
#define RIGDATA_PAGE43          0xED /* transmit page 43 of the rig data */
#define RIGDATA_PAGE44          0xEE /* transmit page 44 of the rig data */
#define RIGDATA_PAGE45          0xEF /* transmit page 45 of the rig data */
#define RIGDATA_PAGE46          0xF0 /* transmit page 46 of the rig data */

/* Test Mode values */
#define TEST_NO_ACTION          0x00 /* do nothing */
#define WRITE_SERIALNUM         0xA0 /* record serial number in NVM */
#define TX_MOSTRECENT_NVM       0xA1 /* transmit most recent NVM Fault Table record */
#define TX_NEXTRECENT_NVM       0xA2 /* transmit next most recent NVM Fault Table record */
#define CLEAR_ALL_NVM           0xA3 /* clear all NVM */
#define CLEAR_ALL_RIG           0xA4 /* clear all rig information */
#define CLEAR_ALL_STATE         0xA5 /* clear all state information */
#define TX_ALL_NVM              0xA6 /* transmit all NVM data */
#define GET_NVM_STATUS          0xA7 /* get NVM status */

#define ADJ_ASYM_OFFSET         0xC0 /* adjust asymoffset to a particular level */
#define TX_RTC			        0xC2 /* Transmit the Run time counter over the GSE Bus */
#define SEND_SIPT_DATA          0xFF /* transmit SIPT data for SIPT tests */

/*      Local Type Definitions */

/* GSE Rig Command for a single MCU channel */
typedef struct
{
    eGseRigCmds_t   eRigCmd;                /* Rig Command */
    bool_t          bVcModeCmd;             /* VC Mode Command */
    bool_t          bSensorFusionEnableCmd; /* Sensor Fusion Enable Command */
    float32_t       f32PositionCmd;         /* Position Command in Inches */
    int16_t         s16PositionCmdQuad;     /* Position Command in Motor Quad Counts */
    float32_t       f32SpeedCmd;            /* Speed Command in Percent of max speed [0-1] */
} tGseRigCmd_t;

#if 0
typedef struct
{
    Uint16 BeginDLE;
    Uint16 ChanDataID;
    Uint16 DataPktSize;
    Uint16 SerialNumLow;
    Uint16 SerialNumHigh;
    Uint16 ROM_CRC_1;
    Uint16 ROM_CRC_2;
    Uint16 ROM_CRC_3;
    Uint16 ROM_CRC_4;
    Uint16 SoftVersion;
    Uint16 SW_VerDisc;
    Uint16 DiscreteSigs;
    Uint16 u16RigPins;
    Uint16 MotorCurrentLow;
    Uint16 MotorCurrentHigh;
    Uint16 RvdtSumLow;
    Uint16 RvdtSumHigh;
    Uint16 RvdtPosLow;
    Uint16 RvdtPosHigh;
    Uint16 BrakeMonLow;
    Uint16 BrakeMonHigh;
} GSE_TX_SIPT_MSG;

/*    brief Message definition for GSE interactive mode selection */
typedef struct
{
    Uint16 DLEbegin;    /* Word No. 00, the starting data link escape character (0x10) */
    Uint16 ChanDataID;  /* Word No. 01, channel data identifier */
    Uint16 DataPktSize; /* Word No. 02, data packet size for message */
    Uint16 ModeSelect;  /* Word No. 03, MCU maintenance mode selection */
    int16 checksum;     /* Word No. 04, the message checksum */
    Uint16 DLEend;      /* Word No. 05, the ending data link escape character (0x10) */
    Uint16 endofTx;     /* Word No. 06, the end of transmission character (0x03) */
} GSE_RX_SELECT_MSG;

/*      brief Message definition for Complex GSE rig interface transmission */
typedef struct
{
    Uint16 DLE_Begin;
    Uint16 ChanDataID;
    Uint16 DataPktSize;
    Uint16 OnQuadPosLow;
    Uint16 OnQuadPosHigh;
    Uint16 FPSU_Pos1Low;
    Uint16 FPSU_Pos1High;
    Uint16 FPSU_Pos2Low;
    Uint16 FPSU_Pos2High;
    Uint16 MotorCurrent;
    Uint16 XQuadPosLow;
    Uint16 XQuadPosHigh;
    Uint16 FPSU_Pos3Low;
    Uint16 FPSU_Pos3High;
    Uint16 FPSU_Pos4Low;
    Uint16 FPSU_Pos4High;
    Uint16 PWM_Percent;
    Uint16 MotorStatus;
    Uint16 Heartbeat;
} GSE_TX_COMPLEX_MSG;

/*      brief Message definition for Complex GSE rig interface reception */
typedef struct
{
    Uint16 DLE_Begin;
    Uint16 ChanDataID;
    Uint16 DataPktSize;
    Uint16 RigDataWord_1;
    Uint16 RigDataWord_2;
    Uint16 RigDataWord_3;
    Uint16 Heartbeat;
    int16 Checksum;
    Uint16 DLE_End;
    Uint16 EndOfTx;
} GSE_RX_COMPLEX_MSG;

TRANSMITTING_MSG Transmitting = {false, false, false, false};
#endif

/*      Global Variables */
GSE_MODE_T tGseMode;
extern float32 SpeedOfMotor;
extern bool_t bRunSensorFusion;
tGseRigInterface_t  tGseRigInterface = { 0 };

/*      Local ROM Constants */

/*      Local Variable Declarations */
static tGseRigCmd_t tGseRigCmd = { 0 };
static tGseRigCmd_t tGseRigCmdx = { 0 };
static Timer_t tMotorReversalTimer = TIMER_DEFAULTS; /* timer used for pausing when reversing direction*/

Uint16 RigCmd = 0;
uint16_t SpecialRequestResponse1 = 0U; /* Response word 1 for the Special Request Message Type Command */
uint16_t SpecialRequestResponse2 = 0U; /* Response word 2 for the Special Request Message Type Command */
uint16_t SpecialRequestResponse3 = 0U; /* Response word 3 for the Special Request Message Type Command */
uint16_t SpecialRequestResponse4 = 0U; /* Response word 4 for the Special Request Message Type Command */

#if 0
/* GSE Broadcast variables */
bool BroadcastNow = false;
GSE_TX_IBIT_MSG Gse_SIPT_Default = GSE_SIPT_DEFAULTS;


/*  message receive variables */
Uint16 GseRxBuf[GSE_RX_BUF_SIZE] = {0}; /* the global SCI port B receive buffer */ 
Uint16 GseRxIndex = 0;                   /* index into the global SCI port B receive buffer */
Uint16 GseRxNumBytes = 0;               /* global count of the number of received bytes */

/*  message transmit variables */
Uint16 GseTxBuf[GSE_TX_BUF_SIZE] = {0}; /* the global transmit buffer for the GSE interface */
Uint16 GseTxIndex = 0;                  /* index into the global transmit buffer */

bool TransmitLeft = false;
bool TransmitRight = false;
bool TransmitBoth = false;

bool FormatNvm = false;
bool ClearRigging = false;
bool ClearState = false;

bool TxMostRecentNvm = false;
bool TxNextRecentNvm = false;
bool TxAllNvm = false;

bool TxRigAll = false;
bool TxRigPage1 = false;
bool TxRigPage2 = false;
bool TxRigPage3 = false;
bool TxRigPage4 = false;
bool TxRigPage5 = false;
bool TxRigPage6 = false;
bool TxRigPage7 = false;
bool TxRigPage8 = false;
bool TxRigPage9 = false;
bool TxRigPage10 = false;
bool TxRigPage11 = false;
bool TxRigPage12 = false;
bool TxRigPage13 = false;
bool TxRigPage14 = false;
bool TxRigPage15 = false;
bool TxRigPage16 = false;
bool TxRigPage17 = false;
bool TxRigPage18 = false;
bool TxRigPage19 = false;
bool TxRigPage20 = false;
bool TxRigPage21 = false;
bool TxRigPage22 = false;
bool TxRigPage23 = false;
bool TxRigPage24 = false;
bool TxRigPage25 = false;
bool TxRigPage26 = false;
bool TxRigPage27 = false;
bool TxRigPage28 = false;
bool TxRigPage29 = false;
bool TxRigPage30 = false;
bool TxRigPage31 = false;
bool TxRigPage32 = false;
bool TxRigPage33 = false;
bool TxRigPage34 = false;
bool TxRigPage35 = false;
bool TxRigPage36 = false;
bool TxRigPage37 = false;
bool TxRigPage38 = false;
bool TxRigPage39 = false;
bool TxRigPage40 = false;
bool TxRigPage41 = false;
bool TxRigPage42 = false;
bool TxRigPage43 = false;
bool TxRigPage44 = false;
bool TxRigPage45 = false;
bool TxRigPage46 = false;

bool IBIT_Test = false;
#endif
Uint16 ReportedFaultCode = 0;
Uint16 ReportedFaultData = 0;

Uint16 nvmidx = 0xFFFF;

/* Rig Position Table in Quad Counts */
const int16_t s16RigPositionQuadCount[NUM_ACTUATOR_TYPES][NUM_RIG_POSITIONS] =
{
/*{   RSL,    N1,  ZERO,    P1,    P2,    P3,    P4,    P5,    P6,    P7,    P8,     P9,    P10,    P11,    ESL } */
  {  -981,  -654,     0,  1338,  1802,  3402,  4485,  6045,  6347,  8649,  10386, 12097,  12481,  14129,  14429 },  /* Actuator 1 */
  {  -981,  -654,     0,  1338,  1802,  3402,  4485,  6045,  6347,  8649,  10386, 12097,  12481,  14129,  14429 },  /* Actuator 2 */
  { -1006,  -760,     0,   795,  1161,  2753,  3647,  4944,  5197,  6260,   7583,  8903,  10212,  11376,  11633 },  /* Actuator 3 */
  { -1006,  -759,     0,   795,  1161,  2755,  3650,  4948,  5201,  6264,   7587,  8907,  10214,  11377,  11633 },  /* Actuator 4 */
};

/* Rig Position Table in Stroke Inches */
const float32_t f32RigPositionStroke[NUM_ACTUATOR_TYPES][NUM_RIG_POSITIONS] =
{
/*{        RSL,        N1,      ZERO,        P1,        P2,        P3,        P4,        P5,        P6,        P7,        P8,        P9,      P10,       P11,        ESL } */
  {  -0.39000F, -0.25997F,  0.00000F,  0.53186F,  0.71600F,  1.35186F,  1.78200F,  2.40200F,  2.52196F,  3.43642F,  4.12662F,  4.80644F,  4.95900F,  5.61378F,  5.73300F },  /* Actuator 1 */
  {  -0.39000F, -0.25997F,  0.00000F,  0.53186F,  0.71600F,  1.35186F,  1.78200F,  2.40200F,  2.52196F,  3.43642F,  4.12662F,  4.80644F,  4.95900F,  5.61378F,  5.73300F },  /* Actuator 2 */
  {  -0.40000F, -0.30209F,  0.00000F,  0.31614F,  0.46143F,  1.09399F,  1.44928F,  1.96452F,  2.06507F,  2.48725F,  3.01297F,  3.53759F,  4.05739F,  4.52013F,  4.62200F },  /* Actuator 3 */
  {  -0.40000F, -0.30175F,  0.00000F,  0.31619F,  0.46158F,  1.09483F,  1.45052F,  1.96619F,  2.06680F,  2.48910F,  3.01472F,  3.53897F,  4.05820F,  4.52032F,  4.62200F }   /* Actuator 4 */
};

Timer_t tRightChnlResponseTmr = TIMER_DEFAULTS;

/*      Local Function Prototypes */
#if 0
SCI_STATUS ReceiveGseMessage(void);
SCI_STATUS SendGseMessage(Uint16);
SCI_STATUS SendRigResponse(void);

SCI_STATUS SendIBIT_Response(void);
SCI_STATUS SendSIPT_Response(void);


void ProcessSelectMsg(void);
void ProcessCommandMsg(void);

void RigAll(GSE_TX_IBIT_MSG *);
#endif

void GseRig_ExecuteCommand(void);
bool_t GseRig_GetCommands(void);
void GseRig_GetGseCommands(void);
void GseRig_SetRigCommands(void);
void GseRig_SetFeedbackSignals(void);

/*      Function Definitions */

/*
    brief Initialize GSE

     Purpose:
        Initialization routine for GSE. GSE communication is initialized as part of the
        ARINC825 Driver. May change this in the future. Initialize any GSE specific
        variables here.

    return  void
    
    Preconditions and Assumptions:
        Call this function only once, prior to the Scheduler loop.


*/
void Gse_Init(void)
{
    /* PATH(Gse_Init,A); */
    asm(" NOP");    /* prevent compiler from removing function */
}

void Gse_EstablishConnection(void)
{
    A825Mgr_RxSrvc(&tA825_Bus[ARINC825_MAINT_BUS]);     /* Run the ARINC 825 Receive Service for the MAINTENANCE BUS */

    /* Set the Communication Established flag for the Control Bus based on the GSE Stale Status */
    if(tA825_Bus[ARINC825_MAINT_BUS].tRx.ptMsgList[A825_MAINT_BUS_RX_RIG_CMD].bMsgStaleStatus == false)
    {
        /* Decode and retrieve the RIG Command Signals */
        GseRig_GetGseCommands();

        if((tGseRigInterface.eCmdL == RIG_ENTER_RIG_MODE) && (tGseRigInterface.eCmdR == RIG_ENTER_RIG_MODE))
        {
            /* Set Maintenance Bus GSE as having established Rig Mode communication */
            tA825_Bus[ARINC825_MAINT_BUS].bCommsEstablished = true;

            /* Set GSE Mode to RIG Mode */
            tGseMode = GSE_RIG_MODE;

            /* Post event to transition to RIG MODE */
            Events_PostEvent(EVENT_ENTER_RIG_MODE, 0);
            MotorCmd.Rigging = true;
        }
    }
//    // For future test mode
//    else if(tA825_Bus[ARINC825_MAINT_BUS].tRx.ptMsgList[A825_MAINT_BUS_RX_TEST_CMD].bMsgStaleStatus == false)
//    {
//        /* Set Maintenance Bus GSE as having established Test Mode communication */
//        tA825_Bus[ARINC825_MAINT_BUS].bCommsEstablished = true;
//
//        /* Set GSE Mode to TEST Mode */
//        tGseMode = GSE_TEST_MODE;
//
//        /* Post event to transition to TEST MODE */
//        Events_PostEvent(EVENT_ENTER_TEST_MODE, 0);
//    }
    return;
}

#if 0
/*
    brief Poll the SCI-B receive buffer.

     Purpose:
        This routine is responsible for checking for received data in Rig or Test mode,
        and responding accordingly.

    param[in] mode  the mode that the GSE interface is currently in, based on system status

    return  void

     Preconditions and Assumptions:
         None.

*/
void Gse_Interactive(GSE_MODE mode)
{
    SCI_STATUS status;

    /* PATH(Gse_Interactive,A); */

    /*shall take action depending on the current mode of the GSE interface */
    switch (mode)
    {
        /* In this mode we first look for the 'R'/'r' or 'T'/'t' mode select message */
        case INTERACTIVE_SELECT:
            /* PATH(Gse_Interactive,B); */
            /*shall poll the SCI port B receive buffer for the presence of received bytes */
            status = ReceiveGseMessage();

            /*shall check for indication that there is a complete message in the receive buffer */
            if (status == VALID_RX_MSG)
            {
                /* PATH(Gse_Interactive,C); */
                /*shall parse the received message as if it was an interactive mode selection message */
                /* when the system has powered up and the presence of GSE has been detected */
                ProcessSelectMsg();
            }
            break;
        case INTERACTIVE_RESPOND:
            /* PATH(Gse_Interactive,D); */

            if (Transmitting.Rig_Message == true)
            {
                /* PATH(Gse_Interactive,F); */
                SendRigResponse();
            }

            if (Transmitting.SIPT_Message == true)
            {
                /* PATH(Gse_Interactive,E); */
                SendSIPT_Response();
            }

            if (Transmitting.NVM_Data == true)
            {
                /* PATH(Gse_Interactive,G); */
                SendTestResponse();
            }

            if (Transmitting.IBIT_Message == true)
            {
                /* PATH(Gse_Interactive,H); */
                SendIBIT_Response();
            }

            status = ReceiveGseMessage();

            /* was there a complete message in the receive buffer? */
            if (status == VALID_RX_MSG)
            {
                /* PATH(Gse_Interactive,I); */
                ProcessCommandMsg();
                /* shall parse the received message as if it was an interactive */
                /* command message once the interactive mode (Rig or Test) has been established */
            }
            break;
        default:
            /* PATH(Gse_Interactive,J); */
            break;
    }

    /* PATH(Gse_Interactive,K); */
}


/*
    brief Parse the mode select message

    Purpose:
        This routine will treat the message in the global receive buffer as if 
        it is the interactive receive mode select message (refer to the Interface Control Document
        for definition of this message).  First, any stuffed data link escape
        characters in the global buffer are removed, and the unstuffed message is 
        put into a temporary buffer.  The received checksum is then verified by adding 
        it to all the meaningful data words in the message.  A valid checksum when added
        to the data words should result in hex 100 (0 with rollover).  If the checksum
        proves to be valid, verify that this MCU was the intended receiver of the message
        by comparing the Channel Data ID data word against the PrimarySide flag.  If the message
        was intended for this MCU, enter Rig mode or Test mode depending on the value
        of the Mode Select data word.
        
    return  void
    
     Preconditions and Assumptions:
        That a valid message resides in the receive buffer, and the interactive receive
        mode has not been selected yet.
  

*/
void ProcessSelectMsg(void)
{
    GSE_RX_SELECT_MSG *selectMsg;        /* pointer for typecasting the received data to a known message type */
    Uint16 Rxbuf[SELECT_MSG_SIZE] = {0}; /* buffer for unstuffing the received message */
    int16 chksum;                        /* used for verifying the checksum */
    Uint16 x, y;                         /* loop variables */

    /* PATH(ProcessSelectMsg,A); */

    /*shall remove any stuffed data link escape characters within the received message */
    for (x = 1, y = 1, Rxbuf[0] = DLE_CHAR; y < GseRxNumBytes; x++, y++)
    {
		/* PATH(ProcessSelectMsg,K); */
        Rxbuf[x] = GseRxBuf[y];

        /* if the current character is not Word 01 of the message and two consecutive words 
           contain the data link escape character, skip past the stuffed escape character */
  
        if (( y != 1) && ((GseRxBuf[y] & 0xFF) == DLE_CHAR) && ((GseRxBuf[y + 1] & 0xFF) == DLE_CHAR))
        {
            /* PATH(ProcessSelectMsg,B); */
            y++;
        }
    }

    /*shall clear out the global number of received bytes */
    GseRxNumBytes = 0;

    /*shall typecast the buffer to the select message type */
    selectMsg = (GSE_RX_SELECT_MSG *)Rxbuf;

    /*shall validate the message by adding the message checksum to message data words (DLE, EOTX, and "stuffed" characters excluded) */
    /*   and verifying the result is 0 */
    chksum = selectMsg->checksum + selectMsg->ChanDataID + selectMsg->DataPktSize +
             selectMsg->ModeSelect;

    /* does the checksum indicate valid received data? */
    if ((chksum & 0xFF) == 0x00)
    {
        /* PATH(ProcessSelectMsg,C); */

        /*shall verify that this message was meant for this MCU */
        if (((G_bChannelA == true) && (selectMsg->ChanDataID == GSE_RX_SELECT_LFT_ID)) ||
            ((G_bChannelA == false) && (selectMsg->ChanDataID == GSE_RX_SELECT_RT_ID)) || (selectMsg->ChanDataID == GSE_RX_SELECT_BOTH_ID))
        {
            /* PATH(ProcessSelectMsg,D); */

            switch (selectMsg->ModeSelect)
            {
                case SELECT_RIG_MODE_R:
                    /* PATH(ProcessSelectMsg,E); */
                case SELECT_RIG_MODE_r:
                    /* PATH(ProcessSelectMsg,F); */
                    /*shall post a system event indicating Rig mode was selected if */
                    /*   the mode select data word is equal to ASCII 'R' or 'r' */
                    Events_PostEvent(EVENT_ENTER_RIG_MODE, 0);
                    MotorCmd.Rigging = true;
                    RigMode = 0;
                    break;
                case SELECT_TEST_MODE_T:
                    /* PATH(ProcessSelectMsg,G); */
                case SELECT_TEST_MODE_t:
                    /* PATH(ProcessSelectMsg,H); */
                    /*shall post a system event indicating Test mode was selected if */
                    /*   the mode select data word is equal to ASCII 'T' or 't' */
                    Events_PostEvent(EVENT_ENTER_TEST_MODE, 0);
                    TestMode = 0;
                    break;
                default:
                    /* PATH(ProcessSelectMsg,I); */
                    break;
            }
        }
    }
    
    /* PATH(ProcessSelectMsg,J); */
}

/*
    brief Parse the interactive mode command message

     Purpose:
        This routine will treat the message in the global receive buffer as if
        it is the interactive mode command message.  First, any stuffed data link escape
        characters in the global buffer are removed, and the unstuffed message is
        put into a temporary buffer.  The received checksum is then verified by adding
        it to all the meaningful data words in the message.  A valid checksum when added
        to the data words should result in hex 100 (0 with rollover).  If the checksum
        proves to be valid, verify that this MCU was the intended receiver of the message
        by comparing the Channel Data ID data word against the PrimarySide flag.  If the message
        was intended for this MCU, take appropriate action based on the value of the Command
        data word and whether the MCU is in Rig or Test mode.

    return  void

     Preconditions and Assumptions:
         None.

	The function violates the SCS requirement of having less than 100 lines of code.
	(Section 3.3.4.F of SCS8025Rev5 document)
	But all these functionalities are essential.


*/
void ProcessCommandMsg(void)
{
    GSE_RX_CMD_MSG *commandMsg;
    Uint16 Rxbuf[COMMAND_MSG_SIZE] = {0}; /* buffer for unstuffing the received message */
    int16 chksum; /* used for verifying the checksum */
    Uint16 x, y; /* loop variables */
    STATE_ID mode;
    bool msgGood = false;
    Uint16 testModeReturn = 0;

    /* PATH(ProcessCommandMsg,A); */

    /*shall remove any data link escape characters within the received message */
    for (x = 1, y = 1, Rxbuf[0] = DLE_CHAR; y < GseRxNumBytes; x++, y++)
    {
		/* PATH(ProcessCommandMsg,M); */
        Rxbuf[x] = GseRxBuf[y];

        /* if the current character is not Word 01 of the message and two consecutive words
           contain the data link escape character, skip past the stuffed escape character */

        if (( y != 1) && ((GseRxBuf[y] & 0xFF) == DLE_CHAR) && ((GseRxBuf[y + 1] & 0xFF) == DLE_CHAR))
         {
            /* PATH(ProcessCommandMsg,B); */
            y++;
        }
    }

    /*shall clear out the number of received bytes */
    GseRxNumBytes = 0;

    /*shall typecast the buffer to the command message type */
    commandMsg = (GSE_RX_CMD_MSG *)Rxbuf;

    /*shall validate the message by adding the message checksum to message data words (DLE, EOTX, and "stuffed" characters excluded)
        and verifying the result is 0 */
    chksum = commandMsg->checksum + commandMsg->ChanDataID + commandMsg->DataPktSize +
             commandMsg->u16ChId + commandMsg->RigCommand + commandMsg->TestCommand +
             commandMsg->SerialNumLSW + commandMsg->SerialNumMSW + commandMsg->reserved1 +
             commandMsg->reserved2;

    /* does the checksum indicate valid received data? */
    if ((chksum & 0xFF) == 0x00)
    {
        /* PATH(ProcessCommandMsg,C); */

        /*shall verify that this message was meant for this MCU */
        if ((G_bChannelA == true) && (commandMsg->ChanDataID == GSE_RX_CMD_LFT_ID))
        {
            /* PATH(ProcessCommandMsg,N); */
            msgGood = true;
        }
        else if ((G_bChannelA == false) && (commandMsg->ChanDataID == GSE_RX_CMD_RT_ID))
        {
            /* PATH(ProcessCommandMsg,O); */
            msgGood = true;
        }
        else if (commandMsg->ChanDataID == GSE_RX_CMD_BOTH_ID)
        {
            /* PATH(ProcessCommandMsg,O); */
            msgGood = true;
        }

        /*shall verify that the received data packet size value is correct */
        if ((msgGood == true) && (commandMsg->DataPktSize == COMMAND_PKT_SIZE))
        {
            /* PATH(ProcessCommandMsg,D); */
            TransmitLeft = false;
            TransmitRight = false;
            TransmitBoth = false;

            if (commandMsg->ChanDataID == GSE_RX_CMD_LFT_ID)
            {
                /* PATH(ProcessCommandMsg,E); */
                TransmitLeft = true;
            }
            if (commandMsg->ChanDataID == GSE_RX_CMD_RT_ID)
            {
                /* PATH(ProcessCommandMsg,F); */
                TransmitRight = true;
            }

            if (commandMsg->ChanDataID == GSE_RX_CMD_BOTH_ID)
            {
                /* PATH(ProcessCommandMsg,F); */
                TransmitBoth = true;
            }

            if ((commandMsg->u16ChId & 0x07) == 0x04) /* Both channels */
            {
                /* PATH(ProcessCommandMsg,G); */
                /* Allow FSL commands */
                SpeedOfMotor = 1.0;
            }
            else /* Individual channel (Left or Right) */
            {
                /* PATH(ProcessCommandMsg,H); */
                /* Disable FSL commands */
                //fslValidInput = FSL_In_Invalid;
                //Fsl_PosCommand = FSL_In_Invalid;
            }

            if (((commandMsg->u16ChId & 0xC0) == 0x80) && (McuGetState() == RIG_MODE)) /* Rig mode */
            {
                /* PATH(ProcessCommandMsg,I); */
                mode = RIG_MODE;
            }
            else if (((commandMsg->u16ChId & 0xC0) == 0x40) && (McuGetState() == TEST_MODE)) /* Test mode */
            {
                /* PATH(ProcessCommandMsg,J); */
                mode = TEST_MODE;
            }
            else
            {
                /* PATH(ProcessCommandMsg,K); */
                mode = INVALID_STATE;
            }

            /*shall take action based on the command data word value depending on
              what mode the MCU is currently in (Rig vs. Test) */
            switch (mode)
            {
                case RIG_MODE:
                    /* PATH(ProcessCommandMsg,L); */
                    GseRig_GetCommands();
                    GseRig_ExecuteCommand();
                    SendRigResponse();
                    break;
                case TEST_MODE:
                    /* PATH(ProcessCommandMsg,BJ); */

                    if (Bit_CheckIBIT(commandMsg->TestCommand))
                    {
                        /* PATH(ProcessCommandMsg,BK); */
                        /* Execute IBIT Test */
                        Ibit_Stat.TestID = commandMsg->TestCommand;
                        Ibit_Stat.TestSupport = commandMsg->reserved1;

                        if (Ibit_Stat.tstCmplt == NOT_STARTED)
                        {
                            /* PATH(ProcessCommandMsg,BL); */
                            Ibit_Stat.tstCmplt = IN_PROCESS;
                            IBITFault = false;
                        }

                        TxAllNvm = false;
                        IBIT_Test = true;
                        testModeReturn = 2;
                    }
                    else
                    {
                        /* PATH(ProcessCommandMsg,BM); */
                        testModeReturn = GseTest_ExecuteCommand(commandMsg->TestCommand, commandMsg);
                    }

                    /* PATH(ProcessCommandMsg,BN); */

                    switch (testModeReturn)
                    {
                        case 1:
                            /* PATH(ProcessCommandMsg,BO); */
                            SendTestResponse();
                            break;
                        case 2:
                            /* PATH(ProcessCommandMsg,BP); */
                            SendIBIT_Response();
                            break;
                        case 4:
                            /* PATH(ProcessCommandMsg,BQ); */
                            SendSIPT_Response();
                            break;
                        default:
                            /* PATH(ProcessCommandMsg,BR); */
                            break;
                    }

                    break;
                default:
                    /* PATH(ProcessCommandMsg,BS); */
                    break;
            } /* End switch (mode) */
        } /* End if ((msgGood == true) && (commandMsg->DataPktSize == COMMAND_PKT_SIZE)) */
  	} /* End if ((chksum & 0xFF) == 0x00) */

    /* PATH(ProcessCommandMsg,BT); */
}


/*
    brief Send the interactive rig mode response message

     Purpose:
        This routine can be called to transmit a message containing
        position quadrature counts when the system is in Rig mode.

    retval TX_IN_PROGRESS - the end of message pattern has not been sent yet
    retval TX_DONE - the end of message pattern has been sent

     Preconditions and Assumptions:
        That the system is in Rig mode and is responding to a received command message.


*/
SCI_STATUS SendRigResponse(void)
{
    static SCI_STATUS status = TX_DONE;
    GSE_TX_RIG_MSG * RigMsg;
    static bool_t bDelayRightOneShot = false;

    /* PATH(SendRigResponse,A); */

    /*shall only set the contents of the Rig response message when status indicates a message is done being sent */
    if (status == TX_DONE)
    {
        /* PATH(SendRigResponse,B); */
        /*shall typecast the transmit buffer to the Rig command response message */
        RigMsg = (GSE_TX_RIG_MSG *)GseTxBuf;
        status = TX_IN_PROGRESS;
        RigMsg->BeginDLE = DLE_CHAR;

        /*shall set the channel and MCU ID data words based on the PrimarySide flag */
        if (G_bChannelA == true)
        {
            /* PATH(SendRigResponse,C); */
            RigMsg->ChanDataID = GSE_RIG_LFT_CHN_ID;
            RigMsg->u16ChId = LEFT_CHAN_ID;
        }
        else
        {
            /* PATH(SendRigResponse,D); */
            RigMsg->ChanDataID = GSE_RIG_RT_CHN_ID;
            RigMsg->u16ChId = RIGHT_CHAN_ID;
        }

        /*shall set the appropriate packet size */
        RigMsg->DataPktSize = RIG_TX_PKT_SIZE;
        /*shall set the status of the rig pins */
        RigMsg->u16ChId |= (!RIG_N << 6);

        /* Set the brake cool down status bit */
        if (MotorCmd.BrakeCooling == true)
        {
            /* PATH(SendRigResponse,P); */
            RigMsg->u16ChId |= 0x04;
        }

        /* Set the rig status bits */
        RigMsg->RigStatus = ((Uint16)Rigging_Status.onsideChannel_Rigged);
        RigMsg->RigStatus |= ((Uint16)Rigging_Status.xsideChannel_Rigged << 1);
        RigMsg->RigStatus |= ((Uint16)Rigging_Status.bSkewSnsrCalibrationComplete << 2);
        RigMsg->RigStatus |= ((Uint16)Rigging_Status.rigData_Written << 3);
        RigMsg->RigStatus |= ((Uint16)Rigging_Status.rigVerify_Ready << 4);
        RigMsg->RigStatus |= ((Uint16)Rigging_Status.RSL_Rigged << 5);
        RigMsg->RigStatus |= ((Uint16)Rigging_Status.ESL_Rigged << 6);

        /* Report the Quad Counts tracked by this channel */
        RigMsg->hall1_LSW = (tHall.Position & 0xFF);
        RigMsg->hall1_MSW = ((tHall.Position & 0xFF00) >> 8);
        RigMsg->rvdt_P_POS_LSW = (RVDT_POS & 0xFF);
        RigMsg->rvdt_P_POS_MSW = ((RVDT_POS & 0xFF00) >> 8);

        /* Set the Rig Fault data in the message */
        RigMsg->RigFault = ReportedFaultCode;

        /* Echo the Rig Command received that the MCU is responding to */
        RigMsg->RigCmd = tGseRigCmd.eRigCmd;

        /* Set the special request response data fields */
        RigMsg->SpecialRequestResponse1 = SpecialRequestResponse1;
        RigMsg->SpecialRequestResponse2 = SpecialRequestResponse2;
        RigMsg->SpecialRequestResponse3 = SpecialRequestResponse3;
        RigMsg->SpecialRequestResponse4 = SpecialRequestResponse4;
    }

    if ((TransmitLeft == true && G_bChannelA == true) ||
        (TransmitRight == true && G_bChannelA == false) || (TransmitBoth == true))
    {
        /* PATH(SendRigResponse,N); */
        Transmitting.Rig_Message = true;

        /*shall send the Rig interactive mode response message as fast as the transmit
            FIFO allows for.  The FIFO is filled to capacity once per frame, with
            any bytes leftover saved for the next frame for transmission. For commands to both channels
            the right side is delayed by 10ms to prevent collisions on the bus */

        //if ((G_Channel_ID == GSE_RX_CMD_BOTH_ID) && (PrimarySide == false)) /* One Shot Logic for Right Side with 10ms Delay */
        if ((TransmitBoth == true) && (G_bChannelA == false)) /* One Shot Logic for Right Side with 10ms Delay */
        {
            /* One shot logic */
            if(bDelayRightOneShot == false)
            {
                bDelayRightOneShot = true;
                /* Set 10ms Timer for Right Side */
                Timer_SetTime(&tRightChnlResponseTmr, TIMER_ONESHOT, TIMER_30ms);
            }
            else
            {
                /* Check if timer has elapsed */
                if(Timer_IsExpired(&tRightChnlResponseTmr) == true)
                {
                    /* Send message after delay has elapsed */
                    status = SendGseMessage(RIG_RSP_MSG_SIZE);
                    bDelayRightOneShot = false; /* Reset the one shot */
                }
            }
        }
        else
        {
            /* Send Rig Response for any other combination of Channel ID and command other than BOTH Actuators */
            status = SendGseMessage(RIG_RSP_MSG_SIZE);
        }
    }

    /* PATH(SendRigResponse,O); */
    return status;
}


/*
    brief Send the interactive test mode response message

     Purpose:
        This routine can be called to transmit a message containing
        non-volatile memory data when the system is in Test mode.

    retval TX_IN_PROGRESS - the end of message pattern has not been sent yet
    retval TX_DONE - the end of message pattern has been sent

     Preconditions and Assumptions:
        That the system is in Test mode and is responding to a received command message.
*/
SCI_STATUS SendTestResponse(void)
{
    static SCI_STATUS status = TX_DONE;
    GSE_TX_TEST_MSG *TestMsg;
    Nvm_State_t state = NVM_STATE_DEFAULTS;
	static bool withNvm = false;
	Uint16 recordID = Nvm_GetCurrentStatePointer();

    /* PATH(SendTestResponse,A); */

    /*shall only set the contents of the Test Response message when status indicates a message is done being sent */
    if (status == TX_DONE)
    {
        /* PATH(SendTestResponse,B); */
		withNvm = false;

        /*shall typecast the transmit buffer to the Test command response message */
        TestMsg = (GSE_TX_TEST_MSG *)GseTxBuf;
        status = TX_IN_PROGRESS;
        TestMsg->BeginDLE = DLE_CHAR;

        /*shall set the channel and MCU ID data words based on the PrimarySide flag */
        if (G_bChannelA == true)
        {
            /* PATH(SendTestResponse,C); */
            TestMsg->ChanDataID = GSE_TEST_LFT_CHN_ID;
            TestMsg->FSCUChanID = LEFT_CHAN_ID;
        }
        else
        {
            /* PATH(SendTestResponse,D); */
            TestMsg->ChanDataID = GSE_TEST_RT_CHN_ID;
            TestMsg->FSCUChanID = RIGHT_CHAN_ID;
        }

        /*shall set the packet size */
        TestMsg->DataPktSize = 1;

        /*shall read and set NVM data if the command message requested it */
        if (TxMostRecentNvm == true)
        {
            /* PATH(SendTestResponse,E); */
            TxMostRecentNvm = false;
            if(recordID > 0)
            {
                recordID = recordID - 1;
            }
            else
            {
                recordID = 0;
            }
            if (Nvm_GetLastFailState(&state) != -1)
            {
                /* PATH(SendTestResponse,F); */
                bytecpy_state(TestMsg->NVMData, &state);
                TestMsg->DataPktSize += NVM_BUF_SIZE;
				withNvm = true;
            }
        }
        else if (TxNextRecentNvm == true)
        {
            /* PATH(SendTestResponse,G); */
            TxNextRecentNvm = false;
            if(recordID > 1)
            {
                recordID = recordID - 2;
            }
            else
            {
                recordID = 0;
            }
            if (Nvm_GetNextToLastFailState(&state) != -1)
            {
                /* PATH(SendTestResponse,H); */
                bytecpy_state(TestMsg->NVMData, &state);
                TestMsg->DataPktSize += NVM_BUF_SIZE;
				withNvm = true;
            }
        }
        else if (TxAllNvm == true)
        {
            /* PATH(SendTestResponse,I); */
            NVM_AllDataSent = false;

            /* start index to state records at 'current state pointer' */
            if (nvmidx == 0xFFFF)
            {
                /* PATH(SendTestResponse,J); */
                nvmidx = Nvm_GetCurrentStatePointer();
            }

            Nvm_GetState(nvmidx, &state);
            bytecpy_state(TestMsg->NVMData, &state);
            TestMsg->DataPktSize += NVM_BUF_SIZE;
			withNvm = true;

            /* wrap to the end of the buffer if just sent first record, else decrement index */
            if (nvmidx == 0)
            {
                /* PATH(SendTestResponse,K); */
                nvmidx = MAX_STATE_RECORDS;
            }
            nvmidx--;
            recordID = nvmidx;
            /* done when all NVM has been traversed (back to 'current state pointer') */
            if (nvmidx == Nvm_GetCurrentStatePointer())
            {
                /* PATH(SendTestResponse,M); */
                TxAllNvm = false;
                nvmidx = 0xFFFF;
                NVM_AllDataSent = true;
            }
        }
        TestMsg->RecordIDL = (recordID & 0x00FF);
        TestMsg->RecordIDH = ((recordID >> 8) & 0x00FF);
        TestMsg->FSCUChanID |= ((Uint16)NVM_AllDataSent << 6);
        TestMsg->FSCUChanID |= ((Uint16)NVM_RigInfoCleared << 5);
        TestMsg->FSCUChanID |= ((Uint16)NVM_StateInfoCleared << 4);
        TestMsg->FSCUChanID |= ((Uint16)NVM_FormatComplete << 3);
        //TestMsg->FSCUChanID |= ((Uint16)NVM_ClearInProgress << 2);    // No need for ClearInProgress anymore
    }

    if ((TransmitLeft == true && G_bChannelA == true) ||
        (TransmitRight == true && G_bChannelA == false))
    {
        /* PATH(SendTestResponse,N); */

        /*shall send the Test interactive mode response message as fast as the transmit
            FIFO allows for.  The FIFO is filled to capacity once per frame, with
            any bytes leftover saved for the next frame for transmission.*/
        Transmitting.NVM_Data = true;

        if (withNvm == false)
        {
            /* PATH(SendTestResponse,O); */
            status = SendGseMessage(TEST_RSP_MSG_WO_NVM);
        }
        else
        {
            /* PATH(SendTestResponse,P); */
            status = SendGseMessage(TEST_RSP_MSG_SIZE);
        }
    }

    /* PATH(SendTestResponse,Q); */
    return (status);
}


/*
    brief Bytewise copy to tx buffer (Uint16 array) from state record.  High byte in each word
        of tx buffer is ignored.

	Param[in]	*dest - Array for the specific values being prepared to be trasmitted over the serial bus.
	Param[in]	*state - State data strucuture that is being transmitted over the serial bus

    retval none
    
*/
void bytecpy_state( Uint16 *dest, const Nvm_State_t *state )
{
    /* PATH(bytecpy_state,A); */
    uint16_t i = 0U;

	if (dest && state)
	{
		/* PATH(bytecpy_state,C); */
	    dest[i++] = (state->timecount & 0x00FF);
	    dest[i++] = ((state->timecount >> 8) & 0x00FF);
	    dest[i++] = (state->bootcount & 0x00FF);
	    dest[i++] = ((state->bootcount & 0xFF00) >> 8);
	    dest[i++] = (state->faultId & 0x00FF);
	    dest[i++] = ((state->faultId & 0xFF00) >> 8);
	    dest[i++] = (state->faultData & 0x00FF);
	    dest[i++] = ((state->faultData & 0xFF00) >> 8);
	    dest[i++] = (state->RunTimeCount & 0x000000FF);
	    dest[i++] = ((state->RunTimeCount >> 8) & 0x000000FF);
	    dest[i++] = ((state->RunTimeCount >> 16) & 0x000000FF);
	    dest[i++] = ((state->RunTimeCount >> 24) & 0x000000FF);
	    dest[i++] = (state->Mode & 0x00FF);
	    dest[i++] = ((state->Mode & 0xFF00) >> 8);
	    dest[i++] = (state->rigstatus & 0x00FF);
	    dest[i++] = ((state->rigstatus & 0xFF00) >> 8);
	    dest[i++] = (state->quadposition.onside & 0x00FF);
	    dest[i++] = ((state->quadposition.onside & 0xFF00) >> 8);
#if defined(__HALLX_CONFIGURED)
	    dest[i++] = (state->quadposition.xside & 0x00FF);
	    dest[i++] = ((state->quadposition.xside & 0xFF00) >> 8);
#endif
	    dest[i++] = (state->bitstatus.all & 0x00FF);
	    dest[i++] = ((state->bitstatus.all & 0xFF00) >> 8);
	    dest[i++] = (state->frameOverCount[0] & 0x00FF);
	    dest[i++] = ((state->frameOverCount[0] & 0xFF00) >> 8);
	    dest[i++] = (state->frameOverCount[1] & 0x00FF);
	    dest[i++] = ((state->frameOverCount[1] & 0xFF00) >> 8);
	    dest[i++] = (state->frameOverCount[2] & 0x00FF);
	    dest[i++] = ((state->frameOverCount[2] & 0xFF00) >> 8);
	    dest[i++] = (state->frameOverCount[3] & 0x00FF);
	    dest[i++] = ((state->frameOverCount[3] & 0xFF00) >> 8);
	    dest[i++] = (state->frameOverCount[4] & 0x00FF);
	    dest[i++] = ((state->frameOverCount[4] & 0xFF00) >> 8);
	    dest[i++] = (state->frameOverCount[5] & 0x00FF);
	    dest[i++] = ((state->frameOverCount[5] & 0xFF00) >> 8);
	    dest[i++] = (state->frameOverCount[6] & 0x00FF);
	    dest[i++] = ((state->frameOverCount[6] & 0xFF00) >> 8);
	    dest[i++] = (state->frameOverCount[7] & 0x00FF);
	    dest[i++] = ((state->frameOverCount[7] & 0xFF00) >> 8);
        dest[i++] = (uint16_t)(state->tMcuStatus.all & 0x000000FFUL);
        dest[i++] = (uint16_t)((state->tMcuStatus.all & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)(state->tMcuFault.all & 0x000000FFUL);
        dest[i++] = (uint16_t)((state->tMcuFault.all & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)(state->tMcuMaint.all & 0x000000FFUL);
        dest[i++] = (uint16_t)((state->tMcuMaint.all & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32StrokeFused) & 0x000000FFUL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32BiasFused) & 0x000000FFUL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32QuadCntResidualFused) & 0x000000FFUL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32SkewResidualFused) & 0x000000FFUL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
        dest[i++] = (uint16_t)(state->tPanel.tSf.s16QuadFused & 0x000000FFUL);
        dest[i++] = (uint16_t)((state->tPanel.tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)(state->tPanel.tSf.u16MemoryPad & 0x000000FFUL);
        dest[i++] = (uint16_t)((state->tPanel.tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
        dest[i++] = (uint16_t)(state->tPanel.tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
        dest[i++] = (uint16_t)((state->tPanel.tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)(state->tPanel.tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
        dest[i++] = (uint16_t)((state->tPanel.tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
        dest[i++] = (uint16_t)((*(uint32_t*)&state->tPanel.tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
        dest[i++] = (uint16_t)(state->tPanel.tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
        dest[i++] = (uint16_t)((state->tPanel.tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
        dest[i++] = (uint16_t)(state->tPanel.tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
        dest[i++] = (uint16_t)((state->tPanel.tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
	}
    
    /* PATH(bytecpy_state,B); */
}

/*
    brief Send the interactive test mode response message

    Purpose:
        This routine can be called to transmit a message containing 
        non-volatile memory data when the system is in Test mode.        

    retval TX_IN_PROGRESS - the end of message pattern has not been sent yet
    retval TX_DONE - the end of message pattern has been sent
    
     Preconditions and Assumptions:
        That the system is in Test mode and is responding to a received command message.

   

*/
SCI_STATUS SendIBIT_Response(void)
{
    static SCI_STATUS status = TX_DONE;
    GSE_TX_IBIT_MSG *IBITMsg;

    /* PATH(SendIBIT_Response,A); */
    
    /*shall only set the contents of the Test Response message when status indicates a message is done being sent */
    if (status == TX_DONE)
    {
        /* PATH(SendIBIT_Response,B); */
        
        /*shall typecast the transmit buffer to the Test command response message */
        IBITMsg = (GSE_TX_IBIT_MSG *)GseTxBuf;
        *IBITMsg = Gse_SIPT_Default;
        status = TX_IN_PROGRESS;
        IBITMsg->BeginDLE = DLE_CHAR;

        /*shall set the channel and MCU ID data words based on the PrimarySide flag */
        if (G_bChannelA == true)
        {
            /* PATH(SendIBIT_Response,C); */
            IBITMsg->ChanDataID = GSE_IBIT_LFT_CHN_ID;
        }
        else if (G_bChannelA == false)
        {
            /* PATH(SendIBIT_Response,D); */
            IBITMsg->ChanDataID = GSE_IBIT_RT_CHN_ID;
        }

        /*shall set the packet size */
        IBITMsg->DataPktSize = IBIT_PKT_SIZE;  /* TBD...depends on size of NVMData requested */
    
        if (IBIT_Test == true)
        {
            /* PATH(SendIBIT_Response,E); */
            IBIT_Test = false;
            IBITMsg->SupportDataID = Ibit_Stat.TestID;
            IBITTest_Result(IBITMsg);
        }
        else if (TxRigPage1 == true)
        {
            /* PATH(SendIBIT_Response,F); */
            TxRigPage1 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE1;
            RigAll(IBITMsg);
        }
        else if (TxRigPage2 == true)
        {
            /* PATH(SendIBIT_Response,G); */
            TxRigPage2 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE2;
            RigAll(IBITMsg);
        }
        else if (TxRigPage3 == true)
        {
            /* PATH(SendIBIT_Response,H); */
            TxRigPage3 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE3;
            RigAll(IBITMsg);
        }
        else if (TxRigPage4 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage4 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE4;
            RigAll(IBITMsg);
        }
        else if (TxRigPage5 == true)
        {
            /* PATH(SendIBIT_Response,F); */
            TxRigPage5 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE5;
            RigAll(IBITMsg);
        }
        else if (TxRigPage6 == true)
        {
            /* PATH(SendIBIT_Response,G); */
            TxRigPage6 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE6;
            RigAll(IBITMsg);
        }
        else if (TxRigPage7 == true)
        {
            /* PATH(SendIBIT_Response,H); */
            TxRigPage7 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE7;
            RigAll(IBITMsg);
        }
        else if (TxRigPage8 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage8 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE8;
            RigAll(IBITMsg);
        }
        else if (TxRigPage9 == true)
        {
            /* PATH(SendIBIT_Response,F); */
            TxRigPage9 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE9;
            RigAll(IBITMsg);
        }
        else if (TxRigPage10 == true)
        {
            /* PATH(SendIBIT_Response,G); */
            TxRigPage10 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE10;
            RigAll(IBITMsg);
        }
        else if (TxRigPage11 == true)
        {
            /* PATH(SendIBIT_Response,H); */
            TxRigPage11 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE11;
            RigAll(IBITMsg);
        }
        else if (TxRigPage12 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage12 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE12;
            RigAll(IBITMsg);
        }
        else if (TxRigPage13 == true)
        {
            /* PATH(SendIBIT_Response,F); */
            TxRigPage13 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE13;
            RigAll(IBITMsg);
        }
        else if (TxRigPage14 == true)
        {
            /* PATH(SendIBIT_Response,G); */
            TxRigPage14 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE14;
            RigAll(IBITMsg);
        }
        else if (TxRigPage15 == true)
        {
            /* PATH(SendIBIT_Response,H); */
            TxRigPage15 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE15;
            RigAll(IBITMsg);
        }
        else if (TxRigPage16 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage16 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE16;
            RigAll(IBITMsg);
        }
        else if (TxRigPage17 == true)
        {
            /* PATH(SendIBIT_Response,F); */
            TxRigPage17 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE17;
            RigAll(IBITMsg);
        }
        else if (TxRigPage18 == true)
        {
            /* PATH(SendIBIT_Response,G); */
            TxRigPage18 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE18;
            RigAll(IBITMsg);
        }
        else if (TxRigPage19 == true)
        {
            /* PATH(SendIBIT_Response,H); */
            TxRigPage19 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE19;
            RigAll(IBITMsg);
        }
        else if (TxRigPage20 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage20 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE20;
            RigAll(IBITMsg);
        }
        else if (TxRigPage21 == true)
        {
            /* PATH(SendIBIT_Response,F); */
            TxRigPage21 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE21;
            RigAll(IBITMsg);
        }
        else if (TxRigPage22 == true)
        {
            /* PATH(SendIBIT_Response,G); */
            TxRigPage22 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE22;
            RigAll(IBITMsg);
        }
        else if (TxRigPage23 == true)
        {
            /* PATH(SendIBIT_Response,H); */
            TxRigPage23 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE23;
            RigAll(IBITMsg);
        }
        else if (TxRigPage24 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage24 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE24;
            RigAll(IBITMsg);
        }
        else if (TxRigPage25 == true)
        {
            /* PATH(SendIBIT_Response,H); */
            TxRigPage25 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE25;
            RigAll(IBITMsg);
        }
        else if (TxRigPage26 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage26 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE26;
            RigAll(IBITMsg);
        }
        else if (TxRigPage27 == true)
        {
            /* PATH(SendIBIT_Response,F); */
            TxRigPage27 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE27;
            RigAll(IBITMsg);
        }
        else if (TxRigPage28 == true)
        {
            /* PATH(SendIBIT_Response,G); */
            TxRigPage28 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE28;
            RigAll(IBITMsg);
        }
        else if (TxRigPage29 == true)
        {
            /* PATH(SendIBIT_Response,H); */
            TxRigPage29 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE29;
            RigAll(IBITMsg);
        }
        else if (TxRigPage30 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage30 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE30;
            RigAll(IBITMsg);
        }
        else if (TxRigPage31 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage31 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE31;
            RigAll(IBITMsg);
        }
		else if (TxRigPage32 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage32 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE32;
            RigAll(IBITMsg);
        }
		else if (TxRigPage33 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage33 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE33;
            RigAll(IBITMsg);
        }
		else if (TxRigPage34 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage34 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE34;
            RigAll(IBITMsg);
        }
		else if (TxRigPage35 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage35 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE35;
            RigAll(IBITMsg);
        }
		else if (TxRigPage36 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage36 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE36;
            RigAll(IBITMsg);
        }
		else if (TxRigPage37 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage37 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE37;
            RigAll(IBITMsg);
        }
		else if (TxRigPage38 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage38 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE38;
            RigAll(IBITMsg);
        }
        else if (TxRigPage39 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage39 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE39;
            RigAll(IBITMsg);
        }
        else if (TxRigPage40 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage40 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE40;
            RigAll(IBITMsg);
        }
        else if (TxRigPage41 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage41 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE41;
            RigAll(IBITMsg);
        }
        else if (TxRigPage42 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage42 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE42;
            RigAll(IBITMsg);
        }
        else if (TxRigPage43 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage43 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE43;
            RigAll(IBITMsg);
        }
        else if (TxRigPage44 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage44 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE44;
            RigAll(IBITMsg);
        }
        else if (TxRigPage45 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage45 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE45;
            RigAll(IBITMsg);
        }
        else if (TxRigPage46 == true)
        {
            /* PATH(SendIBIT_Response,I); */
            TxRigPage46 = false;
            IBITMsg->SupportDataID = RIGDATA_PAGE46;
            RigAll(IBITMsg);
        }
    }

    if ((TransmitLeft == true && G_bChannelA == true) ||
        (TransmitRight == true && G_bChannelA == false))
    {
        /* PATH(SendIBIT_Response,J); */
        
        /*shall send the Test interactive mode response message as fast as the transmit
            FIFO allows for.  The FIFO is filled to capacity once per frame, with
            any bytes leftover saved for the next frame for transmission.*/
        Transmitting.IBIT_Message = true;
        status = SendGseMessage(IBIT_RSP_MSG_SIZE);
    }

    /* PATH(SendIBIT_Response,K); */
    return (status);
}
/*
    brief Prepare the IBIT response message for extracing the Rigging information,
        Due to the size the, the data must be transmitted over four consecutive messages

    Param[in] *IBITMsg - Ibit response message data structure, will contain the data to be tranmistted
                        over the serial busses.

    retval None

*/
void RigAll(GSE_TX_IBIT_MSG *IBITMsg)
{
   switch(IBITMsg->SupportDataID)
   {
   case RIGDATA_PAGE1:
             IBITMsg->SupportDW00 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[RSL].u16RvdtPos & 0x00FF);
             IBITMsg->SupportDW01 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[RSL].u16RvdtPos >> 8) & 0x00FF);
             IBITMsg->SupportDW02 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[RSL].u16RvdtSum & 0x00FF);
             IBITMsg->SupportDW03 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[RSL].u16RvdtSum >> 8) & 0x00FF);
             IBITMsg->SupportDW16 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[RSL].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW17 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[RSL].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW20 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[N1].u16RvdtPos & 0x00FF);
             IBITMsg->SupportDW21 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[N1].u16RvdtPos >> 8) & 0x00FF);
             IBITMsg->SupportDW22 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[N1].u16RvdtSum & 0x00FF);
             IBITMsg->SupportDW23 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[N1].u16RvdtSum >> 8) & 0x00FF);
             IBITMsg->SupportDW36 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[N1].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW37 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[N1].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW40 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[ZERO].u16RvdtPos & 0x00FF);
             break;
   case RIGDATA_PAGE2:
             IBITMsg->SupportDW00 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[ZERO].u16RvdtPos >> 8) & 0x00FF);
             IBITMsg->SupportDW01 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[ZERO].u16RvdtSum & 0x00FF);
             IBITMsg->SupportDW02 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[ZERO].u16RvdtSum >> 8) & 0x00FF);
             IBITMsg->SupportDW15 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[ZERO].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW16 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[ZERO].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW19 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P1].u16RvdtPos & 0x00FF);
             IBITMsg->SupportDW20 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P1].u16RvdtPos >> 8) & 0x00FF);
             IBITMsg->SupportDW21 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P1].u16RvdtSum & 0x00FF);
             IBITMsg->SupportDW22 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P1].u16RvdtSum >> 8) & 0x00FF);
             IBITMsg->SupportDW35 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P1].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW36 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P1].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW39 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P2].u16RvdtPos & 0x00FF);
             IBITMsg->SupportDW40 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P2].u16RvdtPos >> 8) & 0x00FF);
             break;
   case RIGDATA_PAGE3:
             IBITMsg->SupportDW00 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P2].u16RvdtSum & 0x00FF);
             IBITMsg->SupportDW01 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P2].u16RvdtSum >> 8) & 0x00FF);
             IBITMsg->SupportDW14 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P2].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW15 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P2].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW34 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P3].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW35 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P3].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW38 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P4].u16RvdtPos & 0x00FF);
             IBITMsg->SupportDW39 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P4].u16RvdtPos >> 8) & 0x00FF);
             IBITMsg->SupportDW40 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P4].u16RvdtSum & 0x00FF);
             break;
   case RIGDATA_PAGE4:
             IBITMsg->SupportDW00 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P4].u16RvdtSum >> 8) & 0x00FF);
             IBITMsg->SupportDW13 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P4].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW14 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P4].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW17 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P5].u16RvdtPos & 0x00FF);
             IBITMsg->SupportDW18 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P5].u16RvdtPos >> 8) & 0x00FF);
             IBITMsg->SupportDW19 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P5].u16RvdtSum & 0x00FF);
             IBITMsg->SupportDW20 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P5].u16RvdtSum >> 8) & 0x00FF);
             IBITMsg->SupportDW33 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P5].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW34 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P5].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW37 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P6].u16RvdtPos & 0x00FF);
             IBITMsg->SupportDW38 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P6].u16RvdtPos >> 8) & 0x00FF);
             IBITMsg->SupportDW39 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P6].u16RvdtSum & 0x00FF);
             IBITMsg->SupportDW40 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P6].u16RvdtSum >> 8) & 0x00FF);
             break;
   case RIGDATA_PAGE5:
             IBITMsg->SupportDW12 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P6].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW13 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P6].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW16 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P7].u16RvdtPos & 0x00FF);
             IBITMsg->SupportDW17 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P7].u16RvdtPos >> 8) & 0x00FF);
             IBITMsg->SupportDW18 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P7].u16RvdtSum & 0x00FF);
             IBITMsg->SupportDW19 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P7].u16RvdtSum >> 8) & 0x00FF);
             IBITMsg->SupportDW32 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P7].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW33 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P7].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW36 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P8].u16RvdtPos & 0x00FF);
             IBITMsg->SupportDW37 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P8].u16RvdtPos >> 8) & 0x00FF);
             IBITMsg->SupportDW38 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P8].u16RvdtSum & 0x00FF);
             IBITMsg->SupportDW39 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P8].u16RvdtSum >> 8) & 0x00FF);
             break;
   case RIGDATA_PAGE6:
             IBITMsg->SupportDW11 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P8].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW12 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P8].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW15 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P9].u16RvdtPos & 0x00FF);
             IBITMsg->SupportDW16 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P9].u16RvdtPos >> 8) & 0x00FF);
             IBITMsg->SupportDW17 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P9].u16RvdtSum & 0x00FF);
             IBITMsg->SupportDW18 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P9].u16RvdtSum >> 8) & 0x00FF);
             IBITMsg->SupportDW31 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P9].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW32 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P9].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW35 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P10].u16RvdtPos & 0x00FF);
             IBITMsg->SupportDW36 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P10].u16RvdtPos >> 8) & 0x00FF);
             IBITMsg->SupportDW37 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P10].u16RvdtSum & 0x00FF);
             IBITMsg->SupportDW38 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P10].u16RvdtSum >> 8) & 0x00FF);
             break;
   case RIGDATA_PAGE7:
             IBITMsg->SupportDW10 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P10].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW11 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P10].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW14 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P11].u16RvdtPos & 0x00FF);
             IBITMsg->SupportDW15 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P11].u16RvdtPos >> 8) & 0x00FF);
             IBITMsg->SupportDW16 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P11].u16RvdtSum & 0x00FF);
             IBITMsg->SupportDW17 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P11].u16RvdtSum >> 8) & 0x00FF);
             IBITMsg->SupportDW30 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[P11].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW31 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[P11].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW34 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[ESL].u16RvdtPos & 0x00FF);
             IBITMsg->SupportDW35 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[ESL].u16RvdtPos >> 8) & 0x00FF);
             IBITMsg->SupportDW36 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[ESL].u16RvdtSum & 0x00FF);
             IBITMsg->SupportDW37 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[ESL].u16RvdtSum >> 8) & 0x00FF);
             break;
   case RIGDATA_PAGE8:
             IBITMsg->SupportDW09 = (tNvm.tData.Nvm_Rigging.tSkewSnsr[ESL].u16RvdtAvg & 0x00FF);
             IBITMsg->SupportDW10 = ((tNvm.tData.Nvm_Rigging.tSkewSnsr[ESL].u16RvdtAvg >> 8) & 0x00FF);
             IBITMsg->SupportDW13 = (tNvm.tData.Nvm_Rigging.quad[RSL].onside & 0x00FF);
             IBITMsg->SupportDW14 = ((tNvm.tData.Nvm_Rigging.quad[RSL].onside >> 8) & 0x00FF);
             IBITMsg->SupportDW15 = (tNvm.tData.Nvm_Rigging.quad[N1].onside & 0x00FF);
             IBITMsg->SupportDW16 = ((tNvm.tData.Nvm_Rigging.quad[N1].onside >> 8) & 0x00FF);
             IBITMsg->SupportDW17 = (tNvm.tData.Nvm_Rigging.quad[ZERO].onside & 0x00FF);
             IBITMsg->SupportDW18 = ((tNvm.tData.Nvm_Rigging.quad[ZERO].onside >> 8) & 0x00FF);
             IBITMsg->SupportDW19 = (tNvm.tData.Nvm_Rigging.quad[P1].onside & 0x00FF);
             IBITMsg->SupportDW20 = ((tNvm.tData.Nvm_Rigging.quad[P1].onside >> 8) & 0x00FF);
             IBITMsg->SupportDW21 = (tNvm.tData.Nvm_Rigging.quad[P2].onside & 0x00FF);
             IBITMsg->SupportDW22 = ((tNvm.tData.Nvm_Rigging.quad[P2].onside >> 8) & 0x00FF);
             IBITMsg->SupportDW23 = (tNvm.tData.Nvm_Rigging.quad[P3].onside & 0x00FF);
             IBITMsg->SupportDW24 = ((tNvm.tData.Nvm_Rigging.quad[P3].onside >> 8) & 0x00FF);
             IBITMsg->SupportDW25 = (tNvm.tData.Nvm_Rigging.quad[P4].onside & 0x00FF);
             IBITMsg->SupportDW26 = ((tNvm.tData.Nvm_Rigging.quad[P4].onside >> 8) & 0x00FF);
             IBITMsg->SupportDW27 = (tNvm.tData.Nvm_Rigging.quad[P5].onside & 0x00FF);
             IBITMsg->SupportDW28 = ((tNvm.tData.Nvm_Rigging.quad[P5].onside >> 8) & 0x00FF);
             IBITMsg->SupportDW29 = (tNvm.tData.Nvm_Rigging.quad[P6].onside & 0x00FF);
             IBITMsg->SupportDW30 = ((tNvm.tData.Nvm_Rigging.quad[P6].onside >> 8) & 0x00FF);
             IBITMsg->SupportDW31 = (tNvm.tData.Nvm_Rigging.quad[P7].onside & 0x00FF);
             IBITMsg->SupportDW32 = ((tNvm.tData.Nvm_Rigging.quad[P7].onside >> 8) & 0x00FF);
             IBITMsg->SupportDW33 = (tNvm.tData.Nvm_Rigging.quad[P8].onside & 0x00FF);
             IBITMsg->SupportDW34 = ((tNvm.tData.Nvm_Rigging.quad[P8].onside >> 8) & 0x00FF);
             IBITMsg->SupportDW35 = (tNvm.tData.Nvm_Rigging.quad[P9].onside & 0x00FF);
             IBITMsg->SupportDW36 = ((tNvm.tData.Nvm_Rigging.quad[P9].onside >> 8) & 0x00FF);
             IBITMsg->SupportDW37 = (tNvm.tData.Nvm_Rigging.quad[P10].onside & 0x00FF);
             IBITMsg->SupportDW38 = ((tNvm.tData.Nvm_Rigging.quad[P10].onside >> 8) & 0x00FF);
             IBITMsg->SupportDW39 = (tNvm.tData.Nvm_Rigging.quad[P11].onside & 0x00FF);
             IBITMsg->SupportDW40 = ((tNvm.tData.Nvm_Rigging.quad[P11].onside >> 8) & 0x00FF);
             break;
   case RIGDATA_PAGE9:
             IBITMsg->SupportDW00 = (tNvm.tData.Nvm_Rigging.quad[ESL].onside & 0x00FF);
             IBITMsg->SupportDW01 = ((tNvm.tData.Nvm_Rigging.quad[ESL].onside >> 8) & 0x00FF);
#if defined(__HALLX_CONFIGURED)
             IBITMsg->SupportDW02 = (tNvm.tData.Nvm_Rigging.quad[RSL].xside & 0x00FF);
             IBITMsg->SupportDW03 = ((tNvm.tData.Nvm_Rigging.quad[RSL].xside >> 8) & 0x00FF);
             IBITMsg->SupportDW04 = (tNvm.tData.Nvm_Rigging.quad[N1].xside & 0x00FF);
             IBITMsg->SupportDW05 = ((tNvm.tData.Nvm_Rigging.quad[N1].xside >> 8) & 0x00FF);
             IBITMsg->SupportDW06 = (tNvm.tData.Nvm_Rigging.quad[ZERO].xside & 0x00FF);
             IBITMsg->SupportDW07 = ((tNvm.tData.Nvm_Rigging.quad[ZERO].xside >> 8) & 0x00FF);
             IBITMsg->SupportDW08 = (tNvm.tData.Nvm_Rigging.quad[P1].xside & 0x00FF);
             IBITMsg->SupportDW09 = ((tNvm.tData.Nvm_Rigging.quad[P1].xside >> 8) & 0x00FF);
             IBITMsg->SupportDW10 = (tNvm.tData.Nvm_Rigging.quad[P2].xside & 0x00FF);
             IBITMsg->SupportDW11 = ((tNvm.tData.Nvm_Rigging.quad[P2].xside >> 8) & 0x00FF);
             IBITMsg->SupportDW12 = (tNvm.tData.Nvm_Rigging.quad[P3].xside & 0x00FF);
             IBITMsg->SupportDW13 = ((tNvm.tData.Nvm_Rigging.quad[P3].xside >> 8) & 0x00FF);
             IBITMsg->SupportDW14 = (tNvm.tData.Nvm_Rigging.quad[P4].xside & 0x00FF);
             IBITMsg->SupportDW15 = ((tNvm.tData.Nvm_Rigging.quad[P4].xside >> 8) & 0x00FF);
             IBITMsg->SupportDW16 = (tNvm.tData.Nvm_Rigging.quad[P5].xside & 0x00FF);
             IBITMsg->SupportDW17 = ((tNvm.tData.Nvm_Rigging.quad[P5].xside >> 8) & 0x00FF);
             IBITMsg->SupportDW18 = (tNvm.tData.Nvm_Rigging.quad[P6].xside & 0x00FF);
             IBITMsg->SupportDW19 = ((tNvm.tData.Nvm_Rigging.quad[P6].xside >> 8) & 0x00FF);
             IBITMsg->SupportDW20 = (tNvm.tData.Nvm_Rigging.quad[P7].xside & 0x00FF);
             IBITMsg->SupportDW21 = ((tNvm.tData.Nvm_Rigging.quad[P7].xside >> 8) & 0x00FF);
             IBITMsg->SupportDW22 = (tNvm.tData.Nvm_Rigging.quad[P8].xside & 0x00FF);
             IBITMsg->SupportDW23 = ((tNvm.tData.Nvm_Rigging.quad[P8].xside >> 8) & 0x00FF);
             IBITMsg->SupportDW24 = (tNvm.tData.Nvm_Rigging.quad[P9].xside & 0x00FF);
             IBITMsg->SupportDW25 = ((tNvm.tData.Nvm_Rigging.quad[P9].xside >> 8) & 0x00FF);
             IBITMsg->SupportDW26 = (tNvm.tData.Nvm_Rigging.quad[P10].xside & 0x00FF);
             IBITMsg->SupportDW27 = ((tNvm.tData.Nvm_Rigging.quad[P10].xside >> 8) & 0x00FF);
             IBITMsg->SupportDW28 = (tNvm.tData.Nvm_Rigging.quad[P11].xside & 0x00FF);
             IBITMsg->SupportDW29 = ((tNvm.tData.Nvm_Rigging.quad[P11].xside >> 8) & 0x00FF);
             IBITMsg->SupportDW30 = (tNvm.tData.Nvm_Rigging.quad[ESL].xside & 0x00FF);
             IBITMsg->SupportDW31 = ((tNvm.tData.Nvm_Rigging.quad[ESL].xside >> 8) & 0x00FF);
#endif
             IBITMsg->SupportDW32 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32StrokeFused) & 0x000000FFUL);
             IBITMsg->SupportDW33 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW34 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW35 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW36 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32BiasFused) & 0x000000FFUL);
             IBITMsg->SupportDW37 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW38 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW39 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW40 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
             break;
   case RIGDATA_PAGE10:
             IBITMsg->SupportDW00 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW01 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW02 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW03 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32SkewResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW04 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW05 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW06 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW07 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.s16QuadFused & 0x000000FFUL);
             IBITMsg->SupportDW08 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW09 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW10 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[RSL].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW11 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
             IBITMsg->SupportDW12 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW13 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW14 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW15 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW16 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW17 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW18 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW19 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW20 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW21 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
             IBITMsg->SupportDW22 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW23 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
             IBITMsg->SupportDW24 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW25 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW26 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW27 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW28 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW29 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW30 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW31 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW32 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW33 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW34 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[RSL].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);

             break;
   case RIGDATA_PAGE11:

             IBITMsg->SupportDW38 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32StrokeFused) & 0x000000FFUL);
             IBITMsg->SupportDW39 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW40 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
             break;
   case RIGDATA_PAGE12:
             IBITMsg->SupportDW00 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW01 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32BiasFused) & 0x000000FFUL);
             IBITMsg->SupportDW02 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW03 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW04 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW05 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW06 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW07 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW08 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW09 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32SkewResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW10 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW11 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW12 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW13 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.s16QuadFused & 0x000000FFUL);
             IBITMsg->SupportDW14 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW15 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW16 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[N1].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW17 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
             IBITMsg->SupportDW18 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW19 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW20 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW21 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW22 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW23 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW24 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW25 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW26 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW27 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
             IBITMsg->SupportDW28 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW29 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
             IBITMsg->SupportDW30 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW31 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW32 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW33 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW34 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW35 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW36 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW37 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW38 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW39 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW40 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[N1].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             break;
   case RIGDATA_PAGE13:

             break;
   case RIGDATA_PAGE14:
             IBITMsg->SupportDW03 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32StrokeFused) & 0x000000FFUL);
             IBITMsg->SupportDW04 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW05 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW06 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW07 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32BiasFused) & 0x000000FFUL);
             IBITMsg->SupportDW08 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW09 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW10 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW11 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW12 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW13 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW14 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW15 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32SkewResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW16 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW17 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW18 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW19 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.s16QuadFused & 0x000000FFUL);
             IBITMsg->SupportDW20 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW21 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW22 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[ZERO].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW23 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
             IBITMsg->SupportDW24 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW25 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW26 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW27 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW28 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW29 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW30 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW31 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW32 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW33 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
             IBITMsg->SupportDW34 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW35 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
             IBITMsg->SupportDW36 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW37 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW38 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW39 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW40 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             break;
   case RIGDATA_PAGE15:
            IBITMsg->SupportDW00 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW01 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW02 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
            IBITMsg->SupportDW03 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW04 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
            IBITMsg->SupportDW05 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[ZERO].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             break;
   case RIGDATA_PAGE16:
            IBITMsg->SupportDW09 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32StrokeFused) & 0x000000FFUL);
            IBITMsg->SupportDW10 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW11 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW12 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW13 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32BiasFused) & 0x000000FFUL);
            IBITMsg->SupportDW14 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW15 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW16 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW17 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
            IBITMsg->SupportDW18 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW19 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW20 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW21 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32SkewResidualFused) & 0x000000FFUL);
            IBITMsg->SupportDW22 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW23 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW24 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW25 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.s16QuadFused & 0x000000FFUL);
            IBITMsg->SupportDW26 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW27 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.u16MemoryPad & 0x000000FFUL);
            IBITMsg->SupportDW28 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P1].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW29 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
            IBITMsg->SupportDW30 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW31 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW32 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW33 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
            IBITMsg->SupportDW34 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW35 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW36 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW37 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
            IBITMsg->SupportDW38 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW39 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
            IBITMsg->SupportDW40 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
            break;
   case RIGDATA_PAGE17:
            IBITMsg->SupportDW00 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
            IBITMsg->SupportDW01 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW02 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW03 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW04 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
            IBITMsg->SupportDW05 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW06 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW07 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW08 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
            IBITMsg->SupportDW09 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW10 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
            IBITMsg->SupportDW11 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P1].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
            break;
   case RIGDATA_PAGE18:
            IBITMsg->SupportDW15 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32StrokeFused) & 0x000000FFUL);
            IBITMsg->SupportDW16 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW17 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW18 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW19 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32BiasFused) & 0x000000FFUL);
            IBITMsg->SupportDW20 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW21 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW22 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW23 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
            IBITMsg->SupportDW24 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW25 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW26 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW27 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32SkewResidualFused) & 0x000000FFUL);
            IBITMsg->SupportDW28 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW29 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW30 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW31 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.s16QuadFused & 0x000000FFUL);
            IBITMsg->SupportDW32 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW33 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.u16MemoryPad & 0x000000FFUL);
            IBITMsg->SupportDW34 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P2].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW35 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
            IBITMsg->SupportDW36 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW37 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW38 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW39 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
            IBITMsg->SupportDW40 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
            break;
   case RIGDATA_PAGE19:
            IBITMsg->SupportDW00 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW01 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW02 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
            IBITMsg->SupportDW03 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW04 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
            IBITMsg->SupportDW05 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW06 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
            IBITMsg->SupportDW07 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW08 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW09 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW10 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
            IBITMsg->SupportDW11 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW12 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW13 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW14 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
            IBITMsg->SupportDW15 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW16 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
            IBITMsg->SupportDW17 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P2].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
            break;
   case RIGDATA_PAGE20:
            IBITMsg->SupportDW21 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32StrokeFused) & 0x000000FFUL);
            IBITMsg->SupportDW22 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW23 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW24 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW25 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32BiasFused) & 0x000000FFUL);
            IBITMsg->SupportDW26 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW27 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW28 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW29 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
            IBITMsg->SupportDW30 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW31 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW32 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW33 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32SkewResidualFused) & 0x000000FFUL);
            IBITMsg->SupportDW34 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW35 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW36 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW37 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.s16QuadFused & 0x000000FFUL);
            IBITMsg->SupportDW38 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW39 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.u16MemoryPad & 0x000000FFUL);
            IBITMsg->SupportDW40 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P3].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
            break;
   case RIGDATA_PAGE21:
            IBITMsg->SupportDW00 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
            IBITMsg->SupportDW01 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW02 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW03 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW04 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
            IBITMsg->SupportDW05 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW06 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW07 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW08 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
            IBITMsg->SupportDW09 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW10 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
            IBITMsg->SupportDW11 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW12 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
            IBITMsg->SupportDW13 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW14 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW15 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW16 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
            IBITMsg->SupportDW17 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW18 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW19 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW20 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
            IBITMsg->SupportDW21 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW22 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
            IBITMsg->SupportDW23 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P3].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
            break;
   case RIGDATA_PAGE22:
            IBITMsg->SupportDW28 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW29 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW30 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW31 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32BiasFused) & 0x000000FFUL);
            IBITMsg->SupportDW32 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW33 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW34 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW35 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
            IBITMsg->SupportDW36 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW37 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW38 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW39 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32SkewResidualFused) & 0x000000FFUL);
            IBITMsg->SupportDW40 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
            break;
   case RIGDATA_PAGE23:
             IBITMsg->SupportDW00 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW01 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW02 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.s16QuadFused & 0x000000FFUL);
             IBITMsg->SupportDW03 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW04 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW05 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P4].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW06 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
             IBITMsg->SupportDW07 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW08 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW09 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW10 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW11 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW12 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW13 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW14 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW15 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW16 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
             IBITMsg->SupportDW17 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW18 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
             IBITMsg->SupportDW19 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW20 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW21 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW22 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW23 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW24 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW25 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW26 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW27 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW28 = (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW29 = (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P4].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             break;
   case RIGDATA_PAGE24:
             IBITMsg->SupportDW33 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32StrokeFused) & 0x000000FFUL);
             IBITMsg->SupportDW34 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW35 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW36 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW37 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32BiasFused) & 0x000000FFUL);
             IBITMsg->SupportDW38 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW39 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW40 = (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
             break;
    case RIGDATA_PAGE25:
             IBITMsg->SupportDW00  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW01  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW02  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW03  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW04  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32SkewResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW05  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW06  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW07  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW08  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.s16QuadFused & 0x000000FFUL);
             IBITMsg->SupportDW09  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW10  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW11  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P5].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW12  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
             IBITMsg->SupportDW13  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW14  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW15  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW16  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW17  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW18  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW19  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW20  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW21  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW22  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
             IBITMsg->SupportDW23  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW24  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
             IBITMsg->SupportDW25  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW26  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW27  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW28  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW29  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW30  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW31  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW32  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW33  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW34  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW35  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P5].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             break;
    case RIGDATA_PAGE26:
             IBITMsg->SupportDW39  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32StrokeFused) & 0x000000FFUL);
             IBITMsg->SupportDW40  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
             break;
    case RIGDATA_PAGE27:
             IBITMsg->SupportDW00  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW01  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW02  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32BiasFused) & 0x000000FFUL);
             IBITMsg->SupportDW03  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW04  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW05  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW06  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW07  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW08  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW09  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW10  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32SkewResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW11  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW12  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW13  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW14  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.s16QuadFused & 0x000000FFUL);
             IBITMsg->SupportDW15  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW16  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW17  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P6].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW18  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
             IBITMsg->SupportDW19  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW20  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW21  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW22  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW23  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW24  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW25  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW26  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW27  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW28  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
             IBITMsg->SupportDW29  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW30  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
             IBITMsg->SupportDW31  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW32  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW33  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW34  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW35  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW36  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW37  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW38  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW39  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW40  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
             break;
    case RIGDATA_PAGE28:
             IBITMsg->SupportDW00  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P6].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             break;
    case RIGDATA_PAGE29:
             IBITMsg->SupportDW04  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32StrokeFused) & 0x000000FFUL);
             IBITMsg->SupportDW05  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW06  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW07  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW08  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32BiasFused) & 0x000000FFUL);
             IBITMsg->SupportDW09  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW10  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW11  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW12  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW13  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW14  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW15  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW16  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32SkewResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW17  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW18  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW19  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW20  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.s16QuadFused & 0x000000FFUL);
             IBITMsg->SupportDW21  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW22  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW23  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P7].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW24  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
             IBITMsg->SupportDW25  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW26  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW27  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW28  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW29  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW30  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW31  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW32  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW33  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW34  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
             IBITMsg->SupportDW35  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW36  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
             IBITMsg->SupportDW37  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW38  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW39  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW40  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             break;
    case RIGDATA_PAGE30:
             IBITMsg->SupportDW00  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW01  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW02  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW03  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW04  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW05  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW06  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P7].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             break;
    case RIGDATA_PAGE31:
             IBITMsg->SupportDW10  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32StrokeFused) & 0x000000FFUL);
             IBITMsg->SupportDW11  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW12  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW13  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW14  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32BiasFused) & 0x000000FFUL);
             IBITMsg->SupportDW15  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW16  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW17  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW18  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW19  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW20  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW21  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW22  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32SkewResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW23  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW24  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW25  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW26  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.s16QuadFused & 0x000000FFUL);
             IBITMsg->SupportDW27  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW28  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW29  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P8].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW30  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
             IBITMsg->SupportDW31  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW32  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW33  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW34  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW35  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW36  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW37  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW38  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW39  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW40  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
             break;
    case RIGDATA_PAGE32:
             IBITMsg->SupportDW00  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW01  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
             IBITMsg->SupportDW02  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW03  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW04  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW05  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW06  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW07  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW08  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW09  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW10  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW11  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW12  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P8].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             break;
    case RIGDATA_PAGE33:
             IBITMsg->SupportDW16  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32StrokeFused) & 0x000000FFUL);
             IBITMsg->SupportDW17  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW18  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW19  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW20  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32BiasFused) & 0x000000FFUL);
             IBITMsg->SupportDW21  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW22  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW23  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW24  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW25  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW26  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW27  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW28  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32SkewResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW29  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW30  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW31  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW32  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.s16QuadFused & 0x000000FFUL);
             IBITMsg->SupportDW33  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW34  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW35  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P9].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW36  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
             IBITMsg->SupportDW37  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW38  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW39  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW40  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             break;
    case RIGDATA_PAGE34:
             IBITMsg->SupportDW00  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW01  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW02  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW03  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW04  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW05  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
             IBITMsg->SupportDW06  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW07  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
             IBITMsg->SupportDW08  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW09  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW10  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW11  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW12  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW13  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW14  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW15  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW16  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW17  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW18  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P9].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             break;
    case RIGDATA_PAGE35:
             IBITMsg->SupportDW22  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32StrokeFused) & 0x000000FFUL);
             IBITMsg->SupportDW23  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW24  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW25  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW26  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32BiasFused) & 0x000000FFUL);
             IBITMsg->SupportDW27  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW28  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW29  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW30  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW31  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW32  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW33  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW34  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32SkewResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW35  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW36  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW37  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW38  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.s16QuadFused & 0x000000FFUL);
             IBITMsg->SupportDW39  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW40  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.u16MemoryPad & 0x000000FFUL);
             break;
    case RIGDATA_PAGE36:
             IBITMsg->SupportDW00  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P10].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW01  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
             IBITMsg->SupportDW02  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW03  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW04  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW05  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW06  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW07  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW08  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW09  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW10  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW11  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
             IBITMsg->SupportDW12  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW13  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
             IBITMsg->SupportDW14  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW15  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW16  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW17  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW18  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW19  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW20  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW21  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW22  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW23  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW24  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P10].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             break;
    case RIGDATA_PAGE37:
             IBITMsg->SupportDW28  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32StrokeFused) & 0x000000FFUL);
             IBITMsg->SupportDW29  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW30  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW31  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW32  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32BiasFused) & 0x000000FFUL);
             IBITMsg->SupportDW33  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW34  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW35  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW36  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW37  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW38  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW39  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW40  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32SkewResidualFused) & 0x000000FFUL);
             break;
    case RIGDATA_PAGE38:
             IBITMsg->SupportDW00  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW01  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW02  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW03  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.s16QuadFused & 0x000000FFUL);
             IBITMsg->SupportDW04  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW05  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW06  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P11].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW07  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
             IBITMsg->SupportDW08  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW09  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW10  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW11  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW12  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW13  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW14  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW15  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW16  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW17  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
             IBITMsg->SupportDW18  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW19  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
             IBITMsg->SupportDW20  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW21  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW22  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW23  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW24  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW25  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW26  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW27  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW28  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW29  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW30  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[P11].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             break;
    case RIGDATA_PAGE39:
             IBITMsg->SupportDW34  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32StrokeFused) & 0x000000FFUL);
             IBITMsg->SupportDW35  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW36  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW37  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW38  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32BiasFused) & 0x000000FFUL);
             IBITMsg->SupportDW39  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW40  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
             break;
    case RIGDATA_PAGE40:
             IBITMsg->SupportDW00  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW01  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32QuadCntResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW02  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW03  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW04  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW05  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32SkewResidualFused) & 0x000000FFUL);
             IBITMsg->SupportDW06  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW07  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW08  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW09  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.s16QuadFused & 0x000000FFUL);
             IBITMsg->SupportDW10  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW11  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW12  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[ESL].tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW13  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
             IBITMsg->SupportDW14  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW15  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW16  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW17  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW18  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW19  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW20  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW21  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW22  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW23  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
             IBITMsg->SupportDW24  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW25  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
             IBITMsg->SupportDW26  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW27  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW28  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW29  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
             IBITMsg->SupportDW30  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW31  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
             IBITMsg->SupportDW32  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
             IBITMsg->SupportDW33  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
             IBITMsg->SupportDW34  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
             IBITMsg->SupportDW35  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
             IBITMsg->SupportDW36  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPanel[ESL].tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);
             break;
    case RIGDATA_PAGE41:
             IBITMsg->SupportDW40  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a0) & 0x000000FFUL);
             break;
    case RIGDATA_PAGE42:
            IBITMsg->SupportDW00  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a0 & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW01  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a0 & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW02  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a0 & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW03  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a1) & 0x000000FFUL);
            IBITMsg->SupportDW04  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a1 & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW05  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a1 & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW06  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a1 & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW07  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a2) & 0x000000FFUL);
            IBITMsg->SupportDW08  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a2 & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW09  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a2 & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW10  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a2 & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW11  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a3) & 0x000000FFUL);
            IBITMsg->SupportDW12  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a3 & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW13  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a3 & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW14  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Rigging.tRvdtCalibration.f32a3 & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW31  =  (uint16_t)(tNvm.tData.Nvm_Rigging.tPrgm.all & 0x00FF);
            IBITMsg->SupportDW32  =  (uint16_t)((tNvm.tData.Nvm_Rigging.tPrgm.all >> 8) & 0x00FF);
            IBITMsg->SupportDW37  =  (uint16_t)(tNvm.tData.Nvm_Rigging.crc & 0x00FF);
            IBITMsg->SupportDW38  =  (uint16_t)((tNvm.tData.Nvm_Rigging.crc >> 8) & 0x00FF);
            IBITMsg->SupportDW39  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_RunTimeCount) & 0x000000FFUL);
            IBITMsg->SupportDW40  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_RunTimeCount & 0x0000FF00UL) >> 8UL);

             break;
    case RIGDATA_PAGE43:
            IBITMsg->SupportDW00  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_RunTimeCount & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW01  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_RunTimeCount & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW02  =  (uint16_t)(tNvm.tData.Nvm_State_Pointer & 0x00FF);
            IBITMsg->SupportDW03  =  (uint16_t)((tNvm.tData.Nvm_State_Pointer >> 8) & 0x00FF);
            IBITMsg->SupportDW04  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.timecount & 0x00FF);
            IBITMsg->SupportDW05  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.timecount >> 8) & 0x00FF);

            IBITMsg->SupportDW08  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.bootcount & 0x00FF);
            IBITMsg->SupportDW09  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.bootcount >> 8) & 0x00FF);
            IBITMsg->SupportDW10  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.faultId & 0x00FF);
            IBITMsg->SupportDW11  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.faultId >> 8) & 0x00FF);
            IBITMsg->SupportDW12  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.faultData & 0x00FF);
            IBITMsg->SupportDW13  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.faultData >> 8) & 0x00FF);
            IBITMsg->SupportDW16  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.RunTimeCount) & 0x000000FFUL);
            IBITMsg->SupportDW17  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.RunTimeCount & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW18  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.RunTimeCount & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW19  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.RunTimeCount & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW20  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.Mode & 0x00FF);
            IBITMsg->SupportDW21  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.rigstatus & 0x00FF);
            IBITMsg->SupportDW22  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.quadposition.onside & 0x00FF);
            IBITMsg->SupportDW23  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.quadposition.onside >> 8) & 0x00FF);
#if defined(__HALLX_CONFIGURED)
            IBITMsg->SupportDW24  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.quadposition.xside & 0x00FF);
            IBITMsg->SupportDW25  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.quadposition.xside >> 8) & 0x00FF);
#endif
            IBITMsg->SupportDW26  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.bitstatus.all & 0x00FF);
            IBITMsg->SupportDW27  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.bitstatus.all >> 8) & 0x00FF);
            IBITMsg->SupportDW28  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.frameOverCount[0] & 0x00FF);
            IBITMsg->SupportDW29  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.frameOverCount[0] >> 8) & 0x00FF);
            IBITMsg->SupportDW30  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.frameOverCount[1] & 0x00FF);
            IBITMsg->SupportDW31  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.frameOverCount[1] >> 8) & 0x00FF);
            IBITMsg->SupportDW32  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.frameOverCount[2] & 0x00FF);
            IBITMsg->SupportDW33  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.frameOverCount[2] >> 8) & 0x00FF);
            IBITMsg->SupportDW34  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.frameOverCount[3] & 0x00FF);
            IBITMsg->SupportDW35  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.frameOverCount[3] >> 8) & 0x00FF);
            IBITMsg->SupportDW36  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.frameOverCount[4] & 0x00FF);
            IBITMsg->SupportDW37  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.frameOverCount[4] >> 8) & 0x00FF);
            IBITMsg->SupportDW38  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.frameOverCount[5] & 0x00FF);
            IBITMsg->SupportDW39  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.frameOverCount[5] >> 8) & 0x00FF);
            IBITMsg->SupportDW40  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.frameOverCount[6] & 0x00FF);

             break;
    case RIGDATA_PAGE44:
            IBITMsg->SupportDW00  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.frameOverCount[6] >> 8) & 0x00FF);
            IBITMsg->SupportDW01  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.frameOverCount[7] & 0x00FF);
            IBITMsg->SupportDW02  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.frameOverCount[7] >> 8) & 0x00FF);
            IBITMsg->SupportDW03  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.tMcuStatus.all & 0x000000FFUL);
            IBITMsg->SupportDW04  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.tMcuStatus.all & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW05  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.tMcuFault.all & 0x000000FFUL);
            IBITMsg->SupportDW06  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.tMcuFault.all & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW07  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.tMcuMaint.all & 0x000000FFUL);
            IBITMsg->SupportDW08  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.tMcuMaint.all & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW09  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32StrokeFused) & 0x000000FFUL);
            IBITMsg->SupportDW10  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32StrokeFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW11  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32StrokeFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW12  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32StrokeFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW13  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32BiasFused) & 0x000000FFUL);
            IBITMsg->SupportDW14  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32BiasFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW15  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32BiasFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW16  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32BiasFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW17  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32QuadCntResidualFused) & 0x000000FFUL);
            IBITMsg->SupportDW18  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32QuadCntResidualFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW19  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32QuadCntResidualFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW20  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32QuadCntResidualFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW21  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32SkewResidualFused) & 0x000000FFUL);
            IBITMsg->SupportDW22  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32SkewResidualFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW23  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32SkewResidualFused & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW24  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.f32SkewResidualFused & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW25  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.s16QuadFused & 0x000000FFUL);
            IBITMsg->SupportDW26  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.s16QuadFused & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW27  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.u16MemoryPad & 0x000000FFUL);
            IBITMsg->SupportDW28  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.tPanel.tSf.u16MemoryPad & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW29  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tStroke.f32Stroke) & 0x000000FFUL);
            IBITMsg->SupportDW30  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tStroke.f32Stroke & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW31  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tStroke.f32Stroke & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW32  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tStroke.f32Stroke & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW33  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tStroke.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
            IBITMsg->SupportDW34  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW35  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW36  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tStroke.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW37  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tStroke.s16AdcAvgRaw & 0x000000FFUL);
            IBITMsg->SupportDW38  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tStroke.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW39  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tStroke.s16QuadCnt & 0x000000FFUL);
            IBITMsg->SupportDW40  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tStroke.s16QuadCnt & 0x0000FF00UL) >> 8UL);

             break;
     case RIGDATA_PAGE45:
            IBITMsg->SupportDW00  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tFlapAngle.f32FlapAngle) & 0x000000FFUL);
            IBITMsg->SupportDW01  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tFlapAngle.f32FlapAngle & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW02  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tFlapAngle.f32FlapAngle & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW03  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tFlapAngle.f32FlapAngle & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW04  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated) & 0x000000FFUL);
            IBITMsg->SupportDW05  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW06  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW07  =  (uint16_t)((*(uint32_t*)&tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tFlapAngle.f32RvdtAdcAvgCalibrated & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW08  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tFlapAngle.s16AdcAvgRaw & 0x000000FFUL);
            IBITMsg->SupportDW09  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tFlapAngle.s16AdcAvgRaw & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW10  =  (uint16_t)(tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tFlapAngle.u16MemoryPad & 0x000000FFUL);
            IBITMsg->SupportDW11  =  (uint16_t)((tNvm.tData.Nvm_Powerdown_State.tPanel.tRvdt.tFlapAngle.u16MemoryPad & 0x0000FF00UL) >> 8UL);

             break;
     case RIGDATA_PAGE46:
            IBITMsg->SupportDW15  =  (uint16_t)(tNvm.tData.Nvm_BoardSerialNumber & 0x00FF);
            IBITMsg->SupportDW16  =  (uint16_t)((tNvm.tData.Nvm_BoardSerialNumber >> 8) & 0x00FF);
            IBITMsg->SupportDW17  =  (uint16_t)(tNvm.tData.WDRS & 0x00FF);
            IBITMsg->SupportDW18  =  (uint16_t)((tNvm.tData.WDRS >> 8) & 0x00FF);
            IBITMsg->SupportDW19  =  (uint16_t)(tNvm.tData.WDRS_Counter & 0x00FF);
            IBITMsg->SupportDW20  =  (uint16_t)((tNvm.tData.WDRS_Counter >> 8) & 0x00FF);
            IBITMsg->SupportDW21  =  (uint16_t)(tNvm.tData.tCH_ID & 0x00FF);
            IBITMsg->SupportDW22  =  (uint16_t)(tNvm.tData.tLRU_ID & 0x00FF);
            IBITMsg->SupportDW23  =  (uint16_t)((*(uint32_t*)&tNvm.tData.f32PositionCmdOld) & 0x000000FFUL);
            IBITMsg->SupportDW24  =  (uint16_t)((*(uint32_t*)&tNvm.tData.f32PositionCmdOld & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW25  =  (uint16_t)((*(uint32_t*)&tNvm.tData.f32PositionCmdOld & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW26  =  (uint16_t)((*(uint32_t*)&tNvm.tData.f32PositionCmdOld & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW27  =  (uint16_t)((*(uint32_t*)&tNvm.tData.f32SpeedCmdOld) & 0x000000FFUL);
            IBITMsg->SupportDW28  =  (uint16_t)((*(uint32_t*)&tNvm.tData.f32SpeedCmdOld & 0x0000FF00UL) >> 8UL);
            IBITMsg->SupportDW29  =  (uint16_t)((*(uint32_t*)&tNvm.tData.f32SpeedCmdOld & 0x00FF0000UL) >> 16UL);
            IBITMsg->SupportDW30  =  (uint16_t)((*(uint32_t*)&tNvm.tData.f32SpeedCmdOld & 0xFF000000UL) >> 24UL);
            IBITMsg->SupportDW31  =  0; // Reserved;
            IBITMsg->SupportDW32  =  0; // Reserved;
            IBITMsg->SupportDW33  =  0; // Reserved;
            IBITMsg->SupportDW34  =  0; // Reserved;
            IBITMsg->SupportDW35  =  0; // Reserved;
            IBITMsg->SupportDW36  =  0; // Reserved;
            IBITMsg->SupportDW37  =  0; // Reserved;
            IBITMsg->SupportDW38  =  0; // Reserved;
            IBITMsg->SupportDW39  =  0; // Reserved;
            IBITMsg->SupportDW40  =  0; // Reserved;
             break;
    default:
             break;
   }
}

/*
    brief Prepare the IBIT response message for extracing the data to support an IBIT test that
			is either executuing or has recently complted.
 
 	Param[in] *IBITMsg - Ibit response message data structure, will contain the data to be tranmistted
						over the serial busses.

	retval None  

*/
void IBITTest_Result(GSE_TX_IBIT_MSG *IBITMsg)
{
    /* PATH(IBITTest_Result,A); */
    /*Copy the results of the IBIT test to the Message data structure*/
	IBITMsg->SupportDW00 = (Uint16)(((Ibit_Stat.result & 0x0F) << 4) | (Ibit_Stat.tstCmplt & 0x0F));
    IBITMsg->SupportDW01 = (tHall.Position & 0x00FF);
    IBITMsg->SupportDW02 = ((tHall.Position >> 8) & 0x00FF);
    
    if ((Ibit_Stat.result == 0)    & (Ibit_Stat.TestID != 0x07) & (Ibit_Stat.TestID != 0xC0))
    /* If monitor has NOT tripped & command is not 0x07 or 0xC0 */ 
    {
    	/* PATH(IBITTest_Result,C); */
	  IBITMsg->SupportDW03 = 0;
      IBITMsg->SupportDW04 = 0;
      IBITMsg->SupportDW05 = 0;
      IBITMsg->SupportDW06 = 0;    		
  	}
    else	/* If monitor HAS tripped */
    {
      /* PATH(IBITTest_Result,D); */
      IBITMsg->SupportDW03 = (Ibit_Stat.TestData & 0x00FF);
      IBITMsg->SupportDW04 = ((Ibit_Stat.TestData >> 8) & 0x00FF);
      IBITMsg->SupportDW05 = ((Ibit_Stat.TestData >> 16) & 0x00FF);
      IBITMsg->SupportDW06 = ((Ibit_Stat.TestData >> 24) & 0x00FF);
  	}
    /* PATH(IBITTest_Result,B); */
}


/*
    brief Send the SIPT interactive mode response message

     Purpose:
        This routine can be called to transmit a message containing 
        the SIPT information when the system is in Rig mode.  

    retval TX_IN_PROGRESS - the end of message pattern has not been sent yet
    retval TX_DONE - the end of message pattern has been sent
    
     Preconditions and Assumptions:
        That the system is in Rig mode and is responding to a received command message.

*/
SCI_STATUS SendSIPT_Response(void)
{
    static SCI_STATUS status = TX_DONE;
    GSE_TX_SIPT_MSG * SIPT_Msg;
    unsigned int *pAddr = (unsigned int *)BIT_ROMTEST_CRC_ADDR;
    Uint32 CRC_ID = 0;
    t16ByteBits t16Temp;

    /* PATH(SendSIPT_Response,A); */

    /*shall only set the contents of the Test Response message when status indicates a message is done being sent */
    if (status == TX_DONE)
    {
        /* PATH(SendSIPT_Response,B); */
        /*shall typecast the transmit buffer to the Test command response message */
        SIPT_Msg = (GSE_TX_SIPT_MSG *)GseTxBuf;
        status = TX_IN_PROGRESS;
        SIPT_Msg->BeginDLE = DLE_CHAR;

        /*shall set the channel ID data word based on the PrimarySide flag */
        if (G_bChannelA == true)
        {
            /* PATH(SendSIPT_Response,C); */
            SIPT_Msg->ChanDataID = GSE_SIPT_LFT_CHN_ID;
        }
        else
        {
            /* PATH(SendSIPT_Response,D); */
            SIPT_Msg->ChanDataID = GSE_SIPT_RT_CHN_ID;
        }

        /*shall set the packet size */
        SIPT_Msg->DataPktSize = SIPT_PKT_SIZE;

        /*shall read NVM for board serial number */
        SIPT_Msg->SerialNumLow = (tNvm.tData.Nvm_BoardSerialNumber & 0xFF);
        SIPT_Msg->SerialNumHigh = ((tNvm.tData.Nvm_BoardSerialNumber & 0xFF00) >> 8);

        /*shall retrieve ROM CRC value (32-bit) */
        CRC_ID = (unsigned long)*pAddr;
        CRC_ID += (((unsigned long)*(pAddr + 1)) << 16);

        SIPT_Msg->ROM_CRC_1 = (CRC_ID & 0x000000FF);
        SIPT_Msg->ROM_CRC_2 = (((CRC_ID) >> 8) & 0x000000FF);
        SIPT_Msg->ROM_CRC_3 = (((CRC_ID) >> 16) & 0x000000FF);
        SIPT_Msg->ROM_CRC_4 = (((CRC_ID) >> 24) & 0x000000FF);

        /*shall retrieve the software version and model letter, -14V_MON, and PRGM pins */
        SIPT_Msg->SoftVersion = 0x00; /* x.x of wx.x (ex. 0x72 -> w7.2) */
		SIPT_Msg->SW_VerDisc = (PRGM1 << 1U);
		SIPT_Msg->SW_VerDisc |= (PRGM2 << 2U);
		SIPT_Msg->SW_VerDisc |= (PRGM3 << 3U);
		SIPT_Msg->SW_VerDisc |= (PRGM4 << 4U);
		SIPT_Msg->SW_VerDisc |= (PRGM5 << 5U);
		SIPT_Msg->SW_VerDisc |= (PRGM6 << 6U);


        /*shall retrieve the discrete signals */
		t16Temp.all = 0U;
        t16Temp.bit.b0 = CHA_HS1_RAW;    /* bit 0 */
        t16Temp.bit.b1 = CHA_HS2_RAW;    /* bit 1 */
        t16Temp.bit.b2 = CHA_HS3_RAW;    /* bit 2 */
        //t16Temp.bit.b3 = SCU_MCU_ENABLE;   /* bit 3 */
        //t16Temp.bit.b4 = CHB_ICC_HS1_RAW;  /* bit 4 */
        //t16Temp.bit.b5 = CHB_ICC_HS2_RAW;  /* bit 5 */
		//t16Temp.bit.b6 = CHB_ICC_HS3_RAW;	 /* bit 6 */

		SIPT_Msg->DiscreteSigs = t16Temp.all;

        t16Temp.all = 0U;
        t16Temp.bit.b6 = !RIG_N;    /* bit 6 */
        SIPT_Msg->u16RigPins = t16Temp.all;

        /*shall retrieve the primary sum */         
        SIPT_Msg->RvdtSumLow = (RVDT_SUM & 0xFF);
        SIPT_Msg->RvdtSumHigh = ((RVDT_SUM & 0xFF00) >> 8);

        /*shall retrieve the primary pos */
        SIPT_Msg->RvdtPosLow = (RVDT_POS & 0xFF);
        SIPT_Msg->RvdtPosHigh = ((RVDT_POS & 0xFF00) >> 8);
    }

    if ((TransmitLeft == true && G_bChannelA == true) ||
        (TransmitRight == true && G_bChannelA == false))
    {
        /* PATH(SendSIPT_Response,E); */
        /*shall send the SIPT interactive mode response message as fast as the transmit
            FIFO allows for.  The FIFO is filled to capacity once per frame, with
            any bytes leftover saved for the next frame for transmission.*/
        Transmitting.SIPT_Message = true;
        status = SendGseMessage(SIPT_RSP_MSG_SIZE);
    }

    /* PATH(SendSIPT_Response,F); */
    
    return (status);
}


/*
    brief Send GSE message via serial port.

     Purpose:
        Send a GSE message via SCI port B.  Uses the serial port's FIFO to send all
        message bytes as a single burst.  Garmin DLE stuffing is handled as the individual
        message bytes are loaded into the transmit buffer.  The ending DLE character as well as 
        the end of transmission character are automatically inserted here.  

    param[in] length  the length of the message to send, not counting the checksum, closing 
                       data link escape and end of transmission

    retval TX_IN_PROGRESS - the end of message pattern has not been sent yet
    retval TX_DONE - the end of message pattern has been sent
    
     Preconditions and Assumptions:
         None.

*/
SCI_STATUS SendGseMessage(Uint16 length)
{   /*Contents of this function are commented out as in MCU the GSE functionality will be moved from RS485 to ARINC*/
    SCI_STATUS status = TX_IN_PROGRESS;

    static Uint16 checksum = 0;

    /* PATH(SendGseMessage,A); */
    /* Disable the Transceiver Receive Functionality to Enable the Transmit function. */
    Gse_DisableRxLine();

    /* loop through the message buffer */
    /*shall verify there is still room in the FIFO for at least 2 bytes, otherwise exit */
    while ((GseTxIndex < length) && (GseSciRegs.SCIFFTX.bit.TXFFST <= MAX_STUFFED_FIFO_COUNT))
    {
        /* PATH(SendGseMessage,K); */
        /* shall check the value of the byte transmitted for a match with the Data Link Escape  
           character. If one is ound, another DLE character will be immediately afterwards 
           (as long as the current word is not Word 00 or Word 01) */
 
        GseSciRegs.SCITXBUF.all = GseTxBuf[GseTxIndex];

        if (((GseTxBuf[GseTxIndex] & 0xFF) == DLE_CHAR) && (GseTxIndex > 1))
        {
            /* PATH(SendGseMessage,B); */
            /* since the buffer value matched DLE_CHAR, just assign #define */
            /* insert an escape character if the current buffer value matches hex 10 */
            GseSciRegs.SCITXBUF.all = DLE_CHAR;
        }

        /* add byte value to the checksum, but not if it is a beginning DLE */
        if (GseTxIndex != 0)
        {
            /* PATH(SendGseMessage,D); */
            checksum += GseTxBuf[GseTxIndex];
        }

        GseTxIndex++;
    }

    /* send out the checksum, if there's room in the FIFO */
    if ((GseTxIndex == length) && (GseSciRegs.SCIFFTX.bit.TXFFST <= MAX_STUFFED_FIFO_COUNT))
    {
        /* PATH(SendGseMessage,E); */
        /*shall calculate the message checksum by taking the 2's complement of the summation */
        checksum = -checksum;

        /*shall check to see if the checksum matches the value of the data link escape character
               and insert another DLE character immediately afterwards if a match is found */
        if ((checksum & 0x00FF) == DLE_CHAR)
        {
            /* PATH(SendGseMessage,F); */
            /* since the checksum value matched DLE_CHAR, just assign DLE_CHAR to the buffer */
            GseSciRegs.SCITXBUF.all = DLE_CHAR;
            /* followup with another data link escape character */
            GseSciRegs.SCITXBUF.all = DLE_CHAR;
        }
        else
        {
            /* PATH(SendGseMessage,G); */
            GseSciRegs.SCITXBUF.all = (checksum & 0x00FF);
        }

        GseTxIndex++;
    }

    /*shall send the closing data link escape character once all of the message data words
        have been transmitted. */
    if ((GseTxIndex == (length + 1)) && (GseSciRegs.SCIFFTX.bit.TXFFST < MAX_FIFO_COUNT))
    {
        /* PATH(SendGseMessage,H); */
        GseSciRegs.SCITXBUF.all = DLE_CHAR;
        GseTxIndex++;
    }

    /*shall send the end of transmission character following the closing data link escape character */
    if ((GseTxIndex == (length + 2)) && (GseSciRegs.SCIFFTX.bit.TXFFST < MAX_FIFO_COUNT))
    {
        /* PATH(SendGseMessage,I); */
        GseSciRegs.SCITXBUF.all = EOTX_CHAR;
        checksum = 0;
        status = TX_DONE;
        Transmitting.SIPT_Message = false;
        Transmitting.Rig_Message = false;
        Transmitting.NVM_Data = false;
        Transmitting.IBIT_Message = false;
        GseTxIndex = 0;
    }

    /* PATH(SendGseMessage,J); */

    return (status);
}
#endif

void Gse_CheckRigTest(void)
{
    //Uint16 Wow_Raw = 0, Rig_Raw = 0;
    //bool RigTestTimerExpired = false;
    Timer_t RigTestTimer = TIMER_DEFAULTS;
    Timer_t WowPinsTimer = TIMER_DEFAULTS;

    /* PATH(Gse_CheckRigTest,A); */

    /*shall create a timer for WOW pins and wait 10 mS */
    /* Wait 10 ms for WOW and RIG pins to be valid???????????? (IF COLD BOOT) */
    Timer_SetTime(&WowPinsTimer, TIMER_ONESHOT, TIMER_10ms);

    while (Timer_IsExpired(&WowPinsTimer) == false)
    {
        /* PATH(Gse_CheckRigTest,H); */
    }

    /* Provide 120 seconds GSE to connect either in Rig Mode or Test Mode */
    Timer_SetTime(&RigTestTimer, TIMER_ONESHOT, TIMER_120s);

    /* Initialize GSE Mode to No Connection */
    tGseMode = GSE_NO_CONNECTION;

    /* Test RIG pins; if on then wait to connect to GSE, otherwise continue normal bootup sequence */
    //if ((Wow_Raw == 2) && (Rig_Raw == 2))
    if(RIG_N_RAW == 0U)
    {
        /* PATH(Gse_CheckRigTest,C); */

        while ((Timer_IsExpired(&RigTestTimer) == FALSE) && (tGseMode == GSE_NO_CONNECTION))
        {
            /* Check to see if GSE has sent a command to enter RIG or TEST mode */
            /* PATH(Gse_CheckRigTest,D); */
            Gse_EstablishConnection();
            //Gse_Interactive(GseMode);
        }

        /* Check to see if the timer is expired and connection wasn't made, if so then we have an error of some sort */
        if ((Timer_IsExpired(&RigTestTimer) == TRUE) && (tGseMode == GSE_NO_CONNECTION))
        {
            /* PATH(Gse_CheckRigTest,E); */

            /* Proceed on with powering up the system, and let PBIT find the fault */
            Events_PostEvent(EVENT_INIT_DONE, 0);
            //RigTestTimerExpired = true;
        }

    }
    else
    {
        /* RIG pins were not set to try and connect with GSE. Continue with normal boot-up sequence. */
        Events_PostEvent(EVENT_INIT_DONE, 0);
    }

    /* PATH(Gse_CheckRigTest,G); */
}

void GseRig_ExecuteCommand(void)
{
    uint16_t u16RigPosition = 0U;
    /* Initialize Special Request Response to zeros */
    SpecialRequestResponse1 = 0U;
    SpecialRequestResponse2 = 0U;
    SpecialRequestResponse3 = 0U;
    SpecialRequestResponse4 = 0U;

    /* Set the speed based on the commanded speed */
    MotorCmd.MotorSpeed = tGseRigCmd.f32SpeedCmd;

    // Need to save old rig command and compare with new command. Only operate on new commands.
    /* Decode the Rig Command */
	switch (tGseRigCmd.eRigCmd)
    {
	    case RTN_TO_RSL:
	    case RTN_TO_N1:
	    case RTN_TO_ZERO:
        case RTN_TO_P1:
        case RTN_TO_P2:
        case RTN_TO_P3:
        case RTN_TO_P4:
        case RTN_TO_P5:
        case RTN_TO_P6:
        case RTN_TO_P7:
        case RTN_TO_P8:
        case RTN_TO_P9:
        case RTN_TO_P10:
        case RTN_TO_P11:
        case RTN_TO_ESL:
        {
            /* convert the command to a rig position value */
            u16RigPosition = (tGseRigCmd.eRigCmd - RTN_TO_RSL);

            /* Verify rig position is a valid position to run to */
            if(u16RigPosition < NUM_RIG_POSITIONS)
            {
                /* Validate the Actuator number for this channel to base the rig default settings on. */
                if(G_eActuatorNumber < NUM_ACTUATOR_TYPES)
                {
                    if (MotorCmd.BrakeCooling == false)
                    {
                        /* Set the rig position to run to based on the defaults per actuator */
                        GseRig_GoToPosition(s16RigPositionQuadCount[G_eActuatorNumber][u16RigPosition], tGseRigCmd.f32SpeedCmd);
                    }
                }
            }
            break;
        }
        case EXTEND_QUAD_250:
            if (MotorCmd.BrakeCooling == false)
            {
                GseRig_GoToPosition(tHall.Position + 250, tGseRigCmd.f32SpeedCmd);
            }
            break;
        case RETRACT_QUAD_250:
            if (MotorCmd.BrakeCooling == false)
            {
                GseRig_GoToPosition(tHall.Position - 250, tGseRigCmd.f32SpeedCmd);
            }
            break;
        case EXTEND_QUAD_25:
            if (MotorCmd.BrakeCooling == false)
            {
                GseRig_GoToPosition(tHall.Position + 25, tGseRigCmd.f32SpeedCmd);
            }
            break;
        case RETRACT_QUAD_25:
            if (MotorCmd.BrakeCooling == false)
            {
                GseRig_GoToPosition(tHall.Position - 25, tGseRigCmd.f32SpeedCmd);
            }
            break;
        case EXTEND_QUAD_1:
            if (MotorCmd.BrakeCooling == false)
            {
                GseRig_GoToPosition(tHall.Position + 1, tGseRigCmd.f32SpeedCmd);
            }
            break;
        case RETRACT_QUAD_1:
            if (MotorCmd.BrakeCooling == false)
            {
                GseRig_GoToPosition(tHall.Position - 1, tGseRigCmd.f32SpeedCmd);
            }
            break;
        case JOG_EXTEND:
            if (MotorCmd.BrakeCooling == false)
            {
                MotorCmd.StopPosition = NO_STOP;
                MotorCmd.MotorDirection = CW;
                MotorCmd.MotorStop = false;
                MotorCmd.MotorStart = true;
                MotorCmd.MotorSpeed = tGseRigCmd.f32SpeedCmd;
            }
            break;
        case JOG_RETRACT:
            if (MotorCmd.BrakeCooling == false)
            {
                MotorCmd.StopPosition = NO_STOP;
                MotorCmd.MotorDirection = CCW;
                MotorCmd.MotorStop = false;
                MotorCmd.MotorStart = true;
                MotorCmd.MotorSpeed = tGseRigCmd.f32SpeedCmd;
            }
            break;
        case RIG_POSITION_RSL:
        case RIG_POSITION_N1:
        case RIG_POSITION_ZERO:
        case RIG_POSITION_P1:
        case RIG_POSITION_P2:
        case RIG_POSITION_P3:
        case RIG_POSITION_P4:
        case RIG_POSITION_P5:
        case RIG_POSITION_P6:
        case RIG_POSITION_P7:
        case RIG_POSITION_P8:
        case RIG_POSITION_P9:
        case RIG_POSITION_P10:
        case RIG_POSITION_P11:
        case RIG_POSITION_ESL:
        {
            /* convert the command to a rig position value */
            u16RigPosition = (tGseRigCmd.eRigCmd - RIG_POSITION_RSL);

            /* Verify rig position is a valid position */
            if(u16RigPosition < NUM_RIG_POSITIONS)
            {
                Nvm_SetPosOnside((RIG_POSITIONS_T) u16RigPosition);
#if defined(__HALLX_CONFIGURED)
                Nvm_SetPosXside((RIG_POSITIONS_T) u16RigPosition);
#endif

                /* Set RSL and ESL status flags for UI. */
                if (u16RigPosition == (RIG_POSITIONS_T) ESL)
                {
                    /* Set the ESL status flag */
                    Rigging_Status.ESL_Rigged = true;
                }
                else if (u16RigPosition == (RIG_POSITIONS_T) RSL)
                {
                    /* Set the RSL Status Flag */
                    Rigging_Status.RSL_Rigged = true;
                }
            }
            break;
        }
        case RIG_START_RIG:
            tHall.Position = 0;         /* Reset quad count in reference position (command to be received in position ZERO) */
#if defined(__HALLX_CONFIGURED)
            tHallx.Position = 0;         /* Reset cross-side quad count in reference position (commmand to be received in position ZERO) */
#endif

            /* Initialize the Sensor Fusion Initial Position Estimate to 0 */
            G_ASW_DATA.tPanelOutputData.f32StrokeFused = 0;

            /* Initialize the Rig Procedure */
            RigModeScheduler_Start_Rig();

#if defined(__SKEW_SNSR_ENCODER__)
            /* Calculate the offset from the known ZERO position and store it in NVM to use for future position readings */
            Nvm_Rigging_Temp.tSkewSnsr.tEncoderCalibration.u32RawPositionAtStartOfRig = tIncoderInputs.u32Position;
            Nvm_Rigging_Temp.tSkewSnsr.tEncoderCalibration.u32Offset = u32EncoderIdealCnt[G_eActuatorNumber][ZERO] - Nvm_Rigging_Temp.tSkewSnsr.tEncoderCalibration.u32RawPositionAtStartOfRig;
#endif

//            /* Begin running the sensor fusion algorithms */
//            bRunSensorFusion = true;

            break;
        case STOP:
			Act_SetP(_IQ(0.552));
            MotorCmd.MotorSpeed = 0.0;
            MotorCmd.StopPosition = NO_STOP;
            MotorCmd.MotorStop = true;

#ifndef __GPD_LITE_NO_BRAKE
            if ((MotorCmd.BrakeActivated == true) &&
                (MotorCmd.MotorStop == true))
            {
                /* PATH(Gse_DoRigModeCommand,AJ); */
                Brake_Deactivate();
            }
#endif
            break;
        case RLS_BRAKE:
#ifndef __GPD_LITE_NO_BRAKE
            if (MotorCmd.BrakeActivated == false)
            {
                /* PATH(Gse_DoRigModeCommand,AL); */
                MotorCmd.ManualBrkRls = true;
                Brake_Activate();
            }
#endif
            break;
        case APPLY_BRAKE:
#ifndef __GPD_LITE_NO_BRAKE
            if ((MotorCmd.BrakeActivated == true) &&
                (MotorCmd.MotorStop == true))
            {
                /* PATH(Gse_DoRigModeCommand,AN); */
                MotorCmd.ManualBrkRls = false;
                Brake_Deactivate();
            }
#endif
            break;
        case TX_QUAD_RSL:
        case TX_QUAD_ZERO:
        case TX_QUAD_N1:
        case TX_QUAD_P1:
        case TX_QUAD_P2:
        case TX_QUAD_P3:
        case TX_QUAD_P4:
        case TX_QUAD_P5:
        case TX_QUAD_P6:
        case TX_QUAD_P7:
        case TX_QUAD_P8:
        case TX_QUAD_P9:
        case TX_QUAD_P10:
        case TX_QUAD_P11:
        case TX_QUAD_ESL:
        {
            /* convert the command to a rig position value */
            uint16_t u16RigPosition = (tGseRigCmd.eRigCmd - TX_QUAD_RSL);

            /* Verify rig position is a valid position */
            if(u16RigPosition < NUM_RIG_POSITIONS)
            {
                /* Send Quad Count for this Rig Position */
                SpecialRequestResponse1 = ((Uint16)(Nvm_Rigging_Temp.quad[u16RigPosition].onside) & 0x00FF);
                SpecialRequestResponse2 = (((Uint16)(Nvm_Rigging_Temp.quad[u16RigPosition].onside) & 0xFF00) >> 8U);
//                /* Send RVDT P_POS Count for this Rig Position */
//                SpecialRequestResponse3 = ((Uint16)(Nvm_Rigging_Temp.tSkewSnsr.tRvdtPosData[u16RigPosition].u16RvdtPos) & 0xFF);
//                SpecialRequestResponse4 = (((Uint16)(Nvm_Rigging_Temp.tSkewSnsr.tRvdtPosData[u16RigPosition].u16RvdtPos) & 0xFF00) >> 8U);
            }
            break;
        }
        case RIG_NO_ACTION:
        default:
            /* PATH(Gse_DoRigModeCommand,AW); */
            break;
    } /* End switch (tGseRigCmd.eRigCmd) */
    
    /* PATH(Gse_DoRigModeCommand,AX); */
}

#if 0
Uint16 GseTest_ExecuteCommand(Uint16 command, GSE_RX_CMD_MSG *msg)
{
	Uint16 serialNum;  /* board serial number */
	Uint16 retVal = 0; /* return value */

	/* PATH(DoTestModeCommand,A); */

	switch (command)
	{
		case RLS_BRAKE:
			/* PATH(DoTestModeCommand,B); */

			if (MotorCmd.BrakeActivated == false)
            {
                /* PATH(DoTestModeCommand,C); */
                MotorCmd.ManualBrkRls = true;
                Brake_Activate();
            }
			break;
		case APPLY_BRAKE:
			/* PATH(DoTestModeCommand,D); */

			if ((MotorCmd.BrakeActivated == true) &&
                (MotorCmd.MotorStop == true))
            {
                /* PATH(DoTestModeCommand,E); */
                MotorCmd.ManualBrkRls = false;
                Brake_Deactivate();
            }
			break;
		case WRITE_SERIALNUM:
            /* PATH(DoTestModeCommand,F); */
            serialNum = ((msg->SerialNumMSW << 8) | msg->SerialNumLSW);

            if ((serialNum != 0) && (serialNum != tNvm.tData.Nvm_BoardSerialNumber))
            {
                /* PATH(DoTestModeCommand,G); */
                tNvm.tData.Nvm_BoardSerialNumber = serialNum;
            }
            break;
		case TX_MOSTRECENT_NVM:
            /* PATH(DoTestModeCommand,H); */
            TxMostRecentNvm = true;
			retVal = 1;
            break;
		case TX_NEXTRECENT_NVM:
            /* PATH(DoTestModeCommand,I); */
            TxNextRecentNvm = true;
			retVal = 1;
            break;
		case TX_ALL_NVM:
            /* PATH(DoTestModeCommand,J); */
            TxAllNvm = true;
			retVal = 1;
            break;
		case CLEAR_ALL_NVM:
            /* PATH(DoTestModeCommand,K); */
            FormatNvm = true;
			retVal = 1;
            break;
		case CLEAR_ALL_RIG:
            /* PATH(DoTestModeCommand,L); */
            ClearRigging = true;
			retVal = 1;
            break;
		case CLEAR_ALL_STATE:
            /* PATH(DoTestModeCommand,M); */
            ClearState = true;
			retVal = 1;
            break;
		case GET_NVM_STATUS:
            /* PATH(DoTestModeCommand,N); */
			retVal = 1;
            break;
		case RIGDATA_PAGE1:
            TxRigPage1 = true;
			retVal = 2;
            break;
		case RIGDATA_PAGE2:
            TxRigPage2 = true;
			retVal = 2;
            break;
		case RIGDATA_PAGE3:
            TxRigPage3 = true;
			retVal = 2;
            break;
		case RIGDATA_PAGE4:
            TxRigPage4 = true;
			retVal = 2;
            break;
        case RIGDATA_PAGE5:
            TxRigPage5 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE6:
            TxRigPage6 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE7:
            TxRigPage7 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE8:
            TxRigPage8 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE9:
            TxRigPage9 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE10:
            TxRigPage10 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE11:
            TxRigPage11 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE12:
            TxRigPage12 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE13:
            TxRigPage13 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE14:
            TxRigPage14 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE15:
            TxRigPage15 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE16:
            TxRigPage16 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE17:
            TxRigPage17 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE18:
            TxRigPage18 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE19:
            TxRigPage19 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE20:
            TxRigPage20 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE21:
            TxRigPage21 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE22:
            TxRigPage22 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE23:
            TxRigPage23 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE24:
            TxRigPage24 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE25:
            TxRigPage25 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE26:
            TxRigPage26 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE27:
            TxRigPage27 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE28:
            TxRigPage28 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE29:
            TxRigPage29 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE30:
            TxRigPage30 = true;
            retVal = 2;
            break;
		case RIGDATA_PAGE31:
            TxRigPage31 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE32:
            TxRigPage32 = true;
            retVal = 2;
            break;
		case RIGDATA_PAGE33:
            TxRigPage33 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE34:
            TxRigPage34 = true;
            retVal = 2;
            break;
		case RIGDATA_PAGE35:
            TxRigPage35 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE36:
            TxRigPage36 = true;
            retVal = 2;
            break;
		case RIGDATA_PAGE37:
            TxRigPage37 = true;
            retVal = 2;
            break;
		case RIGDATA_PAGE38:
            TxRigPage38 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE39:
            TxRigPage39 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE40:
            TxRigPage40 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE41:
            TxRigPage41 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE42:
            TxRigPage42 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE43:
            TxRigPage43 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE44:
            TxRigPage44 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE45:
            TxRigPage45 = true;
            retVal = 2;
            break;
        case RIGDATA_PAGE46:
            TxRigPage46 = true;
            retVal = 2;
            break;
		case TX_RTC:
			/* PATH(DoTestModeCommand,AB); */
			Ibit_Stat.TestData = tNvm.tData.Nvm_RunTimeCount;
			Ibit_Stat.TestID = TX_RTC;
			Ibit_Stat.tstCmplt = COMPLETED;
			Ibit_Stat.result = true;
			IBIT_Test = true;
			retVal = 2;
			break;
		case SEND_SIPT_DATA:
            /* PATH(DoTestModeCommand,V); */
            Ibit_Stat.tstCmplt = NOT_STARTED;
            Ibit_Stat.TestID = 0;
            Ibit_Stat.result = false;

            if (TxAllNvm == true)
            {
				/* PATH(DoTestModeCommand,W); */
                nvmidx = 0xFFFF;
                TxAllNvm = false;
            }

            IBITFault = false;
			retVal = 4;
            break;
		default:
			/* PATH(DoTestModeCommand,Y); */
			retVal = 0;
			break;
	}

	/* PATH(DoTestModeCommand,Z); */
    return (retVal);
}
#endif
/****************************************************************************************************
*  Function: GseRig_GetGseCommands
*  Purpose: Read in the Commands received from the GSE
*  Global Inputs: tGseRigInterface
*  Global Outputs:
*  Input:
*  Output:
****************************************************************************************************/
void GseRig_GetGseCommands(void)
{
    float32_t f32Temp = 0.0F; /* Use temp to protect from interrupts */
    bool_t bTemp = false;  /* Use temp to protect from interrupts */
    t64Types_t t64Temp = { 0 }; /* temp 64-bit structure for opaque signals */

    /* Get Primary Bus Sync Commands */
    /* Get SYNC Command Signals */
    A825Mgr_GetSignal_BOOL(&tA825_Bus[ARINC825_MAINT_BUS].tRx, A825_MAINT_BUS_RX_RIG_CMD, SIGNAL1, &bTemp);
    tGseRigInterface.bVcModeCmdL = bTemp;
    A825Mgr_GetSignal_BOOL(&tA825_Bus[ARINC825_MAINT_BUS].tRx, A825_MAINT_BUS_RX_RIG_CMD, SIGNAL2, &bTemp);
    tGseRigInterface.bSensorFusionEnableL = bTemp;
    A825Mgr_GetSignal_BNR(&tA825_Bus[ARINC825_MAINT_BUS].tRx, A825_MAINT_BUS_RX_RIG_CMD, SIGNAL3, &f32Temp);
    tGseRigInterface.f32PositionCmdL = f32Temp;
    A825Mgr_GetSignal_BNR(&tA825_Bus[ARINC825_MAINT_BUS].tRx, A825_MAINT_BUS_RX_RIG_CMD, SIGNAL4, &f32Temp);
    tGseRigInterface.f32SpeedCmdL = f32Temp;
    A825Mgr_GetSignal_BOOL(&tA825_Bus[ARINC825_MAINT_BUS].tRx, A825_MAINT_BUS_RX_RIG_CMD, SIGNAL5, &bTemp);
    tGseRigInterface.bVcModeCmdR = bTemp;
    A825Mgr_GetSignal_BOOL(&tA825_Bus[ARINC825_MAINT_BUS].tRx, A825_MAINT_BUS_RX_RIG_CMD, SIGNAL6, &bTemp);
    tGseRigInterface.bSensorFusionEnableR = bTemp;
    A825Mgr_GetSignal_BNR(&tA825_Bus[ARINC825_MAINT_BUS].tRx, A825_MAINT_BUS_RX_RIG_CMD, SIGNAL7, &f32Temp);
    tGseRigInterface.f32PositionCmdR = f32Temp;
    A825Mgr_GetSignal_BNR(&tA825_Bus[ARINC825_MAINT_BUS].tRx, A825_MAINT_BUS_RX_RIG_CMD, SIGNAL8, &f32Temp);
    tGseRigInterface.f32SpeedCmdR = f32Temp;
    A825Mgr_GetSignal_OPAQUE(&tA825_Bus[ARINC825_MAINT_BUS].tRx, A825_MAINT_BUS_RX_RIG_CMD, SIGNAL9, &t64Temp);
    tGseRigInterface.eCmdL = (eGseRigCmds_t)(t64Temp.tu16.u16_0);
    A825Mgr_GetSignal_OPAQUE(&tA825_Bus[ARINC825_MAINT_BUS].tRx, A825_MAINT_BUS_RX_RIG_CMD, SIGNAL10, &t64Temp);
    tGseRigInterface.eCmdR = (eGseRigCmds_t)(t64Temp.tu16.u16_0);

    return;
}

/****************************************************************************************************
*  Function: GseRig_SetCommand
*  Purpose: Set the panel command based upon the status of the ARINC825 control bus.
*  Global Inputs: tGseRigInterface
*  Global Outputs: tGseRigCmd, tGseRigCmdx
*  Input:
*  Output:
****************************************************************************************************/
void GseRig_SetRigCommands(void)
{

    /* Set the Communication Established flag for the Control Bus based on the GSE Stale Status */
    if(tA825_Bus[ARINC825_MAINT_BUS].tRx.ptMsgList[A825_MAINT_BUS_RX_RIG_CMD].bMsgStaleStatus == false)
    {
        /* Set Control Bus SCU as having established communication */
        tA825_Bus[ARINC825_MAINT_BUS].bCommsEstablished = true;
    }

    /* Decode Channel to save off proper Panel Commands for Actuator */
    if(G_eActId == LEFT_ACTUATOR)
    {   /* Left Actuator is On-Side, Right Actuator is Cross-Side */
        /* Save Rig Commands */
        tGseRigCmd.eRigCmd = tGseRigInterface.eCmdL;
        tGseRigCmdx.eRigCmd = tGseRigInterface.eCmdR;
        /* Save VC Mode Commands */
        tGseRigCmd.bVcModeCmd = tGseRigInterface.bVcModeCmdL;
        tGseRigCmdx.bVcModeCmd = tGseRigInterface.bVcModeCmdR;
        /* Save Sensor Fusion Enable Commands */
        tGseRigCmd.bSensorFusionEnableCmd = tGseRigInterface.bSensorFusionEnableL;
        tGseRigCmdx.bSensorFusionEnableCmd = tGseRigInterface.bSensorFusionEnableR;
        /* Save Position Commands */
        tGseRigCmd.f32PositionCmd = tGseRigInterface.f32PositionCmdL;
        tGseRigCmdx.f32PositionCmd = tGseRigInterface.f32PositionCmdR;
        /* Save Speed Commands */
        tGseRigCmd.f32SpeedCmd = tGseRigInterface.f32SpeedCmdL;
        tGseRigCmdx.f32SpeedCmd = tGseRigInterface.f32SpeedCmdR;
    }
    else /* if(G_eActId == RIGHT_ACTUATOR) */
    {   /* Right Actuator is On-Side, Left Actuator is Cross-Side */
        /* Save Position Commands */
        tGseRigCmd.eRigCmd = tGseRigInterface.eCmdR;
        tGseRigCmdx.eRigCmd = tGseRigInterface.eCmdL;
        /* Save VC Mode Commands */
        tGseRigCmd.bVcModeCmd = tGseRigInterface.bVcModeCmdR;
        tGseRigCmdx.bVcModeCmd = tGseRigInterface.bVcModeCmdL;
        /* Save Sensor Fusion Enable Commands */
        tGseRigCmd.bSensorFusionEnableCmd = tGseRigInterface.bSensorFusionEnableR;
        tGseRigCmdx.bSensorFusionEnableCmd = tGseRigInterface.bSensorFusionEnableL;
        /* Save Position Commands */
        tGseRigCmd.f32PositionCmd = tGseRigInterface.f32PositionCmdR;
        tGseRigCmdx.f32PositionCmd = tGseRigInterface.f32PositionCmdL;
        /* Save Speed Commands */
        tGseRigCmd.f32SpeedCmd = tGseRigInterface.f32SpeedCmdR;
        tGseRigCmdx.f32SpeedCmd = tGseRigInterface.f32SpeedCmdL;
    }

    /* Set the BUS Command HL/VC based on the VC Mode Command signal on the interface */
    MCU_BusSetVcModeCmd(tGseRigCmd.bVcModeCmd);

    return;
}

/****************************************************************************************************
*  Function: GseRig_GetCommands
*  Purpose: Read in the Commands received from the SYNC Controller
*  Global Inputs: tGseRigInterface
*  Global Outputs:
*  Input:
*  Output:
****************************************************************************************************/
bool_t GseRig_GetCommands(void)
{
    bool bValidCmd = false;

    /* Get the latest GSE Rig Commands for both channels */
    GseRig_GetGseCommands();

    /* Set the Rig command associated with this channel */
    GseRig_SetRigCommands();

    /* Validity check for Speed Command */
    bValidCmd = Panel_IsSpeedCommandValid(tGseRigCmd.f32SpeedCmd);

    return bValidCmd;
}

/****************************************************************************************************
*  Function: GseRig_Command
*  Purpose: Read the GSE Rig Command Inputs, Save to appropriate channel, and decode the command.
*  Call the function to execute the command.
*  Global Inputs: tGseRigInterface, tGseRigCmd, tGseRigCmdx
*  Global Outputs: tGseRigInterface, tGseRigCmd, tGseRigCmdx
*  Input:
*  Output:
****************************************************************************************************/
void GseRig_Command( void )
{
    bool_t bValidCmd = false;

    /* Get GSE Rig Command Signals */
    bValidCmd = GseRig_GetCommands();

    if(bValidCmd == true)
    {
        GseRig_ExecuteCommand();
    }

    return;
}

/****************************************************************************************************
*  Function: GseRig_SetFeedback
*  Purpose: Set the signals in the Rig Position Status Feedback and Sensor Data Messages to go to GSE
*  Global Inputs: tGseRigInterface, tGseRigCmd, tGseRigCmdx
*  Global Outputs:
*  Input:
*  Output:
****************************************************************************************************/
void GseRig_SetFeedback(void)
{
    STATE_ID tMcuState = McuGetState();

    /* MCU Status Bits */
    tMcuStatus.bRig.Normal_Op_State = ((tMcuState == IDLE_MODE) | (tMcuState == RUN_MODE));
    tMcuStatus.bRig.Ground_Maint_State = true;   /* in RIG MODE which is a Maintenance Mode Function */
    tMcuStatus.bRig.Maint_Bus_Input_Fault = (LatchedFaults_Stat.bit.bA825MaintBusPassive | LatchedFaults_Stat.bit.bA825MaintBusOff);

    if ((Inhibits_Stat.all != 0U) || (CriticalFaults_Stat.all != 0U) || (LatchedFaults_Stat.all != 0U))
    {
        tMcuStatus.bRig.MCU_Control_Valid = false;
        CH_STATUS_clr(); /* Set Channel Status output to not healthy */
    }
    else
    {
        tMcuStatus.bRig.MCU_Control_Valid = true;
        CH_STATUS_set(); /* Set Channel Status output to healthy */
    }
    tMcuStatus.bRig.MCU_Engaged = (tMcuStatus.bRig.MCU_Control_Valid & !LatchedWarnings_Stat.bit.bMcuDisabled);

    tMcuStatus.bRig.Maint_Bus_Valid = !(Inhibits_Stat.bit.bA825MaintBusCommsStale | LatchedFaults_Stat.bit.bA825MaintBusPassive | LatchedFaults_Stat.bit.bA825MaintBusOff);
    tMcuStatus.bRig.Flap_In_Motion = ((MotorCmd.MotorRunning) | (tSpeed.Speed > 0.0F));
    tMcuStatus.bRig.Brake_Engaged = !MotorCmd.BrakeActivated;
    tMcuStatus.bRig.Rig_Status = Nvm_State.rigstatus;
    tMcuStatus.bRig.Rig_ESL_Rigged = Rigging_Status.ESL_Rigged;
    tMcuStatus.bRig.Rig_RSL_Rigged = Rigging_Status.RSL_Rigged;
    tMcuStatus.bRig.Rig_Verify_Ready = Rigging_Status.rigVerify_Ready;

    /* MCU Fault Bits */
    tMcuFault.bit.Sensor_Fusion_Covariance = (CriticalFaults_Stat.bit.SfBiasCheck_M | CriticalFaults_Stat.bit.SfMeasResidualCheck_M);
    tMcuFault.bit.Sensor_Fault = (tMcuMaint.bit.RVDT_Position_Fault | tMcuMaint.bit.RVDT_Excitation_Fault |
                                  tMcuMaint.bit.Hall_Sensor_Fault | tMcuMaint.bit.Brake_Hold_Fault | tMcuMaint.bit.Brake_Drive_Fault);
    tMcuFault.bit.Flap_Jam_Active = Inhibits_Stat.bit.FlapJam_M;
    tMcuFault.bit.Motor_Brake_Fault = CriticalFaults_Stat.bit.BrakeHold_M;

    /* MCU Maintenance Bits */
    tMcuMaint.bRig.Fault_Code = (ReportedFaultCode & 0xFFU);        /* Fault Code is only 8-bit resolution */
    tMcuMaint.bit.RVDT_Position_Fault = LatchedWarnings_Stat.bit.bRvdtPositionFault_M;
    tMcuMaint.bit.RVDT_Excitation_Fault = LatchedFaults_Stat.bit.RvdtExcitationFault;
    tMcuMaint.bit.Hall_Sensor_Fault = (LatchedFaults_Stat.bit.HallSequence_M | LatchedFaults_Stat.bit.InvalidHall_M);
    tMcuMaint.bit.Brake_Hold_Fault = CriticalFaults_Stat.bit.BrakeHold_M;
    tMcuMaint.bit.Brake_Drive_Fault = LatchedFaults_Stat.bit.BrakeSwitch_M;


// Previous Rig Feedback */
//    /* Echo the Rig Command received that the MCU is responding to */
//    RigMsg->RigCmd = tGseRigCmd.eRigCmd;
//
//    /* Set the special request response data fields */
//    RigMsg->SpecialRequestResponse1 = SpecialRequestResponse1;
//    RigMsg->SpecialRequestResponse2 = SpecialRequestResponse2;
//    RigMsg->SpecialRequestResponse3 = SpecialRequestResponse3;
//    RigMsg->SpecialRequestResponse4 = SpecialRequestResponse4;

    /* Set the signals to return to the GSE for Rig Mode */
    GseRig_SetFeedbackSignals();     /* MCU Signals to send to GSE */

    return;
}

/****************************************************************************************************
*  Function: GseRig_SetFeedbackSignals
*  Purpose: Set the signals in the MCU Position Status Feedback and Senor Data Messages to go to SYNC
*  Global Inputs: tGseRigInterface, tGseRigCmd
*  Global Outputs:
*  Input:
*  Output:
****************************************************************************************************/
void GseRig_SetFeedbackSignals(void)
{
    /* SKELETON FOR FUTURE, REWORK WHEN SIGNALS ARE DEFINED FOR RIG INTERFACE */
    /* Set the signals for the Position Status Fault Maintenance Message to GSE */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_MAINT_BUS].tTx, A825_MAINT_BUS_TX_MCU_PSN_STATUS_FLT_MAINT, SIGNAL1, G_ASW_DATA.tPanelOutputData.f32StrokeFused);  /* Position Estimate from Sensor Fusion */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_MAINT_BUS].tTx, A825_MAINT_BUS_TX_MCU_PSN_STATUS_FLT_MAINT, SIGNAL2, (!RIG_N_RAW));  /* Rig Pins Signal */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_MAINT_BUS].tTx, A825_MAINT_BUS_TX_MCU_PSN_STATUS_FLT_MAINT, SIGNAL3, Rigging_Status.bSkewSnsrCalibrationComplete);  /* Rig RVDT Calibrated Signal */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_MAINT_BUS].tTx, A825_MAINT_BUS_TX_MCU_PSN_STATUS_FLT_MAINT, SIGNAL4, Rigging_Status.rigData_Written);  /* Rig NVM Data Written Signal*/
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_MAINT_BUS].tTx, A825_MAINT_BUS_TX_MCU_PSN_STATUS_FLT_MAINT, SIGNAL5, tMcuStatus.all);  /* MCU Status Signals */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_MAINT_BUS].tTx, A825_MAINT_BUS_TX_MCU_PSN_STATUS_FLT_MAINT, SIGNAL6, tMcuFault.all);   /* MCU Fault Signals  */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_MAINT_BUS].tTx, A825_MAINT_BUS_TX_MCU_PSN_STATUS_FLT_MAINT, SIGNAL7, tMcuMaint.all);   /* MCU Maint Signals  */

    /* Set the signals for the Sensor Data to GSE */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_MAINT_BUS].tTx, A825_MAINT_BUS_TX_MCU_SENSOR_DATA, SIGNAL1, f32MotorCurrentAmps);                 /* Current Feedback */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_MAINT_BUS].tTx, A825_MAINT_BUS_TX_MCU_SENSOR_DATA, SIGNAL2, tHall.Position);                      /* Quad Count Feedback */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_MAINT_BUS].tTx, A825_MAINT_BUS_TX_MCU_SENSOR_DATA, SIGNAL3, tSkewSnsrCalcs.tStroke.f32Stroke);    /* Skew Sensor Position in Inches Feedback */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_MAINT_BUS].tTx, A825_MAINT_BUS_TX_MCU_SENSOR_DATA, SIGNAL4, SkewSnsr_GetPositionCount());         /* Skew Sensor RVDT_POS ADC Counts or IncOder Counts Upper 16 bits */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_MAINT_BUS].tTx, A825_MAINT_BUS_TX_MCU_SENSOR_DATA, SIGNAL5, SkewSnsr_GetSensorType());            /* Skew Sensor Type */

    return;
}

/*
    brief Commands the motor to move to the given position

    Purpose:
        Commands the motor to move to the given position, with motor speed
        clamped at 1.0.

    param[in] s16PositionCmdQuad New target position, in terms of hall sensor counts
    param[in] f32Speed    Targeted speed of motor

    Global Variables Referenced:
        #MotorCmd
        #ReversalTimer
        #MotorReversing

    return  void

*/
void GseRig_GoToPosition(int16_t s16PositionCmdQuad, float32_t f32Speed)
{
    if ((MotorCmd.StopPosition != s16PositionCmdQuad))
    {
        if ((MotorCmd.MotorStop == false) && (MotorReversing == false)) /* If motor is moving and not reversing */
        {
            if (((tHall.Position > s16PositionCmdQuad) && (MotorCmd.MotorDirection == CW)) ||
                ((tHall.Position < s16PositionCmdQuad) && (MotorCmd.MotorDirection == CCW)))
            {
                /* keep running until position has been met or exceeded */
                MotorCmd.StopPosition = tHall.Position;
                MotorReversing = true;
                Timer_SetTime(&tMotorReversalTimer, TIMER_ONESHOT, TIMER_1500ms);
                Events_PostEvent(EVENT_POSITION_COMPLETE, 0); /* Transition back into IDLE_MODE */
            }
        }
        else if ((Timer_IsExpired(&tMotorReversalTimer) == true) && (MotorReversing == true)) /* Delay has expired */
        {
            MotorReversing = false;
            Events_PostEvent(EVENT_POSITION_CMD, 0); /* Post Event to transition back into RUN_MODE */
        }

        if(MotorReversing == false) /* We are no longer reversing */
        {
            MotorCmd.StopPosition = s16PositionCmdQuad;

            /* Set direction */
            if (MotorCmd.StopPosition >= tHall.Position)
            {
                MotorCmd.MotorDirection = CW;
            }
            else
            {
                MotorCmd.MotorDirection = CCW;
            }

            MotorCmd.MotorStop = false;
            MotorCmd.MotorStart = true;

            /* Limit Check the commanded speed */
            if (f32Speed > MAX_SPEED_COMMAND_FACTOR)
            {
                /* PATH(Panel_GoToPosition,I); */
                f32Speed = MAX_SPEED_COMMAND_FACTOR;
            }
            else if (f32Speed < 0.0)
            {
                //f32Speed = SpeedOfMotor;
                f32Speed = 0.0;
            }

            MotorCmd.MotorSpeed = f32Speed;
        }
    }
    else if (tHall.Position == s16PositionCmdQuad)
    {
        Events_PostEvent(EVENT_POSITION_COMPLETE, 0); /* Transition back into IDLE_MODE */
    }
}
/* end gse.c */
