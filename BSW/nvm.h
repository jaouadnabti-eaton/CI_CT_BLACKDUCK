/****************************************************************************************************
*  File name: nvm.h
*
*  Purpose: Types and definitions to support NVM functionality for MCU
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
****************************************************************************************************/

#ifndef NVM_H__
#define NVM_H__

/*      Include Files
*/
#include "F2837xD_device.h"
#include "parameter.h"
#include "bit.h"
#include "hsm.h"
#include "timer.h"
#include "gse.h"
#include "ASW_BSW_I.h"
#include "skewsensor.h"
#include "panel.h"

/*
   nvm Non-Volatile Memory driver
  
  brief The NVM Driver is an abstraction for writing and erasing NVM memory.

*/
#if defined(DRV8312_DEV_KIT)
    /* 2837xD Development Kit Hardware */
	#define MRAM_BASEADDR       0x016000UL /* Start Address of MRAM */
	#define SIZEOF_MRAM         0x002000UL /* Total size of MRAM in 16-bit words:  NVM_DATA + FAULT_TABLE */
	#define SIZEOF_NVM_DATA     0x001000UL /* Size of NVM Data in MRAM in 16-bit words */
	#define SIZEOF_FAULT_TABLE  0x001000UL /* Size of Fault Record Table in MRAM in 16-bit words */
#else
	/* MCU Target HW */
	#define MRAM_BASEADDR       0x100000UL  /* Start Address of MRAM */
	#define SIZEOF_MRAM         0x40000UL   /* Total size of MRAM in 16-bit words:  NVM_DATA + FAULT_TABLE */
	#define SIZEOF_NVM_DATA     0x10000UL   /* Size of NVM Data in MRAM in 16-bit words */
	#define SIZEOF_FAULT_TABLE  0x30000UL   /* Size of Fault Record Table in MRAM in 16-bit words */
#endif

#define SIZEOF_FAULT_RECORD     128U    /* Size of a single Fault Record in 16-bit words*/
#define NUMBER_OF_FAULT_RECORDS (SIZEOF_FAULT_TABLE / SIZEOF_FAULT_RECORD)  /* Number of Fault Records in the Fault Table */

/*      Public Type Definitions
*/

/* brief Data structure type for hall counter (quadrature) position(s) */
typedef struct
{
    int16  onside;
#if defined(__HALLX_CONFIGURED)
    int16  xside;
#endif
} Nvm_QuadPosition_t;

typedef enum
{
    NOT_RIGGED,
    RIG_COMPLETE,
    RIG_VERIFIED
} RIG_STATUS;

typedef struct
{
    bool onsideChannel_Rigged;
    bool xsideChannel_Rigged;
    bool bSkewSnsrCalibrationComplete;
    bool rigData_Written;
    bool rigVerify_Ready;
    bool RSL_Rigged;
    bool ESL_Rigged;
} RigStatus_t;

typedef struct
{
    uint16_t bPRGM1:1;      /* PRGM1 bit*/
    uint16_t bPRGM2:1;      /* PRGM2 bit*/
    uint16_t bPRGM3:1;      /* PRGM3 bit*/
    uint16_t bPRGM4:1;      /* PRGM4 bit*/
    uint16_t bPRGM5:1;      /* PRGM5 bit*/
    uint16_t bPRGM6:1;      /* PRGM6 bit*/
    uint16_t bNotUsed:10;   /* Not used bits*/
} tPrgmBits_t;

typedef union
{
    uint16_t all;
    tPrgmBits_t bit;
} tPrgm_t;

/* brief Data structure type to represent all data in the Rigging partition of NVM, including the CRC*/
typedef struct
{
    SKEW_SNSR_NVM_DATA_T    tSkewSnsr;                  /* Skew sensor data for rigging including calibration data */
    Nvm_QuadPosition_t      quad[NUM_RIG_POSITIONS];    /* Quad count data for rigging */
    tPanelNvmData_t         tPanel[NUM_RIG_POSITIONS];  /* Panel data including sensor fusion and skew sensor calcs */
    tPrgm_t                 tPrgm;
    Uint16                  crc;
} Nvm_Rigging_t;

/* Definition of the Fault Record Data */
/* brief Data structure type to represent a single State entry for the State partition of NVM*/
typedef struct
{
    uint16_t              timecount;
    uint16_t              FlightCycleCount;
    uint16_t              bootcount;
    uint16_t              faultId;
    uint16_t              faultData;
    uint32_t              RunTimeCount;
    STATE_ID              Mode;
    RIG_STATUS            rigstatus;
    Nvm_QuadPosition_t    quadposition;
    Bit_Flags_t           bitstatus;
    uint16_t              frameOverCount[8];
    tMcuStatus_t          tMcuStatus;
    tMcuFault_t           tMcuFault;
    tMcuMaint_t           tMcuMaint;
    tPanelNvmData_t       tPanel;
} Nvm_State_t; /* tFaultRecordData_t */

///* brief Data structure type to represent the Current State Pointer buffer*/
//typedef struct
//{
//    Uint16 CurrentStatePointer[30];
//} CurrentStatePointers_t;

/* Structure for NVM DATA */
typedef struct
{
  Nvm_Rigging_t     Nvm_Rigging;                    /* Rigging Data stored in NVM */
  uint32_t          Nvm_RunTimeCount;               /* LRU run time counter in 10s of minutes */
  uint16_t          Nvm_State_Pointer;              /* Location in the Fault Table to store the next Fault Record (used to be Current_NVM_State_Pointer) */
  Nvm_State_t       Nvm_Powerdown_State;            /* Working copy of the latest Nvm_State. Used for Powerdown/Powerup */
  uint16_t          Nvm_BoardSerialNumber;
  uint16_t          WDRS;                           /* Watchdog Reset Status on startup */
  uint16_t          WDRS_Counter;                   /* Running count of Watchdog Reset Events */
  eActId_t          tCH_ID;                         /* Channel Identification */
  eLruId_t          tLRU_ID;                        /* LRU Identification */
  float32_t         f32PositionCmdOld;              /* Copy of the last valid position command in stroke inches */
  float32_t         f32SpeedCmdOld;                 /* Copy of the last valid speed command in % x100 */
} tNvmDataTypes;

/* UNION for storing NVM DATA */
typedef union
{
    uint16_t        ch[SIZEOF_NVM_DATA];
    tNvmDataTypes   tData;
} tNmvData_t;

/********************** FAULT RECORD TYPEDEFS ********************************************************/

/* Definition of the Data stored as part of a Fault Record */
//typedef struct
//{
//    uint16_t u16Id;
//    uint16_t u16Type;
//    uint16_t u16Time;
//    uint16_t rsrvd[125];
//} tFaultRecordData_t;

/* Definition of a Fault Record */
typedef union
{
    uint16_t            ch[SIZEOF_FAULT_RECORD]; /* 256 bytes, 128 16-bit locations */
   //tFaultRecordData_t tData;                   /* tData for future design */
   Nvm_State_t          Nvm_State;               /* 42 bytes of Data [legacy design] */
} tFaultRecord_t;

/* Define the Fault Record Table */
typedef union
{
    uint16_t        ch[SIZEOF_FAULT_TABLE];
    tFaultRecord_t  tRecord[NUMBER_OF_FAULT_RECORDS];
} tFaultData_t;

/*      Public Variable Declarations
*/

/*      Public ROM Constants
*/

/*      Public Defines
*/
//#define BIT_RIGTEST_CRC_CODE	0x8005
#define BIT_RIGTEST_CRC_CODE    0x1021U
#define NVM_MAXWRITESIZE        SIZEOF_FAULT_RECORD /* size in 16-bit words */
//
#define NVM_BASEADDR        MRAM_BASEADDR   /* NVM data uses MRAM */
#define NVM_SIZE            SIZEOF_MRAM     /* Size of NVM is size of MRAM */

#define NVM_BUF_SIZE            ((sizeof(Nvm_State_t) * 2)) /* size of the NVM data buffer in the GSE Test interactive transmit message */

/*   brief Message definition for GSE Test interactive mode command response  */
typedef struct
{
    Uint16 BeginDLE;              /* Word No. 00, the beginning data link escape character */
    Uint16 ChanDataID;            /* Word No. 01, channel data identifier */
    Uint16 DataPktSize;           /* Word No. 02, data packet size for message */
    Uint16 FSCUChanID;            /* Word No. 03, left or right channel */
    Uint16 RecordIDL;             /* Word No. 04, record ID */
    Uint16 RecordIDH;             /* Word No. 05, record ID */
    Uint16 NVMData[NVM_BUF_SIZE]; /* Word No. 06 thru ?, data from NVM */
} GSE_TX_TEST_MSG;

#define TEST_RSP_MSG_SIZE       sizeof(GSE_TX_TEST_MSG)
#define TEST_RSP_MSG_WO_NVM     (TEST_RSP_MSG_SIZE - NVM_BUF_SIZE)
//
//
//#define NVM_RIGBASEPAGE1        (0x0000)
//#define NVM_RIGBASEPAGE2        (0x0080)
//#define NVM_CURRENTPOINTER      (0x0120)
//#define NVM_SERIALNUMBER        (0x0178)
//#define NVM_STATEBASE           (0x0180)
//
#define NVM_RIGSIZE                     sizeof(Nvm_Rigging_t)
#define NVM_RIGSIZE_BYTES               (NVM_RIGSIZE * BYTES_PER_WORD)
#define NVM_RIGSIZE_LESS_CRC_BYTES      (NVM_RIGSIZE_BYTES - 2)
#define NVM_RIG_WORDS_TO_CLEAR_PER_CYCLE (16U)

//#define NVM_RIGSIZEP1       sizeof(Nvm_RvdtPosition_t)  /* Page 1 is just the RVDT positions*/
//#define NVM_RIGSIZEP2       NVM_RIGSIZE - NVM_RIGSIZEP1
//
//#define NVM_STATESIZE                sizeof(Nvm_State_t)
//#define NVM_STATEPARTITIONSIZE       NVM_SIZE - NVM_STATEBASE
//#define NVM_CURRENTPOINTERSIZE       sizeof(CurrentStatePointers_t)
//#define NVM_STATE_BLOCK_SHIFT        6 /* Power of 2 increment of State Block Size*/
//                                       /*   (64 bytes block = 2^6, so 6 would go here)*/
//
//#define NVM_RUNTIMECOUNTERSIZE       sizeof(RunTimeCounters_t)

#define MAX_STATE_RECORDS (NUMBER_OF_FAULT_RECORDS) /* Max number of State Records in NVM*/

#define NVM_STATE_DEFAULTS { 0 }
//#define NVM_POINTER_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
//
#define RIG_STATUS_DEFAULTS { 0 }

///* Rigging Default Positions */
//#define RIG_RSL_QUAD_POSITION_DEFAULT   (-450) /* RSL ON-SIDE, RMS X-SIDE  */
//#define RIG_N1_QUAD_POSITION_DEFAULT    (-304) /* N1 ON-SIDE, RMS X-SIDE */
//#define RIG_ZERO_QUAD_POSITION_DEFAULT  (   0) /* ZERO ON-SIDE, RMS X-SIDE  */
//#define RIG_P1_QUAD_POSITION_DEFAULT    ( 317) /* P1 ON-SIDE, RMS X-SIDE */
//#define RIG_P2_QUAD_POSITION_DEFAULT    ( 463) /* P2 ON-SIDE, RMS X-SIDE */
//#define RIG_P3_QUAD_POSITION_DEFAULT    (1099) /* P3 ON-SIDE, RMS X-SIDE */
//#define RIG_P4_QUAD_POSITION_DEFAULT    (1457) /* P4 ON-SIDE, RMS X-SIDE */
//#define RIG_P5_QUAD_POSITION_DEFAULT    (1975) /* P5 ON-SIDE, RMS X-SIDE */
//#define RIG_P6_QUAD_POSITION_DEFAULT    (2076) /* P6 ON-SIDE, RMS X-SIDE */
//#define RIG_P7_QUAD_POSITION_DEFAULT    (2500) /* P7 ON-SIDE, RMS X-SIDE */
//#define RIG_P8_QUAD_POSITION_DEFAULT    (3029) /* P8 ON-SIDE, RMS X-SIDE */
//#define RIG_P9_QUAD_POSITION_DEFAULT    (3556) /* P9 ON-SIDE, RMS X-SIDE */
//#define RIG_P10_QUAD_POSITION_DEFAULT   (4079) /* P10 ON-SIDE, RMS X-SIDE */
//#define RIG_P11_QUAD_POSITION_DEFAULT   (4544) /* P11 ON-SIDE, RMS X-SIDE */
//#define RIG_ESL_QUAD_POSITION_DEFAULT   (4700) /* ESL ON-SIDE, RMS X-SIDE  */

/*      Public Interface Function Prototypes
*/
void Nvm_InitReadRig(void);
void Nvm_ReadRigging(Nvm_Rigging_t *);
bool_t Nvm_WriteRigging(Nvm_Rigging_t *);
bool_t Nvm_ClearRigging(void);

void Nvm_InitReadState(void);
void Nvm_ReadState(Nvm_State_t *);
bool Nvm_WriteState(Nvm_State_t *);
Uint32 Nvm_ClearState(Uint32);
void Nvm_GetState(Uint16, Nvm_State_t *);

int16 Nvm_GetLastFailState(Nvm_State_t *);
int16 Nvm_GetNextToLastFailState(Nvm_State_t *);
//
Uint32 Nvm_Format(Uint32);
void Nvm_Init(void);

void RigModeScheduler_RigVerification(void);
void RigModeScheduler_Start_Rig(void);
void Nvm_ValidateRigStatus(void);
void Nvm_RigStatusSetup(RIG_STATUS);

void Nvm_WriteOutStateInformation(void);

///* The following Functions are intended to be used mainly internally
// to the NVM function.*/
//Uint16 Nvm_Read(unsigned long);
//void Nvm_Write(unsigned long, const Uint16 *, unsigned int);
bool Nvm_Erase(unsigned long, unsigned int);
//bool Nvm_StateWritePending(void);
void Nvm_InitReadCurrentStatePointer(void);
Uint16 Nvm_GetCurrentStatePointer(void);
//
//void Nvm_ResetRigDefaults(void);
//
void Nvm_SetPosOnside(RIG_POSITIONS_T position);
int Nvm_GetPosOnside(RIG_POSITIONS_T position);
#if defined(__HALLX_CONFIGURED)
void Nvm_SetPosXside(RIG_POSITIONS_T position);
int Nvm_GetPosXside(RIG_POSITIONS_T position);
#endif

void bytecpy_state( Uint16 *dest, const Nvm_State_t *state );

extern tNmvData_t tNvm;
extern tFaultData_t tFault;
extern bool InitiateStateWrite;
//extern bool RigWriteSuccess;
extern Nvm_Rigging_t Nvm_Rigging_Temp;
extern Nvm_State_t Nvm_State;
extern Nvm_State_t Nvm_State_Temp;
extern Nvm_State_t Nvm_Powerdown_State;
extern Nvm_State_t Current_NVM_State;
extern Nvm_QuadPosition_t xSideRigPosData[NUM_RIG_POSITIONS];
extern Timer_t StatusWriteTimer;
extern RigStatus_t Rigging_Status;
extern bool StartOfRig;
extern bool riggingCheckComplete;
extern uint16_t u16FinalCheckTimer;
extern bool rigWriteInitiate;
extern Uint16 BoardSerialNumber;
extern bool NVM_ClearInProgress;
extern bool NVM_FormatComplete;
extern bool NVM_StateInfoCleared;
extern bool NVM_RigInfoCleared;
extern bool NVM_AllDataSent;
//extern bool Nvm_BootModePwrDwn;

extern Timer_t RunTimeCountTimer;
extern Uint32 RunTimeCountMinutes;

#endif

/* end nvm.h*/

