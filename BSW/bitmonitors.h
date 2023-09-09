/****************************************************************************************************
*  File name: bitmonitors.h
*
*  Purpose: Interface file for Built-In-Test content.
*    This file describes the Interface for the BIT monitor handler functions.  The interface provides
*    routines necessary to execute BIT testing for the MCU.
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

#ifndef BITMONITORS_H__
#define BITMONITORS_H__

/*      Include Files */
#include "parameter.h"
//#include "dsp281x.h"
#include "bit.h"
#include "csci.h"

/*      Public Defines */
#define BIT_MAXMONITORS     0x7F          /* defines an upper bound for the monitor matrix tables */
//#define NULL                0             /* defines NULL as 0 */
#define NUM_OF_FRAMES       8             /* defines the number of scheduler frames the system has */

#define BIT_PBIT            0x0001        /* defines PBIT monitor type as 1, used to determine when a monitor can be executed */
#define BIT_CBIT            0x0002        /* defines CBIT monitor type as 2, used to determine when a monitor can be executed */
#define BIT_IBIT            0x0004        /* defines IBIT monitor type as 4, used to determine when a monitor can be executed */
#define STACK_END           0x800         /* defines the size of the stack the system is going to use, used for Monitor 0x79 */

#define BIT_ROMTEST_CRC_ADDR    0x3F7FF0  /* Address in ROM of CRC_ID for ROM test, used for the ROM CRC test, monitor 0x70 */

/*  Hardware divider for VBUS_MON = 100K / 5.23K Divider = 5.23/105.23 = 0.0497 Scale factor*/

/*  Input to A/D converter =  2.0 * 0.497 = 0.099V;  
    A/D Counts = (0.099V / 3.00Vref)*4095 = 135 counts */
#define BIT_VBUSMONCRIT_LIMIT   135       /* Maximum of "Critical" range for Bus voltage A/D reading (0.099V) */

/*  Input to A/D converter = 18.0 * 0.497 = 0.895V;  
    A/D Counts = (0.895V / 3.00Vref)*4095 = 1222 counts */
#define BIT_VBUSMONFAIL_LIMIT   1222      /* Maximum of "Failure" range for Bus voltage A/D reading (0.895 V) */

/*  Input to A/D converter = 22.0 * 0.497 = 1.093V;  
    A/D Counts = (1.093V / 3.00Vref)*4095 = 1492 counts */
#define BIT_VBUSMONNOBRK_LIMIT  1492      /* Maximum of "No Brake" range for Bus voltage A/D reading (1.093 V) */

/*  Input to A/D converter = 24 * 0.497 = 1.1928V;  
    A/D Counts = (1.1928V / 3.00Vref)*4095 = 1628 counts */
#define BIT_VBUSMONDROOP_LIMIT  1628      /* Maximum of "Droop" range for Bus voltage A/D reading (1.1928 V) */

//#define BIT_WDTEST_mSEC         105 /* WD timeout 105*125us = 13.1ms */
#define BIT_WDTEST_mSEC         124 /* WD timeout 124*125us = 15.5ms */

#define BIT_ILLEGALHALL_LIMIT       10	 /*  Maximum allowable illegal Hall sequences */
#define BIT_ILLEGALHALLSTATE_LIMIT	250  /*  Maximum allowable illegal Hall states */

/*Lower limit & Upper Limit for different voltages*/
/*Suffix used for variables,  VC- Variable Camber Mode, HL- High Lift Mode, LL- Lower Limit, UL- Upper Limit*/
#define VBUS270_VSENSE_VC_LL        205  /* VBUS270_VSENSE_VC_LL = ( 0.165V / 3.3Vref)*4095 */
#define VBUS270_VSENSE_VC_UL        212   /* VBUS270_VSENSE_VC_HL = ( 0.171V / 3.3Vref)*4095 */
#define VBUS270_VSENSE_HL_LL        1971  /* VBUS270_VSENSE_HL_LL = ( 1.588V / 3.3Vref)*4095 */
#define VBUS270_VSENSE_HL_UL        2050  /* VBUS270_VSENSE_HL_UL = ( 1.652V / 3.3Vref)*4095 */
#define VBUS28_VSENSE_LL            2894  /* VBUS28_VSENSE_LL = ( 2.332V / 3.3Vref)*4095 */
#define VBUS28_VSENSE_UL            3013  /* VBUS28_VSENSE_UL = ( 2.428V / 3.3Vref)*4095 */
#define POSITIVE_V15_SENSE_LL       3526  /* POSITIVE_V15_SENSE_LL = ( 2.583V / 3Vref)*4095 */
#define POSITIVE_V15_SENSE_UL       3598  /* POSITIVE_V15_SENSE_UL = ( 2.636V / 3Vref)*4095 */
#define NEGATIVE_V15_SENSE_LL       3324  /* NEGATIVE_V15_SENSE_LL = ( 2.435V / 3Vref)*4095 */
#define NEGATIVE_V15_SENSE_UL       3453  /* NEGATIVE_V15_SENSE_UL = ( 2.53V / 3Vref)*4095 */
#define V5_SENSE_LL                 3378  /* V5_SENSE_LL = (2.475V / 3Vref)*4095 */
#define V5_SENSE_UL                 3447  /* V5_SENSE_UL = (2.525V / 3Vref)*4095 */

/*#defines for One phase open circuit fault and phase to neutral asymmetrical fault are commented as these are now part of PDI*/
/*Can be deleted*/
/*Phase Neutral Short Asymmertic fault Threshold for Highlift Mode*/
//#define PhNeuShortAsym_HL_UL        2318U  /*PhaseNeutralShortAsym_HL_UL = 1.5A, (2318-2047)* 0.00554228 ~= 1.5A*/
/*Phase Neutral Short Asymmetric fault Threshold for VC mode*/
//#define PhNeuShortAsym_VC_UL        2227U  /*PhaseNeutralShortAsym_VC_UL = 1A, (2227-2047)* 0.00554228 ~= 1A*/
//#define OnePhOpenCkt_LL             2053U  /*OnePhaseOpenCkt_LL = 32mA, (2053-2047)* 0.00554228 ~= 33mA*/
//#define UnbalCurrIndex_UL           0.2f  /*Unbalance Current Index Threshold is 0.2*/


#define BAD_HALL_MONITORS_STOP_STATE (50U) /* 250 ms duartion. 5ms period * 50 = 250ms*/
#define ICC_INVALID_CRC_COUNTER  (10U)  /* 10 times CRC mismatch*/
#define ICC_TIMOUT_LIMIT         (3U)   /* 12 milliseconds*/

/*      Public Type Definitions */
/* brief Structure to store definition information for a fault monitor */
struct bitmonitor_s
{
    Uint16          type;           /* type of test: CBIT, IBIT, PBIT, or some combination thereof */
    CRITICALITY     criticality;    /* Criticality level of the monitor */
    Bit_TestSetup_t *test;          /* points to function used to set up test conditions; may be NULL for CBIT and some PBIT tests */
    Bit_Check_t     *tripcheck;     /* points to function used to measure trip condition */
    Bit_Effect_t    *effect;        /* points to function used to execute trip effect */
    Bit_Check_t     *resetcheck;    /* points to function used to measure reset condition(s) */
    Uint16          persistRpt;     /* number of times trip condition must repeat to trigger effect */
    Uint32          persistTime;    /* amt of time trip condition must persist to trigger effect (in ms?) */
};

typedef enum
{
    TST_BIDIRECT_INIT = 0U,
    TST_BIDIRECT_INIT_CMD,
    TST_BIDIRECT_INIT_MOTOR_START,
    TST_BIDIRECT_DELAY_MOTOR_START,
    TST_BIDIRECT_START_MOTOR,
    TST_BIDIRECT_MOVE_TO_POSITION,
    TST_BIDIRECT_WAIT_TO_RETRY,
    TST_BIDIRECT_WAIT_TO_RETRACT,
    TST_BIDIRECT_INIT_TEST_COMPLETE_SEQUENCE,
    TST_BIDIRECT_COMPLETE
}tBiDirectTstStates_t;

typedef enum
{
    TST_RIG_VERIFY_INIT = 0U,
    TST_RIG_VERIFY_RIG_STATUS,
    TST_RIG_VERIFY_CHK_QUAD_ONSIDE,
#if defined(__HALLX_CONFIGURED)
    TST_RIG_VERIFY_CHK_QUAD_XSIDE,
#endif
    TST_RIG_VERIFY_CHK_RVDT_STROKE,
    TST_RIG_VERIFY_FINAL_CHECK_STATE = TST_RIG_VERIFY_CHK_RVDT_STROKE
}tRigVerifyTstStates_t;

typedef enum    // Eventually move to BLD when we Create it
{
    CSCI_INTEGRITY_CHECK_PASSED,
    CSCI_HEADER_INVALID,
    CSCI_INTEGRITY_CHECK_FAILED
} tCSCI_Status;

/* brief Structure type for handling the different functionality for the individual monitors in the system. */
typedef struct bitmonitor_s bitmonitor_t;


/*      Public Variable Declarations */
extern const CSCI_TypeHeader CSCI_APP_Header;
extern const CSCI_TypeHeader CSCI_PDI_HEADER;
extern Bit_t bitStatus[BIT_MAXMONITORS + 1];                    /* Data structure used to maintain the status of the individual monitors */
extern const bitmonitor_t bitMonitors[BIT_MAXMONITORS + 1];     /* Data structure contianing handling data for each specific monitor */
extern Timer_t brakeHoldTimer;                                  /* Timer variable used as part of executing the BrakeHold test (Monitor 0x23) */
extern Timer_t hwAsymMonitorTimer;                              /* Timer used to help facilitate the execution of the HwAsyTest (Monitor 0x58) */
extern Timer_t Bit_XCommsDelayTimer;                            /* Timer used to delay the execution of XComms (Monitor 0x07) */
extern Timer_t Bit_PhNeuDelayTimer;                             /*Timer used to delay the execution of Phase to Neutral Fault (Monitor 0x2A)*/
extern Timer_t Bit_OpCktDelayTimer;                             /*Timer  used to delay the execution of start Open Circuit Fault (Monitor 0x2B)*/
extern bool BridgeMonTestCmplt;
extern bool BrakeSwitchTestCmplt;
extern bool BrakeHoldTestCmplt;
extern bool HwAsymTestComplete;
extern bool bRigVerifyTstCmplt;
extern int32 uncomInitialFpsuOnside;                            /* Variable used to caputre the state of the OnSide FPSU position when the system stops */
extern int32 uncomInitialFpsuCross;                             /* Variable used to caputer the state of the CrossSide FPSU position when the system stops */
extern Uint32 xCommsInitialCount;

extern int16 uncomInitialQuadOnside;                            /* Variable used to caputre the state of the OnSide Quad position when the system stops */
extern int16 uncomInitialQuadCross;                             /* Variable used to caputer the state of the CrossSide Quad position when the system stops */
extern Uint16 PowerDownTrpCnt;                                  /* Variable used to determine how may consecutive sample of a drooping +14V have occured indicating a powerdown situation */

extern Uint16 BridgeMonStage;
extern Uint16 BrakeSwitchMonStage;
extern Uint16 HwAsymStage;
extern Uint16 FreePlayStage;
extern tBiDirectTstStates_t tBiDirectTstState;
extern tRigVerifyTstStates_t tRigVerifyTstState;
extern Timer_t underspeedMonitorTimer;          /* Timer used to start the execution of the underSpeed monitor */
extern tMuxInputs MuxInputs;                                    /* Variable used to capture state of nFAULT & READY signals*/
extern float32_t G_f32imbal_index;              /*Variable used to store the current imbal index*/
extern Uint16 BitmonitorsIllegalHalls;
/*      Public ROM Constants */

/*      Public Interface Function Prototypes */
extern void dsp_reset(void);
extern Uint16 stack_pointer(void);
bool IBIT_RamTest(void);
extern bool latchedFault_eff( int16 );
extern bool latchedWarning_eff( int16 );
extern bool inhibit_eff( int16 idx );

#endif
/* end bitmonitors.h */

