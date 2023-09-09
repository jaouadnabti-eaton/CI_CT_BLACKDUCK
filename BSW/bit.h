/****************************************************************************************************
*  File name: bit.h
*
*  Purpose: Interface file for Built-In-Test management.
*      This file describes the Public Interface for the BIT framework.  The interface provides
*      routines necessary to manage, invoke, and monitor BIT testing.
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

#ifndef BIT_H__
#define BIT_H__

/*      Include Files
*/
#include "parameter.h"
//#include "dsp281x.h"
#include "timer.h"
//#include "panel.h"

/*
  defgroup builtintest Built-In-Test Framework
  
  brief The BIT Framework is an abstraction for invoking and managing BIT tests.

    The Built-In-Test (BIT) Framework is designed to manage the collection of fault monitors, as defined by the Fault
    Monitor Matrix in the Product Specification requirements.  Each monitor is represented by an entry in the tables
    provided by the framework.  There is a table that contains the Definition for each fault monitor, and there is a
    separate table that contains the Status of each fault monitor.  The tables are separate so that the status can be
    easily stored off to NVM without extra overhead.

    The Trip and Reset Conditions defined for each fault monitor are implemented by handler functions of the type 
    ::Bit_Check_t.  These functions have a boolean return type to allow the framework to quickly determine a positive 
    (condition exists) or negative (tested condition does not exist) result.

    The Trip Effects defined for each fault monitor are implemented by handler functions of the type ::Bit_Effect_t.
    These are the functions that will trigger state changes or ARINC825 notification changes, or whatever is the defined
    effect, and is invoked for a given monitor only when the Trip Condition is met with the defined Persistence.

    The Test effects defined for fault monitors are also implemented by handlers of the type ::Bit_Effect_t.  These
    functions are used particularly for IBIT-type tests to perform operations that set up testable situations.  Each
    IBIT monitor should use a single Test effect handler, and use the stage input argument to perform multiple test
    steps one at a time, if necessary.

    The Bit_CheckMonitor() function is the routine that invokes the handler routines for the given fault monitor.  The
    handlers should not be invoked directly by external users, and should be declared as static functions within the
    implementation file.
*/


/*      Public Type Definitions
*/

typedef enum
{
    NO_FAULT,   /* this is not a valid fault code*/
    WARNING = NO_FAULT, /* Warning, not a fault */
    INHIBIT,    /* the fault is an inhibit*/
    LATCHED,    /* the fault is a latched fault*/
    CRITICAL    /* the fault is a critical fault*/
} CRITICALITY;

typedef enum
{
    NOT_STARTED,
    IN_PROCESS,
    COMPLETED
} IBIT_STATUS;

/* brief Structure type for managing a single BIT monitor*/

/* The framework implements an array of these entries as a table that reflects the Monitor Matrix from the requirements.*/
typedef struct bitstatus_s Bit_t;

/*brief Bitfield for storing overall BIT status flags*/
struct bitflags_s
{
    Uint16 FrameOverrun:1;      /* Frame overrun flag*/
    Uint16 reserved:15;         /* reserved bits*/
};

/* brief Structure type for maintaining overall status of BIT tests*/

/*Implemented as a bitfield, and stored as part of Status partition in NVM*/
typedef union
{
    Uint16 all;
    struct bitflags_s bit;
} Bit_Flags_t;

/*
    brief Function type for trip/reset event check routines

    Each Monitor will have its own "check" routines to observe trip and reset conditions.  The check routine must return
    a boolean value that tells the caller whether the test passed or failed.

    param  stat        pointer to Bit_t structure containing information about this BIT monitor

    retval true        Test is positive, meaning that the tested condition exists
    retval false       Test is negative, meaning that the tested condition does not exist
*/
typedef bool Bit_Check_t( Bit_t *stat );

/*
    brief Function type for effect execution routines

    Each monitor, when tripped, will invoke an "effect" function that causes the required effect for that monitor.  It
    should be expected that some effect functions will be used by more than one monitor.

    retval true        Test effect is complete
    retval false       Test effect could not be completed
*/
typedef bool Bit_Effect_t( int16 idx );

/*
    brief Function type for test setup routines

    This function type is to be used for operating IBIT- or PBIT-type test procedures prior to 
    and after checking for trip conditions.

    param  stage       integer describing the step in the test procedure

    retval true        Test effect is complete, including all steps in a multipart test
    retval false       Test effect is incomplete and should be invoked again to complete the multipart test effect
*/
typedef bool Bit_TestSetup_t( Uint16  stage );


/* brief Structure to store management information about a BIT monitor*/
struct bitstatus_s
{
    Uint16 tripCount;      /* any positive trip check increments count; count cleared on negative tripcheck*/
    Uint32 tripTime;       /* snapshot of free-running counter to measure persistence; cleared on negative tripcheck*/
    Uint16 tripBoot;       /* Nonzero if test fails and effect() should be/has been invoked; value corresponds to MCU's Boot Count at time of fault occurrence.  Cleared by successful resetcheck.*/
    Uint16 data;           /* storage for diagnostic information about tripped fault. See also ICD7020A, GSE word 17.*/
};

struct WDtest_s
{
    Uint32 startTime;
    Uint32 endTime;
    bool PBitServiced;
};

typedef struct WDtest_s WDtest_t;

/* brief Bitfield for storing status of the individual Inhibit monitor flags*/
struct InhibitMonitors_s
{
    Uint16 FlapJam_M:1;                 /* b00:  Monitor 0x04 */
    Uint16 XChannelComm_M:1;            /* b01:  Monitor 0x07 */
	Uint16 MotorPhaseOverTemp_M:1;      /* b02:  Monitor 0x16 */
    Uint16 bA825MaintBusCommsStale:1;   /* b03:  Monitor 0x4F */
    Uint16 reserved3:3;                 /* b04-b06 Reserved */
    Uint16 b07:1;                       /* b07:  */
    Uint16 bA825MaintBusFlapCmdSkew:1;  /* b08:  Monitor 0x49 */
    Uint16 b09:1;                       /* b09    */
    Uint16 b10:1;                       /* b10:   */
    Uint16 b11:1;                       /* b11:   */
    Uint16 b12:1;                       /* b12:   */
    Uint16 b13:1;                       /* b13:   */
    Uint16 b14:1;                       /* b14:   */
    Uint16 b15:1;                       /* b15:   */
};

/*brief Structure type for maintaining overall status of Inhibit monitors*/

/* Implemented as a bitfield*/
typedef union
{
    Uint16 all;
    struct InhibitMonitors_s bit;
} InhibitMonitors_t;

/* brief Bitfield for storing status of the individual Latching monitor flags*/
struct LatchedFaults_s
{
    Uint16 b00:1;                   /* b00:  */
    Uint16 HallSequence_M:1;        /* b01:  Monitor 0x21 */
    Uint16 BrakeSwitch_M:1;         /* b02:  Monitor 0x25 */
    Uint16 RvdtExcitationFault:1;   /* b03:  Monitor 0x30 */
    Uint16 b04:1;                   /* b04:   */
    Uint16 b05:1;                   /* b05:   */
    Uint16 b06:1;                   /* b06:   */
    Uint16 BridgeMonitor_M:1;       /* b07:  Monitor 0x50 */
    Uint16 b08_available:1;         /* b08:  Monitor 0x40 */
    Uint16 InvGateDriver_M:1;       /* b09:  Monitor 0x56 */
    Uint16 ProgramPin_M:1;          /* b10:  Monitor 0x57 */
    Uint16 b11:1;                   /* b11:  Monitor 0x65 */
    Uint16 RamCheck_M:1;            /* b12:  Monitor 0x71 */
    Uint16 WDPowerUp_M:1;           /* b13:  Monitor 0x73 */
    Uint16 FrameOverrun_M:1;        /* b14:  Monitor 0x74 */
    Uint16 SubFrameIndex_M:1;       /* b15:  Monitor 0x76 */
    Uint16 UndefinedInt_M:1;        /* b16:  Monitor 0x77 */
    Uint16 StackOverrun_M:1;        /* b17:  Monitor 0x79 */
    Uint16 InvalidHall_M:1;         /* b18:  Monitor 0x22 */
    Uint16 b19:1;                   /* b19:  */
    Uint16 bA825CntrlBusPassive:1;    /* b20:  Monitor 0x4A */
    Uint16 bA825MaintBusPassive:1;    /* b21:  Monitor 0x4B */
    Uint16 bA825CntrlBusOff:1;        /* b22:  Monitor 0x4C */
    Uint16 bA825MaintBusOff:1;        /* b23:  Monitor 0x4D */
    Uint16 bA825CntrlBusCommsStale:1; /* b24:  Monitor 0x4E */
    Uint16 b25:1;                     /* b25:  */
    Uint16 DcBusCha_M:1;              /* b26:  Monitor 0X51 */
    Uint16 p28vdcCha_M:1;             /* b27:  Monitor 0X52 */
    Uint16 p15vSense_M:1;             /* b28:  Monitor 0X53 */
    Uint16 n15vSense_M:1;             /* b29:  Monitor 0X54 */
    Uint16 p5vSense_M:1;              /* b30:  Monitor 0X55 */
    Uint16 reserved:1;    	          /* b31:  Reserved */
};

/* brief Structure type for maintaining overall status of Latching monitors*/

/* Implemented as a bitfield*/
typedef union
{
    Uint32 all;
    struct LatchedFaults_s bit;
} LatchedFaults_t;

/* brief Bitfield for storing status of the individual Critical monitor flags*/
struct CriticalFaults_s
{
    Uint16 MramCheck_M:1;                   /* b0: Monitor 0x7C */
    Uint16 GfdSense_M:1;                    /* b1: Monitor 0x7D */
    Uint16 OverSpeed_M:1;                   /* b2: Monitor 0x05 */
    Uint16 EncoderHealthFault_M:1;          /* b3: Monitor 0x36 */
    Uint16 PhaseNeutralShortAsym_M:1;       /* b4: Monitor 0x2A */
    Uint16 OnePhaseOpenCkt_M:1;             /* b5: Monitor 0x2B */
    Uint16 RigValue_M:1;                    /* b6: Monitor 0x10 */
    Uint16 RigModeFault_M:1;                /* b7: Monitor 0x12 */
    Uint16 b08:1;                           /* b8:  */
    Uint16 b09:1;                           /* b9:  */
    Uint16 BrakeHold_M:1;                   /* b10: Monitor 0x23 */
    Uint16 b11:1;                           /* b11: */
    Uint16 b12:1;                           /* b12: */
    Uint16 b13:1;                           /* b13: */
    Uint16 b14:1;                           /* b14: */
    Uint16 RiggingCheck_M:1;                /* b15: Monitor 0x64 */
    Uint16 b16:1;                           /* b16: */
    Uint16 ROMcrcCheck_M:1;                 /* b17: Monitor 0x70 */
    Uint16 PDI_CRC_Check_M:1;               /* b18: Monitor 0x7A */
    Uint16 PDI_OOR_Check_M:1;               /* b19: Monitor 0x7B */
    Uint16 reserved0:2;                     /* b20-b21: Reserved */
    Uint16 SfBiasCheck_M:1;                 /* b22: Monitor 0x1A */
    Uint16 SfMeasResidualCheck_M:1;         /* b23: Monitor 0x1B */
    Uint16 reserved1:8;                     /* b24-b31:  Reserved */
};

/* brief Structure type for maintaining overall status of Critical monitors */

/* Implemented as a bitfield */
typedef union
{
    Uint32 all;
    struct CriticalFaults_s bit;
} CriticalFaults_t;

/* brief Bitfield for storing status of the individual Latching monitor warning flags */
struct LatchedWarnings_s
{
    uint16_t bMcuDisabled:1;              /* b00:  Monitor 0x02 */
    uint16_t bRvdtPositionFault_M:1;      /* b01:  Monitor 0x34 */
    uint16_t xChannelEnable_M:1;          /* b02:  Monitor 0x06 */
    uint16_t xChanneCRC_M:1;              /* b03:  Monitor 0x07 */
    uint16_t p15vSense_M:1;               /* b04:  Monitor 0x53 */
    uint16_t n15vSense_M:1;               /* b05:  Monitor 0x54 */
    uint16_t reserved:10;                 /* b06-b15:  Reserved */
};

/* brief Structure type for maintaining overall status of Latched Warning monitors*/

/* Implemented as a bitfield*/
typedef union
{
    uint16_t all;
    struct LatchedWarnings_s bit;
} LatchedWarnings_t;

typedef struct 
{
    Uint16 TestID;
    Uint16 TestSupport;
    Uint32 TestData;
    bool result;
    IBIT_STATUS tstCmplt;
} Ibit_Status;

/*      Public Variable Declarations
*/
extern Uint16 LatestFaultCode;                  /* Contains the fault code of the monitor that last tripped */
extern Uint16 LatestFaultData;                  /* Contains the fault data of the monitor that last tripped*/
extern Uint16 SystemFaultCode;
extern Uint16 SystemFaultData;
extern bool xSideJamFlg;                        /* Flag indicating if a jam  condition occured on the opposite channel */
extern bool onSideJamFlg;                       /* Flag indicating if a jam condition occured on the onside channel */
extern Uint16 BadSubframeCnt;                   /* Variable used to determine the number of bad subframes that have occured */
extern bool OnSideFault;                        /* Flag indicating if a fault is active on the onside channel */
extern bool XSideSpiFault;                      /* Flag indicating if a fault is active on the opposite channel */
extern bool XSideFaultNvmFlg;                   /* Flag used to dermine what value is written to the upper byte of the Mode word */
extern bool OnGroundCondition;                  /* Flag used to indicate if a valid onGround Condition exists */
extern bool PowerdownFault;                     /* Flag used to indicate if Monitor 59 tripped during the previous boot cycle */
extern Ibit_Status Ibit_Stat;                   /* Data structure containing the status of the latest IBIT test */
extern bool ROM_CRC_done;                       /* Flag used to indicate when the ROM_CRC monitor is done executing */
extern bool PDI_CRC_done;                       /* Flag used to indicate when the ROM PDI CRC monitor is done executing */
extern bool RAM_Test_done;                      /* Flag used to indicate when the RAM_Test monitor is done executing */
extern bool RIG_CRC_done;                       /* Flag used to indicate when the RIG_CRC monitor is done executing */
extern Uint16 Ibit_AsymOffset_val;              /* Variable input used durig TEST_MODE to set the AsymOffset voltage */
extern Uint16 FlightCycleCount;                 /* Variable used to determine how many flights the system has had */

extern CriticalFaults_t CriticalFaults_Stat;    /* Data structure containing the status of the Critical faults */
extern LatchedFaults_t LatchedFaults_Stat;      /* Data structure containing the status of the Latched faults */
extern InhibitMonitors_t Inhibits_Stat;         /* Data structure containing the status of the Inhibits */
extern LatchedWarnings_t LatchedWarnings_Stat;  /* Data structure containing the status of the Latched warnings */
extern bool Bit_FslTimeMtr;                         /* Flag used to indicate if the FSL enable time out has occurred */
extern bool Bit_FslPosMtr;                          /* Flag used to indicate if the FSL position agreement time out has occurred */

extern Timer_t IBITDelayTimer;
extern Timer_t ADCDelayTimer;
extern Timer_t WarmBootDelay;
extern Timer_t CreepDelay;
extern Timer_t tBiDirectTstTimer;

extern bool CreepTestStart;
extern bool CreepTestCmplt;
extern Uint16 CreepPhase;
extern Timer_t CreepMonitorTimer;
extern Timer_t CreepBrakeTimer;
extern bool PwrDownWrite;

extern WDtest_t WDTestStatus;
extern bool IBITFault;
extern Uint16 b200msmonStage;

/*      Public ROM Constants
*/

/*      Public Defines
*/
#define BITTEST_MAX 127

#define COLD_BOOT 1
#define WARM_BOOT 0

/* Watchdog Defines */
#define WDENINT_MASK         0xFFFDU    /* Mask off WDENINT bit of WD SCSR */
#define ENABLE_WDRST         0x0000U    /* Watchdog Reset Enabled:      WDRSTn output signal Enabled. WDINTn output signal Disabled */
#define ENABLE_WDINT         0x0002U    /* Watchdog Interrupt Enabled:  WDINTn output signal Enabled. WDRSTn output signal Disabled */
#define RESET_WDRS      1U

/*      Public Interface Function Prototypes
*/
bool Bit_DoTest( Uint16 testId, Uint16 testStage ); /*  operates system in predefined manner for test purposes*/
bool Bit_CheckMonitor( Uint16 testId );             /*  returns true if 'tripped' is set, false otherwise*/
bool FaultCheck( void );

void Bit_Pbit( Uint16 subframe );                   /*  operates all PBIT tests*/
bool Bit_Cbit( Uint16 subframe );
void Bit_Rbit( Uint16 subframe );
void Bit_Ibit (Uint16 testId);
bool InhibitCheck(Uint16 FaultId, Uint16 FaultData);
void Bit_ReportToGse(Uint16 index);
bool Bit_LogToNvm( Uint16 index );
bool Bit_CheckIBIT(Uint16 testId);

void Bit_InhibitMonitors(Uint16 testId, bool runTest);
void Bit_LatchedMonitors(Uint16 testId, bool runTest);
void Bit_CriticalMonitors(Uint16 testId, bool runTest);
bool Bit_JamCheck( void );
void Bit_FaultReporting(void);

void Bit_Run_CBit(Uint16 subframe);
CRITICALITY Bit_GetFaultCriticality(Uint16 faultId);
void SetSystemCriticality(CRITICALITY set);
CRITICALITY GetSystemCriticality(void);

extern void Bit_StartDog(void);

void Bit_InitializeStructures(void);
void Bit_HandleLatchedFaults(void);

#endif
/*  end bit.h */

