/****************************************************************************************************
*  File name: bitmonitors.c
*
*  Purpose: Provides the BIT monitors trip, test and reset functions.
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
#include "PDI_Mgr.h"
#include "nvm.h"
#include "gpio.h"
#include "mcu.h"
#include "actuation.h"
#include "adc.h"
#include "hallsensor.h"
#include "skewsensor.h"
#include "motor.h"
#include "brake.h"
#include "timer.h"
#include "bitmonitors.h"
#include "spi.h"
#include "csci.h"
#include "crc.h"
#include "A825_MsgInfo.h"
#include "panel.h"
#include "icc.h"

/*      Local Type Definitions */
/* 0x19:  Bi-Directional Movement Check Monitor data typedef */
typedef struct
{
    uint16_t bAttemptCnt:4; /* bits 0-3 */
    uint16_t bPriChnl:1;    /* b4 Primary Channel Identifier */
    uint16_t bSecChnl:1;    /* b5 Secondary Channel Identifier */
    uint16_t bExtend:1;     /* b6 Extend State */
    uint16_t bRetract:1;    /* b7 Retract State */
    uint16_t u8Unused:8;    /* bits 8-15 unused */
} tBiDirectDataBits;

typedef union
{
    uint16_t all;
    tBiDirectDataBits   bit;
} t16BiDirectDataByteBits;

/* 0x64:  Rig Verify Check Monitor data typedef */
typedef struct
{
    uint16_t uRigPosition:4; /* bits 0-3: Identifier of the Rig Position */
    uint16_t uRigTest:4;     /* bits 4-7: Identifier of the the Various Rig Tests */
    uint16_t u8Unused:8;     /* bits 8-15 unused */
} tRigVerifyDataBits;

typedef union
{
    uint16_t all;
    tRigVerifyDataBits   bit;
} t16RigVerifyDataByteBits;

/*      Local Defines */


/*-----------------------------------------------------------------
 A/D voltage reading design limits and their ideal reading values

  Ideal count = voltage*(full-scale count)/(full-scale voltage) 
              = voltage*(4095)/(3.0)
 
  signal                        level         a/d in    counts
  -----------------             ------        ------    ------
    VBUS critical limit         2 VDC         0.1       137
    VBUS failure limit          18 VDC        0.9       1229
    VBUS no brake limit         22 VDC        1.1       1502
    VBUS droop limit            24 VDC        1.2       1638
    Motor overcurrent           6.5 A         2.9       3959
    Motor jam current           6 A           2.7       3686
    Motor undercurrent          1 A           0.7       956
    +14v supply minimum         ? VDC         2.44      3331
    +14v supply maximum         ? VDC         2.62      3576
    MTR_MON low threshold
    MTR_MON high threshold
    ASYM_OFFSET signal ideal    0.417 VDC     0.417     569
    ASYM_OFFSET sgnl tolerance  0.010 VDC     0.010     14
    Asymmetry limit on FPSU     1.89 degFlap  0.062     85
    Uncommanded position limit  0.2 degFlap   0.007     10
    RVDT SUM signal ideal       2.275 VDC     2.275     3105
    RVDT SUM sgnl tolerance     0.2275 VDC    0.2275    310
    RVDT POS sgnl mismatch lmt  0.1428 VDC    0.1428    195 
-----------------------------------------------------------------*/ 

#define BIT_OVERSPEED_LIMIT       6067   /*  Maximum valid speed for motor, in RPM */
#define BIT_UNDERSPEED_LIMIT      1700   /*  Minimum valid operating speed for motor, in RPM */
#define BIT_WRONGDIR_LIMIT        172    /*  Maximum quad count difference for wrong direction */

#define BIT_JAM_QUAD_LIMIT        10     /*  Ten quadrature counts is the limit for detecting flap jams */
#define BIT_JAMCURRENT_LIMIT      2170   /*  Threshold for motor current A/D reading denoting "flap jam" (1.59 V) */
#define BIT_UNDERCURRENT_LIMIT    956    /*  Minimum of valid range for motor current A/D reading (0.7 V) */

/* Monitor 0x25 */
#define BIT_BRKOFF_MIN            710    /*  Minimum of valid range for BRK_MON A/D reading (0.52 V) with both brake signals off */
#define BIT_BRKOFF_MAX            1365   /*  Maximum of valid range for BRK_MON A/D reading (1.00 V) with both brake signals off */
#define BIT_BRKUPPER_MIN          1856   /*  Minimum of valid range for BRK_MON A/D reading (1.36 V) with upper brake signal on */
#define BIT_BRKUPPER_MAX          3276   /*  Maximum of valid range for BRK_MON A/D reading (2.40 V) with upper brake signal on */
#define BIT_BRKLOWER_MAX          137    /*  Maximum of valid range for BRK_MON A/D reading (0.10 V) with lower brake signal on */

/* Monitor 0x50 */
#define BIT_BRIDGENONE_MIN        710    /*  Minimum of valid range for MTR_MON A/D reading (0.52 V) with all uppers and lowers off */ 
#define BIT_BRIDGENONE_MAX        1365   /*  Maximum of valid range for MTR_MON A/D reading (1.00 V) with all uppers and lowers off */      
#define BIT_BRIDGEUPPER_MIN       1856   /*  Minimum of valid range for MTR_MON A/D reading (1.36 V) with an upper on */
#define BIT_BRIDGELOWER_MAX       137    /*  Maximum of valid range for MTR_MON A/D reading (0.10 V) with a lower on */
#define BIT_VBUS_MAX              1188   /*  Maximum of valid range for VBUS A/D reading (0.87 V) */

#define BIT_MTRMONLOW_THRESHOLD   1000   /*  Threshold below which MTR_MON is considered 'low' (tbd V) */
#define BIT_MTRMONHIGH_THRESHOLD  3000   /*  Threshold above which MTR_MON is considered 'high' (tbd V) */

#define BIT_ASYMOFFSET_IDEAL      569    /*  Design ideal value of ASYM_OFFSET analog signal (0.417 V) */
#define BIT_ASYMOFFSET_TOLERANCE  14     /*  Tolerance on ASYM_OFFSET analog signal (0.010 V) */

#define BIT_CREEP_LIMIT           10     /*  Ten quadrature counts is maximum change for creep monitor */
#define BIT_UNCOMMANDEDQUAD_LIMIT 15     /*  Fifteen quadrature counts is maximum change for uncommanded motion monitor */
#define BIT_FREEPLAY_LIMIT        600    /*  Sixty-five quadrature counts is maximum change for free play */
#define BIT_FREEPLAY_MOVE         5
#define BIT_QUAD_LIMIT            65

#define BIT_CAL_EXTEND_LIMIT      20     /*  Amount of quad counts the flaps need to travel when extending during cal. */
#define BIT_CAL_EXTEND_TOLERANCE  5      /*  Tolerance on the extend quad count limit */
#define BIT_CAL_RETRACT_LIMIT     -5      /*  Amount of quad counts the flaps need to travel when retracting during cal */

#define BIT_BIDIRECT_MOVE_QUAD_CNTS 20  /* Bi-Directional test number of quad counts to move in both directions. */
#define BIT_BIDIRECT_QUAD_TOLERANCE 2   /* Tolerance on the quad count target position */

/* Sensor Fusion Bias and Measurement Residual Monitors Tolerances */
#define BIT_SF_BIAS_TOLERANCE               (0.08F * 2)  /* Should be 0.08":  Sensor Fusion Bias tolerance in inches */
#define BIT_SF_RESIDUAL_SKEW_TOLERANCE      (0.05F * 2)  /* Should be 0.05":  Sensor Fusion Measurement Residual tolerance in skew stroke inches (f32 format) */
#define BIT_SF_RESIDUAL_QUAD_TOLERANCE      (3.0F * 2)   /* Should be 3.0 Cnts:  Sensor Fusion Measurement Residual tolerance in quad counts (f32 format) */

#define BIT_SPIFAIL_LIMIT		  11	 /*  11 consecutive bad SPI messages is the maximum allowed for the xcomms monitor */

/* 89mV * (1.000V/1000mV)*(4095 Counts/3.000V)*(694 FPSU Deg/32768 Counts) * (16 16ths Deg / 1.0 Deg) = 41.17 (16th's FPSU Deg) */
#define BIT_UNCOMMANDEDFPSU_LIMIT 40     /* Changed by PR7613 to match removal of software normalization  */

#define BIT_ASYMMETRICFPSU_LIMIT  170

/* Rigging Verification Tolerances */
#define BIT_QUADDIFF_LIMIT               5
#define BIT_RIG_DELTA_STROKE_LIMIT      (0.02F * 5.0F) /* Should be 0.02" of stroke max threshold for comparison of sensor fusion position, quad, and skew stroke positions */
#define BIT_RIG_DELTA_STROKE_QUAD_LIMIT (BIT_RIG_DELTA_STROKE_LIMIT) /* Should be 0.02" of stroke max threshold for comparison of quad to stroke vs SF stroke */
/* Max allowable difference is 0.006" Rigging Error + 0.012" Demodulation Error = 0.018" between ideal stroke and calibrated stroke calculated from RVDT ADC */
#define BIT_RIG_DELTA_RVDT_CALIBRATION_STROKE_LIMIT (0.018F)
/* RVDT linear least squares equation thresholds */
#define BIT_RVDT_EQUATION_SLOPE_THRESHOLD       (0.05)      /* Allow up to 5% difference from ideal to actual */
#define BIT_RVDT_EQUATION_SLOPE_UPPER_LIMIT     (1.0F + BIT_RVDT_EQUATION_SLOPE_THRESHOLD)
#define BIT_RVDT_EQUATION_SLOPE_LOWER_LIMIT     (1.0F - BIT_RVDT_EQUATION_SLOPE_THRESHOLD)
#define BIT_RVDT_EQUATION_OFFSET_THRESHOLD      (200.0F)     /* Allow up to 200 ADC counts of offset for line of best fit */
#define BIT_RVDT_EQUATION_OFFSET_UPPER_LIMIT    (BIT_RVDT_EQUATION_OFFSET_THRESHOLD)
#define BIT_RVDT_EQUATION_OFFSET_LOWER_LIMIT    (-BIT_RVDT_EQUATION_OFFSET_THRESHOLD)

#define BIT_CMDACTINCMPLT         70

#define BIT_RVDTSUM_IDEAL         3105   /*  Design Ideal value of RVDT "Sum" analog signals (2.275 V) */
#define BIT_RVDT_TOLERANCE        310    /*  Tolerance ('k' value) on RVDT analog signals (0.2275 V) */
#define BIT_RVDTPOS_MISMATCH      195    /*  Tolerance of mismatch on RVDT "Pos" analog signals (0.1428 V) */

#define ASYM_DAC_DIFF             778    /*  Tolerance for Asymmetry circuit, DAC counts. */

#define BIT_ROM_CRC_ADDR          0x3F7FF0
#define BIT_ROM_CRC_VAL           0x0

#define BIT_RAMTEST_M1_START      0x000400    /*  Starting address, inclusive, of RAM region under BIT test (M0-M1) */
#define BIT_RAMTEST_M1_END        0x000800    /*  Ending address, non-inclusive, of RAM region under BIT test (M0-M1) */
#define BIT_RAMTEST_LS_START      0x008000    /*  Starting address, inclusive, of RAM region under BIT test (L0-L1) */
#define BIT_RAMTEST_LS_END        0x00B000    /*  Ending address, non-inclusive, of RAM region under BIT test (L0-L1) */
#define BIT_RAMTEST_GS_START      0x00C000    /*  Starting address, inclusive, of RAM region under BIT test (H0) */
#define BIT_RAMTEST_GS_END        0x01C000    /*  Ending address, non-inclusive, of RAM region under BIT test (H0) */
#define BIT_RAMTEST_PATTERN1      0xffff
#define BIT_RAMTEST_PATTERN2      0xaaaa
#define BIT_RAMTEST_PATTERN3      0x5555
#define BIT_RAMTEST_PATTERN4      0x0000

#define BIT_MRAM_TEST_START       0x100000  /* Starting address of MRAM */
#define BIT_MRAM_SIZE             0x040000  /* Size of MRAM */
#define BIT_MRAM_TEST_END         (BIT_MRAM_TEST_START + BIT_MRAM_SIZE) /* Ending address non-inclusive. */

#define BIT_WDTEST_uSEC           2000000L    /*  2 seconds, in microseconds units */

/*      Global Variables */
float32_t G_f32imbal_index;         //variable is made global as it will be used in DFT

/*      Local ROM Constants */


/*      Local Variable Declarations */
int16   uncomInitialQuadOnside = 0;
int16   uncomInitialQuadCross = 0;
float32_t   uncomInitialFlapAngleOnside = 0;
float32_t   uncomInitialFlapAngleXside = 0;
Uint32  xCommsInitialCount = 0;
int16   wrongdirInitialOnQuad = 0;
#if defined(__HALLX_CONFIGURED)
int16   wrongdirInitialXQuad = 0;
#endif
Uint16  freeplayInitialQuad = 0;
Uint16  freeplayFinalQuad = 0;
int16   s16BiDirectTstTargetPosition = 0;
int16   brakeholdInitialQuad = 0;
bool    b200msmonComplete = false;
bool    BridgeMonTestCmplt = false;
bool    BrakeSwitchTestCmplt = false;
bool    BrakeHoldTestCmplt = false;
bool    HwAsymTestComplete = false;
bool    bRigVerifyTstCmplt = false;
Timer_t b200msmonTimer = TIMER_DEFAULTS;
Timer_t brakeHoldTimer = TIMER_DEFAULTS;
Timer_t hwAsymMonitorTimer = TIMER_DEFAULTS;
Timer_t FlapJamTimer = TIMER_DEFAULTS;
Timer_t Bit_XCommsDelayTimer = TIMER_DEFAULTS;
Timer_t Bit_PhNeuDelayTimer = TIMER_DEFAULTS;               /*Timer to start Phase to Neutral Fault*/
Timer_t Bit_OpCktDelayTimer = TIMER_DEFAULTS;               /*Timer to start Open Circuit Fault*/
bool    ROM_CRC_done = false;
bool    PDI_CRC_done = false;
bool    RAM_Test_done = false;
bool    RIG_CRC_done = false;
int16   dac_out1 = 0;
int16   dac_out2 = 0;
Uint16  BridgeMonStage = 0;
Uint16  BrakeSwitchMonStage = 0;
Uint16  FreePlayStage = 0;
tBiDirectTstStates_t tBiDirectTstState = TST_BIDIRECT_INIT;
tRigVerifyTstStates_t tRigVerifyTstState = TST_RIG_VERIFY_INIT;
Uint16  HwAsymStage = 1;
Timer_t Mon63CaseBTimer = TIMER_DEFAULTS;

/* These are declared locally but should be moved elsewhere */
bool Inhibit_UnderspeedMonitor = false;
bool Inhibit_Rig = false;

Timer_t underspeedMonitorTimer = TIMER_DEFAULTS;

Uint16 BitmonitorsIllegalHalls = 0;
extern bool_t bRunSensorFusion;

/* end to be moved */

/* Note: commented-out lines in the following table are reserved for future use; they are not needed for SOF */

/* brief Definition for fault monitor status table */
Bit_t bitStatus[BIT_MAXMONITORS + 1] = 
{
    /*00*/ {0, 0, 0, 0},
    /*01*/ {0, 0, 0, 0},
    /*02*/ {0, 0, 0, 0},
    /*03*/ {0, 0, 0, 0},
    /*04*/ {0, 0, 0, 0},
    /*05*/ {0, 0, 0, 0},
    /*06*/ {0, 0, 0, 0},
    /*07*/ {0, 0, 0, 0},
    /*08*/ {0, 0, 0, 0},
    /*09*/ {0, 0, 0, 0},
    /*0A*/ {0, 0, 0, 0},
    /*0B*/ {0, 0, 0, 0},
    /*0C*/ {0, 0, 0, 0},
    /*0D*/ {0, 0, 0, 0},
    /*0E*/ {0, 0, 0, 0},
    /*0F*/ {0, 0, 0, 0},
    /*10*/ {0, 0, 0, 0},
    /*11*/ {0, 0, 0, 0},
    /*12*/ {0, 0, 0, 0},
    /*13*/ {0, 0, 0, 0},
    /*14*/ {0, 0, 0, 0},
    /*15*/ {0, 0, 0, 0},
    /*16*/ {0, 0, 0, 0},
    /*17*/ {0, 0, 0, 0},
    /*18*/ {0, 0, 0, 0},
    /*19*/ {0, 0, 0, 0},
    /*1A*/ {0, 0, 0, 0},
    /*1B*/ {0, 0, 0, 0},
    /*1C*/ {0, 0, 0, 0},
    /*1D*/ {0, 0, 0, 0},
    /*1E*/ {0, 0, 0, 0},
    /*1F*/ {0, 0, 0, 0},
    /*20*/ {0, 0, 0, 0},
    /*21*/ {0, 0, 0, 0},
    /*22*/ {0, 0, 0, 0},
    /*23*/ {0, 0, 0, 0},
    /*24*/ {0, 0, 0, 0},
    /*25*/ {0, 0, 0, 0},
    /*26*/ {0, 0, 0, 0},
    /*27*/ {0, 0, 0, 0},
    /*28*/ {0, 0, 0, 0},
    /*29*/ {0, 0, 0, 0},
    /*2A*/ {0, 0, 0, 0},
    /*2B*/ {0, 0, 0, 0},
    /*2C*/ {0, 0, 0, 0},
    /*2D*/ {0, 0, 0, 0},
    /*2E*/ {0, 0, 0, 0},
    /*2F*/ {0, 0, 0, 0},
    /*30*/ {0, 0, 0, 0},
    /*31*/ {0, 0, 0, 0},
    /*32*/ {0, 0, 0, 0},
    /*33*/ {0, 0, 0, 0},
    /*34*/ {0, 0, 0, 0},
    /*35*/ {0, 0, 0, 0},
    /*36*/ {0, 0, 0, 0},
    /*37*/ {0, 0, 0, 0},
    /*38*/ {0, 0, 0, 0},
    /*39*/ {0, 0, 0, 0},
    /*3A*/ {0, 0, 0, 0},
    /*3B*/ {0, 0, 0, 0},
    /*3C*/ {0, 0, 0, 0},
    /*3D*/ {0, 0, 0, 0},
    /*3E*/ {0, 0, 0, 0},
    /*3F*/ {0, 0, 0, 0},
    /*40*/ {0, 0, 0, 0},
    /*41*/ {0, 0, 0, 0},
    /*42*/ {0, 0, 0, 0},
    /*43*/ {0, 0, 0, 0},
    /*44*/ {0, 0, 0, 0},
    /*45*/ {0, 0, 0, 0},
    /*46*/ {0, 0, 0, 0},
    /*47*/ {0, 0, 0, 0},
    /*48*/ {0, 0, 0, 0},
    /*49*/ {0, 0, 0, 0},
    /*4A*/ {0, 0, 0, 0},
    /*4B*/ {0, 0, 0, 0},
    /*4C*/ {0, 0, 0, 0},
    /*4D*/ {0, 0, 0, 0},
    /*4E*/ {0, 0, 0, 0},
    /*4F*/ {0, 0, 0, 0},
    /*50*/ {0, 0, 0, 0},
    /*51*/ {0, 0, 0, 0},
    /*52*/ {0, 0, 0, 0},
    /*53*/ {0, 0, 0, 0},
    /*54*/ {0, 0, 0, 0},
    /*55*/ {0, 0, 0, 0},
    /*56*/ {0, 0, 0, 0},
    /*57*/ {0, 0, 0, 0},
    /*58*/ {0, 0, 0, 0},
    /*59*/ {0, 0, 0, 0},
    /*5A*/ {0, 0, 0, 0},
    /*5B*/ {0, 0, 0, 0},
    /*5C*/ {0, 0, 0, 0},
    /*5D*/ {0, 0, 0, 0},
    /*5E*/ {0, 0, 0, 0},
    /*5F*/ {0, 0, 0, 0},
    /*60*/ {0, 0, 0, 0},
    /*61*/ {0, 0, 0, 0},
    /*62*/ {0, 0, 0, 0},
    /*63*/ {0, 0, 0, 0},
    /*64*/ {0, 0, 0, 0},
    /*65*/ {0, 0, 0, 0},
    /*66*/ {0, 0, 0, 0},
    /*67*/ {0, 0, 0, 0},
    /*68*/ {0, 0, 0, 0},
    /*69*/ {0, 0, 0, 0},
    /*6A*/ {0, 0, 0, 0},
    /*6B*/ {0, 0, 0, 0},
    /*6C*/ {0, 0, 0, 0},
    /*6D*/ {0, 0, 0, 0},
    /*6E*/ {0, 0, 0, 0},
    /*6F*/ {0, 0, 0, 0},
    /*70*/ {0, 0, 0, 0},
    /*71*/ {0, 0, 0, 0},
    /*72*/ {0, 0, 0, 0},
    /*73*/ {0, 0, 0, 0},
    /*74*/ {0, 0, 0, 0},
    /*75*/ {0, 0, 0, 0},
    /*76*/ {0, 0, 0, 0},
    /*77*/ {0, 0, 0, 0},
    /*78*/ {0, 0, 0, 0},
    /*79*/ {0, 0, 0, 0},
    /*7A*/ {0, 0, 0, 0},
    /*7B*/ {0, 0, 0, 0},
    /*7C*/ {0, 0, 0, 0},
    /*7D*/ {0, 0, 0, 0},
    /*7E*/ {0, 0, 0, 0},
    /*7F*/ {0, 0, 0, 0}
};


/* Note: commented-out function prototypes in the following are reserved for future use; they are not needed for SOF 
   name Trip Event check functions: System Health 
   These handler functions, of type ::Bit_Check_t, test for the existence/occurrence of fault conditions.  This group
   is for the "Flap System Health" category of fault monitors. */

static bool scuMcuEnable_trp( Bit_t *stat );
static bool flapJam_trp( Bit_t *stat );
static bool xenable_trp( Bit_t *stat );
static bool xcomms_trp( Bit_t *stat );
//static bool a825CntrlBusCommsMon_trp( Bit_t *stat );
//static bool a825MaintBusCommsMon_trp( Bit_t *stat );
//static bool a825CntrlBusFlapCmdSkewMon_trp( Bit_t *stat );
static bool a825MaintBusFlapCmdSkewMon_trp( Bit_t *stat );
static bool a825CntrlCommsStaleMon_trp( Bit_t *stat );
static bool a825MaintCommsStaleMon_trp( Bit_t *stat );
static bool a825CntrlBusPassiveMon_trp( Bit_t *stat );
static bool a825MaintBusPassiveMon_trp( Bit_t *stat );
static bool a825CntrlBusOffMon_trp( Bit_t *stat );
static bool a825MaintBusOffMon_trp( Bit_t *stat );

static bool rigCrc_trp( Bit_t *stat );
static bool rigPins_trp( Bit_t *stat );
static bool freePlay_trp( Bit_t *stat );
static bool sfBias_trp( Bit_t *stat );
static bool sfMeasurementResidual_trp( Bit_t *stat );
static bool rigVerify_trp( Bit_t *stat );


/* name Trip Event check functions: Motor Health 
 These handler functions, of type ::Bit_Check_t, test for the existence/occurrence of fault conditions.  This group
    is for the "Motor Health" category of fault monitors. */

static bool hallBadSeq_trp( Bit_t *stat );
static bool hallBadState_trp( Bit_t *stat );
static bool brakeHold_trp( Bit_t *stat );
static bool vbrakeMin_trp( Bit_t *stat );
static bool OverMotorWindingTemp_trp( Bit_t *stat );
static bool PhaseNeutralShortAsym_trp( Bit_t *stat);
static bool OnePhaseOpenCkt_trp( Bit_t *stat);

#if defined(__SKEW_SNSR_RVDT__)
/* name Trip Event check functions: RVDT Health 
   These handler functions, of type ::Bit_Check_t, test for the existence/occurrence of fault conditions.  This group
   is for the "RVDT Health" category of fault monitors. */
static bool rvdtExc_trp( Bit_t *stat );
static bool rvdtPos_trp( Bit_t *stat );
#endif

#if defined(__SKEW_SNSR_ENCODER__)
/* name Trip Event check functions: Encoder Health
   These handler functions, of type ::Bit_Check_t, test for the existence/occurrence of fault conditions.  This group
   is for the "Encoder Health" category of fault monitors. */
static bool encoderHealth_trp( Bit_t *stat );
#endif

/* name Trip Event check functions: Control Channel Health 
   These handler functions, of type ::Bit_Check_t, test for the existence/occurrence of fault conditions.  This group
   is for the "Control Channel Health" category of fault monitors. */

static bool bridgeMon_trp( Bit_t *stat );
static bool DcBusCha_trp( Bit_t *stat );
static bool p28vdc_trp( Bit_t *stat );
static bool p15vdc_trp( Bit_t *stat );
static bool n15vdc_trp( Bit_t *stat );
static bool p5vdc_trp( Bit_t *stat );
static bool prgmPins_trp( Bit_t *stat );
static bool InvGateDriver_trp(Bit_t *stat );


/* name Trip Event check functions: DSP Health 
   These handler functions, of type ::Bit_Check_t, test for the existence/occurrence of fault conditions.  This group
   is for the "DSP Health" category of fault monitors. */

static bool romCrc_trp( Bit_t *stat );
static bool ramTest_trp( Bit_t *stat );
static bool wdogTest_trp( Bit_t *stat );
static bool frameOrun_trp( Bit_t *stat );
static bool frameBad_trp( Bit_t *stat );
static bool intrptBad_trp( Bit_t *stat );
static bool stackOflow_trp( Bit_t *stat );
static bool pdiCrc_trp( Bit_t *stat );
static bool pdiOutOfRange_trp( Bit_t *stat );
static bool mramTest_trp( Bit_t *stat );
static bool gfd_i_sense_trp( Bit_t *stat );

/* name Trip Effect execution functions
   These handler functions, of type ::Bit_Effect_t, execute the "trip effects" called for in
    the Fault Monitor Matrix.  One of the first two functions, latchedFault_eff() or inhibit_eff(),
    is used by most of the fault monitors.*/

static bool flapJam_eff( int16 idx );
#if defined(__SKEW_SNSR_RVDT__)
static bool rvdtExcFault_eff( int16 idx );
#endif
static bool rigVerify_eff( int16 idx );
static bool hallBadSeq_eff( int16 idx );
static bool hallBadState_eff( int16 idx );
static bool brakeHold_eff( int16 idx );
static bool MotorPhaseCurrent_eff( int16 idx );



/* name Reset Condition check functions
These handler functions, of type ::Bit_Check_t, measure the MCU system for the reset conditions
    called for in the Fault Monitor Matrix.  One of the first two functions, groundCycle_rst() or
    powerCycle_rst(), is used by most of the fault monitors. */

/*------- "Reset Condition" check functions -------*/
static bool groundCycle_rst( Bit_t *stat );
static bool powerCycle_rst( Bit_t *stat );
static bool scuMcuEnable_rst( Bit_t *stat );
static bool xenable_rst( Bit_t *stat );
static bool rigVerify_rst( Bit_t *stat );
static bool flapJam_rst( Bit_t *stat );
static bool xcomms_rst( Bit_t *stat );
//static bool a825CntrlBusFlapCmdSkewMon_rst( Bit_t *stat );
static bool a825MaintBusFlapCmdSkewMon_rst( Bit_t *stat );
static bool a825MaintCommsStaleMon_rst( Bit_t *stat );
static bool OverMotorWindingTemp_rst( Bit_t *stat );
/* name Test Condition setup functions 
These handler functions, of type ::Bit_Effect_t, configure the MCU for certain test conditions
    necessary for certain fault monitors, especially IBIT-type monitors. */

static bool freePlay_tst( Uint16 stage );
static bool brakeHold_tst( Uint16 stage );
static bool vbrakeMin_tst( Uint16 stage );
static bool bridgeMon_tst( Uint16 stage );
static bool rigVerify_tst( Uint16 stage );

/* CSCI Integrity Check Function Prototype */
tCSCI_Status CSCI_IntegrityCheck(const CSCI_TypeHeader *ptr_csci_headeraddress);

/* Note: commented-out lines in the following table are reserved for future use; they are not needed for SOF */

/* brief Definition for fault monitors table. */
const bitmonitor_t bitMonitors[BIT_MAXMONITORS + 1] =
{
    /*id   type               criticality           testfn                       tripfn                       effectfn                     resetfn                      persistence(x, ms) */
    /*00*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*01*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*02*/{BIT_CBIT,          WARNING,              NULL,                        &scuMcuEnable_trp,           &latchedWarning_eff,         &scuMcuEnable_rst,           1, 0},
    /*03*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*04*/{BIT_CBIT,          INHIBIT,              NULL,                        &flapJam_trp,                &flapJam_eff,                &flapJam_rst,                1, 0},
    /*05*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*06*/{BIT_CBIT,          WARNING,              NULL,                        &xenable_trp,                &latchedWarning_eff,         &xenable_rst,                1, 0},
    /*07*/{BIT_CBIT,          WARNING,              NULL,                        &xcomms_trp,                 &latchedWarning_eff,         &xcomms_rst,                 1, 0},
    /*08*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*09*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*0A*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*0B*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*0C*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*0D*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*0E*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*0F*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*10*/{BIT_CBIT,          CRITICAL,             NULL,                        &rigCrc_trp,                 &latchedFault_eff,           &groundCycle_rst,            1, 0},
    /*11*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*12*/{BIT_PBIT|BIT_CBIT, CRITICAL,             NULL,                        &rigPins_trp,                &latchedFault_eff,           &groundCycle_rst,            1, 0},
    /*13*/{BIT_IBIT,          CRITICAL,             &freePlay_tst,               &freePlay_trp,               &latchedFault_eff,           &groundCycle_rst,            1, 0},
    /*14*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*15*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*16*/{BIT_CBIT,          INHIBIT,              NULL,                        &OverMotorWindingTemp_trp,   &inhibit_eff,                &OverMotorWindingTemp_rst,   1, 500},
    /*17*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*18*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*19*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*1A*/{BIT_CBIT,          CRITICAL,             NULL,                        &sfBias_trp,                 &latchedFault_eff,           &groundCycle_rst,            1, 100},
    /*1B*/{BIT_CBIT,          CRITICAL,             NULL,                        &sfMeasurementResidual_trp,  &latchedFault_eff,           &groundCycle_rst,            1, 0},
    /*1C*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*1D*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*1E*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*1F*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*20*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*21*/{BIT_CBIT,          LATCHED,              NULL,                        &hallBadSeq_trp,             &hallBadSeq_eff,             &powerCycle_rst,             1, 0},
    /*22*/{BIT_CBIT,          LATCHED,              NULL,                        &hallBadState_trp,           &hallBadState_eff,           &powerCycle_rst,          	1, 0},
    /*23*/{BIT_PBIT,          CRITICAL,             &brakeHold_tst,              &brakeHold_trp,              &brakeHold_eff,              &groundCycle_rst,            1, 0},
    /*24*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*25*/{BIT_PBIT|BIT_IBIT, LATCHED,              &vbrakeMin_tst,              &vbrakeMin_trp,              &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*26*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*27*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*28*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*29*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*2A*/{BIT_CBIT,          CRITICAL,             NULL,                        &PhaseNeutralShortAsym_trp,  &MotorPhaseCurrent_eff,      &groundCycle_rst,            1, 100},
    /*2B*/{BIT_CBIT,          CRITICAL,             NULL,                        &OnePhaseOpenCkt_trp,        &MotorPhaseCurrent_eff,      &groundCycle_rst,            1, 100},
    /*2C*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*2D*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*2E*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*2F*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
#if defined(__SKEW_SNSR_RVDT__)
    /*30*/{BIT_CBIT,          LATCHED,              NULL,                        &rvdtExc_trp,                &rvdtExcFault_eff,           &powerCycle_rst,             1, 100},
#else
    /*30*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
#endif
    /*31*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*32*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*33*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
#if defined(__SKEW_SNSR_RVDT__)
    /*34*/{BIT_CBIT,          LATCHED,              NULL,                        &rvdtPos_trp,                &latchedWarning_eff,         &powerCycle_rst,             1, 100},
#else
    /*34*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
#endif
    /*35*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
#if defined(__SKEW_SNSR_ENCODER__)
    /*36*/{BIT_CBIT,          CRITICAL,             NULL,                        &encoderHealth_trp,          &latchedFault_eff,           &groundCycle_rst,            1, 0},
#else
    /*36*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
#endif
    /*37*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*38*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*39*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*3A*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*3B*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*3C*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*3D*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*3E*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*3F*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*40*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*41*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*42*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*43*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*44*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*45*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*46*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*47*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*48*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*49*/{BIT_CBIT,          INHIBIT,              NULL,                        &a825MaintBusFlapCmdSkewMon_trp,    &inhibit_eff,         &a825MaintBusFlapCmdSkewMon_rst,  1, 0},
    /*4A*/{BIT_CBIT,          LATCHED,              NULL,                        &a825CntrlBusPassiveMon_trp, &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*4B*/{BIT_CBIT,          LATCHED,              NULL,                        &a825MaintBusPassiveMon_trp, &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*4C*/{BIT_CBIT,          LATCHED,              NULL,                        &a825CntrlBusOffMon_trp,     &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*4D*/{BIT_CBIT,          LATCHED,              NULL,                        &a825MaintBusOffMon_trp,     &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*4E*/{BIT_CBIT,          LATCHED,              NULL,                        &a825CntrlCommsStaleMon_trp, &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*4F*/{BIT_CBIT,          INHIBIT,              NULL,                        &a825MaintCommsStaleMon_trp, &inhibit_eff,                &a825MaintCommsStaleMon_rst, 1, 0},
    /*50*/{BIT_PBIT,          LATCHED,              &bridgeMon_tst,              &bridgeMon_trp,              &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*51*/{BIT_CBIT,          LATCHED,              NULL,                        &DcBusCha_trp,               &latchedFault_eff,           &powerCycle_rst,             1, 500},
    /*52*/{BIT_CBIT,          LATCHED,              NULL,                        &p28vdc_trp,                 &latchedFault_eff,           &powerCycle_rst,             1, 50},
    /*53*/{BIT_CBIT,          WARNING,              NULL,                        &p15vdc_trp,                 &latchedWarning_eff,         &powerCycle_rst,             1, 50},
    /*54*/{BIT_CBIT,          WARNING,              NULL,                        &n15vdc_trp,                 &latchedWarning_eff,         &powerCycle_rst,             1, 50},
    /*55*/{BIT_CBIT,          LATCHED,              NULL,                        &p5vdc_trp,                  &latchedFault_eff,           &powerCycle_rst,             1, 50},
    /*56*/{BIT_CBIT,          LATCHED,              NULL,                        &InvGateDriver_trp,          &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*57*/{BIT_PBIT,          LATCHED,              NULL,                        &prgmPins_trp,               &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*58*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*59*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*5A*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*5B*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*5C*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*5D*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*5E*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*5F*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*60*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*61*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*62*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*63*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*64*/{BIT_PBIT,          CRITICAL,             &rigVerify_tst,              &rigVerify_trp,              &rigVerify_eff,              &rigVerify_rst,              1, 0},
    /*65*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*66*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*67*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*68*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*69*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*6A*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*6B*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*6C*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*6D*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*6E*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*6F*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*70*/{BIT_PBIT,          CRITICAL,             NULL,                        &romCrc_trp,                 &latchedFault_eff,           &groundCycle_rst,            1, 0},
    /*71*/{BIT_PBIT,          LATCHED,              NULL,                        &ramTest_trp,                &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*72*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*73*/{BIT_PBIT,          LATCHED,              NULL,                        &wdogTest_trp,               &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*74*/{BIT_CBIT,          LATCHED,              NULL,                        &frameOrun_trp,              &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*75*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*76*/{BIT_CBIT,          LATCHED,              NULL,                        &frameBad_trp,               &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*77*/{BIT_CBIT,          LATCHED,              NULL,                        &intrptBad_trp,              &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*78*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*79*/{BIT_CBIT,          LATCHED,              NULL,                        &stackOflow_trp,             &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*7A*/{BIT_PBIT,          CRITICAL,             NULL,                        &pdiCrc_trp,                 &latchedFault_eff,           &groundCycle_rst,            1, 0},
    /*7B*/{BIT_PBIT,          CRITICAL,             NULL,                        &pdiOutOfRange_trp,          &latchedFault_eff,           &groundCycle_rst,            1, 0},
    /*7C*/{BIT_PBIT,          CRITICAL,             NULL,                        &mramTest_trp,               &latchedFault_eff,           &groundCycle_rst,            1, 0},
    /*7D*/{BIT_CBIT,          CRITICAL,             NULL,                        &gfd_i_sense_trp,            &latchedFault_eff,           &powerCycle_rst,             1, 0},
    /*7E*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0},
    /*7F*/{NULL,              NO_FAULT,             NULL,                        NULL,                        NULL,                        NULL,                        0, 0}
};

/*      Function Definitions */


/*------- "Trip Condition/Event" Check Functions -------*/
/*---------- System health check functions -------------*/

/**
    brief Checks Trip Event for the sych enable monitor, 0x02.  Function uses
        the form of function type ::Bit_Check_t.

    This function checks if the SCU to MCU Enable discrete input has disabled
    the MCU

    MTR_EN_CHA = LOW:   Enabled
    MTR_EN_CHA = HIGH:  Disabled
*/
static bool scuMcuEnable_trp( Bit_t *stat )
{
    bool status = false;

    if (tPdi.bSyncMonitorEnable == true)
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* Check for SCU to MCU Enable discrete input */
            if (MTR_EN_CHA == 0)
            {
                status = true;

                /* shall report 'fault data' as 0x01 for the Left channel, 0x02 for the Right channel */
                if (G_bChannelA == true)
                {
                    /* PATH(hwAsym_trp,E); */
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(hwAsym_trp,F); */
                    stat->data = 0x02;
                }
            }
        }
    }


   /* PATH(hwAsym_trp,G); */
   return (status);

}


/**
    brief Checks Trip Event for the Flap Jam fault monitor, 0x04.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects a "jammed" flap by measuring a high motor current along with minimal
    change in the quadrature count for a perion of 200ms.

	GlobalsReferenced
	Hall_t tHall
	Motor_Commands_t MotorCmd
	#define I_ANALG (Adc_Averaged.ch4)
	Timer_t FlapJamTimer

	retval	true: Jam is active in the system
			false: Jam is not active in the system

*/
static bool flapJam_trp( Bit_t *stat )
{
    bool status = false;
    static int16 LastHallCount = 0;
    int32 delta;

    if(tPdi.bJamMonitorEnable)
    {
        /* PATH(flapJam_trp,A); */
        delta = (int32)tHall.Position - (int32)LastHallCount;

        /* shall ensure that 'stat' pointer is not null */
        if (stat && (MotorCmd.MotorRunning == true))
        {
            /* PATH(flapJam_trp,B); */

            /* shall detect a flap jam if current goes above 6A and less than 10 quad counts are seen for 200ms */
            //if ((I_ANLG > BIT_JAMCURRENT_LIMIT) && ((delta < BIT_JAM_QUAD_LIMIT) && (delta > -BIT_JAM_QUAD_LIMIT))) // VLJ
            //if ((I_ANLG > tPdi.PDI_Monitor_Jam_Current_Limit) && ((delta < tPdi.PDI_Monitor_Jam_Quad_Limit) && (delta > -tPdi.PDI_Monitor_Jam_Quad_Limit)))                    // PDI
            // AB: Need to revisit when monitors are defined
            if (0)
            {
                /* PATH(flapJam_trp,C); */

                /* shall record the number of flap-jam events detected in the stat->data field */
                if (Timer_IsExpired(&FlapJamTimer) == true)
                {
                    /* PATH(flapJam_trp,D); */
                    status = true;
                    stat->data &= 0x0F;
                    stat->data++;

                    if (G_bChannelA == false)
                    {
                        /* PATH(flapJam_trp,E); */
                        stat->data |= 0x10;
                    }

                    LastHallCount = tHall.Position;
                }
            }
            else
            {
                /* PATH(flapJam_trp,F); */

                /* shall set hard stop timer for 200ms */
                Timer_SetTime(&FlapJamTimer, TIMER_ONESHOT, TIMER_200ms);
                LastHallCount = tHall.Position;
            }
        }
    }
    /* PATH(flapJam_trp,G); */
    return (status);
}

/**
    brief Checks Trip Event for the Cross-channel Health fault monitor, 0x06.  Function uses
        the form of function type ::Bit_Check_t.

    This function monitors the cross-channel status signal during normal operation.

*/
static bool xenable_trp( Bit_t *stat )
{
    bool status = false;

    if(tPdi.bxChMonitorEnable)
    {
        /* PATH(xenable_trp,A); */
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* PATH(xenable_trp,B); */

            //if ((SCU_MCU_ENABLE == true) && (CHX_STATUS == true))
            if (CHX_STATUS == false)
            {
                /* PATH(xenable_trp,C); */
                status = true;

                if (G_bChannelA == true)
                {
                    stat->data = 0x01;
                }
                else
                {
                    stat->data = 0x02;
                }
            }
        }
    }
    /* PATH(xenable_trp,J); */
    return (status);
}


/*
    brief Checks Trip Event for the Cross-channel communications fault monitor, 0x07.  Function uses
        the form of function type ::Bit_Check_t.

*/
static bool xcomms_trp( Bit_t *stat )
{
    bool status = false;

    /* PATH(xcomms_trp,A); */
    if(tPdi.bxChCommMonitorEnable)
    {
        /* shall return true only if the most recent cross-channel communications message is corrupt */
        if (stat)
        {
            /* PATH(xcomms_trp,B); */

            if (ICC_InvalidCRCcounter >= ICC_INVALID_CRC_COUNTER)
            {
                status = true;
               if (G_bChannelA == true)
               {
                   stat->data = 0x01;
               }
               else
               {
                   stat->data = 0x03;
               }
            }
            if (ICC_TimeOutCounter < ICC_TIMOUT_LIMIT)
            {
                ICC_TimeOutCounter++;
            }
            else
            {
                status = true;
               if (G_bChannelA == true)
               {
                   stat->data = 0x02;
               }
               else
               {
                   stat->data = 0x04;
               }

            }
        }

    }

    /* PATH(xcomms_trp,J); */
    return (status);
}


/**
    brief Checks Trip Event for the Rigging Corruption fault monitor, 0x10.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects corruption in the Rigging storage by comparing a computed CRC against
    the stored CRC.  Returns TRUE if there is no match.

*/
static bool rigCrc_trp( Bit_t *stat )
{
    static Uint16 nvmptr = 0;
    static Uint16 CRC16 = 0;
    Uint16 * ptr = (Uint16 *)&tNvm.tData.Nvm_Rigging;
    bool status = false;

    if(tPdi.bRigValueMonitorEnable)
    {
        /* PATH(rigCrc_trp,A); */
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* PATH(rigCrc_trp,B); */

            /* shall compute a portion of the NVM CRC on if the 'nvmptr' continues to point
               within the Rigging partition of NVM */
            if (nvmptr < (NVM_RIGSIZE - 1))
            {
                /* PATH(rigCrc_trp,C); */

                /* shall retrieve Rigging data from NVM, if NVM is not busy, and add the byte to the CRC computation */
                CRC16 = (CRC16 ^ (*(ptr + nvmptr)));

                if (CRC16 & 0x0001)
                {
                    /* PATH(rigCrc_trp,E); */
                    CRC16 = (CRC16 >> 1);
                    CRC16 = (CRC16 ^ BIT_RIGTEST_CRC_CODE);
                }
                else
                {
                    /* PATH(rigCrc_trp,F); */
                    CRC16 = (CRC16 >> 1);
                }

                /* shall increment 'nvmptr' after byte is added to CRC */
                nvmptr++;
            }
            else
            {
                /* PATH(rigCrc_trp,G); */

                /* shall compare computed CRC with stored CRC after CRC computation is complete, and
                   return true only if they do not match  */
                if (CRC16 != tNvm.tData.Nvm_Rigging.crc)
                {
                    /* PATH(rigCrc_trp,H); */
                    status = true;

                    /* shall report 'fault data' as 0x01 for the Left channel, 0x11 for the Right channel */
                    if (G_bChannelA == true)
                    {
                        /* PATH(rigCrc_trp,I); */
                        stat->data = 0x01;
                    }
                    else
                    {
                        /* PATH(rigCrc_trp,J); */
                        stat->data = 0x02;
                    }
                }

                CRC16 = 0;
                nvmptr = 0;
                RIG_CRC_done = true;
            }
        }
    }
    /* PATH(rigCrc_trp,K); */
    return (status);
}

/**
    brief Checks Trip Event for the Rig Mode pins fault monitor, 0x12.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects a circuit fault in the Rig Pins by verifying that they are in the proper
    state while not in Rig mode.  If a fault is detected, returns true.

*/
static bool rigPins_trp( Bit_t *stat )
{
    bool status = false;
    Uint16 rigpins = 0;

    if(tPdi.bRigModeFaultMonitorEnable)
    {
        /* PATH(rigPins_trp,A); */

        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* PATH(rigPins_trp,B); */

            /* shall use "raw" RIG pins while in BOOT_MODE, else use debounced inputs */
            if (McuGetState() == BOOT_MODE)
            {
                /* PATH(rigPins_trp,C); */

                /* shall read raw states of WOW pins if in BOOT mode */
                rigpins = RIG_N_RAW;
            }
            else
            {
                /* PATH(rigPins_trp,D); */

                /* shall read debounced states of WOW pins */
                rigpins = RIG_N;
            }

            if (McuGetState() != RIG_MODE)
            {
                /* PATH(rigPins_trp,E); */

                /* shall return true if any of the two RIG pins is active (low) while NOT in RIG_MODE. */
                if (rigpins == 0)
                {
                    /* PATH(rigPins_trp,F); */
                    status = true;

                    /* shall report state of RIG pins in fault data. */
                    stat->data = rigpins;
                }
            }
            else
            {
                /* PATH(rigPins_trp,G); */

                /* shall return true if both of the RIG pins are inactive (high) while in RIG_MODE. */
                if (rigpins != 0)
                {
                    /* PATH(rigPins_trp,H); */
                    status = true;

                    Nvm_State.faultId = 0x12;
                    Nvm_State.faultData = ((~rigpins) & 0x0003);
                }
            }
        }
    }
    /* PATH(rigPins_trp,I); */
    return (status);
}

/******** Reserved for future use -- not needed for SOF *******
**
    brief Checks Trip Event for the Backlash/Free Play fault monitor, 0x13.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects the amount of mechanical "free play" in the system by measuring
    the quadrature count while the flap load is reversed with the brake engaged.  If the
    quadrature count changes by more than 65, returns TRUE.

	GlobalsReferenced
	None

	retval	true: monitor 0x13 trip conditions are valid
			false: Monitor 0x13 trip conditions are invalid.
*/
static bool freePlay_trp( Bit_t *stat )
{
    bool status = false;
    int32 delta;

    if(tPdi.bBacklashFreePlayMonitorEnable)
    {
        /* PATH(freePlay_trp,A); */

        if (stat)
        {
            /* PATH(freePlay_trp,B); */

            /* shall measure quadrature changes since start of test condition, which was captured in global variable */
            delta = (int32)freeplayInitialQuad - (int32)freeplayFinalQuad;

            /* shall return true if quadrature change is greater than free-play threshold */
//            if (delta > BIT_QUAD_LIMIT)     // VLJ
            if (delta > tPdi.u16BacklashFreeplayMonitorQuadLimit)  // PDI
            {
                /* PATH(freePlay_trp,C); */

                if (G_bChannelA == true)
                {
                    /* PATH(freePlay_trp,D); */
                    stat->data = 0x01;
                }
                else
                {
                    /* PATH(freePlay_trp,E); */
                    stat->data = 0x02;
                }

                status = true;
            }
        }
    }
    /* PATH(freePlay_trp,F); */
    return (status);
}

/**
    brief Checks Trip Event for the Sensor Fusion Bias fault monitor, 0x1A.  Function uses
        the form of function type ::Bit_Check_t.

    This function checks if either the Sensor Fusion Bias output is out of tolerance.

*/
static bool sfBias_trp( Bit_t *stat )
{
    bool status = false;
    STATE_ID tMode = McuGetState();

    if(tPdi.bSensorFusionBiasCheckMonitorEnable)
    {
        /* ensure that 'stat' pointer is not null and this is either the primary or secondary side */
        if (stat)
        {
            /* trip bi-direcitonal monitor if onside didn't extend to the target within the specified threshold */
            if ((G_ASW_DATA.tPanelOutputData.f32BiasFused > BIT_SF_BIAS_TOLERANCE) ||
                (G_ASW_DATA.tPanelOutputData.f32BiasFused < -BIT_SF_BIAS_TOLERANCE))
            {
                if (G_bChannelA == true)
                {
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    stat->data = 0x02;
                }
                status = true;
            }
        }
    }

    return (status);
}

/**
    brief Checks Trip Event for the Sensor Fusion Measurement Residual fault monitor, 0x1B.  Function uses
        the form of function type ::Bit_Check_t.

    This function checks if either the Measurement Residual Quad Count or Skew Stroke Inches is out of
    tolerance.

*/
static bool sfMeasurementResidual_trp( Bit_t *stat )
{
    bool status = false;
    STATE_ID tMode = McuGetState();
    static uint16_t u16StartupDelay = 25U;  /* 25 counts @250Hz = 100ms delay on startup */

    /* Check if the startup delay has completed prior to running monitor */
    if(u16StartupDelay == 0U)
    {
        /* Startup delay is complete. Monitor can be ran now */
        if(tPdi.bSensorFusionMeasResidualMonitorEnable)
        {
            /* ensure that 'stat' pointer is not null and this is either the primary or secondary side */
            if (stat)
            {
                /* init fault data */
                stat->data = 0U;

                /* Trip Measurement Residual Monitor if residual skew is out of tolerance. */
                if ((G_ASW_DATA.tPanelOutputData.f32SkewResidualFused > BIT_SF_RESIDUAL_SKEW_TOLERANCE) ||
                    (G_ASW_DATA.tPanelOutputData.f32SkewResidualFused < -BIT_SF_RESIDUAL_SKEW_TOLERANCE))
                {
                    if (G_bChannelA == true)
                    {
                        stat->data |= 0x01;
                    }
                    else if (G_bChannelA == false)
                    {
                        stat->data |= 0x02;
                    }
                    status = true;
                }

                /* Trip Measurement Residual Monitor if quad count is out of tolerance. */
                if ((G_ASW_DATA.tPanelOutputData.f32QuadCntResidualFused > BIT_SF_RESIDUAL_QUAD_TOLERANCE) ||
                    (G_ASW_DATA.tPanelOutputData.f32QuadCntResidualFused < -BIT_SF_RESIDUAL_QUAD_TOLERANCE))
                {
                    if (G_bChannelA == true)
                    {
                        stat->data |= 0x10;
                    }
                    else if (G_bChannelA == false)
                    {
                        stat->data |= 0x20;
                    }
                    status = true;
                }
            }
        }
    }
    else
    {
        /* Service the startup delay counter */
        u16StartupDelay--;
    }

    return (status);
}

/**
    brief Checks Trip Event for the Rig Verification fault monitor, 0x64.  Function uses
        the form of function type ::Bit_Check_t.

    This function returns TRUE when an FPSU position does not match the FPSU position received from the
    cross-channel via SPI.
*/
static bool rigVerify_trp( Bit_t *stat )
{
    bool status = false;
    int32 delta;
    /* Pointer structure to view the monitor data with specific definition for this monitor */
    t16RigVerifyDataByteBits *ptData = (t16RigVerifyDataByteBits*) &stat->data;
    uint16_t i = (uint16_t) ptData->bit.uRigPosition;
    float32_t f32Delta;
    float32_t f32Stroke;
    float32_t f32PositionScaledCalibrated;

    if(tPdi.bRiggingCheckMonitorEnable)         // PDI
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* When in RIG_MODE verify all the rig data against tolerances. Trip if any are
            * outside the range.
            * Run only the TST_VERIFY_RIG_STATUS check when not in RIG_MODE (during PBIT).
            * It is not necessary to check all these rig values in modes
            * other than RIG_MODE since the rigCrc_Trp test will fault if any of the rig values
            * change. Only need to verify the values from rigging during the rigging routine. */

            /* Decode the State to determine which Rig Check to conduct */
            switch((tRigVerifyTstStates_t) ptData->bit.uRigTest)
            {
                case TST_RIG_VERIFY_RIG_STATUS:
                {
                    /* IMPORTANT:  This test is the only one designed to be called during PBIT */
                    /* Not in RIG_MODE:  Test if the rigstatus is NOT_RIGGED */
                    if((Nvm_State.rigstatus != RIG_COMPLETE) && (Nvm_State.rigstatus != RIG_VERIFIED))
                    {
                        /* Channel is not rigged. Trip fault */
                        status = true;
                    }
                    break;
                }
                /* IMPORTANT:  Rest of Tests are only designed to be called during RIG_MODE and not for PBIT */
                case TST_RIG_VERIFY_CHK_QUAD_ONSIDE:
                {
                    /* Check is only ran on positions N1 thru P11 */
                    if((i >= (uint16_t) N1) && (i <= (uint16_t) P11))
                    {
                        /* Validate the Actuator number for this channel to base the rig default settings on. */
                        if(G_eActuatorNumber < NUM_ACTUATOR_TYPES)
                        {
                            /* Check onside quad rig data */
                            //delta = (int32)Nvm_Rigging_Temp.quad[i].onside - (int32)xSideRigPosData[i].onside;
                            delta = (int32)Nvm_Rigging_Temp.quad[i].onside - s16RigPositionQuadCount[G_eActuatorNumber][i];

                            //if ((delta > BIT_QUADDIFF_LIMIT) || (delta < -BIT_QUADDIFF_LIMIT))
                            //if ((delta > tPdi.u16RiggingCheckMonitorQuadDiff) || (delta < -tPdi.u16RiggingCheckMonitorQuadDiff))      // PDI
                            if ((delta > 12) || (delta < -12))      // PDI
                            {
                                status = true;
                            }
                        }
                        else
                        {
                            /* error */
                            status = true;
                        }
                    }
                    break;
                }
#if defined(__HALLX_CONFIGURED)
                case TST_RIG_VERIFY_CHK_QUAD_XSIDE:
                {
                    /* Check is only ran on positions N1 thru P11 */
                    if((i >= (uint16_t) N1) && (i <= (uint16_t) P11))
                    {
                        /* Check xside quad rig data */
                        delta = (int32)Nvm_Rigging_Temp.quad[i].xside - (int32)xSideRigPosData[i].xside;

                        //if ((delta > BIT_QUADDIFF_LIMIT) || (delta < -BIT_QUADDIFF_LIMIT))
                        if ((delta > tPdi.u16RiggingCheckMonitorQuadDiff) || (delta < -tPdi.u16RiggingCheckMonitorQuadDiff))      // PDI
                        {
                            status = true;
                        }
                    }
                    break;
                }
#endif
                case TST_RIG_VERIFY_CHK_RVDT_STROKE:
                {
                    /* Check is only ran on positions N1 thru P11 */
                    if((i >= (uint16_t) N1) && (i <= (uint16_t) P11))
                    {
#if defined (__SKEW_SNSR_RVDT__)
                        /* Convert the 12-bit positional data into a fraction between 0 and 1 */
                        f32PositionScaledCalibrated = (float32_t)Nvm_Rigging_Temp.tSkewSnsr.tRvdtPosData[i].u16RvdtAvg * RVDT_CNT2FRAC;

                        /* Apply the calibration equation */
                        f32PositionScaledCalibrated = SKEW_SNSR_APPLY_CALIBRATION(f32PositionScaledCalibrated);
#elif defined (__SKEW_SNSR_ENCODER__)
                        uint32_t u32Position = Nvm_Rigging_Temp.tSkewSnsr.tEncoderPosData[i].u32Position;

                        /* Apply the rigging offset */
                        u32Position = Encoder_ApplyOffset(u32Position);

                        /* Convert the 12-bit positional data into a fraction between 0 and 1 */
                        f32PositionScaledCalibrated = (float32_t)u32Position * ENCODER_CNT2FRAC;
#endif
                        /* Calculate the Stroke */
                        f32Stroke = SkewSnsr_CalcStroke(G_eActuatorNumber, f32PositionScaledCalibrated);

                        /* Calculate difference in stroke between ideal and actual. */
                        f32Delta = f32Stroke - f32RigPositionStroke[G_eActuatorNumber][i];

                        /* Check if the difference is greater than allowable threshold */
                        if((f32Delta > BIT_RIG_DELTA_RVDT_CALIBRATION_STROKE_LIMIT) || (f32Delta < -BIT_RIG_DELTA_RVDT_CALIBRATION_STROKE_LIMIT))
                        {
                            /* Calibration equation does not satisfy accuracy required for RVDT strokes */
                            status = true;
                        }
                    }
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
    }
    return (status);
}


/*---------- Motor health check functions ----------*/

/*
    breif This fucntion will monitor the status of a counter that is
	incremented everytime a illegal sequence is detected.

	GlobalsReferenced
	Motor_Command_t MotorCmd
	bool PrimarySide

	retval	true: The number of bad transitions has surpassed the trip limit
			false: The number of bad transisitons has not surpassed the trip limit.
*/
static bool hallBadSeq_trp( Bit_t *stat )
{
    bool status = false;

    if(tPdi.bHallSensorSeqMonitorEnable)         // PDI
    {
        /* PATH(hallBadSeq_trp,A); */
        if ((stat) && ((G_bChannelA == true)||(G_bChannelA == false)))
        {
            /* PATH(hallBadSeq_trp,B); */
            /*Check to see if the number of bad transitions as surpassed the trip limit*/
//            if (MotorCmd.BadTransitions >= BIT_ILLEGALHALL_LIMIT)  // VLJ
            if (MotorCmd.BadTransitions >= tPdi.u16HallSensorSeqMonitorIllegalLimit)       // PDI
            {
                /* PATH(hallBadSeq_trp,C); */
                status = true;

                /*Set the appropriate fault data*/
                if (G_bChannelA == true)
                {
                    /* PATH(hallBadSeq_trp,D); */
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(hallBadSeq_trp,E); */
                    stat->data = 0x02;
                }
            }
        }
    }
    /* PATH(hallBadSeq_trp,F); */
    return (status);
}


/*
    brief Checks Trip Event for the Invalid Hall Sensor fault monitor, 0x22.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects a Hall sensor circuit fault when the 3 sensors all read 0 or 1 at the
    same time.

	GlobalsReferenced
	Motor_Commands_t MotorCmd
	bool PrimarySide

	retval	true: If the number of illegal halls that the system has detected has surpassed
					PDI_Monitor_Invalid_Hall_Count_Limit counts.
			false: If the number of illegal halls that the system has detected has not
					surpassed PDI_Monitor_Invalid_Hall_Count_Limit counts.
*/
static bool hallBadState_trp( Bit_t *stat )
{
    bool status = false;
    tHallWord_t tHallbitmonitor ;

    if(tPdi.bInvalidHallSensorMonitorEnable)     // PDI
    {
        /* PATH(hallBadState_trp,A); */
        if (stat)
        {
            /* PATH(hallBadState_trp,B); */

            /*Check to see if the number of illegal halls has surpassed the trip limit*/
            if (MotorCmd.LastBadHallState != GOOD_HALLS)
            {
                /* PATH(hallBadState_trp,C); */
                status = true;

                /*If the illegal halls were all 0's, set a specific fault data*/
                if (MotorCmd.LastBadHallState == ALL_ZEROS)
                {
                    /* PATH(hallBadState_trp,D); */

                    if (G_bChannelA == true)
                    {
                        /* PATH(hallBadState_trp,E); */
                        stat->data = 0x01;
                    }
                    else if(G_bChannelA == false)
                    {
                        /* PATH(hallBadState_trp,F); */
                        stat->data = 0x03;
                    }
                }
                /*If the illegal halls were all 1's, set a specific fault data*/
                else if (MotorCmd.LastBadHallState == ALL_ONES)
                {
                    /* PATH(hallBadState_trp,G); */

                    if (G_bChannelA == true)
                    {
                        /* PATH(hallBadState_trp,H); */
                        stat->data = 0x02;
                    }
                    else if(G_bChannelA == false)
                    {
                        /* PATH(hallBadState_trp,I); */
                        stat->data = 0x04;
                    }
                }
            }
            else if ((McuGetState() != RUN_MODE) && (McuGetState() != RIG_MODE) && (McuGetState() !=TEST_MODE)) /* check monitor in stop state only*/
            {
                tHallbitmonitor.all = 0;
                tHallbitmonitor.bit.bHall1 = CHA_HS1_RAW;   /* bit 0 */
                tHallbitmonitor.bit.bHall2 = CHA_HS2_RAW;   /* bit 1 */
                tHallbitmonitor.bit.bHall3 = CHA_HS3_RAW;   /* bit 2 */

                if ((tHallbitmonitor.all == 0) || (tHallbitmonitor.all == 7))          /* Record which bad hall state it is */
                {
                    if (BitmonitorsIllegalHalls < BAD_HALL_MONITORS_STOP_STATE) /* ensure condition valid for few ms*/
                    {
                        BitmonitorsIllegalHalls++; 
                    }
                    else
                    {
                        status = true;
                        if (tHallbitmonitor.all == 0)
                        {
                            MotorCmd.LastBadHallState = ALL_ZEROS;
                            if (G_bChannelA == true)
                            {
                                /* PATH(hallBadState_trp,H); */
                                stat->data = 0x05;
                            }
                            else
                            {
                            /* PATH(hallBadState_trp,I); */
                                stat->data = 0x07;
                            }
                        }
                        else
                        {
                            MotorCmd.LastBadHallState = ALL_ONES;
                           if (G_bChannelA == true)
                           {
                               /* PATH(hallBadState_trp,H); */
                               stat->data = 0x06;
                           }
                           else
                           {
                           /* PATH(hallBadState_trp,I); */
                               stat->data = 0x08;
                           }
                        }

                    }
                }
                else
                {
                    if (BitmonitorsIllegalHalls)
                    {
                        BitmonitorsIllegalHalls--;
                    }
                }

             }
        }
    }
    /* PATH(hallBadState_trp,J); */
    return (status);
}


/**
    brief Checks Trip Event for the Brake Holding fault monitor, 0x23.  Function uses
        the form of function type ::Bit_Check_t.

    This function monitors the quadrature counter to detect motion while the brake is engaged,
    during a power-up test condition (PBIT).

*/
static bool brakeHold_trp( Bit_t *stat )
{
    bool status = false;
    int32 delta;

    if(tPdi.bBrakeHoldMonitorEnable)      // PDI
    {
        /* PATH(brakeHold_trp,A); */

        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* PATH(brakeHold_trp,B); */

            /* shall measure quadrature changes since start of test condition, which was captured in global variable */
            delta = (int32)tHall.Position - (int32)brakeholdInitialQuad;

            if (CriticalFaults_Stat.bit.BrakeHold_M == 1)
            {
                /* PATH(brakeHold_trp,G); */
                status = true;
                stat->data = 0x09;
            }
            else if ((delta > BIT_CREEP_LIMIT) || (delta < -BIT_CREEP_LIMIT))
            {
                /* PATH(brakeHold_trp,C); */

                /* shall return true if quadrature change is greater than brake-hold (motor creep) threshold */
                status = true;

                /* shall report 'fault data' as 0x01 for Left channel, and 0x02 for Right channel */
                if (G_bChannelA == true)
                {
                    /* PATH(brakeHold_trp,D); */
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(brakeHold_trp,E); */
                    stat->data = 0x02;
                }
            }
        }
    }
    /* PATH(brakeHold_trp,F); */
    return (status);
}

/*
    brief Checks Trip Event for the Brake Switching fault monitor, 0x25.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects a brake circuitry fault by monitoring the Brake Voltage signal under
    a power-up test (PBIT) condition where the brake is activated.

	GlobalsReferenced
	#define BRK_MON (Adc_Averaged.ch5)
	bool PrimarySide


	retval	true: Jam is active in the system
			false: Jam is not active in the system

*/
static bool vbrakeMin_trp( Bit_t *stat )
{
    bool status = false;
	Uint32 brkmon_normalized = 0;

	if(tPdi.bBrakeSwitchMonitorEnable)
    {
	    /* PATH(vbrakeMin_trp,A); */

        /* shall return true if BRK_MON analog input is not within brake operational limit */
        if (stat)
        {
            /* PATH(vbrakeMin_trp,B); */
            /* Compute normalized value for BRK_MON, depending on VBUS_MON.
               1900 represents 1.3916 V on VBUS_MON, the nominal case for monitor tests.  */
            /* Rework for MCU when Brake is integrated */
//            if (VBUS_MON != 0)
//            {
//                /* PATH(vbrakeMin_trp,Q); */
//                brkmon_normalized = (1900 * BRK_MON) / VBUS_MON;
//            }
            brkmon_normalized = 1000;

            switch (BrakeSwitchMonStage)
            {
                case 1:
                    /* PATH(vbrakeMin_trp,C); */

                    /* Test for both brake signals off */
//                    if ((brkmon_normalized < BIT_BRKOFF_MIN) || (brkmon_normalized > BIT_BRKOFF_MAX))  // VLJ
                    if ((brkmon_normalized < tPdi.u16BrakeSwitchMonitorBrkOffMin) || (brkmon_normalized > tPdi.u16BrakeSwitchMonitorBrkOffMax))            // PDI
                    {
                        /* PATH(vbrakeMin_trp,D); */
                        status = true;

                        if (G_bChannelA == true)
                        {
                            /* PATH(vbrakeMin_trp,E); */
                            stat->data = 0x05;
                        }
                        else
                        {
                            /* PATH(vbrakeMin_trp,F); */
                            stat->data = 0x06;
                        }
                    }
                    break;
                case 2:
                    /* PATH(vbrakeMin_trp,G); */

                    /* Test for the upper brake signal on only */
//                    if ((brkmon_normalized < BIT_BRKUPPER_MIN) || (brkmon_normalized > BIT_BRKUPPER_MAX))  // VLJ
                    if ((brkmon_normalized < tPdi.u16BrakeSwitchMonitorBrkUpperMin) || (brkmon_normalized > tPdi.u16BrakeSwitchMonitorBrkUpperMax))            // PDI
                    {
                        /* PATH(vbrakeMin_trp,H); */
                        status = true;

                        if (G_bChannelA == true)
                        {
                            /* PATH(vbrakeMin_trp,I); */
                            stat->data = 0x01;
                        }
                        else
                        {
                            /* PATH(vbrakeMin_trp,J); */
                            stat->data = 0x03;
                        }
                    }
                    break;
                case 3:
                    /* PATH(vbrakeMin_trp,K); */

                    /* Test for the lower brake signal on only */
//                    if (brkmon_normalized > BIT_BRKLOWER_MAX)  // VLJ
                    if (brkmon_normalized > tPdi.u16BrakeSwitchMonitorBrkLowerMax)       // PDI
                    {
                        /* PATH(vbrakeMin_trp,L); */
                        status = true;

                        if (G_bChannelA == true)
                        {
                            /* PATH(vbrakeMin_trp,M); */
                            stat->data = 0x02;
                        }
                        else
                        {
                            /* PATH(vbrakeMin_trp,N); */
                            stat->data = 0x04;
                        }
                    }
                    break;
                default:
                    /* PATH(vbrakeMin_trp,O); */
                    break;
            }
        }
    }
    /* PATH(vbrakeMin_trp,P); */
    return (status);
}


/*
    brief Checks Trip Event for the Phase to Neutral Asymmetric Short , 0x2A.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects a Phase (Single) to Neutral short by monitoring the Currents

    GlobalsReferenced
    #define I_PHA_MOV_AVG    (Adc_MovAveraged.val.u16_I_PHA)
    #define I_PHB_MOV_AVG    (Adc_MovAveraged.val.u16_I_PHB)
    #define I_PHC_MOV_AVG    (Adc_MovAveraged.val.u16_I_PHC)
    #define MODE_CMD_ARINC825  (tPanel.bVcModeCmd)
    bool PrimarySide


    retval  true: Phase to Neutral Asymmetric short detected
            false: No Phase to Neutral Asymmetric short fault present.

*/

static bool PhaseNeutralShortAsym_trp( Bit_t *stat)
{
    bool status = false;
    uint16_t u16Avg_curr = 0;
    uint16_t u16Pha_imbal = 0;
    uint16_t u16Phb_imbal = 0;
    uint16_t u16Phc_imbal = 0;
    uint16_t u16Max_curr = 0;
    uint16_t u16Max_imbal = 0;


    bool_t bVcModeCmd = MCU_BusGetVcModeCmd();
    
    if(tPdi.bPhNShortAsymMonitorEnable)
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /*Run Monitor Only in Run State*/
            if (McuGetState() == RUN_MODE)
            {
                u16Avg_curr = (uint16_t)((I_PHA_MOV_AVG + I_PHB_MOV_AVG + I_PHC_MOV_AVG)/3);   /*Calculate the Average current*/

                if (I_PHA_MOV_AVG >= u16Avg_curr)                                /*Calculate current imbalance for phA*/
                {
                    u16Pha_imbal = (uint16_t)(I_PHA_MOV_AVG - u16Avg_curr);
                }
                else
                {
                    u16Pha_imbal = (uint16_t)(u16Avg_curr - I_PHA_MOV_AVG);
                }

                if (I_PHB_MOV_AVG >= u16Avg_curr)                               /*Calculate current imbalance for phA*/
                {
                    u16Phb_imbal = (uint16_t)(I_PHB_MOV_AVG - u16Avg_curr);
                }
                else
                {
                    u16Phb_imbal = (uint16_t)(u16Avg_curr - I_PHB_MOV_AVG);
                }

                if (I_PHC_MOV_AVG >= u16Avg_curr)                              /*Calculate current imbalance for phC*/
                {
                    u16Phc_imbal = (uint16_t)(I_PHC_MOV_AVG - u16Avg_curr);
                }
                else
                {
                    u16Phc_imbal = (uint16_t)(u16Avg_curr - I_PHC_MOV_AVG);
                }


                /*Save the Highest current*/
                if (I_PHA_MOV_AVG > I_PHB_MOV_AVG)
                {
                    if(I_PHA_MOV_AVG > I_PHC_MOV_AVG)
                    {
                        u16Max_curr = I_PHA_MOV_AVG;
                    }
                    else
                    {
                        u16Max_curr = I_PHC_MOV_AVG;
                    }
                }
                else
                {
                    if (I_PHB_MOV_AVG > I_PHC_MOV_AVG)
                    {
                        u16Max_curr = I_PHB_MOV_AVG;
                    }
                    else
                    {
                        u16Max_curr = I_PHC_MOV_AVG;
                    }
                }


                /*Save the Highest current Imbalance*/
                if (u16Pha_imbal > u16Phb_imbal)
                {
                    if(u16Pha_imbal > u16Phc_imbal)
                    {
                        u16Max_imbal = u16Pha_imbal;
                    }
                    else
                    {
                        u16Max_imbal = u16Phc_imbal;
                    }
                }
                else
                {
                    if (u16Phb_imbal > u16Phc_imbal)
                    {
                        u16Max_imbal = u16Phb_imbal;
                    }
                    else
                    {
                        u16Max_imbal = u16Phc_imbal;
                    }
                }


                /*Calculate the Current imbalance index*/
                /*while calculating the current imbalance the ADC offset from numerator is already removed
                 * Hence removing ADC offset only from the Denominator*/
                G_f32imbal_index = ((float32_t)(u16Max_imbal)/((float32_t)(u16Avg_curr - ADC_CONSTANT_COUNT)));

                /*Check if the current imbalance index is greater than 0.2 & if the max_current is greater than 1.5A (HighLift mode) or 1A in (VC mode) */
                /*tPdi.u16PhNShortAsymHLUL = 1.5A, (2318-2047)* 0.00554228 ~= 1.5A
                 * tPdi.u16PhNShortAsymVCUL = 1A, (2227-2047)* 0.00554228 ~= 1A
                 * tPdi.f32UnbalCurIndexUL = 0.2
                 * */
                if (((G_f32imbal_index > tPdi.f32UnbalCurIndexVCUL)  && 
                    ((bVcModeCmd == BUS_VC_MODE) &&
                    (u16Max_curr > tPdi.u16PhNShortAsymVCUL)))||
                    ((G_f32imbal_index > tPdi.f32UnbalCurIndexHLUL)  &&
                    ((bVcModeCmd == BUS_HL_MODE) &&
                    (u16Max_curr > tPdi.u16PhNShortAsymHLUL))))
                {
                    status = true;

                    if (I_PHA_MOV_AVG > tPdi.u16PhNShortAsymHLUL)
                    {
                        stat->data = 0x01;
                    }
                    else if (I_PHB_MOV_AVG > tPdi.u16PhNShortAsymHLUL)
                    {
                        stat->data = 0x02;
                    }
                    else if (I_PHC_MOV_AVG > tPdi.u16PhNShortAsymHLUL)
                    {
                        stat->data = 0x03;
                    }

                }

                if ((status == true) && (G_bChannelA == false))
                {
                    /* Set the upper nibble if the fault has occured on the right side. */
                    stat->data |= 0x10;
                }
            }
         }
    }
    return (status);
}


/*
    brief Checks Trip Event for the One Phase Open Circuit fault , 0x2B.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects a One Phase Open Circuit fault by monitoring the currents

    GlobalsReferenced
    #define I_PHA_MOV_AVG    (Adc_MovAveraged.val.u16_I_PHA)
    #define I_PHB_MOV_AVG    (Adc_MovAveraged.val.u16_I_PHB)
    #define I_PHC_MOV_AVG    (Adc_MovAveraged.val.u16_I_PHC)
    #define MODE_CMD_ARINC825  (tPanel.bVcModeCmd)
    bool PrimarySide


    retval  true: One Phase Open circuit fault is detected
            false: No One Phase Open circuit fault present.

*/
static bool OnePhaseOpenCkt_trp( Bit_t *stat)
{
    bool status = false;
    if(tPdi.bOnePhOpenCktMonitorEnable)
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /*Perform this test only when the Motor is Running*/
            if (McuGetState() == RUN_MODE)
            {
                /*Check if any current is below the open circuit threshold currently set to 32mA*/
                /*tPdi.u16OnePhOpenCktLL = 33mA, (2053-2047)* 0.00554228 ~= 33mA*/
                if ((I_PHA_MOV_AVG  < tPdi.u16OnePhOpenCktLL) ||
                        (I_PHB_MOV_AVG < tPdi.u16OnePhOpenCktLL) ||
                        (I_PHC_MOV_AVG < tPdi.u16OnePhOpenCktLL))
                {
                    status = true;
                    if (I_PHA_MOV_AVG < tPdi.u16OnePhOpenCktLL)
                    {
                        stat->data = 0x01;
                    }
                    else if (I_PHB_MOV_AVG < tPdi.u16OnePhOpenCktLL)
                    {
                        stat->data = 0x02;
                    }
                    else if (I_PHC_MOV_AVG < tPdi.u16OnePhOpenCktLL)
                    {
                        stat->data = 0x03;
                    }
                }

                if ((status == true) && (G_bChannelA == false))
                {
                    /* PATH(bridgeMon_trp,Q); */

                    /* Set the upper nibble if the fault has occured on the right side. */
                    stat->data |= 0x10;
                }
            }


        }

    }
    return (status);
}

#if defined(__SKEW_SNSR_RVDT__)
/*---------- RVDT health check functions ----------*/

/**
    brief Checks Trip Event for the Position Sensor fault monitor, 0x30.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects a fault in the RVDT circuitry by monitoring the RVDT_SUM signal to ensure
    that it falls within design limits.

*/
static bool rvdtExc_trp( Bit_t *stat )
{
    bool status = false;

    if(tPdi.bFpsuExcMonitorEnable)
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* shall return true if the #RVDT_SUM reading is less than #BIT_RVDTSUM_IDEAL - #BIT_RVDT_TOLERANCE.
               shall return true if the #RVDT_SUM reading is greater than #BIT_RVDTSUM_IDEAL + #BIT_RVDT_TOLERANCE. */
            //if ((P_SUM < (BIT_RVDTSUM_IDEAL - BIT_RVDT_TOLERANCE)) ||
            //    (P_SUM > (BIT_RVDTSUM_IDEAL + BIT_RVDT_TOLERANCE)))         // VLJ
            if ((Adc_Averaged.val.u16_RVDT_SUM < (tPdi.u16FpsuExcMonitorSumIdeal - tPdi.u16FpsuExcMonitorSumTolerance)) ||
                (Adc_Averaged.val.u16_RVDT_SUM > (tPdi.u16FpsuExcMonitorSumIdeal + tPdi.u16FpsuExcMonitorSumTolerance)))
            {
                status = true;

                /* shall report 'fault data' as 0x01 for Channel A, and 0x02 for Channel B*/
                if (G_bChannelA == true)
                {
                    stat->data = 0x01;
                }
                else// (G_bChannelA == false)
                {
                    stat->data = 0x02;
                }
            }
        }
    }

    return (status);
}

/**
    brief Checks Trip Event for the Position Sensor fault monitor, 0x34.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects a fault in the RVDT circuitry by monitoring the analog signal RVDT_POS
    for a mismatch between the two channels.

*/
static bool rvdtPos_trp( Bit_t *stat )
{
    bool status = false;
    int32 delta = 0;

    if(tPdi.bFpsuPosMonitorEnable == true)
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* No direct signal of Cross Side RVDT involved in MCU by design.
             * keeping below implementation considering Cross Side RVDT data
             * will be received over ICC bus.
             * AB:  Add logic back in once cross-side RVDT data is implemented in ICC.
             */
//            if (G_bChannelA == true)
//            {
//                /* PATH(rvdtLtPos_trp,C); */
//                delta = (int32)P_POS - (int32)S_POS_IN;
//            }
//            else if(G_bChannelA == false)
//            {
//                /* PATH(rvdtLtPos_trp,D); */
//                delta = (int32)S_POS - (int32)P_POS_IN;
//            }
            /* PATH(rvdtLtPos_trp,C); */
            delta = (int32)Adc_Averaged.val.u16_RVDT_POS - (int32)ICC_xData.tData.tIcc_5.rvdt_pos_in;
            /* shall return true only if the difference is greater than #BIT_RVDTPOS_MISMATCH. */
//            if ((delta > BIT_RVDTPOS_MISMATCH) || (delta < -BIT_RVDTPOS_MISMATCH))    // VLJ
            if ((delta > tPdi.u16FpsuPosMonitorDeltaThreshold) || (delta < -tPdi.u16FpsuPosMonitorDeltaThreshold))
            {
                status = true;
    
                /* shall report 'fault data' as 0x01 for Channel A, and 0x02 for Channel B*/
                if (G_bChannelA == true)
                {
                    stat->data = 0x01;
                }
                else// (G_bChannelA == false)
                {
                    stat->data = 0x02;
                }
            }
        }
    }

    /* PATH(rvdtLtPos_trp,H); */
    return (status);
}
#endif

#if defined(__SKEW_SNSR_ENCODER__)
/**
    brief Checks Trip Event for the Encoder Skew Sensor fault monitor, 0x36.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects a fault in the skew sensor circuitry by monitoring the CRC, Position
    Valid, and Ready signals from the IncOder and McBSP driver.

*/
static bool encoderHealth_trp( Bit_t *stat )
{
    bool status = false;

    if(tPdi.bEncoderHealthMonitorEnable == true)
    {
        if (stat != NULL)
        {
            /* Check if the CRC Error Counter has reached the trip threshold */
            if ((u16IncoderCrc8CheckFailedCntr >= tPdi.u16IncoderCrcChkThreshold)&&(tPdi.bIncoderCrcChkEnable == true))
            {
                status = true;

                /* shall report 'fault data' as 0x01 for Channel A, and 0x02 for Channel B*/
                if (G_bChannelA == true)
                {
                    stat->data = 0x01;
                }
                else// (G_bChannelA == false)
                {
                    stat->data = 0x02;
                }
            }
            /* Check if the Position Not Valid Error Counter has reached the trip threshold.*/
            else if((u16IncoderPositionNotValidCntr >= tPdi.u16IncoderPvInvalidChkThreshold)&&(tPdi.bIncoderPvInvalidChkEnable == true))
            {
                status = true;

                /* shall report 'fault data' as 0x04 for Channel A, and 0x08 for Channel B*/
                if (G_bChannelA == true)
                {
                    stat->data = 0x04;
                }
                else// (G_bChannelA == false)
                {
                    stat->data = 0x08;
                }
            }
            /* Check if the Receive Not Ready Error Counter has reached the trip threshold */
            else if((u16IncoderRxNotReadyCntr >= tPdi.u16IncoderRxReadyChkThreshold)&&(tPdi.bIncoderRxReadyChkEnable == true))
            {
                status = true;

                /* shall report 'fault data' as 0x10 for Channel A, and 0x20 for Channel B*/
                if (G_bChannelA == true)
                {
                    stat->data = 0x10;
                }
                else// (G_bChannelA == false)
                {
                    stat->data = 0x20;
                }
            }
        }
    }

    return status;
}
#endif

/*---------- SCU Communications Health Check Functions ----------*/
/* Commented out for now since there may still be a need for this type of monitor on the MCU
 * for both the Control and Maintenance busses */
///*
//    brief Checks Trip Event for the ARINC825 Control Bus Communication Monitor:  0x40.
//
//    Function uses the form of function type ::Bit_Check_t.
//
//    retval  true: ARINC825 Control bus has at least one fault
//            false: ARINC825 Control bus is healthy and SCU is communicating on it.
//*/
//static bool a825CntrlBusCommsMon_trp( Bit_t *stat )
//{
//    bool status = false;
//    bool_t bCommsFault = false;
//
//    /* shall ensure that 'stat' pointer is not null */
//    if (stat)
//    {
//        /* Logical OR the Faults Together */
//        bCommsFault = (  (LatchedWarnings_Stat.bit.bA825CntrlBusPassive) |
//                         (LatchedWarnings_Stat.bit.bA825CntrlBusOff) |
//                         (LatchedWarnings_Stat.bit.bA825CntrlBusCommsStale));
//
//        /* Check if there is a fault on the SCU ARINC825 Bus */
//        if(bCommsFault == true)
//        {
//            /* Both channels faulted. Trip the monitor */
//            status = true;
//
//            /* shall report 'fault data' as 0x01 for the Left channel, 0x02 for the Right channel */
//            if (G_bChannelA == true)
//            {
//                stat->data = 0x01;
//            }
//            else if (G_bChannelA == false)
//            {
//                stat->data = 0x02;
//            }
//        }
//    }
//
//    /* PATH(hwAsym_trp,G); */
//    return (status);
//}

///*
//    brief Checks Trip Event for the ARINC825 Maintenance Bus Communication Monitor:  0x41.
//
//    Function uses the form of function type ::Bit_Check_t.
//
//    retval  true: ARINC825 Maintenance bus has at least one fault
//            false: ARINC825 Maintenance bus is healthy and GSE is communicating on it.
//*/
//static bool a825MaintBusCommsMon_trp( Bit_t *stat )
//{
//    bool status = false;
//    bool_t bCommsFault = false;
//
//    /* shall ensure that 'stat' pointer is not null */
//    if (stat)
//    {
//        /* Logical OR the Faults Together */
//        bCommsFault = (  (LatchedWarnings_Stat.bit.bA825MaintBusPassive) |
//                         (LatchedWarnings_Stat.bit.bA825MaintBusOff) |
//                         (LatchedWarnings_Stat.bit.bA825MaintBusCommsStale));
//
//        /* Check if there is a fault on the SCU ARINC825 Bus */
//        if(bCommsFault == true)
//        {
//            /* Both channels faulted. Trip the monitor */
//            status = true;
//
//            /* shall report 'fault data' as 0x01 for the Left channel, 0x02 for the Right channel */
//            if (G_bChannelA == true)
//            {
//                stat->data = 0x01;
//            }
//            else if (G_bChannelA == false)
//            {
//                stat->data = 0x02;
//            }
//        }
//    }
//
//    /* PATH(hwAsym_trp,G); */
//    return (status);
//}

///*
//    brief Checks Trip Event for the SCU Flap Command Skew Monitor for the
//    ARINC825 Control Bus:  0x48.
//
//    Function uses the form of function type ::Bit_Check_t.
//
//    retval  true: Flap Command is out of tolerance between channels.
//            false: Flap command is within tolerance between channels.
//*/
//static bool a825CntrlBusFlapCmdSkewMon_trp( Bit_t *stat )
//{
//    bool status = false;
//    float32_t f32DiffInPosCmd = 0.0F;
//
//    if (tPdi.PDI_Monitor_Flap_Pos_Cmd_Match_Enable)
//    {
//        /* shall ensure that 'stat' pointer is not null */
//        if (stat)
//        {
//            /* Calculate difference in position commands between Left and Right Actuators */
//            f32DiffInPosCmd = tScuInterface.f32PositionCmdL - tScuInterface.f32PositionCmdR;
//
//            /* Check if position command between actuators is outside allowable tolerance */
//            if((f32DiffInPosCmd < -MAX_POS_CMD_DIFF_ALLOWED) || (f32DiffInPosCmd > MAX_POS_CMD_DIFF_ALLOWED))
//            {
//                status = true;
//
//                /* shall report 'fault data' as 0x01 for the Left channel, 0x02 for the Right channel */
//                if (G_bChannelA == true)
//                {
//                    stat->data = 0x01;
//                }
//                else if (G_bChannelA == false)
//                {
//                    stat->data = 0x02;
//                }
//            }
//        }
//    }
//
//    /* PATH(hwAsym_trp,G); */
//    return (status);
//
//}

/*
    brief Checks Trip Event for the GSE Flap Command Skew Monitor for the
    ARINC825 Maintenance Bus:  0x49.

    Function uses the form of function type ::Bit_Check_t.

    retval  true: Flap Command is out of tolerance between channels.
            false: Flap command is within tolerance between channels.
*/
static bool a825MaintBusFlapCmdSkewMon_trp( Bit_t *stat )
{
    bool status = false;
    float32_t f32DiffInPosCmd = 0.0F;

    if (tPdi.bA825MaintBusFlapCmdSkewMonitorEnable)
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* Calculate difference in position commands between Left and Right Actuators */
            // AB: need to generate the GSE Rigging Command interface but should have L/R commands to check.
            f32DiffInPosCmd = tGseRigInterface.f32PositionCmdL - tGseRigInterface.f32PositionCmdR;

            /* Check if position command between actuators is outside allowable tolerance */
            if((f32DiffInPosCmd < -tPdi.f32A825MaintBusFlapSkewMaxDiffAllowed) || (f32DiffInPosCmd > tPdi.f32A825MaintBusFlapSkewMaxDiffAllowed))
            {
                status = true;

                /* shall report 'fault data' as 0x01 for the Left channel, 0x02 for the Right channel */
                if (G_bChannelA == true)
                {
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    stat->data = 0x02;
                }
            }
        }
    }

    /* PATH(hwAsym_trp,G); */
    return (status);

}

/*
    brief Checks Trip Event for the Communications Stale Monitor for the
    ARINC825 Control Bus:  0x4E.  Function uses the form of function type ::Bit_Check_t.


    retval  true: Communications were established on the bus and then have gone stale.
            false: Communications are fresh or have not been established yet.

*/
static bool a825CntrlCommsStaleMon_trp( Bit_t *stat )
{
    bool status = false;

    if (tPdi.bA825CntrlBusCommsStaleMonitorEnable)
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            if((tA825_Bus[ARINC825_CNTRL_BUS].bCommsEstablished == true) && (tA825_Bus[ARINC825_CNTRL_BUS].tRx.ptMsgList[A825_CNTRL_BUS_RX_FLAP_CMD].bMsgStaleStatus == true))
            {
                status = true;

                /* shall report 'fault data' as 0x01 for the Left channel, 0x02 for the Right channel */
                if (G_bChannelA == true)
                {
                    /* PATH(hwAsym_trp,E); */
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(hwAsym_trp,F); */
                    stat->data = 0x02;
                }
            }
        }
    }

    /* PATH(hwAsym_trp,G); */
    return (status);
}

/*
    brief Checks Trip Event for the Communications Stale Monitor for the
    ARINC825 Maintenance Bus:  0x4F.  Function uses the form of function type ::Bit_Check_t.


    retval  true: Communications were established on the bus and then have gone stale.
            false: Communications are fresh or have not been established yet.

*/
static bool a825MaintCommsStaleMon_trp( Bit_t *stat )
{
    bool status = false;

    if (tPdi.bA825MaintBusCommsStaleMonitorEnable)
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            if((tA825_Bus[ARINC825_MAINT_BUS].bCommsEstablished == true) && (tA825_Bus[ARINC825_MAINT_BUS].tRx.ptMsgList[A825_MAINT_BUS_RX_RIG_CMD].bMsgStaleStatus == true))
            {
                status = true;

                /* shall report 'fault data' as 0x01 for the Left channel, 0x02 for the Right channel */
                if (G_bChannelA == true)
                {
                    /* PATH(hwAsym_trp,E); */
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(hwAsym_trp,F); */
                    stat->data = 0x02;
                }
            }
        }
    }

    /* PATH(hwAsym_trp,G); */
    return (status);
}

/*
    brief Checks Trip Event for the SYNC Communications A825 : Bus Passive Monitor 0x4A.  Function uses
        the form of function type ::Bit_Check_t.


    retval  true: A825 bus in Passive Mode.
            false: A825 bus not in Passive Mode.

*/
static bool a825CntrlBusPassiveMon_trp( Bit_t *stat )
{
    bool status = false;

    if (tPdi.bA825CntrlBusPassiveMonitorEnable)
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            if((tA825_Bus[ARINC825_CNTRL_BUS].ptDrvrData->ptCanRegs->CAN_ERRC.bit.TEC > 127) ||
               (tA825_Bus[ARINC825_CNTRL_BUS].ptDrvrData->ptCanRegs->CAN_ERRC.bit.RP == 1))
            {
                status = true;

                /* shall report 'fault data' as 0x01 for the Left channel, 0x02 for the Right channel */
                if (G_bChannelA == true)
                {
                    /* PATH(hwAsym_trp,E); */
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(hwAsym_trp,F); */
                    stat->data = 0x02;
                }
            }
        }
    }

    /* PATH(hwAsym_trp,G); */
    return (status);
}

/*
    brief Checks Trip Event for the Rigging Communications A825 : Bus Passive Monitor 0x4B.  Function uses
        the form of function type ::Bit_Check_t.


    retval  true: A825 bus in Passive Mode.
            false: A825 bus not in Passive Mode.

*/
static bool a825MaintBusPassiveMon_trp( Bit_t *stat )
{
    bool status = false;

    if (tPdi.bA825MaintBusPassiveMonitorEnable)
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            if((tA825_Bus[ARINC825_MAINT_BUS].ptDrvrData->ptCanRegs->CAN_ERRC.bit.TEC > 127) ||
                (tA825_Bus[ARINC825_MAINT_BUS].ptDrvrData->ptCanRegs->CAN_ERRC.bit.RP == 1))
            {
                status = true;

                /* shall report 'fault data' as 0x01 for the Left channel, 0x02 for the Right channel */
                if (G_bChannelA == true)
                {
                    /* PATH(hwAsym_trp,E); */
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(hwAsym_trp,F); */
                    stat->data = 0x02;
                }
            }
        }
    }

    /* PATH(hwAsym_trp,G); */
    return (status);
}

/*
    brief Checks Trip Event for the SYNC Communications A825 : Bus OFF Monitor 0x4C.  Function uses
        the form of function type ::Bit_Check_t.


    retval  true: A825 bus in Bus OFF Mode.
            false: A825 bus not in Bus OFF Mode.

*/
static bool a825CntrlBusOffMon_trp( Bit_t *stat )
{
    bool status = false;

    if (tPdi.bA825CntrlBusOffMonitorEnable)
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            if(tA825_Bus[ARINC825_CNTRL_BUS].ptDrvrData->ptCanRegs->CAN_ES.bit.BOff)
            {
                status = true;

                /* shall report 'fault data' as 0x01 for the Left channel, 0x02 for the Right channel */
                if (G_bChannelA == true)
                {
                    /* PATH(hwAsym_trp,E); */
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(hwAsym_trp,F); */
                    stat->data = 0x02;
                }
            }
        }
    }

    /* PATH(hwAsym_trp,G); */
    return (status);
}

/*
    brief Checks Trip Event for the Rigging Communications A825 : Bus OFF Monitor 0x4D.  Function uses
        the form of function type ::Bit_Check_t.


    retval  true: A825 bus in Bus OFF Mode.
            false: A825 bus not in Bus OFF Mode.

*/
static bool a825MaintBusOffMon_trp( Bit_t *stat )
{
    bool status = false;

    if (tPdi.bA825MaintBusOffMonitorEnable)
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            if(tA825_Bus[ARINC825_MAINT_BUS].ptDrvrData->ptCanRegs->CAN_ES.bit.BOff)
            {
                status = true;

                /* shall report 'fault data' as 0x01 for the Left channel, 0x02 for the Right channel */
                if (G_bChannelA == true)
                {
                    /* PATH(hwAsym_trp,E); */
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(hwAsym_trp,F); */
                    stat->data = 0x02;
                }
            }
        }
    }

    /* PATH(hwAsym_trp,G); */
    return (status);
}


/*---------- Control channel health check functions ----------*/

/*
    brief Checks Trip Event for the Motor Bridge fault monitor, 0x50.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects a fault condition on the motor bridge circuitry by monitoring the
    "phase voltage signal" while the bridge circuit is commanded
    for test purposes.

	GlobalsReferenced
	#define MTR_MON (Adc_Averaged.ch7)
	bool PrimarySide


	retval	true: Voltage is not within the expected range.
			false: Voltage is within the expected voltage range.

*/
static bool bridgeMon_trp( Bit_t *stat )
{
    bool status = false;
	Uint32 mtrmon_normalized = 0;

    /* PATH(bridgeMon_trp,A); */
    if(tPdi.bBridgeMonitorEnable)
    {
        /* shall compare analog signal MTR_MON with valid range, per test condition stored in global variable.
           shall return true only if MTR_MON falls outside the valid range under test */
        if (stat)
        {
            /* PATH(bridgeMon_trp,B); */
            /* Compute normalized value for MTR_MON, depending on VBUS_MON.
               1900 represents 1.3916 V on VBUS_MON, the nominal case for monitor tests.  */
#if 0 // AB: Need to revisit when monitors are defined
            if (VBUS_MON != 0)
            {
                /* PATH(bridgeMon_trp,T); */
                mtrmon_normalized = (1900 * MTR_MON) / VBUS_MON;
            }
#endif
            mtrmon_normalized = 1000;

            /* PATH(bridgeMon_trp,U); */

            switch (BridgeMonStage)
            {
                case 1:
                    /* PATH(bridgeMon_trp,C); */

                    /* Test for all uppers and lowers off */
//                    if ((mtrmon_normalized < BIT_BRIDGENONE_MIN) || (mtrmon_normalized > BIT_BRIDGENONE_MAX))  // VLJ
                    if ((mtrmon_normalized < tPdi.u16BridgeMonitorBridgeNoneMin) || (mtrmon_normalized > tPdi.u16BridgeMonitorBridgeNoneMax))       // PDI
                    {
                        /* PATH(bridgeMon_trp,S); */
                        stat->data = 0x07;
                        status = true;
                    }
                    break;
                case 2:
                    /* PATH(bridgeMon_trp,D); */

                    /* Test for Upper A */
//                    if (mtrmon_normalized < BIT_BRIDGEUPPER_MIN)  // VLJ
                    if (mtrmon_normalized < tPdi.u16BridgeMonitorBridgeUpperMin)
                    {
                        /* PATH(bridgeMon_trp,E); */
                        stat->data = 0x01;
                        status = true;
                    }
                    break;
                case 4:
                    /* PATH(bridgeMon_trp,F);*/

                    /* Test for Lower A */
//                    if (mtrmon_normalized > BIT_BRIDGELOWER_MAX) // VLJ
                    if (mtrmon_normalized > tPdi.u16BridgeMonitorBridgeLowerMax)
                    {
                        /* PATH(bridgeMon_trp,G); */
                        stat->data = 0x02;
                        status = true;
                    }
                    break;
                case 5:
                    /* PATH(bridgeMon_trp,H); */

                    /* Test for Upper B */
//                    if (mtrmon_normalized < BIT_BRIDGEUPPER_MIN) // VLJ
                    if (mtrmon_normalized < tPdi.u16BridgeMonitorBridgeUpperMin)
                    {
                        /* PATH(bridgeMon_trp,I); */
                        stat->data = 0x03;
                        status = true;
                    }
                    break;
                case 7:
                    /* PATH(bridgeMon_trp,J); */

                    /* Test for Lower B */
//                    if (mtrmon_normalized > BIT_BRIDGELOWER_MAX) // VLJ
                    if (mtrmon_normalized > tPdi.u16BridgeMonitorBridgeLowerMax)
                    {
                        /* PATH(bridgeMon_trp,K); */
                        stat->data = 0x04;
                        status = true;
                    }
                    break;
                case 8:
                    /* PATH(bridgeMon_trp,L); */

                    /* Test for Upper C */
//                    if (mtrmon_normalized < BIT_BRIDGEUPPER_MIN) // VLJ
                    if (mtrmon_normalized < tPdi.u16BridgeMonitorBridgeUpperMin)
                    {
                        /* PATH(bridgeMon_trp,M); */
                        stat->data = 0x05;
                        status = true;
                    }
                    break;
                case 10:
                    /* PATH(bridgeMon_trp,N); */

                    /* Test for Lower C */
//                    if (mtrmon_normalized > BIT_BRIDGELOWER_MAX) // VLJ
                    if (mtrmon_normalized > tPdi.u16BridgeMonitorBridgeLowerMax)
                    {
                        /* PATH(bridgeMon_trp,O); */
                        stat->data = 0x06;
                        status = true;
                    }
                    break;
                default:
                    /* PATH(bridgeMon_trp,P); */
                    break;
            }

            if ((status == true) && (G_bChannelA == false))
            {
                /* PATH(bridgeMon_trp,Q); */

                /* Set the upper nibble if the fault has occured on the right side. */
                stat->data |= 0x10;
            }
        }
    }
    /* PATH(bridgeMon_trp,R); */
    /* shall report 'fault data' as the test condition that failed */
    return (status);
}



/**
    brief Checks Trip Event for the DC_BUS_CHA Monitor, 0x51.  Function uses
        the form of function type :Bit_Check_t.

    This function detects a fault condition by Monitoring DC_BUS_CHA bus.
    Monitor will trip if the DC_BUS_CHA is outside the specified limit.

    GlobalsReferenced
    Adc_Raw.val.u16_DC_BUS_VSENSE
    bool G_bChannelA
    #define MODE_CMD_ARINC825  (tPanel.bVcModeCmd)
    DC_BUS_STATE CurrentBusState
    #define BUS_HL_MODE (0U)
    #define BUS_VC_MODE (1U)

    retval  true: Voltage is not within the expected range.
            false: Voltage is within the expected voltage range.
 */
static bool DcBusCha_trp(Bit_t*stat)
{
    bool status = false;
    bool_t bVcModeCmd = MCU_BusGetVcModeCmd();
    bool_t bRunMonitor = true;

    if(tPdi.bDCBusMonitorEnable)
    {
        /*Shall ensure that 'stat' pointer is not null*/
        if (stat)
        {
            /* Do not run this monitor during the transition period of discharging the bus due to
             * fluctuations of the DC_BUS_SENSE voltage that otherwise will trip this monitor.
             * This is a temporary fix until HW is updated to remove the issue with the +3.3V_HS
             * power supply from being dragged down during discharging. */
            if(Timer_IsSet(&tDcBusMonitorDelayTimer) == true)
            {
                /* Bus discharge event has started. Check if the delay is complete */
                if(Timer_IsExpired(&tDcBusMonitorDelayTimer) == true)
                {
                    /* Monitor delay has expired. Reset the Timer */
                    Timer_ResetTimer(&tDcBusMonitorDelayTimer);
                }
                else
                {
                    /* Monitor delay has not completed. Do not run monitor */
                    bRunMonitor = false;
                    status = false;
                }
            }

            if(bRunMonitor == true)
            {
                /*Check if signal is within limits during VC mode*/
                if (bVcModeCmd == BUS_VC_MODE)
                {
                    /*Check if signal is below the specified lower limit*/
                    if (Adc_Raw.val.u16_DC_BUS_VSENSE < tPdi.u16DCBusMonitorVCLL)               /*Update limit with PDI item*/
                    {
                        status = true;
                        if (CurrentBusState == BUS_FAILURE_STATE)
                        {
                            stat->data = 0x01;
                        }
                        else
                        {
                            stat->data = 0x05;
                        }
                    }
                    /*Check if signal is above the specified upper limit*/
                    else if (Adc_Raw.val.u16_DC_BUS_VSENSE > tPdi.u16DCBusMonitorVCUL)          /*Update limit with PDI item*/
                    {
                        status = true;
                        if (CurrentBusState == BUS_FAILURE_STATE)
                        {
                            stat->data = 0x02;
                        }
                        else
                        {
                            stat->data = 0x06;
                        }

                    }
                }
                /*Check if signal is within limits during Highlift  mode*/
                else
                {
                    /*Check if signal is below the specified lower limit*/
                    if (Adc_Raw.val.u16_DC_BUS_VSENSE < tPdi.u16DCBusMonitorHLLL)               /*Update limit with PDI item*/
                    {
                        status = true;
                        if (CurrentBusState == BUS_FAILURE_STATE)
                        {
                            stat->data = 0x03;
                        }
                        else
                        {
                            stat->data = 0x07;
                        }
                    }
                    /*Check if signal is above the specified upper limit*/
                    else if (Adc_Raw.val.u16_DC_BUS_VSENSE > tPdi.u16DCBusMonitorHLUL)          /*Update limit with PDI item*/
                    {
                        status = true;
                        if (CurrentBusState == BUS_FAILURE_STATE)
                        {
                            stat->data = 0x04;
                        }
                        else
                        {
                            stat->data = 0x08;
                        }
                    }

                }

                if ((status == true) && (G_bChannelA == false))
                {

                    /* Set the upper nibble if the fault has occured on Channel B. */
                    stat->data |= 0x10;
                }
            }
        }
    }
    /* shall return true if signal voltage is outside the specified limit */
    return(status);
}


/**
    brief Checks Trip Event for the +28 VDC monitor, 0x52.  Function uses
        the form of function type :Bit_Check_t.

    This function detects a fault condition by Monitoring +28VDC bus.
    Monitor will trip if the +28VDC bus is outside the specified limit.

    GlobalsReferenced
    Adc_Raw.val.u16_28VBUS_VSENSE
    bool G_bChannelA


    retval  true: Voltage is not within the expected range.
            false: Voltage is within the expected voltage range.
 */
static bool p28vdc_trp(Bit_t*stat)
{
    bool status = false;
    bool_t bRunMonitor = true;

    if(tPdi.bp28VDCMonitorEnable)
    {
        /*Shall ensure that 'stat' pointer is not null*/
        if (stat)
        {
            /* Do not run this monitor during the transition period of discharging the bus due to
             * fluctuations of the 28V_SENSE voltage that otherwise causes this monitor to trip.
             * This is a temporary fix until HW is updated to remove the issue with the +3.3V_HS
             * power supply from being dragged down during discharging. */
            if(Timer_IsSet(&t28VdcMonitorDelayTimer) == true)
            {
                /* Bus discharge event has started. Check if the delay is complete */
                if(Timer_IsExpired(&t28VdcMonitorDelayTimer) == true)
                {
                    /* Monitor delay has expired. Reset the Timer */
                    Timer_ResetTimer(&t28VdcMonitorDelayTimer);

                    /* Re-Enable the GFD_I_SENSE Interrupt since the discharge is complete. */
                    XintRegs.XINT1CR.bit.ENABLE = INTERRUPT_ENABLE;
                }
                else
                {
                    /* Monitor delay has not completed. Do not run monitor */
                    bRunMonitor = false;
                    status = false;
                }
            }

            if(bRunMonitor == true)
            {
                /*Check if signal is below the specified lower limit*/
                if (Adc_Raw.val.u16_28VBUS_VSENSE < tPdi.u16p28VDCMonitorLL)               /*Update limit with PDI item*/
                {
                    status = true;
                    if (G_bChannelA == true)
                    {
                        stat->data = 0x01;
                    }
                    else if (G_bChannelA == false)
                    {
                        stat->data = 0x03;
                    }
                }
                /*Check if signal is above the specified upper limit*/
                else if (Adc_Raw.val.u16_28VBUS_VSENSE > tPdi.u16p28VDCMonitorUL)          /*Update limit with PDI item*/
                {
                    status = true;
                    if (G_bChannelA == true)
                    {
                        stat->data = 0x02;
                    }
                    else if (G_bChannelA == false)
                    {
                        stat->data = 0x04;
                    }
                }
            }
        }
    }
    /* shall return true if signal voltage is outside the specified limit */
    return(status);
}


/**
    brief Checks Trip Event for the +15V_SENSE monitor, 0x53.  Function uses
        the form of function type :Bit_Check_t.

    This function detects a fault condition by Monitoring +15V_SENSE (+15VISO_CHA rail).
    Monitor will trip if the 15V_SENSE signal is outside the specified limit.

    GlobalsReferenced
    Adc_Raw.val.u16_POSITIVE_15V_SENSE
    bool G_bChannelA


    retval  true: Voltage is not within the expected range.
            false: Voltage is within the expected voltage range.
 */
static bool p15vdc_trp(Bit_t*stat)
{
    bool status = false;
    if(tPdi.bp15VDCMonitorEnable)
    {
        /*Shall ensure that 'stat' pointer is not null*/
        if (stat)
        {
            /*Check if signal is below the specified lower limit*/
            if (Adc_Raw.val.u16_POSITIVE_15V_SENSE < tPdi.u16p15VDCMonitorLL)               /*Update limit with PDI item*/
            {
                status = true;
                if (G_bChannelA == true)
                {
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    stat->data = 0x03;
                }
            }
            /*Check if signal is above the specified upper limit*/
            else if (Adc_Raw.val.u16_POSITIVE_15V_SENSE > tPdi.u16p15VDCMonitorUL)          /*Update limit with PDI item*/
            {
                status = true;
                if (G_bChannelA == true)
                {
                    stat->data = 0x02;
                }
                else if (G_bChannelA == false)
                {
                    stat->data = 0x04;
                }
            }
        }
    }
    /* shall return true if signal voltage is outside the specified limit */
    return(status);
}


/**
    brief Checks Trip Event for the -15V_SENSE monitor, 0x54.  Function uses
        the form of function type :Bit_Check_t.

    This function detects a fault condition by Monitoring -15V_SENSE (-15VISO_CHA rail).
    Monitor will trip if the -15V_SENSE signal is outside the specified limit.

    GlobalsReferenced
    Adc_Raw.val.u16_NEGATIVE_15V_SENSE
    bool G_bChannelA


    retval  true: Voltage is not within the expected range.
            false: Voltage is within the expected voltage range.
 */
static bool n15vdc_trp(Bit_t*stat)
{
    bool status = false;
    if(tPdi.bn15VDCMonitorEnable)
    {
        /*Shall ensure that 'stat' pointer is not null*/
        if (stat)
        {
            /*Check if signal is below the specified lower limit*/
            if (Adc_Raw.val.u16_NEGATIVE_15V_SENSE < tPdi.u16n15VDCMonitorLL)               /*Update limit with PDI item*/
            {
                status = true;
                if (G_bChannelA == true)
                {
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    stat->data = 0x03;
                }
            }
            /*Check if signal is above the specified upper limit*/
            else if (Adc_Raw.val.u16_NEGATIVE_15V_SENSE > tPdi.u16n15VDCMonitorUL)          /*Update limit with PDI item*/
            {
                status = true;
                if (G_bChannelA == true)
                {
                    stat->data = 0x02;
                }
                else if (G_bChannelA == false)
                {
                    stat->data = 0x04;
                }
            }
        }
    }
    /* shall return true if signal voltage is outside the specified limit */
    return(status);
}


/**
    brief Checks Trip Event for the 5V_SENSE monitor, 0x55.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects a fault condition by Monitoring 5V_SENSE (+5V_AN_CHA rail).
    Monitor will trip if the 5V_SENSE is outside the specified limit.

    GlobalsReferenced
    Adc_Raw.val.u16_5V_SENSE
    bool G_bChannelA


    retval  true: Voltage is not within the expected range.
            false: Voltage is within the expected voltage range.
 */
static bool p5vdc_trp(Bit_t*stat)
{
    bool status = false;
    if(tPdi.bp5VDCMonitorEnable)
    {
        /*Shall ensure that 'stat' pointer is not null*/
        if (stat)
        {
            /*Check if signal is below the specified lower limit*/
            if (Adc_Raw.val.u16_5V_SENSE < tPdi.u16p5VDCMonitorLL)               /*Update limit with PDI item*/
            {
                status = true;
                if (G_bChannelA == true)
                {
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    stat->data = 0x03;
                }
            }
            /*Check if signal is above the specified upper limit*/
            else if (Adc_Raw.val.u16_5V_SENSE > tPdi.u16p5VDCMonitorUL)          /*Update limit with PDI item*/
            {
                status = true;
                if (G_bChannelA == true)
                {
                    stat->data = 0x02;
                }
                else if (G_bChannelA == false)
                {
                    stat->data = 0x04;
                }
            }
        }
    }
    /* shall return true if signal voltage is outside the specified limit */
    return(status);
}

/**
    brief Checks Trip Event for the Inverter Gate Driver monitor, 0x56.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects a fault condition on Inverter Gate Driver by monitoring the
    "PHx_HS_nFAULT, PHx_LS_nFAULT, PHx_HS_READY, PHx_LS_READY" signals for phase A,B & C

    GlobalsReferenced
    tMuxInputs MuxInputs


    retval  true: One or more Inverter Gate Drivers has failure.
            false: All the Inverter Gate Drivers are healthy.
 */

static bool InvGateDriver_trp (Bit_t *stat)
{
    bool status = false;
    if(tPdi.bGateDriverMonitorEnable)
    {
        /*Shall ensure that 'stat' pointer is not null*/
        if (stat)
        {
            /* Shall verify if any nFault signal is 0.
             * GD_FAULT = false indicates short circuit failure in inverter gate driver  */
            if (GD_FAULT_RAW == false)
            {
                status = true;
                if (G_bChannelA == true)
                {
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    stat->data = 0x03;
                }
            }
            /* Shall verify if any Ready signal is 0.
             * GD_READY = false indicates under voltage lockout or thermal shutdown of inverter gate driver */
            else if (GD_READY_RAW == false)
            {
                status = true;
                if (G_bChannelA == true)
                {
                    stat->data = 0x02;
                }
                else if (G_bChannelA == false)
                {
                    stat->data = 0x04;
                }
            }
        }
    }
    /* shall return true if there is a failure in any of the inverter gate drivers */
    return (status);
}


/**
    brief Checks Trip Event for the Channel Program Pins fault monitor, 0x57.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects a fault in PRGM pins by monitoring the discrete signals.  An
    invalid state (PRGM1==PRGM2) causes return=TRUE.  An invalid state of the program pins
    that are used to decode the panel type sets the state to true.
    This function also compares the
    PRGM pin readings with PRGM readings retrieved from NVM.

*/
static bool prgmPins_trp( Bit_t *stat )
{
    bool status = false;
    tPrgm_t tPrgm = {0};
    t16ByteBits tPrgm3to6 = {0U};

    if(tPdi.bProgPinMonitorEnable)
    {
        if (McuGetState() == INITIALIZATION_MODE)
        {
            /* Use the RAW values during initialization routine for Program pins */
            tPrgm.bit.bPRGM1 = PRGM1_RAW;
            tPrgm.bit.bPRGM2 = PRGM2_RAW;
            tPrgm.bit.bPRGM3 = PRGM3_RAW;
            tPrgm.bit.bPRGM4 = PRGM4_RAW;
            tPrgm.bit.bPRGM5 = PRGM5_RAW;
            tPrgm.bit.bPRGM6 = PRGM6_RAW;
        }
        else
        {
            /* Use the debounced values if out of initialization mode for Program pins */
            tPrgm.bit.bPRGM1 = PRGM1;
            tPrgm.bit.bPRGM2 = PRGM2;
            tPrgm.bit.bPRGM3 = PRGM3;
            tPrgm.bit.bPRGM4 = PRGM4;
            tPrgm.bit.bPRGM5 = PRGM5;
            tPrgm.bit.bPRGM6 = PRGM6;
        }

        /* Setup for testing pins 3 thru 6 */
        tPrgm3to6.bit.b0 = tPrgm.bit.bPRGM6;
        tPrgm3to6.bit.b1 = tPrgm.bit.bPRGM5;
        tPrgm3to6.bit.b2 = tPrgm.bit.bPRGM4;
        tPrgm3to6.bit.b3 = tPrgm.bit.bPRGM3;

        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* PATH(prgmPins_trp,B); */

            if (Adc_Raw.val.u16_5V_SENSE > PRGM_PIN_READ_LEVEL)
            {
                /* PATH(prgmPins_trp,F); */

                /* shall return true and report 'fault data' as 0x01 if the PRGM1 and PRGM2 pins are high */
                if ((tPrgm.bit.bPRGM1 == true) && (tPrgm.bit.bPRGM2 == true))
                {
                    /* PATH(prgmPins_trp,C); */
                    status = true;
                    stat->data = 0x01;
#if defined (__SKEW_SNSR_RVDT__)
                    /* shall set FPSU excitation signal to 0 */
                    Rvdt_SinewaveSend(false);
#endif
                }
                /* shall return true and report 'fault data' as 0x02 if the two PRGM pins are low */
                else if ((tPrgm.bit.bPRGM1 == false) && (tPrgm.bit.bPRGM2 == false))
                {
                    /* PATH(prgmPins_trp,D); */
                    status = true;
                    stat->data = 0x02;
#if defined (__SKEW_SNSR_RVDT__)
                    /* shall set FPSU excitation signal to 0 */
                    Rvdt_SinewaveSend(false);
#endif
                }
#if !defined(TRL4_NO_PDI_RIG)
                /* shall return true and report 'fault data' as 0x03 if the any PRGM pin differs
                       from NVM-stored PRGM pin state */
                else if (tPrgm.all != tNvm.tData.Nvm_Rigging.tPrgm.all)
                {
                    /* PATH(prgmPins_trp,E); */
                    status = true;
                    stat->data = 0x03;
                }
#endif
                /* Test whether there is an invalid program pin inputs for the panel decoding */
                else if( (tPrgm3to6.all != MCU_LIB_L) && (tPrgm3to6.all != MCU_LIB_R) &&
                         (tPrgm3to6.all != MCU_RIB_L) && (tPrgm3to6.all != MCU_RIB_R) &&
                         (tPrgm3to6.all != MCU_LOB_L) && (tPrgm3to6.all != MCU_LOB_R) &&
                         (tPrgm3to6.all != MCU_ROB_L) && (tPrgm3to6.all != MCU_ROB_R)  )
                {
                    /* Invalid state of program pins that determine panel type */
                    status = true;
                    stat->data = 0x05;
                }
            }
#if !defined(TRL4_NO_PDI_RIG)
            /* test the PRGM1 and PRGM2 values stored during rigging for invalid values */
            if ( ((tNvm.tData.Nvm_Rigging.tPrgm.bit.bPRGM1 == true) && (tNvm.tData.Nvm_Rigging.tPrgm.bit.bPRGM2 == true)) ||
                 ((tNvm.tData.Nvm_Rigging.tPrgm.bit.bPRGM1 == false) && (tNvm.tData.Nvm_Rigging.tPrgm.bit.bPRGM2 == false)) )
            {
                /* PATH(prgmPins_trp,H); */
                status = true;
                stat->data |= 0x10;
            }

            /* Setup to test PRGM3-6 stored during rigging for invalid values */
            tPrgm3to6.all = 0U;
            tPrgm3to6.bit.b0 = tNvm.tData.Nvm_Rigging.tPrgm.bit.bPRGM6;
            tPrgm3to6.bit.b1 = tNvm.tData.Nvm_Rigging.tPrgm.bit.bPRGM5;
            tPrgm3to6.bit.b2 = tNvm.tData.Nvm_Rigging.tPrgm.bit.bPRGM4;
            tPrgm3to6.bit.b3 = tNvm.tData.Nvm_Rigging.tPrgm.bit.bPRGM3;
            /* test the PRGM3-6 values stored during rigging for invalid values */
            if( (tPrgm3to6.all != MCU_LIB_L) && (tPrgm3to6.all != MCU_LIB_R) &&
                (tPrgm3to6.all != MCU_RIB_L) && (tPrgm3to6.all != MCU_RIB_R) &&
                (tPrgm3to6.all != MCU_LOB_L) && (tPrgm3to6.all != MCU_LOB_R) &&
                (tPrgm3to6.all != MCU_ROB_L) && (tPrgm3to6.all != MCU_ROB_R)  )
            {
                /* Invalid state of program pins that determine panel type */
                status = true;
                stat->data |= 0x20;
            }
#endif
        }
    }
    /* PATH(prgmPins_trp,G); */
    return (status);
}


/*---------- DSP health check functions -----------*/

/**
    brief Checks Trip Event for the ROM CRC fault monitor, 0x70.  Function uses
        the form of function type ::Bit_Check_t.

    This function confirms the CRC of the program memory, and returns TRUE if the CRC match
    fails.

*/
static bool romCrc_trp( Bit_t *stat )
{
    bool status = false;
    tCSCI_Status tCrcResult;
    if(tPdi.bROMCrcCheckMonitorEnable)
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            tCrcResult = CSCI_IntegrityCheck(&CSCI_APP_HEADER);
            if(tCrcResult != CSCI_INTEGRITY_CHECK_PASSED)
            {
                status = true;

                if (G_bChannelA == true)
                {
                    /* PATH(romCrc_trp,K); */
                    stat->data = 1;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(romCrc_trp,L); */
                    stat->data = 2;
                }
            }

            ROM_CRC_done = true;
        }
    }
    /* PATH(romCrc_trp,J); */
    return (status);
}


/**
    brief Checks Trip Event for the RAM fault monitor, 0x71.  Function uses
        the form of function type ::Bit_Check_t.

    This function performs a write/read test of all on-chip RAM memory locations, and returns
    TRUE if there are two successive mismatches.

    Needs to be rewritten for Delfino DSP with consideration given to ECC RAM

    note This function assumes that interrupts are disabled during the test, so as not to corrupt
        the memory regions under test with context switches.

*/
static bool ramTest_trp( Bit_t *stat )
{
    register bool status = false;
//    register unsigned int *pAddr; /* want to use registers for these variables */
//    register unsigned int backup;
//
//    /* PATH(ramTest_trp,A); */
//    if(tPdi.bRAMCheckMonitorEnable)
//    {
//        /* shall ensure that 'stat' pointer is not null */
//        if (stat)
//        {
//            /* PATH(ramTest_trp,B); */
//
//            /* shall loop on range of RAM locations in the M0-M1 bank. */
//            pAddr = (unsigned int *)BIT_RAMTEST_M1_START;
//
//            while ((pAddr < (unsigned int *)BIT_RAMTEST_M1_END) && (status == false))
//            {
//                /* PATH(ramTest_trp,C); */
//
//                /* shall save existing contents of each location in M0-M1 */
//                backup = *pAddr;
//
//                /* shall write test pattern to each location in M0-M1, for each of four test patterns
//                   shall read contents of each location in M0-M1 */
//                *pAddr = BIT_RAMTEST_PATTERN1;
//
//                if (*pAddr != BIT_RAMTEST_PATTERN1)
//                {
//                    /* PATH(ramTest_trp,D); */
//                    status = true;
//                }
//
//                *pAddr = BIT_RAMTEST_PATTERN2;
//
//                if (*pAddr != BIT_RAMTEST_PATTERN2)
//                {
//                    /* PATH(ramTest_trp,E); */
//                    status = true;
//                }
//
//                *pAddr = BIT_RAMTEST_PATTERN3;
//
//                if (*pAddr != BIT_RAMTEST_PATTERN3)
//                {
//                    /* PATH(ramTest_trp,F); */
//                    status = true;
//                }
//
//                *pAddr = BIT_RAMTEST_PATTERN4;
//
//                if (*pAddr != BIT_RAMTEST_PATTERN4)
//                {
//                    /* PATH(ramTest_trp,G); */
//                    status = true;
//                }
//
//                /* shall retest the location in M0-M1, with all four test patterns, if a mismatch was detected */
//                if (status == true)
//                {
//                    /* PATH(ramTest_trp,H); */
//                    status = false;
//                    *pAddr = BIT_RAMTEST_PATTERN1;
//
//                    if (*pAddr != BIT_RAMTEST_PATTERN1)
//                    {
//                        /* PATH(ramTest_trp,I); */
//                        status = true;
//                    }
//
//                    *pAddr = BIT_RAMTEST_PATTERN2;
//
//                    if (*pAddr != BIT_RAMTEST_PATTERN2)
//                    {
//                        /* PATH(ramTest_trp,J); */
//                        status = true;
//                    }
//
//                    *pAddr = BIT_RAMTEST_PATTERN3;
//
//                    if (*pAddr != BIT_RAMTEST_PATTERN3)
//                    {
//                        /* PATH(ramTest_trp,K); */
//                        status = true;
//                    }
//
//                    *pAddr = BIT_RAMTEST_PATTERN4;
//
//                    if (*pAddr != BIT_RAMTEST_PATTERN4)
//                    {
//                        /* PATH(ramTest_trp,L); */
//                        status = true;
//                    }
//                }
//
//                /* shall restore original contents of each location in M0-M1 */
//                *pAddr = backup;
//                pAddr++;
//            }
//
//            /* shall loop on range of RAM locations in the L0-L1 bank. */
//            pAddr = (unsigned int *)BIT_RAMTEST_LS_START;
//
//            while ((pAddr < (unsigned int *)BIT_RAMTEST_LS_END) && (status == false))
//            {
//                /* PATH(ramTest_trp,M); */
//
//                /* shall save existing contents of each location in L0-L1 */
//                backup = *pAddr;
//
//                /* shall write test pattern to each location in L0-L1, for each of four test patterns
//                   shall read contents of each location in L0-L1 */
//                *pAddr = BIT_RAMTEST_PATTERN1;
//
//                if (*pAddr != BIT_RAMTEST_PATTERN1)
//                {
//                    /* PATH(ramTest_trp,N); */
//                    status = true;
//                }
//
//                *pAddr = BIT_RAMTEST_PATTERN2;
//
//                if (*pAddr != BIT_RAMTEST_PATTERN2)
//                {
//                    /* PATH(ramTest_trp,O); */
//                    status = true;
//                }
//
//                *pAddr = BIT_RAMTEST_PATTERN3;
//
//                if (*pAddr != BIT_RAMTEST_PATTERN3)
//                {
//                    /* PATH(ramTest_trp,P); */
//                    status = true;
//                }
//
//                *pAddr = BIT_RAMTEST_PATTERN4;
//
//                if (*pAddr != BIT_RAMTEST_PATTERN4)
//                {
//                    /* PATH(ramTest_trp,Q); */
//                    status = true;
//                }
//
//                /* shall retest the location in L0-L1, with all four test patterns, if a mismatch was detected */
//                if (status == true)
//                {
//                    /* PATH(ramTest_trp,R); */
//                    status = false;
//                    *pAddr = BIT_RAMTEST_PATTERN1;
//
//                    if (*pAddr != BIT_RAMTEST_PATTERN1)
//                    {
//                        /* PATH(ramTest_trp,S); */
//                        status = true;
//                    }
//
//                    *pAddr = BIT_RAMTEST_PATTERN2;
//
//                    if (*pAddr != BIT_RAMTEST_PATTERN2)
//                    {
//                        /* PATH(ramTest_trp,T); */
//                        status = true;
//                    }
//
//                    *pAddr = BIT_RAMTEST_PATTERN3;
//
//                    if (*pAddr != BIT_RAMTEST_PATTERN3)
//                    {
//                        /* PATH(ramTest_trp,U); */
//                        status = true;
//                    }
//
//                    *pAddr = BIT_RAMTEST_PATTERN4;
//
//                    if (*pAddr != BIT_RAMTEST_PATTERN4)
//                    {
//                        /* PATH(ramTest_trp,V); */
//                        status = true;
//                    }
//                }
//
//                /* shall restore original contents of each location in L0-L1 */
//                *pAddr = backup;
//                 pAddr++;
//            }
//
//            /* shall loop on range of RAM locations in the H0-H1 bank. */
//            pAddr = (unsigned int *)BIT_RAMTEST_GS_START;
//
//            while ((pAddr < (unsigned int *)BIT_RAMTEST_GS_END) && (status == false))
//            {
//                /* PATH(ramTest_trp,W); */
//
//                /* shall save existing contents of each location in H0 */
//                backup = *pAddr;
//
//                /* shall write test pattern to each location in H0, for each of four test patterns
//                   shall read contents of each location */
//                *pAddr = BIT_RAMTEST_PATTERN1;
//
//                if (*pAddr != BIT_RAMTEST_PATTERN1)
//                {
//                    /* PATH(ramTest_trp,X); */
//                    status = true;
//                }
//
//                *pAddr = BIT_RAMTEST_PATTERN2;
//
//                if (*pAddr != BIT_RAMTEST_PATTERN2)
//                {
//                    /* PATH(ramTest_trp,Y); */
//                    status = true;
//                }
//
//                *pAddr = BIT_RAMTEST_PATTERN3;
//
//                if (*pAddr != BIT_RAMTEST_PATTERN3)
//                {
//                    /* PATH(ramTest_trp,Z); */
//                    status = true;
//                }
//
//                *pAddr = BIT_RAMTEST_PATTERN4;
//
//                if (*pAddr != BIT_RAMTEST_PATTERN4)
//                {
//                    /* PATH(ramTest_trp,AA); */
//                    status = true;
//                }
//
//                /* shall retest the location in H0, with all four test patterns, if a mismatch was detected */
//                if (status == true)
//                {
//                    /* PATH(ramTest_trp,AB); */
//                    status = false;
//                    *pAddr = BIT_RAMTEST_PATTERN1;
//
//                    if (*pAddr != BIT_RAMTEST_PATTERN1)
//                    {
//                        /* PATH(ramTest_trp,AC); */
//                        status = true;
//                    }
//
//                    *pAddr = BIT_RAMTEST_PATTERN2;
//
//                    if (*pAddr != BIT_RAMTEST_PATTERN2)
//                    {
//                        /* PATH(ramTest_trp,AD); */
//                        status = true;
//                    }
//
//                    *pAddr = BIT_RAMTEST_PATTERN3;
//
//                    if (*pAddr != BIT_RAMTEST_PATTERN3)
//                    {
//                        /* PATH(ramTest_trp,AE); */
//                        status = true;
//                    }
//
//                    *pAddr = BIT_RAMTEST_PATTERN4;
//
//                    if (*pAddr != BIT_RAMTEST_PATTERN4)
//                    {
//                        /* PATH(ramTest_trp,AF); */
//                        status = true;
//                    }
//                }
//
//                /* shall restore original contents of each location in H0 */
//                *pAddr = backup;
//                pAddr++;
//            }
//
//            /* PATH(ramTest_trp,AH); */
//            if (status == true && PrimarySide == true)
//            {
//                /* PATH(ramTest_trp,AI); */
//                stat->data = 1;
//            }
//            else if (status == true && PrimarySide == false)
//            {
//                /* PATH(ramTest_trp,AJ); */
//                stat->data = 2;
//            }
//        }
//    }
//    /* PATH(ramTest_trp,AG); */

    /* shall return true if there is a mismatch (on retest) between contents and test pattern */
    return (status);
}

/******** Reserved for future use -- not needed for SOF *******
**
    brief Checks Trip Event for the Watchdog Timeout fault monitor, 0x73.  Function uses
        the form of function type ::Bit_Check_t.

    This function verifies that the watchdog circuitry effectively resets the processor by
    monitoring the time until the reset occurs.  If greater than 2 ms, returns TRUE.
*/
static bool wdogTest_trp( Bit_t *stat )
{
    bool status = false;

    /* PATH(wdogTest_trp,A); */
    if(tPdi.bWDTimerPowerUpMonitorEnable)    // PDI
    {
        if ((stat) && ((G_bChannelA == true)||(G_bChannelA == false)))
        {
            /* PATH(wdogTest_trp,B); */

//            if ((WDTestStatus.PBitServiced == false) || ((WDTestStatus.endTime - WDTestStatus.startTime) > BIT_WDTEST_mSEC))    // VLJ
            if ((WDTestStatus.PBitServiced == false) || ((WDTestStatus.endTime - WDTestStatus.startTime) > tPdi.u16WDTimerPowerUpMonitorMSec))     // PDI
            {
                /* PATH(wdogTest_trp,C); */
                status = true;

                if (G_bChannelA == true)
                {
                    /* PATH(wdogTest_trp,D); */
                    stat->data = 0x01;
                }
                else if(G_bChannelA == false)
                {
                    /* PATH(wdogTest_trp,E); */
                    stat->data = 0x02;
                }
            }
        }
    }
    /* PATH(wdogTest_trp,F); */
    /* shall return true if execution not interrupted by watchdog reset */
    return (status);
}


/**
    brief Checks Trip Event for the Scheduler Frame Overrun fault monitor, 0x74.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects frame overruns in the Scheduler by monitoring the Master Frame Counter.
    If the frame counter becomes greater than 1, there is a frame overrun.

*/
static bool frameOrun_trp( Bit_t *stat )
{
    static Uint16 total = 0;
    bool status = false;

    /* PATH(frameOrun_trp,A); */
    if(tPdi.bFrameOverrunMonitorEnable)   // PDI
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* PATH(frameOrun_trp,B); */

            /* shall increment NVM status word for the current subframe if the Counter > 1. */
            if (Timer_MasterFrameCount > 1)
            {
                /* PATH(frameOrun_trp,C); */
                status = true;
                Nvm_State.frameOverCount[subframe]++;

                /* shall report most-recent faulty subframe in 'fault data' */
                stat->data = ((subframe & 0x07) << 4);
                Nvm_State.bitstatus.bit.FrameOverrun = 1;
                total += Nvm_State.frameOverCount[subframe];

                /* shall report total overrun count in 'fault data' */
                stat->data |= (total & 0x0F);

                if (G_bChannelA == true)
                {
                    /* PATH(frameOrun_trp,G); */
                    stat->data |= 0x80;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(frameOrun_trp,H); */
                    stat->data &= 0x7F;
                }

            }
        }
    }
    /* PATH(frameOrun_trp,F); */
    return (status);
}

/**
    brief Checks Trip Event for the Invalid Scheduler Frame fault monitor, 0x76.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects an invalid frame index number by checking the Subframe Error status bit
    from NVM.  The bit will be set by the "default" switch-case handler in the scheduler that
    operates on all undefined subframe indices, which then stores the flags to NVM prior to DSP reset.

*/
static bool frameBad_trp( Bit_t *stat )
{
    bool status = false;

    /* PATH(frameBad_trp,A); */
    if(tPdi.bSubframeIndexMonitorEnable)      // PDI
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* PATH(frameBad_trp,B); */

            /* shall return true if the "bad subframe" count is greater than zero */
            if (BadSubframeCnt > 0)
            {
                /* PATH(frameBad_trp,C); */

                if (G_bChannelA == true)
                {
                    /* PATH(frameBad_trp,E); */
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(frameBad_trp,F); */
                    stat->data = 0x02;
                }

                status = true;
            }
        }
    }
    /* PATH(frameBad_trp,J); */
    return (status);
}


/**
    brief Checks Trip Event for the Unused Interrupt fault monitor, 0x77.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects the use of an undefined interrupt by checking the Bad Interrupt status
    bit from NVM.  The bit will be set by the common interrupt handler that is assigned to all
    otherwise undefined interrupt vectors, which then stores the flags to NVM prior to DSP reset.

*/
static bool intrptBad_trp( Bit_t *stat )
{
    bool status = false;

    /* PATH(intrptBad_trp,A); */
    if(tPdi.bUndefinedINTMonitorEnable)       // PDI
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* PATH(intrptBad_trp,B); */

            /* shall return true if the "bad interrupt" count is greater than zero */
            if (UndefinedIntCnt > 0)
            {
                /* PATH(intrptBad_trp,C); */

                if (G_bChannelA == true)
                {
                    /* PATH(intrptBad_trp,E); */
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(intrptBad_trp,F); */
                    stat->data = 0x02;
                }

                status = true;
            }
        }
    }
    /* PATH(intrptBad_trp,J); */
    return (status);
}


/**
    brief Checks Trip Event for the Stack Overflow fault monitor, 0x79.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects a stack overflow by checking the Stack Overflow status bit from NVM.
    The bit will be set by the stack overflow interrupt handler, which then stores the flags
    to NVM prior to DSP reset.
*/
static bool stackOflow_trp( Bit_t *stat )
{
    bool status = false;
    Uint16 stackPtr = 0;

    /* PATH(stackOflow_trp,A); */
    if(tPdi.bStackOverrunMonitorEnable)       // PDI
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* PATH(stackOflow_trp,B); */

            /* shall retrieve the software stack pointer */
            stackPtr = stack_pointer();

            /* shall return true if the stack pointer is greater than the size of the stack, signaling an overflow */
            if (stackPtr >= STACK_END)
            {
                /* PATH(stackOflow_trp,D); */

                if (G_bChannelA == true)
                {
                    /* PATH(stackOflow_trp,F); */
                    stat->data = 0x01;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(stackOflow_trp,G); */
                    stat->data = 0x02;
                }

                status = true;
            }
        }
    }
    /* PATH(stackOflow_trp,K); */
    return (status);
}

/**
    brief Checks Trip Event for the PDI ROM fault monitor, 0x7A.  Function uses
        the form of function type ::Bit_Check_t.

    This function confirms the CRC of the PDI Memory, and returns TRUE if the CRC match
    fails.

*/
static bool pdiCrc_trp( Bit_t *stat )
{
    bool status = false;
    tCSCI_Status tCrcResult;
    if(tPdi.bPDICrcCheckMonitorEnable)       // PDI
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {

            /* Valid Starting address of PDI CSCI. Run the CRC Integrity check */
            tCrcResult = CSCI_IntegrityCheck(&CSCI_PDI_HEADER);
            if(tCrcResult != CSCI_INTEGRITY_CHECK_PASSED)
            {
                /* CRC Check didn't match. Trip monitor and set monitor data to which
                 * channel failed the monitor.
                 */
                status = true;

                if (G_bChannelA == true)
                {
                    /* PATH(romCrc_trp,K); */
                    stat->data = 1;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(romCrc_trp,L); */
                    stat->data = 2;
                }
            }

            PDI_CRC_done = true;
        }
    }
    /* PATH(romCrc_trp,J); */
    return (status);
}

/**
    brief Checks Trip Event for the PDI ROM fault monitor, 0x75.  Function uses
        the form of function type ::Bit_Check_t.

    This function performs range checks on the PDI data. The function will return a
    true if there are any parameters that are out of range and false if all
    parameters are within their respective ranges.

*/
static bool pdiOutOfRange_trp( Bit_t *stat )
{
    bool status = false;
    if(tPdi.bPDIRangeCheckMonitorEnable)       // PDI
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* Valid Starting address of PDI CSCI. Run the Range Check */
            //status = PDI_Mgr_RangeCheckParams();
            status = PDI_Mgr_RangeCheck();
            if(status != false)
            {
                /* PDI Range Check didn't pass. Trip monitor and set monitor data to which
                 * channel failed the monitor.
                 */
                status = true;

                if (G_bChannelA == true)
                {
                    /* PATH(romCrc_trp,K); */
                    stat->data = 1;
                }
                else if (G_bChannelA == false)
                {
                    /* PATH(romCrc_trp,L); */
                    stat->data = 2;
                }
            }
        }
    }
    return (status);
}

/**
    brief Checks Trip Event for the RAM fault monitor, 0x7C.  Function uses
        the form of function type ::Bit_Check_t.

    This function performs a write/read test of all MRAM IC memory locations, and returns
    TRUE if there are any mismatches.

    note This function assumes that interrupts are disabled during the test, so as not to corrupt
        the memory regions under test with context switches.

*/
static bool mramTest_trp( Bit_t *stat )
{
    register bool status = false;
    register uint16_t *pu16Addr; /* want to use registers for these variables */
    register uint16_t u16Backup;
    if(tPdi.bMRAMCheckMonitorEnable)       // PDI
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            /* shall loop thru range of MRAM */
            pu16Addr = (uint16_t *)BIT_MRAM_TEST_START;

            while ((pu16Addr < (uint16_t *)BIT_MRAM_TEST_END) && (status == false))
            {
                /* shall save existing contents of each location in M0-M1 */
                u16Backup = *pu16Addr;

                /* shall write test pattern to each location in MRAM, for each of four test patterns
                   shall read contents of each location in MRAM */
                /* Test pattern 1 */
                *pu16Addr = BIT_RAMTEST_PATTERN1;
                if (*pu16Addr != BIT_RAMTEST_PATTERN1)
                {
                    status = true;
                }

                /* Test pattern 2 */
                *pu16Addr = BIT_RAMTEST_PATTERN2;
                if (*pu16Addr != BIT_RAMTEST_PATTERN2)
                {
                    status = true;
                }

                /* Test pattern 3 */
                *pu16Addr = BIT_RAMTEST_PATTERN3;
                if (*pu16Addr != BIT_RAMTEST_PATTERN3)
                {
                    status = true;
                }

                /* Test pattern 4 */
                *pu16Addr = BIT_RAMTEST_PATTERN4;
                if (*pu16Addr != BIT_RAMTEST_PATTERN4)
                {
                    status = true;
                }

                /* shall restore original contents of each location in M0-M1 */
                *pu16Addr = u16Backup;
                pu16Addr++;
            }

            if ((status == true) && (G_bChannelA == true))
            {
                stat->data = 0x01;
            }
            else if ((status == true) && (G_bChannelA == false))
            {
                stat->data = 0x02;
            }
        }
    }

    /* shall return true if there is a mismatch (on retest) between contents and test pattern */
    return (status);
}

/**
    brief Checks Trip Event for the GFD_I_SENSE monitor, 0x7D.  Function uses
        the form of function type ::Bit_Check_t.

    This function performs a check on bGFD_I_SENSE_ISR, and returns
    True if its value is updated to False.

*/
static bool gfd_i_sense_trp( Bit_t *stat )
{
    bool status = false;
    if(tPdi.bGFDMonitorEnable)       // PDI
    {
        /* shall ensure that 'stat' pointer is not null */
        if (stat)
        {
            if (!bGFD_I_SENSE_ISR)
            {
                if (G_bChannelA == true)
                {
                    stat->data = 0x01;
                }
                else
                {
                    stat->data = 0x02;
                }
                status = true;
            }
        }
    }
    /* shall return true if there is a mismatch (on retest) between contents and test pattern */
    return (status);
}


/**
    brief Checks Trip Event for the Motor Phase Temp. monitor, 0x16.  Function uses
        the form of function type ::Bit_Check_t.

    This function performs a check on Motor Phase Temp., and returns
    True if its value is updated to False.

*/
static bool OverMotorWindingTemp_trp( Bit_t *stat )
{
    bool status = false;
    if(tPdi.bOverTempMonitorEnable)
    {
        if (stat)
        {
            if ((Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHA > MOTOR_MAX_POSITIVE_TEMP_LIMIT) || (Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHB > MOTOR_MAX_POSITIVE_TEMP_LIMIT) ||
                (Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHC > MOTOR_MAX_POSITIVE_TEMP_LIMIT))
            {

                if (G_bChannelA == true)
                {
                   stat->data = 0x01;
                }
                else
                {
                   stat->data = 0x02;
                }
                status = true;

            }
        }
    }
    /* shall return true if there is a mismatch (on retest) between contents and test pattern */
    return (status);
}

/*------- "Trip Effect" execution functions -------*/

/**
    brief Executes "Latched Fault" Trip Effect for numerous fault monitors.  Function uses
        the form of function type ::Bit_Effect_t.

    This effect sets the "FLAP FAIL" bit, and presumes that the caller
    will then put the MCU into the FAULT state.

*/
bool latchedFault_eff( int16 idx )
{

    /* A latched fault will put the system into FAS_FAIL mode. */
    /* shall set "FLAP FAIL" bit for overall MCU status reporting */
    /* Non-Compliance: The following case statements lack a break statement.
       Justification: The same functionality is required for all of the following cases.
    */
    switch (idx)
    {
        case 0x10:
        case 0x25:
		case 0x30:
		case 0x4A:
        case 0x4C:
        case 0x4E:
        case 0x50:
        case 0x57:
        case 0x70:
        case 0x71:
        case 0x73:
        case 0x7A:
        case 0x7B:
			bMcuFail = true;
			break;
        /* Non-Compliance: The following case statements lack a break statement.
           Justification: The same functionality is required for all of the following cases.
        */
        case 0x12:
        case 0x1A:
        case 0x1B:
        case 0x4B:
        case 0x4D:
        case 0x51:
        case 0x52:
        case 0x55:
        case 0x56:
        case 0x74:
        case 0x76:
        case 0x77:
        case 0x79:
        case 0x7D:

			if (McuGetState() == RIG_MODE)
			{
				bInhibitFlap = true;
			}

			bMcuFail = true;
            break;
        default:
            break;
    }

    bFlapFail = true;

    /* PATH(latchedFault_eff,AH); */
    return true;
}


/**
    brief Executes "Inhibit" Trip Effect for numerous fault monitors.  Function uses
        the form of function type ::Bit_Effect_t.

    This effect sets the "FLAP FAIL" bit, and presumes that the caller
    will restrict motor motion in the prescribed way.

*/
bool inhibit_eff( int16 idx )
{
    /* PATH(inhibit_eff,A); */

    /* shall set "Flap Fail" bit for overall MCU status reporting */
	/* If any of the inhibits are set, currently the only inhibits that use
	this effect function are SYNC monitors.*/
    switch (idx)
    {
        case 0x16:/*over Motor winding temp. monitor*/
            bMcuFail = true;
            break;
        case 0x49:
        case 0x4F:
            bScuFail = true;
            break;
        default:
            break;
    }

    if (McuGetState() == RIG_MODE)
    {
        bInhibitFlap = true;
    }

	/* Set Flap_Fail to true to be communicated on DFT and ARINC825 busses. */
    bFlapFail = true;

    /* PATH(inhibit_eff,N); */
    return true;
}

/**
    brief Executes "Latched" Trip Effect for numerous WARNING monitors.  Function uses
        the form of function type ::Bit_Effect_t.

*/
bool latchedWarning_eff( int16 idx )
{
    /* Set the MCU_Warning bit to true for status reporting. */
    bMcuWarning = true;

    /* PATH(inhibit_eff,N); */
    return true;
}

/**
    brief Executes "Flap Jam" Trip Effect for fault monitor 0x04.  Function uses
        the form of function type ::Bit_Effect_t.

    This is a complex trip effect that requires "Inhibit" action on the first 2 trip occurrences,
    then a "Latched Fault" on the third occurrence.  The third occurrence is handled by monitor 0x0F.
    The "inhibit" action is presumed to be carried out by the caller.

*/
static bool flapJam_eff( int16 idx )
{
    /* PATH(flapJam_eff,A); */

    bFlapFail = true;

    /* set status bit to inhibit motor for this control channel */
	MotorCmd.MotorStop = true;

	bFlapJam = true;

    /* PATH(flapJam_eff,H); */
    return true;
}

#if defined(__SKEW_SNSR_RVDT__)
/**
    brief Executes "Latched Fault" Trip Effect for RVDT fault monitors.  Function uses
        the form of function type ::Bit_Effect_t.

    This effect sets FPSU excitation signal to 0, then
    invokes the common 'latched Fault' effect to set the FAS FAIL bit and latch the fault.

*/
static bool rvdtExcFault_eff( int16 idx )
{
    /* PATH(rvdtLeftFault_eff,A); */

    if ((McuGetState() != RIG_MODE) && (McuGetState() != TEST_MODE))
    {
        /* PATH(rvdtLeftFault_eff,C); */
        /* shall set FPSU excitation signal to 0 */
        Rvdt_SinewaveSend(false);
    }

    /* PATH(rvdtLeftFault_eff,B); */

    /* shall invoke common fault effect for latched fault */
    return (latchedFault_eff(idx));
}
#endif

/**
    brief Executes "Rigging Check" Trip Effect for fault monitor 0x64.  Function uses
        the form of function type ::Bit_Effect_t.

    This effect inhibits further Rig operation.

*/
static bool rigVerify_eff( int16 idx )
{
    /* PATH(rigVerify_eff,A); */

	if(idx != 0)
    {
        /* PATH(rigVerify_eff,B); */
	    /* shall inhibit further Rig operation by setting a global flag. */
	    Inhibit_Rig = true;
        bFlapFail = true;
		bMcuFail = true;
    }

    /* PATH(rigVerify_eff,C); */
    return true;
}


static bool hallBadSeq_eff( int16 idx )
{
    /* PATH(hallBadSeq_eff,A); */
    bFlapFail = true;
    bMotorFail = true;

    if (McuGetState() == RIG_MODE)
    {
        bInhibitFlap = true;
    }

    /* PATH(hallBadSeq_eff,D); */
    return true;
}


static bool hallBadState_eff( int16 idx )
{
    /* PATH(hallBadState_eff,A); */
    bFlapFail = true;
    bMotorFail = true;

    if (McuGetState() == RIG_MODE)
    {
        bInhibitFlap = true;
    }

	/* PATH(hallBadState_eff,E); */
    return true;
}


static bool brakeHold_eff( int16 idx )
{
    /* PATH(brakeHold_eff,A); */
    bFlapFail = true;
    bMotorFail = true;

    /* PATH(brakeHold_eff,D); */
    return true;
}

static bool MotorPhaseCurrent_eff( int16 idx )
{

    bFlapFail = true;
    bMotorFail = true;

    if (McuGetState() == RIG_MODE)
    {
        bInhibitFlap = true;
    }

    return true;
}

/*------- "Reset Condition" check functions -------*/

/**
    brief Checks Reset Condition "on-ground = true AND FSCU power reset" for numerous fault monitors.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects the condition by checking the stored "boot count" to see if it has changed since the
    fault was detected, then verifying that the aircraft's WOW pins show that the craft is on the ground.

*/
static bool groundCycle_rst( Bit_t *stat )
{
    bool status = false;

    /* PATH(groundCycle_rst,A); */

    /* shall ensure that 'stat' pointer is not null */
    if (stat)
    {
        /* PATH(groundCycle_rst,B); */

        /* shall return true only if a Power Cycle has elapsed AND the WOW pin (WOW_N) indicates
           that the aircraft is on the ground. */

        /*Commenting out as WOW_N_RAW signal is obsolete in MCU SW,
         * Need to re-check this part of code during WOW clean-up
         */
        /*if ((WOW_N_RAW == 0) && powerCycle_rst(stat) && (McuGetState() == BOOT_MODE))*/
        if (powerCycle_rst(stat) && (McuGetState() == BOOT_MODE))
        {
            /* PATH(groundCycle_rst,C); */
            status = true;
            bFlapFail = false;
        }
    }

    /* PATH(groundCycle_rst,D); */
    return (status);
}

static bool scuMcuEnable_rst( Bit_t *stat )
{
    bool status = false;
    /* PATH(groundCycle_rst,A); */

    /* shall ensure that 'stat' pointer is not null */
    if (stat)
    {
        /* PATH(groundCycle_rst,B); */

        /* Reset when the SCU enables the MCU by turning on the SCU_MCU_ENABLE signal  */
        if ((powerCycle_rst(stat)) || (MTR_EN_CHA == 1))
        {
            /* PATH(groundCycle_rst,C); */
            status = true;
            bMcuWarning = false;
        }
    }

    /* PATH(groundCycle_rst,D); */
    return (status);
}

static bool xenable_rst( Bit_t *stat )
{
    bool status = false;

    /* shall ensure that 'stat' pointer is not null */
    if (stat)
    {
        /* Reset when the CHX_STATUS is true (healthy)  */
        if ((powerCycle_rst(stat)) || (CHX_STATUS == true))
        {
            status = true;
            bMcuWarning = false;
        }
    }

    return (status);
}


/**
    brief Checks Reset Condition "on-ground = true AND MCU power reset" for a latched fault jam monitor fault.
	Will reset if the number of Jam is less than 3, and a valid FSL command has been received.


	GlobalsReferenced
	bool Fls_Valid
	bool RightFlapJam
	bool LeftFlapJam
	bool MotorCmd.MotorStart


	retval	true: Reset condition has been met
			false: Reset condition has not been met

*/
static bool flapJam_rst( Bit_t *stat)
{
    bool status = false;

    /* PATH(flapJam_rst,A); */

    if (stat)
    {
        /* PATH(flapJam_rst,B); */
        /* Check the reset condition if the jam monitor is latching*/
        if ((McuGetState() == BOOT_MODE) || (((stat->data & 0x0F) >= 3) && (McuGetState() != RIG_MODE)))
        {
            /* PATH(flapJam_rst,C); */
            status = groundCycle_rst(stat);
        }
		/*Check the reset condition if the jam montior is an inhibit*/
        else if (((stat->data & 0x0F) < 3) || (McuGetState() == RIG_MODE))
        {
            /* PATH(flapJam_rst,D); */

			/* MotorCmd.MotorStart covers the reset condition for RIG_MODE */
            if (MotorCmd.MotorStart == true)
            {
                /* PATH(flapJam_rst,E); */
            	/*If the reset is valid, the reset the flags that could be
				set as a result of this monitor tripping*/
                bFlapJam = false;
                bFlapFail = false;
                status = true;
            }
        }
    }

    /* PATH(flapJam_rst,F); */
    return (status);
}


/**
    brief Checks Reset Condition "MCU power reset" for numerous fault monitors.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects the condition by checking the stored "boot count" to see if it has changed since the
    fault was detected.

*/
static bool powerCycle_rst( Bit_t *stat )
{
    bool status = false;

    /* PATH(powerCycle_rst,A); */

    /* shall ensure that 'stat' pointer is not null */
    if (stat)
    {
        /* PATH(powerCycle_rst,B); */

        /* shall compare current Boot Count to boot count recorded with fault, and return true if current BootCount is greater */
        if (Nvm_State.bootcount > stat->tripBoot)
        {
            /* PATH(powerCycle_rst,C); */
            status = true;
            /* Power Cycle clears all warnings */
            bMcuWarning = false;
        }
    }

    /* PATH(powerCycle_rst,D); */
    return (status);
}

/**
    brief Checks Reset Condition "Re-enter Rig" for fault monitor 0x64.  Function uses
        the form of function type ::Bit_Check_t.

    A monitor that requires a re-rig to clear, must enter rig mode and successfully complete
    a rigging process. When this occurs a new NVM_State record is written into NVM, without
    any fault codes, and by default this would clear any failure that requires a re-rig. If
    a fault is active in the system that requires a re-rig, and this process has not been
    competed yet, then this reset function will be invoke and continuously return a false value.

*/
static bool rigVerify_rst( Bit_t *stat )
{
    /* PATH(rigVerify_rst,A); */
    return false;
}

///**
//    brief Checks if the conditions to reset the Flap Command Skew Monitor for the
//    ARINC825 Control Bus have been met.
//
//
//    retvale true: If the reset condition for this monitor has been met.
//            false: If the reset condition for this monitor has not been met.
//*/
//static bool a825CntrlBusFlapCmdSkewMon_rst( Bit_t *stat )
//{
//    bool status = false;
//    float32_t f32DiffInPosCmd = 0.0F;
//
//    /* shall ensure that 'stat' pointer is not null */
//    if (stat)
//    {
//
//        /* Calculate difference in position commands between Left and Right Actuators */
//        f32DiffInPosCmd = tScuInterface.f32PositionCmdL - tScuInterface.f32PositionCmdR;
//
//        /* Reset fault if position command between actuators is within allowable tolerance */
//        if((f32DiffInPosCmd >= -MAX_POS_CMD_DIFF_ALLOWED) && (f32DiffInPosCmd <= MAX_POS_CMD_DIFF_ALLOWED))
//        {
//            status = true;
//            bFlapFail = false;
//        }
//    }
//
//    return (status);
//}

/**
    brief Checks if the conditions to reset the Flap Command Skew Monitor for the
    ARINC825 Maintenance bus have been met.


    retvale true: If the reset condition for this monitor has been met.
            false: If the reset condition for this monitor has not been met.
*/
static bool a825MaintBusFlapCmdSkewMon_rst( Bit_t *stat )
{
    bool status = powerCycle_rst(stat); /* Set status to status of power cycle reset */
    float32_t f32DiffInPosCmd = 0.0F;

    /* shall ensure that 'stat' pointer is not null */
    if (stat)
    {

        /* Calculate difference in position commands between Left and Right Actuators */
        // AB:  need to define the GSE Interface for rigging. Should have an L/R commands though.
        //f32DiffInPosCmd = tGseInterface.f32PositionCmdL - tGseInterface.f32PositionCmdR;

        /* Reset fault if a power reset occured or position command between actuators is within allowable tolerance */
        if((status == true) || ((f32DiffInPosCmd >= -MAX_POS_CMD_DIFF_ALLOWED) && (f32DiffInPosCmd <= MAX_POS_CMD_DIFF_ALLOWED)))
        {
            status = true;
            bFlapFail = false;
        }
    }

    return (status);
}

/**
    brief Checks if the conditions to reset the Stale Comms for the
    ARINC825 Maintenance bus have been met.


    retvale true: If the reset condition for this monitor has been met.
            false: If the reset condition for this monitor has not been met.
*/
static bool a825MaintCommsStaleMon_rst( Bit_t *stat )
{
    bool status = powerCycle_rst(stat); /* Set status to status of power cycle reset */

    /* shall ensure that 'stat' pointer is not null */
    if (stat)
    {
        /* Reset fault if GSE comms are not stale any longer */
        if((status == true) || (tA825_Bus[ARINC825_MAINT_BUS].tRx.ptMsgList[A825_MAINT_BUS_RX_RIG_CMD].bMsgStaleStatus == false))
        {
            status = true;
            bFlapFail = false;
            bInhibitFlap = false;
        }
    }

    return (status);
}

static bool xcomms_rst( Bit_t *stat )
{
    bool status = false;

	/* PATH(xcomms_rst,A); */

    if (stat)
    {
    	/* PATH(xcomms_rst,B); */

		if ((powerCycle_rst(stat) == true) || ((ICC_InvalidCRCcounter < ICC_INVALID_CRC_COUNTER) && (ICC_TimeOutCounter < ICC_TIMOUT_LIMIT)))
        {
			/* PATH(xcomms_rst,C); */
		    bMcuWarning = false;
            status = true;
        }
    }

	/* PATH(xcomms_rst,D); */
    return (status);
}

/**
    brief Checks Reset Condition "Motor Over Winding Temp." for numerous fault monitors.  Function uses
        the form of function type ::Bit_Check_t.

    This function detects the condition by checking the stored "boot count" to see if it has changed since the
    fault was detected.

*/

static bool OverMotorWindingTemp_rst( Bit_t *stat )
{
    bool status = false;

    if (stat)
    {
        if ((Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHA < MOTOR_FAULT_RESET_TEMP_LIMIT) && (Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHB < MOTOR_FAULT_RESET_TEMP_LIMIT) &&
            (Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHC < MOTOR_FAULT_RESET_TEMP_LIMIT))
        {

            status = true;
        }
    }
    /* shall return true if there is a mismatch (on retest) between contents and test pattern */
return (status);
}


/*------- IBIT-type Test Condition Setup execution functions -------*/


/*------- Reserved for future use -- not needed for SOF -------*/
/*
    brief Stores "initial" quadrature count for "Free Play" monitor 0x13.  The 'trip check' function
        for this monitor will compare any new quadrature count value to this stored value.
        Uses the form of function type ::Bit_TestSetup_t.

*/
static bool freePlay_tst( Uint16  stage )
{
    bool status = false;
    static Uint16 minValue = 4095;
    static Uint16 InitialFPSU = 0;

    /* PATH(freePlay_tst,A); */

    switch (stage)
    {
        case 0:
            /* PATH(freePlay_tst,B); */
            bInhibitFlap = false;
            InitialFPSU = 0;
            minValue = 4095;
            Timer_ResetTimer(&IBITDelayTimer);

            if (tHall.Position < Nvm_GetPosOnside(P11))
            {
                /* PATH(freePlay_tst,C); */
                Panel_GoToPosition(Nvm_Rigging_Temp.quad[P11].onside, 1.0);
            }
            else
            {
                /* PATH(freePlay_tst,D); */
                FreePlayStage++;
            }
            break;
        case 1:
            /* PATH(freePlay_tst,E); */
            MotorCmd.StopPosition = NO_STOP;
            MotorCmd.MotorDirection = CW;
            MotorCmd.MotorStop = false;
            MotorCmd.MotorStart = true;
            MotorCmd.MotorSpeed = 0.1;
            MotorCmd.Rigging = true;
            MotorCmd.RiggingHardStop = true;
            FreePlayStage++;
            break;
        case 2:
            /* PATH(freePlay_tst,F); */

            if ((MotorCmd.RiggingHardStop == false) && (MotorCmd.Rigging == true))
            {
                /* PATH(freePlay_tst,G); */
                MotorCmd.Rigging = false;

                /* shall store current Quadrature count reading for use by Free Play Monitor trip check function */
                freeplayInitialQuad = tHall.Position;
                FreePlayStage++;
            }
            break;
        case 3:
            /* PATH(freePlay_tst,H); */

            /* Grab FPSU Data */
            if (Timer_IsSet(&IBITDelayTimer) == false)
            {
                /* PATH(freePlay_tst,I); */
                Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_1s);
                minValue = Adc_Averaged.val.u16_RVDT_POS;
            }
            else if (Timer_IsExpired(&IBITDelayTimer) == false)
            {
                /* PATH(freePlay_tst,J); */

                if (minValue > Adc_Averaged.val.u16_RVDT_POS)
                {
                    /* PATH(freePlay_tst,K); */
                    minValue = Adc_Averaged.val.u16_RVDT_POS;
                }
            }
            else if (InitialFPSU == 0)
            {
                /* PATH(freePlay_tst,L); */
                InitialFPSU = minValue;
                FreePlayStage++;
            }
            else
            {
                /* PATH(freePlay_tst,M); */
                FreePlayStage++;
            }
            break;
        case 4:
            /* PATH(freePlay_tst,N); */

            /* Look for 0.1 degFlap (5 ADC counts) */
            if ((InitialFPSU - minValue) > BIT_FREEPLAY_MOVE)
            {
                /* PATH(freePlay_tst,O); */
                freeplayFinalQuad = tHall.Position;
                FreePlayStage = 6;
            }
            else
            {
                /* PATH(freePlay_tst,P); */
                FreePlayStage = 5;
            }
            break;
        case 5:
            /* PATH(freePlay_tst,Q); */

            /* Retract by 1 quad count */
            Panel_GoToPosition(tHall.Position - 1, 0.1);
            Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_1s);
            FreePlayStage = 3;
            break;
        case 6:
            /* PATH(freePlay_tst,R); */

            /* Drive back to Retract */
            Panel_GoToPosition(Nvm_Rigging_Temp.quad[ZERO].onside, 1.0);
            FreePlayStage++;
            break;
        case 7:
            /* PATH(freePlay_tst,S); */
            InitialFPSU = minValue;

            if (tHall.Position == Nvm_GetPosOnside(ZERO))
            {
                /* PATH(freePlay_tst,T); */

                /* Call BitCheckMonitor function to run test */
                status = Bit_CheckMonitor(0x13);
                Ibit_Stat.tstCmplt = COMPLETED;
                Ibit_Stat.result = false;
                FreePlayStage = 0;
                bInhibitFlap = true;
            }
            break;
        default:
            /* PATH(freePlay_tst,U); */
            break;
    }

    /* PATH(freePlay_tst,V); */
    return (status);
}

/**
    brief Executes "Brake Hold" test condition setup routine for monitor 0x23.
        Uses the form of function type ::Bit_TestSetup_t.

    The Brake Hold test is a three-stage test setup:
        - Stage 1: Start of test, ensure brake is engaged
        - Stage 2: After cross-channel enable occurs, drive motor to maximum torque
        - Stage 3: After 5 seconds of testing trip condition, cease motor drive

    note This function assumes that it is operated in the context of the scheduler, so
        that the commanded actuation activities occur without further intervention.
*/
static bool brakeHold_tst( Uint16  stage )
{
    bool result = true;

    /* PATH(brakeHold_tst,A); */

    switch (stage)
    {
        case 1:
            /* PATH(brakeHold_tst,B); */

            /* shall de-activate the brake (to engage it) during the first test stage */
            Brake_Deactivate();
            break;
        case 2:
            /* PATH(brakeHold_tst,C); */

            /* shall store current Quadrature count reading for use by Brake Hold Monitor trip check function during stage 2 */
            brakeholdInitialQuad = tHall.Position;

            /* shall drive the motor to maximum during stage 2 */
            MotorCmd.StopPosition = NO_STOP;
            MotorCmd.MotorDirection = CW;
            MotorCmd.MotorStop = false;
            MotorCmd.MotorStart = true;
            MotorCmd.MotorSpeed = 1.0;
            bInhibitFlap = false;

            /* shall set a 500 ms timer during stage 2 */
            Timer_SetTime(&brakeHoldTimer, TIMER_ONESHOT, 5 * TIMER_100ms);
            break;
        case 3:
            /* PATH(brakeHold_tst,D); */

            /* shall stop the motor during stage 3 */
            MotorCmd.MotorSpeed = 0.0;
            MotorCmd.MotorStop = true;
            bInhibitFlap = true;
            //EvaRegs.ACTRA.all = 0x0CCC;
            Motor_DynamicBrakeWindings();   /* Dynamic Brake the motor */
            result = false;
            BrakeHoldTestCmplt = true;
            break;
        default: /* shall do nothing when 'stage' is not between 1 and 3, inclusive. */
            /* PATH(brakeHold_tst,E); */
            break;
    }

    /* PATH(brakeHold_tst,F); */
    return (result);
}


/******** Reserved for future use -- not needed for SOF *******
**
    brief Executes "Brake Switch circuit" test condition setup routine for monitor 0x25.
        Uses the form of function type ::Bit_TestSetup_t.

*/
static bool vbrakeMin_tst( Uint16  stage )
{
    bool status = false;

    // AB: REWORK FOR MCU WHEN TEST MODE IS ADDED
//    /* PATH(vbrakeMin_tst,A); */
//
//    switch (stage)
//    {
//        case 0:
//            /* PATH(vbrakeMin_tst,B); */
//
//            /* Clear the GPIO pins that go to the brake */
//            EALLOW;
//            GPIO_A_BIT25_mux();
//            GPIO_A_BIT25_dir();
//            EDIS;
//            U_BRAKE_N_set();
//            L_BRAKE_N_set();
//            BrakeSwitchMonStage++;
//            InhibitFlap = false;
//            Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_50ms);
//            break;
//        case 1:
//            /* PATH(vbrakeMin_tst,C); */
//            Bit_LatchedMonitors(0x25, true);
//
//			if (LatchedFaults_Stat.bit.BrakeSwitch_M == 1)
//			{
//    	        /* PATH(vbrakeMin_tst,I); */
//				BrakeSwitchTestCmplt = true;
//			}
//			else
//			{
//	            /* PATH(vbrakeMin_tst,J); */
//	            U_BRAKE_N_clr();
//	            BrakeSwitchMonStage++;
//	            Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_50ms);
//			}
//            break;
//        case 2:
//            /* PATH(vbrakeMin_tst,D); */
//            Bit_LatchedMonitors(0x25, true);
//            U_BRAKE_N_set();
//            L_BRAKE_N_clr();
//            Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_50ms);
//            BrakeSwitchMonStage++;
//            break;
//        case 3:
//            /* PATH(vbrakeMin_tst,E); */
//            Bit_LatchedMonitors(0x25, true);
//            BrakeSwitchMonStage++;
//            break;
//        case 4:
//            /* PATH(vbrakeMin_tst,F); */
//            U_BRAKE_N_set();
//            L_BRAKE_N_set();
//            BrakeSwitchTestCmplt = true;
//            break;
//        default:
//            /* PATH(vbrakeMin_tst,G); */
//            break;
//    }
//
//	if (BrakeSwitchTestCmplt == true)
//	{
//	    /* PATH(vbrakeMin_tst,K); */
//        EALLOW;
//        GPIO_A_mux();
//        GPIO_A_dir();
//        EDIS;
//        //EvaRegs.ACTRA.all = 0x0CCC;
//        Motor_DynamicBrakeWindings();   /* Dynamic Brake the motor */
//        InhibitFlap = true;
//        Brake_SetPWM(MIN_DUTY_CYCLE);
//	}

    status = (bool)LatchedFaults_Stat.bit.BrakeSwitch_M;

    /* PATH(vbrakeMin_tst,H); */
    return (status);
}


/*
    brief Executes "Bridge Monitor" test condition setup routine for monitor 0x50.
        This function will toggle the following signals and test for voltage tolerances
		to be met.
		MOTORUA_N (DSP Pin 106,
  		MOTORUB_N (DSP Pin 95, Upper
		MOTORUC_N (DSP Pin 93, Upper
		PWMLA_N (DSP Pin 98, Lower
		PWMLB_N (DSP Pin 94, Lower
		PWMLC_N (DSP Pin 92, Lower

		retval	true: Monitor has tripped.
				false: Monitor has not tripped.

*/
static bool bridgeMon_tst( Uint16 stage )
{
    /* PATH(bridgeMon_tst,A); */

    // AB: REWORK FOR MCU WHEN TEST MODE IS ADDED
//    /*shall select valid analog input range for given test stage so that 'trip check' function can test against it */
//    switch (stage)
//    {
//        case 0:
//            /* PATH(bridgeMon_tst,B); */
//
//            /* Clear the GPIO pins that go to the bridge */
//            EALLOW;
//            GPIO_A_BIT50_mux();
//            GPIO_A_BIT50_dir();
//            EDIS;
//            MOTORUA_N_set();
//            MOTORUB_N_set();
//            MOTORUC_N_set();
//            PWMLA_N_set();
//            PWMLB_N_set();
//            PWMLC_N_set();
//            BridgeMonStage++;
//            InhibitFlap = false;
//            Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_50ms);
//            break;
//        case 1:
//            /* PATH(bridgeMon_tst,C); */
//            Bit_LatchedMonitors(0x50, true);
//            MOTORUA_N_clr();
//            BridgeMonStage++;
//            Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_50ms);
//            break;
//        case 2:
//            /* PATH(bridgeMon_tst,D); */
//            Bit_LatchedMonitors(0x50, true);
//            MOTORUA_N_set();
//            BridgeMonStage++;
//            break;
//		case 3:
//    		/* PATH(bridgeMon_tst,M); */
//            PWMLA_N_clr();
//    		BridgeMonStage++;
//    		Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_50ms);
//    		break;
//        case 4:
//            /* PATH(bridgeMon_tst,E); */
//            Bit_LatchedMonitors(0x50, true);
//            PWMLA_N_set();
//            MOTORUB_N_clr();
//            BridgeMonStage++;
//            Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_50ms);
//            break;
//        case 5:
//            /* PATH(bridgeMon_tst,F); */
//            Bit_LatchedMonitors(0x50, true);
//            MOTORUB_N_set();
//            BridgeMonStage++;
//            break;
//        case 6:
//            /* PATH(bridgeMon_tst,N); */
//            PWMLB_N_clr();
//            BridgeMonStage++;
//            Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_50ms);
//            break;
//        case 7:
//            /* PATH(bridgeMon_tst,G); */
//            Bit_LatchedMonitors(0x50, true);
//            PWMLB_N_set();
//            MOTORUC_N_clr();
//            BridgeMonStage++;
//            Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_50ms);
//            break;
//        case 8:
//            /* PATH(bridgeMon_tst,H); */
//            Bit_LatchedMonitors(0x50, true);
//            MOTORUC_N_set();
//            BridgeMonStage++;
//            break;
//        case 9:
//            /* PATH(bridgeMon_tst,O); */
//            PWMLC_N_clr();
//            BridgeMonStage++;
//            Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_50ms);
//            break;
//        case 10:
//            /* PATH(bridgeMon_tst,I); */
//            Bit_LatchedMonitors(0x50, true);
//			PWMLC_N_set();
//            BridgeMonStage++;
//            Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_50ms);
//            break;
//        case 11:
//            /* PATH(bridgeMon_tst,J); */
//            MOTORUA_N_set();
//            MOTORUB_N_set();
//            MOTORUC_N_set();
//            PWMLA_N_set();
//            PWMLB_N_set();
//            PWMLC_N_set();
//
//            EALLOW;
//            GPIO_A_mux();
//            GPIO_A_dir();
//            EDIS;
//
//            /* Set dynamic brake */
//            EvaRegs.ACTRA.all = 0x0CCC;
//            BridgeMonTestCmplt = true;
//            InhibitFlap = true;
//            break;
//        default:
//            /* PATH(bridgeMon_tst,K); */
//            break;
//    }

    /* PATH(bridgeMon_tst,L); */
    return ((bool)LatchedFaults_Stat.bit.BridgeMonitor_M);
}

/**
    brief Executes the state machine to traverse through the Rig Verifiction monitor 0x64.
        The 'trip check' function will run the according test for the current commanded
        test number and test index.

    Test state machine is designed to be called until it has completed. There is no
    complete state because the state is used in the bit monitor data to determine which test
    case failed. Need to keep it in the final state that was ran before failing or completion.

    status:  False = test in progress.
    status:  True = test complete (successfully or unsuccessfully).

*/
static bool rigVerify_tst( Uint16 stage )
{
    bool status = false;

    /* Save the monitor data off into a temporary structure that is easier to work with */
    t16RigVerifyDataByteBits *ptData = (t16RigVerifyDataByteBits*) &bitStatus[0x64].data;
    STATE_ID tMode = McuGetState();

    /* Set the Rig Verify Test to Run for the trip function */
    ptData->bit.uRigTest = (tRigVerifyTstStates_t) stage;

    /* Rig Verification Check State Machine */
    switch((tRigVerifyTstStates_t) stage)
    {
        case TST_RIG_VERIFY_INIT:
        {
            /* Reset the monitor data */
            ptData->all = 0U;

            /* Check if in RIG_MODE or not */
            if(tMode != RIG_MODE)
            {
                /* When not in rig mode only the Rig Status check is completed */
                tRigVerifyTstState = TST_RIG_VERIFY_RIG_STATUS;
            }
            else
            {
                /* In Rig Mode the Rig Status check is not done, and all the other tests are.
                 * Start with the TST_RIG_VERIFY_CHK_QUAD_ONSIDE test and progress from there. */
                tRigVerifyTstState = TST_RIG_VERIFY_CHK_QUAD_ONSIDE;
            }
            break;
        }
        case TST_RIG_VERIFY_RIG_STATUS:
        {
            /* case is only for P-BIT (not RIG_MODE) */
            /* Run the critical monitor trip function. This is the only test case during
             * PBIT. Transition straight to complete when done */
            Bit_CriticalMonitors(0x64, true);

            /* PBIT test is complete. Fault flag is checked after PBIT during boot */
            /* Test Failed. Mark as Test Complete and remain in current state */
            status = true;

            break;
        }
        /* All other test states except COMPLETE are only for RIG_MODE */
        case TST_RIG_VERIFY_CHK_QUAD_ONSIDE:
#if defined(__HALLX_CONFIGURED)
        case TST_RIG_VERIFY_CHK_QUAD_XSIDE:
#endif
        case TST_RIG_VERIFY_CHK_RVDT_STROKE:
        {
            if(ptData->bit.uRigPosition < NUM_RIG_POSITIONS)
            {
                /* Run the critical monitor trip function */
                Bit_CriticalMonitors(0x64, true);

                /* Check if monitor passed to proceed to next step */
                if(CriticalFaults_Stat.bit.RiggingCheck_M != true)
                {
                    /* Increment the rig index to check */
                    ptData->bit.uRigPosition++;

                    /* Check if all rig positions have been checked */
                    if(ptData->bit.uRigPosition == NUM_RIG_POSITIONS)
                    {
                        if(tRigVerifyTstState != TST_RIG_VERIFY_FINAL_CHECK_STATE)
                        {
                            /* Init rig position index for next state */
                            ptData->bit.uRigPosition = 0U;
                            /* Go to next state */
                            tRigVerifyTstState++;
                        }
                        else
                        {
                            /* Verify Check successfully completed. Mark as Test Complete and remain in current state */
                            status = true;
                        }
                    }
                }
                else
                {
                    /* Test Failed. Go to complete state. */
                    /* Test Failed. Mark as Test Complete and remain in current state */
                    status = true;
                }
            }
            break;
        }
        default:
        {
            /* Invalid state. Go to initialization state */
            tRigVerifyTstState = TST_RIG_VERIFY_INIT;
            break;
        }
    }

    return (status);
}

bool IBIT_RamTest(void)
{
    register bool status = false;
    // AB: REWORK FOR MCU WHEN TEST MODE IS ADDED
//    register unsigned int *pAddr; /* want to use registers for these variables */
//    register unsigned int backup;
//    static unsigned int *lastAddr = 0;
//    static bool RAM_Test_Started = false;
//    int16 i = 0;
//
//    /* PATH(IBIT_RamTest,A); */
//
//
//    if (RAM_Test_Started == false)
//    {
//        /* PATH(IBIT_RamTest,B); */
//        pAddr = (unsigned int *)BIT_RAMTEST_MSTART;
//        RAM_Test_Started = true;
//    }
//    else
//    {
//        /* PATH(IBIT_RamTest,C); */
//        pAddr = lastAddr;
//    }
//
//    for (i = 0; i < 10; i++)
//    {
//        /* PATH(IBIT_RamTest,D); */
//        backup = *pAddr;
//
//        *pAddr = BIT_RAMTEST_PATTERN1;
//
//	    if (*pAddr != BIT_RAMTEST_PATTERN1)
//	    {
//	        /* PATH(IBIT_RamTest,E); */
//	        status = true;
//	    }
//
//	    *pAddr = BIT_RAMTEST_PATTERN2;
//
//	    if (*pAddr != BIT_RAMTEST_PATTERN2)
//	    {
//	        /* PATH(IBIT_RamTest,F); */
//	        status = true;
//	    }
//
//	    *pAddr = BIT_RAMTEST_PATTERN3;
//
//	    if (*pAddr != BIT_RAMTEST_PATTERN3)
//	    {
//	        /* PATH(IBIT_RamTest,G); */
//	        status = true;
//	    }
//
//	    *pAddr = BIT_RAMTEST_PATTERN4;
//
//	    if (*pAddr != BIT_RAMTEST_PATTERN4)
//	    {
//	        /* PATH(IBIT_RamTest,H); */
//	        status = true;
//	    }
//
//	    /* shall retest the location in M0-M1, with all four test patterns, if a mismatch was detected */
//	    if (status == true)
//	    {
//	        /* PATH(IBIT_RamTest,I); */
//	        status = false;
//	        *pAddr = BIT_RAMTEST_PATTERN1;
//
//	        if (*pAddr != BIT_RAMTEST_PATTERN1)
//	        {
//	            /* PATH(IBIT_RamTest,J); */
//	            status = true;
//	        }
//
//	        *pAddr = BIT_RAMTEST_PATTERN2;
//
//	        if (*pAddr != BIT_RAMTEST_PATTERN2)
//	        {
//	            /* PATH(IBIT_RamTest,K); */
//	            status = true;
//	        }
//
//	        *pAddr = BIT_RAMTEST_PATTERN3;
//
//	        if (*pAddr != BIT_RAMTEST_PATTERN3)
//	        {
//	            /* PATH(IBIT_RamTest,L); */
//	            status = true;
//	        }
//
//	        *pAddr = BIT_RAMTEST_PATTERN4;
//
//	        if (*pAddr != BIT_RAMTEST_PATTERN4)
//	        {
//	            /* PATH(IBIT_RamTest,M); */
//	            status = true;
//	        }
//	    }
//
//	    /* shall restore original contents of each location in M0-M1 */
//	    *pAddr = backup;
//
//        if (status == false)
//        {
//            /* PATH(IBIT_RamTest,N); */
//            pAddr++;
//
//            if (pAddr == (unsigned int *)BIT_RAMTEST_MEND)
//            {
//                /* PATH(IBIT_RamTest,O); */
//                pAddr = (unsigned int *)BIT_RAMTEST_LSTART;
//            }
//            else if (pAddr == (unsigned int *)BIT_RAMTEST_LEND)
//            {
//                /* PATH(IBIT_RamTest,P); */
//                pAddr = (unsigned int *)BIT_RAMTEST_HSTART;
//            }
//            else if (pAddr == (unsigned int *)BIT_RAMTEST_HEND)
//            {
//                /* PATH(IBIT_RamTest,Q); */
//                RAM_Test_Started = false;
//                RAM_Test_done = true;
//                i = 10;
//            }
//        }
//        else
//        {
//            /* PATH(IBIT_RamTest,R); */
//            RAM_Test_Started = false;
//            RAM_Test_done = true;
//            i = 10;
//        }
//    }
//
//    lastAddr = pAddr;

    /* shall return true if there is a mismatch (on retest) between contents and test pattern */
	/* PATH(IBIT_RamTest,S); */
    return (status);
}

/*************************************************************************************************\
* Function: BldMgr_VerifyCSCI
*
* Purpose:  Verify integrity of CSCI image with CRC calculation.
*
* Global Input(s):  None
* Global Output(s): None
* Input(s):         ptr_csci_headeraddress - Address of CSCI header structure.
* Output(s):        tCSCI_Status
*                   CSCI_INTEGRITY_CHECK_PASSED - CRC check successful.
*                   CSCI_HEADER_INVALID - Invalid CSCI header.
*                   CSCI_INTEGRITY_CHECK_FAILED - CRC check unsuccessful.
*
\*************************************************************************************************/
tCSCI_Status CSCI_IntegrityCheck(const CSCI_TypeHeader *ptr_csci_headeraddress)
{
    tCSCI_Status result;
    const uint16_t          *ptr_imagestart;
    uint32_t                imagesize;
    uint32_t                crcresult;

    result = CSCI_HEADER_INVALID;

    if (ptr_csci_headeraddress != NULL)
    {
        /* First check the header size field for some assurance that a real header exists */
        if (ptr_csci_headeraddress->HeaderSize != (sizeof(CSCI_TypeHeader) * BYTES_PER_WORD))
        {
            /* Return error code for invalid or uninitialized header */
            result = CSCI_HEADER_INVALID;
        }
        else
        {
            /* Starting point for CRC calculation is the field after the CRC field */

            /*Justification for Coding Standard Deviation:
             *    Cast from pointer to pointer is required here
             *    to allow memory access. TI mechanism of memory access always involves
             *    a cast from x to pointer.
             *    An exception to MISRA Rules 1.2, and 11.4 is required.
             */
            /*lint --e(929) # intended as is */
            /*lint --e(740) # intended as is */
            ptr_imagestart = (const uint16_t *)&ptr_csci_headeraddress->CodeEntryPoint;
            //ptr_imagestart = (const uint16_t *) 0x080004;

            /* Compute the size by taking the image byte size from the header */
            /* and subtracting off 4 bytes to adjust for the 32-bit CRC field  */
            /* that is not included in the CRC calculation */
            imagesize = (ptr_csci_headeraddress->ImageSize - \
                         (sizeof(ptr_csci_headeraddress->CRC) * BYTES_PER_WORD));

            //G_CRC_Configtable = FALSE;
            /* Compute 32-bit CRC of image */
            crcresult = CRC32Api_GetCRC(ptr_imagestart, imagesize);

            /* Compare computed CRC value against CRC value in header and return result */
            if (ptr_csci_headeraddress->CRC != crcresult)
            {

#ifdef SKIP_CRC_CHECK
                result = CSCI_INTEGRITY_CHECK_PASSED;
#else
                result = CSCI_INTEGRITY_CHECK_FAILED;
#endif
            }
            else
            {
                result = CSCI_INTEGRITY_CHECK_PASSED;
            }
        }
    }
    return(result);
}

/* end bitmonitors.c */

