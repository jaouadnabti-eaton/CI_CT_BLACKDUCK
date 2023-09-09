/****************************************************************************************************
*  File name: parameter.h
*
*  Purpose: Parameters file for the Real Implementation of Sensored
*  Trapezoidal Drive for a Three Phase Brushless DC Motor (BLDC)
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

#ifndef PARAMETER_H__
#define PARAMETER_H__

#include "typedefs.h"
#include "F2837xD_GlobalPrototypes.h"
//#include "F2837xD_Gpio_defines.h"
#include "F28x_Project.h"
#include "F2837xD_struct.h"
#include "pdi.h"

#define STRINGIFY(sym) #sym

/* PATH(funcname, letter) funcname##_##letter: THREAD_RESULT(STRINGIFY(funcname##_##letter)) */
#define PATH(funcname, letter)

/*      Include Files */

/*      Public Type Definitions */
//typedef enum {false, true} bool;
//typedef float                float32_t;
//typedef long double          float64_t;

/*      Public Variable Declarations */

/*      Public ROM Constants */

/*      Public Defines */
#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#ifndef NULL
#define NULL	0  
#endif

#define PI 3.14159265358979
#define FSL_NUM_POSITIONS   5U  /* 5 Valid Positions to select from with FSL */

/* Define the SYSTEM and TIMER 2 Clock Frequencies  */

/* Define the system frequency (MHz) */
#define SYSTEM_FREQUENCY    200

/* TIMER 2 uses a 20 MHz clock. Divide System Clock 200 MHz by 10 to get 20 MHz */
#define COUNTER_DIVIDER     10

/* WCTA Timer is divided by 20 to yield a 10MHz clock for counting */
#define WCTA_COUNTER_DIVIDER    20


#define COUNTER_FREQUENCY SYSTEM_FREQUENCY * 1000 / COUNTER_DIVIDER /* TIMER 2 Clock Frequency: 20 MHz (15 MHz original VLJ)*/

/* Define the PWM frequency (Hz) - based on Timer 1 */
#define PWM_FREQUENCY_PRI 15425     /* Carrier Frequency for Inverter is 15.425 kHz */
//#define PWM_FREQUENCY_SEC 14590

/* Interrupt Enable Defines */
#define INTERRUPT_DISABLE           0U
#define INTERRUPT_ENABLE            1U

#define TBCTR_FREE_RUN                          2U
#define HZ_TO_KHZ_MULTIPLIER                    1000L       /* Conversion factor Hz to kHz */
#define KHZ_TO_MHZ_MULTIPLIER                   1000L       /* Conversion factor kHz to MHz */
#define SYSCLK_TO_EPWMCLK_NET_DIVIDE_VALUE      2U          /* Net divider value for EPWM Clock from SYSCLK */

/* Actuator Defines */
#define MIN_STROKE_LIMIT_IB                      -0.39000F    /* Inboard Panel Minimum stroke limit in inches:  RSL */
#define MAX_STROKE_LIMIT_IB                       4.62230F    /* Inboard Panel Maximum stroke limit in inches:  ESL */
#define MIN_STROKE_LIMIT_OB                      -0.39000F    /* Outboard Panel Minimum stroke limit in inches:  RSL */
#define MAX_STROKE_LIMIT_OB                       5.73300F    /* Outboard Panel Maximum stroke limit in inches:  ESL */
#define MAX_POS_CMD_DIFF_ALLOWED                    0.020F    /* Maximum difference in position command allowed in VC mode in inches*/

/* Trap errors in skew sensor configuration */
#if (defined(__SKEW_SNSR_RVDT__) && defined(__SKEW_SNSR_ENCODER__))
    #error "MCU cannot be configured to use both an RVDT and an Encoder"
#elif (!defined(__SKEW_SNSR_RVDT__) && !defined(__SKEW_SNSR_ENCODER__))
    #error "The MCU Software must be build with either an RVDT or Encoder selected as the skew sensor."
#endif

/* Motor is different dependent upon hardware being used */
#if defined(DRV8312_DEV_KIT)
    /* 2837xD Development Kit Hardware */

    /* Define the BLDC motor parameters */
    #define RS                  (0.3884405)            /* Stator resistance (ohm) */
    #define RR                  (0)                    /* Rotor resistance (ohm) */
    #define LS                  (0.0006353572)         /* Stator inductance (H) */
    #define LR                  (0)                    /* Rotor inductance (H)  */
    #define LM                  (0.0006353572)         /* Magnetizing inductance (H) */
    #define NUM_POLE_PAIR       (4)                    /* Pole Pairs */
    #define NUM_POLES           (NUM_POLE_PAIR * 2)    /* Number of poles */

    /* Define the base quantities */
//    #define MOTOR_MAX_SPEED_RPM     6000        /* Maximum speed of motor in RPM */
//    #define BASE_SPEED      2835                /* Desired speed to close the loop on while moving actuator. Selected to get same electrical cycle as VLJ motor */
//    #define SEC_PER_MIN     60                  /* Seconds in a minute conversion factor */
//    #define BASE_FREQ       189                 /* Base hall frequency (Hz) = Base speed (RPM)/30 */
//    //#define BASE_FREQ       ((MOTOR_SET_SPEED_RPM * NUM_POLE_PAIR) / SEC_PER_MIN)          /* 189:  Base electrical cycle frequency (Hz) = Base speed (RPM) * electrical cycles per motor revolution / 60 Seconds per Min*/
//    // BASE_RPM = SEC_PER_MIN * BASE_FREQ / NUM_POLE_PAIR

    #define MOTOR_MAX_SPEED_RPM     (4500)            /* Maximum speed of motor in RPM (nameplate is 6000RPM). Chosen at 4500RPM to match base speed between dev kit and MCU target hardware. */

#else /* MCU HW */
    /* 28377D Version 1 Hardware of VLJ */

    /* Same motor as VLJ. May change in future dependent upon plan */
    /* Define the BLDC motor parameters */
    #define RS                  (1.23)                 /* Stator resistance (ohm) */
    #define RR                  (0)                    /* Rotor resistance (ohm) */
    #define LS                  (0.00435)              /* Stator inductance (H) */
    #define LR                  (0)                    /* Rotor inductance (H)  */
    #define LM                  (0)                    /* Magnetizing inductance (H) */
    #define NUM_POLE_PAIR       (5)                    /* Pole Pairs */
    #define NUM_POLES           (NUM_POLE_PAIR * 2)    /* Number of poles */
    #define Ke                  (3.11)                 /* Ke Motor Constant in V/kRPM */

    /* Define the base quantities */
//    /* old Base Speed based on VLJ system */
//    #define BASE_SPEED      5670                /* Desired speed to close the loop on while moving actuator. */
//    #define SEC_PER_MIN     60                  /* Seconds in a minute conversion factor */
//    #define BASE_FREQ       189                 /* Base hall frequency (Hz) = Base speed (RPM)/30 */
//    //#define BASE_FREQ       ((MOTOR_SET_SPEED_RPM * NUM_POLE_PAIR) / SEC_PER_MIN)          /* 189:  Base electrical cycle frequency (Hz) = Base speed (RPM) * electrical cycles per motor revolution / 60 Seconds per Min*/
//    // BASE_RPM = SEC_PER_MIN * BASE_FREQ / NUM_POLE_PAIR
    #define MOTOR_MAX_SPEED_RPM     (10500)//(9000)    /* Maximum speed of motor in RPM */


#endif

/* Kinematics */
#define BALL_SCREW_LEAD                         (0.2F)         /* Ball screw lead in inches for the actuator */
#define GEAR_RATIO                              (1.0F / ((74.0F/12.0F) * (69.0F/14.0F) * (53.0F/16.0F)))    /* Gear Ratio of the Actuator */
#define ELECTRICAL_CYCLES_PER_STROKE_INCH       (NUM_POLE_PAIR / (GEAR_RATIO * BALL_SCREW_LEAD))    /* Number of Electrical Cycles in one stroke inch of actuator movement */
#define STROKE_INCHES_PER_ELECTRICAL_CYCLE      (1.0F / ELECTRICAL_CYCLES_PER_STROKE_INCH)          /* 1 / ELECTRICAL_CYCLES_PER_STROKE_INCH */
//#define ELECTRICAL_CYCLES_PER_STROKE_INCH       2516.908482F  /* 1 / (1/5 [Num Pole Pairs] * 1/100.676339 [Gear Ratio] * 0.2 [Ball Screw Lead] ) */
//#define STROKE_INCHES_PER_ELECTRICAL_CYCLE      0.000397313F  /* 1 / ELECTRICAL_CYCLES_PER_STROKE_INCH */

/* Define the base quantities */
/* speed commands are in the form of percent base speed: 0 to 200%. */
#define MAX_SPEED_COMMAND_FACTOR  (2.0F)  /*  2x, OR 200% is maximum allowable speed command */
#define BASE_SPEED      (5250)            /* Speed to run at with 100% speed command. */
#define SEC_PER_MIN     (60)              /* Seconds in a minute conversion factor */
#define BASE_FREQ       ((BASE_SPEED * NUM_POLE_PAIR) / SEC_PER_MIN)          /* 150:  Base electrical cycle frequency (Hz) = Base speed (RPM) * electrical cycles per motor revolution / 60 Seconds per Min*/
// BASE_RPM = SEC_PER_MIN * BASE_FREQ / NUM_POLE_PAIR


/*      Public Interface Function Prototypes */
extern bool G_bChannelA;        /* Channel A Boolean */
extern eChId_t G_eChId;         /* Channel Identification */
extern eLruId_t G_eLruId;       /* LRU Identification */
extern eActId_t G_eActId;       /* Actuator Identification */
extern eActuator_t G_eActuatorNumber;   /* Actuator Identifier */
extern eActuator_t G_eActuatorNumberX; /* Cross-side Channel Actuator Identifier */
extern tMcuStatus_t tMcuStatus;
extern tMcuFault_t  tMcuFault;
extern tMcuMaint_t  tMcuMaint;
extern t64Types_t   t64Debug;
extern float32_t G_pwmMax;
extern t64Types_t t64SpiSendCounters1;
extern t64Types_t t64SpiSendCounters2;
extern t64Types_t t64SpiRecvCounters1;
extern t64Types_t t64SpiRecvCounters2;
#endif
/* end parameter.h */


