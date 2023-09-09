/****************************************************************************************************
*  File name: motor.h
*
*  Purpose: Interface file for motor control driver
*  This file describes the Public Interface for the motor driver.  The interface provides
*  routines necessary to abstract the details of the control of the motor from the higher-
*  level software blocks.
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

#ifndef MOTOR_H__
#define MOTOR_H__

/*      Include Files
*/
//#include "dsp281x_regs.h"
#include "F2837xD_device.h"
#include "parameter.h"
#include "timer.h"

/*
  Motor Driver
  
  brief The motor control driver is an abstraction for controlling the motor
    via PWM.  Included in this driver is access to the analog signal used
    for monitoring the motor's control signal(s).
*/

/*      Public Type Definitions */
/*-----------------------------------------------------------------------------
Define the structure of the PWM Driver Object 
-----------------------------------------------------------------------------*/

typedef struct
{  
    Uint16 CmtnPointer;     /* Input: Commutation (or switching) state pointer input (Q0) */
    int16 MfuncPeriod;      /* Input: Duty ratio of the PWM outputs (Q15) */
    Uint16 PeriodMax;       /* Parameter: Maximum period (Q0) */
    int16 DutyFunc;         /* Input: PWM period modulation input (Q15) */
} Motor_t;

enum rig_e
{
    DoNotRig = 0,
    RigNow = 1,
    Done = 2
};
typedef enum rig_e HardStopRig_t;

/*
The "PwmActive" setting depends on Power devices. 
when PwmActive = 1 (active high) implies the power device turns ON with a HIGH gate signal. 
     PwmActive = 0 (active low) implies the power device turns ON with a LOW gate signal. 

This polarity definition is not the same as the PWM polarity(Active High/Active Low) 
defined by the ACTRA register in x281x. The setting chosen here is applicable only 
when ACTRA is configured to generate Active High PWM. 
*/

/*-----------------------------------------------------------------------------
Define a PWMGEN_handle
-----------------------------------------------------------------------------*/

/* Public Variable Declarations */

/* Public ROM Constants */

/* Public Defines */
/*----------------------------------------------------------------------------
Initialization constant for the F281X Timer TxCON for PWM Generation. 
Sets up the timer to run free upon emulation suspend, continuous up mode
prescaler 1, timer enabled.
----------------------------------------------------------------------------*/
#define PWM_INIT_STATE  (FREE_RUN_FLAG +          \
                         TIMER_CONT_UP +          \
                         TIMER_CLK_PRESCALE_X_1 + \
                         TIMER_ENABLE +           \
                         TIMER_ENABLE_COMPARE)

/*------------------------------------------------------------------------------
Default Initializers for the F281X PWMGEN Object 
------------------------------------------------------------------------------*/
#define EMB_McuMOTOR {0x0000, /* CmtnPointer                   */ \
                        0x7FFF, /* MfuncPeriod - Duty Ratio 100% */ \
                        0x0000, /* PeriodMax - PWM Period        */ \
                        0x7FFF, /* DutyFunc                      */ }

#define MOTOR_DEFAULTS  EMB_McuMOTOR
#define MAX_DUTY_CYCLE 100  /* maximum percentage duty cycle */
#define MIN_DUTY_CYCLE 0   /* minimum percentage duty cycle */
#define MAX_CAL_ATTEMPTS   3   /* Maxiumum number of calibration attempts */

extern int16 OnCalibratedPos;
extern int16 X_CalibratedPos;
extern bool InCalMode;
extern bool CalTestStart;
extern bool CalRestart;
extern Uint16 CalAttempts;
extern Uint16 CalPhase;
extern Timer_t HardStopTimer;
extern Timer_t CalibrationDelayTimer;
extern Timer_t CalibrationExtendTimer;
extern Timer_t CalibrationRetractTimer;
extern Timer_t CalibrationRESTimer;
extern Timer_t Monitor18Timer;
extern Timer_t CalibrationBrakeTimer;
extern Timer_t BrakeReleaseTimer;
extern Timer_t CalibrationReverseTimer;

/* Public Interface Function Prototypes */

void Motor_Init( Motor_t *p );
void Motor_Update( Motor_t *p );
void Motor_UpdatePWM( Motor_t *p );
void Motor_Stop( Motor_t *p );
void Motor_Capture( void );
void Motor_CalculateCurrent(void);
void Motor_OpenWindings(void);
void Motor_DynamicBrakeWindings(void);


#endif
/* end motor.h */


