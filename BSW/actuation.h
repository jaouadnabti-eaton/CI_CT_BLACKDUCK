/****************************************************************************************************
*  File name: actuation.h
*
*  Purpose: The Actuation Manager handles the primary feature of the MCU, controlling flap actuation.
*  This file describes the Public Interface for the Actuation Manager.  It provides the
*  routines necessary to operate the flap actuation from the necessary scheduler timeslices.
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

#ifndef ACTUATION_H__
#define ACTUATION_H__

/*      Include Files
*/
#include "parameter.h"
#include "hallsensor.h"
#include "timer.h"
#include "motor.h"

/**
  defgroup actuation Actuation Manager
  
  brief The Actuation Manager handles the primary feature of the MCU, controlling flap actuation.
*/


/*      Public Type Definitions
*/
/* brief Container for Commands to motor control algorithm*/
typedef struct
{
    int     MotorDirection;  /* CW = -1, CCW = 1 (defined below)*/
    bool    MotorStop;       /* Command to full stop*/
    bool    MotorStart;      /* Command to begin turning motor*/
    float32 MotorSpeed;      /* Target motor speed, as a fraction of 2 (200%), for speed control*/
    int16   rsvd1;            /* reserved*/
	bool    rsvd2;            /* reserved*/
    int     StopPosition;    /* Position where motor should stop for discrete steps*/
    Uint16  IllegalHalls;    /* Maintains count of "illegal" Hall sensor states detected*/
    HallState_t  LastBadHallState;/* Maintains specific state of last "illegal" Hall sensor state detected*/
    Uint16  BadTransitions;  /* Maintains count of "bad" Hall sensor state transitions detected*/
    bool    BrakeActivated;  /* Status: the brake has been activated*/
    bool    MotorRunning;    /* Status: the motor is being commanded to turn*/
    bool    Rigging;         /* rigging*/
    bool    rsvd3;            /* reserved*/
    bool    RiggingHardStop; /* rigging hardstop*/
    bool    BrakeCooling;    /* brake is in cool-down state*/
    bool    ManualBrkRls;    /* manually release the brake*/
    uint16_t u16BrakeDutyCycle; /* duty cycle commanded to brake PWM_RST_N pin */
    float32_t f32StopPosition;  /* Stop Position in stroke inches */
} Motor_Commands_t;

/**
 *  brief The Speed computation object
 *
 *  The speed compuation object is a container for all variables and methods relating to the
 *  computation of motor speed.  Included are time measurements and computed RPMs.
*/ 
typedef struct
{
    Uint32 NewTimeStamp;     /* Variable : New 'Timestamp' corresponding to a capture event (Q0) - independently with global Q */
    Uint32 OldTimeStamp;     /* Variable : Old 'Timestamp' corresponding to a capture event (Q0) - independently with global Q */
    Uint32 TimeStamp;        /* Input : Current 'Timestamp' corresponding to a capture event (Q0) - independently with global Q */
    int16 EventCount;        /* Input/Variable :  Event Count (Q0) */
    Uint32 SpeedScaler;      /* Parameter :  Scaler converting 1/N cycles to a GLOBAL_Q speed (Q0) - independently with global Q*/
    Uint32 EventPeriod;      /* Input/Variable :  Event Period (Q0) - independently with global Q*/
    uint32_t u32ECapPeriod;  /* ECAP CAP counts */
    _iq Speed;               /* Output :  speed in per-unit*/
    Uint32 BaseRpm;          /* Parameter : Scaler converting GLOBAL_Q speed to rpm (Q0) speed - independently with global Q*/
    Uint32 BaseHps;          /* Parameter : Scaler converting GLOBAL_Q speed to halls per millisecond (Q0) speed - independently with global Q*/
    int32 SpeedRpm;          /* Output : speed in r.p.m. (Q0) - independently with global Q*/
    int32 SpeedHps;          /* Output : speed in Halls per second (Q0) - independently with global Q*/
    int RampPosition;        /**/
    bool RampingUp;          /* Flag:  true if motor is to accelerate*/
    bool RampingDown;        /* Flag:  true if motor is to decelerate*/
    void (*calc)();          /* Pointer to the calculation function */
} Speed_t; 

/*      Public Variable Declarations
*/
extern Motor_t tPwm;
extern Motor_Commands_t MotorCmd;   /* Contains commands and status for controlling the motor*/
extern Speed_t tSpeed;              /* Speed computation and monitoring object for the on-side motor*/
extern float32_t f32SpeedRef;       /* Reference Speed for PID Regulator */
extern uint16_t u16EventPos_SpdFilter;
#if defined(__HALLX_CONFIGURED)
extern Speed_t tSpeedx;              /* Speed computation and monitoring object for the cross-side motor*/
#endif

extern Timer_t MotorTimer;
extern int16_t ResetIntegralGain ;
extern uint16_t u16EventStart_SpdFilter;


/*      Public ROM Constants
*/

/*      Public Defines
*/
#define CW -1                           /* Clockwise */
#define CCW 1                           /* Counter Clockwise */
#define SPD_WINDOW_LENGTH 30            /* Window length to calculate moving average of speed */
#define SPD_WINDOW_LENGTH_PLUSONE 31    /* Window length plus one */


#define NO_STOP -32768
extern uint32_t u32EventRaw_SpdFilter[SPD_WINDOW_LENGTH];

/*      Public Interface Function Prototypes
*/

void Act_Init( void );
void Act_MeasureSpeed( void );
void Act_DoPid( void );
void Act_DoControl( void );
void Act_Ramp( void );
void Act_CommutationCallback( void );
void Act_StopMotor(void);
void Act_SetP(_iq);



#endif
/* end actuation.h*/

