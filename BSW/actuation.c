/****************************************************************************************************
*  File name: actuation.c
*
*  Purpose: The Actuation Manager handles the primary feature of the MCU, controlling flap actuation.
*  This file provides the routines (public and private) necessary to operate the flap actuation from
*  the scheduler timeslices.
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

/*      Include Files
*/
//#include "dsp281x.h"
//#include "dsp281x_regs.h"
#include "F2837xD_device.h"
#include "IQmathLib.h"
#include "parameter.h"
#include "actuation.h"
#include "motor.h"
//#include "gse.h"
#include "brake.h"
//#include "panel.h"
//#include "rvdt.h"
#include "hallsensor.h"
#include "gpio.h"
#include "timer.h"
#include "events.h"
#include "mcu.h"
#include "adc.h"
#include "bitmonitors.h"
//#include "nvm.h"
//#include "spi.h"
#include "ASW_BSW_I.h"
#include "PID_Loop.h"
#include "OFP.h"

/*      Local Type Definitions
*/

/*      Container type for ramp calculation for use in speed control algorithm*/
typedef struct
{
    _iq    TargetValue;    /* Input: Target input (pu)                               */
    _iq    SetpointValue;  /* Output: Target output (pu)                             */       
    Uint32 EqualFlag;      /* Output: Flag output (Q0) - independently with global Q */
    void (*calc)();        /* Pointer to calculation function                        */
} speedramp_t;     

typedef speedramp_t *speedramp_handle;

/*  Container type for PID algorithm calculations*/
typedef struct
{
    _iq  Ref;        /* Input: Reference input                */ 
    _iq  Fdb;        /* Input: Feedback input                 */
    _iq  Err;        /* Variable: Error                       */
    _iq  Kp;         /* Parameter: Proportional gain          */
    _iq  Up;         /* Variable: Proportional output         */
    _iq  Ui;         /* Variable: Integral output             */ 
    _iq  Ud;         /* Variable: Derivative output           */      
    _iq  OutPreSat;  /* Variable: Pre-saturated output        */
    _iq  OutMax;     /* Parameter: Maximum output             */
    _iq  OutMin;     /* Parameter: Minimum output             */
    _iq  Out;        /* Output: PID output                    */
    _iq  SatErr;     /* Variable: Saturated difference        */
    _iq  Ki;         /* Parameter: Integral gain              */
    _iq  Kc;         /* Parameter: Integral correction gain   */
    _iq  Kd;         /* Parameter: Derivative gain            */
    _iq  Up1;        /* History: Previous proportional output */
    void  (*calc)(); /* Pointer to calculation function       */
} pid_t;       

typedef pid_t *pid_handle;
typedef Speed_t *Speed_handle;

int16_t ResetIntegralGain = 0;

/*      Local Defines
*/

/* Default Initializer for the Speed Object*/
#define SPEED_DEFAULTS { 0,                              /* NewTimeStamp */ \
                         0,                              /* OldTimeStamp */ \
                         0,                              /* TimeStamp    */ \
                         0,                              /* EventCount   */ \
                         0,                              /* SpeedScaler  */ \
                         0,                              /* EventPeriod  */ \
                         0,                              /* ECAP CAP Period  */ \
                         0,                              /* Speed        */ \
                         0,                              /* BaseRpm      */ \
                         0,                              /* BaseHps      */ \
                         0,                              /* SpeedRpm     */ \
                         0,                              /* SpeedHps     */ \
                         0,                              /* RampPosition */ \
                         false,                          /* RampingUp    */ \
                         false,                          /* RampingDown  */ \
                         (void (*)(Uint32))compute_speed /* *calc        */ }


/* Default Initializer for the Speed Ramp Object*/
#define SPEEDRAMP_DEFAULTS { 0,                               /* TargetValue   */ \
                             0,                               /* SetpointValue */ \
                             0,                               /* EqualFlag     */ \
                             (void (*)(Uint32))speedramp_calc /* *calc         */ }


/* Default Initializer for the PID Object*/
#define PID_DEFAULTS { 0,                         /* Ref       */ \
                       0,                         /* Fdb       */ \
                       0,                         /* Err       */ \
                       0,                         /* Kp        */ \
                       0,                         /* Up        */ \
                       0,                         /* Ui        */ \
                       0,                         /* Ud        */ \
                       0,                         /* OutPreSat */ \
                       0,                         /* OutMax    */ \
                       0,                         /* OutMin    */ \
                       0,                         /* Out       */ \
                       0,                         /* SatErr    */ \
                       0,                         /* Ki        */ \
                       0,                         /* Kc        */ \
                       0,                         /* Kd        */ \
                       0,                         /* Up1       */ \
                       (void (*)(Uint32))pid_calc /* *calc     */ }


/* Default Initializer for the Motor State*/
#define MOTORCMD_DEFAULTS { CW,      /* MotorDirection */ \
                            true,    /* MotorStop      */ \
                            false,   /* MotorStart     */ \
                            0.0,     /* MotorSpeed     */ \
                            0,       /* rsvd1           */ \
                            false,   /* rsvd2           */ \
                            NO_STOP, /* StopPosition   */ \
                            0,       /* IllegalHalls   */ \
							GOOD_HALLS,/* LastBadHallState*/\
                            0,       /* BadTransitions */ \
                            false,   /* BrakeActivated */ \
                            false,   /* MotorRunning   */ \
                            false,   /* Rigging        */ \
                            false,   /* rsvd3           */ \
                            false,   /* RiggingHardStop*/ \
                            false,   /* BrakeCooling   */ \
                            false,   /* ManualBrkRls   */ \
                            0,       /* u16BrakeDutyCycle */ \
                            NAN      /* f32StopPosition */ }

float32_t f32SpeedRef = 0.0;
void compute_speed( Speed_t *v );
void speedramp_calc( speedramp_t *v );
void pid_calc( pid_t *v );
void Average(Speed_t *v);
/*      Global Variables
*/

Motor_Commands_t MotorCmd = MOTORCMD_DEFAULTS;
Timer_t MotorTimer = TIMER_DEFAULTS;

/*      Local ROM Constants
*/

/*      Local Variable Declarations
*/

/* Instance PID regulator to regulate the DC-bus current and speed*/
 //pid_t tPidSpeed = PID_DEFAULTS;

/* Instance a PWM driver instance*/
 Motor_t tPwm = MOTOR_DEFAULTS;

/* Instance a SPEED_PR Module*/
 Speed_t tSpeed = SPEED_DEFAULTS;

#if defined(__HALLX_CONFIGURED)
/* Instance a SPEED_PR Module for xside*/
 Speed_t tSpeedx = SPEED_DEFAULTS;
#endif

/* Instance a ramp controller to smoothly ramp the frequency*/
 speedramp_t tRamp = SPEEDRAMP_DEFAULTS;

 /*Array to hold 30 samples (1 Mechanical Cycle) of Hall Periods*/
 uint32_t u32EventRaw_SpdFilter[SPD_WINDOW_LENGTH] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
                                                    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
                                                    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};

 /*Variable to hold current index of the Array - u32EventRaw_SpdFilter[]*/
 uint16_t u16EventPos_SpdFilter = 0;

 /*Variable to handle the speed filtering when speed is increases from 0*/
 uint16_t u16EventStart_SpdFilter = 0;




/*      Local Function Prototypes
*/

/*      Function Definitions
*/


/**
    brief Initialization routine for Actuation manager.

     Purpose:
        This routine sets up memory space for, and initializes the algorithm(s) of, the
        Actuation Manager component.

     Global Data Referenced:
        #tHall 
        #tHallx
    
    return  void
    
     Preconditions and Assumptions:
        This routine should be called only once, prior to main scheduler loop.

*/
void Act_Init( void )
{
    /* PATH(Act_Init,A); */
    
    /* shall call Motor_Init to initialize PWM module*/
    Motor_Init(&tPwm);

    /* shall initialize the on-side motor's speed object (speed1) by setting BaseRPM and 
    SpeedScaler based on Max Motor RPM and Counter Frequency*/
    tSpeed.BaseRpm = (Uint32)(BASE_SPEED);
    /* Base Hall Frequency (halls/sec at set RPM) = BASE_FREQ (electrical cycle frequency) * 6 hall edges per electrical cycle */
    tSpeed.BaseHps = (Uint32)(BASE_FREQ) * 6;
    /* scaler to convert counts per hall edge to actual RPM 
     actual RPM = SpeedScaler/(clock counts for hall edge width))*/  
    tSpeed.SpeedScaler = (Uint32)((1000L * COUNTER_FREQUENCY) / (6L * BASE_FREQ));
    /* COUNTER_FREQUENCY = 15MHz */
#if defined(__HALLX_CONFIGURED)
    /* shall initialize the cross-side motor's speed object (speedx) by setting BaseRPM and
       SpeedScaler the same as the on-side motor's speed object*/
    tSpeedx.BaseHps = tSpeed.BaseHps;
    tSpeedx.BaseRpm = tSpeed.BaseRpm;
    tSpeedx.SpeedScaler = (tSpeed.SpeedScaler * 6);
#endif

#if !defined(__HALLX_CONFIGURED)
    /* shall call Hall_Init to initialize the hall object*/
    Hall_Init(&tHall); /* Initialize onside Halls*/
#else
    /* shall call Hall_Init to initialize the on-side and cross-side hall object*/
    Hall_Init(&tHall, &tHallx); /* Initialize onside Halls*/
#endif

    /*Shall set Bad Transition counter to 0 as this counter is incremented to 15 during Initialization*/
    MotorCmd.BadTransitions = 0U;
}


/**
    brief Executes control algorithm for Actuation Manager.

     Purpose:
        Handles determining commutation state and commanding the motors to control
        movement of the flap(s).  

     Global Data Referenced:
        #EvaRegs 
        #GpioDataRegs 
        #MotorCmd 
        #tHall
        #tPidSpeed
    
    return  void
    
     Preconditions and Assumptions:
         This function should be called from the scheduler, at a 1kHz rate

*/
void Act_DoControl( void )
{
    /*  PATH(Act_DoControl,A); */ 
    ResetIntegralGain = 1;
    
    /* shall set PWM Duty Cycle to the PID's Output*/
    //tPwm.DutyFunc = (int16)_IQtoIQ15(tPidSpeed.Out); /* controlled Speed duty-cycle      */
    //tPwm.DutyFunc = (int16) (32768.0 * tPidSpeed.Out); /* controlled Speed duty-cycle      */
    
    tPwm.DutyFunc = (int16) (32768.0 * G_ASW_DATA.tAnalogOutputData.f32SpeedPidOutput);

    /* shall call the Act_StopMotor function to actively stop the motor if commanded*/
    if ((MotorCmd.MotorStop == true) || (bInhibitFlap == true)) /* STOP MOTOR */
    {
        /* PATH(Act_DoControl,B); */
        Act_StopMotor();
    }
    else if (MotorCmd.MotorStart == true) /* STARTING MOTOR*/
    {
        /* PATH(Act_DoControl,C); */          
        if (MotorCmd.MotorRunning == false) /* Release brake and start timer*/
        {
            /* PATH(Act_DoControl,D); */
            ResetIntegralGain = 0;
			//tPidSpeed.Ui = _IQ(0.00);

			//PID_Loop_initialize(&(OFP_DW.ASW_InstanceData.rtdw.SpeedRegulator_InstanceData.rtdw));

            switch (McuGetState())
            {
                case RUN_MODE:
                    /* PATH(Act_DoControl,E); */
                case RIG_MODE:
                    /* PATH(Act_DoControl,F); */
                case TEST_MODE:
                    /* PATH(Act_DoControl,H); */

                    /* shall prepare to start the motor when commanded by activating the brake and starting the brake activation delay timer*/
                    Brake_Activate();
                    Timer_SetTime(&MotorTimer, TIMER_ONESHOT, TIMER_20ms);
                    MotorCmd.MotorRunning = true;
                    break;
                case BOOT_MODE:
                    /* PATH(Act_DoControl,G); */
                    Timer_SetTime(&MotorTimer, TIMER_ONESHOT, TIMER_1ms);
                    MotorCmd.MotorRunning = true;
                    break;
                default:
                    /* PATH(Act_DoControl,M); */
                    break;
            }

            Timer_SetTime(&Bit_PhNeuDelayTimer, TIMER_ONESHOT, 6 * TIMER_100ms);        /*Setting a 600Ms timer when Motor is commanded to RUN*/
                                                                                        /*This timer is used for fault monitor 2A, Ph-Neutral Assymetrical short*/
            Timer_SetTime(&Bit_OpCktDelayTimer, TIMER_ONESHOT, 2 * TIMER_100ms);        /*Setting a 200Ms timer when Motor is commanded to RUN*/
                                                                                        /*This timer is used for fault monitor 2B*/
        }
        else if (Timer_IsExpired(&MotorTimer) == true) /* After 20ms start motor (or 100ms creep monitor time) */
        {
            /* Disable ECAP interrupts to avoid corrupted commutation */
            PieCtrlRegs.PIEIER4.all = (PieCtrlRegs.PIEIER4.all & HALL_ECAP_ISR_DISABLE_MASK);
            /* Wait 5 cycles to service any ECAP Interrupts that may have been propogating during the write */
            asm (" RPT #5 || NOP");

            /* shall determine current commutation state of motor after the brake activation timer expires*/
            tHall.HallGpio = ((GpioDataRegs.GPADAT.all & HALL_MASK) >> HALL_SHIFT); /* HallGpio.2-0 = GPIO8-GPIO6*/
            tHall.HallGpioBuffer = tHall.HallGpio; /* Previous state now equals current state*/
            tHall.HallGpioDblBuffer = tHall.HallGpio; /* Previous state now equals current state*/

            /* Commutate the motor per latest read hall state */
            Act_CommutationCallback();
            /* Re-Enable the ECAP interrupts */
            PieCtrlRegs.PIEIER4.all = (PieCtrlRegs.PIEIER4.all | HALL_ECAP_ISR_ENABLE_MASK);

            MotorCmd.MotorStart = false;

        }
    }

    /* shall update the PWM signals to the motors*/
    Motor_UpdatePWM(&tPwm);
    
    /* PATH(Act_DoControl,R); */
}

/**
    brief Gets input from Hall sensors to measure motor speed

     Purpose:
        This routine is responsible for measuring the speed at which the motor is turning.  Speed
        is computed for both the on-side and cross-side motors.

     Global Data Referenced:
         None.
    
    return  void
    
     Preconditions and Assumptions:
         This function should be called from the scheduler, at a 1kHz rate, to ensure proper
        timing of internal counters.
*/
void Act_MeasureSpeed( void )
{
    static int s16MeasureSpeed = 0;
#if defined(__HALLX_CONFIGURED)
    static int s16MeasureSpeedx = 0;
#endif

    /* PATH(Act_MeasureSpeed,A); */
#if defined(__HALLX_CONFIGURED)
    //if (s16MeasureSpeedx < 24)
    //{
        /* PATH(Act_MeasureSpeed,C); */
        s16MeasureSpeedx++;
    //}
#endif
    
    /*need to disable hall interrupt here*/
    DINT;
    /* shall increment local counters to measure time out*/
    s16MeasureSpeed++;
    /* shall compute the speed of on-side motor after every 1 Hall count.*/
    /* Added timeout for 500ms.*/
    if ((tSpeed.EventCount > 0)&&(s16MeasureSpeed <= 500))
    {
        tSpeed.calc(&tSpeed);
        s16MeasureSpeed = 0;
    }
    else if(s16MeasureSpeed > 500)
    {
        tSpeed.calc(&tSpeed);
        s16MeasureSpeed = 0;
    }

#if defined(__HALLX_CONFIGURED)
    /* shall compute the speed of cross-side motor when EventCount is one or 75 mS, whichever comes first.*/
    if ((tSpeedx.EventCount == 1) && (s16MeasureSpeedx >= 75))
    {
        /* PATH(Act_MeasureSpeed,G); */
        s16MeasureSpeedx = 0;
        tSpeedx.EventCount = 0;
        tSpeedx.calc(&tSpeedx);
    }
    else if (tSpeedx.EventCount == 1)
    {
        /* PATH(Act_MeasureSpeed,H); */
        tSpeedx.calc(&tSpeedx);
        s16MeasureSpeedx = 0;
        tSpeedx.EventCount = 0;
    }
    else if (s16MeasureSpeedx >= 75)
    {
        /* PATH(Act_MeasureSpeed,I); */
        tSpeedx.EventCount = 0;
        tSpeedx.calc(&tSpeedx);
    }
#endif
    /* need to enable hall interrupt here*/
    EINT;

    /* PATH(Act_MeasureSpeed,J); */
}

/**
    brief Executes PID algorithm

     Purpose:
        This function is responsible for feeding the motor speed into the PID function to control
                the motor velocity.

     Global Data Referenced:
         None.
    
    return  void
    
     Preconditions and Assumptions:
         None.
*/
void Act_DoPid( void )
{
    //static _iq pwmMax = _IQ(0.99);
    int16 delta = 0;
    float32_t f32Delta = 0.0F;

    STATE_ID tFscuMode = McuGetState();

    /* PATH(Act_DoPid,A); */

    f32SpeedRef = tRamp.SetpointValue;
    
    /* Test if in normal mode or not */
    if(tFscuMode == RUN_MODE)
    {
        /* Normal mode closes loop on the output of sensor fusion */

        /* calculate position delta based on the motor direction*/
        if (MotorCmd.MotorDirection == CW)
        {
            f32Delta = MotorCmd.f32StopPosition - G_ASW_DATA.tPanelOutputData.f32StrokeFused;
        }
        else if (MotorCmd.MotorDirection == CCW)
        {
            f32Delta = G_ASW_DATA.tPanelOutputData.f32StrokeFused - MotorCmd.f32StopPosition;
        }
        if (f32Delta <= 0.0F)
        {
            f32SpeedRef = _IQ(0.0);
            MotorCmd.MotorStop = true;
        }
    }
    else
    {
        /* Not in normal mode. Close loop on position of hall sensor */
        if (MotorCmd.StopPosition != NO_STOP)
        {
            /* shall calculate position delta based on the motor direction*/
            if (MotorCmd.MotorDirection == CW)
            {
                /* PATH(Act_DoPid,C); */
                delta = MotorCmd.StopPosition - tHall.Position;
            }
            else if (MotorCmd.MotorDirection == CCW)
            {
                /* PATH(Act_DoPid,D); */
                delta = tHall.Position - MotorCmd.StopPosition;
            }

            if ((delta <= 0) || (McuGetState() == IDLE_MODE))
            {
                /* PATH(Act_DoPid,E); */
                f32SpeedRef = _IQ(0.0);
                MotorCmd.MotorStop = true;
            }
        }
    }

    /* shall connect inputs of the PID object to the Ramp and Speed outputs*/
    //tPidSpeed.Ref = f32SpeedRef;  /* Reference Speed = Ramped speed setpoint*/
    //tPidSpeed.Fdb = tSpeed.Speed; /* Feedback Speed = measured speed*/

    /*  shall call the PID calculate function to determine the commanded PWM to 
         control the motor's speed.*/
    //tPidSpeed.OutMax = pwmMax;
    //tPidSpeed.calc(&tPidSpeed);    /* Do the PID calculation.*/

    /*  shall check motor current and update current loop*/
    /* PATH(Act_DoPid,F); */
    // Need to add MCU logic for current limiting once defined.
//	if (((I_ANLG_raw > CURRENT_LIMIT_NORMAL) && (bitStatus[0x61].tripBoot == 0)) ||
//	    ((I_ANLG_raw > CURRENT_LIMIT_REDUCE) && (bitStatus[0x61].tripBoot != 0)))
//    if (0)
//	{
//		/* PATH(Act_DoPid,G); */
//        pwmMax = pwmMax - _IQ(0.0025);
//	}
//    // Need to revist when current limiting is defined.
////	else if (((I_ANLG_raw < CURRENT_LIMIT_NORMAL) && (bitStatus[0x61].tripBoot == 0)) ||
////	         ((I_ANLG_raw < CURRENT_LIMIT_REDUCE) && (bitStatus[0x61].tripBoot != 0)))
//    else
//	{
//		/* PATH(Act_DoPid,H); */
//
//		if (pwmMax < _IQ(0.01))
//		{
//			/* PATH(Act_DoPid,I); */
//			pwmMax = _IQ(0.01);
//		}
//		else
//		{
//			/* PATH(Act_DoPid,J); */
//			pwmMax = pwmMax + _IQ(0.00125);
//		}
//	}
//
//    if (pwmMax > _IQ(0.99))
//    {
//        /* PATH(Act_DoPid,K); */
//        pwmMax = _IQ(0.99);
//    }
//    else if (pwmMax < _IQ(0.00))
//    {
//        /* PATH(Act_DoPid,L); */
//        pwmMax = _IQ(0.00);
//    }
//
//    if (tPidSpeed.Out > pwmMax)
//    {
//        /* PATH(Act_DoPid,M); */
//        tPidSpeed.Out = pwmMax;
//    }
//    G_pwmMax = pwmMax;

    /* PATH(Act_DoPid,N); */
}


/**
    brief Handles acceleration/deceleration of motor speed.

     Purpose:
        Computes controlled acceleration/deceleration ramp at beginning and
        end of flap travel.

     Global Data Referenced:
        #MotorCmd 
        #tRamp 
        #tHall 
        #tHallx
    
    return  void
    
     Preconditions and Assumptions:
         None.

*/
void Act_Ramp( void )
{
    /* Target Value is MotorSpeed since there is no cross channel motor following this motor */
    tRamp.TargetValue = _IQ(MotorCmd.MotorSpeed);

    /* shall call the Ramp calculate routine to implement a smooth ramp */
    tRamp.calc(&tRamp);
    /* PATH(Act_Ramp,D); */
}


/**
    brief Signals a commutation state change to Actuation Manager.

     Purpose:
        Intended to be called from Hall Sensor interrupt.

     Global Data Referenced:
        #MotorCmd 
        #tHall    
        #tPwm     
        #Rigging_Status 
        #Nvm_Rigging_Temp

    return  void
    
    Preconditions and Assumptions:
         Only to be called from the Hall interrupt service routine when an edge is detected.

*/
void Act_CommutationCallback( void )
{
    /* PATH(Act_CommutationCallback,A); */
    
    /* shall set the PWM objects stored Commutation pointer to the current hall state
        if the motor is not commanded to be stopped.*/
#if 0 
/*As per Motor configuration same table will not work for CCW
so disabling the CCW part and a new CCW table implemented in motor.c*/
    if (MotorCmd.MotorDirection == CCW)
    {
        /* PATH(Act_CommutationCallback,B); */

        /* Counter Clockwise*/
        /* shall invert the Hall Signals if the commanded motion is counter-clockwise*/
        tPwm.CmtnPointer = (tHall.HallGpio ^ 0x0007); /* Invert bits*/
    }
    else
#endif
    {
        /* PATH(Act_CommutationCallback,C); */

        /* Clockwise*/
        tPwm.CmtnPointer = tHall.HallGpio;
    }

    //if (McuGetState() != INITIALIZATION_MODE)
    {
        /* PATH(Act_CommutationCallback,D); */

        /* shall call the Motor_Update routine to commutate the Motor*/
        Motor_Update(&tPwm);
    }

    /* PATH(Act_CommutationCallback,E); */
}


/**
    brief Stops the Motor

     Purpose:
        Turns off all PWM outputs to the motor drive circuit.

     Global Data Referenced:
        #tPwm
    
    return  void
    
     Preconditions and Assumptions:
         None.

*/
void Act_StopMotor(void)
{
    /* PATH(Act_StopMotor,A); */

    /* shall call the Motor_Stop routine to command the motor to stop.*/
    Motor_Stop(&tPwm);

    /* PATH(Act_StopMotor,B); */
}

/**
    brief Computes speed values from hall sensor inputs.

    Purpose:
        Computes speed values from local and cross-channel hall sensor inputs. To be
        invoked regularly from scheduler.

    param[in] v    Pointer to Speed_t speed computation object

     Global Data Referenced:
         None.
    
    return  void
    
     Preconditions and Assumptions:
        Designed for regular invocation by scheduler.  This function has a no particular
        timing dependencies.  The speed computation is based on CPU timing, not scheduler
        timing.

*/
void compute_speed( Speed_t *v )
{

    uint16_t i = 0;
    /* PATH(compute_speed,A); */
    /* shall ensure that 'v' pointer is not null */
    if (v)
    {	
    	/* PATH(compute_speed,E); */
        if(v->EventCount > 0)
        {
            /* PATH(compute_speed,B); */
            /* shall set the Speed Object's speed input to a percentage of maximum speed*/
            Average(&tSpeed);
            v->Speed = _IQdiv(v->SpeedScaler,v->EventPeriod); /* Speed as a percentage of Max*/
            /* shall set the Speed Object's Halls per sec input to the calculated Halls per second*/
            v->SpeedHps = _IQmpy(v->BaseHps,v->Speed); /* Halls per second */
            /* shall set the Speed Object's RPM input to the calculated RPM*/
            v->SpeedRpm = _IQmpy(v->BaseRpm,v->Speed);
            v->EventCount = 0;
//            prevEventCount = v->EventCount;
        }
        else
        {
            /* shall set the Speed Object's speed and RPM inputs to 0 if no hall edges
            were detected since last call*/
            v->SpeedRpm = 0;
            v->Speed = 0;

            /*If speed is set to zero then the elements of the array become absolute and hence need to reset them to 0*/
            for (i = 0; i < SPD_WINDOW_LENGTH; i++)
            {
                u32EventRaw_SpdFilter[i] = 0U;
            }
            u16EventPos_SpdFilter = 0;                                  /*Reset index of array to 0 as elements of array are reset*/
            u16EventStart_SpdFilter = 0;                                /*Reset the variable as u32EventRaw_SpdFilter[] array will have no valid elements*/

        }
    }
    /* PATH(compute_speed,D); */
}

void Average(Speed_t *vspeed)
{
    uint32_t avg = 0;
    uint16_t i = 0;

    /*if all the elements of array u32EventRaw_SpdFilter[] are valid
     * then add all elements and divide the sum with the Window Length*/
    if (u16EventStart_SpdFilter >= SPD_WINDOW_LENGTH_PLUSONE)
    {
        for(i = 0; i < SPD_WINDOW_LENGTH ; i++)
        {
            avg += u32EventRaw_SpdFilter[i];
        }
        vspeed->EventPeriod = (avg/SPD_WINDOW_LENGTH);
    }
    else
    {
        /*if all the elements of array u32EventRaw_SpdFilter[] are not valid
         * then add only elements that are valid and divide the sum with the Total number of valid elements*/
        for(i =0; i < (u16EventStart_SpdFilter - 1U); i++)
        {
            avg += u32EventRaw_SpdFilter[i];
        }
        vspeed->EventPeriod = (avg/((u16EventStart_SpdFilter)-1U));

    }

}
/*===========================================================*/

/**
    brief Ramps the commanded speed linearly

     Purpose:
        This routine ramps the Speed Target either positively or negatively 
                in a linear fashion.

     Global Data Referenced:
         None.
    
    return  void
    
     Preconditions and Assumptions:
        none

*/
void speedramp_calc(speedramp_t *v)
{
    //static _iq RPM_Limit = _IQ(1.0220575);  /* 2.2% above BASE_SPEED */
    //static _iq Ramp = _IQ(0.0044115);  /* 0.44115% RPM Ramp:  % of BASE_SPEED */
    static _iq Ramp = _IQ(0.002777);  /* 25RPM Ramp = 25/9000 = 0.002777% of MAX SPEED 9,000RPM */

    /* PATH(speedramp_calc,A); */

	/* shall ensure that 'v' pointer is not null */
    if (v)
    {	
        /* PATH(speedramp_calc,H); */
        
	    if (v->TargetValue > _IQ(MAX_SPEED_COMMAND_FACTOR))
	    {
	        /* PATH(speedramp_calc,B); */
	        v->TargetValue = _IQ(MAX_SPEED_COMMAND_FACTOR);
	    }

	    if (v->TargetValue < _IQ(0.0))
	    {
	        /* PATH(speedramp_calc,C); */
	        v->TargetValue = _IQ(0.0);
	    }

        if (v->SetpointValue < v->TargetValue)
        {
            /* Need to apply acceleration ramp */
            if ((v->SetpointValue + Ramp) < v->TargetValue)
            {
                /* PATH(speedramp_calc,D); */
                v->SetpointValue += Ramp;
            }
            else
            {
                /* PATH(speedramp_calc,E); */
                v->SetpointValue = v->TargetValue;
            }
        }
        else if(v->SetpointValue > v->TargetValue)
	    {
	        /* Need to apply deceleration ramp */
            if ((v->SetpointValue - Ramp) > v->TargetValue)
            {
                /* PATH(speedramp_calc,D); */
                v->SetpointValue -= Ramp;
            }
            else
            {
                /* PATH(speedramp_calc,E); */
                v->SetpointValue = v->TargetValue;
            }
	    }

	    if (v->SetpointValue < _IQ(0.0))
	    {
	        /* PATH(speedramp_calc,F); */
	        v->SetpointValue = _IQ(0.0);
	    }
        /* PATH(speedramp_calc,I); */
	}
    /* PATH(speedramp_calc,G); */
}


/**
    brief Implements the PID algorithm

    Purpose:
        This routine Calculated the PWM Duty cycle via a PID algorithm

     Global Data Referenced:
         None.
    
    return  void
    
     Preconditions and Assumptions:
        none

*/
void pid_calc( pid_t *v)
{
    /* PATH(pid_calc,A); */

    /*shall implement a PID algorithm with anti-windup compensation per the following:
    
     image html pid_img.png "Figure: Block Diagram of PID controller with anti-windup"
     htmlinclude pid.htm*/

	/* shall ensure that 'v' pointer is not null */
//    if (v)
//    {
//	    /* PATH(pid_calc,F); */
//
//	    /* Compute the error*/
//	    v->Err = v->Ref - v->Fdb;
//	    /* Compute the proportional output*/
//	    v->Up = _IQmpy(v->Kp,v->Err);
//	    /* Compute the integral output*/
//	    v->Ui = v->Ui + _IQmpy(v->Ki,v->Up) + _IQmpy(v->Kc,v->SatErr);
//	    /* Compute the derivative output*/
//	    v->Ud = _IQmpy(v->Kd,(v->Up - v->Up1));
//	    /* Compute the pre-saturated output*/
//	    v->OutPreSat = v->Up + v->Ui + v->Ud;
//
//	    /* Saturate the output*/
//	    if (v->OutPreSat > v->OutMax)
//	    {
//	        /* PATH(pid_calc,B); */
//	        v->Out = v->OutMax;
//	    }
//	    else if (v->OutPreSat < v->OutMin)
//	    {
//	        /* PATH(pid_calc,C); */
//	        v->Out = v->OutMin;
//	    }
//	    else
//	    {
//	        /* PATH(pid_calc,D); */
//	        v->Out = v->OutPreSat;
//	    }
//
//	    /* Compute the saturate difference*/
//	    v->SatErr = v->Out - v->OutPreSat;
//	    /* Update the previous proportional output*/
//	    v->Up1 = v->Up;
//	}
    /* PATH(pid_calc,E); */
}


void Act_SetP(_iq val)
{
	/* PATH(Act_SetP,A); */
	//tPidSpeed.Kp = val;
	/* PATH(Act_SetP,B); */
}

/* end actuation.c*/

