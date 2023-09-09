/****************************************************************************************************
*  File name: brake.c
*
*  Purpose: Implements brake control driver
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
#include "brake.h"


/*Commenting out as Brake functionality is deferred for MCU*/
#if 0
#include "F2837xD_device.h"
#include "parameter.h"
#include "motor.h"
#include "gpio.h"
#include "mcu.h"
#include "actuation.h"
#include "timer.h"
#include "panel.h"
#include "nvm.h"
#include "adc.h"
#include "ASW_BSW_I.h"

/*      Local Type Definitions */

/*      Local Defines */

/* Action Qualifier Continuous Software Force Definitions */
#define AQCSFRC_FORCE_LOW         1U
#define AQCSFRC_FORCE_HIGH        2U
#define AQCSFRC_DISABLE_FORCE     0U

/*      Global Variables */

/*      Local ROM Constants */

/*      Local Variable Declarations */
Timer_t BrakeTimer;     /* Timer_t object used to implement brake activation delay */
Uint32 BrakeCounter;    /* Counter used to keep track of the amount of time (in ms) the brake is activated */

/*      Local Function Prototypes */
Uint16 Brake_CalcDutyCycle(Uint16 vBusVoltage);

/*      Function Definitions */

/*
    brief Initialize brake control driver.

     Purpose:
        Configures GPIOs and PWMs for brake control.

     Global Data Referenced:
        #EvaRegs
    
    return  void
    
    Preconditions and Assumptions:
        This function must be called at some point after Motor_Init has setup the Timer 1 Period register.

*/
#endif

void Brake_Init(void)
{
/*Commenting out Brake_Init function as Brake functionality is deferred for MCU*/
/*Brake functionality will be implemented in future. But the GPIO's will change*/
#ifndef __GPD_LITE_NO_BRAKE
    /* PATH(Brake_Init,A); */

    /* ------------------------------------------------------------
    * MCU Configuration (TBD)
    * EPWM Module 8 is used with it's respective timer and Compare
    * registers to control the PWM_BRK_N and PWM_RST_N outputs
    * connected to the BRAKE and OverCurrent / Brake Circuitry
    * respectively. Both outputs are set to the same Period but
    * have different PWMs and therefore both CPMA and CMPB are
    * required for the functionality.
    *  -------------------------------------------
    *  SIGNAL      Active  Pin     PWM         CMP
    *  -------------------------------------------
    *  PWM_BRK_N   Low       6     EPWM8A      CMPA
    *  PWM_RST_N   Low       7     EPWM8B      CMPB
    */

    EALLOW;


    Configure U_BRAKE_N Output
    GpioCtrlRegs.GPCGMUX2.bit.GPIO90 = GPIO_MUX_GRP_0;       //Select group 0
    GpioCtrlRegs.GPCMUX2.bit.GPIO90 = GPIO_MUX_TYPE_0;       //Set as GPIO pin type
    GpioCtrlRegs.GPCDIR.bit.GPIO90 = GPIO_OUTPUT;            //Set as an output
    GpioCtrlRegs.GPCPUD.bit.GPIO90 = GPIO_DISABLE_PULLUP;    //Disable the pullups
    GpioCtrlRegs.GPCODR.bit.GPIO90 = GPIO_NORMAL_OUTPUT;     //Configure as push-pull normal output */
    U_BRAKE_N_set();    /* Engage brake by Deactivating upper corner drive */

    // GPIO 14 and 15 are used for neutral switching
    /* Configure PWM_BRK_N Output */
    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO14 (EPWM8A) */
    GpioCtrlRegs.GPADIR.bit.GPIO14 = GPIO_OUTPUT;            /* Configure GPIO14 (EPWM8A) as an output */
    GpioCtrlRegs.GPAODR.bit.GPIO14 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO14 (EPWM8A) as push-pull normal output */
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = GPIO_MUX_TYPE_1;       /* Configure GPIO14 as EPWM8A */

    /* Configure PWM_RST_N Output */
    GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO15 (EPWM8B) */
    GpioCtrlRegs.GPADIR.bit.GPIO15 = GPIO_OUTPUT;            /* Configure GPIO15 (EPWM8B) as an output */
    GpioCtrlRegs.GPAODR.bit.GPIO15 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO15 (EPWM8B) as push-pull normal output */
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = GPIO_MUX_TYPE_1;       /* Configure GPIO15 as EPWM8B */


    /* Configure EPWM8 Module */
    EPwm8Regs.TBCTL.bit.FREE_SOFT = TBCTR_FREE_RUN;
    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; /* Set to Count-Up */
    EPwm8Regs.TBCTL.bit.PRDLD = TB_SHADOW;     /* Set the TBPRD to be loaded from Shadow Buffer on event TBCNT = 0 */
    EPwm8Regs.TBPRD = ((((SYSTEM_FREQUENCY * HZ_TO_KHZ_MULTIPLIER) * KHZ_TO_MHZ_MULTIPLIER) / SYSCLK_TO_EPWMCLK_NET_DIVIDE_VALUE) / PWM_FREQUENCY_PRI); /* Set the Period to equivalent of 15.425 kHz */
    EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;    /* Disable phase loading */
    EPwm8Regs.TBPHS.bit.TBPHS = 0x0000;        /* Disable the Time-Base Counter Phase Functionality. Do not load from TBPHS */
    EPwm8Regs.TBPHS.bit.TBPHSHR = 0x0000;      /* Disable the phase offset functionality */
    EPwm8Regs.TBCTR = 0x0000;                  /* Clear the Time-Base Counter to start */
    /* note, if using the HSPWM module, then TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV). */
    EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;      /* Set TBCLK to SYSCLK / 2; EPWMCLKDIV is set to /2 upstream of this Divider. */
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   /* Set High Speed Clock Diver to / 1 since the EPWMCLKDIV is already set to /2 upstream */

    /* Setup PWM_BRK_N Timer Compare Functionality */
    EPwm8Regs.CMPA.bit.CMPA = ((((((SYSTEM_FREQUENCY * HZ_TO_KHZ_MULTIPLIER) * KHZ_TO_MHZ_MULTIPLIER) / SYSCLK_TO_EPWMCLK_NET_DIVIDE_VALUE) / PWM_FREQUENCY_PRI)) >> 1); /* Initially set CMPA to the periodMax / 2 */
    EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         /* Use the Shadow Registers for synchronous loads to the Active CMPA Register */
    EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;   /* Load Active Counter-Compare A (CMPA) from Shadow on Time-Base Counter equal to zero, TBCTR = 0 */
    EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;          /* EPWM8A:  Clear on CMPA when Counting Up */
    EPwm8Regs.AQCTLA.bit.PRD = AQ_CLEAR;        /* EPWM8A:  Set on PERIOD when Counting Up */
    EPwm8Regs.AQCTLA.bit.ZRO = AQ_CLEAR;        /* EPWM8A:  Set on ZERO when Counting Up */

    /* Setup PWM_RST_N Timer Compare Functionality */
    /* 80 is a magic number calculated based off the magic number of 30 from VLJ. The ratio of the period to the magic number was used 30/2431 = 80/6482 to figure
     * out the new number. */
    EPwm8Regs.CMPB.bit.CMPB = ((((((SYSTEM_FREQUENCY * HZ_TO_KHZ_MULTIPLIER) * KHZ_TO_MHZ_MULTIPLIER) / SYSCLK_TO_EPWMCLK_NET_DIVIDE_VALUE) / PWM_FREQUENCY_PRI)) - 80U); /* Initially set CMPB to the periodMax - 80 */
    EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;         /* Use the Shadow Registers for synchronous loads to the Active CMPB Register */
    EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;   /* Load Active Counter-Compare B (CMPB) from Shadow on Time-Base Counter equal to zero, TBCTR = 0 */
    EPwm8Regs.AQCTLB.bit.CBU = AQ_SET;          /* EPWM8B:  Clear on CMPB when Counting Up */
    EPwm8Regs.AQCTLB.bit.PRD = AQ_CLEAR;        /* EPWM8B:  Set on PERIOD when Counting Up */
    EPwm8Regs.AQCTLB.bit.ZRO = AQ_CLEAR;        /* EPWM8B:  Set on ZERO when Counting Up */

    /* Configure EPWM8 Module Action Qualifier Software Force Control Registers */
    /* EPWM8A signal (Lower Gate for BRAKE controlled by PWM_BRK_N):  "00: Disable:  Disable Software forces for pin" for EPWM8A. */
    EPwm8Regs.AQCSFRC.bit.CSFA = AQCSFRC_DISABLE_FORCE;        /* Disable all Software Forces for PWM_BRK_N pin */
    /* EPWM8B signal (Lower Gate for OverCurrent Reset controlled by PWM_BRK_N):  "00: Disable:  Disable Software forces for pin" for EPWM8B. */
    EPwm8Regs.AQCSFRC.bit.CSFB = AQCSFRC_DISABLE_FORCE;        /* Disable all Software Forces for PWM_RST_N pin */
//
    EDIS;

    Brake_SetPWM(MIN_DUTY_CYCLE);
    /* PATH(Brake_Init,B); */
    #endif
}


/*
    brief Controls PWM output for the brake.

     Purpose:
        Configures the duty cycle for the PWM output controlling the brake.  A bounds check
        is performed to insure that the dutycycle param cannot be greater than MAX_DUTY_CYCLE.

    param[in]  dutycycle   duty cycle value (percentage) for the PWM output

    Global Data Referenced:
         None.
    
    return  void
    
    Preconditions and Assumptions:
         None.

*/
/*Commenting out Brake_SetPWM function as Brake functionality is deferred for MCU*/
#ifndef __GPD_LITE_NO_BRAKE
void Brake_SetPWM(Uint16 dutycycle)
{

    uint32_t u32Temp;

    /* PATH(Brake_SetPWM,A); */

    /* shall perform a boundary check on the requested duty cycle to ensure that 
       it cannot exceed a predetermined %, to allow for the current limit reset
       PWM pulse. */
    if (dutycycle > MAX_DUTY_CYCLE)
    {
        /* PATH(Brake_SetPWM,B); */
        dutycycle = MAX_DUTY_CYCLE;
    }

    /* Save off the duty cycle */
    MotorCmd.u16BrakeDutyCycle = dutycycle;

    /* Scale up to 32-bit math to calculate new compare value. */
    u32Temp = (uint32_t)EPwm8Regs.TBPRD * (uint32_t)MotorCmd.u16BrakeDutyCycle;   /* Multiply Period by Desired Duty Cycle Percent (out of 100) */
    u32Temp = u32Temp / 100;    /* Divide by 100 to get final Compare Count */

    /*Commenting out as EPWM8 is used for other purposes in MCU SW*/
    /*EPwm8Regs.CMPA.bit.CMPA = (uint16_t) u32Temp;*/   /* Convert back from 32-bit math to 16-bit and save to Compare Register */

    /* PATH(Brake_SetPWM,C); */
}
#endif


/*
    brief Periodic control of the brake.

     Purpose:
        This routine performs the necessary maintenance of the brake.  A check is made
        on the InhibitFlap flag to see if the PWM should be driven or not.  If the flap is 
        currently not inhibited, then check the VBUS voltage and adjust the PWM accordingly
        to achieve 14VDC average.

     Global Data Referenced:
        #InhibitFlap  
        #MotorCmd   
        #BrakeTimer 
        #InhibitFlap 
        #BrakeCounter
    
    return  void
*/
void Brake_Control(void)
{
/*Commenting out Brake_Control function as Brake functionality is deferred for MCU*/
#ifndef __GPD_LITE_NO_BRAKE
    Uint16 dutycycle;

    /* PATH(Brake_Control,A); */
    
    if (InhibitFlap == false)
    {
        /* PATH(Brake_Control,D); */

        /* shall activate the brake if "BrakeActivated" flag is set and activation delay timer has expired */
        if ((MotorCmd.BrakeActivated == true) &&
            (Timer_IsExpired(&BrakeTimer) == true))
        {
            /* PATH(Brake_Control,E); */

            /* shall read the VBUS A/D count and calculate the brake duty cycle*/
            dutycycle = Brake_CalcDutyCycle(VBUS_MON);

            /* shall adjust the dutycycle of the brake PWM to allow for an average of 
               14VDC (BRK_PWM_AVG_VOLT) based on the VBUS voltage. */
            Brake_SetPWM(dutycycle);
        }
    }

    /* shall only use brake timer in rig mode */
    if ((McuGetState() == RIG_MODE) && (MotorCmd.ManualBrkRls == false))
    {
        /* PATH(Brake_Control,F); */

        /* shall increment brake counter and if it is activated for 5 minutes without
           being deactivated, stop motor and deactivate brake */
        if (MotorCmd.BrakeActivated == true)
        {
            /* PATH(Brake_Control,G); */

            if (BrakeCounter < 300000)
            {
                /* PATH(Brake_Control,H); */
                BrakeCounter++;
            }

            if (BrakeCounter >= 300000)
            {
                /* PATH(Brake_Control,I); */
                MotorCmd.MotorSpeed = 0.0;
                MotorCmd.StopPosition = NO_STOP;
                MotorCmd.MotorStop = true;
                MotorCmd.BrakeCooling = true;
            }
        }
        /* shall decrement brake counter if brake is deactivated, minimum value is zero */
        else
        {
            /* PATH(Brake_Control,J); */

            if (BrakeCounter > 0)
            {
                /* PATH(Brake_Control,K); */
                BrakeCounter--;
            }
        }

        /* shall deactivate brake while brake is cooling and brake timer decreases to 240000 (1 min) */
        if ((MotorCmd.BrakeCooling == true) /*&& (MotorCmd.BrakeActivated == false)*/ )
        {
            /* PATH(Brake_Control,L); */

            if (BrakeCounter < 240000)
            {
                /* PATH(Brake_Control,M); */
                MotorCmd.BrakeCooling = false;
                BrakeCounter = 0;
            }
        }
    }

    /* PATH(Brake_Control,N); */
    #endif
}


/*Commenting out Brake_CalcDutyCycle function as Brake functionality is deferred for MCU*/
#ifndef __GPD_LITE_NO_BRAKE
Uint16 Brake_CalcDutyCycle(Uint16 vBusVoltage)
{
	Uint16 dutyCycle = 0;

	/* PATH(Brake_CalcDutyCycle,A); */

	if (vBusVoltage < 1492)       /* 21-22V */
    {
        /* PATH(Brake_CalcDutyCycle,F); */
        dutyCycle = 0;
		InhibitFlap = true;                     /* Voltage out of Range, stop motor */
		Events_PostEvent(PositionComplete, 0);  /* Post the event to stop */
    }
    else if (vBusVoltage < 1560)       /* 22-23V */
    {
        /* PATH(Brake_CalcDutyCycle,G); */
        dutyCycle = 61;
    }
    else if (vBusVoltage < 1628)       /* 23-24V */
    {
        /* PATH(Brake_CalcDutyCycle,H); */
        dutyCycle = 58;
    }
    else if (vBusVoltage < 1696)       /* 24-25V */
    {
        /* PATH(Brake_CalcDutyCycle,I); */
        dutyCycle = 56;
    }
    else if (vBusVoltage < 1764)       /* 25-26V */
    {
        /* PATH(Brake_CalcDutyCycle,J); */
        dutyCycle = 53;
    }
    else if (vBusVoltage < 1832)       /* 26-27V */
    {
        /* PATH(Brake_CalcDutyCycle,K); */
        dutyCycle = 51;
    }
    else if (vBusVoltage < 1900)       /* 27-28V */
    {
        /* PATH(Brake_CalcDutyCycle,L); */
        dutyCycle = 50;
    }
    else if (vBusVoltage < 1967)       /* 28-29V */
    {
        /* PATH(Brake_CalcDutyCycle,M); */
        dutyCycle = 48;
    }
    else if (vBusVoltage < 2035)       /* 29-30V */
    {
        /* PATH(Brake_CalcDutyCycle,N); */
        dutyCycle = 46;
    }
    else if (vBusVoltage < 2103)       /* 30-31V */
    {
        /* PATH(Brake_CalcDutyCycle,O); */
        dutyCycle = 45;
    }
    else                            /* > 31V or greater */
    {
        /* PATH(Brake_CalcDutyCycle,P); */
        dutyCycle = 40; /* VBUS voltage too high */
    }

	/* PATH(Brake_CalcDutyCycle,Q); */
	return (dutyCycle);
}
#endif


/*
    brief Wrapper function for deactivating the brake."Engage the Brake"

     Purpose:
        Sets the PWM to 0% and sets U_BRAKE_N high.

     Global Data Referenced:
        #MotorCmd  
        #Nvm_State 
        #BrakeReleaseTimer 
        #tHall  
        #tHallx
    
    return  void
    
    Preconditions and Assumptions:
         None.

*/
void Brake_Deactivate(void)
{
/*Commenting out Brake_Deactivate function as Brake functionality is deferred for MCU but might be used in future*/
#ifndef __GPD_LITE_NO_BRAKE
    float32_t f32Delta = 0.0F;
    STATE_ID tFscuMode = McuGetState();

    /* PATH(Brake_Deactivate,A); */

    /* shall check if the brake is not being manually released */
    if ((MotorCmd.ManualBrkRls == false) && (Timer_IsExpired(&BrakeReleaseTimer) == true))
    {
        /* PATH(Brake_Deactivate,B); */

        /* shall set the PWM to 0% duty cycle */
        Brake_SetPWM(MIN_DUTY_CYCLE);

        /* shall set the upper corner drive inactive (logic level 1) */
        /*U_BRAKE_N_set();*/        /*Commenting out as GPIO 90 is used for other purpose in MCU SW*/
    
        /* shall post a "Position Complete" event if brake is activated and motor reaches its desired position */
        if (MotorCmd.BrakeActivated == true)
        {
            /* PATH(Brake_Deactivate,C); */
            /* Determine to close loop on Sensor Fusion output or hall feedback based on mode */
            if(tFscuMode == RUN_MODE)
            {
                /* calculate position delta based on the motor direction */
                if (MotorCmd.MotorDirection == CW)
                {
                    f32Delta = MotorCmd.f32StopPosition - G_ASW_DATA.tPanelOutputData.f32StrokeFused;
                }
                else if (MotorCmd.MotorDirection == CCW)
                {
                    f32Delta = G_ASW_DATA.tPanelOutputData.f32StrokeFused - MotorCmd.f32StopPosition;
                }
                /* If flap has reached desired stroke then post the PositionComplete Event */
                if ((f32Delta <= 0.0F) && (MotorReversing == false))
                {
                    Nvm_State.quadposition.onside = tHall.Position;
                    Nvm_State.quadposition.xside = tHallx.Position;
                    Events_PostEvent(PositionComplete, 0);
                }
            }
            else
            {
                /* All other modes close loop on hall effect position */
                if ((MotorCmd.StopPosition == tHall.Position) && (MotorReversing == false))
                {
                    /* PATH(Brake_Deactivate,D); */
                    Nvm_State.quadposition.onside = tHall.Position;
                    Nvm_State.quadposition.xside = tHallx.Position;
                    Events_PostEvent(PositionComplete, 0);
                }
            }
        }
    
        /* shall clear "BrakeActivated" flag */
        MotorCmd.BrakeActivated = false;
    }
    #endif

    /* PATH(Brake_Deactivate,E); */
}

/*
    brief Wrapper function for fully activating the brake. "disengage the brake"

    Purpose:
        Sets the PWM to maximum and sets U_BRAKE_N low.

     Global Data Referenced:
        #MotorCmd   
        #BrakeTimer
    
    return  void
    
    Preconditions and Assumptions:
         None.

*/
void Brake_Activate(void)
{
/*Commenting out Brake_Activate function as Brake functionality is deferred for MCU*/
#ifndef __GPD_LITE_NO_BRAKE
    /* PATH(Brake_Activate,A); */

    /* shall set the PWM to the maximum allowed duty cycle */
    Brake_SetPWM(MAX_DUTY_CYCLE);

    /* shall set the upper corner drive active (logic level 0) */
    /*U_BRAKE_N_clr();*/                /*Commenting out as GPIO 90 is used for other purpose in MCU SW*/

    /* shall start oneshot timer to delay actuation of brake */
    Timer_SetTime(&BrakeTimer, TIMER_ONESHOT, TIMER_50ms);

    /* shall release dynamic braking */
    Motor_OpenWindings();

    /* shall set "BrakeActivated" flag */
    MotorCmd.BrakeActivated = true;

    Timer_SetTime(&BrakeReleaseTimer, TIMER_ONESHOT, TIMER_10ms);

    /* PATH(Brake_Activate,B); */
    #endif
}


/* end brake.c */

