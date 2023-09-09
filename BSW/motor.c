/****************************************************************************************************
*  File name: motor.c
*
*  Purpose: Implements motor control Output driver.
*  Description of contents/purpose of this file.
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
//#include "dsp281x.h"
#include "F2837xD_device.h"
//#include "F2837xD_epwm.h"
#include "motor.h"
#include "parameter.h"
#include "gpio.h"
#include "actuation.h"
#include "nvm.h"
#include "brake.h"
#include "events.h"
#include "mcu.h"
#include "adc.h"
#include "timer.h"
//#include "panel.h"
#include "gse.h"
#include "spi.h"

/*      Local Type Definitions */
typedef struct
{
    uint16_t ePwm1A;
    uint16_t ePwm1B;
    uint16_t ePwm2A;
    uint16_t ePwm2B;
    uint16_t ePwm3A;
    uint16_t ePwm3B;
} tEpwmCommutationState;

/*      Local Defines */
#define NUM_COMMSTATES      8

#define EPWM1_REGS_BASE_ADDRESS                 0x00004000
#define EPWM2_REGS_BASE_ADDRESS                 0x00004100
#define EPWM3_REGS_BASE_ADDRESS                 0x00004200

#define INVALID_HALL_STATE  0U

/* Action Qualifier Continuous Software Force Definitions */
#define AQCSFRC_FORCE_LOW         1U
#define AQCSFRC_FORCE_HIGH        2U
#define AQCSFRC_DISABLE_FORCE     0U
#define I_ANLG_OFFSET             360UL

/*      Global Variables */
extern float32_t f32MotorCurrentAmps;

/*      Local ROM Constants */

/* 
  brief Hall State-indexed array of PWM controller settings for motor 
        commutation
  
  This array represents the PWM controller settings necessary to turn the 
  motor, based on the Hall Sensor state input, and on the desired motor 
  direction.  The Hall State is to be used to index the array.  For CW, use 
  the State value directly. For CCW, invert the State bitwise and use that 
  value as the array index.
  
  The value in each entry of the array can be copied directly to the EV's 
  ACTR register, for PWM 'enable' control.
  
  
  motor
*/
#if defined(DRV8312_DEV_KIT)
/* 28377D Development Kit */

const tEpwmCommutationState CommutationStateTable[NUM_COMMSTATES] =  { \
    /*          EPWM1A / CH,           EPWM1B / CL,           EPWM2A / BH,           EPWM2B / BL,           EPWM3A / AH,           EPWM3B / AL   */
    {     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW   },  /* Invalid Hall State */
    {     AQCSFRC_FORCE_LOW,    AQCSFRC_FORCE_HIGH,     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW, AQCSFRC_DISABLE_FORCE,    AQCSFRC_FORCE_HIGH   },  /* State 1:  ANC */
    {     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW, AQCSFRC_DISABLE_FORCE,    AQCSFRC_FORCE_HIGH,     AQCSFRC_FORCE_LOW, AQCSFRC_DISABLE_FORCE   },  /* State 2:  BNA */
    {     AQCSFRC_FORCE_LOW, AQCSFRC_DISABLE_FORCE, AQCSFRC_DISABLE_FORCE,    AQCSFRC_FORCE_HIGH,     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW   },  /* State 3:  BNC */
    { AQCSFRC_DISABLE_FORCE,    AQCSFRC_FORCE_HIGH,     AQCSFRC_FORCE_LOW,    AQCSFRC_FORCE_HIGH,     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW   },  /* State 4:  CNB */
    {     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW,    AQCSFRC_FORCE_HIGH, AQCSFRC_DISABLE_FORCE,    AQCSFRC_FORCE_HIGH   },  /* State 5:  ANB */
    { AQCSFRC_DISABLE_FORCE,    AQCSFRC_FORCE_HIGH,     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW,    AQCSFRC_FORCE_HIGH   },  /* State 6:  CNA */
    {     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW   },  /* Invalid Hall State */
};
#else
/* MCU HW */
const tEpwmCommutationState CommutationStateTable[NUM_COMMSTATES] =  { \
/*          EPWM1A / AH,           EPWM1B / AL,             EPWM2A / BH,            EPWM2B / BL,           EPWM3A / CH,             EPWM3B / CL   */
   {   AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW   },  /* Invalid Hall State */
   {   AQCSFRC_DISABLE_FORCE,  AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_HIGH  },  /* State 1:  ANC */
   {   AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_HIGH,     AQCSFRC_DISABLE_FORCE,  AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW   },  /* State 2:  BNA */
   {   AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_DISABLE_FORCE,  AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_HIGH  },  /* State 3:  BNC */
   {   AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_HIGH,     AQCSFRC_DISABLE_FORCE,  AQCSFRC_FORCE_LOW   },  /* State 4:  CNB */
   {   AQCSFRC_DISABLE_FORCE,  AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_HIGH,     AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW   },  /* State 5:  ANB */
   {   AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_HIGH,     AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_DISABLE_FORCE,  AQCSFRC_FORCE_LOW   },  /* State 6:  CNA */
   {   AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW   },  /* Invalid Hall State */
};

const tEpwmCommutationState CommutationStateTableCCW[NUM_COMMSTATES] =  { \
/*          EPWM1A / AH,           EPWM1B / AL,             EPWM2A / BH,            EPWM2B / BL,           EPWM3A / CH,             EPWM3B / CL   */
   {   AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW   },  /* Invalid Hall State */
   {   AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_HIGH,     AQCSFRC_DISABLE_FORCE,  AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW   },  /* State 1:  ANC */
   {   AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_HIGH,     AQCSFRC_DISABLE_FORCE,  AQCSFRC_FORCE_LOW   },  /* State 2:  BNA */
   {   AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_HIGH,     AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_DISABLE_FORCE,  AQCSFRC_FORCE_LOW   },  /* State 3:  BNC */
   {   AQCSFRC_DISABLE_FORCE,  AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_HIGH  },  /* State 4:  CNB */
   {   AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_DISABLE_FORCE,  AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_HIGH  },  /* State 5:  ANB */
   {   AQCSFRC_DISABLE_FORCE,  AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_HIGH,      AQCSFRC_FORCE_LOW,     AQCSFRC_FORCE_LOW   },  /* State 6:  CNA */
   {   AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW,      AQCSFRC_FORCE_LOW   },  /* Invalid Hall State */
};
#endif
/* Action Qualifier Continuous Software Force Register settings for EPWM Module to Dynamically Brake the Motor */
const tEpwmCommutationState tEpwmDynamicBraking = { \
    AQCSFRC_FORCE_LOW,     /* EPWM 1A signal (Upper Gate):  "01: Clear:  Force Low"   */ \
    AQCSFRC_FORCE_HIGH,    /* EPWM 1B signal (Lower Gate):  "10: Set:    Force High"  */ \
    AQCSFRC_FORCE_LOW,     /* EPWM 2A signal (Upper Gate):  "01: Clear:  Force Low"   */ \
    AQCSFRC_FORCE_HIGH,    /* EPWM 2B signal (Lower Gate):  "10: Set:    Force High"  */ \
    AQCSFRC_FORCE_LOW,     /* EPWM 3A signal (Upper Gate):  "01: Clear:  Force Low"   */ \
    AQCSFRC_FORCE_HIGH     /* EPWM 3B signal (Lower Gate):  "10: Set:    Force High"  */ \
};

/* Action Qualifier Continuous Software Force Register settings for EPWM Module to Open Windings of Motor */
const tEpwmCommutationState tEpwmOpenWindings = { \
    AQCSFRC_FORCE_LOW,     /* EPWM 1A signal (Upper Gate):  "01: Set:  Force Low"  */ \
    AQCSFRC_FORCE_LOW,     /* EPWM 1B signal (Lower Gate):  "01: Set:  Force Low"  */ \
    AQCSFRC_FORCE_LOW,     /* EPWM 2A signal (Upper Gate):  "01: Set:  Force Low"  */ \
    AQCSFRC_FORCE_LOW,     /* EPWM 2B signal (Lower Gate):  "01: Set:  Force Low"  */ \
    AQCSFRC_FORCE_LOW,     /* EPWM 3A signal (Upper Gate):  "01: Set:  Force Low"  */ \
    AQCSFRC_FORCE_LOW      /* EPWM 3B signal (Lower Gate):  "01: Set:  Force Low"  */ \
};



/*      Local Variable Declarations */
Timer_t BrakeReleaseTimer = TIMER_DEFAULTS;

int16 OnCalibratedPos = -32767;
int16 X_CalibratedPos = -32767;

/*      Local Function Prototypes */

/*      Function Definitions */
/* workaround this issue for now
Uint16 initInverterEPwmRegs(EPWM_REGS *pEPwmReg, Uint16 u16Period); */
void initInverterEPwmRegs(uint16_t u16Period);


/*
    brief Initialize motor control driver.

    Purpose:
        Configures GPIOs, PWMs for motor control, and the ADC pin for
        monitoring the signals.

    param[in]  p     pointer to Motor_t structure containing PWM parameters

    Global Data Referenced:
        PrimarySide 
        EvaRegs 
        EvbRegs
        p->PeriodMax
        #PWM_INIT_STATE
        #SYSTEM_FREQUENCY
        #PWM_FREQUENCY_PRI
        #PWM_FREQUENCY_SEC         
    
    return  void
    
    Preconditions and Assumptions:
        This function should be called only once, prior to the scheduler loop.

*/
void Motor_Init( Motor_t *p )
{
    //Uint16 u16Error = 0U;

    /* PATH(Motor_Init,A); */

    /* shall return immediately if  p argument is NULL, otherwise continue */
    if (p)
    {
        /* PATH(Motor_Init,E); */

        /* MCU Hardware */
        /* ------------------------------------------------------------
        * MCU (28377D) Configuration
        * EPWM Modules 1,2,3 are used with their respective timers
        * and Compare registers to control the High and Low side
        * FETs for PWM respectively. Low sides are PWM'd and high sides
        * are forced. A single Compare Register is used for PWM of each
        * phase as the uppers are forced at each step and lowers are PWM'd
        *  ---------------------------------------------------
        *  SIGNAL           Active   Pin     PWM         CMP
        *  ---------------------------------------------------
        *  PHA_HS_CNTL_CHA   high    160     EPWM3A      CMPA
        *  PHA_LS_CNTL_CHA   high    161     EPWM3B      N/A
        *  PHB_HS_CNTL_CHA   high    162     EPWM2A      CMPA
        *  PHB_LS_CNTL_CHA   high    163     EPWM2B      N/A
        *  PHC_HS_CNTL_CHA   high    164     EPWM1A      CMPA
        *  PHC_LS_CNTL_CHA   high    165     EPWM1B      N/A
        */
        /* PATH(Motor_Init,B); */
        /* PWM Period. */
        p->PeriodMax = ((((SYSTEM_FREQUENCY * HZ_TO_KHZ_MULTIPLIER) * KHZ_TO_MHZ_MULTIPLIER) / SYSCLK_TO_EPWMCLK_NET_DIVIDE_VALUE) / PWM_FREQUENCY_PRI);


        /*
        The proper procedure for enabling ePWM clocks is as follows:
        0. Configure GPIO to use EPWM Modules for Phase Gate Pins
        1. Enable ePWM module clocks in the PCLKCRx register (done in InitPeripheralClocks() from InitSysCtrl()
        2. Set TBCLKSYNC= 0
        3. Configure ePWM modules
        4. Set TBCLKSYNC= 1
        */

        /* STEP 0:  Configure the GPIO for Inverter Gates to EPWM Modules. */
        EALLOW;

        /* Select Group 0 for GPIO0/1 (Phase C) gate pins */
        GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = GPIO_MUX_GRP_0;      /* Select group 0 */
        GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = GPIO_MUX_GRP_0;      /* Select group 0 */
        /* Select Group 0 for GPIO2/3 (Phase B) gate pins */
        GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = GPIO_MUX_GRP_0;      /* Select group 0 */
        GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = GPIO_MUX_GRP_0;      /* Select group 0 */
        /* Select Group 0 for GPIO4/5 (Phase A) gate pins */
        GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = GPIO_MUX_GRP_0;      /* Select group 0 */
        GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = GPIO_MUX_GRP_0;      /* Select group 0 */

        /* Disable pull-up functionality for GPIO0/1 (Phase C) gate pins */
        GpioCtrlRegs.GPAPUD.bit.GPIO0 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO0 (EPWM1A) */
        GpioCtrlRegs.GPAPUD.bit.GPIO1 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO1 (EPWM1B) */
        /* Disable pull-up functionality for GPIO2/3 (Phase B) gate pins */
        GpioCtrlRegs.GPAPUD.bit.GPIO2 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO2 (EPWM2A) */
        GpioCtrlRegs.GPAPUD.bit.GPIO3 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO3 (EPWM2B) */
        /* Disable pull-up functionality for GPIO4/5 (Phase A) gate pins */
        GpioCtrlRegs.GPAPUD.bit.GPIO4 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO4 (EPWM3A) */
        GpioCtrlRegs.GPAPUD.bit.GPIO5 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO5 (EPWM3B) */

        /* Set GPIO0/1 (Phase C) gate pins as Push Pull Outputs */
        GpioCtrlRegs.GPADIR.bit.GPIO0 = GPIO_OUTPUT;            /* Configure GPIO0 (EPWM1A) as an output */
        GpioCtrlRegs.GPADIR.bit.GPIO1 = GPIO_OUTPUT;            /* Configure GPIO1 (EPWM1B) as an output */
        GpioCtrlRegs.GPAODR.bit.GPIO0 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO0 (EPWM1A) as push-pull normal output */
        GpioCtrlRegs.GPAODR.bit.GPIO1 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO0 (EPWM1B) as push-pull normal output */
        /* Set GPIO2/3 (Phase B) gate pins as Push Pull Outputs */
        GpioCtrlRegs.GPADIR.bit.GPIO2 = GPIO_OUTPUT;            /* Configure GPIO2 (EPWM2A) as an output */
        GpioCtrlRegs.GPADIR.bit.GPIO3 = GPIO_OUTPUT;            /* Configure GPIO3 (EPWM2B) as an output */
        GpioCtrlRegs.GPAODR.bit.GPIO2 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO2 (EPWM2A) as push-pull normal output */
        GpioCtrlRegs.GPAODR.bit.GPIO3 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO3 (EPWM2B) as push-pull normal output */
        /* Set GPIO4/5 (Phase A) gate pins as Push Pull Outputs */
        GpioCtrlRegs.GPADIR.bit.GPIO4 = GPIO_OUTPUT;            /* Configure GPIO4 (EPWM3A) as an output */
        GpioCtrlRegs.GPADIR.bit.GPIO5 = GPIO_OUTPUT;            /* Configure GPIO5 (EPWM3B) as an output */
        GpioCtrlRegs.GPAODR.bit.GPIO4 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO4 (EPWM3A) as push-pull normal output */
        GpioCtrlRegs.GPAODR.bit.GPIO5 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO5 (EPWM3B) as push-pull normal output */

        /* Configure GPIO0/1 (Phase C) to EPWM1A/1B */
        GpioCtrlRegs.GPAMUX1.bit.GPIO0 = GPIO_MUX_TYPE_1;       /* Configure GPIO0 as EPWM1A */
        GpioCtrlRegs.GPAMUX1.bit.GPIO1 = GPIO_MUX_TYPE_1;       /* Configure GPIO1 as EPWM1B */
        /* Configure GPIO2/3 (Phase B) to EPWM2A/2B */
        GpioCtrlRegs.GPAMUX1.bit.GPIO2 = GPIO_MUX_TYPE_1;       /* Configure GPIO2 as EPWM2A */
        GpioCtrlRegs.GPAMUX1.bit.GPIO3 = GPIO_MUX_TYPE_1;       /* Configure GPIO3 as EPWM2B */
        /* Configure GPIO4/5 (Phase A) to EPWM3A/3B */
        GpioCtrlRegs.GPAMUX1.bit.GPIO4 = GPIO_MUX_TYPE_1;       /* Configure GPIO4 as EPWM3A */
        GpioCtrlRegs.GPAMUX1.bit.GPIO5 = GPIO_MUX_TYPE_1;       /* Configure GPIO5 as EPWM3B */


        /* STEP 1:  Configure the EWPM Modules */

        /* Disable Synchronization of EPWM */
        CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
        EDIS;

        /* Configure EPWM1/EWPM2/EPWM3 Modules */
        initInverterEPwmRegs(p->PeriodMax);
        /* F2837xD_epwm.h must be reworked to use typedefs instead of structs to be able to use
         * this approach. For now, since just proving technology, work around issue by setting up
         * each register autonomously in a series; EPwm1Regs, EPwm2Regs, EPwm3Regs */
//        u16Error |= initInverterEPwmRegs(&EPwm1Regs, p->PeriodMax);   /* Phase C Inverter Gates */
//        u16Error |= initInverterEPwmRegs(&EPwm2Regs, p->PeriodMax);   /* Phase B Inverter Gates */
//        u16Error |= initInverterEPwmRegs(&EPwm3Regs, p->PeriodMax);   /* Phase A Inverter Gates */

        EALLOW;
        /* Synchronize all Enabled EPWM modules to the same TBCLK */
        CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
        EDIS;

    }
    
    /* PATH(Motor_Init,D); */
}


/*
    brief Updates PWM control registers with a new Commutation state

    Purpose:
        Enables and disables the proper PWM outputs controlling the motor by 
        copying the new Motor Commutation State from the state table.

    param[in]  p     pointer to Motor_t structure containing PWM parameters

    Global Data Referenced:
        CommutationStateTable 
        EvaRegs 
        MotorCmd
        BrakeReleaseTimer
        #TIMER_ONESHOT
        #TIMER_10ms
    
    return  void
    
*/
void Motor_Update( Motor_t *p )
{
    uint16_t u16CommutationStep = 0U;

    /* PATH(Motor_Update,A); */

    /* shall return immediately if  p argument is NULL, otherwise continue */
    if (p)
    {
        /* PATH(Motor_Update,E); */
        /* MCU HW */
        /* shall commutate the motor by setting the Action Qualifier Control Registers
         * based on the current hall state */
        if (MotorCmd.MotorStop == false)
        {
            /* Open all gates prior to commutating to next step to protect against dead shorts */
            Motor_OpenWindings();

            /* Setup Action Control Registers for Delfino */
            /* PATH(Motor_Update,B); */
            /* Want to make active the high side and force the low side. Can force the low sides
             * through the AQCSFRC (continuous force) registers. Need to "unforce" the low side
             * for the gate that is going to be switched on.
             * Use the normal AQCTL (control) registers to make active the high side gate that
             * will be PWM'd. Again use the AQCSFRC (continuous force) registers to force
             * the inactive high side gates to be off.
             * We will need to write to all 6 AQSFRC registers to accomplish this. The AQCTL
             * registers for PWMing the high side gates were setup during the initialization phase.
             */
            /* Save copy to prevent overwrite before finalizing registers */
            u16CommutationStep = p->CmtnPointer;

            /* Set up the Action Qualifier Continuous Software Force Registers per the hall state to commutate motor */
            if (MotorCmd.MotorDirection == CCW)
            {
                EPwm1Regs.AQCSFRC.bit.CSFA = CommutationStateTableCCW[u16CommutationStep].ePwm1A;
                EPwm1Regs.AQCSFRC.bit.CSFB = CommutationStateTableCCW[u16CommutationStep].ePwm1B;
                EPwm2Regs.AQCSFRC.bit.CSFA = CommutationStateTableCCW[u16CommutationStep].ePwm2A;
                EPwm2Regs.AQCSFRC.bit.CSFB = CommutationStateTableCCW[u16CommutationStep].ePwm2B;
                EPwm3Regs.AQCSFRC.bit.CSFA = CommutationStateTableCCW[u16CommutationStep].ePwm3A;
                EPwm3Regs.AQCSFRC.bit.CSFB = CommutationStateTableCCW[u16CommutationStep].ePwm3B;
            }
            else
            {
                EPwm1Regs.AQCSFRC.bit.CSFA = CommutationStateTable[u16CommutationStep].ePwm1A;
                EPwm1Regs.AQCSFRC.bit.CSFB = CommutationStateTable[u16CommutationStep].ePwm1B;
                EPwm2Regs.AQCSFRC.bit.CSFA = CommutationStateTable[u16CommutationStep].ePwm2A;
                EPwm2Regs.AQCSFRC.bit.CSFB = CommutationStateTable[u16CommutationStep].ePwm2B;
                EPwm3Regs.AQCSFRC.bit.CSFA = CommutationStateTable[u16CommutationStep].ePwm3A;
                EPwm3Regs.AQCSFRC.bit.CSFB = CommutationStateTable[u16CommutationStep].ePwm3B;
            }

            Timer_SetTime(&BrakeReleaseTimer, TIMER_ONESHOT, TIMER_10ms);
        }
        /* shall set the PWM control register to halt commutation when the
           MotorStop command is issued */
        else
        {
            /* Setup Action Control Registers for Delfino */
            /* Set the motor to dynamic braking state by controlling the AQCSFRC registers
             * for both the High and Low Side (A & B). Set the AQCSFRC Registers A and B */
           if (McuGetState() == FAS_FAIL) /*to avoid current flow during short circuit fault*/
           {
           	  /*If the MCU detected the fault then avoid dynamic brake to the motor*/
               Motor_OpenWindings();
           }
           else
           {
              Motor_DynamicBrakeWindings(); /*dynamic brake the motor */
           }
        }

    }
    
    /* PATH(Motor_Update,D); */
}


/*
    brief Controls PWM output for motor drive.

    Purpose:
        Configures the duty cycles for the PWM outputs controlling the motor.

    param[in]  p     pointer to Motor_t structure containing PWM parameters

    par Global Data Referenced:
        MotorCmd
        EvaRegs 
    
    return  void
    
    Preconditions and Assumptions:
         None.


*/
void Motor_UpdatePWM( Motor_t *p )
{
    int32 Tmp;
    int16 Period, PWM_Inverted;
    Uint32 PWM_Normalized;

    /* PATH(Motor_UpdatePWM,A); */

    /* shall return immediately if  p argument is NULL, otherwise continue */
    if (p)
    {
        /* PATH(Motor_UpdatePWM,D); */

        /* shall calculate the desired Compare Registers value to attain the 
           commanded Duty cycle */
        
        /* Q15 = Q0*Q15 */
        Tmp = (((int32)p->PeriodMax) * ((int32)p->MfuncPeriod));
        /* Q15 -> Q0 (Period) */
        Period = (int16)(Tmp >> 15);         
        
        // REWORK:  Set this up for development kit VBUS = 12.54VDC.
        /* Normalize PWM to expected motor voltage */
        PWM_Normalized = (Uint32) p->DutyFunc;
        
        /* Invert PWM Duty Cycle */
        PWM_Inverted = (0x7FFF - PWM_Normalized);
        /* (Q15 = Q0*Q15) -> Q0 */
        Tmp = ((((int32)Period) * ((int32)PWM_Inverted)) >> 15);
        //Tmp = ((((int32)Period) * ((int32)PWM_Normalized)) >> 15);

        /* Set the Compare Registers associated with the New Duty Cycle to all 3 phases */
        EPwm1Regs.CMPA.bit.CMPA = (int16_t) Tmp;    /* EPWM1A Output Compare Value */
        EPwm2Regs.CMPA.bit.CMPA = (int16_t) Tmp;    /* EPWM2A Output Compare Value */
        EPwm3Regs.CMPA.bit.CMPA = (int16_t) Tmp;    /* EPWM3A Output Compare Value */
    }
    
    /* PATH(Motor_UpdatePWM,C); */
}


/*
    brief Applies "full stop" to PWM output for motor drive.

    Purpose:
        Controls the bridge outputs to Stop the motor.

    param[in]  p     pointer to Motor_t structure containing PWM parameters
    
    Global Data Referenced:
         tHall
         tHallx 
         InCalMode 
         MotorCmd
         STATE_ID
         speed1
         EvaRegs
         xChannelEnableStart
         Nvm_State
            
    return  void
    
    Preconditions and Assumptions:
         None.

*/
void Motor_Stop( Motor_t *p )
{
    /* PATH(Motor_Stop,A); */
    
    /* shall return immediately if  p argument is NULL, otherwise continue */
    if (p)
    {
        /* PATH(Motor_Stop,B); */

        /* shall set motor control parameters to bring the motor to a stop */
        
        /*Set the Revolutions data element of the tHall data structure to 0*/ 
        tHall.Revolutions = 0;
        /*Set the Speed data element of the speed1 data structure to 0*/
        tSpeed.Speed = 0;               
        /*Set the MotorRunning data element of the MotorCmd data structure to false*/
        MotorCmd.MotorRunning = false;  
        /*Set the MotorSpeed data element of the MotorCmd data structure to 0*/
        MotorCmd.MotorSpeed = 0.0;      

        if (MotorCmd.ManualBrkRls == true)
        {
            /* Open the motor windings */
            Motor_OpenWindings(); /*do not dynamic brake the motor */
        }
        else
        {
            if (McuGetState() == FAS_FAIL)
            {
             /*If the MCU detected the fault then avoid dynamic brake to the motor*/
                Motor_OpenWindings();
            }
           else
            {
              Motor_DynamicBrakeWindings(); /*dynamic brake the motor */
            }
        }

        switch (McuGetState())
        {
            case RUN_MODE:
                /* PATH(Motor_Stop,E); */
            case RIG_MODE:
                /* PATH(Motor_Stop,F); */
            case FAS_FAIL:
                /* PATH(Motor_Stop,H); */
            case TEST_MODE:
                /* PATH(Motor_Stop,I); */
            case IDLE_MODE:
                /* PATH(Motor_Stop,M); */
                /* shall deactivate brake if McuGetState returns RUN_MODE or
                   RIG_MODE or POWER_DOWN or FAS_FAIL or TEST_MODE or IDLE_MODE */
                Brake_Deactivate();
                break;
            default:
                /* PATH(Motor_Stop,K); */
                break;
        }
        
    }

    /* PATH(Motor_Stop,L); */
}


/*
    Purpose:
        To monitor motor commutation while approaching mechanical stops
        when in either RIG mode or executing a calibration function.

    Global Data Referenced:
        Adc_Averaged
        CalibrationModeDone
        tHall
        tHallx
        HardStopTimer
        InCalMode
        MotorCmd
        #TIMER_200ms
        #RIG_CAL_LIMIT
        HardStopTimer
        X_CalibratedPos
        EvaRegs
        OnCalibratedPos
        SendRigOverSpi
        Rigging_Status

    return  void

    Preconditions and Assumptions:
        Intended to be called regularly by the scheduler, but there is
        no timing requirement or dependency within this function.


*/
//void Motor_Capture( void )
//{
//    static HardStopRig_t RigRMS = DoNotRig;
//    static HardStopRig_t RigEMS = DoNotRig;
//    static int16 LastHallCount = -32768;
//    int16 onsideRMS = 0;
//
//    /* PATH(Motor_Capture,A); */
//
//    if (((MotorCmd.Rigging == true) || (InCalMode == true)) &&
//        ((MotorCmd.RiggingHardStop == true) || ((InCalMode == true) &&
//         (CalibrationModeDone == false))))
//    {
//        /* PATH(Motor_Capture,F); */
//        onsideRMS = Nvm_GetPosOnside(RSL);
//
//        /* shall detect a stall if current goes above xA and no quad counts
//           are seen for 200ms */
//        if (((I_ANLG >= RIG_CAL_LIMIT) && (LastHallCount == tHall.Position) && (Timer_IsExpired(&HardStopTimer) == true)) ||
//            ((onsideRMS - tHall.Position > 65) && (InCalMode == true)))
//        {
//            /* PATH(Motor_Capture,G); */
//
//            /* motor has stop, so set motor speed to 0, reset stopPosition to
//               no stop and rigging hard stop to false and set motor stop to
//               true, also dynamic brake the motor */
//            MotorCmd.MotorSpeed = 0.0;
//            MotorCmd.StopPosition = NO_STOP;
//            MotorCmd.MotorStop = true;
//            MotorCmd.RiggingHardStop = false;
//
//            /* Dynamic Brake the motor. */
//            EPwm1Regs.AQCSFRC.bit.CSFA = tEpwmDynamicBraking.ePwm1A;
//            EPwm1Regs.AQCSFRC.bit.CSFB = tEpwmDynamicBraking.ePwm1B;
//            EPwm2Regs.AQCSFRC.bit.CSFA = tEpwmDynamicBraking.ePwm2A;
//            EPwm2Regs.AQCSFRC.bit.CSFB = tEpwmDynamicBraking.ePwm2B;
//            EPwm3Regs.AQCSFRC.bit.CSFA = tEpwmDynamicBraking.ePwm3A;
//            EPwm3Regs.AQCSFRC.bit.CSFB = tEpwmDynamicBraking.ePwm3B;
//
//            Act_SetP(_IQ(0.552));
//
//            /* if in calibrate mode set the on side calibrated postion to the
//               current position and request the other side cross side position
//               via the SPI */
//
//            Timer_ResetTimer(&CalibrationDelayTimer);
//            Timer_ResetTimer(&HardStopTimer);
//
//            if (InCalMode == true)
//            {
//                /* PATH(Motor_Capture,H); */
//                OnCalibratedPos = tHall.Position;
//                Spi_SetOnsideRMSState(true);
//                Spi_SetOnsideCalState(false);
//            }
//            /* if not in calibrate mode then the system is in rig mode and
//               if RigRMS is equal to "DoNotRig" then RIG RMS */
//            else if (RigRMS == DoNotRig)
//            {
//                /* PATH(Motor_Capture,I); */
//                RigRMS = RigNow;
//            }
//            /* if not in calibrate mode then the system is in rig mode and
//               if RigRMS is not equal to "DoNotRig" and RigEMS is equal to
//               "DoNotRig" then RIG EMS */
//            else if (RigEMS == DoNotRig)
//            {
//                /* PATH(Motor_Capture,J); */
//                RigEMS = RigNow;
//            }
//        }
//        /* hard stop detection is delayed until the calibration timer expires */
//        else if ((Timer_IsExpired(&CalibrationDelayTimer) == true))
//        {
//           /* PATH(Motor_Capture,Q); */
//           /* the hard stop timer should be reset if the motor current is greater than the limit,
//              the actuator is moving, or the hard stop timer has not been set */
//           if((I_ANLG < RIG_CAL_LIMIT) || (LastHallCount != tHall.Position) || (Timer_IsSet(&HardStopTimer) == false))
//           {
//               /* PATH(Motor_Capture,K); */
//
//               /* shall reset hard stop timer for 10ms */
//               Timer_SetTime(&HardStopTimer, TIMER_ONESHOT, TIMER_10ms);
//           }
//        }
//
//        /* capture current Hall position */
//        LastHallCount = tHall.Position;
//    }
//    else if ((MotorCmd.Rigging == true) || (InCalMode == true))
//    {
//        /* PATH(Motor_Capture,L); */
//
//        /* If RigRMS equal to "RigNow" then Nvm_SetPosOnside to 1, set RigRMS
//           to "Done" set RMS_Rigged to true */
//        if (RigRMS == RigNow)
//        {
//            /* PATH(Motor_Capture,M); */
//            Nvm_SetPosOnside(RSL);
//            RigRMS = Done;
//            Rigging_Status.RSL_Rigged = true;
//        }
//        /* If RIGRMS not equal to "RigNow" and RigEMS is equal to "RigNow"
//           then Nvm_SetPosOnside to 7, set RigEMS to "Done" set EMS_Rigged
//           to true */
//        else if (RigEMS == RigNow)
//        {
//            /* PATH(Motor_Capture,N); */
//            Nvm_SetPosOnside(ESL);
//            RigEMS = Done;
//            Rigging_Status.ESL_Rigged = true;
//        }
//
//        /* If both RigEMS and RigRMS are "Done" then reset them both to
//           "DoNotRig" and set flag to send Rig data over SPI to true */
//        if ((RigRMS == Done) && (RigEMS == Done))
//        {
//            /* PATH(Motor_Capture,O); */
//            RigRMS = DoNotRig;
//            RigEMS = DoNotRig;
//            SendRigOverSpi = true;
//        }
//    }
//
//    /* PATH(Motor_Capture,P); */
//}

/*
    brief Computes motor current from I_ANLG signal

    Global Data Referenced:

    return  void

*/
void Motor_CalculateCurrent(void)
{
    float32_t f32Current = Adc_f32UnitValRaw.val.f32_I_PHA_CHA;

    /* Identify the maximum current of the 3 windings */
    if (Adc_f32UnitValRaw.val.f32_I_PHB_CHA > f32Current)
    {
        f32Current = Adc_f32UnitValRaw.val.f32_I_PHB_CHA;
    }
    if (Adc_f32UnitValRaw.val.f32_I_PHC_CHA > f32Current)
    {
        f32Current = Adc_f32UnitValRaw.val.f32_I_PHC_CHA;
    }

    /* Save Current off */
    f32MotorCurrentAmps = f32Current;

    /* PATH(Motor_CalculateCurrent,L); */
}

/*
    brief Initialize ePWM Modules that are used to control the Phase Gates for
    Motor Phases A, B, and C inverter circuitry

    Purpose:
        Configures ePWMs for motor control.

        - Set TBCTR to Free Run Mode
        - Count Up Mode
        - 15.425kHz Frequency
        - Clock Divider to /2
        - Disable Phase Loading
        - Disable Phase Offset Functionality
        - Initialize the Compare Value to half the Period
        - Setup Compare A to use Shadow buffer and load on TBCTR = 0 event
        - Initialize Gates to Dynamically Brake the motor

    param[in]

    Global Data Referenced:


    return  void

    Preconditions and Assumptions:
        This function should be called only once, prior to the scheduler loop.

*/
//Uint16 initInverterEPwmRegs(EPWM_REGS * pEPwmReg, Uint16 u16Period)
//{
//    Uint16 u16Error = 1U; /* Initialize to an error condition. Successful execution will reset the error */
//
//    /* Validity check for the pointer to the EPWM Registers */
//    if ( (pEPwmReg == EPWM1_REGS_BASE_ADDRESS) || \
//         (pEPwmReg == EPWM2_REGS_BASE_ADDRESS) || \
//         (pEPwmReg == EPWM3_REGS_BASE_ADDRESS) )
//    {
//        /* Valid argument passed in:  Register to write to is EPwm1Regs, EPwm2Regs, or EPwm3Regs */
//        u16Error = 0U;     /* Reset the Error condition */
//
//        /* Initialize the EPWM Register passed in */
//
//        /* Configure EPWM1/EWPM2/EPWM3 Modules */
//        pEPwmReg->TBCTL.bit.FREE_SOFT = TBCTR_FREE_RUN;
//    }
//
//    return u16Error;
//}
void initInverterEPwmRegs(uint16_t u16Period)
{
    /* Configure EPWM1/EWPM2/EPWM3 Modules */

    /* Configure EPWM1 Module */
    EPwm1Regs.TBCTL.bit.FREE_SOFT = TBCTR_FREE_RUN;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; /* Set to Count-Up */
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;     /* Set the TBPRD to be loaded from Shadow Buffer on event TBCNT = 0 */
    EPwm1Regs.TBPRD = u16Period;               /* Set the Period to equivalent of 15.425 kHz */
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    /* Disable phase loading */
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;        /* Disable the Time-Base Counter Phase Functionality. Do not load from TBPHS */
    EPwm1Regs.TBPHS.bit.TBPHSHR = 0x0000;      /* Disable the phase offset functionality */
    EPwm1Regs.TBCTR = 0x0000;                  /* Clear the Time-Base Counter to start */
    /* note, if using the HSPWM module, then TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV). */
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;      /* Set TBCLK to SYSCLK / 2; EPWMCLKDIV is set to /2 upstream of this Divider. */
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   /* Set High Speed Clock Diver to / 1 since the EPWMCLKDIV is already set to /2 upstream */
    EPwm1Regs.CMPA.bit.CMPA = u16Period;       /* Initially set CMPA to the TBPRD. */
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         /* Use the Shadow Registers for synchronous loads to the Active CMPA Register */
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;   /* Load Active Counter-Compare A (CMPA) from Shadow on Time-Base Counter equal to zero, TBCTR = 0 */
    /* Configure EPWM2 Module */
    EPwm2Regs.TBCTL.bit.FREE_SOFT = TBCTR_FREE_RUN;
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; /* Set to Count-Up */
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;     /* Set the TBPRD to be loaded from Shadow Buffer on event TBCNT = 0 */
    EPwm2Regs.TBPRD = u16Period;               /* Set the Period to equivalent of 15.425 kHz */
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;    /* Disable phase loading */
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;        /* Disable the Time-Base Counter Phase Functionality. Do not load from TBPHS */
    EPwm2Regs.TBPHS.bit.TBPHSHR = 0x0000;      /* Disable the phase offset functionality */
    EPwm2Regs.TBCTR = 0x0000;                  /* Clear the Time-Base Counter to start */
    /* note, if using the HSPWM module, then TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV). */
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;      /* Set TBCLK to SYSCLK / 2; EWPMCLKDIV is set to /2 upstream of this Divider. */
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   /* Set High Speed Clock Diver to / 1 since the EPWMCLKDIV is already set to /2 upstream */
    EPwm2Regs.CMPA.bit.CMPA = u16Period;       /* Initially set CMPA to the TBPRD. */
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         /* Use the Shadow Registers for synchronous loads to the Active CMPA Register */
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;   /* Load Active Counter-Compare A (CMPA) from Shadow on Time-Base Counter equal to zero, TBCTR = 0 */
    /* Configure EPWM3 Module */
    EPwm3Regs.TBCTL.bit.FREE_SOFT = TBCTR_FREE_RUN;
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; /* Set to Count-Up */
    EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;     /* Set the TBPRD to be loaded from Shadow Buffer on event TBCNT = 0 */
    EPwm3Regs.TBPRD = u16Period;               /* Set the Period to equivalent of 15.425 kHz */
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;    /* Disable phase loading */
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;        /* Disable the Time-Base Counter Phase Functionality. Do not load from TBPHS */
    EPwm3Regs.TBPHS.bit.TBPHSHR = 0x0000;      /* Disable the phase offset functionality */
    EPwm3Regs.TBCTR = 0x0000;                  /* Clear the Time-Base Counter to start */
    /* note, if using the HSPWM module, then TBCLK = EPWMCLK / (HSPCLKDIV x CLKDIV). */
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;      /* Set TBCLK to SYSCLK / 2; EWPMCLKDIV is set to /2 upstream of this Divider. */
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   /* Set High Speed Clock Diver to / 1 since the EPWMCLKDIV is already set to /2 upstream */
    EPwm3Regs.CMPA.bit.CMPA = u16Period;       /* Initially set CMPA to the TBPRD. */
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         /* Use the Shadow Registers for synchronous loads to the Active CMPA Register */
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;   /* Load Active Counter-Compare A (CMPA) from Shadow on Time-Base Counter equal to zero, TBCTR = 0 */

    /* MCU HW */
    /* Control Strategy:
     * Upper Gates are PWM'd
     * Lower Gates are On or OFF.
     * The Lower Gates will either be Forced High or Low dependent upon
     * the hall state of the motor and on/off commanded state. The Action Qualifier Continuous
     * Software Force (AQCSFRC) register is used to control this.
     * The Upper Gates are PWM'd. This is controlled through the EPWM using the Count Up
     * method and Action Qualifier Control (AQCTL) Register A. No need for B since those
     * will be forced through the Continuous SW Force Register.
     * Every new hall state will require these registers to be updated with the appropriate
     * values to enable the correct gates of the invert to commutate the motor.
     */

    /* Set the Action Qualifier Control A (AQCTLA) Register to control the
     * EPWM A signals (Upper Gates):
     *  "01: Clear:  Force EPWMxA Low" for EPWM1A/EPWM2A/EPWM3A
     *  because these gates are Active High gates and we only want them on for the
     *  period of the signal. */
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;          /* EPWM1A:  Set on CMPA when Counting Up */
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;          /* EPWM2A:  Set on CMPA when Counting Up */
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;          /* EPWM3A:  Set on CMPA when Counting Up */
    EPwm1Regs.AQCTLA.bit.PRD = AQ_CLEAR;        /* EPWM1A:  Clear on PERIOD when Counting Up */
    EPwm2Regs.AQCTLA.bit.PRD = AQ_CLEAR;        /* EPWM2A:  Clear on PERIOD when Counting Up */
    EPwm3Regs.AQCTLA.bit.PRD = AQ_CLEAR;        /* EPWM3A:  Clear on PERIOD when Counting Up */
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR;        /* EPWM1A:  Clear on ZERO when Counting Up */
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR;        /* EPWM2A:  Clear on ZERO when Counting Up */
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_CLEAR;        /* EPWM3A:  Clear on ZERO when Counting Up */

    /* Initialize the motor to dynamic braking state by controlling the AQCSFRC registers
     * for both the High and Low Side (A & B). Set the AQCSFRC Registers A and B.
     * High Sides are Forced Off.
     * Low Sides are Shorted to Ground. */
    EPwm1Regs.AQCSFRC.bit.CSFA = tEpwmDynamicBraking.ePwm1A;
    EPwm1Regs.AQCSFRC.bit.CSFB = tEpwmDynamicBraking.ePwm1B;
    EPwm2Regs.AQCSFRC.bit.CSFA = tEpwmDynamicBraking.ePwm2A;
    EPwm2Regs.AQCSFRC.bit.CSFB = tEpwmDynamicBraking.ePwm2B;
    EPwm3Regs.AQCSFRC.bit.CSFA = tEpwmDynamicBraking.ePwm3A;
    EPwm3Regs.AQCSFRC.bit.CSFB = tEpwmDynamicBraking.ePwm3B;

}

/*
    Open Windings of motor for free movement

    Purpose:
        Configures ePWM for open windings of motor

    param[in]

    Global Data Referenced:


    return  void

    Preconditions and Assumptions:


*/
void Motor_OpenWindings(void)
{

    /* Open the windings of motor */
    EPwm1Regs.AQCSFRC.bit.CSFA = tEpwmOpenWindings.ePwm1A;
    EPwm1Regs.AQCSFRC.bit.CSFB = tEpwmOpenWindings.ePwm1B;
    EPwm2Regs.AQCSFRC.bit.CSFA = tEpwmOpenWindings.ePwm2A;
    EPwm2Regs.AQCSFRC.bit.CSFB = tEpwmOpenWindings.ePwm2B;
    EPwm3Regs.AQCSFRC.bit.CSFA = tEpwmOpenWindings.ePwm3A;
    EPwm3Regs.AQCSFRC.bit.CSFB = tEpwmOpenWindings.ePwm3B;

}

/*
    Short the lower windings of motor to dynamic brake the motor

    Purpose:
        Configures ePWM for dynamic braking of motor

    param[in]

    Global Data Referenced:


    return  void

    Preconditions and Assumptions:


*/
void Motor_DynamicBrakeWindings(void)
{
    /* Dynamic Brake the Motor */
    EPwm1Regs.AQCSFRC.bit.CSFA = tEpwmDynamicBraking.ePwm1A;
    EPwm1Regs.AQCSFRC.bit.CSFB = tEpwmDynamicBraking.ePwm1B;
    EPwm2Regs.AQCSFRC.bit.CSFA = tEpwmDynamicBraking.ePwm2A;
    EPwm2Regs.AQCSFRC.bit.CSFB = tEpwmDynamicBraking.ePwm2B;
    EPwm3Regs.AQCSFRC.bit.CSFA = tEpwmDynamicBraking.ePwm3A;
    EPwm3Regs.AQCSFRC.bit.CSFB = tEpwmDynamicBraking.ePwm3B;
}


/* end motor.c */

