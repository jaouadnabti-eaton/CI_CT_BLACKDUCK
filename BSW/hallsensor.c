/****************************************************************************************************
*  File name: hallsensor.c
*
*  Purpose: Implements Hall Sensor driver
*  The interface provides routines necessary to monitoring the position and
*  speed of the motor through its magnetic sensors.
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
//#include "dsp281x.h"   /* DSP281x Include File, includes register definition file*/
//#include "dsp281x_regs.h"
#include "F2837xD_device.h"
#include "IQmathLib.h"
#include "actuation.h"
#include "hallsensor.h"
#include "gpio.h"
#include "nvm.h"
//#include "gse.h"
//#include "bitmonitors.h"
#include "PDI_Mgr.h"

/*      Local Type Definitions
*/


/*      Local Defines
*/
//#define ECAP1_REG_START_ADDRESS 0x00005000
//#define ECAP2_REG_START_ADDRESS 0x00005020
//#define ECAP3_REG_START_ADDRESS 0x00005040
//#define ECAP4_REG_START_ADDRESS 0x00005060
//#define ECAP5_REG_START_ADDRESS 0x00005080
//#define ECAP6_REG_START_ADDRESS 0x000050A0
#define BIT_ILLEGALHALL_LIMIT       10   /*  Maximum allowable illegal Hall sequences */
#define BIT_ILLEGALHALLSTATE_LIMIT  250  /*  Maximum allowable illegal Hall states */

/*      Global Variables
*/
/* Instance of Hall effect driver for on-side motor*/
Hall_t tHall = HALL_DEFAULTS;

#if defined(__HALLX_CONFIGURED)
/* Instance of Hall effect driver for cross-side motor*/
Hall_t tHallx = HALL_DEFAULTS;
#endif

/*      Local ROM Constants
*/

/*      Local Variable Declarations
*/
/* Definition of proper sequence of Hall-effect sensor commutation states.  The hall sensor
 state is used to index this array.  
 A "bad transition" occurs when the hall sensor state advances in an improper sequence.
 Traversing through the state table indices of +1 or -1 are valid state transitions. */
 const int stateTable[6] = {5, 1, 3, 2, 6, 4};

 /*0th Element of below array represents, the previous Hall state when current Hall state is 1*/
 /*5th Element of below array represents, the previous Hall state when current Hall state is 6*/
 /* Commutation sequence for Clockwise direction - {5,1,3,2,6,4}*/
 const uint16_t PrevState_CW[6] = {5U, 3U, 1U, 6U, 4U, 2U};         // Array shows ideal previous Hall state while Motor Runs in Clockwise Direction

 /* Commutation sequence for Counter-Clockwise direction - {5,4,6,2,3,1}*/
 const uint16_t PrevState_CCW[6] = {3U, 6U, 2U, 5U, 1U, 4U};        // Array shows ideal previous Hall state while Motor Runs in CounterClockwise Direction

/*      Local Function Prototypes
*/
//void Hall_Init_ECap_Register( ECAP_REGS *pECap_Reg, edgeType_t tEdgeType);

/*      Function Definitions
*/
void Hall_ISR( void );
void Hall_InitECapRegisters(void);
#if defined(__HALLX_CONFIGURED)
void Hallx_ISR(void);
void Hallx_InitECapRegisters(void);
#endif

/**
    brief Initialization routine for hall sensor driver.

     Purpose:
        Configures hall sensor inputs and interrupt handler for proper
        operation of the driver.

    param[in] on        Pointer to Hall_t hall sensor driver object for on-side motor
    param[in] cross     Pointer to Hall_t hall sensor driver object for cross-side motor

     Global Data Referenced:
        #tHall 
        #tHallx 
        #MotorCmd   
        #PieVectTable 
        #PieCtrlRegs 
        #EvaRegs 
        #EvbRegs 
    
    return  void
    
     Preconditions and Assumptions:
        Should only be called a single time during System Initialization.

*/
#if !defined(__HALLX_CONFIGURED)
void Hall_Init( Hall_t *on )
#else
void Hall_Init( Hall_t *on, Hall_t *cross )
#endif
{
    /*  PATH(Hall_Init,A); */
#if !defined(__HALLX_CONFIGURED)
    if(on)
#else
    if (on && cross)
#endif
    {
        /* PATH (Hall_Init,C); */

        /***********************/
        /****** On-side ********/
        /***********************/

        EALLOW; /* This is needed to write to EALLOW protected registers*/
        /* shall map the onside capture (ECAP-3) interrupts to the "Hall_ISR" function*/
        /* map the ECAP ISRs */
        PieVectTable.ECAP1_INT = &Hall_1_ISR;   /* Map Hall 1 ISR */
        PieVectTable.ECAP2_INT = &Hall_2_ISR;   /* Map Hall 2 ISR */
        PieVectTable.ECAP3_INT = &Hall_3_ISR;   /* Map Hall 3 ISR */

#if defined(DRV8312_DEV_KIT)
        /* 2837xD Development Kit Hardware */

        /* Setup GPIO_24 as HALL_A input */
        GpioCtrlRegs.GPAGMUX2.bit.GPIO24 = GPIO_MUX_GRP_0;      /* Select group 0 */
        GpioCtrlRegs.GPAMUX2.bit.GPIO24 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
        GpioCtrlRegs.GPADIR.bit.GPIO24 = GPIO_INPUT;            /* Set as an input */
        GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = GPIO_ASYNC;          /* Set input as asynchronous */

        /* Setup GPIO_25 as HALL_B input */
        GpioCtrlRegs.GPAGMUX2.bit.GPIO25 = GPIO_MUX_GRP_0;      /* Select group 0 */
        GpioCtrlRegs.GPAMUX2.bit.GPIO25 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
        GpioCtrlRegs.GPADIR.bit.GPIO25 = GPIO_INPUT;            /* Set as an input */
        GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = GPIO_ASYNC;          /* Set input as asynchronous */

        /* Setup GPIO_26 as HALL_C input */
        GpioCtrlRegs.GPAGMUX2.bit.GPIO26 = GPIO_MUX_GRP_0;      /* Select group 0 */
        GpioCtrlRegs.GPAMUX2.bit.GPIO26 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
        GpioCtrlRegs.GPADIR.bit.GPIO26 = GPIO_INPUT;            /* Set as an input */
        GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = GPIO_ASYNC;          /* Set input as asynchronous */

#else
        /* Initialize the GPIO used for ECAP Interrupts ECAP1,2.3 */
        /* Initialize the GPIO Pin Configuration for on-side Halls;
         * CHA_HS1, CHA_HS2, CHA_HS3 */
        /* MCU HW */

        /* Setup GPIO_6 as On-Side Hall 1 input */
        GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = GPIO_MUX_GRP_0;      /* Select group 0 */
        GpioCtrlRegs.GPAMUX1.bit.GPIO6 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
        GpioCtrlRegs.GPADIR.bit.GPIO6 = GPIO_INPUT;            /* Set as an input */
        GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = GPIO_ASYNC;          /* Set input as asynchronous */

        /* Setup GPIO_7 as On-Side Hall 2 input */
        GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = GPIO_MUX_GRP_0;      /* Select group 0 */
        GpioCtrlRegs.GPAMUX1.bit.GPIO7 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
        GpioCtrlRegs.GPADIR.bit.GPIO7 = GPIO_INPUT;            /* Set as an input */
        GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = GPIO_ASYNC;          /* Set input as asynchronous */

        /* Setup GPIO_8 as On-Side Hall 3 input */
        GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = GPIO_MUX_GRP_0;      /* Select group 0 */
        GpioCtrlRegs.GPAMUX1.bit.GPIO8 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
        GpioCtrlRegs.GPADIR.bit.GPIO8 = GPIO_INPUT;            /* Set as an input */
        GpioCtrlRegs.GPAQSEL1.bit.GPIO8 = GPIO_ASYNC;          /* Set input as asynchronous */
#endif
        /* Set INPUT_XBAR_REGS for ECAP1,2,3 for GPIO connected to Primary Hall 1,2,3 */
        InputXbarRegs.INPUT7SELECT = CHA_HS1_GPIO_PIN;   /* Configure INPUT 7 (ECAP1) source of Input X-BAR to be Hall 1 GPIO */
        InputXbarRegs.INPUT8SELECT = CHA_HS2_GPIO_PIN;   /* Configure INPUT 8 (ECAP2) source of Input X-BAR to be Hall 2 GPIO */
        InputXbarRegs.INPUT9SELECT = CHA_HS3_GPIO_PIN;   /* Configure INPUT 9 (ECAP3) source of Input X-BAR to be Hall 3 GPIO */

        /* InitECapture - Initialize ECAP1-3 Registers */
        Hall_InitECapRegisters();

        /* Enable ECAP Interrupts for ECAP1,2,3 in the PIE: Group 4 __interrupt 1-3 */
        PieCtrlRegs.PIEIER4.bit.INTx1 = 1;  /* PIE Group 4, INT1 ECAP1 */
        PieCtrlRegs.PIEIER4.bit.INTx2 = 1;  /* PIE Group 4, INT2 ECAP2 */
        PieCtrlRegs.PIEIER4.bit.INTx3 = 1;  /* PIE Group 4, INT3 ECAP3 */

        EDIS;   /* This is needed to disable write to EALLOW protected registers */

        /***********************
        ////// Cross-side ///////
        ***********************/
#if defined(__HALLX_CONFIGURED)
        EALLOW; /* This is needed to write to EALLOW protected registers*/
        /* shall map the cross-side capture (ECAP-3) interrupts to the "Hallx_ISR" function*/
        /* map the ECAP ISRs */
        PieVectTable.ECAP4_INT = &Hallx_1_ISR;   /* Map Hall 1 Cross ISR */
        PieVectTable.ECAP5_INT = &Hallx_2_ISR;   /* Map Hall 2 Cross ISR */
        PieVectTable.ECAP6_INT = &Hallx_3_ISR;   /* Map Hall 3 Cross ISR */
        /* Initialize the GPIO used for ECAP Interrupts ECAP4,5,6 */
        /* Initialize the GPIO Pin Configuration for x-side Halls;
         * CHB_ICC_HS1, CHB_ICC_HS2, CHB_ICC_HS3 */

        #if !defined(DRV8312_DEV_KIT)
        /* MCU HW */

        /* Setup GPIO_9 as Cross-Side Hall 1 input */
        GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = GPIO_MUX_GRP_0;      /* Select group 0 */
        GpioCtrlRegs.GPAMUX1.bit.GPIO9 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
        GpioCtrlRegs.GPADIR.bit.GPIO9 = GPIO_INPUT;            /* Set as an input */
        GpioCtrlRegs.GPAQSEL1.bit.GPIO9 = GPIO_ASYNC;          /* Set input as asynchronous */

        /* Setup GPIO_10 as Cross-Side Hall 2 input */
        GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = GPIO_MUX_GRP_0;      /* Select group 0 */
        GpioCtrlRegs.GPAMUX1.bit.GPIO10 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
        GpioCtrlRegs.GPADIR.bit.GPIO10 = GPIO_INPUT;            /* Set as an input */
        GpioCtrlRegs.GPAQSEL1.bit.GPIO10 = GPIO_ASYNC;          /* Set input as asynchronous */
		
		/* Setup GPIO_11 as Cross-Side Hall 3 input */
        GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = GPIO_MUX_GRP_0;      /* Select group 0 */
        GpioCtrlRegs.GPAMUX1.bit.GPIO11 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
        GpioCtrlRegs.GPADIR.bit.GPIO11 = GPIO_INPUT;            /* Set as an input */
        GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = GPIO_ASYNC;          /* Set input as asynchronous */

        /* Set INPUT_XBAR_REGS for ECAP 4,5 for GPIO connected to Cross Side Hall 1,2 */
        InputXbarRegs.INPUT10SELECT = CHB_ICC_HS1_GPIO_PIN;   /* Configure INPUT 10 (ECAP4) source of Input X-BAR to be tHallx 1 GPIO */
        InputXbarRegs.INPUT11SELECT = CHB_ICC_HS2_GPIO_PIN;   /* Configure INPUT 11 (ECAP5) source of Input X-BAR to be tHallx 2 GPIO */
        InputXbarRegs.INPUT12SELECT = CHB_ICC_HS3_GPIO_PIN;   /* Configure INPUT 11 (ECAP5) source of Input X-BAR to be tHallx 2 GPIO */
        /* InitECapture - Initialize ECAP4-5 Registers */
        Hallx_InitECapRegisters();

        /* Enable ECAP Interrupts for ECAP4,5 in the PIE: Group 4 __interrupt 4-5 */
        PieCtrlRegs.PIEIER4.bit.INTx4 = 1;  /* PIE Group 4, INT1 ECAP4 */
        PieCtrlRegs.PIEIER4.bit.INTx5 = 1;  /* PIE Group 4, INT2 ECAP5 */
        PieCtrlRegs.PIEIER4.bit.INTx6 = 1;  /* PIE Group 4, INT2 ECAP6 */
        #endif

        EDIS;   /* This is needed to disable write to EALLOW protected registers */
#endif
        /**********************
        //////  Common  ///////
        **********************/
        /* Enable CPU INT4 which is connected to ECAP1-6 INT: */
        IER |= M_INT4;   /* Enable CPU INT 4*/

        /* shall initialize the beginning Hall State for the on-side motor*/
        on->Revolutions = 0;
        on->Position = Nvm_State.quadposition.onside; /* copied from Nvm on startup */
#if defined(__HALLX_CONFIGURED)
        /* shall initialize the beginning Hall State for the cross-side motor*/
        cross->Revolutions = 0;
        cross->Position = Nvm_State.quadposition.xside; /* copied from Nvm on startup */
#endif
        Hall_Read(on);
#if defined(__HALLX_CONFIGURED)
        Hallx_Read(cross);
#endif
        Act_CommutationCallback();

    }

	/*  PATH(Hall_Init,B); */
}

/**
    brief Triggers a capture of all on-side hall sensor inputs.

     Purpose:
        Captures hall sensor inputs on discrete input capture pins to
        determine new commutation state

    param[in] p    Pointer to Hall_t hall sensor driver object

     Global Data Referenced:
        #MotorCmd     
        #speed1       
        #EvaRegs      
        #GpioDataRegs 
        #CpuTimer2Regs 
    
    return  void
    
     Preconditions and Assumptions:
         Should only be called during the Hall Interrupt servicing timeframe

*/
void Hall_Read( Hall_t *p )
{
    static uint32_t prevTimestamp = 0;

    /* PATH(Hall_Read,A); */
    if (p)
    {
    	/* PATH(Hall_Read,P); */
    
	    /* shall save off the previous hall state and read and store the new Hall State*/
		p->HallGpioDblBuffer = p->HallGpioBuffer;
	    p->HallGpioBuffer = p->HallGpio;
	    p->HallGpio = ((GpioDataRegs.GPADAT.all & HALL_MASK) >> HALL_SHIFT); /* HallGpio.2-0 = GPIO8-GPIO6*/

	    /* shall correct "illegal" hall state by using the previous hall state instead*/
	    if ((p->HallGpio == 0) || (p->HallGpio == 7))
	    {
	        /* PATH(Hall_Read,B); */
//	        uint16_t u16IlnvalidHallStateLimit = (uint16_t)PDI_Mgr_GetParam(PDI_Monitor_Invalid_Hall_Count_Limit);
		    //if (MotorCmd.IllegalHalls < BIT_ILLEGALHALLSTATE_LIMIT)
	        if(MotorCmd.IllegalHalls < tPdi.u16InvalidHallSensorMonitorCountLimit)
		    {
		        /* PATH(Hall_Read,C); */
		      	MotorCmd.IllegalHalls += 5;	/* Increment bad hall state counter by 5 as per DWM pole config */
		        
		        //if (MotorCmd.IllegalHalls >= BIT_ILLEGALHALLSTATE_LIMIT)
		      	if(MotorCmd.IllegalHalls >= tPdi.u16InvalidHallSensorMonitorCountLimit)
		        {
    		        if(p->HallGpio == 0)          /* Record which bad hall state it is */
    		        {
						/* PATH(Hall_Read,O); */
		        		MotorCmd.LastBadHallState = ALL_ZEROS;
		        	}				       
		        	else
		        	{
						/* PATH(Hall_Read,Q); */    		        	
		        		MotorCmd.LastBadHallState = ALL_ONES;
		        	}
		    	}
		    }
	    }
	    else
	    {    
	        /* PATH(Hall_Read,D); */
//	        uint16_t u16IlllegalHallSequenceLimit = (uint16_t)PDI_Mgr_GetParam(PDI_Monitor_Hall_Sensor_Seq_Illegal_Limit);
	        /*Decrement Illegal Hall state counter as legal Hall state is received*/
			if (MotorCmd.IllegalHalls > 0) 
			{
			    /* PATH(Hall_Read,E); */
			    MotorCmd.IllegalHalls--;
			}
	
			/*Capture the Hall Timing if the transition is a valid transition
			 * Transition is considered valid if for current Hall state, the previous Hall state is as per the commutation table */
            if (((MotorCmd.MotorDirection == CW) && (p->HallGpioBuffer == PrevState_CW[(p->HallGpio) - 1])) ||
                ((MotorCmd.MotorDirection == CCW) && (p->HallGpioBuffer == PrevState_CCW[(p->HallGpio) - 1])))
            {
                /* shall save TimeStamp of the edge detection and increment edge and rotation counters when the hall state transition is valid*/

                /* PATH(Hall_Read,R); */
                p->TimeStamp = CpuTimer2Regs.TIM.all; /* Save Time Stamp for this transition*/
                p->CaptureCount++;                    /* Increment the number of Captures*/

                /*  u16EventStart_SpdFilter variable tracks the valid elements in u32EventRaw_SpdFilter[] Array
                *  When speed is 0 RPM, the prevTimestamp becomes invalid
                *  When speed is 0, u16EventStart_SpdFilter is 0, Do not capture the Hall timing.
                *  When there is 1 valid element in u32EventRaw_SpdFilter[] Array the u16EventStart_SpdFilter is updated to 2
                *  When there are 30 valid element in u32EventRaw_SpdFilter[] Array the u16EventStart_SpdFilter is updated to 31*/
                if (u16EventStart_SpdFilter > 0)
                {
                    /*Verify that Timer is not over-flown*/
                    if (p->TimeStamp < prevTimestamp)
                    {
                        u32EventRaw_SpdFilter[u16EventPos_SpdFilter] = prevTimestamp - p->TimeStamp;
                    }
                    else
                    {
                        /*If the Timer overflows add Initial value of the Timer to the calculated value*/
                        u32EventRaw_SpdFilter[u16EventPos_SpdFilter] = ((int32_t)(prevTimestamp - p->TimeStamp)) + HALL_TIMER_INIT_VALUE;
                    }
                    u16EventPos_SpdFilter++;
                    tSpeed.EventCount++;

                    // Value of u16EventPos_SpdFilter will increment from 0 to 29, after incrementing the value to 30 the value is immediately reset to 0
                    if(u16EventPos_SpdFilter >= SPD_WINDOW_LENGTH)
                    {
                        u16EventPos_SpdFilter = 0;  // Reset index to 0 after last element is updated
                    }
                }

                // Value of u16EventStart_SpdFilter will increment from 0 to 31, value to stay 31 until speed becomes 0
                if (u16EventStart_SpdFilter < SPD_WINDOW_LENGTH_PLUSONE)
                {
                    u16EventStart_SpdFilter++;
                }
                prevTimestamp = p->TimeStamp;   // Copy current time-stamp to previous time-stamp


                /*Decrement the Bad Transition Counter as Valid Transition was received*/
                if ((MotorCmd.BadTransitions > 0) && (MotorCmd.BadTransitions < tPdi.u16HallSensorSeqMonitorIllegalLimit))
                {
                    /* PATH(Hall_Read,J); */
                    MotorCmd.BadTransitions--;
                }
            }
            else
            {
                /*Increment the BadTransition counter as Invalid Transition was received */
                if (MotorCmd.BadTransitions < tPdi.u16HallSensorSeqMonitorIllegalLimit)
                {
                    /* PATH(Hall_Read,M); */
                    MotorCmd.BadTransitions += 5;                   //Increment Bad Transition counter by 5
                }
            }

	    }
    }

	/* PATH(Hall_Read,N); */
}
#if defined(__HALLX_CONFIGURED)
/**
    brief Triggers a capture of all cross-side hall sensor inputs.

     Purpose:
        Captures hall sensor inputs on discrete input capture pins to
        determine speed of cross-side motor

    param[in] p    Pointer to Hall_t hall sensor driver object

     Global Data Referenced:
        #EvbRegs
        #GpioDataRegs
        #CpuTimer2Regs
        #tSpeedx

    return  void

     Preconditions and Assumptions:
        Should only be called during the Hall Interrupt servicing timeframe

*/
void Hallx_Read( Hall_t *p )
{
    tHallWord_t tHalls;

    /* PATH(Hallx_Read,A); */
    tHalls.all = 0;

    if(p)
    {
        /* PATH(Hallx_Read,I); */

        /* shall save off the previous hall state and read and store the new Hall State*/
        p->HallGpioDblBuffer = p->HallGpioBuffer;
        p->HallGpioBuffer = p->HallGpio;
        //p->HallGpio = ((GpioDataRegs.GPBDAT.all & 0x0300) >> 8);

        tHalls.bit.bHall1 = CHB_ICC_HS1_RAW;   /* bit 0 */
        tHalls.bit.bHall2 = CHB_ICC_HS2_RAW;   /* bit 1 */

        p->HallGpio = tHalls.all;

        /* shall capture a Timestamp for this edge and copy it to the cross-side speed computation object*/
        if ((p->HallGpioBuffer & 0x3) == 3)
        {
            /* PATH(Hallx_Read,B); */

            switch (p->HallGpio & 0x3)
            {
                case 1:
                    /* PATH(Hallx_Read,C); */

                    if ((p->HallGpioDblBuffer & 0x3) == 2) /* CCW*/
                    {
                        /* PATH(Hallx_Read,D); */
                        p->TimeStamp = CpuTimer2Regs.TIM.all;
                        tSpeedx.TimeStamp  = p->TimeStamp;
                        tSpeedx.EventCount = 1;
                    }
                    break;
                case 2:
                    /* PATH(Hallx_Read,E); */

                    if ((p->HallGpioDblBuffer & 0x3) == 1) /* CW*/
                    {
                        /* PATH(Hallx_Read,F); */
                        p->TimeStamp = CpuTimer2Regs.TIM.all;
                        tSpeedx.TimeStamp  = p->TimeStamp;
                        tSpeedx.EventCount = 1;
                    }
                    break;
                default:
                    /* PATH(Hallx_Read,G); */
                    break;
            }
        }

    }
    /* PATH(Hallx_Read,H); */
}
#endif

/**
    brief Computes position values from hall sensor inputs.

     Purpose:
        Computes position values from local or cross-channel hall sensor inputs. To be
        invoked regularly from scheduler.

    param[in] p    Pointer to Hall_t hall sensor driver object

     Global Data Referenced:
         None.
    
    return  void
    
     Preconditions and Assumptions:
         None.
*/
void Hall_ComputePosition( Hall_t * p )
{
    /* PATH(Hall_ComputePosition,A); */
	
	if (p)
	{
		/* PATH(Hall_ComputePosition,P); */	
	
	    /* shall determine if there was a valid hall transition*/
	    if ((p->HallGpio & 0x3) != (p->HallGpioBuffer & 0x3))
	    {
	        /* PATH(Hall_ComputePosition,B); */
	
	        /* shall use the current state of the hall inputs to determine direction and position*/

			/* Non-Compliance: The following switch statement has no default case
			   Justification: All possible values are enumerated as explicit cases in the switch, and
			      the LLRs do not imply a default case.
			*/
	        switch (p->HallGpio & 0x3)
	        {
	            case 0:
	                /* PATH(Hall_ComputePosition,C); */
	                p->Direction = 0;
	                p->Armed = true;
	                break;
	            case 1:
	                /* PATH(Hall_ComputePosition,D); */
	
	                if (p->Armed == true)
	                {
	                    /* PATH(Hall_ComputePosition,E); */
#if defined(DRV8312_DEV_KIT)
	                    /* Dev Kit is opposite MCU HW */
	                    p->Direction = -1;
#else /* MCU HW */
	                    /* MCU Hardware Direction */
	                    p->Direction = 1;
#endif
	                }
	                else
	                {
	                    /* PATH(Hall_ComputePosition,F); */
	                    p->Direction = 0;
	                }
	                break;
	            case 2:
	                /* PATH(Hall_ComputePosition,G); */
	
	                if (p->Armed == true)
	                {
	                    /* PATH(Hall_ComputePosition,H); */
#if defined(DRV8312_DEV_KIT)
	                    /* Dev Kit is opposite MCU HW */
	                    p->Direction = 1;
#else /* MCU HW */
	                    /* MCU Hardware Direction */
	                    p->Direction = -1;
#endif
	                }
	                else
	                {
	                    /* PATH(Hall_ComputePosition,I); */
	                    p->Direction = 0;
	                }
	                break;
	            case 3:
	                /* PATH(Hall_ComputePosition,J); */
	
	                if (p->Armed == true)
	                {
	                    /* PATH(Hall_ComputePosition,K); */
	                    p->Position += p->Direction;
	                    p->Direction = 0;
	                    p->Armed = false;
	
	                    if ((p->Position <= -32768) || (p->Position >= 32767))
	                    {
	                        /* PATH(Hall_ComputePosition,L); */
	                        p->Position = 0;
	                    }
	
	                    p->Revolutions++;
	
	                    if (p->Revolutions >= 32767)
	                    {
	                        /* PATH(Hall_ComputePosition,M); */
	                        p->Revolutions = 2;
	                    }
	                }
	                break;
	        }
	    }
	}

	/* PATH(Hall_ComputePosition,O); */
}


/*
    brief Initialize ECAP modules tied to On-Side Hall Effect inputs

    Purpose:
        Configures On-Side ECAPs for motor control.


    param[in]

    Global Data Referenced:


    return  void

    Preconditions and Assumptions:
        This function should be called only once, prior to the scheduler loop.

*/
void Hall_InitECapRegisters(void)
{
    /* ECAP 1 Configuration */
    /* Set ECAP1 ECCTL1 to use 2 capture events on Rising and Falling edge of HALL 1 Signal */
    ECap1Regs.ECCTL1.bit.CAP1POL = ECAP_POLARITY_RISING_EDGE;        /* CAP1 Event tied to Rising Edge */
    ECap1Regs.ECCTL1.bit.CTRRST1 = ECAP_RESET_CTR_ON_CAP_EVENT;      /* Use difference for timing rather than absolute timing. TSCTR is reset every capture event */
    ECap1Regs.ECCTL1.bit.CAP2POL = ECAP_POLARITY_FALLING_EDGE;       /* CAP2 Event tied to Falling Edge */
    ECap1Regs.ECCTL1.bit.CTRRST2 = ECAP_RESET_CTR_ON_CAP_EVENT;      /* Use difference for timing rather than absolute timing. */
    ECap1Regs.ECCTL1.bit.CAPLDEN = ECAP_CAP_LOAD_ENABLE;             /* Enable CAP1-4 Register loads at capture Event. VLJ uses Timer 2 for speed measurements. */
    ECap1Regs.ECCTL1.bit.PRESCALE = ECAP_DISABLE_PRESCALER;          /* Bypass Prescaler. Don't skip any capture events. No pulse train. */
    ECap1Regs.ECCTL1.bit.FREE_SOFT = ECAP_FREE_SOFT_FREE_RUN;        /* Timer Free Run */

    /* Set ECAP1 ECCTL2 Register Configuration */
    ECap1Regs.ECCTL2.bit.CONT_ONESHT = ECAP_CONTINUOUS_MODE;            /* Set to Continuous Mode. */
    ECap1Regs.ECCTL2.bit.STOP_WRAP = ECAP_STOP_WRAP_AFTER_CAP2_EVENT;   /* Wrap after Capture Event 2 in Continuous Mode */
    ECap1Regs.ECCTL2.bit.REARM = 0U;                                    /* Set to no effect */
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = ECAP_TSCTRSTOP_START;              /* Start the timer */
    ECap1Regs.ECCTL2.bit.SYNCI_EN = ECAP_SYNCCI_EN_DISABLE;             /* Disable SYNC-In */
    ECap1Regs.ECCTL2.bit.SYNCO_SEL = ECAP_SYCNO_SEL_DISABLE;            /* Disable SYNC-Out signal */
    ECap1Regs.ECCTL2.bit.CAP_APWM = ECAP_CAP_APWM_CAPTURE_MODE;         /* Set for Capture Mode */

    /* Configure ECAP1 ECEINT */
    ECap1Regs.ECEINT.bit.CEVT1 = ECAP_INTERRUPT_ENABLE;            /* Enable Capture Event 1 Interrupt - Rising Edge */
    ECap1Regs.ECEINT.bit.CEVT2 = ECAP_INTERRUPT_ENABLE;            /* Enable Capture Event 2 Interrupt - Falling Edge */
    ECap1Regs.ECEINT.bit.CEVT3 = ECAP_INTERRUPT_DISABLE;           /* Disable Capture Event 3 Interrupt - N/A */
    ECap1Regs.ECEINT.bit.CEVT4 = ECAP_INTERRUPT_DISABLE;           /* Disable Capture Event 4 Interrupt - N/A */
    ECap1Regs.ECEINT.bit.CTROVF = ECAP_INTERRUPT_DISABLE;          /* Disable Interrupt on Counter Overflow */
    ECap1Regs.ECEINT.bit.CTR_EQ_PRD = ECAP_INTERRUPT_DISABLE;      /* Disable Interrupt on Period Match  - only applicable in APWM mode */
    ECap1Regs.ECEINT.bit.CTR_EQ_CMP = ECAP_INTERRUPT_DISABLE;      /* Disable Interrupt on Compare Match  - only applicable in APWM mode */

    /* ECAP 2 Configuration */
    /* Set ECAP2 ECCTL1 to use 2 capture events on Rising and Falling edge of HALL 2 Signal */
    ECap2Regs.ECCTL1.bit.CAP1POL = ECAP_POLARITY_RISING_EDGE;        /* CAP1 Event tied to Rising Edge */
    ECap2Regs.ECCTL1.bit.CTRRST1 = ECAP_RESET_CTR_ON_CAP_EVENT;      /* Use difference for timing rather than absolute timing. TSCTR is reset every capture event */
    ECap2Regs.ECCTL1.bit.CAP2POL = ECAP_POLARITY_FALLING_EDGE;       /* CAP2 Event tied to Falling Edge */
    ECap2Regs.ECCTL1.bit.CTRRST2 = ECAP_RESET_CTR_ON_CAP_EVENT;      /* Use difference for timing rather than absolute timing. */
    ECap2Regs.ECCTL1.bit.CAPLDEN = ECAP_CAP_LOAD_ENABLE;             /* Enable CAP1-4 Register loads at capture Event. VLJ uses Timer 2 for speed measurements. */
    ECap2Regs.ECCTL1.bit.PRESCALE = ECAP_DISABLE_PRESCALER;          /* Bypass Prescaler. Don't skip any capture events. No pulse train. */
    ECap2Regs.ECCTL1.bit.FREE_SOFT = ECAP_FREE_SOFT_FREE_RUN;        /* Timer Free Run */

    /* Set ECAP2 ECCTL2 Register Configuration */
    ECap2Regs.ECCTL2.bit.CONT_ONESHT = ECAP_CONTINUOUS_MODE;            /* Set to Continuous Mode. */
    ECap2Regs.ECCTL2.bit.STOP_WRAP = ECAP_STOP_WRAP_AFTER_CAP2_EVENT;   /* Wrap after Capture Event 2 in Continuous Mode */
    ECap2Regs.ECCTL2.bit.REARM = 0U;                                    /* Set to no effect */
    ECap2Regs.ECCTL2.bit.TSCTRSTOP = ECAP_TSCTRSTOP_START;              /* Start the timer */
    ECap2Regs.ECCTL2.bit.SYNCI_EN = ECAP_SYNCCI_EN_DISABLE;             /* Disable SYNC-In */
    ECap2Regs.ECCTL2.bit.SYNCO_SEL = ECAP_SYCNO_SEL_DISABLE;            /* Disable SYNC-Out signal */
    ECap2Regs.ECCTL2.bit.CAP_APWM = ECAP_CAP_APWM_CAPTURE_MODE;         /* Set for Capture Mode */

    /* Configure ECAP2 ECEINT */
    ECap2Regs.ECEINT.bit.CEVT1 = ECAP_INTERRUPT_ENABLE;            /* Enable Capture Event 1 Interrupt - Rising Edge */
    ECap2Regs.ECEINT.bit.CEVT2 = ECAP_INTERRUPT_ENABLE;            /* Enable Capture Event 2 Interrupt - Falling Edge */
    ECap2Regs.ECEINT.bit.CEVT3 = ECAP_INTERRUPT_DISABLE;           /* Disable Capture Event 3 Interrupt - N/A */
    ECap2Regs.ECEINT.bit.CEVT4 = ECAP_INTERRUPT_DISABLE;           /* Disable Capture Event 4 Interrupt - N/A */
    ECap2Regs.ECEINT.bit.CTROVF = ECAP_INTERRUPT_DISABLE;          /* Disable Interrupt on Counter Overflow */
    ECap2Regs.ECEINT.bit.CTR_EQ_PRD = ECAP_INTERRUPT_DISABLE;      /* Disable Interrupt on Period Match  - only applicable in APWM mode */
    ECap2Regs.ECEINT.bit.CTR_EQ_CMP = ECAP_INTERRUPT_DISABLE;      /* Disable Interrupt on Compare Match  - only applicable in APWM mode */

    /* ECAP 3 Configuration */
    /* Set ECAP3 ECCTL1 to use 2 capture events on Rising and Falling edge of HALL 3 Signal */
    ECap3Regs.ECCTL1.bit.CAP1POL = ECAP_POLARITY_RISING_EDGE;        /* CAP1 Event tied to Rising Edge */
    ECap3Regs.ECCTL1.bit.CTRRST1 = ECAP_RESET_CTR_ON_CAP_EVENT;      /* Use difference for timing rather than absolute timing. TSCTR is reset every capture event */
    ECap3Regs.ECCTL1.bit.CAP2POL = ECAP_POLARITY_FALLING_EDGE;       /* CAP2 Event tied to Falling Edge */
    ECap3Regs.ECCTL1.bit.CTRRST2 = ECAP_RESET_CTR_ON_CAP_EVENT;      /* Use difference for timing rather than absolute timing. */
    ECap3Regs.ECCTL1.bit.CAPLDEN = ECAP_CAP_LOAD_ENABLE;             /* Enable CAP1-4 Register loads at capture Event. VLJ uses Timer 2 for speed measurements. */
    ECap3Regs.ECCTL1.bit.PRESCALE = ECAP_DISABLE_PRESCALER;          /* Bypass Prescaler. Don't skip any capture events. No pulse train. */
    ECap3Regs.ECCTL1.bit.FREE_SOFT = ECAP_FREE_SOFT_FREE_RUN;        /* Timer Free Run */

    /* Set ECAP3 ECCTL2 Register Configuration */
    ECap3Regs.ECCTL2.bit.CONT_ONESHT = ECAP_CONTINUOUS_MODE;            /* Set to Continuous Mode. */
    ECap3Regs.ECCTL2.bit.STOP_WRAP = ECAP_STOP_WRAP_AFTER_CAP2_EVENT;   /* Wrap after Capture Event 2 in Continuous Mode */
    ECap3Regs.ECCTL2.bit.REARM = 0U;                                    /* Set to no effect */
    ECap3Regs.ECCTL2.bit.TSCTRSTOP = ECAP_TSCTRSTOP_START;              /* Start the timer */
    ECap3Regs.ECCTL2.bit.SYNCI_EN = ECAP_SYNCCI_EN_DISABLE;             /* Disable SYNC-In */
    ECap3Regs.ECCTL2.bit.SYNCO_SEL = ECAP_SYCNO_SEL_DISABLE;            /* Disable SYNC-Out signal */
    ECap3Regs.ECCTL2.bit.CAP_APWM = ECAP_CAP_APWM_CAPTURE_MODE;         /* Set for Capture Mode */

    /* Configure ECAP3 ECEINT */
    ECap3Regs.ECEINT.bit.CEVT1 = ECAP_INTERRUPT_ENABLE;            /* Enable Capture Event 1 Interrupt - Rising Edge */
    ECap3Regs.ECEINT.bit.CEVT2 = ECAP_INTERRUPT_ENABLE;            /* Enable Capture Event 2 Interrupt - Falling Edge */
    ECap3Regs.ECEINT.bit.CEVT3 = ECAP_INTERRUPT_DISABLE;           /* Disable Capture Event 3 Interrupt - N/A */
    ECap3Regs.ECEINT.bit.CEVT4 = ECAP_INTERRUPT_DISABLE;           /* Disable Capture Event 4 Interrupt - N/A */
    ECap3Regs.ECEINT.bit.CTROVF = ECAP_INTERRUPT_DISABLE;          /* Disable Interrupt on Counter Overflow */
    ECap3Regs.ECEINT.bit.CTR_EQ_PRD = ECAP_INTERRUPT_DISABLE;      /* Disable Interrupt on Period Match  - only applicable in APWM mode */
    ECap3Regs.ECEINT.bit.CTR_EQ_CMP = ECAP_INTERRUPT_DISABLE;      /* Disable Interrupt on Compare Match  - only applicable in APWM mode */
}

#if defined(__HALLX_CONFIGURED)
/*
    brief Initialize ECAP modules tied to Cross-Side Hall Effect inputs

    Purpose:
        Configures Cross-Side ECAPs for motor control.


    param[in]

    Global Data Referenced:


    return  void

    Preconditions and Assumptions:
        This function should be called only once, prior to the scheduler loop.

*/
void Hallx_InitECapRegisters(void)
{
    /* ECAP 4 Configuration */
    /* Set ECAP4 ECCTL1 to use 2 capture events on Rising and Falling edge of HALLx 1 Signal */
    ECap4Regs.ECCTL1.bit.CAP1POL = ECAP_POLARITY_RISING_EDGE;        /* CAP1 Event tied to Rising Edge */
    ECap4Regs.ECCTL1.bit.CTRRST1 = ECAP_RESET_CTR_ON_CAP_EVENT;      /* Use difference for timing rather than absolute timing. TSCTR is reset every capture event */
    ECap4Regs.ECCTL1.bit.CAP2POL = ECAP_POLARITY_FALLING_EDGE;       /* CAP2 Event tied to Falling Edge */
    ECap4Regs.ECCTL1.bit.CTRRST2 = ECAP_RESET_CTR_ON_CAP_EVENT;      /* Use difference for timing rather than absolute timing. */
    ECap4Regs.ECCTL1.bit.CAPLDEN = ECAP_CAP_LOAD_ENABLE;             /* Enable CAP1-4 Register loads at capture Event. VLJ uses Timer 2 for speed measurements. */
    ECap4Regs.ECCTL1.bit.PRESCALE = ECAP_DISABLE_PRESCALER;          /* Bypass Prescaler. Don't skip any capture events. No pulse train. */
    ECap4Regs.ECCTL1.bit.FREE_SOFT = ECAP_FREE_SOFT_FREE_RUN;        /* Timer Free Run */

    /* Set ECAP4 ECCTL2 Register Configuration */
    ECap4Regs.ECCTL2.bit.CONT_ONESHT = ECAP_CONTINUOUS_MODE;            /* Set to Continuous Mode. */
    ECap4Regs.ECCTL2.bit.STOP_WRAP = ECAP_STOP_WRAP_AFTER_CAP2_EVENT;   /* Wrap after Capture Event 2 in Continuous Mode */
    ECap4Regs.ECCTL2.bit.REARM = 0U;                                    /* Set to no effect */
    ECap4Regs.ECCTL2.bit.TSCTRSTOP = ECAP_TSCTRSTOP_START;              /* Start the timer */
    ECap4Regs.ECCTL2.bit.SYNCI_EN = ECAP_SYNCCI_EN_DISABLE;             /* Disable SYNC-In */
    ECap4Regs.ECCTL2.bit.SYNCO_SEL = ECAP_SYCNO_SEL_DISABLE;            /* Disable SYNC-Out signal */
    ECap4Regs.ECCTL2.bit.CAP_APWM = ECAP_CAP_APWM_CAPTURE_MODE;         /* Set for Capture Mode */

    /* Configure ECAP4 ECEINT */
    ECap4Regs.ECEINT.bit.CEVT1 = ECAP_INTERRUPT_ENABLE;            /* Enable Capture Event 1 Interrupt - Rising Edge */
    ECap4Regs.ECEINT.bit.CEVT2 = ECAP_INTERRUPT_ENABLE;            /* Enable Capture Event 2 Interrupt - Falling Edge */
    ECap4Regs.ECEINT.bit.CEVT3 = ECAP_INTERRUPT_DISABLE;           /* Disable Capture Event 3 Interrupt - N/A */
    ECap4Regs.ECEINT.bit.CEVT4 = ECAP_INTERRUPT_DISABLE;           /* Disable Capture Event 4 Interrupt - N/A */
    ECap4Regs.ECEINT.bit.CTROVF = ECAP_INTERRUPT_DISABLE;          /* Disable Interrupt on Counter Overflow */
    ECap4Regs.ECEINT.bit.CTR_EQ_PRD = ECAP_INTERRUPT_DISABLE;      /* Disable Interrupt on Period Match  - only applicable in APWM mode */
    ECap4Regs.ECEINT.bit.CTR_EQ_CMP = ECAP_INTERRUPT_DISABLE;      /* Disable Interrupt on Compare Match  - only applicable in APWM mode */

    /* ECAP 5 Configuration */
    /* Set ECAP5 ECCTL1 to use 2 capture events on Rising and Falling edge of HALLx 2 Signal */
    ECap5Regs.ECCTL1.bit.CAP1POL = ECAP_POLARITY_RISING_EDGE;        /* CAP1 Event tied to Rising Edge */
    ECap5Regs.ECCTL1.bit.CTRRST1 = ECAP_RESET_CTR_ON_CAP_EVENT;      /* Use difference for timing rather than absolute timing. TSCTR is reset every capture event */
    ECap5Regs.ECCTL1.bit.CAP2POL = ECAP_POLARITY_FALLING_EDGE;       /* CAP2 Event tied to Falling Edge */
    ECap5Regs.ECCTL1.bit.CTRRST2 = ECAP_RESET_CTR_ON_CAP_EVENT;      /* Use difference for timing rather than absolute timing. */
    ECap5Regs.ECCTL1.bit.CAPLDEN = ECAP_CAP_LOAD_ENABLE;             /* Enable CAP1-4 Register loads at capture Event. VLJ uses Timer 2 for speed measurements. */
    ECap5Regs.ECCTL1.bit.PRESCALE = ECAP_DISABLE_PRESCALER;          /* Bypass Prescaler. Don't skip any capture events. No pulse train. */
    ECap5Regs.ECCTL1.bit.FREE_SOFT = ECAP_FREE_SOFT_FREE_RUN;        /* Timer Free Run */

    /* Set ECAP5 ECCTL2 Register Configuration */
    ECap5Regs.ECCTL2.bit.CONT_ONESHT = ECAP_CONTINUOUS_MODE;            /* Set to Continuous Mode. */
    ECap5Regs.ECCTL2.bit.STOP_WRAP = ECAP_STOP_WRAP_AFTER_CAP2_EVENT;   /* Wrap after Capture Event 2 in Continuous Mode */
    ECap5Regs.ECCTL2.bit.REARM = 0U;                                    /* Set to no effect */
    ECap5Regs.ECCTL2.bit.TSCTRSTOP = ECAP_TSCTRSTOP_START;              /* Start the timer */
    ECap5Regs.ECCTL2.bit.SYNCI_EN = ECAP_SYNCCI_EN_DISABLE;             /* Disable SYNC-In */
    ECap5Regs.ECCTL2.bit.SYNCO_SEL = ECAP_SYCNO_SEL_DISABLE;            /* Disable SYNC-Out signal */
    ECap5Regs.ECCTL2.bit.CAP_APWM = ECAP_CAP_APWM_CAPTURE_MODE;         /* Set for Capture Mode */

    /* Configure ECAP5 ECEINT */
    ECap5Regs.ECEINT.bit.CEVT1 = ECAP_INTERRUPT_ENABLE;            /* Enable Capture Event 1 Interrupt - Rising Edge */
    ECap5Regs.ECEINT.bit.CEVT2 = ECAP_INTERRUPT_ENABLE;            /* Enable Capture Event 2 Interrupt - Falling Edge */
    ECap5Regs.ECEINT.bit.CEVT3 = ECAP_INTERRUPT_DISABLE;           /* Disable Capture Event 3 Interrupt - N/A */
    ECap5Regs.ECEINT.bit.CEVT4 = ECAP_INTERRUPT_DISABLE;           /* Disable Capture Event 4 Interrupt - N/A */
    ECap5Regs.ECEINT.bit.CTROVF = ECAP_INTERRUPT_DISABLE;          /* Disable Interrupt on Counter Overflow */
    ECap5Regs.ECEINT.bit.CTR_EQ_PRD = ECAP_INTERRUPT_DISABLE;      /* Disable Interrupt on Period Match  - only applicable in APWM mode */
    ECap5Regs.ECEINT.bit.CTR_EQ_CMP = ECAP_INTERRUPT_DISABLE;      /* Disable Interrupt on Compare Match  - only applicable in APWM mode */

    /* ECAP 6 Configuration */
    /* Set ECAP6 ECCTL1 to use 2 capture events on Rising and Falling edge of HALLx 2 Signal */
    ECap6Regs.ECCTL1.bit.CAP1POL = ECAP_POLARITY_RISING_EDGE;        /* CAP1 Event tied to Rising Edge */
    ECap6Regs.ECCTL1.bit.CTRRST1 = ECAP_RESET_CTR_ON_CAP_EVENT;      /* Use difference for timing rather than absolute timing. TSCTR is reset every capture event */
    ECap6Regs.ECCTL1.bit.CAP2POL = ECAP_POLARITY_FALLING_EDGE;       /* CAP2 Event tied to Falling Edge */
    ECap6Regs.ECCTL1.bit.CTRRST2 = ECAP_RESET_CTR_ON_CAP_EVENT;      /* Use difference for timing rather than absolute timing. */
    ECap6Regs.ECCTL1.bit.CAPLDEN = ECAP_CAP_LOAD_ENABLE;             /* Enable CAP1-4 Register loads at capture Event. VLJ uses Timer 2 for speed measurements. */
    ECap6Regs.ECCTL1.bit.PRESCALE = ECAP_DISABLE_PRESCALER;          /* Bypass Prescaler. Don't skip any capture events. No pulse train. */
    ECap6Regs.ECCTL1.bit.FREE_SOFT = ECAP_FREE_SOFT_FREE_RUN;        /* Timer Free Run */

    /* Set ECAP6 ECCTL2 Register Configuration */
    ECap6Regs.ECCTL2.bit.CONT_ONESHT = ECAP_CONTINUOUS_MODE;            /* Set to Continuous Mode. */
    ECap6Regs.ECCTL2.bit.STOP_WRAP = ECAP_STOP_WRAP_AFTER_CAP2_EVENT;   /* Wrap after Capture Event 2 in Continuous Mode */
    ECap6Regs.ECCTL2.bit.REARM = 0U;                                    /* Set to no effect */
    ECap6Regs.ECCTL2.bit.TSCTRSTOP = ECAP_TSCTRSTOP_START;              /* Start the timer */
    ECap6Regs.ECCTL2.bit.SYNCI_EN = ECAP_SYNCCI_EN_DISABLE;             /* Disable SYNC-In */
    ECap6Regs.ECCTL2.bit.SYNCO_SEL = ECAP_SYCNO_SEL_DISABLE;            /* Disable SYNC-Out signal */
    ECap6Regs.ECCTL2.bit.CAP_APWM = ECAP_CAP_APWM_CAPTURE_MODE;         /* Set for Capture Mode */

    /* Configure ECAP6 ECEINT */
    ECap6Regs.ECEINT.bit.CEVT1 = ECAP_INTERRUPT_ENABLE;            /* Enable Capture Event 1 Interrupt - Rising Edge */
    ECap6Regs.ECEINT.bit.CEVT2 = ECAP_INTERRUPT_ENABLE;            /* Enable Capture Event 2 Interrupt - Falling Edge */
    ECap6Regs.ECEINT.bit.CEVT3 = ECAP_INTERRUPT_DISABLE;           /* Disable Capture Event 3 Interrupt - N/A */
    ECap6Regs.ECEINT.bit.CEVT4 = ECAP_INTERRUPT_DISABLE;           /* Disable Capture Event 4 Interrupt - N/A */
    ECap6Regs.ECEINT.bit.CTROVF = ECAP_INTERRUPT_DISABLE;          /* Disable Interrupt on Counter Overflow */
    ECap6Regs.ECEINT.bit.CTR_EQ_PRD = ECAP_INTERRUPT_DISABLE;      /* Disable Interrupt on Period Match  - only applicable in APWM mode */
    ECap6Regs.ECEINT.bit.CTR_EQ_CMP = ECAP_INTERRUPT_DISABLE;      /* Disable Interrupt on Compare Match  - only applicable in APWM mode */
}
#endif
/**
    brief Interrupt handler for On-Side Hall sensor inputs.

     Purpose:
        Interrupt handler for On-Side Hall sensor inputs.

     Global Data Referenced:
        #MotorCmd 
        #EvaRegs 
        #PieCtrlRegs
    
    return  void
    
     Preconditions and Assumptions:
        Is not to be called by any other function, only by a on-side hall interrupt.
*/
void Hall_ISR( void )
{
    /* PATH(Hall_ISR,A); */

    /* shall command a read of the current Hall state.*/
    Hall_Read(&tHall);

	if((Timer_IsSet(&MotorTimer) == true) && (Timer_IsExpired(&MotorTimer) == true))
	{
    	/*PATH(Hall_ISR,C);*/
    	Act_CommutationCallback();
	}
    /* shall track on-side flap position by hall count*/
    Hall_ComputePosition(&tHall);

    /* PATH(Hall_ISR,B); */
}

#if defined(__HALLX_CONFIGURED)
/**
    brief Cross-side hall sensor interrupt handler

     Purpose:
        Interrupt handler for Cross-side hall sensor inputs.

     Global Data Referenced:
        #xsideCount
        #EvbRegs
        #PieCtrlRegs

    return  void

     Preconditions and Assumptions:
         Is not to be called by any other function, only by a cross-side hall interrupt.

*/
void Hallx_ISR( void )
{
    /* PATH(Hallx_ISR,A); */

    /* shall call the Hallx_Read function to signal an edge was
          detected to the Actuation Manager.*/
    Hallx_Read(&tHallx);

    /* shall track cross-side flap position by hall count*/
    Hall_ComputePosition(&tHallx);

    /* PATH(Hallx_ISR,B); */
}
#endif
/**
    brief Interrupt handler for On-Side Hall 1 Sensor Input

     Purpose:
        Interrupt handler for On-Side Hall 1 input

     Global Data Referenced:

    return  void

     Preconditions and Assumptions:
        Is not to be called by any other function, only by an on-side hall interrupt.
*/
__interrupt void Hall_1_ISR( void )
{
    tHall.tXintFlag.all = 0U;           /* Clear previous flags for External Interrupts. */
    tHall.tXintFlag.bit.bHall1 = TRUE;  /* Save Hall 1 as flagging interrupt */

    Hall_ISR();

    /* Save Period of P_HS1_IN_N - Hall 1 */
    tSpeed.u32ECapPeriod = ((ECap1Regs.CAP1 + ECap1Regs.CAP2) / 6);

    /* Clear interrupts and arm ECAP */
    ECap1Regs.ECCLR.bit.CEVT1 = 1;
    ECap1Regs.ECCLR.bit.CEVT2 = 1;
    ECap1Regs.ECCLR.bit.INT = 1;

    /* Acknowledge the Interrupt to continue getting them */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
/**
    brief Interrupt handler for On-Side Hall 2 Sensor Input

     Purpose:
        Interrupt handler for On-Side Hall 2 input

     Global Data Referenced:

    return  void

     Preconditions and Assumptions:
        Is not to be called by any other function, only by an on-side hall interrupt.
*/
__interrupt void Hall_2_ISR( void )
{
    tHall.tXintFlag.all = 0U;           /* Clear previous flags for External Interrupts. */
    tHall.tXintFlag.bit.bHall2 = TRUE;  /* Save Hall 2 as flagging interrupt */

    Hall_ISR();

    /* Clear interrupts and arm ECAP */
    ECap2Regs.ECCLR.bit.CEVT1 = 1;
    ECap2Regs.ECCLR.bit.CEVT2 = 1;
    ECap2Regs.ECCLR.bit.INT = 1;

    /* Acknowledge the Interrupt to continue getting them */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
/**
    brief Interrupt handler for On-Side Hall 3 Sensor Input

     Purpose:
        Interrupt handler for On-Side Hall 3 input

     Global Data Referenced:

    return  void

     Preconditions and Assumptions:
        Is not to be called by any other function, only by an on-side hall interrupt.
*/
__interrupt void Hall_3_ISR( void )
{
    tHall.tXintFlag.all = 0U;           /* Clear previous flags for External Interrupts. */
    tHall.tXintFlag.bit.bHall3 = TRUE;  /* Save Hall 3 as flagging interrupt */

    Hall_ISR();

    /* Clear interrupts and arm ECAP */
    ECap3Regs.ECCLR.bit.CEVT1 = 1;
    ECap3Regs.ECCLR.bit.CEVT2 = 1;
    ECap3Regs.ECCLR.bit.INT = 1;

    /* Acknowledge the External Interrupt to continue getting them */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
#if defined(__HALLX_CONFIGURED)
/**
    brief Interrupt handler for Cross-Side Hall 1 Sensor Input

     Purpose:
        Interrupt handler for Cross-Side Hall 2 input

     Global Data Referenced:

    return  void

     Preconditions and Assumptions:
        Is not to be called by any other function, only by an x-side hall interrupt.
*/
__interrupt void Hallx_1_ISR( void )
{
    tHallx.tXintFlag.all = 0U;           /* Clear previous flags for External Interrupts. */
    tHallx.tXintFlag.bit.bHall1 = TRUE;  /* Save Hall 1 as flagging interrupt */

    /* Save Period of X_HS1_IN_N - Hall 1 */
    tSpeedx.u32ECapPeriod = ((ECap4Regs.CAP1 + ECap4Regs.CAP2) / 6);

    Hallx_ISR();

    /* Clear interrupts and arm ECAP */
    ECap4Regs.ECCLR.bit.CEVT1 = 1;
    ECap4Regs.ECCLR.bit.CEVT2 = 1;
    ECap4Regs.ECCLR.bit.INT = 1;

    /* Acknowledge the External Interrupt to continue getting them */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
/**
    brief Interrupt handler for Cross-Side Hall 2 Sensor Input

     Purpose:
        Interrupt handler for Cross-Side Hall 2 input

     Global Data Referenced:

    return  void

     Preconditions and Assumptions:
        Is not to be called by any other function, only by an x-side hall interrupt.
*/
__interrupt void Hallx_2_ISR( void )
{
    tHallx.tXintFlag.all = 0U;           /* Clear previous flags for External Interrupts. */
    tHallx.tXintFlag.bit.bHall2 = TRUE;  /* Save Hall 2 as flagging interrupt */

    Hallx_ISR();

    /* Clear interrupts and arm ECAP */
    ECap5Regs.ECCLR.bit.CEVT1 = 1;
    ECap5Regs.ECCLR.bit.CEVT2 = 1;
    ECap5Regs.ECCLR.bit.INT = 1;

    /* Acknowledge the External Interrupt to continue getting them */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
/**
    brief Interrupt handler for Cross-Side Hall 3 Sensor Input

     Purpose:
        Interrupt handler for Cross-Side Hall 3 input

     Global Data Referenced:

    return  void

     Preconditions and Assumptions:
        Is not to be called by any other function, only by an x-side hall interrupt.
*/
__interrupt void Hallx_3_ISR( void )
{
    tHallx.tXintFlag.all = 0U;           /* Clear previous flags for External Interrupts. */
    tHallx.tXintFlag.bit.bHall3 = TRUE;  /* Save Hall 3 as flagging interrupt */

    Hallx_ISR();

    /* Clear interrupts and arm ECAP */
    ECap6Regs.ECCLR.bit.CEVT1 = 1;
    ECap6Regs.ECCLR.bit.CEVT2 = 1;
    ECap6Regs.ECCLR.bit.INT = 1;

    /* Acknowledge the External Interrupt to continue getting them */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
#endif

/* end hallsensor.c*/

