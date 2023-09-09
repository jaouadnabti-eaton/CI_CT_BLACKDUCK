/****************************************************************************************************
*  File name: pbitscheduler.c
*
*  Purpose: PBIT Scheduler.
*  This file implements the task scheduler for the PBIT section of software.
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

#include "timer.h"
#include "adc.h"
#include "gpio.h"
#include "bit.h"
#include "bitmonitors.h"
#include "parameter.h"
#include "actuation.h"
#include "motor.h"
#include "parameter.h"
#include "rvdt.h"
#include "mcu.h"
#include "spi.h"
#include "nvm.h"
#include "DFT.h"
#include "icc.h"

void PBitScheduler_Initialize(void);
void PBitScheduler_frame0(void);
void PBitScheduler_frame1(void);
void PBitScheduler_frame2(void);
void PBitScheduler_frame3(void);
void PBitScheduler_frame4(void);
void PBitScheduler_frame5(void);
void PBitScheduler_frame6(void);
void PBitScheduler_frame7(void);
bool PBitScheduler_terminate(void);
void RiggingFaultCheck(void);
extern void main_RunAsw(void);

Timer_t PBit_Timer = TIMER_DEFAULTS;
Uint16 PBitMsTimer = 0;

/*
    brief Alternate scheduler implementation used only during PBIT.

    Purpose:
        This routine configures/operates the PBIT scheduler.

    return void

*/
bool PBitScheduler(void)
{
    bool status = false;

    /* shall exercise the 200ms timer monitor (54) during one of the 8 scheduler frames, while "on ground" (Moved to PBIT and IBIT only) */
    /* PATH(PBitScheduler,A); */

    PBitScheduler_Initialize();

    /* Main loop */
    while (Timer_IsExpired(&PBit_Timer) == false)
    {
        /* PATH(PBitScheduler,B); */

        /* shall wait for 125 usec master frame timer to trip */
        while (Timer_MasterFrameCount == 0)
        {
        	/* PATH(PBitScheduler,P); */
        }

        Adc_Sample_motor_phase_currents();

        if(DFT_Tx_block == false)
        {
            DFT_TxSrvc();
        }

        Icc_RxMsg();
        Icc_TxSrvc();

        /* shall schedule other actions in one or more of eight scheduler frames */
        switch (subframe)
        {
            case 0:
                /* PATH(PBitScheduler,C); */
                PBitScheduler_frame0();
                break;
            case 1:
                /* PATH(PBitScheduler,D); */
                PBitScheduler_frame1();
                break;
            case 2:
                /* PATH(PBitScheduler,E); */
                PBitScheduler_frame2();
                break;
            case 3:
                /* PATH(PBitScheduler,F); */
                PBitScheduler_frame3();
                break;
            case 4:
                /* PATH(PBitScheduler,G); */
                PBitScheduler_frame4();
                break;
            case 5:
                /* PATH(PBitScheduler,H); */
                PBitScheduler_frame5();
                break;
            case 6:
                /* PATH(PBitScheduler,I); */
                PBitScheduler_frame6();                         
                break;
            case 7:
                /* PATH(PBitScheduler,J); */
                PBitScheduler_frame7();                 
                PBitMsTimer++;
                break;
            default:
                /* PATH(PBitScheduler,K); */
                break;
        }
    
//        if ((PBitMsTimer > ADC_AVG_SAMPLES) && (McuGetState() != POWER_DOWN))
        if (PBitMsTimer > ADC_AVG_SAMPLES)
        {
            /* PATH(PBitScheduler,L); */
            Bit_Pbit(subframe);
        }
//        // Revisit when ICC implemented
//        if ((G_bChannelA == false) && (SpiTransmitt == true))
//        {
//            /* PATH(PBitScheduler,M); */
//
//            /* shall call the Spi_SendMessage function to fill the Transmit buffer */
//            SpiTxStatus = Spi_SendMessage();
//            SpiTransmitt = false;
//        }

        /* shall invoke MCU state machine to process any posted event*/
        Events_CheckEvent();
        
        Adc_GetResults_motor_phases();

        /* shall decrement master frame timer counter */
        Timer_MasterFrameCount--; 
        
        /* shall increment the testframe counter for the next occurrence */
        subframe++;
		if(subframe > 7 )
		{
			/* PATH(PBitScheduler,S);*/
			subframe = 0;		/* ensure a count between 0-7*/				
		}			        
        /* PATH(PBitScheduler,N); */
    }

    status = PBitScheduler_terminate();

    /* PATH(PBitScheduler,O); */

    /* shall return success/fail status of the fault monitor checks:'true' if one or more tests gives a 'positive' result */
    return (status);
}


/*
    Purpose:
        This initializes all of the variables necessary to support the PBit scheduler.

    return None.
    
    Preconditions and Assumptions:
        None.
  
*/
void PBitScheduler_Initialize(void)
{
    uint16_t i = 0;
    /* PATH(PBitScheduler_Initialize,A); */
    
    Timer_MasterFrameCount = 0;
    //msgCnt = 0;

#if defined(__HALLX_CONFIGURED)
    for(i = 0; i < NUM_RIG_POSITIONS; i++)
    {
        /* Clear xSideRigPosData structure*/
        xSideRigPosData[i].xside = 0;
    }
#endif

//    /* Timer used to make sure that PBit is executed in less than 2 seconds. */
//    Timer_SetTime(&PBit_Timer, TIMER_ONESHOT, TIMER_1900ms);
    /* Execute in less than 8 seconds */
    Timer_SetTime(&PBit_Timer, TIMER_ONESHOT, TIMER_10s);

    /* Set ADC timer delay for ADC to be completely stable (200ms) */
    Timer_SetTime(&ADCDelayTimer, TIMER_ONESHOT, TIMER_200ms);

    Bit_StartDog();

    Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_200ms);

    /* PATH(PBitScheduler_Initialize,B); */
}


/*
    Purpose:
        This routine call the function for frame 0.

    return None.
    
    Preconditions and Assumptions:
        None.
  
*/
void PBitScheduler_frame0(void)
{
    /* PATH(PBitScheduler_frame0,A); */
    SPI_SetCS(); //Set ADC channel and Capture after 125us interval
    Act_Ramp();
#if 0
    if (G_bChannelA == true)
    {
        /* PATH(PBitScheduler_frame0,B); */

        if (SpiaRegs.SPIFFTX.bit.TXFFST == 0)
        {
            /* PATH(PBitScheduler_frame0,C); */

            /* The Tx function now sets up the Spi Tx data structure, Spi_SendMessage will actually
               transmit the data, this now supports the asynchronous processors, and minimizes the
               impact for the slave side when having to check every frame if and determine if it needs
               to transmit data or not.*/
            Spi_TxMessage();

            if (SpiTxStatus == false)
            {
                /* PATH(PBitScheduler_frame0,D); */

                /* shall call the Spi_SendMessage function to fill the Transmit buffer */
                SpiTxStatus = Spi_SendMessage();
            }
        }
    }
    else
    {
        /* PATH(PBitScheduler_frame0,E); */
        Spi_TxMessage();
    }
#endif
    /* PATH(PBitScheduler_frame0,F); */
}


/*
    Purpose:
        This routine call the function for frame 1.

    return None.
    
    Preconditions and Assumptions:
        None.
  
*/
void PBitScheduler_frame1(void)
{
    /* PATH(PBitScheduler_frame1,A); */
    SPI_Capture();  //maintain previous set and Capture difference of 125us interval
    SPI_SetCS();    //Set ADC channel and Capture after 125us interval
    Act_MeasureSpeed();
	Nvm_WriteOutStateInformation();

    /* PATH(PBitScheduler_frame1,D); */
}


/*
    Purpose:
        This routine call the function for frame 2.

    return None.
    
    Preconditions and Assumptions:
        None.
  
*/
void PBitScheduler_frame2(void)
{
    /* PATH(PBitScheduler_frame2,A); */
    SPI_Capture();  //Capture after 125us interval
    SPI_SetCS();    //Set ADC channel and Capture after 125us interval
    Act_DoPid();
    Gpio_Capture();
    Adc_Sample();

    /* PATH(PBitScheduler_frame2,B); */
}


/*
    Purpose:
        This routine call the function for frame 3.

    return None.
    
    Preconditions and Assumptions:
        None.
  
*/
void PBitScheduler_frame3(void)
{
    /* PATH(PBitScheduler_frame3,A); */
    SPI_Capture();    //Capture after 125us interval
    //Act_DoControl();
    Adc_GetResults();
    Adc_Average(McuGetState());

    /* PATH(PBitScheduler_frame3,B); */
}


/*
    Purpose:
        This routine call the function for frame 4.

    return None.
    
    Preconditions and Assumptions:
        None.
  
*/
void PBitScheduler_frame4(void)
{
    /* PATH(PBitScheduler_frame4,A); */
    Adc_CalcTempfromADCcount();
    Adc_CalcUnitvalforPhases();
    Adc_CalcUnitvalforVolts();
    main_RunAsw();

    /* PATH(PBitScheduler_frame4,B); */
}


/*
    Purpose:
        This routine call the function for frame 5.

    return None.
    
    Preconditions and Assumptions:
        None.
  
*/
void PBitScheduler_frame5(void)
{
    /* PATH(PBitScheduler_frame5,A); */
	Act_DoControl();

#if defined(__SKEW_SNSR_ENCODER__) /* Check if Encoder is the skew sensor */
    Encoder_Tx();                   /*Transmit Clock Pulses to Encoder so that data can be received in next minor frame*/
#endif
    /* PATH(PBitScheduler_frame5,B); */
}


/*
    Purpose:
        This routine call the function for frame 6.

    return None.
    
    Preconditions and Assumptions:
        None.
  
*/
void PBitScheduler_frame6(void)
{
    /* PATH(PBitScheduler_frame6,A); */
#if 0
    if (SpiMessageReady == true)
    {
        /* PATH(PBitScheduler_frame6,B); */
        SpiMessageValid = Spi_ProcessSpiMessage();
        SpiMessageReady = false;
    }
#endif
    /* PATH(PBitScheduler_frame6,C); */
}


/*
    Purpose:
        This routine call the function for frame 7.

    return None.
    
    Preconditions and Assumptions:
        None.
  
*/
void PBitScheduler_frame7(void)
{
    /* PATH(PBitScheduler_frame7,A); */

#if defined(__SKEW_SNSR_ENCODER__)
    Encoder_Rx();                       /*Receive Data from Encoder every 1ms*/
#endif

    SkewSnsr_GetFlapAngle();
    SkewSnsr_GetStroke();
    /* PATH(PBitScheduler_frame7,B); */
}


/*
    Purpose:
        This calls all of the necessary functions to complete PBIT.

    return None.
    
        Preconditions and Assumptions:
        None.
  
*/
bool PBitScheduler_terminate(void)
{
    bool status = false;

    /* PATH(PBitScheduler_terminate,A); */

//    if (BridgeMonTestCmplt == false)
//    {
//        /* PATH(PBitScheduler_terminate,C); */
//        Bit_LatchedMonitors(0x50, false);
//        Bit_ReportToGse(0x50);
//    }

//    if (BrakeSwitchTestCmplt == false)
//        /* PATH(PBitScheduler_terminate,D); */
//        Bit_LatchedMonitors(0x25, false);
//        Bit_ReportToGse(0x25);
//    }

    Bit_LatchedMonitors(0x73, true);
    
    /* shall exercise Monitor 0x10 during PBIT. (Rig CRC Monitor) */
    Bit_CriticalMonitors(0x10, true);

    /* shall exercise Monitor 12 during PBIT. (Rig Mode Fault Monitor) */
    Bit_CriticalMonitors(0x12, true);

//    /* shall exercise Monitor 23 during PBIT when "on the ground" */
//    /*     - stage 1: engage brake
//           - wait for X_ENBL_IN to be active
//           - stage 2: drive motor
//           - start 5 second timer
//           - check trip condition during the 600 ms
//           - stage 3: stop motor  */
//    if ((InAirOnGroundStatus == ON_GROUND) && (CriticalFaults_Stat.bit.HW_AsymCM_M == 0) &&
//    	(LatchedFaults_Stat.bit.BridgeMonitor_M == 0) && (LatchedFaults_Stat.bit.BrakeSwitch_M == 0))
//    {
//        /* PATH(PBitScheduler_terminate,E); */
//
//        if (BrakeHoldTestCmplt == false)
//        {
//            /* PATH(PBitScheduler_terminate,F); */
//            CriticalFaults_Stat.bit.BrakeHold_M = 1;
//        }
//
//        Bit_CriticalMonitors(0x23, true);
//    }

    /* shall exercise monitor 57 */
    Bit_LatchedMonitors(0x57, true);

    /* shall exercise monitor 71. */
    Bit_LatchedMonitors(0x71, true);

    RiggingFaultCheck();

//    if (PrimarySide == false)
//    {
//        /* PATH(PBitScheduler_terminate,H); */
//        Bit_DoTest(0x0A, 0);
//    }
//
//    /* exercise monitor 40 */
//    Bit_InhibitMonitors(0x40, true);
//
    if ((Inhibits_Stat.all > 0) || (LatchedFaults_Stat.all > 0) || (CriticalFaults_Stat.all > 0))
    {
        /* PATH(PBitScheduler_terminate,I); */
        status = true;
    }

    /* PATH(PBitScheduler_terminate,J); */
    return (status);
}


void RiggingFaultCheck(void)
{
    /* PATH(RiggingFaultCheck,A); */
    
    if (bRigVerifyTstCmplt == false)
    {
        Bit_CriticalMonitors(0x64, false);
        Bit_ReportToGse(0x64);
    }

    /* PATH(RiggingFaultCheck,M); */
}

