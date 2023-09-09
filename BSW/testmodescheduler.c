/****************************************************************************************************
*  File name: testmodescheduler.c
*
*  Purpose: Test Mode Scheduler.
*  This file implements the task scheduler for test mode.
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

#include "gpio.h"
#include "mcu.h"
#include "brake.h"
#include "gse.h"
#include "timer.h"
#include "actuation.h"
#include "adc.h"
//#include "eicas.h"
#include "rvdt.h"
#include "encoder.h"
#include "motor.h"
#include "nvm.h"
#include "spi.h"
#include "bitmonitors.h"

void testmodescheduler_Initialize(void);
void testmodescheduler_frame0(void);
void testmodescheduler_frame1(void);
void testmodescheduler_frame2(void);
void testmodescheduler_frame3(void);
void testmodescheduler_frame4(void);
void testmodescheduler_frame5(void);
void testmodescheduler_frame6(void);
void testmodescheduler_frame7(void);
extern void main_RunAsw(void);

Uint32 nvmformatstatus = 0;
Uint32 nvmclearstatestatus = 0;
int16 nvmclearrigstatus = 0;

/*
    brief Alternate scheduler implementation used only during TEST mode.
 
    Purpose:
        This routine configures/operates the TEST mode scheduler.

    return  void
*/
void TestModeScheduler(void)
{
    /* PATH(TestModeScheduler,A); */
    testmodescheduler_Initialize();

    /* Main test mode loop */
    for (;;)
    {
        /* PATH(TestModeScheduler,B); */

        /* shall wait for 125 usec master frame timer to trip */
        while (Timer_MasterFrameCount == 0)
        {
            /*PATH(TestModeScheduler,M);*/
        }

#if defined (__WCTA__)
        /* Reset and Start the timer used for WCTA */
        Wcta_ResetTimer();
#endif

        /* There is no FSL_Sample function in the testModeScheduler */
        /* shall schedule other actions in one or more of eight scheduler frames */
        switch (subframe)
        {
            case 0:
                /* PATH(TestModeScheduler,C); */
                testmodescheduler_frame0();
                break;
            case 1:
                /* PATH(TestModeScheduler,D); */
                testmodescheduler_frame1();
                break;
            case 2:
                /* PATH(TestModeScheduler,E); */  
                testmodescheduler_frame2();
                break;
            case 3:
                /* PATH(TestModeScheduler,F); */
                testmodescheduler_frame3();
                break;
            case 4:
                /* PATH(TestModeScheduler,G); */
                testmodescheduler_frame4();
                break;
            case 5:
                /* PATH(TestModeScheduler,H); */
                testmodescheduler_frame5();
                break;
            case 6:
                /* PATH(TestModeScheduler,I); */
                testmodescheduler_frame6();
                break;
            case 7:
                /* PATH(TestModeScheduler,J); */
                testmodescheduler_frame7();
                break;
            default:
                /* PATH(TestModeScheduler,K); */
                break;
        }

        if ((Ibit_Stat.TestID > 0) && (Ibit_Stat.tstCmplt != COMPLETED))
        {
            /* PATH(TestModeScheduler,L); */
            Bit_Ibit(Ibit_Stat.TestID);
        }
        
        
        /* shall decrement master frame timer counter */
        Timer_MasterFrameCount--;
#if 0
        if ((G_bChannelA == false) && (SpiTransmitt == true))
        {
            /* PATH(TestModeScheduler,N); */

            /* shall call the Spi_SendMessage function to fill the Transmit buffer */
            SpiTxStatus = Spi_SendMessage();
            SpiTransmitt = false;
        }
#endif
//        /* If a powerdown situation occurs we need to transition into that mode immediately */
//        if ((onSideSpiPwrDwn == true) && (McuGetState() != POWER_DOWN))
//        {
//            /* PATH(TestModeScheduler,O); */
//            Events_PostEvent(PowerInterrupt, 0);
//        }
//        /* SCS Non-Compliance Justification: The following else if has 5 conditions which violates
//           the SCS (maximum of 4 conditions).  All of the conditions are necessary for power
//           restoration.  One extra condition does not make the testing of this function more
//           complicated */
//        else if ((McuGetState() == POWER_DOWN) && (onSideSpiPwrDwn == true) &&
//                 (xSideSpiPwrDwn == true) && (McuGetOnsideExitPwrDown() == true) &&
//                 (McuGetXsideExitPwrDown() == true))
//        {
//            /* PATH(TestModeScheduler,Q);*/
//            Events_PostEvent(PowerRestoration, 0);
//        }

        /* Save off the critical data for powerdown used to initialize the software when powered
         * back up. POWER_DOWN state doesn't exist due to no stay-up caps on the hardware. Therefore
         * save critical data cyclically for MCU. */
        Current_NVM_State.quadposition.onside = tHall.Position;
        //Current_NVM_State.quadposition.xside = tHallx.Position;
        Current_NVM_State.tPanel.tSf = G_ASW_DATA.tPanelOutputData;
        Current_NVM_State.tPanel.tSkewSnsrCalcs = tSkewSnsrCalcs;
        tNvm.tData.Nvm_Powerdown_State = Current_NVM_State;

        /* shall invoke MCU state machine to process any posted event */
        Events_CheckEvent();

        /* shall increment the testframe counter for the next occurrence */
        subframe++;
        if(subframe > 7 )
        {
            /* PATH(TestModeScheduler,R);*/
            subframe = 0;       /* ensure a count between 0-7*/             
        }               
    }

    /* PATH(TestModeScheduler,P); */
}


/*
    Purpose:
        This routine sets up the global variables needed to support Test mode.

    return  None.
    
    Preconditions and Assumptions:
        None.
*/
void testmodescheduler_Initialize(void)
{
    int i = 0;  
    /* PATH(testmodescheduler_Initialize,A); */

    /* shall initialize Timer_MasterFrameCount to zero */
    Timer_MasterFrameCount = 0;

//    /* shall set GSE to INTERACTIVE_RESPOND mode */
//    GseMode = INTERACTIVE_RESPOND;

    /* shall set flaps not available flag */
    bFasNotAvailable = true;

    /* shall set GSE connect flag to true */
    bGseConnected = true;
    
    /* shall ensure ADCs are ready */
    /* shall ensure GPIO inputs are ready */
    for (i = 0; i < DEBOUNCE_TIME; i++)
    {
        /* PATH(testmodescheduler_Initialize,C); */
        Gpio_Capture();
        Adc_Sample();
//        Adc_Calibrate();
        Adc_Average(McuGetState());
    }
    
    /* PATH(testmodescheduler_Initialize,B); */
}


/*
    Purpose:
        This routine call the function for frame 0.

    return None.
    
    Preconditions and Assumptions:
        None.
  
*/
void testmodescheduler_frame0(void)
{
    /* PATH(testmodescheduler_frame0,A); */

    Act_Ramp();
#if 0
    if (G_bChannelA == true) 
    {
        /* PATH(testmodescheduler_frame0,B); */

        if (SpiaRegs.SPIFFTX.bit.TXFFST == 0)
        {
            /* PATH(testmodescheduler_frame0,C); */
            Spi_TxMessage();

            if (SpiTxStatus == false)
            {
                /* PATH(testmodescheduler_frame0,D); */

                /* shall call the Spi_SendMessage function to fill the Transmit buffer */
                SpiTxStatus = Spi_SendMessage();
            }
        }
    }
    else
    {
        /* PATH(testmodescheduler_frame0,E); */
        Spi_TxMessage();
    }
#endif
    /* PATH(testmodescheduler_frame0,F); */
}


/*
    Purpose:
        This routine call the function for frame 1.

    return None.
  
    Preconditions and Assumptions:
        None.
*/
void testmodescheduler_frame1(void)
{
    /* PATH(testmodescheduler_frame1,A); */

    Motor_CalculateCurrent();
    Act_MeasureSpeed();
    Nvm_WriteOutStateInformation();

    /* PATH(testmodescheduler_frame1,C); */
}


/*
    Purpose:
        This routine call the function for frame 2.

    return  None.
    
    Preconditions and Assumptions:
         None.
  
*/
void testmodescheduler_frame2(void)
{
    /* PATH(testmodescheduler_frame2,A); */

    Act_DoPid();
    Gpio_Capture();
    Adc_Sample();

    /* PATH(testmodescheduler_frame2,B); */
}


/*
    Purpose:
        This routine call the function for frame 3.

    return None.
    
    Preconditions and Assumptions:
        None.
  
*/
void testmodescheduler_frame3(void)
{
    /* PATH(testmodescheduler_frame3,A); */
    //Act_DoControl();
//    Adc_Calibrate();
    Adc_Average(McuGetState());

#if 0
    if (FormatNvm == true)
    {
        /* PATH(testmodescheduler_frame3,B); */
        NVM_ClearInProgress = true;
        NVM_FormatComplete = false;
        nvmformatstatus = Nvm_Format(nvmformatstatus);
        
        if (nvmformatstatus == 0xFFFFFFFF)
        {
            /* PATH(testmodescheduler_frame3,C); */
            FormatNvm = false;
            BoardSerialNumber = 0;
            nvmformatstatus = 0;
            NVM_ClearInProgress = false;
            NVM_FormatComplete = true;
//            Nvm_ResetRigDefaults();
        }
    }

    if (ClearRigging == true)
    {
        /* PATH(testmodescheduler_frame3,D); */
        NVM_ClearInProgress = true;
        NVM_RigInfoCleared = false;
        Nvm_ClearRigging();
        
        if (nvmclearrigstatus == -1)
        {
            /* PATH(testmodescheduler_frame3,E); */
            ClearRigging = false;
            nvmclearrigstatus = 0;
            NVM_ClearInProgress = false;
            NVM_RigInfoCleared = true;
//            Nvm_ResetRigDefaults();
        }
    }

    if (ClearState == true)
    {
        /* PATH(testmodescheduler_frame3,F); */
        NVM_ClearInProgress = true;
        NVM_StateInfoCleared = false;
        //nvmclearstatestatus = Nvm_ClearState( nvmclearstatestatus );
        
        if (nvmclearstatestatus == 0xFFFFFFFF)
        {
            /* PATH(testmodescheduler_frame3,G); */
            ClearState = false;
            nvmclearstatestatus = 0;
            NVM_ClearInProgress = false;
            NVM_StateInfoCleared = true;
        }
    }
#endif
    /* PATH(testmodescheduler_frame3,H); */
}


/*
    Purpose:
        This routine call the function for frame 4.

    return None.
    
    Preconditions and Assumptions:
         None.
  
*/
void testmodescheduler_frame4(void)
{
    /* PATH(testmodescheduler_frame4,A); */

    Bit_FaultReporting();

#if defined(__SKEW_SNSR_ENCODER__) /* Check if Encoder is the skew sensor */
    Encoder_Tx();                   /*Transmit Clock Pulses to Encoder so that data can be received in next minor frame*/
#endif

//    if (Timer_IsExpired(&GSEDelayTimer) == true)
//    {
//        /* PATH(testmodescheduler_frame4,B); */
//        Eicas_Update();
//    }

//    Gse_Interactive(GseMode);

    /* PATH(testmodescheduler_frame4,C); */
}


/*
    Purpose:
        This routine call the function for frame 5.

    return None.
  
    Preconditions and Assumptions:
         None.
*/
void testmodescheduler_frame5(void)
{
    /* PATH(testmodescheduler_frame5,A); */

#if defined(__SKEW_SNSR_ENCODER__)
    Encoder_Rx();                       /*Receive Data from Encoder every 1ms*/
#endif

    SkewSnsr_GetFlapAngle();
    SkewSnsr_GetStroke();

    /* PATH(testmodescheduler_frame5,B); */
}


/*
    Purpose:
        This routine call the function for frame 6.

    return None.
    
    Preconditions and Assumptions:
        None.
  
*/
void testmodescheduler_frame6(void)
{
    /* PATH(testmodescheduler_frame6,A); */

    /* Run Sensor Fusion */
    main_RunAsw();
#if 0
    if (SpiMessageReady == true)
    {
        /* PATH(testmodescheduler_frame6,B); */
        SpiMessageValid = Spi_ProcessSpiMessage();
        SpiMessageReady = false;
    }
#endif
    /* PATH(testmodescheduler_frame6,C); */
}


/*
    Purpose:
        This routine call the function for frame 7.

    return  None.
  
    Preconditions and Assumptions:
        None.
*/
void testmodescheduler_frame7(void)
{
    /* PATH(testmodescheduler_frame7,A); */

//    Motor_Capture();
    Brake_Control();

    /* Process the second write of the stateData structure if necessary */
//    Nvm_Service();

    /* shall check RunTimeCountTimer for expiry and advance the Nvm_RunTimeCount once */
    /*     for every ten expirations of the 1-minute RunTimeCountTimer. */                         
    if (Timer_IsExpired(&RunTimeCountTimer) == true)
    {
        /* PATH(testmodescheduler_frame7,F); */
        RunTimeCountMinutes++;
        Timer_OneMinTimer++;
        
        if (RunTimeCountMinutes >= 10)
        {
            /* PATH(testmodescheduler_frame7,G); */
            tNvm.tData.Nvm_RunTimeCount++;
//            Nvm_WriteRunTimeCount(tNvm.tData.Nvm_RunTimeCount++);
            RunTimeCountMinutes = 0;
            Nvm_State.RunTimeCount = tNvm.tData.Nvm_RunTimeCount++;
        }
        
        Timer_SetTime(&RunTimeCountTimer, TIMER_ONESHOT, TIMER_1min);
    }
    ServiceDog();   /* Service the watchdog */
    /* PATH(testmodescheduler_frame7,E); */
	Act_DoControl();
}
