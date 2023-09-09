/****************************************************************************************************
*  File name: rigmodescheduler.c
*
*  Purpose: Rig Mode Scheduler.
*  This file implements the task scheduler for rig mode.
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
#include "timer.h"
#include "bit.h"
#include "bitmonitors.h"
#include "mcu.h"
#include "brake.h"
#include "gse.h"
#include "actuation.h"
#include "adc.h"
#include "DFT.h"
//#include "rvdt.h"
#include "motor.h"
#include "nvm.h"
#include "spi.h"
#include "parameter.h"
#include "panel.h"
#include "A825Mgr.h"
#include "A825Drvr.h"
#include "icc.h"
#include "skewsensor.h"

/* 2 Quad counts of tolerance for rig verification procedure as resolution of
 * A825 signals result in up to 2 quad counts of loss  */
#define RIG_VERIFICATION_QUAD_COUNT_TOLERANCE (2)
#define SIZEOF_XSIDE_RIG_DATA      (sizeof(Nvm_QuadPosition_t) * NUM_RIG_POSITIONS)
#define LED_FLASH_TIMER_250MS   250

//eRvdtCalibrateRigCompleteStates_t tRvdtCalibrateRigCompleteState = RVDT_CALIBRATE_STATE_INIT;
typedef enum {
    RVDT_CALIBRATE_STATE_INIT = 0,
    RVDT_CALIBRATE_STATE_CALIBRATE_RVDT,
    RVDT_RIG_STATE_VALIDATE_CROSS_CHANNEL_RIG_DATA,
    RVDT_RIG_STATE_VALIDATE_CROSS_CHANNEL_RIG_COMPLETE,
    RVDT_CALIBRATE_RIG_STATE_COMPLETE,
    RVDT_CALIBRATE_RIG_COMPLETE_NUMBER_OF_STATES
}tRvdtCalibrateRigCompleteState_t;

void RigModeScheduler_Initialize(void);
void RigModeScheduler_frame0(void);
void RigModeScheduler_frame1(void);
void RigModeScheduler_frame2(void);
void RigModeScheduler_frame3(void);
void RigModeScheduler_frame4(void);
void RigModeScheduler_frame5(void);
void RigModeScheduler_frame6(void);
void RigModeScheduler_frame7(void);
extern void main_RunAsw(void);
void RigModeScheduler_CompleteRigStateMachine(void);
//extern bool_t Rvdt_CalibrateSensor(void);

bool riggingCheckComplete = false;
uint16_t u16FinalCheckTimer = 100U;
bool rigWriteInitiate = false;
bool StartOfRig = false;
//bool_t bRunSensorFusion = false;
bool RES_PositionVerified = false;
bool EES_PositionVerified = false;
bool_t RigWriteSuccess = false;
bool_t bClearNvmRiggingData = false;
bool_t bRigClearSuccess = false;

Uint16 u16LedFlashCntr = LED_FLASH_TIMER_250MS;

tRvdtCalibrateRigCompleteState_t tRvdtCalibrateRigCompleteState = RVDT_CALIBRATE_STATE_INIT;

/**
    brief Alternate scheduler implementation used only during RIG mode.
 
     Purpose:
        This routine configures/operates the RIG mode scheduler.

    return  void

*/
void RigModeScheduler(void)
{
    /* PATH(RigModeScheduler,A);*/
    RigModeScheduler_Initialize();

    /* Main rig mode loop*/
    for (;;)
    {
        /* PATH(RigModeScheduler,B);*/

        /* shall wait for 125 usec master frame timer to trip*/
        while (Timer_MasterFrameCount == 0)
        {
        	/* PATH(RigModeScheduler,S);*/
        }

#if defined (__WCTA__)
        /* Reset and Start the timer used for WCTA */
        Wcta_ResetTimer();
#endif
#if 0      
        if ((G_bChannelA == false) && (SpiTransmitt == true))
        {
            /* PATH(RigModeScheduler,C);*/

            /* shall call the Spi_SendMessage function to fill the Transmitt buffer*/
            SpiTxStatus = Spi_SendMessage();
            SpiTransmitt = false;
        }
#endif
        /* PATH(allframes_proc,A); */
        Adc_Sample_motor_phase_currents();

        A825Mgr_RxSrvc(&tA825_Bus[ARINC825_MAINT_BUS]);     /* Run the ARINC 825 Receive Service for the MAINTENANCE BUS */
        A825Mgr_TxSrvc(&tA825_Bus[ARINC825_MAINT_BUS]);     /* Run the ARINC 825 Transmit Service for the MAINTENANCE BUS */

        if(DFT_Tx_block == false)
        {
            DFT_TxSrvc();
        }

        Icc_RxMsg();
        Icc_TxSrvc();

#if defined (__WCTA__)
        /* Reset and Start the timer used for WCTA */
        Wcta_StoreResult(WCTA_ALL_SUBFRAMES_INDEX);
#endif
        /* Check if command to clear rigging data has been initiated */
        if(bClearNvmRiggingData == true)
        {
            /* Commanded to clear rigging data in NVM. */
            /* Call the Clear Rigging Data Function */
            bRigClearSuccess = Nvm_ClearRigging();
            /* Check if all rig data has been cleared */
            if(bRigClearSuccess == true)
            {
                /* All rig data has been cleared, reset the Clear NVM Rig data flag */
                bClearNvmRiggingData = false;
            }
        }

        /* RIG Sub-frame Scheduler */
        switch (subframe)
        {
            case 0:
                /* PATH(RigModeScheduler,D);*/
                RigModeScheduler_frame0();
                break;
            case 1:
                /* PATH(RigModeScheduler,E);*/
                RigModeScheduler_frame1();
                break;
            case 2:
                /* PATH(RigModeScheduler,F);*/
                RigModeScheduler_frame2();
                break;
            case 3:
                /* PATH(RigModeScheduler,P);*/
                RigModeScheduler_frame3();
                break;
            case 4:
                /* PATH(RigModeScheduler,G);*/
                RigModeScheduler_frame4();
                break;
            case 5:
                /* PATH(RigModeScheduler,H);*/
                RigModeScheduler_frame5();
                break;
            case 6:
                /* PATH(RigModeScheduler,I);*/
                RigModeScheduler_frame6();
                break;
            case 7:
                /* PATH(RigModeScheduler,J);*/
                RigModeScheduler_frame7();
                break;
        } /*end switch (subframe)*/
        
        if (OneMsTimer > 500)
        {
            /* PATH(RigModeScheduler,M); */
            Bit_Rbit(subframe);
        }

//        if ((Rigging_Status.onsideChannelRigged == true) && (Rigging_Status.xsideChannelRigged == true))
//        {
//            /* PATH(RigModeScheduler,N);*/
//
//            if ((riggingCheckComplete == false) && (Nvm_State.rigstatus == NOT_RIGGED))
//            {
//                /* PATH(RigModeScheduler,O);*/
//                Bit_CriticalMonitors(0x64, true);
//
//                if (CriticalFaults_Stat.bit.RiggingCheck_M == 0)
//                {
//                    /* PATH(RigModeScheduler,T);*/
//                    riggingCheckComplete = true;
//                    /* Set 100msec delay to allow the full 10second V_OFFSET calculation to occur */
//                    u16FinalCheckTimer = 100U;
//                }
//            }
//        }

        /* shall decrement master frame timer counter*/
        Timer_MasterFrameCount--;

        /* shall invoke MCU state machine to process any posted event*/
        Events_CheckEvent();

        /* Save off the critical data for powerdown used to initialize the software when powered
         * back up. POWER_DOWN state doesn't exist due to no stay-up caps on the hardware. Therefore
         * save critical data cyclically for MCU. */
        Current_NVM_State.quadposition.onside = tHall.Position;
#if defined(__HALLX_CONFIGURED)
        Current_NVM_State.quadposition.xside = tHallx.Position;
#endif
        Current_NVM_State.tPanel.tSf = G_ASW_DATA.tPanelOutputData;
        Current_NVM_State.tPanel.tSkewSnsrCalcs = tSkewSnsrCalcs;
        tNvm.tData.Nvm_Powerdown_State = Current_NVM_State;

#if defined (__WCTA__)
        /* Retrieve and store the Result of the WCTA. Counts of 10MHz.
         * 8kHz subframe timer = 10Mhz / 8kHz = 1250 counts  */
        Wcta_StoreResult(subframe);
#endif

        Adc_GetResults_motor_phases();

        if(subframe > 7 )
		{
			/* PATH(RigModeScheduler,K);*/
			BadSubframeCnt++;
		}
        /* shall increment the rigframe counter for the next occurrence*/
        subframe++;
		if(subframe > 7 )
		{
			/* PATH(RigModeScheduler,V);*/        
			subframe = 0;		/* ensure a count between 0-7*/				
		}			
    } /* end for */

    /* PATH(RigModeScheduler,R);*/
}


/*
    brief Initializes RigModeScheduler

    Purpose:
        RigModeScheduler_Initialize provides support for initializing the Rig 
        Mode Scheduler rigging status data structure and initilizing the Eicas 
        and GSE interfaces.
        
    Global Data Referenced:
        Nvm_State.rigstatus
        RIG_COMPLETE
		RIG_VERIFIED
		INTERACTIVE_RESPOND
		#TIMER_ONESHOT
		#TIMER_100ms
		#TIMER_50ms
		RigModeFlg
		BrakeCounter
		Timer_MasterFrameCount
		GSE_Connected
		GseMode
		EicasTimer
		GSEDelayTimer
		InhibitFlap
		Rigging_Status.asymOffset_Done
		Rigging_Status.EMS_Rigged
		Rigging_Status.onsideChannel_Rigged
		Rigging_Status.rigData_Written
		Rigging_Status.xsideChannel_Rigged
		Rigging_Status.rigVerify_Ready
		Rigging_Status.RMS_Rigged 
    
    
    return  void
    
    Preconditions and Assumptions:
         None.


*/
void RigModeScheduler_Initialize(void)
{
    /* PATH(RigModeScheduler_Initialize,A);*/

	/* Set RigModeFlag to true */
	bRigModeFlg = true;
	
    /* Reset BrakeCounter*/
	/*Commenting out as Brake functionality is deferred in MCU*/
    #if 0
    BrakeCounter = 0;
    #endif

    /* Reset Timer_MasterFrameCount*/
    Timer_MasterFrameCount = 0;

    /* shall set gse connected flag true*/
    bGseConnected = true;

    // DFT no longer needs a delay.
    //Timer_SetTime(&GSEDelayTimer, TIMER_ONESHOT, TIMER_50ms);

    /* Uninhibit flaps for rigging*/
    bInhibitFlap = false;

    if ((Nvm_State.rigstatus == RIG_COMPLETE) || (Nvm_State.rigstatus == RIG_VERIFIED))
    {
        /* PATH(RigModeScheduler_Initialize,B);*/
        Rigging_Status.bSkewSnsrCalibrationComplete = true;
        Rigging_Status.ESL_Rigged = true;
        Rigging_Status.onsideChannel_Rigged = true;
        Rigging_Status.rigData_Written = true;
        Rigging_Status.xsideChannel_Rigged = true;
        Rigging_Status.rigVerify_Ready = true;
        Rigging_Status.RSL_Rigged = true;
    }
    else
    {
        /* PATH(RigModeScheduler_Initialize,C);*/

        /* shall initialize rig*/
        RigModeScheduler_Start_Rig();
    }

    /* PATH(RigModeScheduler_Initialize,D);*/
}


/*
    brief frame 0 of RigModeScheduler

    Purpose:
        RigModeScheduler_Frame0 provides periodic support for updating in air 
        on ground status, power down status, motor acceleration, motor current
        measurement and SPI messaging.
        
    Global Data Referenced:
        PrimarySide
		SpiaRegs.SPIFFTX.bit.TXFFST
		SpiTxStatus
		SendRigOverSpi
		SendMonitor0x64_Data
		AIR_STATUS 
		SpiTxStatus
		SendRigOverSpi
		msgCnt
		SendMonitor0x64_Data
    
    return  void
    
    Preconditions and Assumptions:
         None.


*/
void RigModeScheduler_frame0(void)
{
    /* PATH(RigModeScheduler_frame0,A);*/
    SPI_SetCS(); //Set ADC channel and Capture after 125us interval
    Act_Ramp();

// ICC in place of SPI CCC.
//    if (G_bChannelA == true)
//    {
//        /* PATH(RigModeScheduler_frame0,B);*/
//
//        if ((SpiaRegs.SPIFFTX.bit.TXFFST == 0))
//        {
//            /* PATH(RigModeScheduler_frame0,C);*/
//
//            if (SendRigOverSpi == true)
//            {
//                /* PATH(RigModeScheduler_frame0,D);*/
//                msgCnt = 1;
//                SendRigOverSpi = false;
//            }
//
//            if (SendMonitor0x64_Data == true)
//            {
//                /* PATH(RigModeScheduler_frame0,E);*/
//                msgCnt = 9;
//                SendMonitor0x64_Data = false;
//            }
//
//            Spi_TxMessage();
//
//            if (SpiTxStatus == false)
//            {
//                /* PATH(RigModeScheduler_frame0,F);*/
//
//                /* shall call the Spi_SendMessage function to fill the Transmit buffer*/
//                SpiTxStatus = Spi_SendMessage();
//            }
//        }
//    }
//    else
//    {
//        /* PATH(RigModeScheduler_frame0,G);*/
//        Spi_TxMessage();
//    }

    Motor_CalculateCurrent();

    /* PATH(RigModeScheduler_frame0,H);*/
}


/*
    brief frame 1 of RigModeScheduler

    Purpose:
        RigModeScheduler_Frame1 provides periodic support for writing Rig 
        data to NVM.
        
    Global Data Referenced:
		Nvm_State_Temp
		StoreRigDataInNVM
		PrimarySide
		Nvm_Rigging_Temp
		Nvm_Rigging
		InitiateStateWrite
		StartOfRig
		NOT_RIGGED
		RIG_COMPLETE
		Nvm_State
		Rigging_Status
		
    return  void
    
    Preconditions and Assumptions:
         None.

*/
void RigModeScheduler_frame1(void)
{
    /* PATH(RigModeScheduler_frame1,A);*/
    SPI_Capture();  //maintain previous set and Capture difference of 125us interval
    SPI_SetCS();    //Set ADC channel and Capture after 125us interval
    Act_MeasureSpeed();
    GseRig_Command();

    if (StoreRigDataInNVM == true)
    {
        /* PATH(RigModeScheduler_frame1,B);*/

        /* Clear the Programming Bits in NVM prior to writing them. */
        Nvm_Rigging_Temp.tPrgm.all = 0U;

        /* Set Program Pins 1 and 2 */
        if (G_bChannelA == true)
        {
            /* PATH(RigModeScheduler_frame1,C);*/
            Nvm_Rigging_Temp.tPrgm.bit.bPRGM1 = true;
        }
        else
        {
            /* PATH(RigModeScheduler_frame1,D);*/
            Nvm_Rigging_Temp.tPrgm.bit.bPRGM2 = true;
        }

        /* Set Program Pins 3 thru 6 */
        /* Decode MCU Panel Type set at power up */
        switch(G_eLruId)
        {
            case MCU_LIB_LRU:    /* LEFT INBOARD MOTOR CONTROL UNIT */
            {
                if(G_eActId == LEFT_ACTUATOR)
                {
                    /* MCU_LIB_L */
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM3 = false;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM4 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM5 = false;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM6 = true;
                }
                else
                {
                    /* MCU_LIB_R*/
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM3 = false;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM4 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM5 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM6 = false;
                }
                break;
            }
            case MCU_RIB_LRU:    /* RIGHT INBOARD MOTOR CONTROL UNIT */
            {
                if(G_eActId == LEFT_ACTUATOR)
                {
                    /* MCU_RIB_L */
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM3 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM4 = false;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM5 = false;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM6 = true;
                }
                else
                {
                    /* MCU_RIB_R*/
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM3 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM4 = false;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM5 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM6 = false;
                }
                break;
            }
            case MCU_LOB_LRU:    /* LEFT OUTBOARD MOTOR CONTROL UNIT */
            {
                if(G_eActId == LEFT_ACTUATOR)
                {
                    /* MCU_LOB_L */
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM3 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM4 = false;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM5 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM6 = true;
                }
                else
                {
                    /* MCU_LOB_R*/
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM3 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM4 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM5 = false;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM6 = false;
                }
                break;
            }
            case MCU_ROB_LRU:    /* RIGHT OUTBOARD MOTOR CONTROL UNIT */
            {
                if(G_eActId == LEFT_ACTUATOR)
                {
                    /* MCU_ROB_L */
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM3 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM4 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM5 = false;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM6 = true;
                }
                else
                {
                    /* MCU_ROB_R*/
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM3 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM4 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM5 = true;
                    Nvm_Rigging_Temp.tPrgm.bit.bPRGM6 = false;
                }
                break;
            }
        }

        /* Save the Rigging Data to NVM */
        RigWriteSuccess = Nvm_WriteRigging(&Nvm_Rigging_Temp);

        /* Check if all rigging data has been written to NVM */
        if (RigWriteSuccess == true)
        {
            /* Rig data written along with CRC, flag to update the rigstatus State */
            Rigging_Status.rigData_Written = true;
            InitiateStateWrite = true;
            StoreRigDataInNVM = false;
        }
    }

    /* Write the first part of the State data structure if necessary*/
    if (InitiateStateWrite == true)
    {
        /* PATH(RigModeScheduler_frame1,G);*/
        
        if (StartOfRig == true)
        {
            /* PATH(RigModeScheduler_frame1,H);*/
            Nvm_State.rigstatus = NOT_RIGGED;
            StartOfRig = false;
            Nvm_State.faultId = 0;
            Nvm_State.faultData = 0;
        }
        else if (Nvm_State.rigstatus == RIG_COMPLETE)
        {
            /* PATH(RigModeScheduler_frame1,I);*/
            Nvm_State.faultId = 0;
            Nvm_State.faultData = 0;
        }

        Bit_LogToNvm(Nvm_State.faultId);
        InitiateStateWrite = false;
    }

	Nvm_WriteOutStateInformation();

    /* PATH(RigModeScheduler_frame1,K);*/
}


/*
    brief frame 2 of RigModeScheduler

    Purpose:
        RigModeScheduler_Frame2 provides periodic support for PID control, 
        Gpio inputs, ADC sampling and Rig Verification.
        
    Global Data Referenced:
		none
		
		
    return  void
    
    Preconditions and Assumptions:
         None.

*/
void RigModeScheduler_frame2(void)
{
    /* PATH(RigModeScheduler_frame2,A);*/
    SPI_Capture();  //Capture after 125us interval
    SPI_SetCS();    //Set ADC channel and Capture after 125us interval
    Act_DoPid();
    Gpio_Capture();
    Adc_Sample();
    RigModeScheduler_RigVerification();
    MCU_BusStateMachineProcessState();

    /* PATH(RigModeScheduler_frame2,B);*/
}


/*
    brief frame 3 of RigModeScheduler

    Purpose:
        RigModeScheduler_Frame3 provides periodic support for actuation control,
        ADC calibration and averaging functions.
        
    Global Data Referenced:
		none
		
		
    return  void
    
    Preconditions and Assumptions:
         None.

*/
void RigModeScheduler_frame3(void)
{
    /* PATH(RigModeScheduler_frame3,A);*/
    SPI_Capture();    //Capture after 125us interval
    //Act_DoControl();
    Adc_GetResults();
    Adc_Average(McuGetState());

    /* PATH(RigModeScheduler_frame3,B);*/
}


/*
    brief frame 4 of RigModeScheduler

    Purpose:
        RigModeScheduler_Frame4 provides periodic support for fault reporting,
        Eicas communications and GSE communications.
        
    Global Data Referenced:
		GSEDelayTimer
		GseMode
		
    return  void
    
    Preconditions and Assumptions:
         None.

*/
void RigModeScheduler_frame4(void)
{
    /* PATH(RigModeScheduler_frame4,A);*/
    Bit_FaultReporting();

    /* Run ASW (Operational Flight Program) Every 1ms ( 1kHz ) */
    main_RunAsw();
    Adc_CalcTempfromADCcount();
    Adc_CalcUnitvalforPhases();
    Adc_CalcUnitvalforVolts();

    /* PATH(RigModeScheduler_frame4,C);*/
}


/*
    brief frame 5 of RigModeScheduler

    Purpose:
        RigModeScheduler_Frame5 provides support for reading of the on side 
        flap angle and configuring the DC offset output for asymmetry detection.
        
    Global Data Referenced:
		none
				
    return  void
    
    Preconditions and Assumptions:
         None.

*/
void RigModeScheduler_frame5(void)
{
    /* PATH(RigModeScheduler_frame5,A);*/
    GseRig_SetFeedback();   /* Set the Feedback signals to the GSE Interface */

    RigModeScheduler_CompleteRigStateMachine();

    /* PATH(RigModeScheduler_frame5,B);*/
}


/*
    brief frame 6 of RigModeScheduler

    Purpose:
        RigModeScheduler_Frame6 provides periodic support for reading the 
        cross side flap angle and SPI communications. 
           
    Global Data Referenced:
		SpiMessageReady
			
    return  void
    
    Preconditions and Assumptions:
         None.

*/
void RigModeScheduler_frame6(void)
{
	Act_DoControl();
    /* PATH(RigModeScheduler_frame6,A);*/
#ifndef DRV8312_DEV_KIT
    if (u16LedFlashCntr != 0U)
    {
        u16LedFlashCntr--;
    }
    else
    {
        u16LedFlashCntr = LED_FLASH_TIMER_250MS;

        /* Toggle the LED */
        LED1_tgl();
    }
#endif

#if 0
    if (SpiMessageReady == true)
    {
        /* PATH(RigModeScheduler_frame6,B);*/

        if (Spi_ProcessSpiMessage() == true)
        {
            /* PATH(RigModeScheduler_frame6,C);*/
            Spi_CheckMessage();                         
        }

        SpiMessageReady = false;
    }
#endif

#if defined(__SKEW_SNSR_ENCODER__) /* Check if Encoder is the skew sensor */
    Encoder_Tx();                   /*Transmit Clock Pulses to Encoder so that data can be received in next minor frame*/
#endif
    /* PATH(RigModeScheduler_frame6,D);*/
}


/*
    brief frame 7 of RigModeScheduler

    Purpose:
        RigModeScheduler_Frame7 provides periodic support for motor capture, 
        brake control and NVM services. 
           
    Global Data Referenced:
		StatusWriteTimer
		POWER_DOWN
		RunTimeCountTimer
		RunTimeCountMinutes
		tNvm.tData.Nvm_RunTimeCount
		#TIMER_ONESHOT
		#TIMER_1ms
		Timer_OneMinTimer
		Nvm_State	
		
    return  void
    
    Preconditions and Assumptions:
         None.

*/
void RigModeScheduler_frame7(void)
{
    /* PATH(RigModeScheduler_frame7,A);*/
    Brake_Control();

    /* shall check RunTimeCountTimer for expiry and advance the tNvm.tData.Nvm_RunTimeCount once */
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
            RunTimeCountMinutes = 0;
            Nvm_State.RunTimeCount = tNvm.tData.Nvm_RunTimeCount;
        }
        
        Timer_SetTime(&RunTimeCountTimer, TIMER_ONESHOT, TIMER_1min);
    }

//#if defined(__SKEW_SNSR_RVDT__)
//    Rvdt_GetFlapAngle();
//    Rvdt_GetStroke();
//#endif

#if defined(__SKEW_SNSR_ENCODER__)
    Encoder_Rx();                       /*Receive Data from Encoder every 1ms*/
//    Encoder_GetFlapAngle();
//    Encoder_GetStroke();
#endif

    SkewSnsr_GetFlapAngle();
    SkewSnsr_GetStroke();

    ServiceDog();   /* Service the watchdog */

    /* PATH(RigModeScheduler_frame7,D);*/
}

/**
    brief Verifies Rigging after full RIG procedure.
 
     Purpose:
        This routine verifies the RIG procedure by operating the flap, then checking flags 
        after RIG procedure.

    return  void

*/
void RigModeScheduler_RigVerification(void)
{
    /*PATH(RigModeScheduler_RigVerification,A);*/

    /* shall check if system is in rig mode*/
    if (McuGetState() == RIG_MODE)
    {
        /*PATH(RigModeScheduler_RigVerification,B);*/

        /* shall determine if the channel has completed rigging*/
        if ((Rigging_Status.RSL_Rigged == true) &&
            (Rigging_Status.bSkewSnsrCalibrationComplete == true) &&
            (Rigging_Status.ESL_Rigged == true) && (rigWriteInitiate == false))
        {
            /*PATH(RigModeScheduler_RigVerification,C);*/

            /* shall set the onside channel rigged flag*/
            Rigging_Status.onsideChannel_Rigged = true;

            if ((riggingCheckComplete == true) && (LatchedFaults_Stat.all == 0) &&
			    (CriticalFaults_Stat.all == 0) && (Inhibits_Stat.all == 0))
            {
                /*PATH(RigModeScheduler_RigVerification,F);*/
                /* shall set rig status to complete*/
                Nvm_State.rigstatus = RIG_COMPLETE;
            }
        }
    }
    else /* Not Rig Mode*/
    {
        /*PATH(RigModeScheduler_RigVerification,G);*/

        switch (Nvm_State.rigstatus)
        {
            case NOT_RIGGED:
                /*PATH(RigModeScheduler_RigVerification,H);*/
                
                /* shall inhibit the flaps because system is not rigged*/
                bFasNotAvailable = true;
                bInhibitFlap = true;
                break;
            case RIG_COMPLETE:
                /*PATH(RigModeScheduler_RigVerification,I);*/
                
                /* shall report rig in process until rig is verified*/
                bRigInProcess = true;
                bFasNotAvailable = true;
                
//                /* shall perform rig verification */
//
//                /*PATH(RigModeScheduler_RigVerification,J);*/
//
//                if(G_eActuatorNumber < NUM_ACTUATOR_TYPES)
//                {
//
//                    //if(tPanel.s16PositionCmdQuad == s16RigDefaultPosition[G_ACTUATOR_ID][N1])
//                    if( (tPanel.s16PositionCmdQuad <= (s16RigPositionQuadCount[G_eActuatorNumber][N1] + RIG_VERIFICATION_QUAD_COUNT_TOLERANCE)) &&
//                        (tPanel.s16PositionCmdQuad >= (s16RigPositionQuadCount[G_eActuatorNumber][N1] - RIG_VERIFICATION_QUAD_COUNT_TOLERANCE)) )
//                    {
//                        /*PATH(RigModeScheduler_RigVerification,K);*/
//                        //if ((tHall.Position == Nvm_GetPosOnside(N1)) && (EES_PositionVerified == true))
//                        if( (tHall.Position <= (Nvm_GetPosOnside(N1) + RIG_VERIFICATION_QUAD_COUNT_TOLERANCE)) &&
//                            (tHall.Position >= (Nvm_GetPosOnside(N1) - RIG_VERIFICATION_QUAD_COUNT_TOLERANCE)) &&
//                            (EES_PositionVerified == true) )
//                        {
//                            /*PATH(RigModeScheduler_RigVerification,M);*/
//                            RES_PositionVerified = true;
//                        }
//                    }
//                    else if( (tPanel.s16PositionCmdQuad <= (s16RigPositionQuadCount[G_eActuatorNumber][P11] + RIG_VERIFICATION_QUAD_COUNT_TOLERANCE)) &&
//                             (tPanel.s16PositionCmdQuad >= (s16RigPositionQuadCount[G_eActuatorNumber][P11] - RIG_VERIFICATION_QUAD_COUNT_TOLERANCE)) )
//                    {
//                        /*PATH(RigModeScheduler_RigVerification,N);*/
//                        //if ((tHall.Position == Nvm_GetPosOnside(P11)) && (RES_PositionVerified == false))
//                        if( (tHall.Position <= (Nvm_GetPosOnside(P11) + RIG_VERIFICATION_QUAD_COUNT_TOLERANCE)) &&
//                            (tHall.Position >= (Nvm_GetPosOnside(P11) - RIG_VERIFICATION_QUAD_COUNT_TOLERANCE)) &&
//                            (RES_PositionVerified == false) )
//                        {
//                            /*PATH(RigModeScheduler_RigVerification,Q);*/
//                            EES_PositionVerified = true;
//                        }
//                    }
//                }


//                /* shall verify rig if both stages complete*/
//                if ((RES_PositionVerified == true) && (EES_PositionVerified == true))
//                {
                    /*PATH(RigModeScheduler_RigVerification,S);*/

                    /* If we get here then the Rig monitor has passed it's checks as part of BIT, otherwise the rig status would
                     * have been reset back to NOT_RIGGED. Go to RIG_VERIFIED state */
                    Nvm_State.rigstatus = RIG_VERIFIED;
                    Bit_LogToNvm(Nvm_State.faultId);
//                }

                break;
            case RIG_VERIFIED:
                /*PATH(RigModeScheduler_RigVerification,T);*/
                bRigInProcess = false;
                
                if (McuGetState() != FAS_FAIL)
                {
                    /*PATH(RigModeScheduler_RigVerification,V);*/
                    bFasNotAvailable = false;
                }
                
                RES_PositionVerified = true;
                EES_PositionVerified = true;
                break;
            default:
                /*PATH(RigModeScheduler_RigVerification,U);*/
                break;
        }
    }

    /*PATH(RigModeScheduler_RigVerification,W);*/
}


/**
    brief Sets flags for the start of the RIG procedure
 
     Purpose:
        This routine initializes the RIG procedure.

    return void

*/
void RigModeScheduler_Start_Rig(void)
{
//    uint16_t i = 0;
    /*PATH(RigModeScheduler_Start_Rig,A);*/

    /* Set all the NVM Data Associated with Rigging to Default Values */
    //Nvm_ClearRigging();
    bClearNvmRiggingData = true;

    /* Uninhibit flaps for rigging*/
    bInhibitFlap = false;

    /* shall set flaps not available flag*/
    bFasNotAvailable = true;

    /* shall set rig in process flag*/
    bRigInProcess = true;
    
//    for(i = 0; i < NUM_RIG_POSITIONS; i++)
//    {
//        /* Clear xSideRigPosData structure*/
//        xSideRigPosData[i].xside = 0;
//        xSideRigPosData[i].onside = 0;
//    }
    /* Clear xSideRigPosData structure */
    memcpy(&xSideRigPosData[0], 0, SIZEOF_XSIDE_RIG_DATA);


    /* shall designate rig status as not rigged*/
    Nvm_State.rigstatus = NOT_RIGGED;
    Rigging_Status.bSkewSnsrCalibrationComplete = false;
    Rigging_Status.ESL_Rigged = false;
    Rigging_Status.onsideChannel_Rigged = false;
    Rigging_Status.rigData_Written = false;
    Rigging_Status.xsideChannel_Rigged = false;
    Rigging_Status.rigVerify_Ready = false;
    Rigging_Status.RSL_Rigged = false;

    InitiateStateWrite = true;

//    /* Initialize Spi stuff*/
//    SendRigOverSpi = false;
//    CrossSideSpiDone = false;
//    SendMonitor0x64_Data = false;

    riggingCheckComplete = false;
    rigWriteInitiate = false;

    StartOfRig = true;

    RES_PositionVerified = false;
    EES_PositionVerified = false;

    /*PATH(RigModeScheduler_Start_Rig,B);*/
}

/*
    brief Complete the rigging functions involving the RVDTs


    Purpose:
        This function is intended to set the sine wave output of the RVDT, run the calibration
        process of the RVDT, and finish the checks done on the RVDTs required for rigging

     Global Data Referenced:
        #Adc_Averaged
        #CriticalFaults_Stat
        #CrossSideSpiDone
        #tHall
        #LatestFaultCode
        #LatestFaultData
        #McbspbRegs
        #NOT_RIGGED
        #Nvm_Rigging_Temp
        #Nvm_State
        #P_POS
        #G_bChannelA
        #RIG_COMPLETE
        #Rigging_Status
        #riggingCheckComplete
        #rigWriteInitiate
        #S_POS
        #SendMonitor0x64_Data
        #StoreRigDataInNVM
        #xLatestFaultCode
        #xLatestFaultData
        #xSideRigStatus

    return  void

    Preconditions and Assumptions:
        none
*/
void RigModeScheduler_CompleteRigStateMachine(void)
{
    bool_t bSkewSnsrCalComplete = false;

    /* PATH(Rvdt_Offset,A); */
    switch (tRvdtCalibrateRigCompleteState)
    {
        case RVDT_CALIBRATE_STATE_INIT:
        {
            /* start the RVDT Calibration when both RSL and ESL have been rigged and actuator returned to ZERO. */
            if ((Rigging_Status.RSL_Rigged == true) && (Rigging_Status.ESL_Rigged == true) &&
                (tHall.Position <= 12) && (tHall.Position >= -12) &&
                //(tHallx.Position <= 5) && (tHallx.Position >= -5) &&
                (Rigging_Status.bSkewSnsrCalibrationComplete == false))
            {
                tRvdtCalibrateRigCompleteState = RVDT_CALIBRATE_STATE_CALIBRATE_RVDT;
            }
            break;
        }
        case RVDT_CALIBRATE_STATE_CALIBRATE_RVDT:
        {
            /* Calibrate the Skew Sensor */
            bSkewSnsrCalComplete = SKEW_SNSR_CALIBRATE_SNSR();

            /* check if the calibration routine is finished */
            if(bSkewSnsrCalComplete == true)
            {
                Rigging_Status.bSkewSnsrCalibrationComplete = true;

                /* Initialize the Rig Verification Check state machine */
                tRigVerifyTstState = TST_RIG_VERIFY_INIT;

                /* RVDT Calibration Routine is complete, go to Verify the calculation */
                tRvdtCalibrateRigCompleteState = RVDT_RIG_STATE_VALIDATE_CROSS_CHANNEL_RIG_DATA;
            }
            break;
        }
        case RVDT_RIG_STATE_VALIDATE_CROSS_CHANNEL_RIG_DATA:
        {
            //if ((Rigging_Status.onsideChannel_Rigged == true) && (Rigging_Status.xsideChannel_Rigged == true))
            if (Rigging_Status.onsideChannel_Rigged == true)
            {
                /* PATH(RigModeScheduler,N);*/

                if ((riggingCheckComplete == false) && (Nvm_State.rigstatus == NOT_RIGGED))
                {
                    /* PATH(RigModeScheduler,O);*/
                    /* Run the Rig Verify Check until it has completed */
                    if(bRigVerifyTstCmplt == false)
                    {
                        bRigVerifyTstCmplt = Bit_DoTest(0x64, (Uint16)tRigVerifyTstState);
                    }
                    else
                    {
                        /* Rig Verify check is complete. If the test passed then move to next state. */
                        if (CriticalFaults_Stat.bit.RiggingCheck_M != true)
                        {
                            /* PATH(RigModeScheduler,T);*/
                            riggingCheckComplete = true;
                            /* Set time-out to wait for cross-side data to be communicated across */
                            u16FinalCheckTimer = 100U;

                            /* Go to final Complete state */
                            tRvdtCalibrateRigCompleteState = RVDT_RIG_STATE_VALIDATE_CROSS_CHANNEL_RIG_COMPLETE;
                        }
                    }
                }
            }
            break;
        }
        case RVDT_RIG_STATE_VALIDATE_CROSS_CHANNEL_RIG_COMPLETE:
        {
            /* shall check cross-side rig status*/
            if ((riggingCheckComplete == true) && (rigWriteInitiate == false))
            {
                /* PATH(Rvdt_Offset,Y); */
                if(u16FinalCheckTimer == 0U)
                {
                    //if (xSideRigStatus == RIG_COMPLETE)
                    /* Check that opposite channel's rig_status is complete */
                    if(ICC_xData.tData.tIcc_0.CHStatus.bit.rig_status == RIG_COMPLETE)
                    {
                        /* PATH(Rvdt_Offset,Z); */

                        /*shall complete rig*/
                        Rigging_Status.rigVerify_Ready = true;
                        Nvm_State.rigstatus = RIG_COMPLETE;
                        StoreRigDataInNVM = true;
                    }
                    //else if (xSideRigStatus == NOT_RIGGED)
                    else if (ICC_xData.tData.tIcc_0.CHStatus.bit.rig_status == NOT_RIGGED)
                    {
                        /* PATH(Rvdt_Offset,AA); */
                        Rigging_Status.rigVerify_Ready = false;
                        Nvm_State.rigstatus = NOT_RIGGED;
                        //if ((LatestFaultCode == 0x00) && (xLatestFaultCode != 0x00))
                        if ((LatestFaultCode == 0x00) && (ICC_xData.tData.tIcc_4.FaultCode != 0x00))
                        {
                            /* PATH(Rvdt_Offset,AB); */

                            /*shall set on-side fault code and data to cross-side fault info*/
                            //LatestFaultCode = xLatestFaultCode;
                            LatestFaultCode = ICC_xData.tData.tIcc_4.FaultCode;
                            Nvm_State.faultId = LatestFaultCode;
                            //LatestFaultData = xLatestFaultData;
                            LatestFaultData = ICC_xData.tData.tIcc_4.FaultData;
                            Nvm_State.faultData = LatestFaultData;
                        }

                        riggingCheckComplete = false;
                    }

                    rigWriteInitiate = true;
                    tRvdtCalibrateRigCompleteState = RVDT_CALIBRATE_RIG_STATE_COMPLETE;
                }
                else
                {
                    /* Decrement final check timer */
                    u16FinalCheckTimer--;
                }
            }
            break;
        }
        case RVDT_CALIBRATE_RIG_STATE_COMPLETE:
        default:
        {
            break;
        }
    }
}

