/****************************************************************************************************
*  File name: main.c
*
*  Purpose: Scheduler and main entry point for firmware.
*  This file implements the task scheduler for the MCU software,
*  as well as some basic initialization/setup routines.
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
#include "DFT.h"
#include "F28x_Project.h"


/*      Include Files */
#include "timer.h"
#include "adc.h"
#include "motor.h"
#include "brake.h"
#include "actuation.h"
//#include "eicas.h"
#include "gse.h"
#include "hallsensor.h"
#include "panel.h"
/*#include "rvdt.h"*/
#include "gpio.h"
#include "mcu.h"
#include "nvm.h"
#include "spi.h"
#include "parameter.h"
#include "events.h"
#include "bitmonitors.h"
//#include "csci.h"
#include "icc.h"
#include "A825Mgr.h"
//#include "A825Drvr.h"
#include "ASW_BSW_I.h"
#include "OFP.h"
#include "PDI_Mgr.h"
#include "actuation.h"
/*#include "encoder.h"*/
#include "skewsensor.h"

/*      Local Type Definitions */
#define CalModeQuadDiff 50
#define LED_FLASH_TIMER_500MS   500

/*      Global Variables */
Timer_t StatusWriteTimer = TIMER_DEFAULTS;  /* timer used for delayed write of status information to NVM on power down */
Uint16 subframe = 0;      /* Subframe counter used within the scheduler; global for BIT monitoring */
//Boolean_e config_Integrity_Check_Fail = False;
Timer_t RunTimeCountTimer = TIMER_DEFAULTS;
Uint32 RunTimeCountMinutes = 0;

bool DFT_Tx_cmd = true;
bool DFT_Tx_block = false;
Uint16 BadSubframeCnt = 0;
Uint16 UndefinedIntCnt = 0;
int16 hallStartPosition = 0;
bool SystemStopFlg = true;
bool SpiTransmitt = false;
#if 0
static bool SpiTransmit_Delayed = false; /* flag indicating SPI transmit buffer should be filled in next 125us timeslice - PR7310 */
#endif
Nvm_QuadPosition_t xSideRigPosData[NUM_RIG_POSITIONS] = {0};
RigStatus_t Rigging_Status = RIG_STATUS_DEFAULTS;
//bool SendRigOverSpi = false;
//bool SendMonitor0x64_Data = false;

Uint16 PowerDownTrpCnt = 0;

Uint16 ledFlashCntr = LED_FLASH_TIMER_500MS;
//Uint32 nvmformatstatus = 0;
/*      Local ROM Constants */

/*      Local Variable Declarations */

/*      Local Function Prototypes */
void Initialization_Tasks(void);
void Boot_Tasks(void);
void sys_init(void);
void Flash_Init(void);
void allframes_proc(void);
void SystemStop(void);
void SystemStop_Check(void);
void Calibration_Command(void);
bool InCalibrationMode(void);
void PDI_Init(void);
void OnSideCmd_cplt(void);
void main_frameScheduler(void);
void main_frame0(void);
void main_frame1(void);
void main_frame2(void);
void main_frame3(void);
void main_frame4(void);
void main_frame5(void);
void main_frame6(void);
void main_frame7(void);
void Watchdog_Init(void);
void Watchdog_TestWdReset(void);
void main_BswSetAswAnlgData(void);
void main_BwsSetAswDiscData(void);
void main_BwsSetAswPanelData(void);
void main_RunAsw(void);

/*      Function Definitions */

/*
    brief This is the first C code executed by the software.

     Purpose:
        This routine invokes system initialization and configures/operates the scheduler.

    return  void

*/

void main(void)
{
    Flash_Init();                       /* Initialize the Flash so a CRC check can be done  */
    Bit_CriticalMonitors(0x70, true);   /* Run the ROM CRC Check for the Application        */
    Hsm_OnStart(&G_eMcuMode); /* Start the MCU state machine */
    Initialization_Tasks(); /* Perform tasks for Initialization Mode */
    /* PDI Setup:  Init PDIs, Run CRC check, and Range Check Params if CRC passes. */
    PDI_Init();                 /* Initialize PDI data */
    Events_CheckEvent();    /* Check event for any failures */
    Watchdog_Init();        /* Setup and Enable Watchdog for runtime. */
	MCU_BusOutputInit();

    NS_CNTL_PHA_CHA_clr();
    NS_CNTL_PHC_CHA_clr();

#if defined(__SKEW_SNSR_ENCODER__)
    Encoder_Rx();
#endif

    /* Enter Rig, Test, or Boot Mode depending on result of Initialization_Tasks */
    switch (McuGetState())
    {
        case RIG_MODE:  /* Enter Rig Mode */
            /* PATH(main,C); */
            RigModeScheduler();     /* Call rig mode scheduler */
            break;
        case TEST_MODE: /* Enter Test Mode */
            /* PATH(main,D); */
            TestModeScheduler();    /* Call test mode scheduler */
            break;
        case BOOT_MODE: /* Enter Boot Mode */
            /* PATH(main,E); */
            Boot_Tasks();           /* Perform tasks for Boot Mode */
            Events_CheckEvent();    /* Check event for going into Idle Mode or Fail Mode from Boot Mode */
            break;
    }

    /* Initialize Timer_MasterFrameCount and subframe to zero */
    Timer_MasterFrameCount = 0;
    subframe = 0;

    /* Simulink Main Module Initialization */
    OFP_initialize();

    /* Initialize the values for the Skew Sensor used in Sensor Fusion */
//    Rvdt_GetFlapAngle();
//    Rvdt_GetStroke();
//    SkewSnsr_GetFlapAngle();
//    SkewSnsr_GetStroke();

    /* Main loop */
    main_frameScheduler();

}

/*
     Purpose:
        This routine is the scheduler and calls the different functions based on the frame being executed.

    return  None.

    Preconditions and Assumptions:
         None.

*/
void main_frameScheduler(void)
{
    /* PATH(main_frameScheduler,A); */
    for (;;)
    {
        /* PATH(main_frameScheduler,B); */

        /* shall wait for 125 usec master frame timer to trip */
        while (Timer_MasterFrameCount == 0)
        {
            /*PATH(main_frameScheduler,O);*/
        }

#if defined(DRV8312_DEV_KIT)
        TEST_OUTPUT_GPIO17_tgl(); /* Toggle Test Point for Testing */
#endif // DRV8312_DEV_KIT

#if defined (__WCTA__)
        /* Reset and Start the timer used for WCTA */
        Wcta_ResetTimer();
#endif
        /* PATH(main_frameScheduler,C); */
        allframes_proc();

#if defined (__WCTA__)
        /* Reset and Start the timer used for WCTA */
        Wcta_StoreResult(WCTA_ALL_SUBFRAMES_INDEX);
#endif

        /* shall schedule other actions in one or more of eight scheduler frames */
        switch (subframe)
        {
            case 0:
                /* PATH(main_frameScheduler,D); */
                main_frame0();
                break;
            case 1:
                /* PATH(main_frameScheduler,E); */
                main_frame1();
                break;
            case 2:
                /* PATH(main_frameScheduler,F); */
                main_frame2();
                break;
            case 3:
                /* PATH(main_frameScheduler,G); */
                main_frame3();
                break;
            case 4:
                /* PATH(main_frameScheduler,H); */
                main_frame4();
                break;
            case 5:
                /* PATH(main_frameScheduler,I); */
                main_frame5();
                break;
            case 6:
                /* PATH(main_frameScheduler,J); */
                main_frame6();
                break;
            case 7:
                /* PATH(main_frameScheduler,K); */
                main_frame7();
                break;
        }

        Bit_Run_CBit(subframe);

        /* shall decrement master frame timer counter */
        Timer_MasterFrameCount--;

        if(subframe > 7 )
        {
            /*PATH(main_frameScheduler,L);*/
            BadSubframeCnt++;
        }

#if defined (__WCTA__)
        /* Retrieve and store the Result of the WCTA. Counts of 10MHz.
         * 8kHz subframe timer = 10Mhz / 8kHz = 1250 counts  */
        Wcta_StoreResult(subframe);
#endif

        /* shall increment the subframe counter for the next occurrence */
        subframe++;
        if(subframe > 7 )
        {
            /* PATH(main_frameScheduler,M);*/
            subframe = 0;       /* ensure a count between 0-7*/
        }

    }

    /* PATH(main_frameScheduler,N); */
}


/*
     Purpose:
        This routine call the function for frame 0.

    return  None.

    Preconditions and Assumptions:
         None.


*/
void main_frame0(void)
{
    SPI_SetCS(); //Set ADC channel and Capture after 125us interval
    /* PATH(main_frame0,A); */

    Act_Ramp();

#if defined(DRV8312_DEV_KIT)
    /* For Indication that Dev Kit is running */
    if (ledFlashCntr != 0U)
    {
        ledFlashCntr--;
    }
    else
    {
        ledFlashCntr = LED_FLASH_TIMER_500MS;

        /* Toggle the LED */
        LED3_tgl();
    }
#endif
#if 0
    if (G_bChannelA == true)
    {
        /* PATH(main_frame0,B); */

        if (SpiaRegs.SPIFFTX.bit.TXFFST == 0)
        {
            /* PATH(main_frame0,C); */

            /* The Tx function now sets up the Spi Tx data structure, Spi_SendMessage will actually
             transmit the data, this now supports the asynchronous processors, and minimizes the
             impact for the slave side when having to check every frame if and determine if it needs
             to transmit data or not. */
            Spi_TxMessage();

            if (SpiTxStatus == false)
            {
                /* PATH(main_frame0,D); */

                /* shall call the Spi_SendMessage function to fill the Transmit buffer */
                SpiTxStatus = Spi_SendMessage();
            }
        }
    }
    else if (G_bChannelA == false)
    {
        /* PATH(main_frame0,E); */
        Spi_TxMessage();
    }
#endif
    Motor_CalculateCurrent();

    /* PATH(main_frame0,F); */
}


/*
     Purpose:
        This routine calls the functions for frame 1 to perform the following tasks:
            - Measure speed
            - Write state information out to NVM

    return None.

     Preconditions and Assumptions:
         None.

*/
void main_frame1(void)
{
    SPI_Capture();  //maintain previous set and Capture difference of 125us interval
    SPI_SetCS();    //Set ADC channel and Capture after 125us interval
    /* PATH(main_frame1,A); */
    Act_MeasureSpeed();
    Nvm_WriteOutStateInformation();
    Panel_Command();        /* Get the Panel Command from SYNC Interface */

    /* PATH(main_frame1,C); */
}


/*
     Purpose:
        This routine call the function for frame 2.

    return  None.

     Preconditions and Assumptions:
         None.

*/
void main_frame2(void)
{
    /* PATH(main_frame2,A); */
    SPI_Capture();  //Capture after 125us interval
    SPI_SetCS();    //Set ADC channel and Capture after 125us interval
    Act_DoPid();
    Gpio_Capture();
    Adc_Sample();
	MCU_BusStateMachineProcessState();

    /* PATH(main_frame2,B); */
}


/*
     Purpose:
        This routine call the function for frame 3.

    return  None.

     Preconditions and Assumptions:
         None.

*/
void main_frame3(void)
{
    /* PATH(main_frame3,A); */
    SPI_Capture();    //Capture after 125us interval
    //Act_DoControl();
    Adc_GetResults();               /* Get the ADC Resultant Values and store to memory */
    Adc_Average(McuGetState());   /* Run the Averaging routine on the ADC Values */
#if !defined(TRL4_NO_PDI_RIG)
    RigModeScheduler_RigVerification(); /* Validate the Rigging Routine. REWORK THIS! */
#endif
    /* PATH(main_frame3,B); */
}


/*
    Purpose:
        This routine call the function for frame 4.

    return  None.

     Preconditions and Assumptions:
         None.

*/
void main_frame4(void)
{
    /* PATH(main_frame4,A); */
    Bit_FaultReporting();

    /* Run ASW (Operational Flight Program) Every 1ms ( 1kHz ) */
    main_RunAsw();
    Adc_CalcTempfromADCcount();
    Adc_CalcUnitvalforPhases();
    Adc_CalcUnitvalforVolts();
    /* PATH(main_frame4,C); */
}


/*
     Purpose:
        This routine call the function for frame 5.

    return  None.

     Preconditions and Assumptions:
         SCS Non-Compliance Justification: This function violates SCS objective 3.3.4.3 and 3.3.4.1,
         keep nested loops or conditions to 4 or less. This function has 5 nested  conditional
         statements, and a instance of 8 conditional tests. This objective is in place to help simplify
         testing, and structural coverage. With the way the LLR's are documented achieving either complete
         structural coverage, and testing of the requirements are not compromised, and therefore in this
         case using more than 4 nested loops or conditions does not compromise the source code.

*/
void main_frame5(void)
{
    Panel_SetFeedback();    /* Set the Feedback to the SYNC Interface */

    OnSideCmd_cplt();
    Act_DoControl();


    return;
    /* PATH(main_frame5,K); */
}


/*
     Purpose:
        This routine call the function for frame 6.

    return  None.

     Preconditions and Assumptions:
         None.

*/
void main_frame6(void)
{
    /* PATH(main_frame6,A); */
#if defined(DRV8312_DEV_KIT)
        if (ledFlashCntr != 0U)
        {
            ledFlashCntr--;
        }
        else
        {
            ledFlashCntr = LED_FLASH_TIMER_500MS;

            /* Toggle the LED */
            TEST_OUTPUT_GPIO31_tgl(); /* Toggle Test Point for Testing */
        }

#else
        if (ledFlashCntr != 0U)
        {
            ledFlashCntr--;
        }
        else
        {
            ledFlashCntr = LED_FLASH_TIMER_500MS;

            /* Toggle the LED */
            LED1_tgl();
        }

#endif

#if 0
    if (SpiMessageReady == true)
    {
        /* PATH(main_frame6,B); */
        SpiMessageValid = Spi_ProcessSpiMessage();
        SpiMessageReady = false;
    }
#endif


#if defined(__SKEW_SNSR_ENCODER__) /* Check if Encoder is the skew sensor */
    Encoder_Tx();                   /*Transmit Clock Pulses to Encoder so that data can be received in next minor frame*/
#endif


    /* PATH(main_frame6,C); */
}


/*
     Purpose:
        This routine call the function for frame 7.

    return  None.

     Preconditions and Assumptions:
         None.

*/
void main_frame7(void)
{
    if (McuGetState() == RUN_MODE || (McuGetState() == IDLE_MODE))
    {
        /* PATH(main_frame7,B); */
        Brake_Control();
    }

    /* shall check RunTimeCountTimer for expiry and advance the Nvm_RunTimeCount once */
    /*     for every ten expirations of the 1-minute RunTimeCountTimer. */
    if (Timer_IsExpired(&RunTimeCountTimer) == true)
    {
        /* PATH(main_frame7,F); */
        RunTimeCountMinutes++;
        Timer_OneMinTimer++;

        if (RunTimeCountMinutes >= 10)
        {
            /* PATH(main_frame7,G); */
            tNvm.tData.Nvm_RunTimeCount++;
            RunTimeCountMinutes = 0;
            Nvm_State.RunTimeCount = tNvm.tData.Nvm_RunTimeCount;
        }

        Timer_SetTime(&RunTimeCountTimer, TIMER_ONESHOT, TIMER_1s*60);
    }


#if defined(__SKEW_SNSR_ENCODER__)
    Encoder_Rx();                       /*Receive Data from Encoder every 1ms*/
#endif

    SkewSnsr_GetFlapAngle();
    SkewSnsr_GetStroke();

    ServiceDog();   /* Service the watchdog */
}

void OnSideCmd_cplt(void)
{
//    int16 delta = 0;

    /* PATH(OnSideCmd_cplt,A); */

    /* If cross-channel enable is high */
    if (/*((X_ENBL_IN_N >> 17) != 0) &&*/ (MotorCmd.MotorStop == false))
    {
        /* PATH(OnSideCmd_cplt,B); */
        SystemStopFlg = true;


//        /* If in run mode */
//        if (McuGetState() == RUN_MODE)
//        {
//            /* PATH(OnSideCmd_cplt,C); */
//
//            if (MotorCmd.StopPosition >= tHall.Position)
//            {
//                /* PATH(OnSideCmd_cplt,D); */
//                delta = MotorCmd.StopPosition - tHall.Position;
//            }
//            else
//            {
//                /* PATH(OnSideCmd_cplt,E); */
//                delta = tHall.Position - MotorCmd.StopPosition;
//            }
//
//            /* If within x quad counts of desired position */
//            if (MotorCmd.MotorDirection == CCW)
//            {
//                /* PATH(OnSideCmd_cplt,F); */
//                delta = hallStartPosition - tHall.Position;
//            }
//            else if (MotorCmd.MotorDirection == CW)
//            {
//                /* PATH(OnSideCmd_cplt,G); */
//                delta = tHall.Position - hallStartPosition;
//            }
//
//            if (delta > 10)
//            {
//                /* PATH(OnSideCmd_cplt,H); */
//                Events_PostEvent(PositionComplete, 0);
//            }
//        }
    }

    /* PATH(OnSideCmd_cplt,I); */
}

/*
    brief Scheduler implementation for ALL timeslices, prior to the per-timeslice operations.

    note
        This should be invoked once on every scheduler timeslice.

*/
void allframes_proc(void)
{
    Adc_Sample_motor_phase_currents();
    /* PATH(allframes_proc,A); */
    A825Mgr_RxSrvc(&tA825_Bus[ARINC825_CNTRL_BUS]);     /* Run the ARINC 825 Receive Service */
    A825Mgr_TxSrvc(&tA825_Bus[ARINC825_CNTRL_BUS]);     /* Run the ARINC 825 Transmit Service */

    if(DFT_Tx_block == false)
    {
        DFT_TxSrvc();
    }

    Icc_RxMsg();
    Icc_TxSrvc();

#if 0
    if (G_bChannelA == false)
    {
        if (SpiTransmitt == true)
        {
            /* PATH(allframes_proc,G); */

            /* set flag to clear and fill transmit buffer in next timeslice */
            SpiTransmit_Delayed = true;
            SpiTransmitt = false;
        }
        else if (SpiTransmit_Delayed == true)
        {

            /* PATH(allframes_proc,B); */

            /* reset transmit buffer */
            SpiaRegs.SPICCR.bit.SPISWRESET = 0;     /* reset SPI peripheral */
            SpiaRegs.SPICCR.bit.SPISWRESET = 1;     /* reset SPI peripheral */
            SpiaRegs.SPIFFTX.bit.TXFIFO = 0;        /* clear FIFO transmit buffer */
            SpiaRegs.SPIFFTX.bit.TXFIFO = 1;        /* clear FIFO transmit buffer */
            SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;   /* clear FIFO receive buffer */
            SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;   /* clear FIFO receive buffer */
            SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;    /* clear FIFO overflow flag */

            /* call the Spi_SendMessage function to fill the transmit buffer */
            SpiTxStatus = Spi_SendMessage();

            /* clear flag indicating transmit buffer should be cleared and filled */
            SpiTransmit_Delayed = false;
        }
    }
#endif
//    /* If a powerdown situation occurs we need to transition into that mode immediately      */
//    if ((onSideSpiPwrDwn == true) && (McuGetState() != POWER_DOWN))
//    {
//        /* PATH(allframes_proc,C); */
//
//        if (McuGetState() == RUN_MODE)
//        {
//            /*PATH(allframes_proc,D); */
//            bitStatus[0x59].tripBoot = Nvm_State.bootcount;
//            bitStatus[0x59].data = 0;
//            Bit_LogToNvm(0x59);
//        }
//
//        Events_PostEvent(PowerInterrupt, 0);
//    }
//    else if ((McuGetState() == POWER_DOWN) && (onSideSpiPwrDwn == true) &&
//             (xSideSpiPwrDwn == true) && (McuGetOnsideExitPwrDown() == true) &&
//             (McuGetXsideExitPwrDown() == true))
//    {
//        /* PATH(allframes_proc,F); */
//        Events_PostEvent(PowerRestoration, 0);
//    }


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

    /* shall invoke MCU state machine to process any posted event */
    Events_CheckEvent();

	Adc_GetResults_motor_phases();
	Adc_GetMovingAverage();         /*Calculate the Moving Average required for performing fault monitors*/
    /* PATH(allframes_proc,E);      */
}

void Initialization_Tasks(void)
{
    /* PATH(Initialization_Tasks,A); */
    sys_init();                     /* Initialize GPIO, PLL, invoke init routines for DSP peripherals */
    Adc_Sample();                   /* Sample ADC so that +14V_MON can be read when determining channel */
    Gpio_EstablishChannel();        /* Establish primary/secondary control channel by reading PRGM inputs */
    Act_Init();                     /* Initialize Actuation Manager motor, hall inits called from here */
    Brake_Init();                   /* Initialize Brake Controller */

    if (G_bChannelA == true)
    {
        /*PATH(Initialization_Tasks,C);*/

        /* shall initialize which channel should start broadcasting GSE data first */
		//BroadcastNow = true;

		/* Set the Frame Period (FPER):  Register is 12-bit. Primary 3043.5Hz */
		McbspbRegs.SRGR2.bit.FPER = 248; /* 3043.5 Hz +/- 7.5 (((LSPCLK/CLKGDV)/3043.5)/64) - 1 = (((100MHz / 2) / 3043.5Hz) / 64) - 1 = 256 */
    }
    else if (G_bChannelA == false)
    {
        /*PATH(Initialization_Tasks,D);*/

		/* shall initialize which channel should start broadcasting GSE data first */
		//BroadcastNow = false;

		/* Set the Frame Period (FPER):  Register is 12-bit. Secondary 2967.5 Hz*/
        McbspbRegs.SRGR2.bit.FPER = 254; /* 2967.5 Hz +/- 7.5 (((LSPCLK/CLKGDV)/2967.5)/64) - 1 = (((100MHz / 2) / 2967.5) / 64) - 1 = 262 */
    }
#if defined (__SKEW_SNSR_RVDT__)
    Rvdt_SinewaveSend(true);        /* Begin transmitting sine wave for RVDT excitation */
#endif
    A825Mgr_Init();                 /* Initialize ARINC825 Manager for the CAN busses */
    DFT_Init();                     /* Initialize Serial Driver for DFT */
    Bit_InitializeStructures();     /* Initialize the Bit status structures */
    Nvm_ValidateRigStatus();        /* Verify that rig status is valid */
    ICC_GpioInit();
    Icc_Init();
    /* This timer is to be checked for expiry during Scheduler operation, and used to increment
     the global tNvm.tData.Nvm_RunTimeCount */
    Timer_SetTime(&RunTimeCountTimer, TIMER_ONESHOT, TIMER_1min); /* Create a timer for the RunTimeCounter */
    Spi_Init();                     /* Initialize SPI for CCC */
#if defined(SPI_ADC_TEST)
    Spia_Init();                    /* Simulate ADC chip, to be removed on actual hardware*/
#endif
    SPI_LoopbackTest();             /* Do Loopback test for SPI */
    Gse_CheckRigTest();             /* Check to see if the conditions to enter either Rig or Test mode exist. */

    /* PATH(Initialization_Tasks,B); */
}

void Boot_Tasks(void)
{
    Uint16 i;

    /* PATH(Boot_Tasks,A); */

    Bit_HandleLatchedFaults();                  /* Handle latched faults from last boot */
    //DFT_InitializeTimers();                     /* Initialize serial communication timers */
    //GseMode = BROADCAST;                        /* Set GSE to BROADCAST mode */
#if !defined(TRL4_NO_PDI_RIG)
    Nvm_RigStatusSetup(Nvm_State.rigstatus);    /* Based on rig status, setup system ability accordingly */
#endif
    for (i = 0; i < 10; i++)
    {
        /* PATH(Boot_Tasks,B); */
        Gpio_Capture();
    }

    Watchdog_Init();      /* Setup Watchdog again for normal run-time since WD PBIT test was done */
    /* PATH(Boot_Tasks,C); */
}

/*
    brief Process boot actions

     Purpose:
        This routine runs boot actions.

    param [in] fault  true if faults have been detected

    return
         None.

    Preconditions and Assumptions:
         None.

*/
void main_BootHandler(bool fault)
{
    bool PassFail = false;
    int i;

    /* PATH(main_BootHandler,A); */

    /* cold boot occurred */
    /* PATH(main_BootHandler,B); */

    /* shall reset frame overrun counters */
    for (i = 0; i < NUM_OF_FRAMES; i++)
    {
        /* PATH(main_BootHandler,C); */
        Nvm_State.frameOverCount[i] = 0;
    }

    /* shall reset all latched faults */
    /* shall perform PBIT */
    PassFail = PBitScheduler();
    tMcuStatus.bit.PBIT_Passed = !PassFail;

    /* shall reset status information */
    Nvm_State.bitstatus.all = 0x0000;


    if ((PassFail == false) && (fault == false))
    {
        /* PATH(main_BootHandler,J); */

        /* shall post an event indicating boot mode is complete */
        Events_PostEvent(EVENT_BOOT_DONE, 0);
    }
    else if ((PassFail == true) || (fault == true))
    {
        /* PATH(main_BootHandler,K); */

        /* shall post an event indicating a fault was detected */
        Events_PostEvent(EVENT_FAULT_DETECTED, 0);
    }
}

/*
    brief Configures system peripherals.

     Purpose:
        This routine is used to initialize GPIO, the PLL, and invoke initialization
        routines for the DSP peripherals.

    return  void

     Preconditions and Assumptions:
        Assumes that sys_init has not been invoked previously.

    remarks
    This should not be called more than once per reset cycle.

    The initialization sequence implemented herein should follow the sequence:
        -#  DSP system initialization
        -#  Initialize interrupt controller and default vector table
        -#  Initialize I/O pin multiplexers (Gpio_Init())
        -#  Initialize software driver components (CPU Timer, DFT, ARINC825)
        -#  Initialize NVM and retrieve information frame NVM
        -#  Initialize other necessary subsystems of DSP (ADC, GSE, RVDT, actuation, brake)

*/
void sys_init(void)
{
    /* PATH(sys_init,A); */

    /* Initialize System Control:  PLL, WD, and Enable Peripheral Clocks */
    /* shall initialize DSP subsystems */
    InitSysCtrl();

    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;


    /* Initialize the PIE control registers to their default state.
     * The default state is all PIE interrupts disabled and flags are cleared.
     * This function is found in the F2837xD_PieCtrl.c file. */
    /* REWORK */
    InitPieCtrl();

    /* Disable CPU interrupts and clear all CPU interrupt flags */
    IER = 0x0000;
    IFR = 0x0000;

    /* Initialize the PIE vector table with pointers to the shell Interrupt
     * Service Routines (ISR). This will populate the entire table, even if the interrupt
     * is not used in this example.  This is useful for debug purposes.
     * The shell ISR routines are found in F2837xD_DefaultIsr.c.
     * This function is found in F2837xD_PieVect.c. */
    /* REWORK */
    InitPieVectTable();

    /* Initialize the GPIO */
    Gpio_Init();  /* Initialize GPIO pins */

    /* shall initialize software driver components. Must come after GPIO_Init  */
    Timer_Init(); /* Initialize CPU Timer */

    /* shall initialize NVM data structures. Must come after GPIO_Init */
    Nvm_Init();

    Watchdog_TestWdReset(); /* Monitor the WDRST bit. Must be done after NVM init */

    /* shall initialize the necessary subsystems of the DSP */
    Adc_Init();     /* Initialize ADC */
    Gse_Init();     /* Initialize GSE SCI */
    /* Initialize the skew sensor */
#if defined(__SKEW_SNSR_RVDT__) /* Check if RVDT is the skew sensor */
    Rvdt_Init();    /* Initialize McBSP for sinewave generation  */
#elif defined(__SKEW_SNSR_ENCODER__) /* Check if Encoder is the skew sensor */
    Encoder_Init(); /*Initialize McBSP for Encoder communication*/
#endif

    /* shall enable Interrupts after initializing peripherals */
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
    /* PATH(sys_init,D); */
}

void Flash_Init(void)
{
    DisableDog();   /* Disable the Watchdog */

#ifdef _FLASH
    /*
     Copy time critical code and Flash setup code to RAM. This includes the
     following functions: InitFlash()

     The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
     symbols are created by the linker. Refer to the device .cmd file.
    */
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

    /* Setup the flash waitstates. This function must reside in RAM. */
    InitFlash();
#endif

}

void Watchdog_Init(void)
{
    volatile uint16_t u16_SCSR = 0U;

    DINT;
    EALLOW;

    /* Setup WD module for Reset event rather than Interrupt Event */
    /* Reset during normal operating conditions if a WD trip occurs */
    /* WD Interrupts are used for BITs to monitor the WD functionality */

    /* SCSR must be written as 16-bit value or bit flips can occur */
    u16_SCSR = WdRegs.SCSR.all;             /* Read current state of SCSR */
    u16_SCSR = u16_SCSR & WDENINT_MASK;     /* Mask off the Watchdog Interrupt Enable bit */
    u16_SCSR = u16_SCSR | ENABLE_WDRST;     /* Set WD RESET to ENABLED, WD INT is DISABLED */
    WdRegs.SCSR.all = u16_SCSR & 0xFFFEU;   /* Write to WD SCSR register as uint16 with exception of WDOVERRIDE*/

    EINT;

    ServiceDog();   /* Service the WD */

    EDIS;

    EnableDog();    /* Configure, Enable, and Start the WD */
}

void Watchdog_TestWdReset(void)
{
    volatile uint16_t u16_WDRS = 0U;

    /* Read RESC.WDRS on startup to see if WD caused reset or not */
    u16_WDRS = CpuSysRegs.RESC.bit.WDRSn;

    /* Store WDRS value to NVM */
    tNvm.tData.WDRS = u16_WDRS;

    /* Reset the WDRS bit in register if WD caused reset */
    if (u16_WDRS != 0U)
    {
        /* CPU was reset due to a Watchdog event */
        // Store NVM Record here for future release
        CpuSysRegs.RESC.bit.WDRSn = RESET_WDRS; /* Reset the WD Reset Bit */
        tNvm.tData.WDRS_Counter++;              /* Increment number of WD Reset Events in NVM */
    }
}

/****************************************************************************************************
*  Function: PDI_Init
*  Purpose: This function runs CRC and Range integrity checks on the memory. Monitors 0x7A and 0x7B
*  Input:
*  Output:
****************************************************************************************************/
void PDI_Init(void)
{
    /* Run the CRC Check for the PDI memory */
    Bit_CriticalMonitors(0x7A, true);

    /* Run the PDI Out-of-Range Check */
    Bit_CriticalMonitors(0x7B, true);
}

/****************************************************************************************************
*  Function: main_BswSetAswAnlgData
*  Purpose: This function sets the Analog Input data in the ASW/BSW interface for the ASW to use.
*  Input:
*  Output:
****************************************************************************************************/
void main_BswSetAswAnlgData(void)
{
    G_BSW_DATA.tAnalogInputData.f32_5V_SENSE = Adc_f32UnitValAvg.val.f32_5V_SENSE;
    G_BSW_DATA.tAnalogInputData.f32_I_PHA_CHA = Adc_f32UnitValAvg.val.f32_I_PHA_CHA;
    G_BSW_DATA.tAnalogInputData.f32_I_PHB_CHA = Adc_f32UnitValAvg.val.f32_I_PHB_CHA;
    G_BSW_DATA.tAnalogInputData.f32_I_PHC_CHA = Adc_f32UnitValAvg.val.f32_I_PHC_CHA;
    G_BSW_DATA.tAnalogInputData.f32_MOTOR_TEMP_PHA = Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHA;
    G_BSW_DATA.tAnalogInputData.f32_MOTOR_TEMP_PHB = Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHB;
    G_BSW_DATA.tAnalogInputData.f32_MOTOR_TEMP_PHC = Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHC;
    G_BSW_DATA.tAnalogInputData.f32_NEGATIVE_15V_SENSE = Adc_f32UnitValAvg.val.f32_NEGATIVE_15V_SENSE;
    G_BSW_DATA.tAnalogInputData.f32_POSITIVE_15V_SENSE = Adc_f32UnitValAvg.val.f32_POSITIVE_15V_SENSE;
    G_BSW_DATA.tAnalogInputData.f32SpeedRef = f32SpeedRef;
    G_BSW_DATA.tAnalogInputData.f32SpeedFdb = tSpeed.Speed;
    G_BSW_DATA.tAnalogInputData.f32_I_AVR_MAX = Adc_f32UnitValAvg.val.f32_MOTOR_PHASEx_I;
}

/****************************************************************************************************
*  Function: main_BswSetAswDiscData
*  Purpose: This function sets the Discrete Input data in the ASW/BSW interface for the ASW to use.
*  Input:
*  Output:
****************************************************************************************************/
void main_BswSetAswDiscData(void)
{
    G_BSW_DATA.tDiscreteInputData.eLruId = G_eLruId;    /* LRU ID */
    G_BSW_DATA.tDiscreteInputData.eChId = G_eActId;      /* Channel ID */
    G_BSW_DATA.tDiscreteInputData.eActuatorNumber = G_eActuatorNumber;   /* Actuator ID */
    G_BSW_DATA.tDiscreteInputData.bVcModeCmd = tPanel.bVcModeCmd;
    G_BSW_DATA.tDiscreteInputData.ResetIntegralGain = ResetIntegralGain;
    G_BSW_DATA.tDiscreteInputData.bCHX_STATUS = CHX_STATUS;
    G_BSW_DATA.tDiscreteInputData.xChannelEnable_M = LatchedWarnings_Stat.bit.xChannelEnable_M;
    G_BSW_DATA.tDiscreteInputData.bSensorFusionEnableCmd = tPanel.bSensorFusionEnableCmd;
    G_BSW_DATA.tDiscreteInputData.u16SCU_MCU_ENABLE = MTR_EN_CHA;    /* MOTOR_ENABLE */
}

/****************************************************************************************************
*  Function: main_BwsSetAswPanelData
*  Purpose: This function sets the Panel Input data in the ASW/BSW interface for the ASW to use.
*  Input:
*  Output:
****************************************************************************************************/
void main_BwsSetAswPanelData(void)
{
    /* Set Panel Data */
    G_BSW_DATA.tPanelInputData.f32StrokeRvdt = tSkewSnsrCalcs.tStroke.f32Stroke;  /* Actuator Stroke Position in inches from Skew Sensor Count */
    G_BSW_DATA.tPanelInputData.f32FlapAngleRvdt = tSkewSnsrCalcs.tFlapAngle.f32FlapAngle;  /* Flap Angle Degrees Stream-wise from Skew Sensor Count */
    G_BSW_DATA.tPanelInputData.f32StrokeQuad = tPanel.f32StrokeQuad;  /* Actuator Stroke Position in inches from Quad Count */
    G_BSW_DATA.tPanelInputData.f32FlapAngleQuad = 0.0F; /* Flap Angle Degrees Stream-wise from Quad Count */
    G_BSW_DATA.tPanelInputData.f32StrokeFused = G_ASW_DATA.tPanelOutputData.f32StrokeFused;     /* Last copy of Stroke Fused used for SF Initial Position. */
    G_BSW_DATA.tPanelInputData.s16QuadPosition = tHall.Position; /* Quad Count position of the actuator */
    G_BSW_DATA.tPanelInputData.s16QuadCntRvdt = tSkewSnsrCalcs.tStroke.s16QuadCnt; /* Quad count calculated from Skew Sensor Stroke */

}

/****************************************************************************************************
*  Function: main_RunAsw
*  Purpose: This function is will run the ASW OFP program. Prior to calling the ASW the function will
*  copy data over to the ASW/BSW interface so the ASW has the most up-to-date data to consume.
*  After the ASW has run the function will call any necessary functions to consume output data from
*  the ASW.
*  Input:
*  Output:
****************************************************************************************************/
void main_RunAsw(void)
{
    /* Input data (BSW->ASW) is in the G_BSW_DATA Structure */
    // set input data for ASW here
    main_BswSetAswAnlgData();
    main_BswSetAswDiscData();
    main_BwsSetAswPanelData();

    /* Run the ASW (Operational Flight Program) Software */
    OFP_step();

    /* Output data (ASW->BSW) is in the G_ASW_DATA Structure */

}

/* end main.c */

