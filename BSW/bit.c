/****************************************************************************************************
*  File name: bit.c
*
*  Purpose: Provides facilities for managing and invoking BIT tests
*  This file contains the API definition for the BIT Framework.
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
#include "parameter.h"
#include "nvm.h"
#include "mcu.h"
#include "timer.h"
#include "bitmonitors.h"
#include "bit.h"
#include "actuation.h"
#include "gpio.h"
#include "motor.h"
#include "hallsensor.h"
#include "spi.h"
#include "adc.h"
//#include "rvdt.h"
#include "brake.h"
#include "skewsensor.h"

/*      Local Type Definitions
*/

/*      Local Defines */
#define BIT_CREEP_QUAD_LIMIT    125

/*      Global Variables
*/
Uint16 LatestFaultCode = 0; /* Most recent fault code (monitor ID) for GSE reporting*/
Uint16 LatestFaultData = 0; /* Fault data associated with the fault code reported*/
Uint16 SystemFaultCode = 0;
Uint16 SystemFaultData = 0;
CRITICALITY SystemCriticality = NO_FAULT;

bool xSideJamFlg = false;
bool onSideJamFlg = false;
bool OnSideFault = false;
bool XSideSpiFault = false;
bool XSideFaultNvmFlg = false;
bool OnGroundCondition = false;
bool PowerdownFault = false;
Ibit_Status Ibit_Stat = {0, 0, 0, false, NOT_STARTED};
Timer_t IBITDelayTimer = TIMER_DEFAULTS;
Timer_t ADCDelayTimer = TIMER_DEFAULTS;
Timer_t tBiDirectTstTimer = TIMER_DEFAULTS;

Uint16 FlightCycleCount = 0;

#define INHIBIT_QUEUE_SIZE      16
Uint16 InhibitQueue[INHIBIT_QUEUE_SIZE] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
Uint16 InhibitIndex = 0;
Uint16 InhibitSize = 0;

CriticalFaults_t CriticalFaults_Stat = {0};
LatchedFaults_t LatchedFaults_Stat = {0};
InhibitMonitors_t Inhibits_Stat = {0};
LatchedWarnings_t LatchedWarnings_Stat = {0};

bool Bit_FslTimeMtr = false;
bool Bit_FslPosMtr =  false;

bool CreepTestStart = false;
bool CreepTestCmplt = false;
Uint16 CreepPhase = 0;
bool PwrDownWrite = false;
WDtest_t WDTestStatus = {0,0,false};
bool IBITFault = false;
Uint16 b200msmonStage = 1;

bool greaterThan18V = false;
bool bVbusAbove24V = false;
bool greaterThanEqualto22V = false;

/*      Local ROM Constants
*/

/*      Local Variable Declarations
*/


/*      Local Function Prototypes
*/
void Inhibit_QueueAdd(Uint16 id);
Uint16 Inhibit_QueueRemove(void);
Uint16 Inhibit_QueuePeek(void);
bool Inhibit_QueueEmpty(void);
bool FaultResetCheck(void);
void main_BootHandler(bool);
__interrupt void WDINT_ISR(void);

/*      Function Definitions
*/

/*
    brief Conducts all "rig mode" IBIT Tests, based on passed ID.

    Purpose:
        Call the corresponding IBIT test.

        param[in] testId       index into Fault Matrix tables
        
    retval false   Function always returns false.

    par Preconditions and Assumptions:
         None.

    Non-Compliance: This function exceeds 100 LOC.
    Justification: The use of the switch statement makes the complexity of this function manageable
        despite its length.

    Non-Compliance: This function exceeds 3 nesting levels.
    Justification: The algorithm is clear enough that this amount of nesting is not excessively complex.
*/
void Bit_Ibit(Uint16 testId)
{
//    static bool oneShot = false;
//    Uint16 testToRun;
//
//    /* PATH(Bit_Ibit,A);*/
//    testToRun = testId;
//
//	if ((testId == 0x13) && (Nvm_State.rigstatus == NOT_RIGGED))
//    {
//		/* PATH(Bit_Ibit,EG); */
//    	testToRun = 0;
//  	}
//
//    switch (testToRun)
//    {
//        case 0x07:
//            /* PATH(Bit_Ibit,AT);*/
//
//            if (oneShot == false)
//            {
//                /* PATH(Bit_Ibit,AU);*/
//                Bit_DoTest(0x07, 0);
//                oneShot = true;
//                Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_5s);
//            }
//
//            if (Timer_IsExpired(&IBITDelayTimer) == true)
//            {
//                /* PATH(Bit_Ibit,AV);*/
//                Ibit_Stat.result = (bool)(Bit_CheckMonitor(testId));
//                Ibit_Stat.tstCmplt = COMPLETED;
//                xCommsInitialCount = 0;
//            }
//            break;
//        case 0x10:
//            /* PATH(Bit_Ibit,BN); */
//
//            if (RIG_CRC_done == false)
//            {
//                /* PATH(Bit_Ibit,BO); */
//                Ibit_Stat.result = (bool)(Bit_CheckMonitor(testId));
//            }
//            else
//            {
//                /* PATH(Bit_Ibit,BP); */
//                Ibit_Stat.tstCmplt = COMPLETED;
//                RIG_CRC_done = false;
//            }
//            break;
//        case 0x13:
//            /* PATH(Bit_Ibit,BQ); */
//
//            if (Ibit_Stat.tstCmplt != COMPLETED)
//            {
//                /* PATH(Bit_Ibit,BR); */
//                Ibit_Stat.result = (bool)(Bit_DoTest(testId, FreePlayStage));
//            }
//            break;
//        case 0x25:
//            /* PATH(Bit_Ibit,BS);*/
//
//			if (VBUS_MON > BIT_VBUSMONNOBRK_LIMIT)
//			{
//    	        /* PATH(Bit_Ibit,BX);*/
//
//	            if (oneShot == false)
//	            {
//	                /* PATH(Bit_Ibit,BT); */
//	                Bit_DoTest(0x25, 0);
//	                oneShot = true;
//	            }
//
//	            if ((BrakeSwitchTestCmplt == false) && (Timer_IsExpired(&IBITDelayTimer) == true))
//	            {
//	                /* PATH(Bit_Ibit,BU); */
//	                Ibit_Stat.result = (bool)(Bit_DoTest(0x25, BrakeSwitchMonStage));
//	            }
//	            else if (BrakeSwitchTestCmplt == true)
//	            {
//	                /* PATH(Bit_Ibit,BV); */
//	                Ibit_Stat.tstCmplt = COMPLETED;
//	                BrakeSwitchTestCmplt = false;
//	                BrakeSwitchMonStage = 0;
//	            }
//			}
//			else
//			{
//	            /* PATH(Bit_Ibit,BY);*/
//				/* We need to indicate a test failure for IBIT (VBUS_MON too low) */
//			    Ibit_Stat.tstCmplt = COMPLETED;
//                Ibit_Stat.result = true;
//                bitStatus[0x25].data = 0xF0;
//			}
//            break;
//        /* Non-Compliance: The following case statements lack a break statement.
//           Justification: The same functionality is required for all of the following cases.
//        */
//        case 0x30:
//            /* PATH(Bit_Ibit,CE); */
//        case 0x34:
//            /* PATH(Bit_Ibit,CI); */
//        case 0x40:
//            /* PATH(Bit_Ibit,CK); */
//            break;
//        case 0x50:
//            /* PATH(Bit_Ibit,CX); */
//
//            if (VBUS_MON > BIT_VBUSMONFAIL_LIMIT)
//			{
//                /* PATH(Bit_Ibit,EE); */
//	            if (oneShot == false)
//	            {
//	                /* PATH(Bit_Ibit,CY);*/
//	                Bit_DoTest(0x50, BridgeMonStage);
//	                oneShot = true;
//	            }
//
//	            if ((BridgeMonTestCmplt == false) && (Timer_IsExpired(&IBITDelayTimer) == true))
//	            {
//	                /* PATH(Bit_Ibit,CZ);*/
//	                Ibit_Stat.result = (bool)(Bit_DoTest(0x50, BridgeMonStage));
//	            }
//	            else if (BridgeMonTestCmplt == true)
//	            {
//	                /* PATH(Bit_Ibit,DA);*/
//	                Ibit_Stat.tstCmplt = COMPLETED;
//	                BridgeMonTestCmplt = false;
//	                BridgeMonStage = 0;
//	            }
//			}
//			else
//			{
//                /* PATH(Bit_Ibit,EF); */
//				/* We need to indicate a test failure for IBIT (VBUS_MON too low) */
//			    Ibit_Stat.tstCmplt = COMPLETED;
//                Ibit_Stat.result = true;
//                bitStatus[0x50].data = 0xF0;
//			}
//            break;
//        case 0x57:
//            /* PATH(Bit_Ibit,DG); */
//            break;
//        case 0x70:
//            /* PATH(Bit_Ibit,DK); */
//
//            if (ROM_CRC_done == false)
//            {
//                /* PATH(Bit_Ibit,DL); */
//                Ibit_Stat.result = (bool)(Bit_CheckMonitor(testId));
//            }
//            else
//            {
//                /* PATH(Bit_Ibit,DM); */
//                Ibit_Stat.tstCmplt = COMPLETED;
//                ROM_CRC_done = false;
//            }
//            break;
//        case 0x71:
//            /* PATH(Bit_Ibit,DN); */
//
//            /* Stop the sine wave to prevent a DC voltage going into the sine wave.*/
//            if (RAM_Test_done == false)
//            {
//                /* PATH(Bit_Ibit,DO); */
//                Ibit_Stat.result = (bool)(IBIT_RamTest());
//            }
//            else
//            {
//                /* PATH(Bit_Ibit,DP); */
//                Ibit_Stat.tstCmplt = COMPLETED;
//                RAM_Test_done = false;
//            }
//            break;
//        case 0x73:
//            /* PATH(Bit_Ibit,DQ); */
//
//            if (oneShot == false)
//            {
//                /* PATH(Bit_Ibit,DR); */
//                Bit_StartDog(WDTestStatus.stage);
//                Timer_SetTime(&IBITDelayTimer, TIMER_ONESHOT, TIMER_1ms * (BIT_WDTEST_mSEC + 1));
//                oneShot = true;
//            }
//
//            if ((WDTestStatus.PBitServiced == true) || (Timer_IsExpired(&IBITDelayTimer) == true))
//            {
//                /* PATH(Bit_Ibit,DS);*/
//                Ibit_Stat.result = (bool)(Bit_CheckMonitor(testId));
//
//                if (WDTestStatus.PBitServiced == false)
//                {
//                    /* PATH(Bit_Ibit,DT); */
//                    dsp281x_KickDog();
//                    Bit_StartDog(WDTestStatus.stage + 1);
//                    PieCtrlRegs.PIEIER1.bit.INTx8 = 0;          /* Turn off interrupt for the WD */
//                }
//
//                WDTestStatus.stage = 0;
//                WDTestStatus.PBitServiced = false;
//                Ibit_Stat.tstCmplt = COMPLETED;
//            }
//            break;
//        case 0xC0:
//            /* PATH(Bit_Ibit,DU); */
//
//            if (FindDC_Offset == true)
//            {
//                /* PATH(Bit_Ibit,DV); */
//                IBIT_ASYM_Offset();
//            }
//            else
//            {
//                /* PATH(Bit_Ibit,DW); */
//                Ibit_Stat.tstCmplt = COMPLETED;
//                Ibit_Stat.result = false;
//            }
//            break;
//        case 0x0:
//            /* PATH(Bit_Ibit,DX); */
//            Ibit_Stat.TestData = 9999;
//        default:
//            /* PATH(Bit_Ibit,DY); */
//            Ibit_Stat.result = false;
//            Ibit_Stat.tstCmplt = COMPLETED;
//            break;
//    }
//
//    if (Ibit_Stat.tstCmplt == COMPLETED)
//    {
//        /* PATH(Bit_Ibit,DZ); */
//        Timer_ResetTimer(&IBITDelayTimer);
//
//        if (testId != 0xC0)
//        {
//            /* PATH(Bit_Ibit,EA); */
//            Ibit_Stat.TestData = bitStatus[testId].data;
//
//            if (bitStatus[testId].tripBoot != 0)
//            {
//                /* PATH(Bit_Ibit,EB); */
//                IBITFault = true;
//
//                if (Bit_GetFaultCriticality(testId) != INHIBIT)
//                {
//                    /* PATH(Bit_Ibit,EC); */
//                    bitStatus[testId].tripBoot = 0;
//                }
//            }
//
//            oneShot = false;
//        }
//    }
//
//    /* PATH(Bit_Ibit,ED); */
}

/*
    brief Conducts all "rig mode" CBIT Tests, in sequence.

    Purpose:
        Runs all "CBIT" test procedures, for the rig mode scheduler.  This is a convenience function that 
        allows the user to invoke a single function instead of a repeated list of calls to Bit_CheckMonitor().

    param[in] subframe      Scheduler frame in which a specific monitor will be executed.


    retval false   Function always returns  false.

     Preconditions and Assumptions:
        This routine must only be operated as part of the Rig-Mode scheduler.

*/
void Bit_Rbit( Uint16 subframe )
{
    static Uint16 RBitFrame = 0;

    /* shall exercise the RVDT Position sense monitors (30-35) after ADC processing is complete, during frame 5.

     shall exercise the FSL monitors (40-44) during one of the 8 scheduler frames.
     Monitors 0x42-0x45 have been implemented as inline monitors
     Do we need to execute these monitor before we sample our FSL?
     shall exercise the 200ms timer monitor (54) during one of the 8 scheduler frames, while "on ground"(Moved to PBIT and IBIT only)

     This monitor has been implemented at an inline monitor.
     shall exercise the Quadrature count check (18) during one of the 8 scheduler frames,
         while at RMS only, and only when commanded from position 0 to position 1.*/

    /* PATH(Bit_Rbit,A); */

    /* Monitor 0x01 is not executed during RIG_MODE
       The monitor in this requirement are not executed during RIG_MODE
       shall schedule BIT monitors in one eight scheduler frames*/
    switch (subframe)
    {
        case 0:
            /* PATH(Bit_Rbit,B);*/

            /* shall exercise the subframe index monitor (76) during frame 0.*/
            Bit_LatchedMonitors(0x76, true);
            /* Run the GFD_I_SENSE Monitor */
            Bit_CriticalMonitors(0x7D, true);
            break;
        case 1:
            /* PATH(Bit_Rbit,H); */
            
            /* shall schedule BIT monitors in frame 1 using subframes that are executed at 250 Hz instead of 1 kHz*/
            switch (RBitFrame)
            {
                case 0:
                    /* PATH(Bit_Rbit,I); */
                    /* shall exercise the undefined interrupt monitor (77) during frame 0.*/
                    Bit_LatchedMonitors(0x77, true);
                    break;
                case 1:
                    /* PATH(Bit_Rbit,J); */
                    /* shall exercise the stack overrun monitor (79) during frame 0.*/
                    Bit_LatchedMonitors(0x79, true);
                    break;
                case 2:
                    /* PATH(Bit_Rbit,K); */
                    Bit_CriticalMonitors(0x12, true);
                    break;
                case 3:
                    /* PATH(Bit_Rbit,L); */
                    Bit_LatchedMonitors(0x33, true);
                    break;
                default:
                    /* PATH(Bit_Rbit,M); */
                    break;
            }
            break;
        case 2:
            /* PATH(Bit_Rbit,N); */
            /* shall exercise the Flap Jam monitor (04) during one of the 8 scheduler frames.*/
//            if (MotorCmd.RiggingHardStop == false)
//            {
                /* PATH(Bit_Rbit,O); */
                Bit_InhibitMonitors(0x04, true);
//            }

              switch (RBitFrame)
              {
              case 0:
                  /*0x2A - Critical Monitor - Phase to Neutral Short Asymmetric (Motor Winding Failure)*/
                  /*Start the monitor 600ms after the motor starts */
                  if (Timer_IsExpired(&Bit_PhNeuDelayTimer) == true)
                  {
                      Bit_CriticalMonitors(0x2A, true);
                  }
                  break;
              case 1:
                  /*0x2B* - Critical Monitor - One Phase Open Circuit Short (Motor Winding Failure)  */
                  /*Start the monitor 200ms after the motor starts*/
                  if (Timer_IsExpired(&Bit_OpCktDelayTimer) == true)
                  {
                      Bit_CriticalMonitors(0x2B, true);
                  }
                  break;
              case 2:
                  /*For Future Monitors*/
                  break;
              case 3:
                  /*For Future Monitors*/
                  break;
              default:
                  break;

              }

            break;
        case 3:
            /* PATH(Bit_Rbit,P); */
            
            /* shall schedule BIT monitors in frame 3 using subframes that are executed at 250 Hz instead of 1 kHz*/
            switch (RBitFrame)
            {
                case 0:
                    /* PATH(Bit_Rbit,Q); */
#if defined(__SKEW_SNSR_RVDT__)
                    Bit_LatchedMonitors(0x30, true);
#endif
                    break;
                case 1:
                    /* PATH(Bit_Rbit,R); */
#if defined(__SKEW_SNSR_ENCODER__)
                    Bit_CriticalMonitors(0x36, true);
#endif
                    break;
                case 2:
                    /* PATH(Bit_Rbit,S); */
                    /*Run the monitor to check if DC_BUS_CHA is within the specified limit during High-lift & VC mode*/
                    Bit_LatchedMonitors(0x51, true);
                    break;
                case 3:
                    /* PATH(Bit_Rbit,T); */
                    /*Run the monitor to check if +28VDC_CHA bus is within the specified limit*/
                    Bit_LatchedMonitors(0x52, true);
                    break;
                default:
                    /* PATH(Bit_Rbit,U); */
                    break;
            }
            break;
        case 4:
            /* PATH(Bit_Rbit,V); */

            switch (RBitFrame)
            {
                case 0:
                    /* 0x16-Inhibit monitor - Winding over Temp.*/
                     Bit_InhibitMonitors(0x16, true);
                    break;
                case 1:
                    Bit_LatchedMonitors(0x21, true);
                    break;
                case 2:
                    /*Cross channel CRC and timout monitor*/
                    if (Timer_IsExpired(&Bit_XCommsDelayTimer) == true)
                    {
                       Bit_LatchedMonitors(0x07, true);
                    }
                    break;
                case 3:
                    /* shall exercise the Cross-channel enable monitor (06) during one of the 8 scheduler frames.*/
                    Bit_LatchedMonitors(0x06, true);
                    break;
                default:
                    break;
             }

            break;
        case 5:
            /* PATH(Bit_Rbit,W); */
            
            /* shall schedule BIT monitors in frame 5 using subframes that are executed at 250 Hz instead of 1 kHz*/
            switch (RBitFrame)
            {
                case 0:
                    /* PATH(Bit_Rbit,X); */
                    Bit_LatchedMonitors(0x34, true);
                    break;
                case 1:
                    /* PATH(Bit_Rbit,Y); */
                    
                    /*Run the monitor to check the health of Inverter Gate Driver*/
                    Bit_LatchedMonitors(0x56, true);
                    break;
                case 2:
                    /* PATH(Bit_Rbit,Z); */
                    Bit_CriticalMonitors(0x35, true);
                    break;
                case 3:
                    /* PATH(Bit_Rbit,AA); */
                    Bit_LatchedMonitors(0x22, true);
                    break;
                default:
                    /* PATH(Bit_Rbit,AB); */
                    break;
            }
            break;
        case 6:
            /* PATH(Bit_Rbit,AC); */
            
            /* shall schedule BIT monitors in frame 6 using subframes that are executed at 250 Hz instead of 1 kHz*/
            switch (RBitFrame)
            {
                case 0:
                    /* PATH(Bit_Rbit,AD); */
                    /* Run the Flap Position Command Skew Monitor for ARINC825_MAINT_BUS */
                    Bit_InhibitMonitors(0x49, true);
                    break;
                case 1:
                    /* PATH(Bit_Rbit,AE); */
                    /* Run the Bus Passive Monitor for ARINC825_MAINT_BUS */
                    Bit_LatchedMonitors(0x4B, true);
                    break;
                case 2:
                    /* PATH(Bit_Rbit,AF); */
                    /* Run the Bus Off Monitor for ARINC825_MAINT_BUS */
                    Bit_LatchedMonitors(0x4D, true);
                    break;
                case 3:
                    /* Run the SYNC Communications Stale Monitor for ARINC825_MAINT_BUS */
                    Bit_InhibitMonitors(0x4F, true);
                    break;
                default:
                    /* PATH(Bit_Rbit,AH); */
                    break;
            }
            break;
        case 7:
            /* PATH(Bit_Rbit,AI); */
            
            /* shall schedule BIT monitors in frame 7 using subframes that are executed at 250 Hz instead of 1 kHz*/
            switch (RBitFrame)
            {
                case 0:
                    /*Run the monitor to check if +15V_SENSE (+15VISO_CHA) is within the specified limit*/
                    Bit_LatchedMonitors(0x53, true);
                    break;
                case 1:
                    /* PATH(Bit_Rbit,AK); */
                    Bit_LatchedMonitors(0x32, true);
                    break;
                case 2:
                    /*Run the monitor to check if -15V_SENSE (-15VISO_CHA) is within the specified limit*/
                    Bit_LatchedMonitors(0x54, true);
                    break;
                case 3:
                    /*Run the monitor to check if 5V_SENSE (+5V_AN_CHA) is within the specified limit*/
                    Bit_LatchedMonitors(0x55, true);
                    break;
                default:
                    /* PATH(Bit_Rbit,AN); */
                    break;
            }

            /* shall increment the subframe counter for the next occurrence*/
            RBitFrame++;
            RBitFrame = RBitFrame & 3; /* insure a count between 0-3*/
            break;
        default:
            /* PATH(Bit_Rbit,AP); */
            break;
    }

    /* shall exercise Monitor 0x74 during RBIT (Frame Overrun)*/
    Bit_LatchedMonitors(0x74, true);
        
    /* PATH(Bit_Rbit,AQ); */
}


void Bit_Pbit(Uint16 subframe)
{
//    static bool brkTstStart = false;
//    static bool oneShot = false;
//    static bool oneShot2 = false;
    static Uint16 PBitFrame = 0;
    bool bTestDone = false;

    /* PATH(Bit_Pbit,A);*/


    /* shall schedule BIT monitors in one eight scheduler frames*/
    switch (subframe)
    {
        case 1:
            /* PATH(Bit_Pbit,G); */
            /* Run the Rig Verify Test until it has completed */
            if(bRigVerifyTstCmplt == false)
            {
                bRigVerifyTstCmplt = Bit_DoTest(0x64, (Uint16)tRigVerifyTstState);
            }
            break;
        case 2:
            /* PATH(Bit_Pbit,K); */
            
            switch (PBitFrame)
            {
                case 0:
                    /* PATH(Bit_Pbit,L); */
                    Bit_CriticalMonitors(0x10, true);
                    break;
                case 1:
                case 2:
                case 3:
                    /* PATH(Bit_Pbit,O); */
                    break;
				default:
    				/* PATH(Bit_Pbit,BJ); */
    				break;
            }
            break;
        case 4:
            /* PATH(Bit_Pbit,V); */

            if (Timer_IsExpired(&ADCDelayTimer) == true)
            {
                if (Timer_IsExpired(&IBITDelayTimer) == true)
                {

                }
                /*  TBD Inverter Testing and Brake Testing */
//                /* PATH(Bit_Pbit,W); */
//
//                if ((VBUS_MON > BIT_VBUSMONFAIL_LIMIT) || (greaterThan18V == true)) /* Motor voltage above 18V */
//                {
//                    /* PATH(Bit_Pbit,X); */
//					greaterThan18V = true;
//
//					/* Execute Monitor 0x50 */
//                    if ((HwAsymTestComplete == true) && (BridgeMonTestCmplt == false) &&
//                    	(Timer_IsExpired(&IBITDelayTimer) == true))
//                    {
//                        /* PATH(Bit_Pbit,Y); */
//                        Bit_DoTest(0x50, BridgeMonStage);
//                    }
//
//					if (BridgeMonTestCmplt == true) /* Monitor 0x50 is complete */
//					{
//						/* PATH(Bit_Pbit,BL); */
//
//						if ((VBUS_MON >= BIT_VBUSMONNOBRK_LIMIT) || (greaterThanEqualto22V == true)) /* Motor voltage at or above 22V */
//						{
//							/* PATH(Bit_Pbit,BI); */
//
//							if ((VBUS_MON == BIT_VBUSMONNOBRK_LIMIT) && (greaterThanEqualto22V == false)) /* Motor voltage at 22V = Don't run 0x25 */
//							{
//								/* PATH(Bit_Pbit,BM); */
//								BrakeSwitchMonStage = 4;
//								Bit_DoTest(0x25, BrakeSwitchMonStage);
//								BrakeSwitchMonStage++;
//							}
//
//							greaterThanEqualto22V = true;
//
//							if ((BrakeSwitchTestCmplt == false) && (Timer_IsExpired(&IBITDelayTimer) == true))
//							{
//								/* PATH(Bit_Pbit,Z); */
//								Bit_DoTest(0x25, BrakeSwitchMonStage);
//							}
////							else if ((BrakeSwitchTestCmplt == true) && (brkTstStart == false) &&
////									 (InAirOnGroundStatus == ON_GROUND))
////				            {
////				            	/* PATH(Bit_Pbit,AA); */
////
////								if ((CriticalFaults_Stat.bit.HW_AsymCM_M == 0) && (LatchedFaults_Stat.bit.BridgeMonitor_M == 0) &&
////									(LatchedFaults_Stat.bit.BrakeSwitch_M == 0))
////								{
////				                    /* PATH(Bit_Pbit,AB); */
////					                brkTstStart = true;
////					                Bit_DoTest(0x23, 1);
////								}
////				            }
//						}
//						else /* Motor voltage less than 22V */
//						{
//							/* PATH(Bit_Pbit,AC); */
//	                    	BrakeSwitchMonStage = 4;
//							Bit_DoTest(0x25, BrakeSwitchMonStage);
//							BrakeSwitchMonStage++;
//
////							Bit_DoTest(0x23, 3);
//
//							/*This will lock out the execution for the remainder of PBit*/
//		                    Timer_SetTime(&ADCDelayTimer, TIMER_ONESHOT, TIMER_2s);
//
////		                    if (InAirOnGroundStatus == ON_GROUND)
////		                    {
////		                    	/* PATH(Bit_Pbit,BK); */
////		                    	CriticalFaults_Stat.bit.BrakeHold_M = 1;
////		                	}
//						}
//					}
//                }
//                else /* Motor voltage less than or equal to 18V = Don't run 0x50, 0x25, or 0x23 */
//                {
//                    /* PATH(Bit_Pbit,AD); */
//					BridgeMonStage = 11;
//					Bit_DoTest(0x50, BridgeMonStage);
//					BridgeMonStage++;
//
//                    BrakeSwitchMonStage = 4;
//                    Bit_DoTest(0x25, BrakeSwitchMonStage);
//					BrakeSwitchMonStage++;
////
////                    Bit_DoTest(0x23, 3);
//
//                    /*This will lock out the execution for the remainder of PBit*/
////                    Timer_SetTime(&ADCDelayTimer, TIMER_ONESHOT, TIMER_2s);
//                      Timer_SetTime(&ADCDelayTimer, TIMER_ONESHOT, TIMER_10s);
//
////                    if (InAirOnGroundStatus == ON_GROUND)
////                    {
////                    	/* PATH(Bit_Pbit,BH); */
////                    	CriticalFaults_Stat.bit.BrakeHold_M = 1;
////                	}
//                }
            }

            /* Set these flags to true so we can start the brake test.*/
            HwAsymTestComplete = true;


            break;
        case 5:
            /* PATH(Bit_Pbit,AN); */
            
            /* shall schedule BIT monitors in frame 5 using subframes that are executed at 250 Hz instead of 1 kHz */
            switch (PBitFrame)
            {
                case 0:
                    /* PATH(Bit_Pbit,AO); */
#if defined(__SKEW_SNSR_RVDT__)
                    Bit_LatchedMonitors(0x30, true);
#endif
                    break;
                case 1:
                    /* PATH(Bit_Pbit,AP); */
#if defined(__SKEW_SNSR_ENCODER__)
                    Bit_CriticalMonitors(0x36, true);
#endif
                    break;
                case 2:
                    /* PATH(Bit_Pbit,AQ); */
                    break;
                case 3:
                    /* PATH(Bit_Pbit,AR); */
                    break;
                default:
                    /* PATH(Bit_Pbit,AS); */
                    break;
            }
            break;
        case 6:
            /* PATH(Bit_Pbit,AT); */
            
//            if (InAirOnGroundStatus == ON_GROUND)
//            {
//                /* PATH(Bit_Pbit,AU); */
//
//                if ((oneShot == false) && (brkTstStart == true) && (X_ENBL_IN_N == 0))
//                {
//                    /* PATH(Bit_Pbit,AV); */
//
//                    /* Try and commutate the motor with the brake engaged, and start 500ms timer*/
//                    Bit_DoTest(0x23, 2);
//                    oneShot = true;
//                }
//
//                if ((Timer_IsExpired(&brakeHoldTimer) == true) && (oneShot2 == false))
//                {
//                    /* PATH(Bit_Pbit,AW); */
//
//                    /* When 500ms timer is expired, stop the motor commutation commands */
//                    Bit_DoTest(0x23, 3);
//                    oneShot2 = true;
//                }
//            }
            break;
        case 7:
            /* PATH(Bit_Pbit,AX); */
            
            /* shall increment the subframe counter for the next occurrence*/
            PBitFrame++;
            PBitFrame = PBitFrame & 3; /* insure a count between 0-3*/
            break;
        default:
            /* PATH(Bit_Pbit,BE); */
            break;
    }

    /* PATH(Bit_Pbit,BG); */
}


/*
    brief Conducts all "standard mode" CBIT Tests, in sequence.

    Purpose:
        Runs all "CBIT" test procedures, for the standard scheduler.  This is a convenience function that 
        allows the user to invoke a single function instead of a repeated list of calls to Bit_CheckMonitor().

    param[in] subframe      Scheduler frame in which a specific monitor will be executed.

    retval true    a fault is active in the system
    retval false   a fault is not active in the system

    Preconditions and Assumptions:
    This routine must only be operated as part of the standard scheduler.

*/
bool Bit_Cbit( Uint16 subframe )
{
    bool status = false;
    static Uint16 CBitFrame = 0;

    /*shall exercise the RVDT Position sense monitors (30-35) after ADC processing is complete, during frame 5.

     shall exercise the FSL monitors (40-44) during one of the 8 scheduler frames.
      Monitors 0x42-0x45 have been implemented as inline monitors
     Do we need to execute these monitor before we sample our FSL?

     shall exercise the 200ms timer monitor (54) during one of the 8 scheduler frames, while "on ground"(Moved to PBIT and IBIT only)

     This monitor has been implemented at an inline monitor.
     shall exercise the Quadrature count check (18) during one of the 8 scheduler frames,
         while at RMS only, and only when commanded from position 0 to position 1.*/

    /* PATH(Bit_Cbit,A); */ 

    /* shall schedule BIT monitors in one eight scheduler frames*/
    switch (subframe)
    {
        case 0:
            /* PATH(Bit_Cbit,B); */

            /* shall exercise the subframe index monitor (76) during frame 0.*/
            Bit_LatchedMonitors(0x76, true);
            /* Run the GFD_I_SENSE Monitor */
            Bit_CriticalMonitors(0x7D, true);
            /* Run the following monitors at 200Hz rather than 1kHz. */
            switch (CBitFrame)
            {
                case 0:
                    break;
                case 1:
                    break;
                case 2:
                    break;
                case 3:
                    Bit_LatchedMonitors(0x21, true);
                    break;
                case 4:
                    Bit_LatchedMonitors(0x22, true);
                    break;
                default:
                    break;
            }

            break;
        case 1:
            /* PATH(Bit_Cbit,J); */

            /* shall schedule BIT monitors in frame 1 using subframes that are executed at 200 Hz instead of 1 kHz*/
            switch (CBitFrame)
            {
                case 0:
                    /* PATH(Bit_Cbit,K); */
                    /* shall exercise the undefined interrupt monitor (77) during frame 0.*/
                    Bit_LatchedMonitors(0x77, true);
                    break;
                case 1:
                    /* PATH(Bit_Cbit,L); */
                    /* shall exercise the stack overrun monitor (79) during frame 0.*/
                    Bit_LatchedMonitors(0x79, true);
                    break;
                case 2:
                    /* PATH(Bit_Cbit,M); */
                    /* shall exercise the Rigging CRC monitor (10) during one of the 8 scheduler frames.*/
                    Bit_CriticalMonitors(0x10, true);
                    break;
                case 3:
                    /* PATH(Bit_Cbit,N); */
                    /* shall exercise the Rig input pins monitor (12) during one of the 8 scheduler frames, except in RIG mode.*/
                    Bit_CriticalMonitors(0x12, true);
                    break;
                case 4:
                    /* PATH(Bit_Cbit,O); */
                    break;
                default:
                    /* PATH(Bit_Cbit,Q); */
                    break;
            }
            break;
        case 2:
            /* PATH(Bit_Cbit,R); */

            /* Run the following monitors at 200Hz rather than 1kHz. */
            switch (CBitFrame)
            {
                case 0:
                    Bit_LatchedMonitors(0x02, true);
                    break;
                case 1:
                    /* Run the Bus Passive Monitor for ARINC825_CNTRL_BUS */
                    Bit_LatchedMonitors(0x4A, true);
                    break;
                case 2:
                    /* Run the Bus Off Monitor for ARINC825_CNTRL_BUS */
                    Bit_LatchedMonitors(0x4C, true);
                    break;
                case 3:
                    /* Run the SYNC Communications Stale Monitor for ARINC825_CNTRL_BUS */
                    Bit_LatchedMonitors(0x4E, true);
                    break;
                case 4:
                    break;
                default:
                    break;
            }
            break;
        case 3:
            /* PATH(Bit_Cbit,AC); */

            /* shall schedule BIT monitors in frame 3 using subframes that are executed at 200 Hz instead of 1 kHz*/
            switch (CBitFrame)
            {
                case 0:
                    /* PATH(Bit_Cbit,AD); */
#if defined(__SKEW_SNSR_RVDT__)
                    Bit_LatchedMonitors(0x30, true);
#endif
                    break;
                case 1:
                    /* PATH(Bit_Cbit,AE); */
#if defined(__SKEW_SNSR_ENCODER__)
                    Bit_CriticalMonitors(0x36, true);
#endif
                    break;
                case 2:
                    /* PATH(Bit_Cbit,AF); */
                    /*Run the monitor to check if DC_BUS_CHA is within the specified limit during High-lift & VC mode*/
                    Bit_LatchedMonitors(0x51, true);
                    break;
                case 3:
                    /* PATH(Bit_Cbit,AG); */
                    /*Run the monitor to check if +28VDC_CHA bus is within the specified limit*/
                    Bit_LatchedMonitors(0x52, true);
                    break;
                case 4:
                    /* PATH(Bit_Cbit,AH); */
                    break;
                default:
                    break;
            }
            break;
        case 4:
            /* PATH(Bit_Cbit,AK); */

		    /* shall schedule BIT monitors in frame 4 using subframes that are executed at 200 Hz instead of 1 kHz*/
		    switch (CBitFrame)
            {
                case 0:
                    /* 0x16-Inhibit monitor - Winding over Temp.*/
                    Bit_InhibitMonitors(0x16, true);
                    /* PATH(Bit_Cbit,AL); */
                    break;
                case 1:
                    /* PATH(Bit_Cbit,AM); */
                    /*Run the monitor to check if +15V_SENSE (+15VISO_CHA) is within the specified limit*/
                    Bit_LatchedMonitors(0x53, true);
                    break;
                case 2:
                    /* PATH(Bit_Cbit,AO); */
                    /*Run the monitor to check if -15V_SENSE (-15VISO_CHA) is within the specified limit*/
                    Bit_LatchedMonitors(0x54, true);
                    break;
                case 3:
                    /* PATH(Bit_Cbit,AQ); */
                    /*Run the monitor to check if 5V_SENSE (+5V_AN_CHA) is within the specified limit*/
                    Bit_LatchedMonitors(0x55, true);
                    break;
                case 4:
                    /*Cross channel CRC and timout monitor*/
                    if (Timer_IsExpired(&Bit_XCommsDelayTimer) == true)
                    {
                        Bit_LatchedMonitors(0x07, true);
                    }
                    /* PATH(Bit_Cbit,AR);*/
                    break;
                default:
                    /* PATH(Bit_Cbit,AT); */
                    break;
            }

            /* PATH(Bit_Cbit,AU); */
            break;
        case 5:
            /* PATH(Bit_Cbit,AV); */

            /* shall schedule BIT monitors in frame 5 using subframes that are executed at 200 Hz instead of 1 kHz*/
            switch (CBitFrame)
            {
                case 0:
                    /* PATH(Bit_Cbit,AW); */
                    Bit_LatchedMonitors(0x34, true);
                    break;
                case 1:
                    /* PATH(Bit_Cbit,AX); */

                    /*Run the monitor to check the health of Inverter Gate Driver*/
                    Bit_LatchedMonitors(0x56, true);
                    break;
                case 2:
                    /* PATH(Bit_Cbit,AY); */
                    /*0x2A - Critical Monitor - Phase to Neutral Short Asymmetric (Motor Winding Failure)*/
                    /*Start the monitor 600ms after the motor starts */
                    if (Timer_IsExpired(&Bit_PhNeuDelayTimer) == true)
                    {
                        Bit_CriticalMonitors(0x2A, true);
                    }
                    break;
                case 3:
                    /* PATH(Bit_Cbit,AZ); */
                    /*0x2B* - Critical Monitor - One Phase Open Circuit Short (Motor Winding Failure)  */
                    /*Start the monitor 200ms after the motor starts*/
                    if (Timer_IsExpired(&Bit_OpCktDelayTimer) == true)
                    {
                        Bit_CriticalMonitors(0x2B, true);
                    }
                    break;
                case 4:
                    /* PATH(Bit_Cbit,BA); */
                    break;
                default:
                    /* PATH(Bit_Cbit,BD); */
                    break;
            }
            break;
        case 6:
            /* PATH(Bit_Cbit,BE); */

            /* shall schedule BIT monitors in frame 6 using subframes that are executed at 200 Hz instead of 1 kHz*/
            switch (CBitFrame)
            {
                case 0:
                    /* PATH(Bit_Cbit,BF); */
                    Bit_CriticalMonitors(0x35, true);
                    break;
                case 1:
                    /* PATH(Bit_Cbit,BG); */

                    break;
                case 2:
                    /* PATH(Bit_Cbit,BL); */

                    /* shall exercise the Cross-channel enable monitor (06) during one of the 8 scheduler frames.*/
                    Bit_LatchedMonitors(0x06, true);
                    break;
                case 3:
                    /* PATH(Bit_Cbit,BM); */
                    break;
                case 4:
                    /* PATH(Bit_Cbit,BN); */
                    break;
                default:
                    /* PATH(Bit_Cbit,BP); */
                    break;
            }
            break;
        case 7:
            /* PATH(Bit_Cbit,BQ); */

            /* shall schedule BIT monitors in frame 7 using subframes that are executed at 200 Hz instead of 1 kHz */
            switch (CBitFrame)
            {
                case 0:
                    /* PATH(Bit_Cbit,BR); */
                    break;
                case 1:
                    /* PATH(Bit_Cbit,BT); */
                    break;
                case 2:
                    /* PATH(Bit_Cbit,BV); */
                    /* Run the SYNC Communications Fault Monitor */
                    Bit_LatchedMonitors(0x40, true);
                    break;
                case 3:
                    /* PATH(Bit_Cbit,BW); */
                    break;
                case 4:
                    /* PATH(Bit_Cbit,BY); */
                    break;
                default:
                    /* PATH(Bit_Cbit,CB); */
                    break;
            }

            /* Manage the 200Hz CBIT sub-frame counter */
            CBitFrame++;

            if (CBitFrame == 5)
            {
                /* PATH(Bit_Cbit,CC); */
                CBitFrame = 0;
            }

            /* PATH(Bit_Cbit,CD); */
            break;
        default:
            /* PATH(Bit_Cbit,CE); */
            break;
    }

    /* shall exercise Monitor 0x74 during CBIT (Frame Overrun)*/
    Bit_LatchedMonitors(0x74, true);

//    if (Inhibits_Stat.bit.MtrPwr22_24V_M == 1)
//    {
//        /* PATH(Bit_Cbit,CF); */
//        Inhibits_Stat.bit.MtrPwr22_24V_M = 0;
//    }

    if ((LatchedFaults_Stat.all > 0) || (CriticalFaults_Stat.all > 0) || (Inhibits_Stat.all > 0))
    {
        /* PATH(Bit_Cbit,CH); */
        status = true;
    }

    /* PATH(Bit_Cbit,CI); */
    return (status);
}


/*
    brief Checks to see if the test cmd is an IBIT test or not

    Purpose:
        This function is a convenience function provided for checking and executing a IBIT test.

    param[in] testID        Test ID number that is being tested

    Global Data Referenced:
    
    retval true    Test ID is a IBIT test.
    retval false   Test ID is not a Valid IBIT test.
    
    Preconditions and Assumptions:
        None.

*/
bool Bit_CheckIBIT(Uint16 testId)
{
    bool result = false;
    Uint16 TestType = 0;

    /* PATH(Bit_CheckIBIT,A); */

    if (testId <= BITTEST_MAX)
    {
        /* PATH(Bit_CheckIBIT,B); */
        TestType = (bitMonitors[testId].type & BIT_IBIT);

        if (TestType == BIT_IBIT)
        {
            /* PATH(Bit_CheckIBIT,C); */
            result = true;

			/* For IBIT tests that are NOT STARTED, zero the fault-data and trip persistence fields */
            if (Ibit_Stat.tstCmplt == NOT_STARTED)
            {
			    /* PATH(Bit_CheckIBIT,E); */
	            bitStatus[testId].data = 0;
	            bitStatus[testId].tripCount = 0;
	            bitStatus[testId].tripTime = 0;
            }
        }
    }

    /* PATH(Bit_CheckIBIT,D); */
    return (result);
}

/*
    brief Operates MCU in predefined manner for test purposes

    Purpose:
        Runs fault-specific test procedures by calling the monitor's "test" handler function.

    param[in] testId       index into Fault Matrix tables
    param[in] testStage    step in procedure sequence to be executed next

    Global Data Referenced:
    
    retval true        Test effect is complete, including all steps in a multipart test
    retval false       Test effect is incomplete and should be invoked again to complete the multipart test effect
    
    Preconditions and Assumptions:
         None.

    remarks
    The  testStage argument will be passed on to the "test" handler function for the given fault monitor.

*/
bool Bit_DoTest( Uint16 testId, Uint16 testStage ) 
{
    bool result = false;

    /* PATH(Bit_DoTest,A);*/

    /* shall invoke test handler if it is defined*/
    if (bitMonitors[testId].test)
    {
        /* PATH(Bit_DoTest,B); */
        result = bitMonitors[testId].test(testStage);
    }

    /* PATH(Bit_DoTest,C); */
    return (result);
}

/*
    brief Checks given fault monitor

    Purpose:
        This function invokes the handler routines that are configured for the given fault monitor.  If 
        the monitor is not currently 'tripped', it will invoke the trip condition handler and possibly
        the trip effect handler.  If the monitor is currently 'tripped', it will invoke the reset
        condition handler.

    param[in] testId       index into Fault Matrix tables

    Global Data Referenced:
    
    retval true    the fault monitor has fully tripped
    retval false   the fault monitor has not (yet) tripped
    
    Preconditions and Assumptions:
        None.

    remarks
        The logic flow for this function is as follows:
    code
    If tripBoot == 0,
        Invoke tripcheck() handler
        If returned true, 
            If tripCount == 0, copy free-running count time to tripTime
            Increment tripCount
        If returned false,
            Clear tripCount and tripTime

        If tripCount >= persistRpt and persistTime == 0,
            status = true (fault trips).

        If (tripTime - [now]) >= persistTime and persistRpt == 0,
            status = true (fault trips).

        If persistRpt != 0 and persistTime != 0 and tripCount >= persistRpt and ([now] - tripTime) >= persistTime,
            status = true (fault trips).

        If status == true (fault has tripped),
            Set tripBoot = current BootCount
                Log occurrence of Fault to NVM
                Invoke effect() handler
                Report 'fault data' status to GSE
                Clear tripTime and tripCount

    Else If tripBoot is nonzero,
        Invoke resetcheck() handler
        If returned true,
            Clear tripBoot
            status = false (fault condition has cleared, or rest is valid)
        If returned false,
            Invoke effect() handler
            status = true (fault condition persists, or reset is not valid)

*/
bool Bit_CheckMonitor( Uint16 testId )  
{
    bool status = false;
    bool trip = false;
    
    /* shall match table index with given fault monitor ID*/
    int16 idx = testId;

    /* PATH(Bit_CheckMonitor,A); */

    /* shall skip monitoring if given ID is not a valid monitor in the fault monitor table*/
	if ((bitMonitors[idx].type != NULL)  && (testId <= BIT_MAXMONITORS))
    {
        /* PATH(Bit_CheckMonitor,B); */

        /* shall monitor the fault using the pseudocode given above*/
        if (bitStatus[idx].tripBoot == 0)
        {
            /* PATH(Bit_CheckMonitor,C); */

            /* check for and log fault condition*/
            trip = bitMonitors[idx].tripcheck(&bitStatus[idx]);
            
            if (trip == true)
            {
                /* PATH(Bit_CheckMonitor,D); */

                if (bitStatus[idx].tripCount++ == 0)
                {
                    /* PATH(Bit_CheckMonitor,E); */
                    bitStatus[idx].tripTime = OneMsTimer;
                }
            }
            else
            {
                /* PATH(Bit_CheckMonitor,F); */
                bitStatus[idx].tripCount = 0;
            }
         
            /*check against persistence standards*/
            if ((bitStatus[idx].tripCount >= bitMonitors[idx].persistRpt) && (bitMonitors[idx].persistTime == 0))
            {
                /* PATH(Bit_CheckMonitor,H); */
                status = true;
            }
            else if ((bitMonitors[idx].persistRpt != 0) && (bitMonitors[idx].persistTime != 0) && 
                    ((OneMsTimer - bitStatus[idx].tripTime) >= bitMonitors[idx].persistTime) &&
                     (bitStatus[idx].tripCount >= bitMonitors[idx].persistRpt))
            {
                /* PATH(Bit_CheckMonitor,J); */
                status = true;
            }

            /* fault has tripped */
            if (status == true)
            {
                /* PATH(Bit_CheckMonitor,K); */
                bitStatus[idx].tripBoot = Nvm_State.bootcount;

                /* PATH(Bit_CheckMonitor,L); */

                if (bitMonitors[idx].effect != NULL)
                {
                    /* PATH(Bit_CheckMonitor,M); */
                    bitMonitors[idx].effect(idx);
                }

                Bit_LogToNvm(idx);
            	LatestFaultCode = idx;
            	LatestFaultData = bitStatus[idx].data;
            	Bit_ReportToGse(idx);
            	bitStatus[idx].tripCount = 0;
            }
		}   		        
  		else
  		{
			/* PATH(Bit_CheckMonitor,N); */
			status = true;

			/* check for reset condition*/
			if (bitMonitors[idx].resetcheck != NULL)
			{
				/* PATH(Bit_CheckMonitor,O); */

				if (bitMonitors[idx].resetcheck(&bitStatus[idx]))
				{
					/* PATH(Bit_CheckMonitor,P); */
					bitStatus[idx].tripBoot = 0;
					status = false;
					bitStatus[idx].tripTime = OneMsTimer;

					if (LatestFaultCode == idx)
					{
						/* PATH(Bit_CheckMonitor,Q); */
						LatestFaultCode = 0;
						LatestFaultData = 0;
					}
				}
				else if (bitMonitors[idx].effect != NULL)
				{
					/* PATH(Bit_CheckMonitor,S); */
					bitMonitors[idx].effect(idx);
				}
			}
	    }
	}
        
    /* PATH(Bit_CheckMonitor,T); */

    /* shall return "tripped" status of the given fault monitor*/
    return (status);
}


/*      Local Function Definitions
*/


/*
    brief Stores given Fault status information to NVM, assuming that the fault has tripped.

    param[in] stat     pointer to Bit_t status information structure

    retval true     Successfully wrote fault data to EEPROM's page buffer.
    retval false    Fault data not written; EEPROM was busy in a write cycle.

*/
bool Bit_LogToNvm( Uint16 index )
{
    Bit_t *stat = &bitStatus[index];
    Nvm_State_t stateTemp;
    bool retVal = false;

    /* PATH(Bit_LogToNvm,A); */

    /* The runtime counter is updated automatically every 10 minutes for this data structure */
    /* shall capture the necessary information for logging into NVM */
    /* Nvm_State.bootcount = stat->tripBoot; */

    if (index != 0x00)
    {
        /* PATH(Bit_LogToNvm,B); */
        Nvm_State.faultId = index;
        Nvm_State.faultData = stat->data;
    }
    else
    {
        /* PATH(Bit_LogToNvm,C); */
        Nvm_State.faultId = 0x00;
        Nvm_State.faultData = 0x00;
    }

    Nvm_State.Mode = McuGetState();
    Nvm_State.timecount = Timer_OneMinTimer;
    Nvm_State.quadposition.onside = tHall.Position;
#if defined(__HALLX_CONFIGURED)
    Nvm_State.quadposition.xside = tHallx.Position;
#endif

    /* Save off the current status of the MCU */
    Nvm_State.tMcuStatus.all = tMcuStatus.all;
    Nvm_State.tMcuFault.all = tMcuFault.all;
    Nvm_State.tMcuMaint.all = tMcuMaint.all;
    Nvm_State.tPanel.tSf = G_ASW_DATA.tPanelOutputData;
    Nvm_State.tPanel.tSkewSnsrCalcs = tSkewSnsrCalcs;

    stateTemp = Nvm_State;

    if (XSideFaultNvmFlg == true)
    {
        /* PATH(Bit_LogToNvm,D); */

        if (G_bChannelA == true)
        {
            /* PATH(Bit_LogToNvm,E); */
            stateTemp.Mode |= 0x4000;
        }
        else
        {
            /* PATH(Bit_LogToNvm,F); */
            stateTemp.Mode |= 0x8000;
        }

        XSideFaultNvmFlg = false;
    }
    else
    {
        /* PATH(Bit_LogToNvm,G); */

        if (G_bChannelA == true)
        {
            /* PATH(Bit_LogToNvm,H); */
            stateTemp.Mode |= 0x8000;
        }
        else
        {
            /* PATH(Bit_LogToNvm,I); */
            stateTemp.Mode |= 0x4000;
        }
    }

    retVal = Nvm_WriteState(&stateTemp);

    /* PATH(Bit_LogToNvm,J); */
    return (retVal);
}


/*
    brief Reports given Fault status information to GSE, if the fault has tripped.

    param[in] index     index of BIT monitor to be reported

    Global Data Referenced:
        #LatestFaultCode 
        #LatestFaultData 


 */
void Bit_ReportToGse(Uint16 index)
{
    CRITICALITY systemFaultCriticality, latestFaultCriticality;

    /* PATH(Bit_ReportToGse,A); */

    systemFaultCriticality = GetSystemCriticality();
    latestFaultCriticality = bitMonitors[index].criticality;

    if (systemFaultCriticality != CRITICAL)
    {
        /* PATH(Bit_ReportToGse,B); */

        if (systemFaultCriticality < latestFaultCriticality)
        {
            /* PATH(Bit_ReportToGse,C); */
            SystemFaultCode = index;
            SystemFaultData = bitStatus[index].data;
            SetSystemCriticality(latestFaultCriticality);
            systemFaultCriticality = latestFaultCriticality;
        }

        if ((latestFaultCriticality == INHIBIT) && (systemFaultCriticality != LATCHED))
        {
            /* PATH(Bit_ReportToGse,D); */
            Inhibit_QueueAdd(index);
        }
    }

    /* PATH(Bit_ReportToGse,E); */
}


void SetSystemCriticality(CRITICALITY set)
{
    /* PATH(SetSystemCriticality,A); */
    SystemCriticality = set;

    /* PATH(SetSystemCriticality,B); */
}


CRITICALITY GetSystemCriticality(void)
{
    /* PATH(GetSystemCriticality,A); */

    return (SystemCriticality);
}


/*
    brief Determines whether there are latched faults outstanding, used during power-up.

    Purpose:
        Retrieves fault status and returns whether or not a fault is still latched.  If
        a latched fault has passed its "reset" condition, then the fault status is updated
        in the NVM record.

     Global Data Referenced:
        #Nvm_State 
        #McuBootCount 
        #LatestFaultCode 
        #LatestFaultData 
    
    retval true     The latched fault is still active--has not been cleared
    retval false    The latched fault has been cleared.

    
*/
bool FaultCheck( void )
{
    bool reset = false;
    Uint16 index, endIndex;
  
    /* PATH(FaultCheck,A); */

    index = Nvm_GetCurrentStatePointer();
    endIndex = index;
    Nvm_GetState(index, &Nvm_State_Temp);

    /* Need to make sure the fault is a valid fault ID before trying to access the reset function,*/
    /* otherwise an Illegal interrupt will be asserted.*/
    /* shall determine if the system has a latched fault*/
    if (Nvm_State_Temp.bootcount != 0) /* If the bootcount is ever Zero, then the system has not been rigged for the first time.*/
    {
        /* PATH(FaultCheck,B); */

        /* Seek backward to the first NVM State record in the previous boot cycle */
        do
        {
          /* PATH(FaultCheck,C); */

          /* wrap to the end of the buffer if just sent first record, else decrement index*/
          if (index == 0)
          {
              /* PATH(FaultCheck,D); */
              index = MAX_STATE_RECORDS;
          }
          index--;

          Nvm_GetState(index, &Nvm_State_Temp);
        } while ((Nvm_State_Temp.bootcount == u16McuBootCount) && ((Nvm_State_Temp.Mode & 0x00FF) != TEST_MODE) && (index != endIndex)); // should this check for power down too?????

        /* Increment back to the first NVM State record in this boot cycle */
        index++;
        if (index >= MAX_STATE_RECORDS)
        {
          /* PATH(FaultCheck,M); */
          index = 0;
        }

        /* read the first record of this boot cycle */
        Nvm_GetState(index, &Nvm_State_Temp);

        /* read up through the current record by setting endindex to one past current */
        endIndex++;
        if (endIndex >= MAX_STATE_RECORDS)
        {
          /* PATH(FaultCheck,S); */
          endIndex = 0;
        }

        while ((index != endIndex) && ((Nvm_State_Temp.Mode & 0x00FF) != TEST_MODE) )
        {
          /* PATH(FaultCheck,F); */

		  if(Nvm_State_Temp.faultId != 0x04)
		  {
			  /* PATH(FaultCheck,T); */
	          if ((Nvm_State_Temp.faultId != 0) && (Nvm_State_Temp.faultId < 0xFF))
	          {
	              /* PATH(FaultCheck,G); */
	              reset = FaultResetCheck();
	          }
		  }
		  else
		  {
			  /* PATH(FaultCheck,U); */
			  reset = Bit_JamCheck();
		  }

          if (reset == true)
          {
            /* PATH(FaultCheck,H); */

            /* Non-Compliance: The following statement is the fourth nesting level.
               Justification: The algorithm is clear enough that this amount of nesting is not excessively complex.
            */
            switch (bitMonitors[Nvm_State_Temp.faultId].criticality)
            {
              case CRITICAL:
                  /* PATH(FaultCheck,I); */
                  Bit_CriticalMonitors(Nvm_State_Temp.faultId, false);
                  break;
              case LATCHED:
                  /* PATH(FaultCheck,J); */
                  Bit_LatchedMonitors(Nvm_State_Temp.faultId, false);
                  break;
              case INHIBIT:
                  /* PATH(FaultCheck,K); */
                  Bit_InhibitMonitors(Nvm_State_Temp.faultId, false);
                  break;
              default:
                  /* PATH(FaultCheck,L); */
                  break;
            }

            Bit_ReportToGse(Nvm_State_Temp.faultId);
          }

          /* get next record */
          index++;
          if (index >= MAX_STATE_RECORDS)
          {
            /* PATH(FaultCheck,N); */
            index = 0;
          }
          Nvm_GetState(index, &Nvm_State_Temp);

        }
    }

	/* If the data element of the bitStatus data sturcture is not equal to 0, but less than 3,
	the the occurance counter reset condition has not been met, and this fault record needs to
	be re-written to NVM so it can be checked in the next powerCycle.
	*/
	if((bitStatus[0x04].data != 0) && ((bitStatus[0x04].data & 0x0F) < 3))
	{
		/* PATH(FaultCheck,V); */
		Bit_LogToNvm(0x04);
	}

    if ((Inhibits_Stat.all > 0) || (LatchedFaults_Stat.all > 0) || (CriticalFaults_Stat.all > 0))
    {
        /* PATH(FaultCheck,P); */
        reset = true;
    }
    else
    {
        /* PATH(FaultCheck,Q); */
        reset = false;
    }

    /* PATH(FaultCheck,R); */
    return (reset); 
}


bool FaultResetCheck(void)
{
    bool reset = false;

    /* PATH(FaultResetCheck,A); */
	
	if(bitStatus[Nvm_State_Temp.faultId].tripBoot == 0)
	{
		/* PATH(FaultResetCheck,H); */

	    /* shall set the necessary fault information when an latched fault exists*/
	    bitStatus[Nvm_State_Temp.faultId].tripBoot = Nvm_State_Temp.bootcount;
	    bitStatus[Nvm_State_Temp.faultId].tripTime = Nvm_State_Temp.timecount;
	    bitStatus[Nvm_State_Temp.faultId].data = Nvm_State_Temp.faultData;

		if (Nvm_State_Temp.faultId == 0x04)
		{
			Uint16 highnibble = Nvm_State_Temp.faultData & 0xF0;
		    /* PATH(FaultResetCheck,E); */

			if ((highnibble == 0x10 && G_bChannelA == true)
				|| (highnibble == 0x00 && G_bChannelA == false))
			{
			    /* PATH(FaultResetCheck,F); */
				xSideJamFlg = true;
			}
		}

		if (Nvm_State_Temp.faultId == 0x59)
		{
			/* PATH(FaultResetCheck,G); */
			PowerdownFault = true;
		}

		if (InhibitCheck(Nvm_State_Temp.faultId, Nvm_State_Temp.faultData) == false)
		{
			/* PATH(FaultResetCheck,B); */
			/* shall check if the reset condition is valid*/
	    	reset = Bit_CheckMonitor(Nvm_State_Temp.faultId);
		}

		if (reset == false)
		{
			/* PATH(FaultResetCheck,C); */
			/* Clear the data in the Status structure */
	        bitStatus[Nvm_State_Temp.faultId].data = 0;
	        bitStatus[Nvm_State_Temp.faultId].tripBoot = 0;
	        bitStatus[Nvm_State_Temp.faultId].tripCount = 0;
	        bitStatus[Nvm_State_Temp.faultId].tripTime = 0;
		}
	}

    /* PATH(FaultResetCheck,D); */
    return (reset);
}


/*
    brief Determines whether a given fault code should result in an "inhibit".
 
    Purpose:
        Determines whether a given fault code should result in an "inhibit", instead of
        a latched fault.

    param[in] FaultId      index into Fault Matrix tables
    param[in] FaultData    fault result data used for some comparisons

    Global Data Referenced:
        None.

    retval true        The given fault should result in "inhibit".
    retval false       The given fault should not result in "inhibit".

    
*/
bool InhibitCheck(Uint16 FaultId, Uint16 FaultData)
{
    bool inhibit = false;

    /* PATH(InhibitCheck,A); */

    /* shall return true if the fault is an inhibit, otherwise return false */
    
    /* Non-Compliance Justification: A break statement is not needed because the same
       functionality is required for all of the following cases.
    */       
    switch (FaultId)
    {
        case 0x00:
            /* PATH(InhibitCheck,B); */
        case 0x49:
        case 0x16:
            inhibit = true;
            break;
        case 0x04:
            /* PATH(InhibitCheck,R); */
            /* PATH(InhibitCheck,C); */

            if ((FaultData & 0x0F) < 3)
            {
                /* PATH(InhibitCheck,S); */
                inhibit = true;
            }
            break;
        default:
            /* PATH(InhibitCheck,T); */
            break;
    }

    /* PATH(InhibitCheck,U); */    
    return (inhibit);
}

void Bit_InhibitMonitors(Uint16 testId, bool runTest)
{
    Uint16 result;
    bool writeToNVM = true;

    /* PATH(Bit_InhibitMonitors,A); */
    
    if (runTest == true)
    {
        /* PATH(Bit_InhibitMonitors,B); */
        result = Bit_CheckMonitor(testId);
    }
    else
    {
        /* PATH(Bit_InhibitMonitors,C); */
        result = 1;
    }

    switch (testId)
    {
        case 0x04:
            /* PATH(Bit_InhibitMonitors,D); */
            Inhibits_Stat.bit.FlapJam_M = result;
            break;
        case 0x16:
            Inhibits_Stat.bit.MotorPhaseOverTemp_M = result;
            break;
#if !defined(TRL4_NO_PDI_RIG)
        case 0x49:
            Inhibits_Stat.bit.bA825MaintBusFlapCmdSkew = result;
            break;
        case 0x4F:
            Inhibits_Stat.bit.bA825MaintBusCommsStale = result;
            break;
#endif
        default:
            /* PATH(Bit_InhibitMonitors,U); */
            writeToNVM = false;
            break;
    }

    if ((runTest == false) && (writeToNVM == true))
    {
        /* PATH(Bit_InhibitMonitors,V); */
        Bit_LogToNvm(testId);
    }

    /* PATH(Bit_InhibitMonitors,W); */
}


void Bit_LatchedMonitors( Uint16 testId, bool runTest )
{
    Uint16 result;
    bool writeToNVM = true;

    /* PATH(Bit_LatchedMonitors,A); */

    if (runTest == true)
    {
        /* PATH(Bit_LatchedMonitors,B); */
        result = Bit_CheckMonitor(testId);
    }
    else
    {
        /* PATH(Bit_LatchedMonitors,C); */
        result = 1;
    }
    
    switch (testId)
    {
        case 0x02:
            LatchedWarnings_Stat.bit.bMcuDisabled = result;
            break;
        case 0x06:
            /* PATH(Bit_LatchedMonitors,D); */
            LatchedWarnings_Stat.bit.xChannelEnable_M = result;
            break;
        case 0x07:
            LatchedWarnings_Stat.bit.xChanneCRC_M = result;
            break;
        case 0x21:
            /* PATH(Bit_LatchedMonitors,F); */
            LatchedFaults_Stat.bit.HallSequence_M = result;
            break;
        case 0x22:
            /* PATH(Bit_LatchedMonitors,AA); */
            LatchedFaults_Stat.bit.InvalidHall_M = result;
            break;                        
        case 0x25:
            /* PATH(Bit_LatchedMonitors,G); */
            LatchedFaults_Stat.bit.BrakeSwitch_M = result;
            break;
        case 0x30:
            /* PATH(Bit_LatchedMonitors,H); */
#if defined(__SKEW_SNSR_RVDT__)
            LatchedFaults_Stat.bit.RvdtExcitationFault = result;
#endif
            break;
        case 0x34:
            LatchedWarnings_Stat.bit.bRvdtPositionFault_M = result;
            break;
        case 0x4A:
            LatchedFaults_Stat.bit.bA825CntrlBusPassive = result;
            break;
        case 0x4B:
            LatchedFaults_Stat.bit.bA825MaintBusPassive = result;
            break;
        case 0x4C:
            LatchedFaults_Stat.bit.bA825CntrlBusOff = result;
            break;
        case 0x4D:
            LatchedFaults_Stat.bit.bA825MaintBusOff = result;
            break;
        case 0x4E:
            LatchedFaults_Stat.bit.bA825CntrlBusCommsStale = result;
            break;
        case 0x50:
            /* PATH(Bit_LatchedMonitors,L); */
            LatchedFaults_Stat.bit.BridgeMonitor_M = result;
            break;
        case 0x51:
            /* PATH(Bit_LatchedMonitors,L); */
            LatchedFaults_Stat.bit.DcBusCha_M = result;
            break;
        case 0x52:
            /* PATH(Bit_LatchedMonitors,L); */
            LatchedFaults_Stat.bit.p28vdcCha_M = result;
            break;
        case 0x53:
            /* PATH(Bit_LatchedMonitors,L); */
            LatchedWarnings_Stat.bit.p15vSense_M = result;
            break;
        case 0x54:
            /* PATH(Bit_LatchedMonitors,L); */
            LatchedWarnings_Stat.bit.n15vSense_M = result;
            break;
        case 0x55:
            /* PATH(Bit_LatchedMonitors,L); */
            LatchedFaults_Stat.bit.p5vSense_M = result;
            break;
        case 0x56:
            /* PATH(Bit_LatchedMonitors,N); */
            LatchedFaults_Stat.bit.InvGateDriver_M = result;
            break;
        case 0x57:
            /* PATH(Bit_LatchedMonitors,O); */
            LatchedFaults_Stat.bit.ProgramPin_M = result;
            break;
        case 0x71:
            /* PATH(Bit_LatchedMonitors,R); */
            LatchedFaults_Stat.bit.RamCheck_M = result;
            break;
        case 0x73:
            /* PATH(Bit_LatchedMonitors,S); */
            LatchedFaults_Stat.bit.WDPowerUp_M = result;
            break;
        case 0x74:
            /* PATH(Bit_LatchedMonitors,T); */
            LatchedFaults_Stat.bit.FrameOverrun_M = result;
            break;
		case 0x76:
			/* PATH(Bit_LatchedMonitors,AB); */
			LatchedFaults_Stat.bit.SubFrameIndex_M = result;
			break;
        case 0x77:
            /* PATH(Bit_LatchedMonitors,U); */
            LatchedFaults_Stat.bit.UndefinedInt_M = result;
            break;
        case 0x79:
            /* PATH(Bit_LatchedMonitors,W); */
            LatchedFaults_Stat.bit.StackOverrun_M = result;
            break;
        default:
            writeToNVM = false;
            /* PATH(Bit_LatchedMonitors,X); */
            break;
    }

    if ((runTest == false) && (writeToNVM == true))
    {
        /* PATH(Bit_LatchedMonitors,Y); */
        Bit_LogToNvm(testId);
    }
    
    /* PATH(Bit_LatchedMonitors,Z); */
}


void Bit_CriticalMonitors( Uint16 testId, bool runTest )
{
    Uint16 result;
    bool writeToNVM = true;

    /* PATH(Bit_CriticalMonitors,A); */
    
    if (runTest == true)
    {
        /* PATH(Bit_CriticalMonitors,B); */
        result = Bit_CheckMonitor(testId);
    }
    else
    {
        /* PATH(Bit_CriticalMonitors,C); */ 
        result = 1;
    }

    switch (testId)
    {
#if !defined(TRL4_NO_PDI_RIG)
        case 0x10:
            /* PATH(Bit_CriticalMonitors,J); */
            CriticalFaults_Stat.bit.RigValue_M = result;
            break;
        case 0x12:
            /* PATH(Bit_CriticalMonitors,K); */
            CriticalFaults_Stat.bit.RigModeFault_M = result;
            break;
        case 0x1A:
            CriticalFaults_Stat.bit.SfBiasCheck_M = result;
            break;
        case 0x1B:
            CriticalFaults_Stat.bit.SfMeasResidualCheck_M = result;
            break;
#endif
        case 0x23:
            /* PATH(Bit_CriticalMonitors,P); */
            CriticalFaults_Stat.bit.BrakeHold_M = result;
            break;
        case 0x2A:
            CriticalFaults_Stat.bit.PhaseNeutralShortAsym_M = result;
            break;
        case 0x2B:
            CriticalFaults_Stat.bit.OnePhaseOpenCkt_M = result;
            break;
        case 0x36:
#if defined(__SKEW_SNSR_ENCODER__)
            CriticalFaults_Stat.bit.EncoderHealthFault_M = result;
#endif
            break;
#if !defined(TRL4_NO_PDI_RIG)
        case 0x64:
            /* PATH(Bit_CriticalMonitors,U); */
            CriticalFaults_Stat.bit.RiggingCheck_M = result;
            break;
#endif
        case 0x70:
            /* PATH(Bit_CriticalMonitors,W); */
            CriticalFaults_Stat.bit.ROMcrcCheck_M = result;
            break;
#if !defined(TRL4_NO_PDI_RIG)
        case 0x7A:
            CriticalFaults_Stat.bit.PDI_CRC_Check_M = result;
            break;
        case 0x7B:
            CriticalFaults_Stat.bit.PDI_OOR_Check_M = result;
            break;
#endif
        case 0x7C:
            CriticalFaults_Stat.bit.MramCheck_M = result;
            break;
        case 0x7D:
            CriticalFaults_Stat.bit.GfdSense_M = result;
            break;
        default:
            writeToNVM = false;
            /* PATH(Bit_CriticalMonitors,X); */
            break;
    }

    if ((runTest == false) && (writeToNVM == true))
    {
        /* PATH(Bit_CriticalMonitors,Y); */
        Bit_LogToNvm(testId);
    }

    /* PATH(Bit_CriticalMonitors,Z); */
}


bool Bit_JamCheck(void)
{
    bool retVal = false;

    /* PATH(Bit_JamCheck,A); */
	if(McuGetState() == BOOT_MODE)
	{
		/* PATH(Bit_JamCheck,D); */

		/*If Flight cycle counts are eqaul, then the jam occured in the most recent flight cycle,
		 and the counter should not be reset. If the fligh cycle counts are not equal, then a
		 flight cycle has occured since the last jam, and by default, the occurence counter will
		 be reset back to 0. If the # of jams is greater than 3, then invoke the normal reset checking
		 algorithm.
		*/
		//if((Nvm_State_Temp.FlightCycleCount == Nvm_State.FlightCycleCount) && ((WOW1_RAW == 0) && (WOW2_RAW == 0))
#if 0
	    /*Commenting out as WOW_N_RAW signal is obsolete in MCU SW,
	     * Need to re-check this part of code during WOW clean-up
	     */
	    /*if((Nvm_State_Temp.FlightCycleCount == Nvm_State.FlightCycleCount) && (WOW_N_RAW == 1)*/
	    if((Nvm_State_Temp.FlightCycleCount == Nvm_State.FlightCycleCount)
		    && ((Nvm_State_Temp.faultData & 0x0F) < 3))
		{
			/* PATH(Bit_JamCheck,E); */
    			bitStatus[0x04].data = Nvm_State_Temp.faultData;
		}
		else if((Nvm_State_Temp.faultData & 0x0F) >= 3)
		{
			/* PATH(Bit_JamCheck,F); */
			retVal = FaultResetCheck();
		}
#endif

	}
	else if (((bitStatus[0x04].data & 0x0F) >= 3) && (bitStatus[0x04].tripBoot != 0))
	{
   		/* PATH(Bit_JamCheck,B); */
    	retVal = true;
	}


    /* PATH(Bit_JamCheck,C); */
    return (retVal);
}


void Bit_Run_CBit(Uint16 subframe)
{
    /* PATH(Bit_Run_CBit,A); */

    switch (McuGetState())
    {
        /* Non-Compliance: The following case statement lacks a break statement.
           Justification: The same functionality is required for both of the following cases.
        */
        case IDLE_MODE:
            /* PATH(Bit_Run_CBit,B); */
        case RUN_MODE:
            /* PATH(Bit_Run_CBit,C); */

                OnSideFault = Bit_Cbit(subframe);

                if (OnSideFault == true)
                {
                    Events_PostEvent(EVENT_FAULT_DETECTED, 0);
                }

            break;
        case FAS_FAIL:
            /* PATH(Bit_Run_CBit,G); */
            OnSideFault = Bit_Cbit(subframe);

            if ((OnSideFault == false) /*&& (XSideSpiFault == false)*/)
            {
                /* PATH(Bit_Run_CBit,H); */
                Events_PostEvent(EVENT_FAULT_CLEARED, 0);
            }
            break;
        default:
            /* PATH(Bit_Run_CBit,L); */
            break;
    }

    /* PATH(Bit_Run_CBit,M); */
}


void Bit_FaultReporting(void)
{
    /* PATH(Bit_FaultReporting,A);*/

    switch (GetSystemCriticality())
    {
        /* Non-Compliance: The following case statement lacks a break statement.
           Justification: The same functionality is required for both of the following cases.
        */
        case CRITICAL:
            /* PATH(Bit_FaultReporting,B); */
        case LATCHED:
            /* PATH(Bit_FaultReporting,C); */
            ReportedFaultCode = SystemFaultCode;
            ReportedFaultData = SystemFaultData;
            break;
        case INHIBIT:
            /* PATH(Bit_FaultReporting,D); */

            if ((Inhibit_QueueEmpty() == false) && (bitStatus[Inhibit_QueuePeek()].tripBoot == 0))
            {
                /* PATH(Bit_FaultReporting,E); */
                Inhibit_QueueRemove();
            }

            if (Inhibit_QueueEmpty() == false)
            {
                /* PATH(Bit_FaultReporting,F); */
                ReportedFaultCode = Inhibit_QueuePeek();
                ReportedFaultData = bitStatus[ReportedFaultCode].data;
                SystemFaultCode = ReportedFaultCode;
                SystemFaultData = ReportedFaultData;
            }
            else
            {
                /* PATH(Bit_FaultReporting,G); */
                SetSystemCriticality(NO_FAULT);
                SystemFaultCode = 0;
                SystemFaultData = 0;
                ReportedFaultCode = 0;
                ReportedFaultData = 0;
                LatestFaultCode = 0;
                LatestFaultData = 0;
            }
            break;
        case NO_FAULT:
            /* PATH(Bit_FaultReporting,H); */
            ReportedFaultCode = 0;
            ReportedFaultData = 0;
            SystemFaultCode = 0;
            SystemFaultData = 0;
            LatestFaultCode = 0;
            LatestFaultData = 0;
            break;
        default:
            /* PATH(Bit_FaultReporting,I); */
            break;
    }

    /* PATH(Bit_FaultReporting,L); */
}


void Inhibit_QueueAdd(Uint16 id)
{
    Uint16 addHere;

    /* PATH(Inhibit_QueueAdd,A); */

    if ((InhibitSize < INHIBIT_QUEUE_SIZE) && (id > 0))
    {
        /* PATH(Inhibit_QueueAdd,B); */
        addHere = InhibitIndex + InhibitSize;

        if (addHere >= INHIBIT_QUEUE_SIZE)
        {
            /* PATH(Inhibit_QueueAdd,C); */
            addHere -= INHIBIT_QUEUE_SIZE;
        }

        InhibitQueue[addHere] = id;
        InhibitSize++;
    }

    /* PATH(Inhibit_QueueAdd,D); */
}


Uint16 Inhibit_QueueRemove(void)
{
    Uint16 ret = 0;

    /* PATH(Inhibit_QueueRemove,A); */

    if (InhibitSize > 0)
    {
        /* PATH(Inhibit_QueueRemove,B); */
        ret = InhibitQueue[InhibitIndex];
        InhibitQueue[InhibitIndex] = 0;

        if (InhibitIndex < (INHIBIT_QUEUE_SIZE - 1))
        {
            /* PATH(Inhibit_QueueRemove,C); */
            InhibitIndex++;
        }
        else
        {
            /* PATH(Inhibit_QueueRemove,D); */
            InhibitIndex = 0;
        }

        InhibitSize--;
    }

    /* PATH(Inhibit_QueueRemove,E); */
    return (ret);
}


Uint16 Inhibit_QueuePeek(void)
{
    Uint16 retVal;

    /* PATH(Inhibit_QueuePeek,A); */

    if (InhibitSize > 0)
    {
        /* PATH(Inhibit_QueuePeek,B); */
        retVal = InhibitQueue[InhibitIndex];
    }
    else
    {
        /* PATH(Inhibit_QueuePeek,C); */
        retVal = 0;
    }

    /* PATH(Inhibit_QueuePeek,D); */
    return (retVal);
}


bool Inhibit_QueueEmpty(void)
{
    bool empty;

    /* PATH(Inhibit_QueueEmpty,A); */

    if (InhibitSize == 0)
    {
        /* PATH(Inhibit_QueueEmpty,B); */
        empty = true;
    }
    else
    {
        /* PATH(Inhibit_QueueEmpty,C); */
        empty = false;
    }

    /* PATH(Inhibit_QueueEmpty,D); */

    return (empty);
}

void Bit_StartDog(void)
{
    volatile uint16_t u16_SCSR = 0U;

    /* PATH(Bit_StartDog,B); */
    DINT;
    EALLOW;
    u16_SCSR = WdRegs.SCSR.all;             /* Read current state of SCSR */
    u16_SCSR = u16_SCSR & WDENINT_MASK;     /* Mask off the Watchdog Interrupt Enable bit */
    u16_SCSR = u16_SCSR | ENABLE_WDINT;     /* Set WD RESET to DISABLED, WD INT is ENABLED */
    WdRegs.SCSR.all = u16_SCSR & 0xFFFEU;   /* Write to WD SCSR register as uint16 with exception of WDOVERRIDE*/
    PieVectTable.WAKE_INT = &WDINT_ISR;

    /* shall enable WD in the PIE: Group 1 interrupt 8 */
    PieCtrlRegs.PIEIER1.bit.INTx8 = 1;

    /* shall enable WD INT1 which is connected to WD-Timer through PIE Group 1 */
    IER |= M_INT1;
    EINT;

    ServiceDog();
    EDIS;
    EnableDog();
    WDTestStatus.startTime = FrameTimer;
    WDTestStatus.endTime = 0;
    WDTestStatus.PBitServiced = false;

    /* PATH(Bit_StartDog,E); */
}


/* INT1.8 */
__interrupt void WDINT_ISR(void)    /* WD */
{
    /* PATH(WDINT_ISR,A); */
    /* To receive more interrupts from this PIE group, acknowledge this interrupt */
    /* PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; */

    WDTestStatus.endTime = FrameTimer;
    WDTestStatus.PBitServiced = true;
    /* Bug with TI where writing to WD registers while WD is disabled caused immediate reset.
     * fix is to leave WD enabled.      */
    //DisableDog();
    PieCtrlRegs.PIEIER1.bit.INTx8 = 0;      /* Turn off interrupt for the WD */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; /* Acknowledge Pie group1 Interrupt */
    
    /* PATH(WDINT_ISR,C); */
}


void Bit_InitializeStructures(void)
{
    /* PATH(Bit_InitializeStructures,A); */

    LatchedFaults_Stat.all = 0;
    CriticalFaults_Stat.all = 0;
    Inhibits_Stat.all = 0;

    /* PATH(Bit_InitializeStructures,B); */
}


void Bit_HandleLatchedFaults(void)
{
    bool faultExists = false;

    /* PATH(Bit_HandleLatchedFaults,A); */

    u16McuBootCount = Nvm_State.bootcount;
    Nvm_State.bootcount++;

    /* The reset conditions use the output of the coldboot function */
    /* to determine if the latched fault should be reset or not.*/
    /* Check to see if there are latched faults, and if the reset condition is true */
    faultExists = FaultCheck(); /* This function will return a true if there is a latched fault, and the reset condition */
                                /* is not valid, it will return false if there is a latched fault that has been reset */
                                /* or if there is no latched fault.*/
    main_BootHandler(faultExists);

    /* PATH(Bit_HandleLatchedFaults,B); */
}


/* end bit.c */

