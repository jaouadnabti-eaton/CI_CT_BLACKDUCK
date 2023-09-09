/****************************************************************************************************
*  File name: mcu.c
*
*  Purpose: state machine for the system.
*  This file implements a hierarchical state machine for the MCU.
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
#include "mcu.h"
#include "gpio.h"
#include "rvdt.h"
#include "timer.h"
//#include "scimessage.h"
#include "actuation.h"
#include "bit.h"
#include "nvm.h"
#include "motor.h"
#include "bit.h"
#include "bitmonitors.h"
#include "panel.h"
#include "adc.h"
#include "spi.h"
#include "ASW_BSW_I.h"

/*    #include "test.h" */
/*    extern void TEST_PRINT_STATE_MSG(char * fnName, Evt const * event); */

/*      Local Type Definitions
*/

/*      Local Defines

#define YES     0x01
#define NO      0x00

#define CAL_ENTRY_LIMIT       125*/

/*
 * DC BUS Switching Outputs defined as per HW interface
 */

#define BUS_SW_ENABLE_set()      GpioDataRegs.GPESET.bit.GPIO133 = 1  /* GPIO133 */
#define V28_BUS_CNTL_CHA_set()   GpioDataRegs.GPDSET.bit.GPIO99 = 1   /* GPIO 99 */
#define INRUSH_CTR_CHA_set()     GpioDataRegs.GPBSET.bit.GPIO59 = 1
#define VC_MODE_CHA_set()        GpioDataRegs.GPASET.bit.GPIO28 = 1
#define V270_BUS_CNTL_CHA_set()  GpioDataRegs.GPASET.bit.GPIO23 = 1

#define VC_MODE_CHA_clr()        GpioDataRegs.GPACLEAR.bit.GPIO28 = 1
#define BUS_SW_ENABLE_clr()      GpioDataRegs.GPECLEAR.bit.GPIO133 = 1  /* GPIO133 */
#define V28_BUS_CNTL_CHA_clr()   GpioDataRegs.GPDCLEAR.bit.GPIO99 = 1   /* GPIO 99 */
#define INRUSH_CTR_CHA_clr()     GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1
#define V270_BUS_CNTL_CHA_clr()  GpioDataRegs.GPACLEAR.bit.GPIO23 = 1

/*
 * DC BUS Switching timeouts defined as per HW HDD
 */

#define BUS_50ms_TIMEOUT (50U) /* Bus charge and discharge timeout*/
#define BUS_1ms_TIMEOUT (1U) /* this time out presently unused and handled indirectly*/

/*High lift Mode DC BUS operational threshold in voltage, Bus Voltage should be > 245v*/
#define BUS_HIGHLIFT_MODE_OPERATIONAL_THRESHOLD (245.0f)
/*Variable Camber Mode DC BUS operational threshold in voltage, Bus Voltage should be > 26v && <31v*/
#define BUS_VC_MODE_OPERATIONAL_THRESHOLD (26.0f) /* Minimum operational threshold in Voltage >26v*/
#define BUS_VC_MODE_OPERATIONAL_THRESHOLD_MAX (31.0f) /* Maximum operational threshold in Voltage < 31v*/
#define DC_BUS_VOLTAGE_READ_VAR  (VBUS270_VSENSE_UNITV) /* link the ADC variable - linked DC BUS Averaged value- SPI ADC */
#define DC_BUS_VOLTAGE_READ_VAR_RAW (VBUS270_VSENSE_UNITV_raw) /* DC BUS Engineering Value based on RAW ADC Inpt */

#define NS_CNTL_PHA_I_UL    (2050U)
#define NS_CNTL_PHA_I_LL    (2040U)
#define NS_CNTL_PHC_I_UL    (2050U)
#define NS_CNTL_PHC_I_LL    (2040U)
#define NS_SLEEP_TIME       (50U)

/*      Global Variables
*/
STATE_ID G_eMcuMode = INITIALIZATION_MODE;
bool bInhibitFlap = true;    /* Whether or not the flap should be inhibited in the current state or mode*/

bool bFlapFail = false;
bool bFasNotAvailable = false;
bool bRigInProcess = false;
bool bGseConnected = false;
bool bFlapPosMiscompare = false;

bool bMcuFail = false;
bool bMcuWarning = false;
bool bScuFail = false;
bool bFlaFail = false;
bool bFpsuFail = false;
bool bMotorFail = false;
bool bFlapJam = false;

bool bRigModeFlg = false;

Uint16 u16McuBootCount = 0;

Timer_t tMcuResetTimer = TIMER_DEFAULTS;
Timer_t t28VdcMonitorDelayTimer = TIMER_DEFAULTS;
Timer_t tDcBusMonitorDelayTimer = TIMER_DEFAULTS;

/*      Local ROM Constants
*/

/*      Local Variable Declarations
*/

static bool_t bVcModeCmd = false;

DC_BUS_STATE CurrentBusState = BUS_IDLE_STATE;
DC_BUS_SUB_STATE CurrentBusSubState;
static bool bVcModeCmdPrev = BUS_VC_MODE;   /* by default previous mode VC*/
static uint16_t busTimeOutVar = 0;


/*      Local Function Prototypes
 *
 *
*/
/*******Bus Switching State Machine*****************************************/
static void MCU_BusIdleState(void);
static void MCU_BusHighLiftMode(void);
static void MCU_BusVariableCamberMode(void);
static void MCU_BusFailureMode(void);


/*      Function Definitions
*/

/*
    brief Entry event handler for all modes.

    Purpose:
        This is the entry event handler for all modes.  All entry conditions are setup
                during this function.

    param[in] mode  mode into which the state machine is entering into.

    return
                None.

*/
void McuEntryEventHandler(STATE_ID mode)
{
    /*PATH(McuEntryEventHandler,A);*/

    switch (mode)
    {
        case INITIALIZATION_MODE:
            /*PATH(McuEntryEventHandler,B);*/
            
            /* shall inhibit flap movement in this state*/
            bInhibitFlap = true;
            
            break;
        /* TEST_MODE and RIG_MODE perform the same tasks so no break 
           is necessary */
        case TEST_MODE:
            /*PATH(McuEntryEventHandler,C);*/
        case RIG_MODE:
            /*PATH(McuEntryEventHandler,D);*/

            u16McuBootCount = Nvm_State.bootcount;
            Nvm_State.bootcount++;
            break;
        /* BOOT_MODE and IDLE_MODE perform the same tasks so no break 
           is necessary */
        case BOOT_MODE:
            /*PATH(McuEntryEventHandler,E);*/
        case IDLE_MODE:
            /*PATH(McuEntryEventHandler,F);*/
            
            /* shall inhibit flap movement in this state*/
            bInhibitFlap = true;

            Timer_ResetTimer(&MotorTimer);
            break;
        case RUN_MODE:
            /*PATH(McuEntryEventHandler,G);*/
    
            /* shall enable flap movement in this state*/
            bInhibitFlap = false;

            break;
        case FAS_FAIL:
            /*PATH(McuEntryEventHandler,I);*/
            
            /* shall inhibit flap movement in this state */
            bInhibitFlap = true;
            Timer_ResetTimer(&MotorTimer);

            break;
        default:
            /*PATH(McuEntryEventHandler,M);*/
            break;
    }

    /*PATH(McuEntryEventHandler,N);*/
}/* end McuEntryEventHandler */


/*
    brief Event handler for Initialization mode.

    Purpose:
        This is the event handler for Initialization mode.  During initilization mode,
        a test is done for the presence of the Rig/Test mode discrete signals.  If they
        are detected, then the event handler will catch the Rig/Test event and transition
        the state machine to the Rig/Test state.  Otherwise, a transition to Boot mode 
        will be initiated when the InitDone event is posted.

    param[in] event  pointer to the event to be processed.

    return The mode to which the state machine is supposed to transition to.

*/
STATE_ID McuInitModeHandler(Evt const * event)
{
    STATE_ID ret = INITIALIZATION_MODE;
    /*PATH(McuInitModeHandler,G); */
    
    /* shall ensure that 'event' pointer is not null */ 
    if (event)
    {
        /*PATH(McuInitModeHandler,A);*/
    
        switch (event->eID)
        {
            case EVENT_ENTER_RIG_MODE:
                /*PATH(McuInitModeHandler,B);*/
                
                /* shall transition to Rig mode upon receiving an ASCII 'R' character via the serial port*/
                Nvm_State.Mode = RIG_MODE;
                ret = RIG_MODE;
                break;
            case EVENT_ENTER_TEST_MODE:
                /*PATH(McuInitModeHandler,C);*/
                
                /* shall transition to Test mode upon receiving an ASCII 'T' character via the serial port*/
                Nvm_State.Mode = TEST_MODE;
                ret = TEST_MODE;
                break;
            case EVENT_INIT_DONE:
                /*PATH(McuInitModeHandler,D);*/
                
                /* shall transition to Boot mode if initialization is complete and Rig/Test signals are not present*/
                ret = BOOT_MODE; 
                break;
            default:
                /*PATH(McuInitModeHandler,E);*/
                break;
        }
    
        /*PATH(McuInitModeHandler,F);*/
        Timer_SetTime(&Bit_XCommsDelayTimer, TIMER_ONESHOT, TIMER_2s);

    }
    
    /*PATH(McuInitModeHandler,H);*/
    return (ret);
    
}/* end McuInitModeHandler */


/*
    brief Event handler for Boot mode.

    Purpose:
        This is the event handler for Boot mode.  A transition to Idle mode is initiated
        when the BootDone event is passed in (representing boot mode completed).  A
        transition to Fail mode is initiated when the FaultDetected event is passed in
        (representing a fault occurred).  A transition to Powerdown mode is initiated
        when the PowerInterrupt event is passed in (representing loss of power).

    param[in] event  pointer to the event to be processed.

    return The mode to which the state machine is supposed to transition to.

*/
STATE_ID McuBootModeHandler(Evt const * event)
{
    STATE_ID ret = BOOT_MODE;

    /*PATH(McuBootModeHandler,J);*/
    
    if (event)
    {
        /*PATH(McuBootModeHandler,A);*/
    
        switch (event->eID)
        {
            case EVENT_BOOT_DONE:
                /*PATH(McuBootModeHandler,B);*/
                
                /* shall transition to the Idle state upon completion, if no faults are detected*/
                CH_STATUS_set();        /*Indicate that current MCU Channel is healthy*/
                ret = IDLE_MODE;
                break;
            case EVENT_FAULT_DETECTED:
                /*PATH(McuBootModeHandler,E);*/
                
                /* shall transition to the Fail state because a fault was detected during PBIT*/
                CH_STATUS_clr();        /*Indicate that current MCU channel is unhealthy*/
                ret = FAS_FAIL;
                break;
            default:
                /*PATH(McuBootModeHandler,G);*/
                break;
        }
    }
    
    /*PATH(McuBootModeHandler,H);*/
    return (ret);
}/* end McuBootModeHandler */


/*
    brief Event handler for Rig mode.

    Purpose:
        This is the event handler for Rig mode.

    param[in] event  pointer to the event to be processed.

    return The mode to which the state machine is supposed to transition to.

*/
STATE_ID McuRigModeHandler(Evt const * event)
{
    STATE_ID ret = RIG_MODE;
    
    /*PATH(McuRigModeHandler,E);*/
    
    if (event)
    {
        /*PATH(McuRigModeHandler,A);*/
    
        switch (event->eID)
        {
            default:
                /*PATH(McuRigModeHandler,C);*/
                break;
        }
    }
    
    /*PATH(McuRigModeHandler,D);*/
    return (ret);
} /* End McuRigModeHandler */


/*
    brief Event handler for Test mode.

    Purpose:
        This is the event handler for Test mode.

    param[in] event  pointer to the event to be processed.

    return The mode to which the state machine is supposed to transition to.

*/
STATE_ID McuTestModeHandler(Evt const * event)
{
    STATE_ID ret = TEST_MODE;
    
    /*PATH(McuTestModeHandler,E);*/
    
    if (event)
    {
        /*PATH(McuTestModeHandler,A);*/
    
        switch (event->eID)
        {
            default:
                /*PATH(McuTestModeHandler,C);*/
                break;
        }
    }
    
    /*PATH(McuTestModeHandler,D);*/
    return (ret);
} /* End McuTestModeHandler */


/*
    brief Event handler for Idle mode.

    Purpose:
        This is the event handler for Idle mode.  If a valid SYNC position command has been
        received by the MCU (represented by the SYNCcmd event), then a transition to
        Run mode will be initiated.

    param[in] event  pointer to the event to be processed.

    return The mode to which the state machine is supposed to transition to.

*/
STATE_ID McuIdleModeHandler(Evt const * event)
{
    STATE_ID ret = IDLE_MODE;
    
    /*PATH(McuIdleModeHandler,G);*/
    
    if (event)
    {
        /*PATH(McuIdleModeHandler,A);*/
    
        switch (event->eID)
        {
            case EVENT_POSITION_CMD:
                /*PATH(McuIdleModeHandler,B);*/
                
                /* shall transition to Run mode if a valid SCU command was received*/
                hallStartPosition = tHall.Position;
                Timer_SetTime(&underspeedMonitorTimer, TIMER_ONESHOT, TIMER_500ms);
                ret = RUN_MODE;
                break;
            case EVENT_FAULT_DETECTED:
                /*PATH(McuIdleModeHandler,C);*/

                CH_STATUS_clr();        /*Indicate that current MCU channel is unhealthy*/
                ret = FAS_FAIL;
                break;
            default:
                /*PATH(McuIdleModeHandler,F);*/
                break;
        }
    }
    
    /*PATH(McuIdleModeHandler,H);*/
    return (ret);
}/* end McuIdleModeHandler */


/*
    brief Event handler for FAS-Fail mode.

    Purpose:
        This is the event handler for the FAS-Fail mode.  
 
    param[in] event  pointer to the event to be processed.

    return The mode to which the state machine is supposed to transition to.
*/
STATE_ID McuFasFailHandler(Evt const * event)
{
    STATE_ID ret = FAS_FAIL;

    /*PATH(McuFASFailHandler,H);*/

    if (event)
    {
        /*PATH(McuFASFailHandler,A);*/
    
        switch (event->eID)
        {
            case EVENT_FAULT_CLEARED:
                /*PATH(McuFASFailHandler,C);*/
                CH_STATUS_set();        /*Indicate that current MCU channel is healthy*/
                ret = IDLE_MODE;
                break;
            default:
                /*PATH(McuFASFailHandler,F);*/
                break;
        }
    }
    
    /*PATH(McuFASFailHandler,G);*/
    return (ret);
} /* End McuFASFailHandler */


/*
    brief Event handler for Run mode.

    Purpose:
        This is the event handler for Run mode.  Once the system has reached its commanded
        position (represented by the PositionComplete event), a transition back to Idle mode
        will be initiated.

    param[in] event  pointer to the event to be processed.

    return The mode to which the state machine is supposed to transition to.

*/
STATE_ID McuRunModeHandler(Evt const * event)
{
    STATE_ID ret = RUN_MODE;
    
    /*PATH(McuRunModeHandler,H);*/

    if (event)
    {
        /*PATH(McuRunModeHandler,A);*/
    
        switch (event->eID)
        {
            case EVENT_FAULT_DETECTED:
                /*PATH(McuRunModeHandler,B);*/
                
                /* Transition to Idle mode when fault is detected (consistent with legacy code state transition) */
                ret = IDLE_MODE;
                break;
            case EVENT_POSITION_COMPLETE:
                /*PATH(McuRunModeHandler,D);*/
    
                /* shall transition back to Idle mode once the flap has reached its commanded position*/
                ret = IDLE_MODE;
                break;
            default:
                /*PATH(McuRunModeHandler,F);*/
                break;
        }
    }
    
    /*PATH(McuRunModeHandler,G);*/
    return (ret);
}/* end McuRunModeHandler */


/*
    brief Returns the state ID of the current state.

    Purpose:
        This routine retrieves the ID of the current state of the system.

    return STATE_ID
*/
STATE_ID McuGetState(void)
{
    /*PATH(McuGetState,A);*/

    /* shall return the enumerated ID of the current state*/
    return (G_eMcuMode);
} /* End McuGetState */

void MCU_BusSetVcModeCmd(bool_t bVcModeCommand)
{
    bVcModeCmd = bVcModeCommand;
}

bool_t MCU_BusGetVcModeCmd(void)
{
    return (bVcModeCmd);
}


/*
     Bus Idle State processing.

    Purpose:
        To process either HL or VC mode depending on bVcModeCmd command.

    Global Data Referenced: bVcModeCmd

    return void

    Preconditions and Assumptions:
        This routine should be called by the FSCU_BusStateMachineProcessState @1ms.

    remarks
        None
*/
static void MCU_BusIdleState(void)
{
    /* at power up bVcModeCmd = BUS_HL_MODE and bVcModeCmdPrev = BUS_VC_MODE */
    /*DC bus has to enter either mode (HL or VC) at power up or during operation */
    MCU_BusOutputInit(); /* reset the BUS output to default during new transition */

    NS_CNTL_PHA_CHA_clr(); /*deactivate neutral switch during transition*/
    NS_CNTL_PHC_CHA_clr();

    if (bVcModeCmd == BUS_VC_MODE)                    /*Variable Camber mode*/
    {
        CurrentBusState = VC_BUS_STATE;               /* DC BUS Mode state handling */
        CurrentBusSubState = SUBSTATE_VC_BUS_STATE_TRANSITION; /*DC BUS sub state handling*/
    }
    else /* HL mode for bVcModeCmd =0*/
    {
        CurrentBusState = HIGH_LIFT_STATE;            /* DC BUS Mode state handling */
        CurrentBusSubState = SUBSTATE_HIGHLIFT_BUS_TRANSITION; /*DC BUS sub state handling*/
    }

    bVcModeCmdPrev = bVcModeCmd;
    busTimeOutVar = BUS_1ms_TIMEOUT; /* 1 ms time out to be used in VC or HL sub state*/

}

/*
     DC BUS High Lift sub state processing.

    Purpose:
        Execute the DC BUS High Lift sub states like TRANSITION,WAIT,EXECUTE.

    Global Data Referenced: CurrentBusSubState

    return void

    Preconditions and Assumptions:
        This routine should be called by the FSCU_BusStateMachineProcessState @1ms.

    remarks
        None
*/
static void MCU_BusHighLiftMode(void)
{
    switch(CurrentBusSubState)
    {
      case SUBSTATE_HIGHLIFT_BUS_TRANSITION:
            V28_BUS_CNTL_CHA_clr();
            INRUSH_CTR_CHA_clr();
            VC_MODE_CHA_clr();
            CurrentBusSubState = SUBSTATE_HIGH_LIFT_WAIT_STATE;
            busTimeOutVar= BUS_50ms_TIMEOUT;
      break;
      case SUBSTATE_HIGH_LIFT_WAIT_STATE:
            V270_BUS_CNTL_CHA_set();

            if (busTimeOutVar != 0)
            {
               busTimeOutVar--;
            }
            else
            {
                if (DC_BUS_VOLTAGE_READ_VAR_RAW > BUS_HIGHLIFT_MODE_OPERATIONAL_THRESHOLD)
                {
                     CurrentBusSubState = SUBSTATE_HIGH_LIFT_EXECUTE;
                }
                else
                {
                     CurrentBusState = BUS_FAILURE_STATE;
                }

            }

       break;
      case SUBSTATE_HIGH_LIFT_EXECUTE:
              INRUSH_CTR_CHA_set();

              if ((McuGetState() != FAS_FAIL) && (bInhibitFlap != true))/* during critical fault scenario neutral switch should not be ON*/
              {
                  NS_CNTL_PHA_CHA_set();   /*activate neutral switch to be ready for Motor run*/
                  NS_CNTL_PHC_CHA_set();
              }
          break;

      default:
          break;

    }

    if (bVcModeCmd != bVcModeCmdPrev) /*Handle Bus mode switching during operation*/
    {
        CurrentBusState = BUS_IDLE_STATE;
    }

}

/*
     DC BUS Variable Camber sub state processing.

    Purpose:
        Execute the  DC BUS Variable Camber sub states like TRANSITION,WAIT,DISCHARGE,EXECUTE.

    Global Data Referenced: CurrentBusSubState

    return void

    Preconditions and Assumptions:
        This routine should be called by the FSCU_BusStateMachineProcessState @1ms.

    remarks
        None
*/

static void MCU_BusVariableCamberMode(void)
{
    switch (CurrentBusSubState)
    {
          case SUBSTATE_VC_BUS_STATE_TRANSITION:

              if (busTimeOutVar) /*VC mode do process VC output and have 1ms timeout*/
              {
                  V270_BUS_CNTL_CHA_clr();
                  VC_MODE_CHA_set();
                  busTimeOutVar--;
              }
              else
              {
                 if (DC_BUS_VOLTAGE_READ_VAR_RAW < BUS_VC_MODE_OPERATIONAL_THRESHOLD)
                 {
                     INRUSH_CTR_CHA_clr();
                     CurrentBusSubState = SUBSTATE_VC_BUS_WAIT_STATE;
                 }
                 else if (DC_BUS_VOLTAGE_READ_VAR_RAW > BUS_VC_MODE_OPERATIONAL_THRESHOLD_MAX)
                 {
                     /* Delay timers used to cover the period of time when discharging the bus where
                      * the voltages go up to ~34VDC and then back down to 28VDC over ~2 seconds.
                      * SPI ADC Chip +3.3V_HS supply is dragged down during the discharge which causes
                      * the +28V_SENSE and DC_BUS_SENSE conversions to be inaccurately high  */
                     Timer_SetTime(&t28VdcMonitorDelayTimer, TIMER_ONESHOT, TIMER_2s);
                     Timer_SetTime(&tDcBusMonitorDelayTimer, TIMER_ONESHOT, TIMER_2s);

                     /* Disable the GFD_I_SENSE Interrupt during the discharge event. This is a temporary
                      * fix until the HW has been addressed to resolve the issue.  */
                     XintRegs.XINT1CR.bit.ENABLE = INTERRUPT_DISABLE;

                     /* Discharge the Bus */
                     BUS_SW_ENABLE_set();
                     CurrentBusSubState = SUBSTATE_VC_BUS_DISCHARGE_STATE;
                 }
                 else
                 {
                     V28_BUS_CNTL_CHA_set();
                     CurrentBusSubState = SUBSTATE_VC_BUS_EXECUTE;
                 }

                 busTimeOutVar = BUS_50ms_TIMEOUT;
              }

              break;
          case SUBSTATE_VC_BUS_WAIT_STATE:
               V28_BUS_CNTL_CHA_set();

                if ((DC_BUS_VOLTAGE_READ_VAR_RAW > BUS_VC_MODE_OPERATIONAL_THRESHOLD) &&
                     (DC_BUS_VOLTAGE_READ_VAR_RAW < BUS_VC_MODE_OPERATIONAL_THRESHOLD_MAX))
                {
                    CurrentBusSubState = SUBSTATE_VC_BUS_EXECUTE;
                    busTimeOutVar = BUS_50ms_TIMEOUT;
                }
                else
                {
                    if(busTimeOutVar !=0)
                    {
                        busTimeOutVar--;
                    }
                    else
                    {
                        CurrentBusState = BUS_FAILURE_STATE;
                    }

                }
              break;
          case SUBSTATE_VC_BUS_DISCHARGE_STATE:
                if (DC_BUS_VOLTAGE_READ_VAR_RAW < BUS_VC_MODE_OPERATIONAL_THRESHOLD_MAX)
                {
                    BUS_SW_ENABLE_clr();
                    CurrentBusSubState = SUBSTATE_VC_BUS_EXECUTE;
                }
                if (busTimeOutVar !=0)
                {
                    busTimeOutVar--;
                }
                else
                {
                    CurrentBusState = BUS_FAILURE_STATE;
                }
              break;

          case SUBSTATE_VC_BUS_EXECUTE:
                  V28_BUS_CNTL_CHA_set();
                  INRUSH_CTR_CHA_set();

                  if ((McuGetState() != FAS_FAIL) && (bInhibitFlap != true))/* during critical fault scenario neutral switch should not be ON*/
                  {
                      NS_CNTL_PHA_CHA_set();   /*activate neutral switch to be ready for Motor run*/
                      NS_CNTL_PHC_CHA_set();
                  }
              break;
          default:

              break;
     }

     if (bVcModeCmd != bVcModeCmdPrev) /*Handle DC Bus mode switching during operation*/
     {
        CurrentBusState = BUS_IDLE_STATE;
     }

}

static void MCU_BusFailureMode(void)
{
    /*log the faults here */

     MCU_BusOutputInit(); /*set the Bus output to fail-safe state or default state*/

     openNeutralSwitch(); /*deactivate neutral switch in invalid combination*/
     
     if (bVcModeCmd != bVcModeCmdPrev) /*Handle DC Bus mode switching during operation*/
     {
        CurrentBusState = BUS_IDLE_STATE;
     }
}


/*
     DC BUS Main state processing for Highlift and Variable Camber mode.

    Purpose:
        Execute the states DC BUS states based on HL,VC, Bus failure mode.

    Global Data Referenced: None

    return void

    Preconditions and Assumptions:
        This routine should be called by the scheduler @1ms.

    remarks
        None
*/
void MCU_BusStateMachineProcessState(void)
{

    switch(CurrentBusState)
    {
        case  BUS_IDLE_STATE:
            MCU_BusIdleState();
            break;
        case HIGH_LIFT_STATE:
            MCU_BusHighLiftMode();
            break;
        case VC_BUS_STATE:
            MCU_BusVariableCamberMode();
            break;
        case BUS_FAILURE_STATE:
            MCU_BusFailureMode();
            break;
        default:
            MCU_BusFailureMode();
            break;
    }
    
    if ((McuGetState() == FAS_FAIL) || (bInhibitFlap == true))
    {
        openNeutralSwitch(); /* Deactivate neutral switch */
    }

}

/*

    Purpose:
        Open the neutral switch.

    Global Data Referenced: None

    return void

    Preconditions and Assumptions:
        This routine should be called when in bus fail or FAS fail state.

    remarks
        None
*/
void openNeutralSwitch(void)
{
    static uint16_t nsSleepTime = NS_SLEEP_TIME;
    nsSleepTime--;

    if (nsSleepTime > 0)
    {
        if((Adc_Raw.val.u16_I_PHA_CHA < NS_CNTL_PHA_I_UL) &&
                (Adc_Raw.val.u16_I_PHA_CHA > NS_CNTL_PHA_I_LL))
        {
            NS_CNTL_PHA_CHA_clr();            
          /* To avoid the large current flowing through the winding for 50 ms
             the PH C netural switch is also opened, found this behavior during testing*/
            NS_CNTL_PHC_CHA_clr(); 
            
        }
        if((Adc_Raw.val.u16_I_PHC_CHA < NS_CNTL_PHC_I_UL) &&
                (Adc_Raw.val.u16_I_PHC_CHA > NS_CNTL_PHC_I_LL))
        {
            NS_CNTL_PHC_CHA_clr();
          /* To avoid the large current flowing through the winding for 50 ms
             the PH C netural switch is also opened, found this behavior during testing*/
            NS_CNTL_PHA_CHA_clr();
        }
    }
    else
    {
        NS_CNTL_PHA_CHA_clr();
        NS_CNTL_PHC_CHA_clr();
        nsSleepTime = NS_SLEEP_TIME;
    }
}

/*
     DC BUS Output Initialization for Highlift and Variable Camber.

    Purpose:
        Initialize the DC BUS in to known output state.

    Global Data Referenced: None

    return void

    Preconditions and Assumptions:
        This routine should be called only once, prior to the scheduler loop.

    remarks
        None
*/
void MCU_BusOutputInit(void)
{
    VC_MODE_CHA_clr();
    BUS_SW_ENABLE_clr();
    V28_BUS_CNTL_CHA_clr();
    INRUSH_CTR_CHA_clr();
    V270_BUS_CNTL_CHA_clr();
}


/*end MCU.c*/
