/******************************************************************************
*  Copyright (c) 2022 Eaton Corporation
*
*  PROJECT NAME: MCU OFP
*  FILE NAME: panel.c
*  PURPOSE: Command/Feedback interface for the MCU and SCU controllers
*
*  Routines necessary to monitor the signals from the SYNC Controller.  This
*  driver implements the logic for determining valid Panel Commands and setting the
*  signals back to the SYNC controller.
*
*  Copyright Notice:
*  All source code and data contained in this file is Proprietary and
*  Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
*  disclosed; in whole or in part, without the express written permission of
*  Eaton Aerospace.
*
*  Copyright (c) 2022 - Eaton Aerospace Group, All Rights Reserved.
*
*  Author            Date             CR#          Description
*  ------          ---------         ------   ---------------------------------------
*  AB               12/07/2022       N/A      Port to MCU
******************************************************************************/

/*      Include Files
*/
#include "parameter.h"
#include "csci.h"
#include "gpio.h"
#include "panel.h"
//#include "gse.h"
#include "actuation.h"
#include "timer.h"
#include "mcu.h"
#include "nvm.h"
#include "bitmonitors.h"
#include "A825_MsgInfo.h"
#include "A825Mgr.h"
#include "adc.h"
#include "rvdt.h"
#include "ASW_BSW_I.h"

/*      Local Type Definitions
*/

/*      Local Defines
*/

/*      Global Variables
*/
tScuInterface_t tScuInterface = { 0 };
tPanel_t tPanel = { 0 };
tPanel_t tPanelx = { 0 };

bool MotorReversing = false;

/*      Local ROM Constants
*/

/*      Local Variable Declarations
*/

/* VLJ LEGACY Local Variables */
Timer_t tReversalTimer = TIMER_DEFAULTS; /* timer used for pausing when reversing direction*/
float32 SpeedOfMotor = 0.0;

/*      Local Function Prototypes
*/
void Panel_GetSyncCommands(void);
void Panel_SetPanelCommands(void);
void Panel_SetFeedbackSignals(void);

/*      Function Definitions
*/

/****************************************************************************************************
*  Function: Panel_GetSyncCommands
*  Purpose: Read in the Commands received from the SYNC Controller
*  Global Inputs: tScuInterface
*  Global Outputs:
*  Input:
*  Output:
****************************************************************************************************/
void Panel_GetSyncCommands(void)
{
    float32_t f32Temp = 0.0F; /* Use temp to protect from interrupts */
    bool_t bTemp = false;  /* Use temp to protect from interrupts */

    /* Get Primary Bus Sync Commands */
    /* Get SYNC Command Signals */
    A825Mgr_GetSignal_BOOL(&tA825_Bus[ARINC825_CNTRL_BUS].tRx, A825_CNTRL_BUS_RX_FLAP_CMD, SIGNAL1, &bTemp);
    tScuInterface.bVcModeCmdL = bTemp;
    A825Mgr_GetSignal_BOOL(&tA825_Bus[ARINC825_CNTRL_BUS].tRx, A825_CNTRL_BUS_RX_FLAP_CMD, SIGNAL2, &bTemp);
    tScuInterface.bSensorFusionEnableL = bTemp;
    A825Mgr_GetSignal_BNR(&tA825_Bus[ARINC825_CNTRL_BUS].tRx, A825_CNTRL_BUS_RX_FLAP_CMD, SIGNAL3, &f32Temp);
    tScuInterface.f32PositionCmdL = f32Temp;
    A825Mgr_GetSignal_BNR(&tA825_Bus[ARINC825_CNTRL_BUS].tRx, A825_CNTRL_BUS_RX_FLAP_CMD, SIGNAL4, &f32Temp);
    tScuInterface.f32SpeedCmdL = f32Temp;
    A825Mgr_GetSignal_BOOL(&tA825_Bus[ARINC825_CNTRL_BUS].tRx, A825_CNTRL_BUS_RX_FLAP_CMD, SIGNAL5, &bTemp);
    tScuInterface.bVcModeCmdR = bTemp;
    A825Mgr_GetSignal_BOOL(&tA825_Bus[ARINC825_CNTRL_BUS].tRx, A825_CNTRL_BUS_RX_FLAP_CMD, SIGNAL6, &bTemp);
    tScuInterface.bSensorFusionEnableR = bTemp;
    A825Mgr_GetSignal_BNR(&tA825_Bus[ARINC825_CNTRL_BUS].tRx, A825_CNTRL_BUS_RX_FLAP_CMD, SIGNAL7, &f32Temp);
    tScuInterface.f32PositionCmdR = f32Temp;
    A825Mgr_GetSignal_BNR(&tA825_Bus[ARINC825_CNTRL_BUS].tRx, A825_CNTRL_BUS_RX_FLAP_CMD, SIGNAL8, &f32Temp);
    tScuInterface.f32SpeedCmdR = f32Temp;

    return;
}

/****************************************************************************************************
*  Function: Panel_SetPanelCommands
*  Purpose: Set the panel command based upon the status of the ARINC825 control bus.
*  Global Inputs: tScuInterface
*  Global Outputs: tPanel, tPanelx
*  Input:
*  Output:
****************************************************************************************************/
void Panel_SetPanelCommands(void)
{

    /* Set the Communication Established flag for the Control Bus based on the SCU Stale Status */
    if(tA825_Bus[ARINC825_CNTRL_BUS].tRx.ptMsgList[A825_CNTRL_BUS_RX_FLAP_CMD].bMsgStaleStatus == false)
    {
        /* Set Control Bus SCU as having established communication */
        tA825_Bus[ARINC825_CNTRL_BUS].bCommsEstablished = true;
    }

    /* Decode Channel to save off proper Panel Commands for Actuator */
    if(G_eActId == LEFT_ACTUATOR)
    {   /* Left Actuator is On-Side, Right Actuator is Cross-Side */
        /* Save VC Mode Commands */
        tPanel.bVcModeCmd = tScuInterface.bVcModeCmdL;
        tPanelx.bVcModeCmd = tScuInterface.bVcModeCmdR;
        /* Save Sensor Fusion Enable Commands */
        tPanel.bSensorFusionEnableCmd = tScuInterface.bSensorFusionEnableL;
        tPanelx.bSensorFusionEnableCmd = tScuInterface.bSensorFusionEnableR;
        /* Save Position Commands */
        tPanel.f32PositionCmd = tScuInterface.f32PositionCmdL;
        tPanelx.f32PositionCmd = tScuInterface.f32PositionCmdR;
        /* Save Speed Commands */
        tPanel.f32SpeedCmd = tScuInterface.f32SpeedCmdL;
        tPanelx.f32SpeedCmd = tScuInterface.f32SpeedCmdR;
    }
    else /* if(G_eActId == RIGHT_ACTUATOR) */
    {   /* Right Actuator is On-Side, Left Actuator is Cross-Side*/
        /* Save VC Mode Commands */
        tPanel.bVcModeCmd = tScuInterface.bVcModeCmdR;
        tPanelx.bVcModeCmd = tScuInterface.bVcModeCmdL;
        /* Save Sensor Fusion Enable Commands */
        tPanel.bSensorFusionEnableCmd = tScuInterface.bSensorFusionEnableR;
        tPanelx.bSensorFusionEnableCmd = tScuInterface.bSensorFusionEnableL;
        /* Save Position Commands */
        tPanel.f32PositionCmd = tScuInterface.f32PositionCmdR;
        tPanelx.f32PositionCmd = tScuInterface.f32PositionCmdL;
        /* Save Speed Commands */
        tPanel.f32SpeedCmd = tScuInterface.f32SpeedCmdR;
        tPanelx.f32SpeedCmd = tScuInterface.f32SpeedCmdL;
    }

    /* Set the BUS Command HL/VC based on the VC Mode Command signal on the interface */
    MCU_BusSetVcModeCmd(tPanel.bVcModeCmd);

    return;
}

/****************************************************************************************************
*  Function: Panel_GetCommands
*  Purpose: Read in the Commands received from the SYNC Controller
*  Global Inputs: tScuInterface
*  Global Outputs:
*  Input:
*  Output:
****************************************************************************************************/
bool Panel_GetCommands(void)
{
    bool bValidPositionCmd = false;
    bool bValidSpeedCmd = false;
    bool bValidCmd = false;

    /* Get the latest SYNC Controller Commands on both busses */
    Panel_GetSyncCommands();

    /* Set the panel command from the SYNC Bus Commands. */
    Panel_SetPanelCommands();

    /* Validity check for Position Command */
    bValidPositionCmd = Panel_IsPositionCommandValid(tPanel.f32PositionCmd);
    /* Validity check for Speed Command */
    bValidSpeedCmd = Panel_IsSpeedCommandValid(tPanel.f32SpeedCmd);

    /* Check if both Speed and Position Commands are valid. If so, Convert Position commmand to
     * Electrical Position Command and return valid indication. */
    if((bValidPositionCmd == true) && (bValidSpeedCmd == true))
    {
        /* Convert Position Command to Electrical Cycles */
        tPanel.s16PositionCmdQuad = Panel_ConvertStrokeToElectricalCycleCount(tPanel.f32PositionCmd);
        tPanelx.s16PositionCmdQuad = Panel_ConvertStrokeToElectricalCycleCount(tPanelx.f32PositionCmd); // For debug only
        bValidCmd = true;   /* Both Position and Speed Commands are valid. */
    }

    return bValidCmd;
}

/****************************************************************************************************
*  Function: Panel_Command
*  Purpose: Read the SYNC Command Inputs, Save to appropriate channel, and decode the command.
*  Call the function to execute the command.
*  Global Inputs: tScuInterface, tPanel, tPanelx
*  Global Outputs: tScuInterface, tPanel, tPanelx
*  Input:
*  Output:
****************************************************************************************************/
void Panel_Command( void )
{
    /* Get SYNC Command Signals */
    tPanel.bValidCmd = Panel_GetCommands();

    /* shall retrieve the corresponding hall position from NVM for each FSL command ID.*/
    /* shall set the targeted flap position according to the retrieved position.*/
    /* shall retrieve no hall position for FSL command IDs that do not correspond to an*/
    /*        actual flap position (FSL_In_invalid)*/
    if ((McuGetState() == RUN_MODE) || (MotorReversing == true))
    {
        if(tPanel.bValidCmd == true)
        {
            /* Execute Panel to Position */
            Panel_GoToPosition(tPanel.f32PositionCmd, tPanel.f32SpeedCmd);

            /* Save a copy of last known Valid Position Command and Speed Command to use when
             * determining whether or not to start motor up again when position command has
             * been reached or motor was commanded to stop through Speed Command. */
            tNvm.tData.f32PositionCmdOld = tPanel.f32PositionCmd;
            tNvm.tData.f32SpeedCmdOld = tPanel.f32SpeedCmd;
        }
    }
    /* Check if a valid command has been received to determine if a transition to RUN MODE is required */
    else if (tPanel.bValidCmd == true)
    {
        /* Transition to RUN mode if:
         * - New position command.
         * - Last speed command was 0% (STOP Command) and new speed command is not 0% */
        if((tPanel.f32PositionCmd != tNvm.tData.f32PositionCmdOld) ||
           ((tNvm.tData.f32SpeedCmdOld == 0.0F) && (tPanel.f32SpeedCmd != tNvm.tData.f32SpeedCmdOld)))

        {
            /* Post the new event */
            Events_PostEvent(EVENT_POSITION_CMD, 0);
        }
    }

    return;
}

/*
    brief Commands the motor to move to the given position

    Purpose:
        Commands the motor to move to the given position, with motor speed
        clamped at MAX_SPEED_COMMAND_FACTOR.

    param[in] f32PositionCmd New target position, in terms of stroke inches
    param[in] f32Speed    Targeted speed of motor

    Global Variables Referenced:
        #MotorCmd
        #ReversalTimer
        #MotorReversing

    return  void

*/
void Panel_GoToPosition(float32_t f32PositionCmd, float32_t f32Speed)
{
    float32_t f32Delta = 0.0F;

    /* Speed command of 0% means stop. Otherwise check for position reached or new position command */
    if(f32Speed <= 0.0F)
    {
        /* Set stop position to current position */
        MotorCmd.f32StopPosition = G_ASW_DATA.tPanelOutputData.f32StrokeFused;
        MotorCmd.MotorStop = true;
        /* Transition back into IDLE_MODE and stop motor. */
        Events_PostEvent(EVENT_POSITION_COMPLETE, 0);
    }
    /* Positive Speed Command Percentage */
    else
    {
        if (MotorCmd.f32StopPosition != f32PositionCmd)
        {
            /* Received a new command */
            if ((MotorCmd.MotorStop == false) && (MotorReversing == false)) /* If motor is moving and not reversing */
            {
                if (((G_ASW_DATA.tPanelOutputData.f32StrokeFused > f32PositionCmd) && (MotorCmd.MotorDirection == CW)) ||
                    ((G_ASW_DATA.tPanelOutputData.f32StrokeFused < f32PositionCmd) && (MotorCmd.MotorDirection == CCW)))
                {
                    /* keep running until f32PositionCmd has been met or exceeded */
                    MotorCmd.f32StopPosition = G_ASW_DATA.tPanelOutputData.f32StrokeFused;
                    MotorReversing = true;
                    Timer_SetTime(&tReversalTimer, TIMER_ONESHOT, TIMER_1500ms);
                    Events_PostEvent(EVENT_POSITION_COMPLETE, 0); /* Transition back into IDLE_MODE */
                }
            }
            else if ((Timer_IsExpired(&tReversalTimer) == true) && (MotorReversing == true)) /* Delay has expired */
            {
                MotorReversing = false;
                Events_PostEvent(EVENT_POSITION_CMD, 0); /* Post Event to transition back into RUN_MODE */
            }

            if(MotorReversing == false) /* We are no longer reversing */
            {
                MotorCmd.f32StopPosition = f32PositionCmd;

                /* Set direction */
                if (MotorCmd.f32StopPosition >= G_ASW_DATA.tPanelOutputData.f32StrokeFused)
                {
                    MotorCmd.MotorDirection = CW;
                }
                else
                {
                    MotorCmd.MotorDirection = CCW;
                }

                MotorCmd.MotorStop = false;
                MotorCmd.MotorStart = true;
            }
        }
        else
        {
            /* calculate position delta based on the motor direction*/
            if (MotorCmd.MotorDirection == CW)
            {
                f32Delta = MotorCmd.f32StopPosition - G_ASW_DATA.tPanelOutputData.f32StrokeFused;
            }
            else if (MotorCmd.MotorDirection == CCW)
            {
                f32Delta = G_ASW_DATA.tPanelOutputData.f32StrokeFused - MotorCmd.f32StopPosition;
            }

            /* Check if position has been reached */
            if (f32Delta <= 0.0F)
            {
                Events_PostEvent(EVENT_POSITION_COMPLETE, 0); /* Transition back into IDLE_MODE */
            }
        }
    }

    /* Limit Check the commanded speed */
    if (f32Speed > MAX_SPEED_COMMAND_FACTOR)
    {
        /* Set to maximum speed for commands greater than maximum */
        f32Speed = MAX_SPEED_COMMAND_FACTOR;
    }
    else if (f32Speed < 0.0F)
    {
        /* Set to minimum speed for commands less than minimum */
        f32Speed = 0.0F;
    }
    /* Set the motor speed to commanded speed */
    MotorCmd.MotorSpeed = f32Speed;

    return;
}

bool Panel_IsPositionCommandValid(float32 f32PositionCmd)
{
    bool bValid = false;
    float32_t f32MinStrokeLimit, f32MaxStrokeLimit;

    /* Set the Min/Max Stroke Limit based on the Panel Type */
    switch (G_eLruId)
    {
        case MCU_LIB_LRU:
        case MCU_RIB_LRU:
        {
            /* Inboard Panel:  Set Limits */
            f32MinStrokeLimit = MIN_STROKE_LIMIT_IB;
            f32MaxStrokeLimit = MAX_STROKE_LIMIT_IB;
            break;
        }
        case MCU_LOB_LRU:
        case MCU_ROB_LRU:
        default:
        {
            /* Outboard Panel:  Set Limits */
            f32MinStrokeLimit = MIN_STROKE_LIMIT_OB;
            f32MaxStrokeLimit = MAX_STROKE_LIMIT_OB;
            break;
        }
    }

    /* Limit check the stroke command */
    if(f32PositionCmd < f32MinStrokeLimit)
    {   /* Stroke Command is less than minimum stroke limit allowed. */
        bValid = false;
    }   /* Stroke Command is less than minimum stroke limit allowed. */
    else if(f32PositionCmd > f32MaxStrokeLimit)
    {   /* Stroke Command is greater than maximum stroke limit allowed. */
        bValid = false;
    }   /* Stroke Command is greater than maximum stroke limit allowed. */
    else
    {   /* Stroke Command is valid */
        bValid = true;
    }   /* Stroke Command is valid */

#if defined(__NO_STROKE_LIMIT__)
    bValid = true;
#endif

    return bValid;
}

bool Panel_IsSpeedCommandValid(float32 f32SpeedCmd)
{
    bool bValid = false;

    /* Limit check the speed command */
    if(f32SpeedCmd < 0.0)
    {   /* Speed Command is less than minimum speed limit allowed. */
        bValid = false;
    }   /* Speed Command is less than minimum speed limit allowed. */
    else if(f32SpeedCmd > MAX_SPEED_COMMAND_FACTOR)
    {   /* Speed Command is greater than maximum speed limit allowed. */
        bValid = false;
    }   /* Speed Command is greater than maximum speed limit allowed.  */
    else
    {   /* Speed Command is valid */
        bValid = true;
    }   /* Speed Command is valid */

    return bValid;
}

/****************************************************************************************************
*  Function: Panel_ConvertStrokeToElectricalCycleCount
*  Purpose: Transfer function for Panel Command in Inches to Electrical Cycles.
*  Global Inputs:
*  Global Outputs:
*  Input: float32_t f32StrokeInInches     // Stroke Command in inches
*  Output: int16_t - Stroke Position Command in Electrical Cycle Counts
****************************************************************************************************/
int16_t Panel_ConvertStrokeToElectricalCycleCount(float32_t f32StrokeInInches)
{
    int16_t s16ElectricalCycleCounts = 0U;

    /* Transfer function here */
    s16ElectricalCycleCounts = (int16_t) (f32StrokeInInches * ELECTRICAL_CYCLES_PER_STROKE_INCH);

    return s16ElectricalCycleCounts;
}
/****************************************************************************************************
*  Function: Panel_ConvertElectricalCycleCountToStroke
*  Purpose: Transfer function for Panel Command in Inches to Electrical Cycles.
*  Global Inputs:
*  Global Outputs:
*  Input: float32_t f32StrokeCommand     // Stroke Command in inches
*  Output: int16_t - Stroke Position Command in Electrical Cycle Counts
****************************************************************************************************/
float32_t Panel_ConvertElectricalCycleCountToStroke(int16_t s16ElectricalCycleCount)
{
    float32_t f32StrokeInInches = 0.0F;

    /* Transfer function here */
    f32StrokeInInches = (float32_t) (s16ElectricalCycleCount * STROKE_INCHES_PER_ELECTRICAL_CYCLE);

    return f32StrokeInInches;
}

/****************************************************************************************************
*  Function: Panel_SetFeedback
*  Purpose: Set the signals in the PCU Position Status Feedback and Senor Data Messages to go to SYNC
*  Global Inputs: tScuInterface, tPanel, tPanelx
*  Global Outputs:
*  Input:
*  Output:
****************************************************************************************************/
void Panel_SetFeedback(void)
{
    STATE_ID tMcuState = McuGetState();

    /* Set the Position in inches generated from the raw Quad count of each channel (used for
     * debugging purposes only  */
    tPanel.f32StrokeQuad = Panel_ConvertElectricalCycleCountToStroke(tHall.Position);
#if defined(__HALLX_CONFIGURED)
    tPanelx.f32StrokeQuad = Panel_ConvertElectricalCycleCountToStroke(tHallx.Position);
#endif

    /* MCU Status Bits */
    tMcuStatus.bit.Normal_Op_State = ((tMcuState == IDLE_MODE) | (tMcuState == RUN_MODE));
    tMcuStatus.bit.Ground_Maint_State = ((tMcuState == RIG_MODE) | (tMcuState == TEST_MODE));
    tMcuStatus.bit.SCU_Bus_Input_Fault = (LatchedFaults_Stat.bit.bA825CntrlBusPassive | LatchedFaults_Stat.bit.bA825CntrlBusOff);


    if ((Inhibits_Stat.all != 0U) || (CriticalFaults_Stat.all != 0U) || (LatchedFaults_Stat.all != 0U))
    {
        tMcuStatus.bit.MCU_Control_Valid = false;
    }
    else
    {
        tMcuStatus.bit.MCU_Control_Valid = true;
    }
    tMcuStatus.bit.MCU_Engaged = (tMcuStatus.bit.MCU_Control_Valid & !LatchedWarnings_Stat.bit.bMcuDisabled);

    tMcuStatus.bit.SCU_Bus_Valid = !(LatchedFaults_Stat.bit.bA825CntrlBusCommsStale | LatchedFaults_Stat.bit.bA825CntrlBusPassive | LatchedFaults_Stat.bit.bA825CntrlBusOff);
    tMcuStatus.bit.Flap_In_Motion = ((MotorCmd.MotorRunning) | (tSpeed.Speed > 0.0F));
    tMcuStatus.bit.Brake_Engaged = !MotorCmd.BrakeActivated;
    tMcuStatus.bit.Rig_Status = Nvm_State.rigstatus;

    /* MCU Fault Bits */
    tMcuFault.bit.Sensor_Fusion_Covariance = (CriticalFaults_Stat.bit.SfBiasCheck_M | CriticalFaults_Stat.bit.SfMeasResidualCheck_M);
    tMcuFault.bit.Sensor_Fault = (tMcuMaint.bit.RVDT_Position_Fault | tMcuMaint.bit.RVDT_Excitation_Fault |
                                  tMcuMaint.bit.Hall_Sensor_Fault | tMcuMaint.bit.Brake_Hold_Fault | tMcuMaint.bit.Brake_Drive_Fault);
    tMcuFault.bit.Flap_Jam_Active = Inhibits_Stat.bit.FlapJam_M;
    tMcuFault.bit.Motor_Brake_Fault = CriticalFaults_Stat.bit.BrakeHold_M;

    /* MCU Maintenance Bits */
    tMcuMaint.bit.RVDT_Position_Fault = LatchedWarnings_Stat.bit.bRvdtPositionFault_M;
    tMcuMaint.bit.RVDT_Excitation_Fault = LatchedFaults_Stat.bit.RvdtExcitationFault;
    tMcuMaint.bit.Hall_Sensor_Fault = (LatchedFaults_Stat.bit.HallSequence_M | LatchedFaults_Stat.bit.InvalidHall_M);
    tMcuMaint.bit.Brake_Hold_Fault = CriticalFaults_Stat.bit.BrakeHold_M;
    tMcuMaint.bit.Brake_Drive_Fault = LatchedFaults_Stat.bit.BrakeSwitch_M;


    /* Set the signals for the Position Status Fault Maintenance Message to SYNC Controller */
    Panel_SetFeedbackSignals();     /* MCU Bus Signals to send to SCU*/

    return;
}

/****************************************************************************************************
*  Function: Panel_SetFeedbackSignals
*  Purpose: Set the signals in the MCU Position Status Feedback and Sensor Data Messages to go to SYNC
*  Global Inputs: tScuInterface, tPanel, tPanelx
*  Global Outputs:
*  Input:
*  Output:
****************************************************************************************************/
void Panel_SetFeedbackSignals(void)
{
    /* Set the signals for the Position Status Fault Maintenance Message to SYNC Controller */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_CNTRL_BUS].tTx, A825_CNTRL_BUS_TX_MCU_PSN_STATUS_FLT_MAINT, SIGNAL1, G_ASW_DATA.tPanelOutputData.f32StrokeFused);  /* Position Estimate from Sensor Fusion */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_CNTRL_BUS].tTx, A825_CNTRL_BUS_TX_MCU_PSN_STATUS_FLT_MAINT, SIGNAL2, tMcuStatus.all);  /* MCU Status Signals */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_CNTRL_BUS].tTx, A825_CNTRL_BUS_TX_MCU_PSN_STATUS_FLT_MAINT, SIGNAL3, tMcuFault.all);   /* MCU Fault Signals  */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_CNTRL_BUS].tTx, A825_CNTRL_BUS_TX_MCU_PSN_STATUS_FLT_MAINT, SIGNAL4, tMcuMaint.all);   /* MCU Maint Signals  */

    /* Set the signals for the Sensor Data to SYNC Controller */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_CNTRL_BUS].tTx, A825_CNTRL_BUS_TX_MCU_SENSOR_DATA, SIGNAL1, f32MotorCurrentAmps);                 /* Current Feedback */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_CNTRL_BUS].tTx, A825_CNTRL_BUS_TX_MCU_SENSOR_DATA, SIGNAL2, tHall.Position);                      /* Quad Count Feedback */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_CNTRL_BUS].tTx, A825_CNTRL_BUS_TX_MCU_SENSOR_DATA, SIGNAL3, tSkewSnsrCalcs.tStroke.f32Stroke);    /* Skew Sensor Position in Inches Feedback */
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_CNTRL_BUS].tTx, A825_CNTRL_BUS_TX_MCU_SENSOR_DATA, SIGNAL4, SkewSnsr_GetPositionCount());         /* Skew Sensor Position Counts*/
    A825Mgr_SetSignal(&tA825_Bus[ARINC825_CNTRL_BUS].tTx, A825_MAINT_BUS_TX_MCU_SENSOR_DATA, SIGNAL5, SkewSnsr_GetSensorType());            /* Skew Sensor Type */

    return;
}

/* end panel.c */

