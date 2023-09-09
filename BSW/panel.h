/******************************************************************************
*  Copyright (c) 2022 Eaton Corporation
*
*  PROJECT NAME: MCU OFP
*  FILE NAME: panel.h
*  PURPOSE: Command/Feedback interface for the Panel and SYNC controller
*
*  This file describes the Public Interface for the Panel.  The interface provides
*  routines necessary to monitor the signals from the SYNC Controller.  This
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
#ifndef PANEL_H__
#define PANEL_H__

/*      Include Files
*/
#include "hallsensor.h"
#include "timer.h"
#include "A825Mgr.h"
#include "skewsensor.h"
#include "ASW_BSW_I.h"

/*      Public Type Definitions
*/

typedef struct
{
    bool_t      bVcModeCmdL;                /* Variable Camber Mode Command for Left Actuator from SCU:  True = VC, False = HL */
    bool_t      bVcModeCmdR;                /* Variable Camber Mode Command for Right Actuator from SCU:  True = VC, False = HL */
    bool_t      bSensorFusionEnableL;       /* Sensor Fusion Enable Command for Left Actuator from SCU.*/
    bool_t      bSensorFusionEnableR;       /* Sensor Fusion Enable Command for Right Actuator from SCU.*/
    float32_t   f32PositionCmdL;            /* Position Command for Left Actuator from SCU in Inches*/
    float32_t   f32PositionCmdR;            /* Position Command for Right Actuator from SCU in Inches*/
    float32_t   f32SpeedCmdL;               /* Speed Command for Left Actuator from SCU in Percent of max speed [0-1] */
    float32_t   f32SpeedCmdR;               /* Speed Command for Right Actuator from SCU in Percent of max speed [0-1] */
} tScuInterface_t;

typedef struct
{
    bool_t      bValidCmd;              /* Valid Command Received */
    bool_t      bVcModeCmd;             /* VC Mode Command */
    bool_t      bSensorFusionEnableCmd; /* Sensor Fusion Enable Command */
    float32_t   f32PositionCmd;         /* Position Command in Inches */
    int16_t     s16PositionCmdQuad;     /* Position Command in Motor Quad Counts */
    float32_t   f32SpeedCmd;            /* Speed Command in Percent of max speed [0-2] */
    float32_t   f32StrokeQuad;          /* Stroke Position in Inches generated from raw Quad Count */
} tPanel_t;

typedef struct
{
    tPanelOutputs_t  tSf;
    SKEW_SNSR_CALCS_T tSkewSnsrCalcs;
} tPanelNvmData_t;

/*      Public Variable Declarations
*/

extern tScuInterface_t tScuInterface;
extern tPanel_t  tPanel;
extern tPanel_t  tPanelx;
extern float32_t f32MotorCurrentAmps;

/* LEGACY Public Variable Declarations */

extern bool MotorReversing;

/*      Public ROM Constants
*/

/*      Public Defines
*/

/*      Public Interface Function Prototypes
*/

bool Panel_GetCommands(void);
void Panel_Command(void);
void Panel_GoToPosition(float32_t f32PositionCmd, float32_t f32SpeedCmd);
void Panel_SetFeedback(void);
int16_t Panel_ConvertStrokeToElectricalCycleCount(float32_t f32StrokeInInches);
float32_t Panel_ConvertElectricalCycleCountToStroke(int16_t s16ElectricalCycleCount);
bool Panel_IsPositionCommandValid(float32 f32PositionCmd);
bool Panel_IsSpeedCommandValid(float32 f32SpeedCmd);

#endif
/*end panel.h*/

