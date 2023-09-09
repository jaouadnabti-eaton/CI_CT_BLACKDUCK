/****************************************************************************************************
*  File name: skewsensor.h
*
*Purpose: Provides functionality involving Encoder
*
*  Copyright Notice:
*  All source code and data contained in this file is Proprietary and
*  Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
*  disclosed; in whole or in part, without the express written permission of
*  Eaton Aerospace.
*
*  Copyright (c) 2023 - Eaton Aerospace Group, All Rights Reserved.
*
*  Author            Date       CR#          Description
*  ------           ---------   ------     ---------------------------------------
*  Adam Bouwens     06/01/2023  N/A        Initial Rev
*
****************************************************************************************************/
#ifndef SKEWSENSOR_H__
#define SKEWSENSOR_H__

/*      Include Files
*/
#include "parameter.h"
#include "typedefs.h"
//#include "nvm.h"
#if defined(__SKEW_SNSR_RVDT__)
#include "rvdt.h"
#elif defined(__SKEW_SNSR_ENCODER__)
#include "encoder.h"
#endif


/*      Public Defines
*/
#if defined (__SKEW_SNSR_RVDT__)
#define SKEW_SNSR_CALCS_T               tRvdtCalcs_t
#define SKEW_SNSR_NVM_DATA_T            tRvdtNvmData_t
#define SKEW_SNSR_SET_NVM_RIG_DATA      Rvdt_SetNvmRigData
#define SKEW_SNSR_SET_NVM_PANEL_DATA    Rvdt_SetNvmPanelData
#define SKEW_SNSR_CALIBRATE_SNSR        Rvdt_CalibrateSensor
#define SKEW_SNSR_APPLY_CALIBRATION     Rvdt_ApplyCalibrationEquation
#elif defined (__SKEW_SNSR_ENCODER__)
#define SKEW_SNSR_CALCS_T               tEncoderCalcs_t
#define SKEW_SNSR_NVM_DATA_T            tEncoderNvmData_t
#define SKEW_SNSR_SET_NVM_RIG_DATA      Encoder_SetNvmRigData
#define SKEW_SNSR_SET_NVM_PANEL_DATA    Encoder_SetNvmPanelData
#define SKEW_SNSR_CALIBRATE_SNSR        Encoder_CalibrateSensor
#define SKEW_SNSR_APPLY_CALIBRATION     Encoder_ApplyCalibrationEquation
#endif

/*      Public Type Definitions
 */

/*      Public Variable Declarations
*/
extern SKEW_SNSR_CALCS_T tSkewSnsrCalcs;
extern bool StoreRigDataInNVM;

/*      Public ROM Constants
*/

/*      Public Interface Function Prototypes
*/
void SkewSnsr_GetStroke(void);
void SkewSnsr_GetFlapAngle(void);
float32_t SkewSnsr_CalcStroke(eActuator_t tActuatorIndex, float32_t f32PositionScaled0to1);
float32_t SkewSnsr_CalcFlapAngle(eActuator_t tActuatorIndex, float32_t f32PositionScaled0to1);
bool_t SkewSnsr_GetSensorType(void);
uint16_t SkewSnsr_GetPositionCount(void);

#endif
/* end encoder.h*/

