/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        ReadfromBSW.h
 *   Module Name: ReadfromBSW
 *   Module Ver:  10.19
 *   Date:        Thu Jul 20 13:12:15 2023
 */

#ifndef RTW_HEADER_ReadfromBSW_h_
#define RTW_HEADER_ReadfromBSW_h_
#ifndef ReadfromBSW_COMMON_INCLUDES_
#define ReadfromBSW_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* ReadfromBSW_COMMON_INCLUDES_ */

#include "ASW_BSW_I.h"
#include "typedefs.h"
#include "ReadfromBSW_types.h"

extern void ReadfromBSW(const BSW_DATA_t *rtu_G_BSW_DATA, float32_t
  *rty_f32CurrentPhaseA, float32_t *rty_f32CurrentPhaseB, float32_t
  *rty_f32CurrentPhaseC, float32_t *rty_f32VoltSense5V, float32_t
  *rty_f32VoltSensePos15V, float32_t *rty_f32VoltSenseNeg15V, float32_t
  *rty_f32MotorTempPhaseA, float32_t *rty_f32MotorTempPhaseB, float32_t
  *rty_f32MotorTempPhaseC, eLruId_t *rty_eLRU_ID, eActId_t *rty_eCH_ID,
  eActuator_t *rty_eACTUATOR_ID, uint16_t *rty_MOTOR_ENABLE, bool_t
  *rty_CHX_STATUS, int16_t *rty_s16QuadPosition, float32_t *rty_f32StrokeRvdt,
  float32_t *rty_f32StrokeFused, float32_t *rty_f32FlapAngleRvdt, float32_t
  *rty_f32FlapAngleQuad, float32_t *rty_f32StrokeQuad, float32_t *rty_f32IAvrMax,
  float32_t *rty_f32RefSpeed, float32_t *rty_f32FdbSpeed, bool_t *rty_bVcModeCmd,
  bool_t *rty_bSensorFusionEnableCmd, int16_t *rty_Rst_Int_Gain, uint16_t
  *rty_xChannel_Enable_M);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'ReadfromBSW'
 */

/*-
 * Requirements for '<Root>': ReadfromBSW

 */
#endif                                 /* RTW_HEADER_ReadfromBSW_h_ */

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
