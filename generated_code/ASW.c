/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        ASW.c
 *   Module Name: ASW
 *   Module Ver:  10.44
 *   Date:        Thu Jul 20 13:12:34 2023
 */

#include "ASW.h"
#include "ASW_BSW_I.h"
#include "typedefs.h"
#include "ASW_private.h"
#include <string.h>
#include "SensorFusion.h"
#include "PID_Loop.h"
#include "ReadfromBSW.h"
#include "WriteToBSW.h"

/*
 *   Function:    ASW_Init
 *   Model name:  ASW
 *   Description: System initialize for referenced model: 'ASW'
 *   Arguments:   DW_ASW_fwu4_T *localDW
 *   Return:      void
 */
void ASW_Init(DW_ASW_fwu4_T *localDW)
{
  /* SystemInitialize for ModelReference: '<Root>/Speed Regulator' */
  PID_Loop_Init(&(localDW->SpeedRegulator_InstanceData.rtdw));
}

/*
 *   Function:    ASW
 *   Model name:  ASW
 *   Description: Output and update for referenced model: 'ASW'
 *   Arguments:   const BSW_DATA_t *rtu_G_BSW_DATA
 *                ASW_DATA_t *rty_G_ASW_OUT
 *                DW_ASW_fwu4_T *localDW
 *   Return:      void
 */
void ASW(const BSW_DATA_t *rtu_G_BSW_DATA, ASW_DATA_t *rty_G_ASW_OUT,
         DW_ASW_fwu4_T *localDW)
{
  /* local block i/o variables */
  float32_t rtb_ReadFromBsw_o1;
  float32_t rtb_ReadFromBsw_o2;
  float32_t rtb_ReadFromBsw_o3;
  float32_t rtb_ReadFromBsw_o4;
  float32_t rtb_ReadFromBsw_o5;
  float32_t rtb_ReadFromBsw_o6;
  float32_t rtb_ReadFromBsw_o7;
  float32_t rtb_ReadFromBsw_o8;
  float32_t rtb_ReadFromBsw_o9;
  float32_t rtb_ReadFromBsw_o16;
  float32_t rtb_ReadFromBsw_o17;
  float32_t rtb_ReadFromBsw_o18;
  float32_t rtb_ReadFromBsw_o19;
  float32_t rtb_ReadFromBsw_o20;
  float32_t rtb_ReadFromBsw_o21;
  float32_t rtb_ReadFromBsw_o22;
  float32_t rtb_ReadFromBsw_o23;
  float32_t rtb_f32StrokeFused;
  float32_t rtb_f32EstimatedBias;
  float32_t rtb_f32QuadResidual;
  float32_t rtb_f32SkewResidual;
  float32_t rtb_SpeedRegulator;
  eLruId_t rtb_ReadFromBsw_o10;
  eActuator_t rtb_ReadFromBsw_o12;
  eActId_t rtb_ReadFromBsw_o11;
  uint16_t rtb_ReadFromBsw_o13;
  uint16_t rtb_ReadFromBsw_o27;
  int16_t rtb_ReadFromBsw_o15;
  int16_t rtb_ReadFromBsw_o26;
  int16_t rtb_s16QuadFused;
  bool_t rtb_ReadFromBsw_o14;
  bool_t rtb_ReadFromBsw_o24;
  bool_t rtb_ReadFromBsw_o25;

  /* ModelReference: '<Root>/ReadFromBsw' */
  ReadfromBSW(rtu_G_BSW_DATA, &rtb_ReadFromBsw_o1, &rtb_ReadFromBsw_o2,
              &rtb_ReadFromBsw_o3, &rtb_ReadFromBsw_o4, &rtb_ReadFromBsw_o5,
              &rtb_ReadFromBsw_o6, &rtb_ReadFromBsw_o7, &rtb_ReadFromBsw_o8,
              &rtb_ReadFromBsw_o9, &rtb_ReadFromBsw_o10, &rtb_ReadFromBsw_o11,
              &rtb_ReadFromBsw_o12, &rtb_ReadFromBsw_o13, &rtb_ReadFromBsw_o14,
              &rtb_ReadFromBsw_o15, &rtb_ReadFromBsw_o16, &rtb_ReadFromBsw_o17,
              &rtb_ReadFromBsw_o18, &rtb_ReadFromBsw_o19, &rtb_ReadFromBsw_o20,
              &rtb_ReadFromBsw_o21, &rtb_ReadFromBsw_o22, &rtb_ReadFromBsw_o23,
              &rtb_ReadFromBsw_o24, &rtb_ReadFromBsw_o25, &rtb_ReadFromBsw_o26,
              &rtb_ReadFromBsw_o27);

  /* ModelReference: '<Root>/Sensor Fusion' */
  SensorFusion(&rtb_ReadFromBsw_o15, &rtb_ReadFromBsw_o16, &rtb_ReadFromBsw_o17,
               &rtb_ReadFromBsw_o25, &rtb_f32StrokeFused, &rtb_f32EstimatedBias,
               &rtb_f32QuadResidual, &rtb_f32SkewResidual, &rtb_s16QuadFused,
               &(localDW->SensorFusion_InstanceData.rtdw));

  /* ModelReference: '<Root>/Speed Regulator' */
  PID_Loop(&rtb_ReadFromBsw_o22, &rtb_ReadFromBsw_o23, &rtb_ReadFromBsw_o21,
           &rtb_ReadFromBsw_o24, &rtb_ReadFromBsw_o26, &rtb_ReadFromBsw_o27,
           &rtb_SpeedRegulator, &(localDW->SpeedRegulator_InstanceData.rtdw));

  /* ModelReference: '<Root>/WriteToBsw' incorporates:
   *  Constant: '<Root>/Constant1'
   *  Constant: '<Root>/Constant2'
   *  Constant: '<Root>/Constant3'
   *  Constant: '<Root>/Constant4'
   *  Constant: '<Root>/Constant5'
   *  Constant: '<Root>/Constant6'
   *  Constant: '<Root>/Constant7'
   *  Constant: '<Root>/Constant8'
   */
  WriteToBSW(&rtb_f32StrokeFused, &rtb_f32EstimatedBias, &rtb_f32QuadResidual,
             &rtb_f32SkewResidual, &rtb_s16QuadFused, &ASW_ConstP.pooled1,
             &ASW_ConstP.pooled2, &ASW_ConstP.pooled2, &ASW_ConstP.pooled2,
             &ASW_ConstP.pooled2, &ASW_ConstP.pooled2, &ASW_ConstP.pooled1,
             &ASW_ConstP.pooled1, &rtb_SpeedRegulator, rty_G_ASW_OUT);
}

/*
 *   Function:    ASW_initialize
 *   Model name:  ASW
 *   Description: Model initialize function
 *   Arguments:   DW_ASW_fwu4_T *localDW
 *   Return:      void
 */
void ASW_initialize(DW_ASW_fwu4_T *localDW)
{
  /* Registration code */

  /* states (dwork) */
  (void) memset((void *)localDW, 0,
                sizeof(DW_ASW_fwu4_T));

  /* Model Initialize function for ModelReference Block: '<Root>/Sensor Fusion' */
  SensorFusion_initialize(&(localDW->SensorFusion_InstanceData.rtdw));

  /* Model Initialize function for ModelReference Block: '<Root>/Speed Regulator' */
  PID_Loop_initialize(&(localDW->SpeedRegulator_InstanceData.rtdw));
}

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
