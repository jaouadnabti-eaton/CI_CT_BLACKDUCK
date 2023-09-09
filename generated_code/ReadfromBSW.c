/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        ReadfromBSW.c
 *   Module Name: ReadfromBSW
 *   Module Ver:  10.19
 *   Date:        Thu Jul 20 13:12:15 2023
 */

#include "ReadfromBSW.h"
#include "ASW_BSW_I.h"
#include "typedefs.h"

/*
 *   Function:    ReadfromBSW
 *   Model name:  ReadfromBSW
 *   Description: Output and update for referenced model: 'ReadfromBSW'
 *   Arguments:   const BSW_DATA_t *rtu_G_BSW_DATA
 *                float32_t *rty_f32CurrentPhaseA
 *                float32_t *rty_f32CurrentPhaseB
 *                float32_t *rty_f32CurrentPhaseC
 *                float32_t *rty_f32VoltSense5V
 *                float32_t *rty_f32VoltSensePos15V
 *                float32_t *rty_f32VoltSenseNeg15V
 *                float32_t *rty_f32MotorTempPhaseA
 *                float32_t *rty_f32MotorTempPhaseB
 *                float32_t *rty_f32MotorTempPhaseC
 *                eLruId_t *rty_eLRU_ID
 *                eActId_t *rty_eCH_ID
 *                eActuator_t *rty_eACTUATOR_ID
 *                uint16_t *rty_MOTOR_ENABLE
 *                bool_t *rty_CHX_STATUS
 *                int16_t *rty_s16QuadPosition
 *                float32_t *rty_f32StrokeRvdt
 *                float32_t *rty_f32StrokeFused
 *                float32_t *rty_f32FlapAngleRvdt
 *                float32_t *rty_f32FlapAngleQuad
 *                float32_t *rty_f32StrokeQuad
 *                float32_t *rty_f32IAvrMax
 *                float32_t *rty_f32RefSpeed
 *                float32_t *rty_f32FdbSpeed
 *                bool_t *rty_bVcModeCmd
 *                bool_t *rty_bSensorFusionEnableCmd
 *                int16_t *rty_Rst_Int_Gain
 *                uint16_t *rty_xChannel_Enable_M
 *   Return:      void
 */
void ReadfromBSW(const BSW_DATA_t *rtu_G_BSW_DATA, float32_t
                 *rty_f32CurrentPhaseA, float32_t *rty_f32CurrentPhaseB,
                 float32_t *rty_f32CurrentPhaseC, float32_t *rty_f32VoltSense5V,
                 float32_t *rty_f32VoltSensePos15V, float32_t
                 *rty_f32VoltSenseNeg15V, float32_t *rty_f32MotorTempPhaseA,
                 float32_t *rty_f32MotorTempPhaseB, float32_t
                 *rty_f32MotorTempPhaseC, eLruId_t *rty_eLRU_ID, eActId_t
                 *rty_eCH_ID, eActuator_t *rty_eACTUATOR_ID, uint16_t
                 *rty_MOTOR_ENABLE, bool_t *rty_CHX_STATUS, int16_t
                 *rty_s16QuadPosition, float32_t *rty_f32StrokeRvdt, float32_t
                 *rty_f32StrokeFused, float32_t *rty_f32FlapAngleRvdt, float32_t
                 *rty_f32FlapAngleQuad, float32_t *rty_f32StrokeQuad, float32_t *
                 rty_f32IAvrMax, float32_t *rty_f32RefSpeed, float32_t
                 *rty_f32FdbSpeed, bool_t *rty_bVcModeCmd, bool_t
                 *rty_bSensorFusionEnableCmd, int16_t *rty_Rst_Int_Gain,
                 uint16_t *rty_xChannel_Enable_M)
{
  /* SignalConversion: '<Root>/Signal Conversion' */
  *rty_f32CurrentPhaseA = rtu_G_BSW_DATA->tAnalogInputData.f32_I_PHA_CHA;

  /* SignalConversion: '<Root>/Signal Conversion1' */
  *rty_MOTOR_ENABLE = rtu_G_BSW_DATA->tDiscreteInputData.u16SCU_MCU_ENABLE;

  /* SignalConversion: '<Root>/Signal Conversion10' */
  *rty_f32StrokeQuad = rtu_G_BSW_DATA->tPanelInputData.f32StrokeQuad;

  /* SignalConversion: '<Root>/Signal Conversion11' */
  *rty_f32CurrentPhaseB = rtu_G_BSW_DATA->tAnalogInputData.f32_I_PHB_CHA;

  /* SignalConversion: '<Root>/Signal Conversion12' */
  *rty_f32CurrentPhaseC = rtu_G_BSW_DATA->tAnalogInputData.f32_I_PHC_CHA;

  /* SignalConversion: '<Root>/Signal Conversion13' */
  *rty_f32VoltSense5V = rtu_G_BSW_DATA->tAnalogInputData.f32_5V_SENSE;

  /* SignalConversion: '<Root>/Signal Conversion14' */
  *rty_f32VoltSensePos15V =
    rtu_G_BSW_DATA->tAnalogInputData.f32_POSITIVE_15V_SENSE;

  /* SignalConversion: '<Root>/Signal Conversion15' */
  *rty_f32VoltSenseNeg15V =
    rtu_G_BSW_DATA->tAnalogInputData.f32_NEGATIVE_15V_SENSE;

  /* SignalConversion: '<Root>/Signal Conversion16' */
  *rty_f32MotorTempPhaseA = rtu_G_BSW_DATA->tAnalogInputData.f32_MOTOR_TEMP_PHA;

  /* SignalConversion: '<Root>/Signal Conversion17' */
  *rty_f32MotorTempPhaseB = rtu_G_BSW_DATA->tAnalogInputData.f32_MOTOR_TEMP_PHB;

  /* SignalConversion: '<Root>/Signal Conversion18' */
  *rty_f32MotorTempPhaseC = rtu_G_BSW_DATA->tAnalogInputData.f32_MOTOR_TEMP_PHC;

  /* SignalConversion: '<Root>/Signal Conversion19' */
  *rty_CHX_STATUS = rtu_G_BSW_DATA->tDiscreteInputData.bCHX_STATUS;

  /* SignalConversion: '<Root>/Signal Conversion2' */
  *rty_s16QuadPosition = rtu_G_BSW_DATA->tPanelInputData.s16QuadPosition;

  /* SignalConversion: '<Root>/Signal Conversion20' */
  *rty_f32IAvrMax = rtu_G_BSW_DATA->tAnalogInputData.f32_I_AVR_MAX;

  /* SignalConversion: '<Root>/Signal Conversion21' */
  *rty_f32RefSpeed = rtu_G_BSW_DATA->tAnalogInputData.f32SpeedRef;

  /* SignalConversion: '<Root>/Signal Conversion22' */
  *rty_f32FdbSpeed = rtu_G_BSW_DATA->tAnalogInputData.f32SpeedFdb;

  /* SignalConversion: '<Root>/Signal Conversion23' */
  *rty_bVcModeCmd = rtu_G_BSW_DATA->tDiscreteInputData.bVcModeCmd;

  /* SignalConversion: '<Root>/Signal Conversion24' */
  *rty_bSensorFusionEnableCmd =
    rtu_G_BSW_DATA->tDiscreteInputData.bSensorFusionEnableCmd;

  /* SignalConversion: '<Root>/Signal Conversion25' */
  *rty_Rst_Int_Gain = rtu_G_BSW_DATA->tDiscreteInputData.ResetIntegralGain;

  /* SignalConversion: '<Root>/Signal Conversion26' */
  *rty_xChannel_Enable_M = rtu_G_BSW_DATA->tDiscreteInputData.xChannelEnable_M;

  /* SignalConversion: '<Root>/Signal Conversion3' */
  *rty_eLRU_ID = rtu_G_BSW_DATA->tDiscreteInputData.eLruId;

  /* SignalConversion: '<Root>/Signal Conversion4' */
  *rty_eCH_ID = rtu_G_BSW_DATA->tDiscreteInputData.eChId;

  /* SignalConversion: '<Root>/Signal Conversion5' */
  *rty_f32StrokeRvdt = rtu_G_BSW_DATA->tPanelInputData.f32StrokeRvdt;

  /* SignalConversion: '<Root>/Signal Conversion6' */
  *rty_eACTUATOR_ID = rtu_G_BSW_DATA->tDiscreteInputData.eActuatorNumber;

  /* SignalConversion: '<Root>/Signal Conversion7' */
  *rty_f32StrokeFused = rtu_G_BSW_DATA->tPanelInputData.f32StrokeFused;

  /* SignalConversion: '<Root>/Signal Conversion8' */
  *rty_f32FlapAngleRvdt = rtu_G_BSW_DATA->tPanelInputData.f32FlapAngleRvdt;

  /* SignalConversion: '<Root>/Signal Conversion9' */
  *rty_f32FlapAngleQuad = rtu_G_BSW_DATA->tPanelInputData.f32FlapAngleQuad;
}

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
