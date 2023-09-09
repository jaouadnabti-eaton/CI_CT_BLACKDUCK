/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        WriteToBSW.c
 *   Module Name: WriteToBSW
 *   Module Ver:  10.16
 *   Date:        Thu Jul 20 13:12:28 2023
 */

#include "WriteToBSW.h"
#include "typedefs.h"
#include "ASW_BSW_I.h"

/*
 *   Function:    WriteToBSW
 *   Model name:  WriteToBSW
 *   Description: Output and update for referenced model: 'WriteToBSW'
 *   Arguments:   const float32_t *rtu_f32StrokeFused
 *                const float32_t *rtu_f32BiasFused
 *                const float32_t *rtu_f32QuadCntResidualFused
 *                const float32_t *rtu_f32SkewResidualFused
 *                const int16_t *rtu_s16QuadFused
 *                const uint16_t *rtu_u16MemoryPad
 *                const bool_t *rtu_bVC_MODE
 *                const bool_t *rtu_b270V_BUS_CNTL
 *                const bool_t *rtu_b28V_BUS_CNTL
 *                const bool_t *rtu_bINRUSH_CTR
 *                const bool_t *rtu_bBUS_SW_ENABLE
 *                const uint16_t *rtu_ANALOG_OUT_A
 *                const uint16_t *rtu_ANALOG_OUT_B
 *                const float32_t *rtu_f32SpeedPidOutput
 *                ASW_DATA_t *rty_G_ASW_DATA
 *   Return:      void
 */
void WriteToBSW(const float32_t *rtu_f32StrokeFused, const float32_t
                *rtu_f32BiasFused, const float32_t *rtu_f32QuadCntResidualFused,
                const float32_t *rtu_f32SkewResidualFused, const int16_t
                *rtu_s16QuadFused, const uint16_t *rtu_u16MemoryPad, const
                bool_t *rtu_bVC_MODE, const bool_t *rtu_b270V_BUS_CNTL, const
                bool_t *rtu_b28V_BUS_CNTL, const bool_t *rtu_bINRUSH_CTR, const
                bool_t *rtu_bBUS_SW_ENABLE, const uint16_t *rtu_ANALOG_OUT_A,
                const uint16_t *rtu_ANALOG_OUT_B, const float32_t
                *rtu_f32SpeedPidOutput, ASW_DATA_t *rty_G_ASW_DATA)
{
  /* BusCreator: '<Root>/Bus Creator' */
  rty_G_ASW_DATA->tAnalogOutputData.ANALOG_OUT_A = *rtu_ANALOG_OUT_A;
  rty_G_ASW_DATA->tAnalogOutputData.ANALOG_OUT_B = *rtu_ANALOG_OUT_B;
  rty_G_ASW_DATA->tAnalogOutputData.f32SpeedPidOutput = *rtu_f32SpeedPidOutput;

  /* BusCreator: '<Root>/Bus Creator2' */
  rty_G_ASW_DATA->tDiscreteOutputData.bVC_MODE = *rtu_bVC_MODE;
  rty_G_ASW_DATA->tDiscreteOutputData.b270V_BUS_CNTL = *rtu_b270V_BUS_CNTL;
  rty_G_ASW_DATA->tDiscreteOutputData.b28V_BUS_CNTL = *rtu_b28V_BUS_CNTL;
  rty_G_ASW_DATA->tDiscreteOutputData.bINRUSH_CTR = *rtu_bINRUSH_CTR;
  rty_G_ASW_DATA->tDiscreteOutputData.bBUS_SW_ENABLE = *rtu_bBUS_SW_ENABLE;

  /* BusCreator: '<Root>/Bus Creator3' */
  rty_G_ASW_DATA->tPanelOutputData.f32StrokeFused = *rtu_f32StrokeFused;
  rty_G_ASW_DATA->tPanelOutputData.f32BiasFused = *rtu_f32BiasFused;
  rty_G_ASW_DATA->tPanelOutputData.f32QuadCntResidualFused =
    *rtu_f32QuadCntResidualFused;
  rty_G_ASW_DATA->tPanelOutputData.f32SkewResidualFused =
    *rtu_f32SkewResidualFused;
  rty_G_ASW_DATA->tPanelOutputData.s16QuadFused = *rtu_s16QuadFused;
  rty_G_ASW_DATA->tPanelOutputData.u16MemoryPad = *rtu_u16MemoryPad;
}

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
