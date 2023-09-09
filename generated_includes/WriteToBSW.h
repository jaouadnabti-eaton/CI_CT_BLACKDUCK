/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        WriteToBSW.h
 *   Module Name: WriteToBSW
 *   Module Ver:  10.16
 *   Date:        Thu Jul 20 13:12:28 2023
 */

#ifndef RTW_HEADER_WriteToBSW_h_
#define RTW_HEADER_WriteToBSW_h_
#ifndef WriteToBSW_COMMON_INCLUDES_
#define WriteToBSW_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* WriteToBSW_COMMON_INCLUDES_ */

#include "typedefs.h"
#include "ASW_BSW_I.h"
#include "WriteToBSW_types.h"

extern void WriteToBSW(const float32_t *rtu_f32StrokeFused, const float32_t
  *rtu_f32BiasFused, const float32_t *rtu_f32QuadCntResidualFused, const
  float32_t *rtu_f32SkewResidualFused, const int16_t *rtu_s16QuadFused, const
  uint16_t *rtu_u16MemoryPad, const bool_t *rtu_bVC_MODE, const bool_t
  *rtu_b270V_BUS_CNTL, const bool_t *rtu_b28V_BUS_CNTL, const bool_t
  *rtu_bINRUSH_CTR, const bool_t *rtu_bBUS_SW_ENABLE, const uint16_t
  *rtu_ANALOG_OUT_A, const uint16_t *rtu_ANALOG_OUT_B, const float32_t
  *rtu_f32SpeedPidOutput, ASW_DATA_t *rty_G_ASW_DATA);

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
 * '<Root>' : 'WriteToBSW'
 */

/*-
 * Requirements for '<Root>': WriteToBSW

 */
#endif                                 /* RTW_HEADER_WriteToBSW_h_ */

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
