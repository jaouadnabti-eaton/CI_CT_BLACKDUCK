/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        SensorFusion.h
 *   Module Name: SensorFusion
 *   Module Ver:  10.10
 *   Date:        Thu Jul 20 13:12:22 2023
 */

#ifndef RTW_HEADER_SensorFusion_h_
#define RTW_HEADER_SensorFusion_h_
#ifndef SensorFusion_COMMON_INCLUDES_
#define SensorFusion_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* SensorFusion_COMMON_INCLUDES_ */

#include "typedefs.h"
#include "SensorFusion_types.h"

/* Block states (default storage) for model 'SensorFusion' */
typedef struct {
  real32_T Delay1_DSTATE;              /* '<S1>/Delay1' (DWork 1) */
  real32_T Delay2_DSTATE[16];          /* '<S1>/Delay2' (DWork 1) */
  real32_T Delay3_DSTATE;              /* '<S1>/Delay3' (DWork 1) */
  real32_T Delay4_DSTATE[4];           /* '<S1>/Delay4' (DWork 1) */
} DW_SensorFusion_fwu4_T;

/* Invariant block signals for model 'SensorFusion' */
typedef struct {
  const real32_T CT[8];                /* '<S1>/CT' (Output 1) */
} ConstB_SensorFusion_hb4t_T;

typedef struct {
  DW_SensorFusion_fwu4_T rtdw;
} MdlrefDW_SensorFusion_T;

/* Model reference registration function */
extern void SensorFusion_initialize(DW_SensorFusion_fwu4_T *localDW);
extern void SensorFusion(const int16_t *rtu_s16MotorQuadCount, const float32_t
  *rtu_f32SkewStroke, const float32_t *rtu_f32InitialPosition, const bool_t
  *rtu_bSensorFusionEnable, real32_T *rty_f32EstimatedPosition, real32_T
  *rty_f32EstimatedBias, real32_T *rty_f32QuadResidual, real32_T
  *rty_f32SkewResidual, int16_t *rty_s16EstimatedQuadCount,
  DW_SensorFusion_fwu4_T *localDW);

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
 * '<Root>' : 'SensorFusion'
 * '<S1>'   : 'SensorFusion/KalmanFilter'
 * '<S2>'   : 'SensorFusion/KalmanFilter/inv_C_x_Pkm_x_CT_p_R'
 */

/*-
 * Requirements for '<Root>': SensorFusion

 */
#endif                                 /* RTW_HEADER_SensorFusion_h_ */

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
