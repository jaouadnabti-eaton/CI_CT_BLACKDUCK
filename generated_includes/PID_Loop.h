/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        PID_Loop.h
 *   Module Name: PID_Loop
 *   Module Ver:  10.275
 *   Date:        Thu Jul 20 13:12:05 2023
 */

#ifndef RTW_HEADER_PID_Loop_h_
#define RTW_HEADER_PID_Loop_h_
#ifndef PID_Loop_COMMON_INCLUDES_
#define PID_Loop_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* PID_Loop_COMMON_INCLUDES_ */

#include "typedefs.h"
#include "PID_Loop_types.h"

/* Block states (default storage) for model 'PID_Loop' */
typedef struct {
  float32_t SatErr_DSTATE;             /* '<S1>/SatErr' (DWork 1) */
  float32_t int_out_old_DSTATE;        /* '<S1>/int_out_old' (DWork 1) */
  float32_t pwm_max_old_DSTATE;        /* '<S1>/pwm_max_old' (DWork 1) */
} DW_PID_Loop_fwu4_T;

typedef struct {
  DW_PID_Loop_fwu4_T rtdw;
} MdlrefDW_PID_Loop_T;

/* Model reference registration function */
extern void PID_Loop_initialize(DW_PID_Loop_fwu4_T *localDW);
extern void PID_Loop_Init(DW_PID_Loop_fwu4_T *localDW);
extern void PID_Loop(const float32_t *rtu_Ref, const float32_t *rtu_Fdb, const
                     float32_t *rtu_I_Avr_Max, const bool_t *rtu_Vc_Mode, const
                     int16_t *rtu_Rst_Int_Gain, const uint16_t
                     *rtu_xChannel_Enable_M, float32_t *rty_SpeedPidOutput,
                     DW_PID_Loop_fwu4_T *localDW);

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
 * '<Root>' : 'PID_Loop'
 * '<S1>'   : 'PID_Loop/pid_control'
 */

/*-
 * Requirements for '<Root>': PID_Loop

 */
#endif                                 /* RTW_HEADER_PID_Loop_h_ */

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
