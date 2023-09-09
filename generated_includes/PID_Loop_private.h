/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        PID_Loop_private.h
 *   Module Name: PID_Loop
 *   Module Ver:  10.275
 *   Date:        Thu Jul 20 13:12:05 2023
 */

#ifndef RTW_HEADER_PID_Loop_private_h_
#define RTW_HEADER_PID_Loop_private_h_
#include "rtwtypes.h"
#include "typedefs.h"
#include "PID_Loop_types.h"

/* Expression: a_o_speed_limit
 * Referenced by: '<S1>/a_o_speed_limit' (Parameter: Value)
 */
#define rtCP_a_o_speed_limit_Value     (1.85771406F)

/* Pooled Parameter (Mixed Expressions)
 * Referenced by:
 *   '<S1>/hl_kc' (Parameter: Value)
 *   '<S1>/vc_kc' (Parameter: Value)
 */
#define rtCP_pooled1                   (0.2F)

/* Pooled Parameter (Mixed Expressions)
 * Referenced by:
 *   '<S1>/hl_kf' (Parameter: Value)
 *   '<S1>/vc_kf' (Parameter: Value)
 */
#define rtCP_pooled2                   (0.009656F)

/* Pooled Parameter (Mixed Expressions)
 * Referenced by:
 *   '<S1>/hl_ki' (Parameter: Value)
 *   '<S1>/vc_ki' (Parameter: Value)
 */
#define rtCP_pooled3                   (0.02F)

/* Pooled Parameter (Mixed Expressions)
 * Referenced by:
 *   '<S1>/hl_kp' (Parameter: Value)
 *   '<S1>/vc_kp' (Parameter: Value)
 */
#define rtCP_pooled4                   (0.3F)

/* Expression: pwm_decrease_step
 * Referenced by: '<S1>/pwm_decrease_step' (Parameter: Value)
 */
#define rtCP_pwm_decrease_step_Value   (-0.0025F)

/* Expression: pwm_increase_step
 * Referenced by: '<S1>/pwm_increase_step' (Parameter: Value)
 */
#define rtCP_pwm_increase_step_Value   (0.00125F)

/* Pooled Parameter (Expression: ton_pwm_min)
 * Referenced by:
 *   '<S1>/ton_pwm_min' (Parameter: Value)
 *   '<S1>/ton_pwm_min1' (Parameter: Value)
 */
#define rtCP_pooled5                   (0.01F)

/* Pooled Parameter (Mixed Expressions)
 * Referenced by:
 *   '<S1>/integral_reset_flag' (Parameter: Value)
 *   '<S1>/ton_min' (Parameter: Value)
 *   '<S1>/SatErr' (Parameter: InitialCondition)
 *   '<S1>/int_out_old' (Parameter: InitialCondition)
 */
#define rtCP_pooled6                   (0.0F)

/* Pooled Parameter (Expression: ton_pwm_max)
 * Referenced by:
 *   '<S1>/ton_pwm_max' (Parameter: Value)
 *   '<S1>/pwm_max2' (Parameter: Threshold)
 *   '<S1>/pwm_max_old' (Parameter: InitialCondition)
 */
#define rtCP_pooled7                   (0.99F)

/* Expression: max_current_limit
 * Referenced by: '<S1>/pwm_step_change' (Parameter: Threshold)
 */
#define rtCP_pwm_step_change_Threshold (5.0F)

/* Computed Parameter: Constant_Value
 * Referenced by: '<S1>/Constant' (Parameter: Value)
 */
#define rtCP_Constant_Value            (1U)

/* Computed Parameter: Constant1_Value
 * Referenced by: '<S1>/Constant1' (Parameter: Value)
 */
#define rtCP_Constant1_Value           (((uint8_T)0U))
#endif                                 /* RTW_HEADER_PID_Loop_private_h_ */

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
