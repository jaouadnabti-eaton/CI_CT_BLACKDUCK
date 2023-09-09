/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        PID_Loop.c
 *   Module Name: PID_Loop
 *   Module Ver:  10.275
 *   Date:        Thu Jul 20 13:12:05 2023
 */

#include "PID_Loop.h"
#include "typedefs.h"
#include "rtwtypes.h"
#include "PID_Loop_private.h"

/*
 *   Function:    PID_Loop_Init
 *   Model name:  PID_Loop
 *   Description: System initialize for referenced model: 'PID_Loop'
 *   Arguments:   DW_PID_Loop_fwu4_T *localDW
 *   Return:      void
 */
void PID_Loop_Init(DW_PID_Loop_fwu4_T *localDW)
{
  /* InitializeConditions for Delay: '<S1>/pwm_max_old' */
  localDW->pwm_max_old_DSTATE = (float32_t)rtCP_pooled7;
}

/*
 *   Function:    PID_Loop
 *   Model name:  PID_Loop
 *   Description: Output and update for referenced model: 'PID_Loop'
 *   Arguments:   const float32_t *rtu_Ref
 *                const float32_t *rtu_Fdb
 *                const float32_t *rtu_I_Avr_Max
 *                const bool_t *rtu_Vc_Mode
 *                const int16_t *rtu_Rst_Int_Gain
 *                const uint16_t *rtu_xChannel_Enable_M
 *                float32_t *rty_SpeedPidOutput
 *                DW_PID_Loop_fwu4_T *localDW
 *   Return:      void
 */
void PID_Loop(const float32_t *rtu_Ref, const float32_t *rtu_Fdb, const
              float32_t *rtu_I_Avr_Max, const bool_t *rtu_Vc_Mode, const int16_t
              *rtu_Rst_Int_Gain, const uint16_t *rtu_xChannel_Enable_M,
              float32_t *rty_SpeedPidOutput, DW_PID_Loop_fwu4_T *localDW)
{
  float32_t rtb_Add2;
  float32_t rtb_Kc_out;
  float32_t rtb_Kp_out;
  float32_t rtb_Switch2;
  float32_t rtb_int_out_final;
  float32_t rtb_pwm_max;
  float32_t rtb_pwm_max2;
  float32_t rtb_pwm_step_change;
  float32_t rtb_ton_ff_pi;
  float32_t rtb_ton_out;
  bool_t rtb_GreaterThanOrEqual;

  /* Switch: '<S1>/Switch4' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Constant1'
   *  Logic: '<S1>/Logical Operator'
   *  RelationalOperator: '<S1>/Equal'
   *  RelationalOperator: '<S1>/NotEqual1'
   */
  if (((*rtu_xChannel_Enable_M) == rtCP_Constant_Value) && ((*rtu_Vc_Mode) ==
       rtCP_Constant1_Value)) {
    /* RelationalOperator: '<S1>/GreaterThanOrEqual' incorporates:
     *  Constant: '<S1>/a_o_speed_limit'
     */
    rtb_GreaterThanOrEqual = (bool_t)((uint16_T)(((*rtu_Ref) <=
      rtCP_a_o_speed_limit_Value) ? ((int16_T)1) : ((int16_T)0)));

    /* Switch: '<S1>/Switch6' incorporates:
     *  Constant: '<S1>/a_o_speed_limit'
     *  RelationalOperator: '<S1>/GreaterThanOrEqual'
     */
    if (rtb_GreaterThanOrEqual != 0U) {
      rtb_pwm_step_change = *rtu_Ref;
    } else {
      rtb_pwm_step_change = rtCP_a_o_speed_limit_Value;
    }

    /* End of Switch: '<S1>/Switch6' */
  } else {
    rtb_pwm_step_change = *rtu_Ref;
  }

  /* End of Switch: '<S1>/Switch4' */

  /* Sum: '<S1>/Add2' */
  rtb_Add2 = rtb_pwm_step_change - (*rtu_Fdb);

  /* Switch: '<S1>/Switch1' incorporates:
   *  Constant: '<S1>/hl_kp'
   *  Constant: '<S1>/vc_kp'
   */
  if ((*rtu_Vc_Mode) != 0U) {
    rtb_ton_out = rtCP_pooled4;
  } else {
    rtb_ton_out = rtCP_pooled4;
  }

  /* End of Switch: '<S1>/Switch1' */

  /* Product: '<S1>/Kp_out' */
  rtb_Kp_out = rtb_Add2 * rtb_ton_out;

  /* Switch: '<S1>/int_out_final' incorporates:
   *  Constant: '<S1>/integral_reset_flag'
   *  Delay: '<S1>/int_out_old'
   *  Product: '<S1>/Ki_out'
   *  Sum: '<S1>/int_out'
   */
  if ((*rtu_Rst_Int_Gain) != 0) {
    /* Switch: '<S1>/Switch3' incorporates:
     *  Constant: '<S1>/hl_kc'
     *  Constant: '<S1>/vc_kc'
     */
    if ((*rtu_Vc_Mode) != 0U) {
      rtb_Switch2 = rtCP_pooled1;
    } else {
      rtb_Switch2 = rtCP_pooled1;
    }

    /* End of Switch: '<S1>/Switch3' */

    /* Product: '<S1>/Kc_out' incorporates:
     *  Delay: '<S1>/SatErr'
     */
    rtb_Kc_out = rtb_Switch2 * localDW->SatErr_DSTATE;

    /* Switch: '<S1>/Switch2' incorporates:
     *  Constant: '<S1>/hl_ki'
     *  Constant: '<S1>/vc_ki'
     */
    if ((*rtu_Vc_Mode) != 0U) {
      rtb_Switch2 = rtCP_pooled3;
    } else {
      rtb_Switch2 = rtCP_pooled3;
    }

    /* End of Switch: '<S1>/Switch2' */
    rtb_int_out_final = ((rtb_Kp_out * rtb_Switch2) + rtb_Kc_out) +
      localDW->int_out_old_DSTATE;
  } else {
    rtb_int_out_final = (float32_t)rtCP_pooled6;
  }

  /* End of Switch: '<S1>/int_out_final' */

  /* Switch: '<S1>/Switch' incorporates:
   *  Constant: '<S1>/hl_kf'
   *  Constant: '<S1>/vc_kf'
   */
  if ((*rtu_Vc_Mode) != 0U) {
    rtb_ton_out = rtCP_pooled2;
  } else {
    rtb_ton_out = rtCP_pooled2;
  }

  /* End of Switch: '<S1>/Switch' */

  /* Sum: '<S1>/ton_ff_pi' incorporates:
   *  Product: '<S1>/Kf_out'
   */
  rtb_ton_ff_pi = ((rtb_pwm_step_change * rtb_ton_out) + rtb_Kp_out) +
    rtb_int_out_final;

  /* Switch: '<S1>/pwm_step_change' incorporates:
   *  Constant: '<S1>/pwm_decrease_step'
   *  Constant: '<S1>/pwm_increase_step'
   */
  if ((*rtu_I_Avr_Max) > rtCP_pwm_step_change_Threshold) {
    rtb_pwm_step_change = rtCP_pwm_decrease_step_Value;
  } else {
    rtb_pwm_step_change = rtCP_pwm_increase_step_Value;
  }

  /* End of Switch: '<S1>/pwm_step_change' */

  /* Sum: '<S1>/pwm_max' incorporates:
   *  Delay: '<S1>/pwm_max_old'
   */
  rtb_pwm_max = localDW->pwm_max_old_DSTATE + rtb_pwm_step_change;

  /* Switch: '<S1>/pwm_max2' incorporates:
   *  Constant: '<S1>/ton_pwm_max'
   *  Constant: '<S1>/ton_pwm_min1'
   *  RelationalOperator: '<S1>/GreaterThan3'
   *  Switch: '<S1>/pwm_max1'
   */
  if (rtb_pwm_max > ((float32_t)rtCP_pooled7)) {
    rtb_pwm_max2 = (float32_t)rtCP_pooled7;
  } else if (rtb_pwm_max < rtCP_pooled5) {
    /* Switch: '<S1>/pwm_max1' incorporates:
     *  Constant: '<S1>/ton_pwm_min'
     */
    rtb_pwm_max2 = rtCP_pooled5;
  } else {
    /* Switch: '<S1>/pwm_max1' */
    rtb_pwm_max2 = rtb_pwm_max;
  }

  /* End of Switch: '<S1>/pwm_max2' */

  /* Switch: '<S1>/ton_out' incorporates:
   *  Constant: '<S1>/ton_min'
   *  RelationalOperator: '<S1>/GreaterThan1'
   *  RelationalOperator: '<S1>/GreaterThan2'
   *  Switch: '<S1>/ton_min_check'
   */
  if (rtb_ton_ff_pi > rtb_pwm_max2) {
    rtb_ton_out = rtb_pwm_max2;
  } else if (rtb_ton_ff_pi < ((float32_t)rtCP_pooled6)) {
    /* Switch: '<S1>/ton_min_check' incorporates:
     *  Constant: '<S1>/ton_min'
     */
    rtb_ton_out = (float32_t)rtCP_pooled6;
  } else {
    /* Switch: '<S1>/ton_min_check' */
    rtb_ton_out = rtb_ton_ff_pi;
  }

  /* End of Switch: '<S1>/ton_out' */

  /* Switch: '<S1>/ton_final' incorporates:
   *  RelationalOperator: '<S1>/GreaterThan'
   */
  if (rtb_ton_out > rtb_pwm_max2) {
    *rty_SpeedPidOutput = rtb_pwm_max2;
  } else {
    *rty_SpeedPidOutput = rtb_ton_out;
  }

  /* End of Switch: '<S1>/ton_final' */

  /* Update for Delay: '<S1>/SatErr' incorporates:
   *  Sum: '<S1>/Add3'
   */
  localDW->SatErr_DSTATE = rtb_ton_out - rtb_ton_ff_pi;

  /* Update for Delay: '<S1>/int_out_old' */
  localDW->int_out_old_DSTATE = rtb_int_out_final;

  /* Update for Delay: '<S1>/pwm_max_old' */
  localDW->pwm_max_old_DSTATE = rtb_pwm_max2;
}

/*
 *   Function:    PID_Loop_initialize
 *   Model name:  PID_Loop
 *   Description: Model initialize function
 *   Arguments:   DW_PID_Loop_fwu4_T *localDW
 *   Return:      void
 */
void PID_Loop_initialize(DW_PID_Loop_fwu4_T *localDW)
{
  /* Registration code */

  /* states (dwork) */
  localDW->SatErr_DSTATE = 0.0F;
  localDW->int_out_old_DSTATE = 0.0F;
  localDW->pwm_max_old_DSTATE = 0.0F;
}

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
