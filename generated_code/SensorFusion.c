/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        SensorFusion.c
 *   Module Name: SensorFusion
 *   Module Ver:  10.10
 *   Date:        Thu Jul 20 13:12:22 2023
 */

#include "SensorFusion.h"
#include "typedefs.h"
#include "rtwtypes.h"
#include <math.h>
#include "SensorFusion_private.h"

/*
 *   Function:    SensorFusion
 *   Model name:  SensorFusion
 *   Description: Output and update for referenced model: 'SensorFusion'
 *   Arguments:   const int16_t *rtu_s16MotorQuadCount
 *                const float32_t *rtu_f32SkewStroke
 *                const float32_t *rtu_f32InitialPosition
 *                const bool_t *rtu_bSensorFusionEnable
 *                real32_T *rty_f32EstimatedPosition
 *                real32_T *rty_f32EstimatedBias
 *                real32_T *rty_f32QuadResidual
 *                real32_T *rty_f32SkewResidual
 *                int16_t *rty_s16EstimatedQuadCount
 *                DW_SensorFusion_fwu4_T *localDW
 *   Return:      void
 */
void SensorFusion(const int16_t *rtu_s16MotorQuadCount, const float32_t
                  *rtu_f32SkewStroke, const float32_t *rtu_f32InitialPosition,
                  const bool_t *rtu_bSensorFusionEnable, real32_T
                  *rty_f32EstimatedPosition, real32_T *rty_f32EstimatedBias,
                  real32_T *rty_f32QuadResidual, real32_T *rty_f32SkewResidual,
                  int16_t *rty_s16EstimatedQuadCount, DW_SensorFusion_fwu4_T
                  *localDW)
{
  real32_T rtb_A_x_Pkm1_x_AT_p_Q[16];
  real32_T tmp[16];
  real32_T tmp_1[16];
  real32_T rtb_A_x_Pkm1_x_AT_p_Q_0[8];
  real32_T rtb_calcKalmanGain[8];
  real32_T tmp_0[8];
  real32_T rtb_reshape_2x2[4];
  real32_T rtb_xhat[4];
  real32_T rtb_xhat_km[4];
  real32_T rtb_xhat_tmp[2];
  real32_T rtb_A_x_Pkm1_x_AT_p_Q_1;
  real32_T rtb_Kk_x_yk_m_C_x_xhatkm_idx_0;
  real32_T rtb_Kk_x_yk_m_C_x_xhatkm_idx_1;
  real32_T rtb_Kk_x_yk_m_C_x_xhatkm_idx_2;
  real32_T rtb_Kk_x_yk_m_C_x_xhatkm_idx_3;
  real32_T rtb_Select_a;
  real32_T rtb_Select_b;
  real32_T rtb_Select_c;
  real32_T rtb_Select_d;
  real32_T rtb_StrokeToQuadCount;
  real32_T rtb_calcKalmanGain_tmp;
  real32_T rtb_calcKalmanGain_tmp_0;
  real32_T rtb_calcKalmanGain_tmp_1;
  real32_T rtb_calcKalmanGain_tmp_2;
  real32_T rtb_convQuad_f32;
  real32_T rtb_reshape_y_2x1_idx_1;
  real32_T rtb_xhat_km_jaf4;
  real32_T tmp_2;
  real32_T tmp_3;

  /* Switch: '<S1>/Switch1' incorporates:
   *  Constant: '<S1>/Constant6'
   *  Delay: '<S1>/Delay1'
   *  Delay: '<S1>/Delay2'
   *  Sum: '<S1>/A_x_Pkm1_x_AT_p_Q'
   */
  if (localDW->Delay1_DSTATE > rtCP_pooled6) {
    rtb_A_x_Pkm1_x_AT_p_Q[0] = localDW->Delay2_DSTATE[0];
    rtb_A_x_Pkm1_x_AT_p_Q[1] = localDW->Delay2_DSTATE[1];
    rtb_A_x_Pkm1_x_AT_p_Q[2] = localDW->Delay2_DSTATE[2];
    rtb_A_x_Pkm1_x_AT_p_Q[3] = localDW->Delay2_DSTATE[3];
    rtb_A_x_Pkm1_x_AT_p_Q[4] = localDW->Delay2_DSTATE[4];
    rtb_A_x_Pkm1_x_AT_p_Q[5] = localDW->Delay2_DSTATE[5];
    rtb_A_x_Pkm1_x_AT_p_Q[6] = localDW->Delay2_DSTATE[6];
    rtb_A_x_Pkm1_x_AT_p_Q[7] = localDW->Delay2_DSTATE[7];
    rtb_A_x_Pkm1_x_AT_p_Q[8] = localDW->Delay2_DSTATE[8];
    rtb_A_x_Pkm1_x_AT_p_Q[9] = localDW->Delay2_DSTATE[9];
    rtb_A_x_Pkm1_x_AT_p_Q[10] = localDW->Delay2_DSTATE[10];
    rtb_A_x_Pkm1_x_AT_p_Q[11] = localDW->Delay2_DSTATE[11];
    rtb_A_x_Pkm1_x_AT_p_Q[12] = localDW->Delay2_DSTATE[12];
    rtb_A_x_Pkm1_x_AT_p_Q[13] = localDW->Delay2_DSTATE[13];
    rtb_A_x_Pkm1_x_AT_p_Q[14] = localDW->Delay2_DSTATE[14];
    rtb_A_x_Pkm1_x_AT_p_Q[15] = localDW->Delay2_DSTATE[15];
  } else {
    rtb_A_x_Pkm1_x_AT_p_Q[0] = rtCP_Constant6_Value_EL_0;
    rtb_A_x_Pkm1_x_AT_p_Q[1] = rtCP_Constant6_Value_EL_1;
    rtb_A_x_Pkm1_x_AT_p_Q[2] = rtCP_Constant6_Value_EL_2;
    rtb_A_x_Pkm1_x_AT_p_Q[3] = rtCP_Constant6_Value_EL_3;
    rtb_A_x_Pkm1_x_AT_p_Q[4] = rtCP_Constant6_Value_EL_4;
    rtb_A_x_Pkm1_x_AT_p_Q[5] = rtCP_Constant6_Value_EL_5;
    rtb_A_x_Pkm1_x_AT_p_Q[6] = rtCP_Constant6_Value_EL_6;
    rtb_A_x_Pkm1_x_AT_p_Q[7] = rtCP_Constant6_Value_EL_7;
    rtb_A_x_Pkm1_x_AT_p_Q[8] = rtCP_Constant6_Value_EL_8;
    rtb_A_x_Pkm1_x_AT_p_Q[9] = rtCP_Constant6_Value_EL_9;
    rtb_A_x_Pkm1_x_AT_p_Q[10] = rtCP_Constant6_Value_EL_10;
    rtb_A_x_Pkm1_x_AT_p_Q[11] = rtCP_Constant6_Value_EL_11;
    rtb_A_x_Pkm1_x_AT_p_Q[12] = rtCP_Constant6_Value_EL_12;
    rtb_A_x_Pkm1_x_AT_p_Q[13] = rtCP_Constant6_Value_EL_13;
    rtb_A_x_Pkm1_x_AT_p_Q[14] = rtCP_Constant6_Value_EL_14;
    rtb_A_x_Pkm1_x_AT_p_Q[15] = rtCP_Constant6_Value_EL_15;
  }

  /* End of Switch: '<S1>/Switch1' */

  /* Product: '<S1>/A_x_Pkm1' incorporates:
   *  Constant: '<S1>/Constant'
   *  Sum: '<S1>/A_x_Pkm1_x_AT_p_Q'
   */
  tmp[0] = (((rtCP_Constant_Value_EL_0 * rtb_A_x_Pkm1_x_AT_p_Q[0]) +
             (rtCP_Constant_Value_EL_4 * rtb_A_x_Pkm1_x_AT_p_Q[1])) +
            (rtCP_Constant_Value_EL_8 * rtb_A_x_Pkm1_x_AT_p_Q[2])) +
    (rtCP_Constant_Value_EL_12 * rtb_A_x_Pkm1_x_AT_p_Q[3]);
  tmp[4] = (((rtCP_Constant_Value_EL_0 * rtb_A_x_Pkm1_x_AT_p_Q[4]) +
             (rtCP_Constant_Value_EL_4 * rtb_A_x_Pkm1_x_AT_p_Q[5])) +
            (rtCP_Constant_Value_EL_8 * rtb_A_x_Pkm1_x_AT_p_Q[6])) +
    (rtCP_Constant_Value_EL_12 * rtb_A_x_Pkm1_x_AT_p_Q[7]);
  tmp[8] = (((rtCP_Constant_Value_EL_0 * rtb_A_x_Pkm1_x_AT_p_Q[8]) +
             (rtCP_Constant_Value_EL_4 * rtb_A_x_Pkm1_x_AT_p_Q[9])) +
            (rtCP_Constant_Value_EL_8 * rtb_A_x_Pkm1_x_AT_p_Q[10])) +
    (rtCP_Constant_Value_EL_12 * rtb_A_x_Pkm1_x_AT_p_Q[11]);
  tmp[12] = (((rtCP_Constant_Value_EL_0 * rtb_A_x_Pkm1_x_AT_p_Q[12]) +
              (rtCP_Constant_Value_EL_4 * rtb_A_x_Pkm1_x_AT_p_Q[13])) +
             (rtCP_Constant_Value_EL_8 * rtb_A_x_Pkm1_x_AT_p_Q[14])) +
    (rtCP_Constant_Value_EL_12 * rtb_A_x_Pkm1_x_AT_p_Q[15]);
  tmp[1] = (((rtCP_Constant_Value_EL_1 * rtb_A_x_Pkm1_x_AT_p_Q[0]) +
             (rtCP_Constant_Value_EL_5 * rtb_A_x_Pkm1_x_AT_p_Q[1])) +
            (rtCP_Constant_Value_EL_9 * rtb_A_x_Pkm1_x_AT_p_Q[2])) +
    (rtCP_Constant_Value_EL_13 * rtb_A_x_Pkm1_x_AT_p_Q[3]);
  tmp[5] = (((rtCP_Constant_Value_EL_1 * rtb_A_x_Pkm1_x_AT_p_Q[4]) +
             (rtCP_Constant_Value_EL_5 * rtb_A_x_Pkm1_x_AT_p_Q[5])) +
            (rtCP_Constant_Value_EL_9 * rtb_A_x_Pkm1_x_AT_p_Q[6])) +
    (rtCP_Constant_Value_EL_13 * rtb_A_x_Pkm1_x_AT_p_Q[7]);
  tmp[9] = (((rtCP_Constant_Value_EL_1 * rtb_A_x_Pkm1_x_AT_p_Q[8]) +
             (rtCP_Constant_Value_EL_5 * rtb_A_x_Pkm1_x_AT_p_Q[9])) +
            (rtCP_Constant_Value_EL_9 * rtb_A_x_Pkm1_x_AT_p_Q[10])) +
    (rtCP_Constant_Value_EL_13 * rtb_A_x_Pkm1_x_AT_p_Q[11]);
  tmp[13] = (((rtCP_Constant_Value_EL_1 * rtb_A_x_Pkm1_x_AT_p_Q[12]) +
              (rtCP_Constant_Value_EL_5 * rtb_A_x_Pkm1_x_AT_p_Q[13])) +
             (rtCP_Constant_Value_EL_9 * rtb_A_x_Pkm1_x_AT_p_Q[14])) +
    (rtCP_Constant_Value_EL_13 * rtb_A_x_Pkm1_x_AT_p_Q[15]);
  tmp[2] = (((rtCP_Constant_Value_EL_2 * rtb_A_x_Pkm1_x_AT_p_Q[0]) +
             (rtCP_Constant_Value_EL_6 * rtb_A_x_Pkm1_x_AT_p_Q[1])) +
            (rtCP_Constant_Value_EL_10 * rtb_A_x_Pkm1_x_AT_p_Q[2])) +
    (rtCP_Constant_Value_EL_14 * rtb_A_x_Pkm1_x_AT_p_Q[3]);
  tmp[6] = (((rtCP_Constant_Value_EL_2 * rtb_A_x_Pkm1_x_AT_p_Q[4]) +
             (rtCP_Constant_Value_EL_6 * rtb_A_x_Pkm1_x_AT_p_Q[5])) +
            (rtCP_Constant_Value_EL_10 * rtb_A_x_Pkm1_x_AT_p_Q[6])) +
    (rtCP_Constant_Value_EL_14 * rtb_A_x_Pkm1_x_AT_p_Q[7]);
  tmp[10] = (((rtCP_Constant_Value_EL_2 * rtb_A_x_Pkm1_x_AT_p_Q[8]) +
              (rtCP_Constant_Value_EL_6 * rtb_A_x_Pkm1_x_AT_p_Q[9])) +
             (rtCP_Constant_Value_EL_10 * rtb_A_x_Pkm1_x_AT_p_Q[10])) +
    (rtCP_Constant_Value_EL_14 * rtb_A_x_Pkm1_x_AT_p_Q[11]);
  tmp[14] = (((rtCP_Constant_Value_EL_2 * rtb_A_x_Pkm1_x_AT_p_Q[12]) +
              (rtCP_Constant_Value_EL_6 * rtb_A_x_Pkm1_x_AT_p_Q[13])) +
             (rtCP_Constant_Value_EL_10 * rtb_A_x_Pkm1_x_AT_p_Q[14])) +
    (rtCP_Constant_Value_EL_14 * rtb_A_x_Pkm1_x_AT_p_Q[15]);
  tmp[3] = (((rtCP_Constant_Value_EL_3 * rtb_A_x_Pkm1_x_AT_p_Q[0]) +
             (rtCP_Constant_Value_EL_7 * rtb_A_x_Pkm1_x_AT_p_Q[1])) +
            (rtCP_Constant_Value_EL_11 * rtb_A_x_Pkm1_x_AT_p_Q[2])) +
    (rtCP_Constant_Value_EL_15 * rtb_A_x_Pkm1_x_AT_p_Q[3]);
  tmp[7] = (((rtCP_Constant_Value_EL_3 * rtb_A_x_Pkm1_x_AT_p_Q[4]) +
             (rtCP_Constant_Value_EL_7 * rtb_A_x_Pkm1_x_AT_p_Q[5])) +
            (rtCP_Constant_Value_EL_11 * rtb_A_x_Pkm1_x_AT_p_Q[6])) +
    (rtCP_Constant_Value_EL_15 * rtb_A_x_Pkm1_x_AT_p_Q[7]);
  tmp[11] = (((rtCP_Constant_Value_EL_3 * rtb_A_x_Pkm1_x_AT_p_Q[8]) +
              (rtCP_Constant_Value_EL_7 * rtb_A_x_Pkm1_x_AT_p_Q[9])) +
             (rtCP_Constant_Value_EL_11 * rtb_A_x_Pkm1_x_AT_p_Q[10])) +
    (rtCP_Constant_Value_EL_15 * rtb_A_x_Pkm1_x_AT_p_Q[11]);
  tmp_2 = (((rtCP_Constant_Value_EL_3 * rtb_A_x_Pkm1_x_AT_p_Q[12]) +
            (rtCP_Constant_Value_EL_7 * rtb_A_x_Pkm1_x_AT_p_Q[13])) +
           (rtCP_Constant_Value_EL_11 * rtb_A_x_Pkm1_x_AT_p_Q[14])) +
    (rtCP_Constant_Value_EL_15 * rtb_A_x_Pkm1_x_AT_p_Q[15]);

  /* Sum: '<S1>/A_x_Pkm1_x_AT_p_Q' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Constant6'
   *  Math: '<S1>/AT'
   *  Product: '<S1>/A_x_Pkm1_x_AT1'
   */
  rtb_A_x_Pkm1_x_AT_p_Q[0] = ((((tmp[0] * rtCP_Constant_Value_EL_0) + (tmp[4] *
    rtCP_Constant_Value_EL_4)) + (tmp[8] * rtCP_Constant_Value_EL_8)) + (tmp[12]
    * rtCP_Constant_Value_EL_12)) + rtCP_Constant6_Value_EL_0;
  rtb_A_x_Pkm1_x_AT_p_Q[4] = ((((tmp[0] * rtCP_Constant_Value_EL_1) + (tmp[4] *
    rtCP_Constant_Value_EL_5)) + (tmp[8] * rtCP_Constant_Value_EL_9)) + (tmp[12]
    * rtCP_Constant_Value_EL_13)) + rtCP_Constant6_Value_EL_4;
  rtb_A_x_Pkm1_x_AT_p_Q[8] = ((((tmp[0] * rtCP_Constant_Value_EL_2) + (tmp[4] *
    rtCP_Constant_Value_EL_6)) + (tmp[8] * rtCP_Constant_Value_EL_10)) + (tmp[12]
    * rtCP_Constant_Value_EL_14)) + rtCP_Constant6_Value_EL_8;
  rtb_A_x_Pkm1_x_AT_p_Q[12] = ((((tmp[0] * rtCP_Constant_Value_EL_3) + (tmp[4] *
    rtCP_Constant_Value_EL_7)) + (tmp[8] * rtCP_Constant_Value_EL_11)) + (tmp[12]
    * rtCP_Constant_Value_EL_15)) + rtCP_Constant6_Value_EL_12;
  rtb_A_x_Pkm1_x_AT_p_Q[1] = ((((tmp[1] * rtCP_Constant_Value_EL_0) + (tmp[5] *
    rtCP_Constant_Value_EL_4)) + (tmp[9] * rtCP_Constant_Value_EL_8)) + (tmp[13]
    * rtCP_Constant_Value_EL_12)) + rtCP_Constant6_Value_EL_1;
  rtb_A_x_Pkm1_x_AT_p_Q[5] = ((((tmp[1] * rtCP_Constant_Value_EL_1) + (tmp[5] *
    rtCP_Constant_Value_EL_5)) + (tmp[9] * rtCP_Constant_Value_EL_9)) + (tmp[13]
    * rtCP_Constant_Value_EL_13)) + rtCP_Constant6_Value_EL_5;
  rtb_A_x_Pkm1_x_AT_p_Q[9] = ((((tmp[1] * rtCP_Constant_Value_EL_2) + (tmp[5] *
    rtCP_Constant_Value_EL_6)) + (tmp[9] * rtCP_Constant_Value_EL_10)) + (tmp[13]
    * rtCP_Constant_Value_EL_14)) + rtCP_Constant6_Value_EL_9;
  rtb_A_x_Pkm1_x_AT_p_Q[13] = ((((tmp[1] * rtCP_Constant_Value_EL_3) + (tmp[5] *
    rtCP_Constant_Value_EL_7)) + (tmp[9] * rtCP_Constant_Value_EL_11)) + (tmp[13]
    * rtCP_Constant_Value_EL_15)) + rtCP_Constant6_Value_EL_13;
  rtb_A_x_Pkm1_x_AT_p_Q[2] = ((((tmp[2] * rtCP_Constant_Value_EL_0) + (tmp[6] *
    rtCP_Constant_Value_EL_4)) + (tmp[10] * rtCP_Constant_Value_EL_8)) + (tmp[14]
    * rtCP_Constant_Value_EL_12)) + rtCP_Constant6_Value_EL_2;
  rtb_A_x_Pkm1_x_AT_p_Q[6] = ((((tmp[2] * rtCP_Constant_Value_EL_1) + (tmp[6] *
    rtCP_Constant_Value_EL_5)) + (tmp[10] * rtCP_Constant_Value_EL_9)) + (tmp[14]
    * rtCP_Constant_Value_EL_13)) + rtCP_Constant6_Value_EL_6;
  rtb_A_x_Pkm1_x_AT_p_Q[10] = ((((tmp[2] * rtCP_Constant_Value_EL_2) + (tmp[6] *
    rtCP_Constant_Value_EL_6)) + (tmp[10] * rtCP_Constant_Value_EL_10)) + (tmp
    [14] * rtCP_Constant_Value_EL_14)) + rtCP_Constant6_Value_EL_10;
  rtb_A_x_Pkm1_x_AT_p_Q[14] = ((((tmp[2] * rtCP_Constant_Value_EL_3) + (tmp[6] *
    rtCP_Constant_Value_EL_7)) + (tmp[10] * rtCP_Constant_Value_EL_11)) + (tmp
    [14] * rtCP_Constant_Value_EL_15)) + rtCP_Constant6_Value_EL_14;
  rtb_A_x_Pkm1_x_AT_p_Q[3] = ((((tmp[3] * rtCP_Constant_Value_EL_0) + (tmp[7] *
    rtCP_Constant_Value_EL_4)) + (tmp[11] * rtCP_Constant_Value_EL_8)) + (tmp_2 *
    rtCP_Constant_Value_EL_12)) + rtCP_Constant6_Value_EL_3;
  rtb_A_x_Pkm1_x_AT_p_Q[7] = ((((tmp[3] * rtCP_Constant_Value_EL_1) + (tmp[7] *
    rtCP_Constant_Value_EL_5)) + (tmp[11] * rtCP_Constant_Value_EL_9)) + (tmp_2 *
    rtCP_Constant_Value_EL_13)) + rtCP_Constant6_Value_EL_7;
  rtb_A_x_Pkm1_x_AT_p_Q[11] = ((((tmp[3] * rtCP_Constant_Value_EL_2) + (tmp[7] *
    rtCP_Constant_Value_EL_6)) + (tmp[11] * rtCP_Constant_Value_EL_10)) + (tmp_2
    * rtCP_Constant_Value_EL_14)) + rtCP_Constant6_Value_EL_11;
  rtb_A_x_Pkm1_x_AT_p_Q[15] = ((((tmp[3] * rtCP_Constant_Value_EL_3) + (tmp[7] *
    rtCP_Constant_Value_EL_7)) + (tmp[11] * rtCP_Constant_Value_EL_11)) + (tmp_2
    * rtCP_Constant_Value_EL_15)) + rtCP_Constant6_Value_EL_15;

  /* Product: '<S1>/C_x_Pkm' incorporates:
   *  Constant: '<S1>/Constant7'
   *  Sum: '<S1>/A_x_Pkm1_x_AT_p_Q'
   */
  tmp_0[0] = (((rtCP_Constant7_Value_EL_0 * rtb_A_x_Pkm1_x_AT_p_Q[0]) +
               (rtCP_Constant7_Value_EL_2 * rtb_A_x_Pkm1_x_AT_p_Q[1])) +
              (rtCP_Constant7_Value_EL_4 * rtb_A_x_Pkm1_x_AT_p_Q[2])) +
    (rtCP_Constant7_Value_EL_6 * rtb_A_x_Pkm1_x_AT_p_Q[3]);
  tmp_0[2] = (((rtCP_Constant7_Value_EL_0 * rtb_A_x_Pkm1_x_AT_p_Q[4]) +
               (rtCP_Constant7_Value_EL_2 * rtb_A_x_Pkm1_x_AT_p_Q[5])) +
              (rtCP_Constant7_Value_EL_4 * rtb_A_x_Pkm1_x_AT_p_Q[6])) +
    (rtCP_Constant7_Value_EL_6 * rtb_A_x_Pkm1_x_AT_p_Q[7]);
  tmp_0[4] = (((rtCP_Constant7_Value_EL_0 * rtb_A_x_Pkm1_x_AT_p_Q[8]) +
               (rtCP_Constant7_Value_EL_2 * rtb_A_x_Pkm1_x_AT_p_Q[9])) +
              (rtCP_Constant7_Value_EL_4 * rtb_A_x_Pkm1_x_AT_p_Q[10])) +
    (rtCP_Constant7_Value_EL_6 * rtb_A_x_Pkm1_x_AT_p_Q[11]);
  tmp_0[6] = (((rtCP_Constant7_Value_EL_0 * rtb_A_x_Pkm1_x_AT_p_Q[12]) +
               (rtCP_Constant7_Value_EL_2 * rtb_A_x_Pkm1_x_AT_p_Q[13])) +
              (rtCP_Constant7_Value_EL_4 * rtb_A_x_Pkm1_x_AT_p_Q[14])) +
    (rtCP_Constant7_Value_EL_6 * rtb_A_x_Pkm1_x_AT_p_Q[15]);
  tmp_0[1] = (((rtCP_Constant7_Value_EL_1 * rtb_A_x_Pkm1_x_AT_p_Q[0]) +
               (rtCP_Constant7_Value_EL_3 * rtb_A_x_Pkm1_x_AT_p_Q[1])) +
              (rtCP_Constant7_Value_EL_5 * rtb_A_x_Pkm1_x_AT_p_Q[2])) +
    (rtCP_Constant7_Value_EL_7 * rtb_A_x_Pkm1_x_AT_p_Q[3]);
  tmp_0[3] = (((rtCP_Constant7_Value_EL_1 * rtb_A_x_Pkm1_x_AT_p_Q[4]) +
               (rtCP_Constant7_Value_EL_3 * rtb_A_x_Pkm1_x_AT_p_Q[5])) +
              (rtCP_Constant7_Value_EL_5 * rtb_A_x_Pkm1_x_AT_p_Q[6])) +
    (rtCP_Constant7_Value_EL_7 * rtb_A_x_Pkm1_x_AT_p_Q[7]);
  tmp_0[5] = (((rtCP_Constant7_Value_EL_1 * rtb_A_x_Pkm1_x_AT_p_Q[8]) +
               (rtCP_Constant7_Value_EL_3 * rtb_A_x_Pkm1_x_AT_p_Q[9])) +
              (rtCP_Constant7_Value_EL_5 * rtb_A_x_Pkm1_x_AT_p_Q[10])) +
    (rtCP_Constant7_Value_EL_7 * rtb_A_x_Pkm1_x_AT_p_Q[11]);
  tmp_3 = (((rtCP_Constant7_Value_EL_1 * rtb_A_x_Pkm1_x_AT_p_Q[12]) +
            (rtCP_Constant7_Value_EL_3 * rtb_A_x_Pkm1_x_AT_p_Q[13])) +
           (rtCP_Constant7_Value_EL_5 * rtb_A_x_Pkm1_x_AT_p_Q[14])) +
    (rtCP_Constant7_Value_EL_7 * rtb_A_x_Pkm1_x_AT_p_Q[15]);

  /* Sum: '<S1>/C_x_Pkm_x_CT_p_R' incorporates:
   *  Constant: '<S1>/Constant8'
   *  Math: '<S1>/CT'
   *  Product: '<S1>/C_x_Pkm_x_CT'
   *  Reshape: '<S2>/reshape_2x2'
   */
  rtb_reshape_2x2[0] = ((((tmp_0[0] * SensorFusion_ConstB.CT[0]) + (tmp_0[2] *
    SensorFusion_ConstB.CT[1])) + (tmp_0[4] * SensorFusion_ConstB.CT[2])) +
                        (tmp_0[6] * SensorFusion_ConstB.CT[3])) +
    rtCP_Constant8_Value_EL_0;
  rtb_reshape_2x2[2] = ((((tmp_0[0] * SensorFusion_ConstB.CT[4]) + (tmp_0[2] *
    SensorFusion_ConstB.CT[5])) + (tmp_0[4] * SensorFusion_ConstB.CT[6])) +
                        (tmp_0[6] * SensorFusion_ConstB.CT[7])) +
    rtCP_Constant8_Value_EL_2;
  rtb_reshape_2x2[1] = ((((tmp_0[1] * SensorFusion_ConstB.CT[0]) + (tmp_0[3] *
    SensorFusion_ConstB.CT[1])) + (tmp_0[5] * SensorFusion_ConstB.CT[2])) +
                        (tmp_3 * SensorFusion_ConstB.CT[3])) +
    rtCP_Constant8_Value_EL_1;
  rtb_reshape_2x2[3] = ((((tmp_0[1] * SensorFusion_ConstB.CT[4]) + (tmp_0[3] *
    SensorFusion_ConstB.CT[5])) + (tmp_0[5] * SensorFusion_ConstB.CT[6])) +
                        (tmp_3 * SensorFusion_ConstB.CT[7])) +
    rtCP_Constant8_Value_EL_3;

  /* MultiPortSwitch: '<S2>/Select_a' incorporates:
   *  Constant: '<S2>/Position_Index1'
   *  Reshape: '<S2>/reshape_2x2'
   *  Reshape: '<S2>/reshape_4x1'
   */
  rtb_Select_a = rtb_reshape_2x2[rtCP_pooled2];

  /* MultiPortSwitch: '<S2>/Select_d' incorporates:
   *  Constant: '<S2>/Position_Index2'
   *  Reshape: '<S2>/reshape_2x2'
   *  Reshape: '<S2>/reshape_4x1'
   */
  rtb_Select_d = rtb_reshape_2x2[rtCP_pooled1];

  /* MultiPortSwitch: '<S2>/Select_b' incorporates:
   *  Constant: '<S2>/Position_Index3'
   *  Reshape: '<S2>/reshape_2x2'
   *  Reshape: '<S2>/reshape_4x1'
   */
  rtb_Select_b = rtb_reshape_2x2[rtCP_pooled3];

  /* MultiPortSwitch: '<S2>/Select_c' incorporates:
   *  Constant: '<S2>/Position_Index4'
   *  Reshape: '<S2>/reshape_2x2'
   *  Reshape: '<S2>/reshape_4x1'
   */
  rtb_Select_c = rtb_reshape_2x2[rtCP_Position_Index4_Value];

  /* Product: '<S2>/inv_ab_minus_bc' incorporates:
   *  Product: '<S2>/a_x_d'
   *  Product: '<S2>/b_x_c'
   *  Sum: '<S2>/ab_minus_bc'
   */
  rtb_StrokeToQuadCount = 1.0F / ((rtb_Select_a * rtb_Select_d) - (rtb_Select_b *
    rtb_Select_c));

  /* Product: '<S1>/Pk_m_CT' incorporates:
   *  Math: '<S1>/CT'
   *  Sum: '<S1>/A_x_Pkm1_x_AT_p_Q'
   */
  rtb_A_x_Pkm1_x_AT_p_Q_0[0] = (((rtb_A_x_Pkm1_x_AT_p_Q[0] *
    SensorFusion_ConstB.CT[0]) + (rtb_A_x_Pkm1_x_AT_p_Q[4] *
    SensorFusion_ConstB.CT[1])) + (rtb_A_x_Pkm1_x_AT_p_Q[8] *
    SensorFusion_ConstB.CT[2])) + (rtb_A_x_Pkm1_x_AT_p_Q[12] *
    SensorFusion_ConstB.CT[3]);
  rtb_A_x_Pkm1_x_AT_p_Q_0[4] = (((rtb_A_x_Pkm1_x_AT_p_Q[0] *
    SensorFusion_ConstB.CT[4]) + (rtb_A_x_Pkm1_x_AT_p_Q[4] *
    SensorFusion_ConstB.CT[5])) + (rtb_A_x_Pkm1_x_AT_p_Q[8] *
    SensorFusion_ConstB.CT[6])) + (rtb_A_x_Pkm1_x_AT_p_Q[12] *
    SensorFusion_ConstB.CT[7]);
  rtb_A_x_Pkm1_x_AT_p_Q_0[1] = (((rtb_A_x_Pkm1_x_AT_p_Q[1] *
    SensorFusion_ConstB.CT[0]) + (rtb_A_x_Pkm1_x_AT_p_Q[5] *
    SensorFusion_ConstB.CT[1])) + (rtb_A_x_Pkm1_x_AT_p_Q[9] *
    SensorFusion_ConstB.CT[2])) + (rtb_A_x_Pkm1_x_AT_p_Q[13] *
    SensorFusion_ConstB.CT[3]);
  rtb_A_x_Pkm1_x_AT_p_Q_0[5] = (((rtb_A_x_Pkm1_x_AT_p_Q[1] *
    SensorFusion_ConstB.CT[4]) + (rtb_A_x_Pkm1_x_AT_p_Q[5] *
    SensorFusion_ConstB.CT[5])) + (rtb_A_x_Pkm1_x_AT_p_Q[9] *
    SensorFusion_ConstB.CT[6])) + (rtb_A_x_Pkm1_x_AT_p_Q[13] *
    SensorFusion_ConstB.CT[7]);
  rtb_A_x_Pkm1_x_AT_p_Q_0[2] = (((rtb_A_x_Pkm1_x_AT_p_Q[2] *
    SensorFusion_ConstB.CT[0]) + (rtb_A_x_Pkm1_x_AT_p_Q[6] *
    SensorFusion_ConstB.CT[1])) + (rtb_A_x_Pkm1_x_AT_p_Q[10] *
    SensorFusion_ConstB.CT[2])) + (rtb_A_x_Pkm1_x_AT_p_Q[14] *
    SensorFusion_ConstB.CT[3]);
  rtb_A_x_Pkm1_x_AT_p_Q_0[6] = (((rtb_A_x_Pkm1_x_AT_p_Q[2] *
    SensorFusion_ConstB.CT[4]) + (rtb_A_x_Pkm1_x_AT_p_Q[6] *
    SensorFusion_ConstB.CT[5])) + (rtb_A_x_Pkm1_x_AT_p_Q[10] *
    SensorFusion_ConstB.CT[6])) + (rtb_A_x_Pkm1_x_AT_p_Q[14] *
    SensorFusion_ConstB.CT[7]);
  rtb_A_x_Pkm1_x_AT_p_Q_0[3] = (((rtb_A_x_Pkm1_x_AT_p_Q[3] *
    SensorFusion_ConstB.CT[0]) + (rtb_A_x_Pkm1_x_AT_p_Q[7] *
    SensorFusion_ConstB.CT[1])) + (rtb_A_x_Pkm1_x_AT_p_Q[11] *
    SensorFusion_ConstB.CT[2])) + (rtb_A_x_Pkm1_x_AT_p_Q[15] *
    SensorFusion_ConstB.CT[3]);
  rtb_A_x_Pkm1_x_AT_p_Q_1 = (((rtb_A_x_Pkm1_x_AT_p_Q[3] *
    SensorFusion_ConstB.CT[4]) + (rtb_A_x_Pkm1_x_AT_p_Q[7] *
    SensorFusion_ConstB.CT[5])) + (rtb_A_x_Pkm1_x_AT_p_Q[11] *
    SensorFusion_ConstB.CT[6])) + (rtb_A_x_Pkm1_x_AT_p_Q[15] *
    SensorFusion_ConstB.CT[7]);

  /* Product: '<S1>/calcKalmanGain' incorporates:
   *  Gain: '<S2>/Gain'
   *  Gain: '<S2>/Gain1'
   *  Product: '<S2>/inv_ab_minus_bc_x_d_nb_nc_a'
   *  SignalConversion generated from: '<S2>/conc_d_nb_nc_a'
   */
  rtb_calcKalmanGain_tmp = rtb_StrokeToQuadCount * rtb_Select_d;
  rtb_calcKalmanGain_tmp_0 = rtb_StrokeToQuadCount * (rtCP_pooled7 *
    rtb_Select_b);
  rtb_calcKalmanGain[0] = (rtb_A_x_Pkm1_x_AT_p_Q_0[0] * rtb_calcKalmanGain_tmp)
    + (rtb_A_x_Pkm1_x_AT_p_Q_0[4] * rtb_calcKalmanGain_tmp_0);
  rtb_calcKalmanGain_tmp_1 = rtb_StrokeToQuadCount * (rtCP_pooled7 *
    rtb_Select_c);
  rtb_calcKalmanGain_tmp_2 = rtb_StrokeToQuadCount * rtb_Select_a;
  rtb_calcKalmanGain[4] = (rtb_A_x_Pkm1_x_AT_p_Q_0[0] * rtb_calcKalmanGain_tmp_1)
    + (rtb_A_x_Pkm1_x_AT_p_Q_0[4] * rtb_calcKalmanGain_tmp_2);
  rtb_calcKalmanGain[1] = (rtb_A_x_Pkm1_x_AT_p_Q_0[1] * rtb_calcKalmanGain_tmp)
    + (rtb_A_x_Pkm1_x_AT_p_Q_0[5] * rtb_calcKalmanGain_tmp_0);
  rtb_calcKalmanGain[5] = (rtb_A_x_Pkm1_x_AT_p_Q_0[1] * rtb_calcKalmanGain_tmp_1)
    + (rtb_A_x_Pkm1_x_AT_p_Q_0[5] * rtb_calcKalmanGain_tmp_2);
  rtb_calcKalmanGain[2] = (rtb_A_x_Pkm1_x_AT_p_Q_0[2] * rtb_calcKalmanGain_tmp)
    + (rtb_A_x_Pkm1_x_AT_p_Q_0[6] * rtb_calcKalmanGain_tmp_0);
  rtb_calcKalmanGain[6] = (rtb_A_x_Pkm1_x_AT_p_Q_0[2] * rtb_calcKalmanGain_tmp_1)
    + (rtb_A_x_Pkm1_x_AT_p_Q_0[6] * rtb_calcKalmanGain_tmp_2);
  rtb_calcKalmanGain[3] = (rtb_A_x_Pkm1_x_AT_p_Q_0[3] * rtb_calcKalmanGain_tmp)
    + (rtb_A_x_Pkm1_x_AT_p_Q_1 * rtb_calcKalmanGain_tmp_0);
  rtb_calcKalmanGain[7] = (rtb_A_x_Pkm1_x_AT_p_Q_0[3] * rtb_calcKalmanGain_tmp_1)
    + (rtb_A_x_Pkm1_x_AT_p_Q_1 * rtb_calcKalmanGain_tmp_2);

  /* DataTypeConversion: '<Root>/convQuad_f32' */
  rtb_convQuad_f32 = (real32_T)(*rtu_s16MotorQuadCount);

  /* Switch: '<Root>/Switch' incorporates:
   *  Constant: '<Root>/quadCountToStrokeInches1'
   *  Product: '<Root>/StrokeToQuadCount1'
   *  Reshape: '<Root>/reshape_y_2x1'
   */
  if ((*rtu_bSensorFusionEnable) > rtCP_Switch_Threshold) {
    rtb_reshape_y_2x1_idx_1 = (real32_T)(*rtu_f32SkewStroke);
  } else {
    rtb_reshape_y_2x1_idx_1 = rtb_convQuad_f32 * rtCP_pooled5;
  }

  /* End of Switch: '<Root>/Switch' */

  /* Switch: '<S1>/Switch2' incorporates:
   *  Delay: '<S1>/Delay3'
   *  Delay: '<S1>/Delay4'
   *  Product: '<S1>/Kk_x_yk_m_C_x_xhatkm'
   */
  if (localDW->Delay3_DSTATE > rtCP_pooled6) {
    rtb_Kk_x_yk_m_C_x_xhatkm_idx_0 = localDW->Delay4_DSTATE[0];
    rtb_Kk_x_yk_m_C_x_xhatkm_idx_1 = localDW->Delay4_DSTATE[1];
    rtb_Kk_x_yk_m_C_x_xhatkm_idx_2 = localDW->Delay4_DSTATE[2];
    rtb_Kk_x_yk_m_C_x_xhatkm_idx_3 = localDW->Delay4_DSTATE[3];
  } else {
    /* SignalConversion generated from: '<Root>/X0' incorporates:
     *  Constant: '<Root>/f32InitialBias'
     *  Product: '<S1>/Kk_x_yk_m_C_x_xhatkm'
     */
    rtb_Kk_x_yk_m_C_x_xhatkm_idx_3 = rtCP_pooled6;

    /* SignalConversion generated from: '<Root>/X0' incorporates:
     *  Constant: '<Root>/f32InitialAcceleration'
     *  Product: '<S1>/Kk_x_yk_m_C_x_xhatkm'
     */
    rtb_Kk_x_yk_m_C_x_xhatkm_idx_2 = rtCP_pooled6;

    /* SignalConversion generated from: '<Root>/X0' incorporates:
     *  Constant: '<Root>/f32InitialVelocity'
     *  Product: '<S1>/Kk_x_yk_m_C_x_xhatkm'
     */
    rtb_Kk_x_yk_m_C_x_xhatkm_idx_1 = rtCP_pooled6;

    /* SignalConversion generated from: '<Root>/X0' incorporates:
     *  Product: '<S1>/Kk_x_yk_m_C_x_xhatkm'
     */
    rtb_Kk_x_yk_m_C_x_xhatkm_idx_0 = (real32_T)(*rtu_f32InitialPosition);
  }

  /* End of Switch: '<S1>/Switch2' */

  /* Product: '<S1>/A_x_xhat_km1' incorporates:
   *  Constant: '<S1>/Constant'
   *  Product: '<S1>/Kk_x_yk_m_C_x_xhatkm'
   */
  rtb_xhat_km[0] = (((rtCP_Constant_Value_EL_0 * rtb_Kk_x_yk_m_C_x_xhatkm_idx_0)
                     + (rtCP_Constant_Value_EL_4 *
                        rtb_Kk_x_yk_m_C_x_xhatkm_idx_1)) +
                    (rtCP_Constant_Value_EL_8 * rtb_Kk_x_yk_m_C_x_xhatkm_idx_2))
    + (rtCP_Constant_Value_EL_12 * rtb_Kk_x_yk_m_C_x_xhatkm_idx_3);
  rtb_xhat_km[1] = (((rtCP_Constant_Value_EL_1 * rtb_Kk_x_yk_m_C_x_xhatkm_idx_0)
                     + (rtCP_Constant_Value_EL_5 *
                        rtb_Kk_x_yk_m_C_x_xhatkm_idx_1)) +
                    (rtCP_Constant_Value_EL_9 * rtb_Kk_x_yk_m_C_x_xhatkm_idx_2))
    + (rtCP_Constant_Value_EL_13 * rtb_Kk_x_yk_m_C_x_xhatkm_idx_3);
  rtb_xhat_km[2] = (((rtCP_Constant_Value_EL_2 * rtb_Kk_x_yk_m_C_x_xhatkm_idx_0)
                     + (rtCP_Constant_Value_EL_6 *
                        rtb_Kk_x_yk_m_C_x_xhatkm_idx_1)) +
                    (rtCP_Constant_Value_EL_10 * rtb_Kk_x_yk_m_C_x_xhatkm_idx_2))
    + (rtCP_Constant_Value_EL_14 * rtb_Kk_x_yk_m_C_x_xhatkm_idx_3);
  rtb_xhat_km_jaf4 = (((rtCP_Constant_Value_EL_3 *
                        rtb_Kk_x_yk_m_C_x_xhatkm_idx_0) +
                       (rtCP_Constant_Value_EL_7 *
                        rtb_Kk_x_yk_m_C_x_xhatkm_idx_1)) +
                      (rtCP_Constant_Value_EL_11 *
                       rtb_Kk_x_yk_m_C_x_xhatkm_idx_2)) +
    (rtCP_Constant_Value_EL_15 * rtb_Kk_x_yk_m_C_x_xhatkm_idx_3);

  /* Sum: '<S1>/yk_m_C_x_xhatkm' incorporates:
   *  Constant: '<S1>/Constant7'
   *  Product: '<S1>/A_x_xhat_km1'
   *  Product: '<S1>/C_x_xhatkm'
   *  Reshape: '<Root>/reshape_y_2x1'
   *  SignalConversion generated from: '<Root>/y'
   *  Sum: '<Root>/Sum'
   */
  rtb_xhat_tmp[0] = rtb_convQuad_f32 - ((((rtCP_Constant7_Value_EL_0 *
    rtb_xhat_km[0]) + (rtCP_Constant7_Value_EL_2 * rtb_xhat_km[1])) +
    (rtCP_Constant7_Value_EL_4 * rtb_xhat_km[2])) + (rtCP_Constant7_Value_EL_6 *
    rtb_xhat_km_jaf4));
  rtb_xhat_tmp[1] = rtb_reshape_y_2x1_idx_1 - ((((rtCP_Constant7_Value_EL_1 *
    rtb_xhat_km[0]) + (rtCP_Constant7_Value_EL_3 * rtb_xhat_km[1])) +
    (rtCP_Constant7_Value_EL_5 * rtb_xhat_km[2])) + (rtCP_Constant7_Value_EL_7 *
    rtb_xhat_km_jaf4));

  /* Sum: '<S1>/correction_xhat' incorporates:
   *  Product: '<S1>/A_x_xhat_km1'
   *  Product: '<S1>/Kk_x_yk_m_C_x_xhatkm'
   *  Product: '<S1>/calcKalmanGain'
   *  Sum: '<S1>/yk_m_C_x_xhatkm'
   */
  rtb_xhat[0] = rtb_xhat_km[0] + ((rtb_calcKalmanGain[0] * rtb_xhat_tmp[0]) +
    (rtb_calcKalmanGain[4] * rtb_xhat_tmp[1]));
  rtb_xhat[1] = rtb_xhat_km[1] + ((rtb_calcKalmanGain[1] * rtb_xhat_tmp[0]) +
    (rtb_calcKalmanGain[5] * rtb_xhat_tmp[1]));
  rtb_xhat[2] = rtb_xhat_km[2] + ((rtb_calcKalmanGain[2] * rtb_xhat_tmp[0]) +
    (rtb_calcKalmanGain[6] * rtb_xhat_tmp[1]));
  rtb_xhat[3] = rtb_xhat_km_jaf4 + ((rtb_calcKalmanGain[3] * rtb_xhat_tmp[0]) +
    (rtb_calcKalmanGain[7] * rtb_xhat_tmp[1]));

  /* MultiPortSwitch: '<Root>/Residual' incorporates:
   *  Constant: '<Root>/ResidualIndex0'
   *  Sum: '<Root>/Sum'
   */
  *rty_f32QuadResidual = rtb_xhat_tmp[rtCP_pooled2];

  /* MultiPortSwitch: '<Root>/Residual1' incorporates:
   *  Constant: '<Root>/ResidualIndex1'
   *  Sum: '<Root>/Sum'
   */
  *rty_f32SkewResidual = rtb_xhat_tmp[rtCP_pooled3];

  /* Product: '<Root>/StrokeToQuadCount2' incorporates:
   *  Constant: '<Root>/BiasIndex3'
   *  Constant: '<Root>/quadCountToStrokeInches2'
   *  MultiPortSwitch: '<Root>/Bias'
   *  Sum: '<S1>/correction_xhat'
   */
  *rty_f32EstimatedBias = rtCP_pooled5 * rtb_xhat[rtCP_pooled1];

  /* MultiPortSwitch: '<Root>/Position' incorporates:
   *  Constant: '<Root>/PositionIndex0'
   *  Sum: '<S1>/correction_xhat'
   */
  *rty_f32EstimatedPosition = rtb_xhat[rtCP_pooled2];

  /* DataTypeConversion: '<Root>/convQuad_s16' incorporates:
   *  Constant: '<Root>/StrokeToQuadCountConst'
   *  Product: '<Root>/StrokeToQuadCount'
   */
  *rty_s16EstimatedQuadCount = (int16_t)((int16_T)((real32_T)floor((real_T)
    ((real32_T)((*rty_f32EstimatedPosition) * rtCP_StrokeToQuadCountConst_Val)))));

  /* Update for Delay: '<S1>/Delay1' incorporates:
   *  Constant: '<S1>/Init_P0'
   */
  localDW->Delay1_DSTATE = rtCP_pooled4;

  /* Sum: '<S1>/I_m_Kk_x_C' incorporates:
   *  Constant: '<S1>/Constant4'
   *  Constant: '<S1>/Constant7'
   *  Product: '<S1>/Kk_x_C'
   *  Product: '<S1>/calcKalmanGain'
   */
  tmp_1[0] = rtCP_Constant4_Value_EL_0 - ((rtb_calcKalmanGain[0] *
    rtCP_Constant7_Value_EL_0) + (rtb_calcKalmanGain[4] *
    rtCP_Constant7_Value_EL_1));
  tmp_1[4] = rtCP_Constant4_Value_EL_4 - ((rtb_calcKalmanGain[0] *
    rtCP_Constant7_Value_EL_2) + (rtb_calcKalmanGain[4] *
    rtCP_Constant7_Value_EL_3));
  tmp_1[8] = rtCP_Constant4_Value_EL_8 - ((rtb_calcKalmanGain[0] *
    rtCP_Constant7_Value_EL_4) + (rtb_calcKalmanGain[4] *
    rtCP_Constant7_Value_EL_5));
  tmp_1[12] = rtCP_Constant4_Value_EL_12 - ((rtb_calcKalmanGain[0] *
    rtCP_Constant7_Value_EL_6) + (rtb_calcKalmanGain[4] *
    rtCP_Constant7_Value_EL_7));
  tmp_1[1] = rtCP_Constant4_Value_EL_1 - ((rtb_calcKalmanGain[1] *
    rtCP_Constant7_Value_EL_0) + (rtb_calcKalmanGain[5] *
    rtCP_Constant7_Value_EL_1));
  tmp_1[5] = rtCP_Constant4_Value_EL_5 - ((rtb_calcKalmanGain[1] *
    rtCP_Constant7_Value_EL_2) + (rtb_calcKalmanGain[5] *
    rtCP_Constant7_Value_EL_3));
  tmp_1[9] = rtCP_Constant4_Value_EL_9 - ((rtb_calcKalmanGain[1] *
    rtCP_Constant7_Value_EL_4) + (rtb_calcKalmanGain[5] *
    rtCP_Constant7_Value_EL_5));
  tmp_1[13] = rtCP_Constant4_Value_EL_13 - ((rtb_calcKalmanGain[1] *
    rtCP_Constant7_Value_EL_6) + (rtb_calcKalmanGain[5] *
    rtCP_Constant7_Value_EL_7));
  tmp_1[2] = rtCP_Constant4_Value_EL_2 - ((rtb_calcKalmanGain[2] *
    rtCP_Constant7_Value_EL_0) + (rtb_calcKalmanGain[6] *
    rtCP_Constant7_Value_EL_1));
  tmp_1[6] = rtCP_Constant4_Value_EL_6 - ((rtb_calcKalmanGain[2] *
    rtCP_Constant7_Value_EL_2) + (rtb_calcKalmanGain[6] *
    rtCP_Constant7_Value_EL_3));
  tmp_1[10] = rtCP_Constant4_Value_EL_10 - ((rtb_calcKalmanGain[2] *
    rtCP_Constant7_Value_EL_4) + (rtb_calcKalmanGain[6] *
    rtCP_Constant7_Value_EL_5));
  tmp_1[14] = rtCP_Constant4_Value_EL_14 - ((rtb_calcKalmanGain[2] *
    rtCP_Constant7_Value_EL_6) + (rtb_calcKalmanGain[6] *
    rtCP_Constant7_Value_EL_7));
  tmp_1[3] = rtCP_Constant4_Value_EL_3 - ((rtb_calcKalmanGain[3] *
    rtCP_Constant7_Value_EL_0) + (rtb_calcKalmanGain[7] *
    rtCP_Constant7_Value_EL_1));
  tmp_1[7] = rtCP_Constant4_Value_EL_7 - ((rtb_calcKalmanGain[3] *
    rtCP_Constant7_Value_EL_2) + (rtb_calcKalmanGain[7] *
    rtCP_Constant7_Value_EL_3));
  tmp_1[11] = rtCP_Constant4_Value_EL_11 - ((rtb_calcKalmanGain[3] *
    rtCP_Constant7_Value_EL_4) + (rtb_calcKalmanGain[7] *
    rtCP_Constant7_Value_EL_5));
  tmp_1[15] = rtCP_Constant4_Value_EL_15 - ((rtb_calcKalmanGain[3] *
    rtCP_Constant7_Value_EL_6) + (rtb_calcKalmanGain[7] *
    rtCP_Constant7_Value_EL_7));

  /* Update for Delay: '<S1>/Delay2' incorporates:
   *  Product: '<S1>/I_m_Kk_x_C_x_Pkm'
   *  Sum: '<S1>/A_x_Pkm1_x_AT_p_Q'
   */
  localDW->Delay2_DSTATE[0] = (((tmp_1[0] * rtb_A_x_Pkm1_x_AT_p_Q[0]) + (tmp_1[4]
    * rtb_A_x_Pkm1_x_AT_p_Q[1])) + (tmp_1[8] * rtb_A_x_Pkm1_x_AT_p_Q[2])) +
    (tmp_1[12] * rtb_A_x_Pkm1_x_AT_p_Q[3]);
  localDW->Delay2_DSTATE[1] = (((tmp_1[1] * rtb_A_x_Pkm1_x_AT_p_Q[0]) + (tmp_1[5]
    * rtb_A_x_Pkm1_x_AT_p_Q[1])) + (tmp_1[9] * rtb_A_x_Pkm1_x_AT_p_Q[2])) +
    (tmp_1[13] * rtb_A_x_Pkm1_x_AT_p_Q[3]);
  localDW->Delay2_DSTATE[2] = (((tmp_1[2] * rtb_A_x_Pkm1_x_AT_p_Q[0]) + (tmp_1[6]
    * rtb_A_x_Pkm1_x_AT_p_Q[1])) + (tmp_1[10] * rtb_A_x_Pkm1_x_AT_p_Q[2])) +
    (tmp_1[14] * rtb_A_x_Pkm1_x_AT_p_Q[3]);
  localDW->Delay2_DSTATE[3] = (((tmp_1[3] * rtb_A_x_Pkm1_x_AT_p_Q[0]) + (tmp_1[7]
    * rtb_A_x_Pkm1_x_AT_p_Q[1])) + (tmp_1[11] * rtb_A_x_Pkm1_x_AT_p_Q[2])) +
    (tmp_1[15] * rtb_A_x_Pkm1_x_AT_p_Q[3]);
  localDW->Delay2_DSTATE[4] = (((tmp_1[0] * rtb_A_x_Pkm1_x_AT_p_Q[4]) + (tmp_1[4]
    * rtb_A_x_Pkm1_x_AT_p_Q[5])) + (tmp_1[8] * rtb_A_x_Pkm1_x_AT_p_Q[6])) +
    (tmp_1[12] * rtb_A_x_Pkm1_x_AT_p_Q[7]);
  localDW->Delay2_DSTATE[5] = (((tmp_1[1] * rtb_A_x_Pkm1_x_AT_p_Q[4]) + (tmp_1[5]
    * rtb_A_x_Pkm1_x_AT_p_Q[5])) + (tmp_1[9] * rtb_A_x_Pkm1_x_AT_p_Q[6])) +
    (tmp_1[13] * rtb_A_x_Pkm1_x_AT_p_Q[7]);
  localDW->Delay2_DSTATE[6] = (((tmp_1[2] * rtb_A_x_Pkm1_x_AT_p_Q[4]) + (tmp_1[6]
    * rtb_A_x_Pkm1_x_AT_p_Q[5])) + (tmp_1[10] * rtb_A_x_Pkm1_x_AT_p_Q[6])) +
    (tmp_1[14] * rtb_A_x_Pkm1_x_AT_p_Q[7]);
  localDW->Delay2_DSTATE[7] = (((tmp_1[3] * rtb_A_x_Pkm1_x_AT_p_Q[4]) + (tmp_1[7]
    * rtb_A_x_Pkm1_x_AT_p_Q[5])) + (tmp_1[11] * rtb_A_x_Pkm1_x_AT_p_Q[6])) +
    (tmp_1[15] * rtb_A_x_Pkm1_x_AT_p_Q[7]);
  localDW->Delay2_DSTATE[8] = (((tmp_1[0] * rtb_A_x_Pkm1_x_AT_p_Q[8]) + (tmp_1[4]
    * rtb_A_x_Pkm1_x_AT_p_Q[9])) + (tmp_1[8] * rtb_A_x_Pkm1_x_AT_p_Q[10])) +
    (tmp_1[12] * rtb_A_x_Pkm1_x_AT_p_Q[11]);
  localDW->Delay2_DSTATE[9] = (((tmp_1[1] * rtb_A_x_Pkm1_x_AT_p_Q[8]) + (tmp_1[5]
    * rtb_A_x_Pkm1_x_AT_p_Q[9])) + (tmp_1[9] * rtb_A_x_Pkm1_x_AT_p_Q[10])) +
    (tmp_1[13] * rtb_A_x_Pkm1_x_AT_p_Q[11]);
  localDW->Delay2_DSTATE[10] = (((tmp_1[2] * rtb_A_x_Pkm1_x_AT_p_Q[8]) + (tmp_1
    [6] * rtb_A_x_Pkm1_x_AT_p_Q[9])) + (tmp_1[10] * rtb_A_x_Pkm1_x_AT_p_Q[10]))
    + (tmp_1[14] * rtb_A_x_Pkm1_x_AT_p_Q[11]);
  localDW->Delay2_DSTATE[11] = (((tmp_1[3] * rtb_A_x_Pkm1_x_AT_p_Q[8]) + (tmp_1
    [7] * rtb_A_x_Pkm1_x_AT_p_Q[9])) + (tmp_1[11] * rtb_A_x_Pkm1_x_AT_p_Q[10]))
    + (tmp_1[15] * rtb_A_x_Pkm1_x_AT_p_Q[11]);
  localDW->Delay2_DSTATE[12] = (((tmp_1[0] * rtb_A_x_Pkm1_x_AT_p_Q[12]) +
    (tmp_1[4] * rtb_A_x_Pkm1_x_AT_p_Q[13])) + (tmp_1[8] * rtb_A_x_Pkm1_x_AT_p_Q
    [14])) + (tmp_1[12] * rtb_A_x_Pkm1_x_AT_p_Q[15]);
  localDW->Delay2_DSTATE[13] = (((tmp_1[1] * rtb_A_x_Pkm1_x_AT_p_Q[12]) +
    (tmp_1[5] * rtb_A_x_Pkm1_x_AT_p_Q[13])) + (tmp_1[9] * rtb_A_x_Pkm1_x_AT_p_Q
    [14])) + (tmp_1[13] * rtb_A_x_Pkm1_x_AT_p_Q[15]);
  localDW->Delay2_DSTATE[14] = (((tmp_1[2] * rtb_A_x_Pkm1_x_AT_p_Q[12]) +
    (tmp_1[6] * rtb_A_x_Pkm1_x_AT_p_Q[13])) + (tmp_1[10] *
    rtb_A_x_Pkm1_x_AT_p_Q[14])) + (tmp_1[14] * rtb_A_x_Pkm1_x_AT_p_Q[15]);
  localDW->Delay2_DSTATE[15] = (((tmp_1[3] * rtb_A_x_Pkm1_x_AT_p_Q[12]) +
    (tmp_1[7] * rtb_A_x_Pkm1_x_AT_p_Q[13])) + (tmp_1[11] *
    rtb_A_x_Pkm1_x_AT_p_Q[14])) + (tmp_1[15] * rtb_A_x_Pkm1_x_AT_p_Q[15]);

  /* Update for Delay: '<S1>/Delay3' incorporates:
   *  Constant: '<S1>/Init_P1'
   */
  localDW->Delay3_DSTATE = rtCP_pooled4;

  /* Update for Delay: '<S1>/Delay4' incorporates:
   *  Sum: '<S1>/correction_xhat'
   */
  localDW->Delay4_DSTATE[0] = rtb_xhat[0];
  localDW->Delay4_DSTATE[1] = rtb_xhat[1];
  localDW->Delay4_DSTATE[2] = rtb_xhat[2];
  localDW->Delay4_DSTATE[3] = rtb_xhat[3];
}

/*
 *   Function:    SensorFusion_initialize
 *   Model name:  SensorFusion
 *   Description: Model initialize function
 *   Arguments:   DW_SensorFusion_fwu4_T *localDW
 *   Return:      void
 */
void SensorFusion_initialize(DW_SensorFusion_fwu4_T *localDW)
{
  /* Registration code */

  /* states (dwork) */
  localDW->Delay1_DSTATE = 0.0F;
  localDW->Delay2_DSTATE[0] = 0.0F;
  localDW->Delay2_DSTATE[1] = 0.0F;
  localDW->Delay2_DSTATE[2] = 0.0F;
  localDW->Delay2_DSTATE[3] = 0.0F;
  localDW->Delay2_DSTATE[4] = 0.0F;
  localDW->Delay2_DSTATE[5] = 0.0F;
  localDW->Delay2_DSTATE[6] = 0.0F;
  localDW->Delay2_DSTATE[7] = 0.0F;
  localDW->Delay2_DSTATE[8] = 0.0F;
  localDW->Delay2_DSTATE[9] = 0.0F;
  localDW->Delay2_DSTATE[10] = 0.0F;
  localDW->Delay2_DSTATE[11] = 0.0F;
  localDW->Delay2_DSTATE[12] = 0.0F;
  localDW->Delay2_DSTATE[13] = 0.0F;
  localDW->Delay2_DSTATE[14] = 0.0F;
  localDW->Delay2_DSTATE[15] = 0.0F;
  localDW->Delay3_DSTATE = 0.0F;
  localDW->Delay4_DSTATE[0] = 0.0F;
  localDW->Delay4_DSTATE[1] = 0.0F;
  localDW->Delay4_DSTATE[2] = 0.0F;
  localDW->Delay4_DSTATE[3] = 0.0F;
}

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
