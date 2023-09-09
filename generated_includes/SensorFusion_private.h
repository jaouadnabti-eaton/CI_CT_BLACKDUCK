/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        SensorFusion_private.h
 *   Module Name: SensorFusion
 *   Module Ver:  10.10
 *   Date:        Thu Jul 20 13:12:22 2023
 */

#ifndef RTW_HEADER_SensorFusion_private_h_
#define RTW_HEADER_SensorFusion_private_h_
#include "rtwtypes.h"
#include "typedefs.h"
#include "SensorFusion.h"
#include "SensorFusion_types.h"

/* Pooled Parameter (Mixed Expressions)
 * Referenced by:
 *   '<Root>/BiasIndex3' (Parameter: Value)
 *   '<S2>/Position_Index2' (Parameter: Value)
 */
#define rtCP_pooled1                   (((int32_T)3L))

/* Pooled Parameter (Mixed Expressions)
 * Referenced by:
 *   '<Root>/PositionIndex0' (Parameter: Value)
 *   '<Root>/ResidualIndex0' (Parameter: Value)
 *   '<S2>/Position_Index1' (Parameter: Value)
 */
#define rtCP_pooled2                   (((int32_T)0L))

/* Pooled Parameter (Mixed Expressions)
 * Referenced by:
 *   '<Root>/ResidualIndex1' (Parameter: Value)
 *   '<S2>/Position_Index3' (Parameter: Value)
 */
#define rtCP_pooled3                   (((int32_T)1L))

/* Computed Parameter: Position_Index4_Value
 * Referenced by: '<S2>/Position_Index4' (Parameter: Value)
 */
#define rtCP_Position_Index4_Value     (((int32_T)2L))

/* Expression: C
 * Referenced by: '<S1>/Constant7' (Parameter: Value)
 */
#define rtCP_Constant7_Value_EL_0      (2516.90845F)
#define rtCP_Constant7_Value_EL_1      (1.0F)
#define rtCP_Constant7_Value_EL_2      (0.0F)
#define rtCP_Constant7_Value_EL_3      (0.0F)
#define rtCP_Constant7_Value_EL_4      (0.0F)
#define rtCP_Constant7_Value_EL_5      (0.0F)
#define rtCP_Constant7_Value_EL_6      (1.0F)
#define rtCP_Constant7_Value_EL_7      (0.0F)

/* Expression: Q
 * Referenced by: '<S1>/Constant6' (Parameter: Value)
 */
#define rtCP_Constant6_Value_EL_0      (1.0E-6F)
#define rtCP_Constant6_Value_EL_1      (0.0F)
#define rtCP_Constant6_Value_EL_2      (0.0F)
#define rtCP_Constant6_Value_EL_3      (0.0F)
#define rtCP_Constant6_Value_EL_4      (0.0F)
#define rtCP_Constant6_Value_EL_5      (0.0001F)
#define rtCP_Constant6_Value_EL_6      (0.0F)
#define rtCP_Constant6_Value_EL_7      (0.0F)
#define rtCP_Constant6_Value_EL_8      (0.0F)
#define rtCP_Constant6_Value_EL_9      (0.0F)
#define rtCP_Constant6_Value_EL_10     (0.01F)
#define rtCP_Constant6_Value_EL_11     (0.0F)
#define rtCP_Constant6_Value_EL_12     (0.0F)
#define rtCP_Constant6_Value_EL_13     (0.0F)
#define rtCP_Constant6_Value_EL_14     (0.0F)
#define rtCP_Constant6_Value_EL_15     (0.09F)

/* Expression: R
 * Referenced by: '<S1>/Constant8' (Parameter: Value)
 */
#define rtCP_Constant8_Value_EL_0      (1.0F)
#define rtCP_Constant8_Value_EL_1      (0.0F)
#define rtCP_Constant8_Value_EL_2      (0.0F)
#define rtCP_Constant8_Value_EL_3      (2.5E-5F)

/* Pooled Parameter (Expression: )
 * Referenced by:
 *   '<S1>/Init_P0' (Parameter: Value)
 *   '<S1>/Init_P1' (Parameter: Value)
 */
#define rtCP_pooled4                   (1.0F)

/* Expression: quadCountsPerInch
 * Referenced by: '<Root>/StrokeToQuadCountConst' (Parameter: Value)
 */
#define rtCP_StrokeToQuadCountConst_Val (2516.90845F)

/* Pooled Parameter (Expression: inchesPerQuadCount)
 * Referenced by:
 *   '<Root>/quadCountToStrokeInches1' (Parameter: Value)
 *   '<Root>/quadCountToStrokeInches2' (Parameter: Value)
 */
#define rtCP_pooled5                   (0.000397312833F)

/* Expression: A
 * Referenced by: '<S1>/Constant' (Parameter: Value)
 */
#define rtCP_Constant_Value_EL_0       (1.0F)
#define rtCP_Constant_Value_EL_1       (0.0F)
#define rtCP_Constant_Value_EL_2       (0.0F)
#define rtCP_Constant_Value_EL_3       (0.0F)
#define rtCP_Constant_Value_EL_4       (0.001F)
#define rtCP_Constant_Value_EL_5       (1.0F)
#define rtCP_Constant_Value_EL_6       (0.0F)
#define rtCP_Constant_Value_EL_7       (0.0F)
#define rtCP_Constant_Value_EL_8       (5.0E-7F)
#define rtCP_Constant_Value_EL_9       (0.001F)
#define rtCP_Constant_Value_EL_10      (1.0F)
#define rtCP_Constant_Value_EL_11      (0.0F)
#define rtCP_Constant_Value_EL_12      (0.0F)
#define rtCP_Constant_Value_EL_13      (0.0F)
#define rtCP_Constant_Value_EL_14      (0.0F)
#define rtCP_Constant_Value_EL_15      (1.0F)

/* Pooled Parameter (Mixed Expressions)
 * Referenced by:
 *   '<Root>/f32InitialAcceleration' (Parameter: Value)
 *   '<Root>/f32InitialBias' (Parameter: Value)
 *   '<Root>/f32InitialVelocity' (Parameter: Value)
 *   '<S1>/Switch1' (Parameter: Threshold)
 *   '<S1>/Switch2' (Parameter: Threshold)
 *   '<S1>/Delay1' (Parameter: InitialCondition)
 *   '<S1>/Delay2' (Parameter: InitialCondition)
 *   '<S1>/Delay3' (Parameter: InitialCondition)
 *   '<S1>/Delay4' (Parameter: InitialCondition)
 */
#define rtCP_pooled6                   (0.0F)

/* Computed Parameter: Constant4_Value
 * Referenced by: '<S1>/Constant4' (Parameter: Value)
 */
#define rtCP_Constant4_Value_EL_0      (1.0F)
#define rtCP_Constant4_Value_EL_1      (0.0F)
#define rtCP_Constant4_Value_EL_2      (0.0F)
#define rtCP_Constant4_Value_EL_3      (0.0F)
#define rtCP_Constant4_Value_EL_4      (0.0F)
#define rtCP_Constant4_Value_EL_5      (1.0F)
#define rtCP_Constant4_Value_EL_6      (0.0F)
#define rtCP_Constant4_Value_EL_7      (0.0F)
#define rtCP_Constant4_Value_EL_8      (0.0F)
#define rtCP_Constant4_Value_EL_9      (0.0F)
#define rtCP_Constant4_Value_EL_10     (1.0F)
#define rtCP_Constant4_Value_EL_11     (0.0F)
#define rtCP_Constant4_Value_EL_12     (0.0F)
#define rtCP_Constant4_Value_EL_13     (0.0F)
#define rtCP_Constant4_Value_EL_14     (0.0F)
#define rtCP_Constant4_Value_EL_15     (1.0F)

/* Pooled Parameter (Expression: -1)
 * Referenced by:
 *   '<S2>/Gain' (Parameter: Gain)
 *   '<S2>/Gain1' (Parameter: Gain)
 */
#define rtCP_pooled7                   (-1.0F)

/* Computed Parameter: Switch_Threshold
 * Referenced by: '<Root>/Switch' (Parameter: Threshold)
 */
#define rtCP_Switch_Threshold          (((uint8_T)0U))

/* Invariant block signals (default storage) */
extern const ConstB_SensorFusion_hb4t_T SensorFusion_ConstB;

#endif                                 /* RTW_HEADER_SensorFusion_private_h_ */

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
