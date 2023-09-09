/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        ASW.h
 *   Module Name: ASW
 *   Module Ver:  10.44
 *   Date:        Thu Jul 20 13:12:34 2023
 */

#ifndef RTW_HEADER_ASW_h_
#define RTW_HEADER_ASW_h_
#ifndef ASW_COMMON_INCLUDES_
#define ASW_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* ASW_COMMON_INCLUDES_ */

#include "ASW_BSW_I.h"
#include "ASW_types.h"
#include "ReadfromBSW.h"
#include "SensorFusion.h"
#include "PID_Loop.h"
#include "WriteToBSW.h"
#include <string.h>

/* Block states (default storage) for model 'ASW' */
typedef struct {
  MdlrefDW_SensorFusion_T SensorFusion_InstanceData;/* '<Root>/Sensor Fusion' (DWork 1) */
  MdlrefDW_PID_Loop_T SpeedRegulator_InstanceData;/* '<Root>/Speed Regulator' (DWork 1) */
} DW_ASW_fwu4_T;

typedef struct {
  DW_ASW_fwu4_T rtdw;
} MdlrefDW_ASW_T;

/* Model reference registration function */
extern void ASW_initialize(DW_ASW_fwu4_T *localDW);
extern void ASW_Init(DW_ASW_fwu4_T *localDW);
extern void ASW(const BSW_DATA_t *rtu_G_BSW_DATA, ASW_DATA_t *rty_G_ASW_OUT,
                DW_ASW_fwu4_T *localDW);

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
 * '<Root>' : 'ASW'
 */

/*-
 * Requirements for '<Root>': ASW

 */
#endif                                 /* RTW_HEADER_ASW_h_ */

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
