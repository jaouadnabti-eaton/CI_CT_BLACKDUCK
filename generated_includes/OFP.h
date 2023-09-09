/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        OFP.h
 *   Module Name: OFP
 *   Module Ver:  10.7
 *   Date:        Thu Jul 20 13:12:48 2023
 */

#ifndef RTW_HEADER_OFP_h_
#define RTW_HEADER_OFP_h_
#ifndef OFP_COMMON_INCLUDES_
#define OFP_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* OFP_COMMON_INCLUDES_ */

#include "OFP_types.h"
#include "ASW.h"
#include <string.h>

/* user code (top of header file) */
#include "ASW_BSW_I.h"
#include "typedefs.h"

/* Block states (default storage) for system '<Root>' */
typedef struct {
  MdlrefDW_ASW_T ASW_InstanceData;     /* '<Root>/ASW' (DWork 1) */
} DW_OFP_T;

/* Block states (default storage) */
extern DW_OFP_T OFP_DW;

/* Model entry point functions */
extern void OFP_initialize(void);
extern void OFP_step(void);

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
 * '<Root>' : 'OFP'
 */

/*-
 * Requirements for '<Root>': OFP

 */
#endif                                 /* RTW_HEADER_OFP_h_ */

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
