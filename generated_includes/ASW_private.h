/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        ASW_private.h
 *   Module Name: ASW
 *   Module Ver:  10.44
 *   Date:        Thu Jul 20 13:12:34 2023
 */

#ifndef RTW_HEADER_ASW_private_h_
#define RTW_HEADER_ASW_private_h_
#include "rtwtypes.h"
#include "typedefs.h"
#include "ASW_types.h"

/* Constant parameters (default storage) */
typedef struct {
  uint16_t pooled1;                    /* Pooled Parameter (Expression: 0)
                                        * Referenced by:
                                        *   '<Root>/Constant3' (Parameter: Value)
                                        *   '<Root>/Constant4' (Parameter: Value)
                                        *   '<Root>/Constant8' (Parameter: Value)
                                        */
  bool_t pooled2;                      /* Pooled Parameter (Expression: false)
                                        * Referenced by:
                                        *   '<Root>/Constant1' (Parameter: Value)
                                        *   '<Root>/Constant2' (Parameter: Value)
                                        *   '<Root>/Constant5' (Parameter: Value)
                                        *   '<Root>/Constant6' (Parameter: Value)
                                        *   '<Root>/Constant7' (Parameter: Value)
                                        */
} ConstP_ASW_T;

#define rtCP_pooled1                   (ASW_ConstP.pooled1)
#define rtCP_pooled2                   (ASW_ConstP.pooled2)

/* Constant parameters (default storage) */
extern const ConstP_ASW_T ASW_ConstP;

#endif                                 /* RTW_HEADER_ASW_private_h_ */

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
