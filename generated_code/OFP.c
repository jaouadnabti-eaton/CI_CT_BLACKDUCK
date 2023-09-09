/*
 *   Copyright © 2017 Eaton Corporation. Proprietary and Company Confidential.
 *
 *   File:        OFP.c
 *   Module Name: OFP
 *   Module Ver:  10.7
 *   Date:        Thu Jul 20 13:12:48 2023
 */

#include "OFP.h"
#include "ASW_BSW_I.h"
#include <string.h>
#include "ASW.h"

/* Block states (default storage) */
DW_OFP_T OFP_DW;

/*
 *   Function:    OFP_step
 *   Model name:  OFP
 *   Description: Model step function
 *   Arguments:   void
 *   Return:      void
 */
void OFP_step(void)
{
  /* local block i/o variables */
  ASW_DATA_t rtb_G_ASW_DATA;
  BSW_DATA_t rtb_GetBSWDataCCaller;

  /* CCaller: '<Root>/Get BSW Data C Caller' */
  ASW_getBSWdata_Wrapper(&rtb_GetBSWDataCCaller);

  /* ModelReference: '<Root>/ASW' */
  ASW(&rtb_GetBSWDataCCaller, &rtb_G_ASW_DATA, &(OFP_DW.ASW_InstanceData.rtdw));

  /* CCaller: '<Root>/Set BSW Data C Caller' incorporates:
   *  ModelReference: '<Root>/ASW'
   */
  ASW_setBSWdata_Wrapper(&rtb_G_ASW_DATA);
}

/*
 *   Function:    OFP_initialize
 *   Model name:  OFP
 *   Description: Model initialize function
 *   Arguments:   void
 *   Return:      void
 */
void OFP_initialize(void)
{
  /* Registration code */

  /* states (dwork) */
  (void) memset((void *)&OFP_DW, 0,
                sizeof(DW_OFP_T));

  /* Model Initialize function for ModelReference Block: '<Root>/ASW' */
  ASW_initialize(&(OFP_DW.ASW_InstanceData.rtdw));

  /* SystemInitialize for ModelReference: '<Root>/ASW' */
  ASW_Init(&(OFP_DW.ASW_InstanceData.rtdw));
}

/* End of File.  Copyright © Eaton Corporation. Proprietary and Company Confidential. */
