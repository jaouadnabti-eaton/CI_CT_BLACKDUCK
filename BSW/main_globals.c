/****************************************************************************************************
*  File name: main_globals.c
*
*  Purpose: Global variables used in main.c
*
*  Copyright Notice:
*  All source code and data contained in this file is Proprietary and
*  Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
*  disclosed; in whole or in part, without the express written permission of
*  Eaton Aerospace.
*
*  Copyright (c) 2022 - Eaton Aerospace Group, All Rights Reserved.
*
*  Author            Date       CR#          Description
*  ------           ---------   ------     ---------------------------------------
*  Adam Bouwens     12/07/2022  N/A        Ported to MCU
*
****************************************************************************************************/

/*      Include Files
*/

#include "timer.h"
#include "typedefs.h"
#include "parameter.h"

/*      Global Variables
*/
bool InitiateStateWrite = false;
bool StoreRigDataInNVM = false;
Uint32 OneMsTimer = 0;
Uint32 FrameTimer = 0;
float32_t f32MotorCurrentAmps = 0.0F;

tMcuStatus_t tMcuStatus = { 0U };
tMcuFault_t  tMcuFault = { 0U };
tMcuMaint_t  tMcuMaint = { 0U };
t64Types_t   t64Debug = { 0ULL };

float32_t G_pwmMax;
t64Types_t t64SpiSendCounters1 = { 0 };
t64Types_t t64SpiSendCounters2 = { 0 };
t64Types_t t64SpiRecvCounters1 = { 0 };
t64Types_t t64SpiRecvCounters2 = { 0 };

/* end main_globals.c */
