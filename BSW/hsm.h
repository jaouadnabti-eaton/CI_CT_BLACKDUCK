/****************************************************************************************************
*  File name: hsm.h
*
*  Purpose: Interface for the HSM.
*  This file describes the public interface to the Hierarchical state machine
*  for the MCU software.
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

#ifndef _HSM_H_
#define _HSM_H_

/*      Include Files */
#include "events.h"

/* Public Type Definitions */

/* brief Enumeration of state IDs */
typedef enum
{
    INVALID_STATE = 0,    /*  ID for Invalid State */
    INITIALIZATION_MODE,  /*  ID for Initialization mode */
    BOOT_MODE,            /*  ID for Boot Mode */
    RIG_MODE,             /*  ID for Rig Mode */
    TEST_MODE,            /*  ID for Test Mode */
    IDLE_MODE,            /*  ID for Idle Mode */
    RUN_MODE,             /*  ID for Run Mode */
    FAS_FAIL,             /*  ID for Fail Mode */
    NUM_STATES            /*  ID for Number of States */
} STATE_ID;

/*  Public Variable Declarations */

/*  Public ROM Constants */

/*  Public Defines */

/*  Public Interface Function Prototypes */
void Hsm_OnStart(STATE_ID * mode);
void Hsm_OnEvent(STATE_ID * mode, Evt const * event);

/*  Private Function Prototypes */

#endif

/* End of hsm.h */
