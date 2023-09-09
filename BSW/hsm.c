/****************************************************************************************************
*  File name: hsm.c
*
*  Purpose: Framework for a Hierarchical state machine.
*      This file contains the API definition for the HSM.
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

/*      Include Files */
#include "hsm.h"
#include "parameter.h"
#include "mcu.h"

/*      Local Type Definitions */

/*      Local Defines */

/*      Global Variables */

/*      Local ROM Constants */

/*      Local Variable Declarations */

/*      Local Function Prototypes */

/*      Function Definitions */

/**
    brief Starting point for the HSM.

    Purpose:
        This routine starts the HSM.
  
    param[in] mode  pointer to current mode variable.

    Global Data Referenced:
         None.
    
    return void
    
    Preconditions and Assumptions:
        None.

*/
void Hsm_OnStart(STATE_ID * mode)
{
    /* PATH(Hsm_OnStart,A); */
    
	if (mode != 0)
	{
        /* PATH(Hsm_OnStart,C); */
        * mode = INITIALIZATION_MODE;
        SystemEvent.eID = EVENT_INVALID_EVENT;
        SystemEvent.data = 0;
        McuEntryEventHandler(* mode);
	}

    /* PATH(Hsm_OnStart,B); */
}


/**
    brief HSM event handler.

    Purpose:
        This routine is the transition function for the state machine.
  
    param[in] mode  pointer to the current mode the state machine is in.
    param[in] event event that has been posted for processing.

    Global Data Referenced:
        None.
    
    return void
    
    Preconditions and Assumptions:
        None.
*/
void Hsm_OnEvent(STATE_ID * mode, Evt const * event)
{
    STATE_ID old = * mode;

    /* PATH(Hsm_OnEvent,A); */

	if (mode != 0 && event != 0)
	{
        /* PATH(Hsm_OnEvent,M); */
        switch (* mode)
        {
            case INITIALIZATION_MODE:
                /* PATH(Hsm_OnEvent,B); */
                * mode = McuInitModeHandler(event);
                break;
            case BOOT_MODE:
                /* PATH(Hsm_OnEvent,C); */
                * mode = McuBootModeHandler(event);
                break;
            case RIG_MODE:
                /* PATH(Hsm_OnEvent,E); */
                * mode = McuRigModeHandler(event);
                break;
            case TEST_MODE:
                /* PATH(Hsm_OnEvent,F); */
                * mode = McuTestModeHandler(event);
                break;
            case IDLE_MODE:
                /* PATH(Hsm_OnEvent,G); */
                * mode = McuIdleModeHandler(event);
                break;
            case RUN_MODE:
                /* PATH(Hsm_OnEvent,H); */
                * mode = McuRunModeHandler(event);
                break;
            case FAS_FAIL:
                /* PATH(Hsm_OnEvent,I); */
                * mode = McuFasFailHandler(event);
                break;
            default:
                /* PATH(Hsm_OnEvent,J); */
                break;
        }

        if (old != (* mode))
        {
            /* PATH(Hsm_OnEvent,K); */
            McuEntryEventHandler(* mode);
        }
    }
    /* PATH(Hsm_OnEvent,L); */
}

/* end hsm.c */
