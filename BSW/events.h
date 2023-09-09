/****************************************************************************************************
*  File name: events.h
*
*  Purpose: Definitions of system events.
*  This file defines the events in the system that the HSM will act upon.
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

#ifndef EVENTS_H__
#define EVENTS_H__

#include "parameter.h"

/*      Include Files */

/*      Public Type Definitions */

/* brief A list of possible status codes that can be returned by a call to Events_PostEvent()*/
typedef enum
{
    POSTED,           /* the event was successfully posted */
    BAD_PARAM,        /* the value of the event passed in was not within the bounds of the EVENT enum */
    EVENT_PENDING     /* the EventFlag is still set, indicating the last posted event has not been processed yet */
} EVT_STATUS; 


/* brief A list of all possible system level events with START_OF_LIST representing the beginning of the enum and INVALID_EVENT representing the end of the enum.*/
typedef enum
{
    EVENT_INIT,                 /* place holder for beginning of the list (bounds checking) */
    EVENT_INIT_DONE,            /* event representing the initialization mode was completed and no faults were detected */
    EVENT_FAULT_DETECTED,       /* event representing a fault was detected */
    EVENT_FAULT_CLEARED,        /* event representing a fault was cleared */
    EVENT_BOOT_DONE,            /* event representing that booT mode was completed and no faults were detected */
    EVENT_POSITION_CMD,         /* event representing the reception of a new position command */
    EVENT_POSITION_COMPLETE,    /* event representing the flap has reached the commanded position */
    EVENT_ENTER_RIG_MODE,       /* event representing the GSE commanded to enter RIG Mode. */
    EVENT_ENTER_TEST_MODE,      /* event representing the GSE commanded to enter TEST Mode. */
    EVENT_INVALID_EVENT,        /* value representing no current event posted, or for bounds checking */
    NUMBER_OF_EVENTS = EVENT_INVALID_EVENT,
} EVENT; 

/* brief The basic structure for passing events into the HSM */
typedef struct 
{
    EVENT eID;           /* event ID */
    unsigned char data;  /* placeholder for possible data passing */
}Evt; 

/*      Public Variable Declarations */

extern Evt SystemEvent;  

extern bool EventFlag;

/*      Public ROM Constants */

/*      Public Defines */

/*      Public Interface Function Prototypes */

EVT_STATUS Events_PostEvent(EVENT, unsigned char);
void Events_CheckEvent(void);

#endif

/* End of events.h. */
