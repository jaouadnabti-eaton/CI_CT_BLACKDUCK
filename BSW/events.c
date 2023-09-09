/****************************************************************************************************
*  File name: events.c
*
*  Purpose: System event maintenance.
*    This file implements routines for formatting events for processing by the mode control software.
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
#include "events.h"
#include "mcu.h"
#include "gpio.h"

/*      Local Type Definitions */

/*      Local Defines */

/*      Global Variables */
Evt SystemEvent = {NUMBER_OF_EVENTS, 0};   /* Global structure for storing the system event that occurred */
bool EventFlag = false;                 /* Flag for signaling that a system event has occurred.*/

/*      Local ROM Constants */

/*      Local Variable Declarations */

/*      Local Function Prototypes */

/*      Function Definitions */

/**
    brief Post a System Event

    Purpose:
        This routine posts an event for the mode control module.  A check is made
        on the EventFlag to see if an event is still pending processing.  If it is,
        currently this routine will not queue the new event but rather it will not 
        assign it and notify the caller that an event is pending.  The exception to this is
        if the event passed in represents a Fault detected.  In this case, the current
        pending event will be overwritten and replaced with the Fault event, giving it priority.

    param[in] event  the event to post
    param[in] event_data  any data that needs to go along with an event

    return The status of the posting attempt.

*/
EVT_STATUS Events_PostEvent(EVENT event, unsigned char event_data)
{
    
    EVT_STATUS status;

    /* PATH(Events_PostEvent,A); */
    
    /* shall check the bounds of the value of the event passed in for validity */
    if ((event > EVENT_INIT) && (event < NUMBER_OF_EVENTS))
    {
        /* PATH(Events_PostEvent,B); */

        /* shall check the EventFlag to see if it is set.  If it is set, then an event
           has been previously posted but not processed for some reason, so take no action
           unless the new event is FaultDetected. */
        if ((EventFlag == true) && (event != EVENT_FAULT_DETECTED))
        {
            /* PATH(Events_PostEvent,C); */

            /* shall return a value of EVENT_PENDING if the newly posted event is not FaultDetected
               and the EventFlag is set. */
            status = EVENT_PENDING;
        }   /* if (EventFlag == true) && (event != FaultDetected) */
        else
        {
            /* PATH(Events_PostEvent,D); */

            /* shall assign the new event to SystemEvent if it is a valid system event, 
               and the EventFlag is not set or the event is FaultDetected */
            SystemEvent.eID = event;
            SystemEvent.data = event_data;
            /* shall set the EventFlag indicating an event has been posted and is ready to be processed in allframes_proc() */
            EventFlag = true;
            /* shall return a value of POSTED to indicate success to the caller */
            status = POSTED;
        } /* else (EventFlag == true) && (event != FaultDetected) */
    }	/* if ((event > START_OF_LIST) && (event < INVALID_EVENT)) */
    else
    {
        /* PATH(Events_PostEvent,E); */

        /* shall return a value of BAD_PARAM if the event passed in is not within the bounds of the EVENT enumeration */
        status = BAD_PARAM;
    }  /* else ((event > START_OF_LIST) && (event < INVALID_EVENT)) */

    /* PATH(Events_PostEvent,F); */
    return (status);
}  /* end of Events_PostEvent */


/**
    brief Check for a System Event

    Purpose:
        A check is made on the EventFlag to see if an event has been posted.  If one has,
        then pass it into the mode control linked list of event handlers.  Once the event
        has been processed, clear the global EventFlag as well as the SystemEvent struct.

    return
        None.
*/
void Events_CheckEvent(void)
{
    /* PATH(Events_CheckEvent,A); */
    
    if (EventFlag == true)
    {
        /* PATH(Events_CheckEvent,B); */

        /* shall process event */
        Hsm_OnEvent(&G_eMcuMode, &SystemEvent);

        /* shall clear "EventFlag" and SystemEvent struct */
        EventFlag = false;
        SystemEvent.data = 0;
        SystemEvent.eID = EVENT_INVALID_EVENT;
    }   /* if EventFlag == true */

    /* PATH(Events_CheckEvent,C); */
}  /* end of Events_CheckEvent */

