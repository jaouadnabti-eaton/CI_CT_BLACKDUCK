/****************************************************************************************************
*  File name: brake.h
*
*  Purpose: Interface for brake control driver.
*
*      This file describes the Public Interface for the brake control driver.  The
*      interface provides routines necessary to abstract the upper-level software from
*      the hardware details of controlling the brake.
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

#ifndef BRAKE_H__
#define BRAKE_H__


/*      Include Files */

/*      Public Type Definitions */

/*      Public Variable Declarations */
/*Commenting out as Brake functionality is deferred for MCU*/
#if 0
extern Uint32 BrakeCounter;

/*      Public ROM Constants */

/*      Public Defines */

/*      Public Interface Function Prototypes */

/*
  Brake Driver
  
  brief The brake control driver is an abstraction for controlling the brake via PWM
        and logic output.
*/


void Brake_SetPWM(Uint16 dutycycle);
#endif
void Brake_Init(void);
void Brake_Control(void);
void Brake_Deactivate(void);
void Brake_Activate(void);


#endif
/* end brake.h */

