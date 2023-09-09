/****************************************************************************************************
 * File name: Defines.h
 *
 * Purpose : This file contains all common defines for the ARINC825 interface
 *
 * Copyright Notice:
 * All source code and data contained in this file is Proprietary and Confidential to Eaton Aerospace,
 * and must not be reproduced, transmitted, or disclosed; in whole or in part, without the express written
 * permission of Eaton Aerospace.
 *
 * Copyright (c) 2022 - Eaton Aerospace Group, All Rights Reserved.
 *
 * Author            Date       CR#              Description
 * ------          ---------    ------        -------------------------------------
 * Adam Bouwens     12/07/2022  NA             Port to MCU
 ******************************************************************************/



#ifndef DEFINES_H
#define DEFINES_H



/******************** DEFINES ********************/
#ifndef NULL
#define NULL                    (0U)
#endif


#ifdef VV_TEST
/* Assembly label for breakpoints used for testing */
#define TEST_BREAKPOINT(label)    asm (label);
#else

/*Justification for Coding Standard Deviation:
 *    This macros are empty - there is no function macro
 *    the function macros are used only during debug when
 *    the "VV_TEST" compiler switch is enabled.
 *    An exception to MISRA Rule 19.7 is required.
 */

#define TEST_BREAKPOINT(label)    //lint !e961
#endif

#endif
