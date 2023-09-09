/******************************************************************************
 *  File name: PDI_Mgr.h
 *
 *  Purpose : Data types, constants, and function prototypes for the PDI
 *         manager.
 *
 *  Copyright Notice:
 *  All source code and data contained in this file is Proprietary and
 *  Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
 *  disclosed; in whole or in part, without the express written permission of
 *  Eaton Aerospace.
 *
 *  Copyright (c) 2022 Eaton Aerospace Group, All Rights Reserved.
 *
 *  Author          Date            (CR#)         Description
 *  ------          ---------       ---------     -------------------------
 *  Adam Bouwens    12/07/2022      N/A           Refactored for MCU
 ******************************************************************************/
#ifndef PDI_MGR_H_
#define PDI_MGR_H_

/* INCLUDES **************************************************************************************/
#include "pdi.h"
#include "typedefs.h"
#include "F2837xD_device.h"

/* PUBLIC DEFINES ********************************************************************************/

/* PUBLIC TYPEDEFS *******************************************************************************/

/* PUBLIC CONSTANSTS *****************************************************************************/

/* PUBLIC VARIABLES ******************************************************************************/

/* PUBLIC FUNCTION PROTOTYPES ********************************************************************/
bool_t PDI_Mgr_RangeCheck(void);

#endif /* PDI_MGR_H_*/
