/**************************************************************************************************
 *  File name:   pdi.c
 *  Purpose :    The pdi.c file contains the definitions of the Parameter Data Items (PDIs)
 *  Project Name: MCU OFP
 *  Copyright Notice:
 *  All source code and data contained in this file is Proprietary and Confidential to
 *  Eaton Aerospace, and must not be reproduced, transmitted, or disclosed; in whole or in part,
 *  without the express written permission of Eaton Aerospace.
 *
 *  Copyright (c) 2022 Eaton Aerospace Group, All Rights Reserved.
 *
 *  Author          Date            (CR#)         Description
 *  ------          ---------       ---------     -------------------------
 *  Adam Bouwens    12/07/2022      N/A           Refactored for MCU
 *
 **************************************************************************************************/


/******************** INCLUDES *******************************************************************/
#include "pdi.h"

/******************** DEFINES ********************************************************************/

/******************** CONSTANTS ******************************************************************/
#pragma DATA_SECTION(tPdi, "PDI_CSCI_DATA");
const tPdi_t tPdi;  /* Parameter Data Item Structure in Flash Memory */

/* VARIABLES ************************************************************************************/

/* FUNCTION PROTOTYPES **************************************************************************/

/* FUNCTION DEFINITIONS *************************************************************************/

