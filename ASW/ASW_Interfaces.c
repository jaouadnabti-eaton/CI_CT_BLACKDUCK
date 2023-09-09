/****************************************************************************************************
*  File name: ASW_Interfaces.h
*
*  Purpose: Interface for the ASW
*  This file describes the Public Interface for the ASW.  The interface provides
*  routines necessary for passing data between the ASW and BSW.
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

/******************************************** INCLUDES *********************************************/
#include "ASW_BSW_I.h"

/***************************************** GLOBAL VARIABLES *****************************************/
BSW_DATA_t G_BSW_DATA = {0};
ASW_DATA_t G_ASW_DATA = {0};

/***************************************** LOCAL VARIABLES *****************************************/

/**************************************** FUNCTION PROTYPE *****************************************/
void ASW_getBSWdata_Wrapper(BSW_DATA_t *Bus);
void ASW_setBSWdata_Wrapper(const ASW_DATA_t *Bus);



/*************************************************************************************************\
* Function: ASW_getBSWdata_Wrapper
*
* Purpose: This function sets the ASW Data(Simulink) with the data from the base software BSW.
*
* Input(s):  Global - G_BSW_DATA                         
* Output(s): Bus - BSW_DATA_t structure pointer      
*
\*************************************************************************************************/
void ASW_getBSWdata_Wrapper( BSW_DATA_t *Bus)
{
    *Bus = G_BSW_DATA;
}

/*************************************************************************************************\
* Function: ASW_setBSWdata_Wrapper
*
* Purpose: This function sets the base software BSW data with the data calculated in the ASW (Simulink).
*
* Input(s):  Bus - ASW_DATA_t structure pointer                         
* Output(s): Global - ASW_DATA
\*************************************************************************************************/
void ASW_setBSWdata_Wrapper(const ASW_DATA_t *Bus)
{
    G_ASW_DATA = *Bus;
}



