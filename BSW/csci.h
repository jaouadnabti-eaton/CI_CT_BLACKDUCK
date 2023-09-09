/****************************************************************************************************
*  File name: csci.h
*
*  Purpose: Interface of the CSCI
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

#ifndef CSCI_H_
#define CSCI_H_

/******************** INCLUDES *******************************************************************/
#include "typedefs.h"
//#include "stdint.h"

#define CSCI_NAME_SIZE          16U
#define HW_PART_SIZE            16U
#define AIRCRAFT_CONFIG_SIZE    3U

/* Max size of structure is 0x30 words as defined by APPHDR in linker file */
/******************** STRUCTS ********************************************************************/
/* CSCI Header structure */
typedef struct
{
    uint32_t CRC;                                /* CRC (IEEE 802.3 CRC-32) of the code image in memory */
    void (*CodeEntryPoint)(void);                /* Memory address of where the executable code begins */
    uint32_t ImageSize;                          /* # of bytes in the code image including CSCI header */
    uint16_t HeaderSize;                         /* Contains the number of bytes in the CSCI header */
    uint16_t BCID_Group;                         /* Box Compatibility ID: Group */
    uint16_t BCID_Number;                        /* Box Compatibility ID: Number */
    char16_t CSCI_Name[CSCI_NAME_SIZE];          /* CSCI Name */
    tSwVersion_t tSwVersion;                     /* Software Version:  Major, Minor, Revision, Build */
    char16_t HWUPN[HW_PART_SIZE];                /* HW Unit Part Number EG: 72515-nn */
    uint16_t AircraftType[AIRCRAFT_CONFIG_SIZE]; /* UNUSED: A field in which to store the aircraft type */
} CSCI_TypeHeader;

/* Image size variable provided by linker */
extern uint16_t CSCI_Size;
extern const CSCI_TypeHeader CSCI_APP_HEADER;
extern const CSCI_TypeHeader CSCI_PDI_HEADER;
extern const CSCI_TypeHeader CSCI_BLD_HEADER;

#endif /*CSCI_H_*/
