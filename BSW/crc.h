/****************************************************************************************************
*  File name: crc.h
*
*  Purpose: Interface for the CRC calculations
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

#ifndef CRC_H_
#define CRC_H_

/******************** INCLUDES *******************************************************************/
#include "typedefs.h"
//#include "stdint.h"


/******************** DEFINES ********************************************************************/
#define CRC32TBL_SIZE    256
#define CRC16TBL_SIZE    256

/******************** EXTERNS ********************************************************************/
#define CRC32_INIT              0xFFFFFFFFU /* Initial CRC value for the algorithm */
#define CRC32_XORFINAL          0xFFFFFFFFU /* Final XOR value */
#define CRC32_MASK              0xFFFFFFFFU

#define CRC16_INIT            0xFFFFU     /* Initial CRC value for the CRC16 algorithm */
#define CRC16_XORFINAL        0x0U        /* Final XOR value for CRC16 algorithm*/
#define CRC16_MASK            0xFFFFU

#define BYTE_MASK             0xFFU
#define SHIFT_ONE_BYTE        8U
#define SHIFT_BYTE_IN_LONG    24U

extern const uint32_t G_CRC32_table[CRC32TBL_SIZE];

/******************** EXTERNS ********************************************************************/


/******************** FUNCTION PROTOTYPES ********************************************************/
uint32_t CRC32Api_GetCRC(const uint16_t crcapi_getcrcbuffer[],
                       uint32_t       crcapi_sizeinbytes);
uint16_t CRC16Api_GetCRC(const uint16_t crcapi_getcrcbuffer[],
                       uint32_t       crcapi_sizeinbytes);
#endif /* CRC_H_ */
