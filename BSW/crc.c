/****************************************************************************************************
*  File name: crc.c
*
*  Purpose: Logic to compute CRCs for contiguous sections of memory
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

/******************** INCLUDES *******************************************************************/
#include "F28x_Project.h"
#include "crc.h"
#include "PDI_Mgr.h"

/******************** DEFINES ********************************************************************/

/*
 * Constant: CRC32TBL_TABLE
 *
 * Purpose: This array serves as a lookup table for all possible remainders.
 * Note: Polynomial used to calculate tables = 0x4C11DB7 without leading '1' bit
 */
const uint32_t G_CRC32_table[CRC32TBL_SIZE] =
{
    0x0U,         0x4c11db7U,  0x9823b6eU,  0xd4326d9U, 0x130476dcU, 0x17c56b6bU, 0x1a864db2U, 0x1e475005U,
    0x2608edb8U, 0x22c9f00fU, 0x2f8ad6d6U, 0x2b4bcb61U, 0x350c9b64U, 0x31cd86d3U, 0x3c8ea00aU, 0x384fbdbdU,
    0x4c11db70U, 0x48d0c6c7U, 0x4593e01eU, 0x4152fda9U, 0x5f15adacU, 0x5bd4b01bU, 0x569796c2U, 0x52568b75U,
    0x6a1936c8U, 0x6ed82b7fU, 0x639b0da6U, 0x675a1011U, 0x791d4014U, 0x7ddc5da3U, 0x709f7b7aU, 0x745e66cdU,
    0x9823b6e0U, 0x9ce2ab57U, 0x91a18d8eU, 0x95609039U, 0x8b27c03cU, 0x8fe6dd8bU, 0x82a5fb52U, 0x8664e6e5U,
    0xbe2b5b58U, 0xbaea46efU, 0xb7a96036U, 0xb3687d81U, 0xad2f2d84U, 0xa9ee3033U, 0xa4ad16eaU, 0xa06c0b5dU,
    0xd4326d90U, 0xd0f37027U, 0xddb056feU, 0xd9714b49U, 0xc7361b4cU, 0xc3f706fbU, 0xceb42022U, 0xca753d95U,
    0xf23a8028U, 0xf6fb9d9fU, 0xfbb8bb46U, 0xff79a6f1U, 0xe13ef6f4U, 0xe5ffeb43U, 0xe8bccd9aU, 0xec7dd02dU,
    0x34867077U, 0x30476dc0U, 0x3d044b19U, 0x39c556aeU, 0x278206abU, 0x23431b1cU, 0x2e003dc5U, 0x2ac12072U,
    0x128e9dcfU, 0x164f8078U, 0x1b0ca6a1U, 0x1fcdbb16U,  0x18aeb13U,  0x54bf6a4U,  0x808d07dU,  0xcc9cdcaU,
    0x7897ab07U, 0x7c56b6b0U, 0x71159069U, 0x75d48ddeU, 0x6b93dddbU, 0x6f52c06cU, 0x6211e6b5U, 0x66d0fb02U,
    0x5e9f46bfU, 0x5a5e5b08U, 0x571d7dd1U, 0x53dc6066U, 0x4d9b3063U, 0x495a2dd4U, 0x44190b0dU, 0x40d816baU,
    0xaca5c697U, 0xa864db20U, 0xa527fdf9U, 0xa1e6e04eU, 0xbfa1b04bU, 0xbb60adfcU, 0xb6238b25U, 0xb2e29692U,
    0x8aad2b2fU, 0x8e6c3698U, 0x832f1041U, 0x87ee0df6U, 0x99a95df3U, 0x9d684044U, 0x902b669dU, 0x94ea7b2aU,
    0xe0b41de7U, 0xe4750050U, 0xe9362689U, 0xedf73b3eU, 0xf3b06b3bU, 0xf771768cU, 0xfa325055U, 0xfef34de2U,
    0xc6bcf05fU, 0xc27dede8U, 0xcf3ecb31U, 0xcbffd686U, 0xd5b88683U, 0xd1799b34U, 0xdc3abdedU, 0xd8fba05aU,
    0x690ce0eeU, 0x6dcdfd59U, 0x608edb80U, 0x644fc637U, 0x7a089632U, 0x7ec98b85U, 0x738aad5cU, 0x774bb0ebU,
    0x4f040d56U, 0x4bc510e1U, 0x46863638U, 0x42472b8fU, 0x5c007b8aU, 0x58c1663dU, 0x558240e4U, 0x51435d53U,
    0x251d3b9eU, 0x21dc2629U, 0x2c9f00f0U, 0x285e1d47U, 0x36194d42U, 0x32d850f5U, 0x3f9b762cU, 0x3b5a6b9bU,
    0x315d626U,   0x7d4cb91U,  0xa97ed48U,  0xe56f0ffU, 0x1011a0faU, 0x14d0bd4dU, 0x19939b94U, 0x1d528623U,
    0xf12f560eU, 0xf5ee4bb9U, 0xf8ad6d60U, 0xfc6c70d7U, 0xe22b20d2U, 0xe6ea3d65U, 0xeba91bbcU, 0xef68060bU,
    0xd727bbb6U, 0xd3e6a601U, 0xdea580d8U, 0xda649d6fU, 0xc423cd6aU, 0xc0e2d0ddU, 0xcda1f604U, 0xc960ebb3U,
    0xbd3e8d7eU, 0xb9ff90c9U, 0xb4bcb610U, 0xb07daba7U, 0xae3afba2U, 0xaafbe615U, 0xa7b8c0ccU, 0xa379dd7bU,
    0x9b3660c6U, 0x9ff77d71U, 0x92b45ba8U, 0x9675461fU, 0x8832161aU, 0x8cf30badU, 0x81b02d74U, 0x857130c3U,
    0x5d8a9099U, 0x594b8d2eU, 0x5408abf7U, 0x50c9b640U, 0x4e8ee645U, 0x4a4ffbf2U, 0x470cdd2bU, 0x43cdc09cU,
    0x7b827d21U, 0x7f436096U, 0x7200464fU, 0x76c15bf8U, 0x68860bfdU, 0x6c47164aU, 0x61043093U, 0x65c52d24U,
    0x119b4be9U, 0x155a565eU, 0x18197087U, 0x1cd86d30U,  0x29f3d35U,  0x65e2082U,  0xb1d065bU,  0xfdc1becU,
    0x3793a651U, 0x3352bbe6U, 0x3e119d3fU, 0x3ad08088U, 0x2497d08dU, 0x2056cd3aU, 0x2d15ebe3U, 0x29d4f654U,
    0xc5a92679U, 0xc1683bceU, 0xcc2b1d17U, 0xc8ea00a0U, 0xd6ad50a5U, 0xd26c4d12U, 0xdf2f6bcbU, 0xdbee767cU,
    0xe3a1cbc1U, 0xe760d676U, 0xea23f0afU, 0xeee2ed18U, 0xf0a5bd1dU, 0xf464a0aaU, 0xf9278673U, 0xfde69bc4U,
    0x89b8fd09U, 0x8d79e0beU, 0x803ac667U, 0x84fbdbd0U, 0x9abc8bd5U, 0x9e7d9662U, 0x933eb0bbU, 0x97ffad0cU,
    0xafb010b1U, 0xab710d06U, 0xa6322bdfU, 0xa2f33668U, 0xbcb4666dU, 0xb8757bdaU, 0xb5365d03U, 0xb1f740b4U
};

/*
 * Constant: CRC16TBL_TABLE
 *
 * Purpose: This array serves as a lookup table for all possible remainders.
 * Note: Polynomial used to calculate tables = 0x1021 without leading '1' bit
 */
static const uint16_t G_CRC16_table[CRC16TBL_SIZE] =
{
        0x0U,    0x1021U, 0x2042U, 0x3063U, 0x4084U, 0x50a5U, 0x60c6U, 0x70e7U, 0x8108U, 0x9129U,
        0xa14aU, 0xb16bU, 0xc18cU, 0xd1adU, 0xe1ceU, 0xf1efU, 0x1231U, 0x210U,  0x3273U, 0x2252U,
        0x52b5U, 0x4294U, 0x72f7U, 0x62d6U, 0x9339U, 0x8318U, 0xb37bU, 0xa35aU, 0xd3bdU, 0xc39cU,
        0xf3ffU, 0xe3deU, 0x2462U, 0x3443U, 0x420U,  0x1401U, 0x64e6U, 0x74c7U, 0x44a4U, 0x5485U,
        0xa56aU, 0xb54bU, 0x8528U, 0x9509U, 0xe5eeU, 0xf5cfU, 0xc5acU, 0xd58dU, 0x3653U, 0x2672U,
        0x1611U, 0x630U,  0x76d7U, 0x66f6U, 0x5695U, 0x46b4U, 0xb75bU, 0xa77aU, 0x9719U, 0x8738U,
        0xf7dfU, 0xe7feU, 0xd79dU, 0xc7bcU, 0x48c4U, 0x58e5U, 0x6886U, 0x78a7U, 0x840U,  0x1861U,
        0x2802U, 0x3823U, 0xc9ccU, 0xd9edU, 0xe98eU, 0xf9afU, 0x8948U, 0x9969U, 0xa90aU, 0xb92bU,
        0x5af5U, 0x4ad4U, 0x7ab7U, 0x6a96U, 0x1a71U, 0xa50U,  0x3a33U, 0x2a12U, 0xdbfdU, 0xcbdcU,
        0xfbbfU, 0xeb9eU, 0x9b79U, 0x8b58U, 0xbb3bU, 0xab1aU, 0x6ca6U, 0x7c87U, 0x4ce4U, 0x5cc5U,
        0x2c22U, 0x3c03U, 0xc60U,  0x1c41U, 0xedaeU, 0xfd8fU, 0xcdecU, 0xddcdU, 0xad2aU, 0xbd0bU,
        0x8d68U, 0x9d49U, 0x7e97U, 0x6eb6U, 0x5ed5U, 0x4ef4U, 0x3e13U, 0x2e32U, 0x1e51U, 0xe70U,
        0xff9fU, 0xefbeU, 0xdfddU, 0xcffcU, 0xbf1bU, 0xaf3aU, 0x9f59U, 0x8f78U, 0x9188U, 0x81a9U,
        0xb1caU, 0xa1ebU, 0xd10cU, 0xc12dU, 0xf14eU, 0xe16fU, 0x1080U, 0xa1U,   0x30c2U, 0x20e3U,
        0x5004U, 0x4025U, 0x7046U, 0x6067U, 0x83b9U, 0x9398U, 0xa3fbU, 0xb3daU, 0xc33dU, 0xd31cU,
        0xe37fU, 0xf35eU, 0x2b1U,  0x1290U, 0x22f3U, 0x32d2U, 0x4235U, 0x5214U, 0x6277U, 0x7256U,
        0xb5eaU, 0xa5cbU, 0x95a8U, 0x8589U, 0xf56eU, 0xe54fU, 0xd52cU, 0xc50dU, 0x34e2U, 0x24c3U,
        0x14a0U, 0x481U,  0x7466U, 0x6447U, 0x5424U, 0x4405U, 0xa7dbU, 0xb7faU, 0x8799U, 0x97b8U,
        0xe75fU, 0xf77eU, 0xc71dU, 0xd73cU, 0x26d3U, 0x36f2U, 0x691U,  0x16b0U, 0x6657U, 0x7676U,
        0x4615U, 0x5634U, 0xd94cU, 0xc96dU, 0xf90eU, 0xe92fU, 0x99c8U, 0x89e9U, 0xb98aU, 0xa9abU,
        0x5844U, 0x4865U, 0x7806U, 0x6827U, 0x18c0U, 0x8e1U,  0x3882U, 0x28a3U, 0xcb7dU, 0xdb5cU,
        0xeb3fU, 0xfb1eU, 0x8bf9U, 0x9bd8U, 0xabbbU, 0xbb9aU, 0x4a75U, 0x5a54U, 0x6a37U, 0x7a16U,
        0xaf1U,  0x1ad0U, 0x2ab3U, 0x3a92U, 0xfd2eU, 0xed0fU, 0xdd6cU, 0xcd4dU, 0xbdaaU, 0xad8bU,
        0x9de8U, 0x8dc9U, 0x7c26U, 0x6c07U, 0x5c64U, 0x4c45U, 0x3ca2U, 0x2c83U, 0x1ce0U, 0xcc1U,
        0xef1fU, 0xff3eU, 0xcf5dU, 0xdf7cU, 0xaf9bU, 0xbfbaU, 0x8fd9U, 0x9ff8U, 0x6e17U, 0x7e36U,
        0x4e55U, 0x5e74U, 0x2e93U, 0x3eb2U, 0xed1U,  0x1ef0U
};


/*************************************************************************************************\
* Function: CRC32Api_GetCRC
*
* Purpose: This function returns the requested CRC value.
*
* Global Input(s):  None
* Global Output(s): None
* Input(s):         crcapi_getcrcbuffer - Specified buffer
*                   crcapi_sizeinbytes - Length of specified buffer
* Output(s):        CRCApi_GetCRCValue - CRC Value requested
*
\*************************************************************************************************/
uint32_t CRC32Api_GetCRC(const uint16_t crcapi_getcrcbuffer[], uint32_t crcapi_sizeinbytes)
{
    uint32_t crcapi_getcrcvalue = CRC32_INIT;
    uint32_t buffer_index       = 0U;
    uint32_t bytes_index;
    uint16_t crc_byte;
    uint16_t crc_table_index;

    /* This loop processes the least significant first 8 bits then the most significant 8 bits,
     * then increments the buffer to the next 16 bits */
    for (bytes_index = 0U; bytes_index < crcapi_sizeinbytes; bytes_index++)
    {
        /* Due to 16 bit addressing we have to process the upper and lower 8 bits separately */
        if ((bytes_index % BYTES_PER_WORD) == 0U) /* 1st Pass per 16 bit word */
        {
            /* Get most significant byte */
            crc_byte = (crcapi_getcrcbuffer[buffer_index] >> SHIFT_ONE_BYTE) & BYTE_MASK;
        }
        else /* 2nd pass per 16 bit word*/
        {
            /* Get least significant byte */
            crc_byte = crcapi_getcrcbuffer[buffer_index] & BYTE_MASK;
            /* Increment buffer onto next 16 bits */
            buffer_index++;
        }
        /* ^ operator denotes bitwise XOR */
        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_BYTE_IN_LONG) ^ crc_byte)\
                                     & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC32_table[crc_table_index];
    }
    /* ^ operator denotes bitwise XOR */
    crcapi_getcrcvalue ^= CRC32_XORFINAL;
    crcapi_getcrcvalue &= CRC32_MASK;

    return(crcapi_getcrcvalue);
}

/*************************************************************************************************\
* Function: CRC16Api_GetCRC
*
* Purpose: This function returns the requested CRC value.
*
* Global Input(s):  None
* Global Output(s): None
* Input(s):         crcapi_getcrcbuffer - Specified buffer
*                   crcapi_sizeinbytes - Length of specified buffer
* Output(s):        CRCApi_GetCRCValue - CRC Value requested
*
\*************************************************************************************************/
uint16_t CRC16Api_GetCRC(const uint16_t crcapi_getcrcbuffer[], uint32_t crcapi_sizeinbytes)
{
    uint32_t crcapi_getcrcvalue = CRC16_INIT;
    uint32_t word_index = 0U;
    uint16_t crc_byte[16U];
    uint16_t crc_table_index;
    int *calc_address;
    /* 0 thru 16 through out this function are not magic numbers but rather index numbers since
    * this function will process 16 bytes at once in the first stage and 2 bytes at once in the last stage*/

    while (crcapi_sizeinbytes >= 16U)
    {
        /*Justification for Coding Standard Deviation:
        *    pointer arithmetic is required for TI addressing
        *    An exception to MISRA Rules 11.4 and 17.4 is required.
        *    __byte is a TI intrinsic function therefore an exception to
        *    MISRA Rule 8.1 and 8.9 is also required
        */
        /*lint --e{960} # intended as is */
        /*lint --e{718} # intended as is */
        /*lint --e{746} # intended as is */
        /*lint --e{929} # intended as is */
        calc_address = (int *)(crcapi_getcrcbuffer + word_index);
        crc_byte[0U]  = (uint16_t)__byte(calc_address, 1U);
        crc_byte[1U]  = (uint16_t)__byte(calc_address, 0U);
        crc_byte[2U]  = (uint16_t)__byte(calc_address, 3U);
        crc_byte[3U]  = (uint16_t)__byte(calc_address, 2U);
        crc_byte[4U]  = (uint16_t)__byte(calc_address, 5U);
        crc_byte[5U]  = (uint16_t)__byte(calc_address, 4U);
        crc_byte[6U]  = (uint16_t)__byte(calc_address, 7U);
        crc_byte[7U]  = (uint16_t)__byte(calc_address, 6U);
        crc_byte[8U]  = (uint16_t)__byte(calc_address, 9U);
        crc_byte[9U]  = (uint16_t)__byte(calc_address, 8U);
        crc_byte[10U] = (uint16_t)__byte(calc_address, 11U);
        crc_byte[11U] = (uint16_t)__byte(calc_address, 10U);
        crc_byte[12U] = (uint16_t)__byte(calc_address, 13U);
        crc_byte[13U] = (uint16_t)__byte(calc_address, 12U);
        crc_byte[14U] = (uint16_t)__byte(calc_address, 15U);
        crc_byte[15U] = (uint16_t)__byte(calc_address, 14U);

        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[0U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[1U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[2U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[3U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[4U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[5U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[6U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[7U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[8U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[9U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[10U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[11U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[12U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[13U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[14U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[15U]) \
                                      & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];
        crcapi_sizeinbytes -= 16U;
        word_index += 8U;
    }
    while (crcapi_sizeinbytes >= 2U)
    {
        /*Justification for Coding Standard Deviation:
        *    pointer arithmetic is required for TI addressing
        *    An exception to MISRA Rules 11.4 and 17.4 is required.
        *    _byte is a TI intrinsic function therefore an exception to
        *    MISRA Rule 8.1 is also required
        */
        /*lint --e{960} # intended as is */
        /*lint --e{718} # intended as is */
        /*lint --e{746} # intended as is */
        /*lint --e{929} # intended as is */
        calc_address = (int *)(crcapi_getcrcbuffer + word_index);
        crc_byte[0U] = (uint16_t)__byte(calc_address, 1U);
        crc_byte[1U] = (uint16_t)__byte(calc_address, 0U);

        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[0U]) \
                                  & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];


        crc_table_index = (uint16_t)(((crcapi_getcrcvalue >> SHIFT_ONE_BYTE) ^ crc_byte[1U]) \
                                  & BYTE_MASK);
        crcapi_getcrcvalue = (crcapi_getcrcvalue << SHIFT_ONE_BYTE) ^ G_CRC16_table[crc_table_index];

        crcapi_sizeinbytes -=2U;
        word_index ++;
    }

    /* ^ operator denotes bitwise XOR */
    crcapi_getcrcvalue ^= CRC16_XORFINAL;
    crcapi_getcrcvalue &= CRC16_MASK;

    return((uint16_t)crcapi_getcrcvalue);
}

