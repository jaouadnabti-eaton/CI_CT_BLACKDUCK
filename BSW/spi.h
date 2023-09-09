/****************************************************************************************************
*  File name: spi.h
*
*  Purpose: The interface provides routines necessary to intialize the driver and send a message on
*  the channel.
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

#ifndef SPI_H__
#define SPI_H__

/*      Include Files */
#include "F2837xD_device.h"
//#include "eicas.h"
#include "mcu.h"
#include "parameter.h"
#include "timer.h"
#include "nvm.h"

/*
  SPI Communications
  
  brief The SPI Module provides services for communicating to the other channel over a 
         Serial Peripheral Interface bus (SPI).
*/


#define SPI_INVALID      0x00
#define SPI_NORMAL_OP    0x03
#define SPI_RIG_MSG1     0x04
#define SPI_RIG_MSG2     0x05
#define SPI_RIG_MSG3     0x06
#define SPI_RIG_MSG4     0x07
#define SPI_RIG_MSG5     0x08
#define SPI_RIG_MSG6     0x09
#define SPI_RIG_MSG7     0x0A
#define SPI_RIG_MSG8     0x0B
#define SPI_RIG_MSG9     0x0C
#define SPI_RIG_MSG10    0x0D
#define SPI_RIG_MSG11    0x0E
#define SPI_RIG_MSG12    0x0F
#define SPI_RIG_MSG13    0x10
#define SPI_RIG_MSG14    0x11
#define SPI_RIG_MSG15    0x12
#define SPI_RIG_MSG16    0x13
#define SPI_RIG_DUMMY    0x20

#define SPI_TX_MSG_SIZE (sizeof(SPI_MSG) - 2)
#define SPI_RX_MSG_SIZE 16 //sizeof(SPI_MSG)

/*      Public Type Definitions */

/* SPI Status bit definition */
struct SPI_LRUStatus
{
    Uint16 FLA_fail:1;      /* Bit 0 : FLA Fail */
    Uint16 FPSU_fail:1;     /* Bit 1 : FPSU Fail */
    Uint16 motor_fail:1;    /* Bit 2 : Motor Fail */
    Uint16 flap_jam:1;      /* Bit 3 : Flap Jam */
    Uint16 rig_status:2;    /* Bit 4 & 5 : RigStatus */
    Uint16 faultStatus:1;   /* Bit 6 : Cross side fault Status */
    Uint16 reserved:9;      /* Bit 7-15 : reserved */
};

/* Allow access to the LRU Status bit fields or entire word */
union SPI_LRUStatus_Word {
   Uint16  all;
   struct  SPI_LRUStatus bit;
};

/* Spi Message Buffer */
typedef struct
{
    Uint16 BeginChar;                   /* Word No. 00, Beginning SPI character */
    Uint16 PacketID;                    /* Word No. 01, Packet ID */
    union SPI_LRUStatus_Word LRUStatus; /* Word No. 02, LRU Status Bits */
    MOTOR_CURRENT MotorCurrent;         /* Word No. 03, Motor Current */
    Uint16 FaultCode;                   /* Word No. 04, Fault Code */
    Uint16 FaultData;                   /* Word No. 05, Fault Data */
    Uint16 Data1;                       /* Word No. 06, Data Word 1 */
    Uint16 Data2;                       /* Word No. 07, Data Word 2 */
    Uint16 Data3;                       /* Word No. 08, Data Word 3 */
    Uint16 Data4;                       /* Word No. 09, Data Word 4 */
    Uint16 Data5;                       /* Word No. 10, Data Word 5 */
    Uint16 Data6;                       /* Word No. 11, Data Word 6 */
    Uint16 Data7;                       /* Word No. 12, Data Word 7 */
    Uint16 Data8;                       /* Word No. 13, Data Word 8 */
    int16 Checksum;                     /* Word No. 14, Checksum */
    Uint16 EndTransmit;                 /* Word No. 15, Ending SPI character */
} SPI_MSG;

/*      Public Variable Declarations */

//extern Uint16 msgCnt;

//extern bool CrossSideSpiDone;

//extern bool SendRigOverSpi;
//extern bool SendMonitor0x64_Data;
//extern bool SpiMessageValid;
//extern Uint16 xLatestFaultCode;
//extern Uint16 xLatestFaultData;
//extern bool SpiMessageReady;
//extern bool SpiTransmitt;
//extern Uint16 spiHrtBt;
//extern bool SpiTxStatus;
//extern RIG_STATUS xSideRigStatus;
//extern Uint16 xReportedFaultCode;
//extern Timer_t FaultSpiTimer;

//extern bool ONSIDE_ENTER_CAL;
//extern bool XSIDE_ENTER_CAL;
//extern bool ONSIDE_AT_RMS;
//extern bool XSIDE_AT_RMS;

extern Uint16 SpiRxBuf[SPI_RX_MSG_SIZE];
/*      Public Interface Function Prototypes
*/
void Spi_Init(void);
//void Spi_TxMessage(void);
//bool Spi_RxMessage(void);
//bool Spi_ProcessSpiMessage(void);
//void Spi_CheckMessage(void);
//bool Spi_SendMessage(void);
void SPI_LoopbackTest(void);
void SPI_SetCS();
void SPI_ClearCS();
void SPI_Capture();
//void Spia_Init(void);
//void Spib_send();
#endif
/* end eicas.h */
