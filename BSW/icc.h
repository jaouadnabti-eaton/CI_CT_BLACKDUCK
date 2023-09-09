/***********************************************************************
    File name : icc.h

    Purpose: The icc.h file contains the declarations related to ICC
    SCI communication.

    Author              Date           CR#         Description
    Aniket Mahadik     09/07/2022      NA          Initial Software
*************************************************************************/
#ifndef ICC_H_
#define ICC_H_


/*****************************Includes***********************************************/
#include "Typedefs.h"
#include "F28x_Project.h"
#include "scimessage.h"
#include "DFT.h"
/* Bit definitions for serial port (SCI-A) initialization */
#define SCIACHAR_VALUE    7U                 /* 8 bits per char */
#define ICC_MAX_MSG_SIZE_BYTES  (24U)
/* SYSCLKOUT = 200MHz; @LSPCLK = 100.0MHz) SCI Baud rate = LSPCLK / ((BRR + 1) * 8) */
/* BRR = 0x001A */
/* Baud rate = 100.0MHz / ((26 + 1) * 8) = 100Mhz/(27 * 8) = 462963 = ~460800 = 0.47% error */
#define SCIAHBAUD_VALUE    0x00U    /* BRR = 26, High = 0x00 =  0 */
#define SCIALBAUD_VALUE    0x1AU    /* BRR = 26, Low =  0x1A = 26 */

#define ICC_MSG_SIZE  (sizeof(tSciIcc_t) * 2U)    /* Message size in bytes */
#define ICC_MAX_SIZE  (ICC_MSG_SIZE-2)
/*************************Define variables for sci.c**********************************/

/* SPI Status bit definition */
struct SCI_CH_Status
{
    uint16_t CHID:1;          /* Bit 0 : Channel ID; 1 for G_bChannelA 0 for G_bChannelB */
    uint16_t FLA_fail:1;      /* Bit 1 : FLA Fail */
    uint16_t FPSU_fail:1;     /* Bit 2 : FPSU Fail */
    uint16_t motor_fail:1;    /* Bit 3 : Motor Fail */
    uint16_t flap_jam:1;      /* Bit 4 : Flap Jam */
    uint16_t rig_status:2;    /* Bit 5 & 6 : RigStatus */
    uint16_t faultStatus:1;   /* Bit 7 : Cross side fault Status */
    uint16_t reserved:8;      /* Bit 8-16 : reserved */
};

typedef union
{
    uint16_t         all;
    struct SCI_CH_Status    bit;
} tSCI_CH_Status_t;

typedef struct
{
    struct
    {   /* Bytes 0-3 */
        uint16_t BeginDLE:8U;
        uint16_t reserved:8U;
        tSCI_CH_Status_t CHStatus;
    }tIcc_0;
    struct
    {   /* Bytes 4-7 */
        float32_t f32_MOTOR_PHASEx_I;
    } tIcc_1;
    struct
    {   /* Bytes 8-11 */
        uint32_t u32IncoderInputsRaw;
    } tIcc_2;
    struct
    {   /* Bytes 12-15 */
        uint16_t u16IncoderNewData;
        uint16_t bAvailable;
    } tIcc_3;
    struct
    {   /* Bytes 16-19 */
        uint16_t FaultCode;
        uint16_t FaultData;
    } tIcc_4;
    struct
    {   /* Bytes 20-23 */
        uint16_t rvdt_pos_in;
        uint16_t crc;
    } tIcc_5;
}tSciIcc_t;

typedef union
{
  t32Data_t t32[ICC_MAX_MSG_SIZE_BYTES/4U];
  tSciIcc_t tData;
} tSciIccMsgBuf_t;


extern tSciIccMsgBuf_t ICC_xData;
extern uint16_t ICC_InvalidCRCcounter;
extern uint16_t ICC_TimeOutCounter;

/******************************Function Prototypes************************************/
void ICC_GpioInit(void);
void Icc_Init (void);
void Icc_UpdateMsg (void);
void Icc_TxMsg (Uint16 length);
void Icc_TxSrvc(void);
SCI_STATUS Icc_RxMsg (void);
interrupt void Icc_RxSrvc(void);
#endif /* ICC_H_ */
