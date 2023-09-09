/***************************************************************************************************
 * File name: DFT.h
 *
 * Purpose: The DFT.h file contains the declatrations related to the GSE DFT Serial interface.
 *
 * Copyright Notice:
 * All source code and data contained in this file is Proprietary and
 * Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
 * disclosed; in whole or in part, without the express written permission of
 * Eaton Aerospace.
 *
 * Copyright (c) 2022 - Eaton Aerospace Group, All Rights Reserved.
 *
 * Author           Date         CR#        Description
 * ------           ---------    ------     ---------------------------------------
 *  Adam Bouwens     12/07/2022       N/A    Ported to MCU
 *
 ****************************************************************************************************/

#ifndef DFT_H_
#define DFT_H_

/****************************************** INCLUDES *********************************************/
#include "Typedefs.h"
#include "F28x_Project.h"
#include "parameter.h"
#include "gse.h"
#include "scimessage.h"
#include "rvdt.h"
#include "encoder.h"

/* Bit definitions for serial port (SCI-A) initialization */
#define SCICCHAR_VALUE    7U                 /* 8 bits per char */

/* SYSCLKOUT = 200MHz; @LSPCLK = 100.0MHz) SCI Baud rate = LSPCLK / ((BRR + 1) * 8) */
/* BRR = 0x001A */
/* Baud rate = 100.0MHz / ((26 + 1) * 8) = 100Mhz/(27 * 8) = 462963 = ~460800 = 0.47% error */
#define SCICHBAUD_VALUE    0x00U    /* BRR = 26, High = 0x00 =  0 */
#define SCICLBAUD_VALUE    0x1AU    /* BRR = 26, Low =  0x1A = 26 */

/* SYSCLKOUT = 200MHz; @LSPCLK = 100.0MHz) SCI Baud rate = LSPCLK / ((BRR + 1) * 8) */
/* BRR = 0x006B */
/* Baud rate = 100.0MHz / ((108 + 1) * 8) = 100Mhz/(109 * 8) = 114679 = ~115200 = -0.45% error */
//#define SCIAHBAUD_VALUE    0x00U    /* BRR = 108, High = 0x00 =  0  */
//#define SCIALBAUD_VALUE    0x6CU    /* BRR = 108, Low =  0x6C = 108 */

/* The number of SCI loopback test messages */
#define SCI_LOOPBACK_TEST_LENGTH         2U
#define SCI_TEST_PATTERN1                0x55U
#define SCI_TEST_PATTERN2                0xAAU

/* Maximum size of the DFT message in bytes is desired ((min period / srvc period) * number bytes tx per period)
* = 10ms / 0.125ms = 80 services * 4 bytes per service = 320 bytes total.  */
#define DFT_MAX_MSG_SIZE_BYTES  (320U)

/********** Define DFT variable list ***********/
typedef struct
{
    char16_t* variableName;
    uint32_t variableValue;
}SCITxListEntry;

/*    brief Message definition for GSE interactive mode selection */
typedef struct
{
    Uint16 DLEbegin;        /* Word No. 00, the starting data link escape character (0x10) */
//    Uint16 ChanDataID;      /* Word No. 01, channel data identifier */
//    Uint16 DataPktSize;     /* Word No. 02, data packet size for message */
//    Uint16 FSCUChanID;      /* Word No. 03, left, right, or both channels as well as rig or test mode */
    Uint16 Addbyte0;        /* Word No. 04, byte0 */
    Uint16 Addbyte1;        /* Word No. 05, byte1 */
    Uint16 Addbyte2;        /* Word No. 06, byte2 */
    Uint16 Addbyte3;        /* Word No. 07, byte3 */
    Uint16 AddID;       /* Word No. 08, reserved */
//    Uint16 reserved2;       /* Word No. 09, reserved */
    int16 checksum;         /* Word No. 10, the message checksum */
    Uint16 DLEend;          /* Word No. 11, the ending data link escape character (0x10) */
    Uint16 endofTx;         /* Word No. 12, the end of transmission character (0x03) */
} DFT_ADDRESS;

typedef enum
{
    DFT_MSG_SUCCESSFUL,
    DFT_MSG_FAILURE,
    DFT_MSG_INPROGRESS
} DFT_MSG_STATUS;

/* 32-bit Bit Description */
typedef struct
{
    uint16_t byte0:8U;
    uint16_t byte1:8U;
} t16Bytes_t;

typedef union
{
    uint16_t u16Data;
    t16Bytes_t t8;
} t16Data_t;

typedef struct
{
    uint16_t byte0:8U;
    uint16_t byte1:8U;
    uint16_t byte2:8U;
    uint16_t byte3:8U;
} t32Bytes_t;

/* DFT Memmber Union Type */
typedef union
{
    uint32_t u32Data;
    float32_t f32Data;
    t32Bytes_t t8;
    t16Data_t t16[2];
} t32Data_t;

/*  DFT Message Packet:  Important!!! Must be grouped in uint32_t types. The driver will service
 * the buffer in 4 byte intervals (uint32_t). This is to ensure the data is refreshed just prior
 * to sending on the bus along with making sure that partial parameters are not sent out prior to
 * being updated. */
typedef struct
{
    struct
    {   /* Bytes 0-3 */
        uint16_t BeginDLE:8U;
        uint16_t bMcuChannelA:1U;
        uint16_t bMcuChannelB:1U;
        uint16_t bFasStatusFlapFail:1U;
        uint16_t bFasStatusNotAvailable:1U;
        uint16_t bFasStatusRigInProcess:1U;
        uint16_t bFasStatusGseConnected:1U;
        uint16_t bFasStatusFlapPositionFail:1U;
        uint16_t bValidCmd:1U;
        uint16_t DataPktSize;
    } tDft_0;
    struct
    {   /* Bytes 4-7 */
        uint32_t u32OfpCrc;
    } tDft_1;
    struct
    {   /* Bytes 8-11 */
        uint32_t u32PdiCrc;
    } tDft_2;
    struct
    {   /* Bytes 12-15 */
        uint32_t u32BldCrc;
    } tDft_3;
    struct
    {   /* Bytes 16-19 */
        uint16_t u16OfpSwVersion_Build:8U;
        uint16_t u16OfpSwVersion_Revision:8U;
        uint16_t u16OfpSwVersion_Minor:8U;
        uint16_t u16OfpSwVersion_Major:8U;
    } tDft_4;
    struct
    {   /* Bytes 20-23 */
        uint16_t u32BldSwVersion_Build:8U;
        uint16_t u32BldSwVersion_Revision:8U;
        uint16_t u32BldSwVersion_Minor:8U;
        uint16_t u32BldSwVersion_Major:8U;
    } tDft_5;
    struct
    {   /* Bytes 24-27 */
        uint16_t u32PdiSwVersion_Build:8U;
        uint16_t u32PdiSwVersion_Revision:8U;
        uint16_t u32PdiSwVersion_Minor:8U;
        uint16_t u32PdiSwVersion_Major:8U;
    } tDft_6;
    struct
    {   /* Bytes 28-31 */
        uint16_t LatchedFaultsStatb0:8U;
        uint16_t LatchedFaultsStatb1:8U;
        uint16_t LatchedFaultsStatb2:8U;
        uint16_t LatchedFaultsStatb3:8U;
    } tDft_7;
    struct
    {   /* Bytes 32-35 */
        uint16_t CriticalFaultsStatb0:8U;
        uint16_t CriticalFaultsStatb1:8U;
        uint16_t CriticalFaultsStatb2:8U;
        uint16_t McuMode:4U;
        uint16_t u16Nvm_StateMode:4U;
    } tDft_8;
    struct
    {   /* Bytes 36-39 */
        uint16_t u16InhibitsStat;
        uint16_t u16LatchedWarnings;
    } tDft_9;
    struct
    {   /* Bytes 40-43 */
        uint16_t FaultCode:8U;
        uint16_t FaultData:8U;
        tFasLruStatus_t tFasLruStatus;
    } tDft_10;
    struct
    {   /* Bytes 44-47 */
        float32_t f32FlapAngleSkewSnsr;
    } tDft_11;
    struct
    {   /* Bytes 48-51 */
        float32_t f32StrokeSkewSnsr;
    } tDft_12;
    struct
    {   /* Bytes 52-55 */
        float32_t f32StrokeQuad;
    } tDft_13;
    struct
    {   /* Bytes 56-59 */
        float32_t f32BiasFused;
    } tDft_14;
    struct
    {   /* Bytes 60-63 */
        float32_t f32QuadCntResidualFused;
    } tDft_15;
    struct
    {   /* Bytes 64-67 */
        float32_t f32SkewResidualFused;
    } tDft_16;
    struct
    {   /* Bytes 68-71 */
        float32_t f32StopPosition;
    } tDft_17;
    struct
    {   /* Bytes 72-75 */
        float32_t f32StrokeScaledPositionCalibrated;
    } tDft_18;
    struct
    {   /* Bytes 76-79 */
        float32_t f32RvdtCalCoeffA1;
    } tDft_19;
    struct
    {   /* Bytes 80-83 */
        float32_t f32RvdtCalCoeffA0;
    } tDft_20;
    struct
    {   /* Bytes 84-87 */
        float32_t f32RvdtCalCoeffA3;
    } tDft_21;
    struct
    {   /* Bytes 88-91 */
        float32_t f32RvdtCalCoeffA2;
    } tDft_22;
    struct
    {   /* Bytes 92-95 */
        float32_t f32PositionCmd;
    } tDft_23;
    struct
    {   /* Bytes 96-99 */
        float32_t f32SpeedCmd;
    } tDft_24;
    struct
    {   /* Bytes 100-103 */
        float32_t f32PositionFdbk;
    } tDft_25;
    struct
    {   /* Bytes 104-107 */
        float32_t f32Speed;
    } tDft_26;
    struct
    {   /* Bytes 108-111 */
        uint32_t G_tGpioInputs;
    } tDft_27;
    struct
    {   /* Bytes 112-115 */
        float32_t f32MotorSpeed;
    } tDft_28;
    struct
    {   /* Bytes 116-119 */
        int32_t s32SpeedRpm;
    } tDft_29;
    struct
    {   /* Bytes 120-123 */
        int16_t s16QuadFused;
        int16_t s16StrokeRvdtAdcAvgRaw;
    } tDft_30;
    struct
    {   /* Bytes 124-127 */
        uint16_t u16MotorCmdLastBadHallState:3U;
        uint16_t G_ACTUATOR_ID:3U;
        uint16_t CurrentBusState:2U;
        uint16_t G_eLruId:3U;
        uint16_t MotorRunning:1U;
        uint16_t BrakeActivated:1U;
        uint16_t MotorStart:1U;
        uint16_t MotorStop:1U;
        uint16_t RVDTEnable:1U;
        uint16_t u16MotorCmdIllegalHalls;
    } tDft_31;
    struct
    {   /* Bytes 128-131 */
        uint16_t u16Hall1HallGpio;
        int16_t s16Hall1Revolutions;
    } tDft_32;
    struct
    {   /* Bytes 132-135 */
        int16_t s16Hall1Direction;
        int16_t s16Hall1Position;
    } tDft_33;
    struct
    {   /* Bytes 136-139 */
        int16_t s16PositionCmdQuad;
        int16_t s16StopPosition;
    } tDft_34;
    struct
    {   /* Bytes 140-143 */
        int16_t s16MotorDirection;
        uint16_t u16BrakeDutyCycle;
    } tDft_35;
    struct
    {   /* Bytes 144-147 */
        uint16_t u16RVDT_POS_raw;
        uint16_t u16RVDT_SUM_raw;
    } tDft_36;
    struct
    {   /* Bytes 148-151 */
        int16_t s16QuadCntSkewSnsr;
        uint16_t u16NVM_Rig_PRGM_Pins;
    } tDft_37;
    struct
    {   /* Bytes 152-155 */
        uint16_t u16Nvm_State_Pointer;
        uint16_t u16Nvm_RiggingCRC;
    } tDft_38;
    struct
    {   /* Bytes 156-159 */
        uint16_t u16Pwm1CmtnPointer;
        int16_t s16Pwm1DutyFunc;
    } tDft_39;
    struct
    {   /* Bytes 160-163 */
        float32_t f32G_SpeedPidOutput;
    } tDft_40;
    struct
    {   /* Bytes 164-167 */
        uint16_t u16WDRS;
        uint16_t u16WdrsCounter;
    } tDft_41;
    struct
    {   /* Bytes 168-171 */
        uint16_t tWctaFrame0_u16Avg;
        uint16_t tWctaFrame1_u16Avg;
    } tDft_42;
    struct
    {   /* Bytes 172-175 */
        uint16_t tWctaFrame2_u16Avg;
        uint16_t tWctaFrame3_u16Avg;
    } tDft_43;
    struct
    {   /* Bytes 176-179 */
        uint16_t tWctaFrame4_u16Avg;
        uint16_t tWctaFrame5_u16Avg;
    } tDft_44;
    struct
    {   /* Bytes 180-183 */
        uint16_t tWctaFrame6_u16Avg;
        uint16_t tWctaFrame7_u16Avg;
    } tDft_45;
    struct
    {   /* Bytes 184-187 */
        uint16_t tWctaFrameAll_u16Avg;
        uint16_t tWctaFrame0_u16Max;
    } tDft_46;
    struct
    {   /* Bytes 188-191 */
        uint16_t tWctaFrame1_u16Max;
        uint16_t tWctaFrame2_u16Max;
    } tDft_47;
    struct
    {   /* Bytes 192-195 */
        uint16_t tWctaFrame3_u16Max;
        uint16_t tWctaFrame4_u16Max;
    } tDft_48;
    struct
    {   /* Bytes 196-199 */
        uint16_t tWctaFrame5_u16Max;
        uint16_t tWctaFrame6_u16Max;
    } tDft_49;
    struct
    {   /* Bytes 200-203 */
        uint16_t tWctaFrame7_u16Max;
        uint16_t tWctaFrameAll_u16Max;
    } tDft_50;
    struct
    {   /* Bytes 204-207 */
        uint32_t G_address0;
    } tDft_51;
    struct
    {   /* Bytes 208-211 */
        uint32_t G_address1;
    } tDft_52;
    struct
    {   /* Bytes 212-215 */
        float32_t f32_5V_SENSE;
    } tDft_53;
    struct
    {   /* Bytes 216-219 */
        float32_t f32_28VBUS_VSENSE;
    } tDft_54;
    struct
    {   /* Bytes 220-223 */
        float32_t f32_POSITIVE_15V_SENSE;
    } tDft_55;
    struct
    {   /* Bytes 224-227 */
        float32_t f32_NEGATIVE_15V_SENSE;
    } tDft_56;
    struct
    {   /* Bytes 228-231 */
        float32_t f32_DC_BUS_VSENSE;
    } tDft_57;
    struct
    {   /* Bytes 232-235 */
        float32_t f32_I_PHA_CHA;
    } tDft_58;
    struct
    {   /* Bytes 236-239 */
        float32_t f32_I_PHB_CHA;
    } tDft_59;
    struct
    {   /* Bytes 240-243 */
        float32_t f32_I_PHC_CHA;
    } tDft_60;
    struct
    {   /* Bytes 244-247 */
        float32_t f32_MOTOR_TEMP_PHA;
    } tDft_61;
    struct
    {   /* Bytes 248-251 */
        float32_t f32_MOTOR_TEMP_PHB;
    } tDft_62;
    struct
    {   /* Bytes 252-255 */
        float32_t f32_MOTOR_TEMP_PHC;
    } tDft_63;
    struct
    {   /* Bytes 256-259 */
        float32_t f32_Peak_I;
    } tDft_64;
    struct
    {   /* Bytes 260-263 */
        uint16_t u16NVMfaultData:8U;
        uint16_t u16NVMfaultId:8U;
        uint16_t u16MSB_OUT:1U;
        uint16_t bBRAKE_CNTL_CHA:1U;
        uint16_t bSHDN_BRK_CTRL_CHA:1U;
        uint16_t bCHA_STATUS:1U;
        uint16_t bNS_CNTL_PHA_CHA:1U;
        uint16_t bNS_CNTL_PHC_CHA:1U;
        uint16_t bBUS_SW_ENABLE:1U;
        uint16_t bVC_MODE_CHA:1U;
        uint16_t b270V_BUS_CNTL_CHA:1U;
        uint16_t bGD_RESET:1U;
        uint16_t bENC_ZERO_SET_CNTL:1U;
        uint16_t bnCS_270VBUS_VSENSE:1U;
        uint16_t bnCS_28VBUS_VSENSE:1U;
        uint16_t bnCS_IBRK_SENSE:1U;
        uint16_t b28V_BUS_CNTL_CHA:1U;
        uint16_t bINRUSH_CTR_CHA:1U;
    } tDft_65;
    struct
    {   /* Bytes 264-267 */
        uint16_t bPHA_HS_CNTL_CHA:1U;
        uint16_t bPHA_LS_CNTL_CHA:1U;
        uint16_t bPHB_HS_CNTL_CHA:1U;
        uint16_t bPHB_LS_CNTL_CHA:1U;
        uint16_t bPHC_HS_CNTL_CHA:1U;
        uint16_t bPHC_LS_CNTL_CHA:1U;
        uint16_t u16NVMrigstatus:2U;
        uint16_t bVcModeCmd:1U;
        uint16_t bSensorFusionEnableCmd:1U;
        uint16_t CurrentBusSubState:3U;
        uint16_t Reserved1:3U;
        uint16_t u16IncoderPositionNotValidCntr;
    } tDft_66;
    struct
    {   /* Bytes 268-271 */
        float32_t f32imbCurrent;
    } tDft_67;
    struct
    {   /* Bytes 272-275 */
        uint16_t BitmonitorsIllegalHalls;
        uint16_t BadTransitions;
    } tDft_68;
    struct
    {   /* Bytes 276-279 */
        uint32_t u32IncoderPosition;
    } tDft_69;
    struct
    {   /* Bytes 280-283 */
        uint16_t u16IncoderCrc8CheckFailedCntr;
        uint16_t u16IncoderRxNotReadyCntr;
    } tDft_70;
    struct
    {   /* Bytes 284-287 */
        uint32_t u32IncoderPositionOffset;
    } tDft_71;
    struct
    {   /* Bytes 288-291 */
        uint32_t u32OneMsTimer;
    } tDft_72;
    /* end:  Cannot have more than 69 32-bit words */
} tDftMsg_t;

/* 268 / 4 = 67. Max bytes can send for DFT Message. 8.375ms period @0.125 srvc intervals = 67 total services.
 * @460kbps can send max 5 chars in .125ms. */
typedef union
{
  t32Data_t t32[DFT_MAX_MSG_SIZE_BYTES/4U];
  tDftMsg_t tData;
} tDftMsgBuf_t;

/* DFT Message Type */
typedef uint16_t         DFT_TypeMsg[];

/* Size of MSE Message Type */
typedef int16_t          DFT_TypeMsgSize;

/* Pointer to MSE Message Type */
typedef const uint16_t * DFT_TypeMsgPtr;
extern bool DFT_Tx_cmd;
extern bool DFT_Tx_block;
extern Timer_t DFT_Tx_Timer;
//extern SCITxListEntry G_DFT_Table_Tx_Messages[5];
//extern float64 G_DFT_Tx_Payload_Tbl_Msgs[SCI_DFT_NUM_TX_MSGS];

/**************************************** FUNCTION PROTOTYPES ************************************/
interrupt void DFT_TxIsr(void);
interrupt void DFT_RxIsr(void);
void DFT_DisableRxLine(void);
void DFT_EnableRxLine(void);
int16_t DFT_TxData(const uint16_t serial_txinputbuffer[],
                      int16_t        serial_txinputsize);
int16_t DFT_RxData(uint16_t *ptr_rxinputbuffer,
                      int16_t  serial_rxinputsize);
bool DFT_TxBufferEmpty(void);
void DFT_Init(void);
void DFT_TxSrvc(void);
interrupt void DFT_RxSrvc(void);
SCI_STATUS DFT_TxMsg(Uint16 length);
//void DFT_InitializeTimers(void);
#endif /* DFT_H_ */
