/****************************************************************************************************
*  File name: A825_MsgIDD.c
*
*  Purpose: Defines ARINC825 Messages for both Transmit and Receive for MCU
*
*  Copyright Notice:
*  All source code and data contained in this file is Proprietary and
*  Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
*  disclosed; in whole or in part, without the express written permission of
*  Eaton Aerospace.
*
*  Copyright (c) 2020 - Eaton Aerospace Group, All Rights Reserved.
*
 * Author            Date       CR#              Description
 * ------          ---------    ------        -------------------------------------
 * Adam Bouwens     11/08/2022  NA             Port to MCU LRU
****************************************************************************************************/
#include "typedefs.h"
#include "A825_MsgInfo.h"
#include "A825Mgr.h"

/* Preprocessor macro to convert the starting bit number defined in the CAN format (Little-Endian)
 * to the corresponding Big-Endian format used for all signal data.
 *
 *                  SIGNAL BIT# (RAW BIT#)
 *CAN DB#  bit-7   bit-6   bit-5   bit-4   bit-3   bit-2   bit-1   bit-0
 *-------  --------------------------------------------------------------
 *CAN DB0  63(07)  62(06)  61(05)  60(04)  59(03)  58(02)  57(01)  56(00)
 *CAN DB1  55(15)  54(14)  53(13)  52(12)  51(11)  50(10)  49(09)  48(08)
 *CAN DB2  47(23)  46(22)  45(21)  44(20)  43(19)  42(18)  41(17)  40(16)
 *CAN DB3  39(31)  38(30)  37(29)  36(28)  35(27)  34(26)  33(25)  32(24)
 *CAN DB4  31(39)  30(38)  29(37)  28(36)  27(35)  26(34)  25(33)  24(32)
 *CAN DB5  23(47)  22(46)  21(45)  20(44)  19(43)  18(42)  17(41)  16(40)
 *CAN DB6  15(55)  14(54)  13(53)  12(52)  11(51)  10(50)  09(49)  08(48)
 *CAN DB7  07(63)  06(62)  05(61)  04(60)  03(59)  02(58)  01(57)  00(56)
*/
/* This is a pre-compiled macro and therefore has no impact on processor timing. */
#define u16ConvertLittleToBigEndianBitNumber(x)  ((uint16_t) (64U - ((8U - (x % 8U)) + (8U * (x / 8U)))))

/* Driver requires startPos to begin at at Data Byte 7 bit 0 (bit 63) and end at Data Byte 0 bit 7
 * (bit 0). This is counter intuitive and a result of having to modify the protocol to match
 * customer requirements after the software was originally developed. This resulted in having to
 * define the startPos (start position) element of the signal as an offset from the last bit of the
 * data bits rather than the start bit position of the data bits.
 * startPos is defined as 64 minus the length in bits that the signal tarts at as an offset from the
 * very first bit in the data field.
 * Example:  MCU Flap Panel Position Feedback has been defined to begin (lsb) on bit 11 (in Data Byte 1)
 * and end (msb) on bit 7 (in Data Byte 0). The signal occupies the first 13 bits of the data bytes
 * in the CAN frame. Therefore, the startPos is 64 - 13 = 51.
 */

/**************************** TRANSMIT SIGNAL LISTS *****************************************************************************/
/* Signal Definitions for the ARINC825 MCU Position, Status, Fault, Maintenance Message for reuse on multiple busses */

/* ARINC825 CONTROL BUS TX SIGNAL DEFINITIONS */
static const Signal_Info G_A825_CNTRL_BUS_TX_MCU_PSN_STATUS_FLT_MAINT_signal_list_t[] =
{                                                                                                                                                                                                            \
 { SIGNAL1, u16ConvertLittleToBigEndianBitNumber(11U), 13U,  SIGNAL_BNR,      (7.0F/4096.0F),     (4096.0F/7.0F) },    /* MCU_Flap_Panel_Position_Fdbk:  MCU Channel A/B Position Feedback in Stroke Inches */ \
 { SIGNAL2, u16ConvertLittleToBigEndianBitNumber(24U), 16U,  SIGNAL_OPAQUE,           (1.0F),             (1.0F) },    /* MCU_Status Signals tMcuStatus bit packed */                                        \
 { SIGNAL3, u16ConvertLittleToBigEndianBitNumber(40U), 16U,  SIGNAL_OPAQUE,           (1.0F),             (1.0F) },    /* MCU_Fault Signals tMcuFault bit packed */                                          \
 { SIGNAL4, u16ConvertLittleToBigEndianBitNumber(56U), 16U,  SIGNAL_OPAQUE,           (1.0F),             (1.0F) },    /* MCU_Maint Signals tMcuMaint bit packed */                                          \
};

static const Signal_Info G_A825_CNTRL_BUS_TX_MCU_SENSOR_DATA_signal_list_t[] =
{                                                                                                                                                                                   \
 { SIGNAL1, u16ConvertLittleToBigEndianBitNumber( 8U), 12U,  SIGNAL_UBNR,    (12.0F/4096.0F),    (4096.0F/12.0F) },    /* MCU Channel Current Feedback  */                          \
 { SIGNAL2, u16ConvertLittleToBigEndianBitNumber(24U), 16U,  SIGNAL_SHORT,            (1.0F),             (1.0F) },    /* MCU Channel Position Quad Count */                        \
 { SIGNAL3, u16ConvertLittleToBigEndianBitNumber(40U), 13U,  SIGNAL_BNR,      (7.0F/4096.0F),     (4096.0F/7.0F) },    /* MCU Onside Skew Sensor Feedback in Stroke Inches. */      \
 { SIGNAL4, u16ConvertLittleToBigEndianBitNumber(56U), 16U,  SIGNAL_SHORT,            (1.0F),             (1.0F) },    /* MCU Skew Sensor Feedback in Counts. */                    \
 { SIGNAL5, u16ConvertLittleToBigEndianBitNumber(39U),  1U,  SIGNAL_BOOL,             (1.0F),             (1.0F) },    /* MCU Skew Sensor Type:  Rvdt or Encoder */                 \
};

/* ARINC825 MAINTENANCE BUS TX SIGNAL DEFINITIONS */
static const Signal_Info G_A825_MAINT_BUS_TX_MCU_PSN_STATUS_FLT_MAINT_signal_list_t[] =
{                                                                                                                                                                                                            \
 { SIGNAL1, u16ConvertLittleToBigEndianBitNumber(11U), 13U,  SIGNAL_BNR,      (7.0F/4096.0F),     (4096.0F/7.0F) },    /* Rig_Flap_Position Feedback:  MCU Channel A/B Position Feedback in Stroke Inches */ \
 { SIGNAL2, u16ConvertLittleToBigEndianBitNumber(10U),  1U,  SIGNAL_BOOL,             (1.0F),             (1.0F) },    /* Rig_Pins Status Signal */                                                          \
 { SIGNAL3, u16ConvertLittleToBigEndianBitNumber( 9U),  1U,  SIGNAL_BOOL,             (1.0F),             (1.0F) },    /* Rig_RVDT_Calibrated Status Signal */                                               \
 { SIGNAL4, u16ConvertLittleToBigEndianBitNumber( 8U),  1U,  SIGNAL_BOOL,             (1.0F),             (1.0F) },    /* Rig_NVM_Data_Writtend Status Signal */                                             \
 { SIGNAL5, u16ConvertLittleToBigEndianBitNumber(24U), 16U,  SIGNAL_OPAQUE,           (1.0F),             (1.0F) },    /* Rig_Status Signals tMcuStatus bit packed */                                        \
 { SIGNAL6, u16ConvertLittleToBigEndianBitNumber(40U), 16U,  SIGNAL_OPAQUE,           (1.0F),             (1.0F) },    /* Rig_Fault Signals tMcuFault bit packed */                                          \
 { SIGNAL7, u16ConvertLittleToBigEndianBitNumber(56U), 16U,  SIGNAL_OPAQUE,           (1.0F),             (1.0F) },    /* Rig_Maint Signals tMcuMaint bit packed */                                          \
};

static const Signal_Info G_A825_MAINT_BUS_TX_MCU_SENSOR_DATA_signal_list_t[] =
{                                                                                                                                                                                   \
 { SIGNAL1, u16ConvertLittleToBigEndianBitNumber( 8U), 12U,  SIGNAL_UBNR,    (12.0F/4096.0F),    (4096.0F/12.0F) },    /* MCU Channel Current Feedback  */                          \
 { SIGNAL2, u16ConvertLittleToBigEndianBitNumber(24U), 16U,  SIGNAL_SHORT,            (1.0F),             (1.0F) },    /* MCU Channel Position Quad Count */                        \
 { SIGNAL3, u16ConvertLittleToBigEndianBitNumber(40U), 13U,  SIGNAL_BNR,      (7.0F/4096.0F),     (4096.0F/7.0F) },    /* MCU Skew Sensor Feedback in Stroke Inches. */             \
 { SIGNAL4, u16ConvertLittleToBigEndianBitNumber(56U), 16U,  SIGNAL_SHORT,            (1.0F),             (1.0F) },    /* MCU Skew Sensor Feedback in counts. */                    \
 { SIGNAL5, u16ConvertLittleToBigEndianBitNumber(39U),  1U,  SIGNAL_BOOL,             (1.0F),             (1.0F) },    /* MCU Skew Sensor Type:  Rvdt or Encoder */                 \
};

/**************************** RECEIVE SIGNAL LISTS *****************************************************************************/
/* ARINC825 CONTROL BUS RX SIGNAL DEFINITIONS */
static const Signal_Info G_A825_CNTRL_BUS_RX_FLAP_CMD_signal_list_t[] =
{
 { SIGNAL1,  u16ConvertLittleToBigEndianBitNumber( 7U),  1U,  SIGNAL_BOOL,             (1.0F),             (1.0F) },    /* GSE_XYB_L_VC_Mode_Cmd. Boolean    */
 { SIGNAL2,  u16ConvertLittleToBigEndianBitNumber( 6U),  1U,  SIGNAL_BOOL,             (1.0F),             (1.0F) },    /* GSE_XYB_L_Sensor_Fusion_Enable. Boolean    */
 { SIGNAL3,  u16ConvertLittleToBigEndianBitNumber( 8U), 13U,  SIGNAL_BNR,      (7.0F/4096.0F),     (4096.0F/7.0F) },    /* GSE_XYB_L_Panel_Position_Cmd. 12-bit signed BNR between -7 and 7    */
 { SIGNAL4,  u16ConvertLittleToBigEndianBitNumber(16U),  8U,  SIGNAL_UBNR,      (2.0F/256.0F),      (256.0F/2.0F) },    /* GSE_XYB_L_Speed Adjust Percent. 8-bit unsigned BNR between 0 and 2  */
 { SIGNAL5,  u16ConvertLittleToBigEndianBitNumber(31U),  1U,  SIGNAL_BOOL,             (1.0F),             (1.0F) },    /* GSE_XYB_R_VC_Mode_Cmd. Boolean    */
 { SIGNAL6,  u16ConvertLittleToBigEndianBitNumber(30U),  1U,  SIGNAL_BOOL,             (1.0F),             (1.0F) },    /* GSE_XYB_R_Sensor_Fusion_Enable. Boolean    */
 { SIGNAL7,  u16ConvertLittleToBigEndianBitNumber(32U), 13U,  SIGNAL_BNR,      (7.0F/4096.0F),     (4096.0F/7.0F) },    /* GSE_XYB_R_Panel_Position_Cmd. 12-bit signed BNR between -7 and 7    */
 { SIGNAL8,  u16ConvertLittleToBigEndianBitNumber(40U),  8U,  SIGNAL_UBNR,      (2.0F/256.0F),      (256.0F/2.0F) },    /* GSE_XYB_R_Speed_Adjust_Percent. 8-bit unsigned BNR between 0 and 2  */
};

static const Signal_Info G_A825_CNTRL_BUS_RX_FIRST_MSG_signal_list_t[] =
{
 { SIGNAL1, u16ConvertLittleToBigEndianBitNumber(56U), 1U,   SIGNAL_BOOL,             (1.0F),             (1.0F) },       /* Placeholder, available to redefine */
};

/* ARINC825 MAINTENANCE BUS RX SIGNAL DEFINITIONS */
static const Signal_Info G_A825_MAINT_BUS_RX_RIG_CMD_signal_list_t[] =
{
 { SIGNAL1,  u16ConvertLittleToBigEndianBitNumber( 7U),  1U,  SIGNAL_BOOL,             (1.0F),             (1.0F) },    /* GSE_XYB_L_VC_Mode_Cmd. Boolean    */
 { SIGNAL2,  u16ConvertLittleToBigEndianBitNumber( 6U),  1U,  SIGNAL_BOOL,             (1.0F),             (1.0F) },    /* GSE_XYB_L_Sensor_Fusion_Enable. Boolean    */
 { SIGNAL3,  u16ConvertLittleToBigEndianBitNumber( 8U), 13U,  SIGNAL_BNR,      (7.0F/4096.0F),     (4096.0F/7.0F) },    /* GSE_XYB_L_Panel_Position_Cmd. 12-bit signed BNR between -7 and 7    */
 { SIGNAL4,  u16ConvertLittleToBigEndianBitNumber(16U),  8U,  SIGNAL_UBNR,      (2.0F/256.0F),      (256.0F/2.0F) },    /* GSE_XYB_L_Speed Adjust Percent. 8-bit unsigned BNR between 0 and 2  */
 { SIGNAL5,  u16ConvertLittleToBigEndianBitNumber(31U),  1U,  SIGNAL_BOOL,             (1.0F),             (1.0F) },    /* GSE_XYB_R_VC_Mode_Cmd. Boolean    */
 { SIGNAL6,  u16ConvertLittleToBigEndianBitNumber(30U),  1U,  SIGNAL_BOOL,             (1.0F),             (1.0F) },    /* GSE_XYB_R_Sensor_Fusion_Enable. Boolean    */
 { SIGNAL7,  u16ConvertLittleToBigEndianBitNumber(32U), 13U,  SIGNAL_BNR,      (7.0F/4096.0F),     (4096.0F/7.0F) },    /* GSE_XYB_R_Panel_Position_Cmd. 12-bit signed BNR between -7 and 7    */
 { SIGNAL8,  u16ConvertLittleToBigEndianBitNumber(40U),  8U,  SIGNAL_UBNR,      (2.0F/256.0F),      (256.0F/2.0F) },    /* GSE_XYB_R_Speed_Adjust_Percent. 8-bit unsigned BNR between 0 and 2  */
 { SIGNAL9,  u16ConvertLittleToBigEndianBitNumber(48U),  8U,  SIGNAL_OPAQUE,           (1.0F),             (1.0F) },    /* GSE_XYB_L_Rig_Cmd. 8-bit OPAQUE    */
 { SIGNAL10, u16ConvertLittleToBigEndianBitNumber(56U),  8U,  SIGNAL_OPAQUE,           (1.0F),             (1.0F) },    /* GSE_XYB_R_Rig_Cmd. 8-bit OPAQUE    */
};

static const Signal_Info G_A825_MAINT_BUS_RX_FIRST_MSG_signal_list_t[] =
{
 { SIGNAL1, u16ConvertLittleToBigEndianBitNumber(56U), 1U,   SIGNAL_BOOL,             (1.0F),             (1.0F) },       /* Placeholder, available to redefine */
};

/**************************** TRANSMIT MESSAGE LISTS *****************************************************************************/
/* ARINC825 CONTROL BUS TX MESSAGE LIST */
A825TxTableEntry tA825CntrlBusTxMsgList[A825_CNTRL_BUS_TX_NUM_MSGS] =
{                                                                                                                                                                                                                 \
 /* {  { ---------- lookUp_Id----------------------,  -A825MsgId--, -#Sig-, -DLC-, -Tx Period-, ------Signal Definition List-------------------------------- }, -A825_TX_SCHEDULE_FREQ-, Data, LastCount }, */    \
    {  { A825_CNTRL_BUS_TX_MCU_PSN_STATUS_FLT_MAINT,  0x08530000UL,    4U ,   8U ,        10U , G_A825_CNTRL_BUS_TX_MCU_PSN_STATUS_FLT_MAINT_signal_list_t   }, A825_TX_SCHEDULE_A     ,  0LL, 0L        },       \
    {  { A825_CNTRL_BUS_TX_MCU_SENSOR_DATA         ,  0x085300FCUL,    4U ,   8U ,        10U , G_A825_CNTRL_BUS_TX_MCU_SENSOR_DATA_signal_list_t            }, A825_TX_SCHEDULE_B     ,  0LL, 0L        },       \
};

/* ARINC825 MAINTENANCE BUS TX MESSAGE LIST */
A825TxTableEntry tA825MaintBusTxMsgList[A825_MAINT_BUS_TX_NUM_MSGS] =
{                                                                                                                                                                                                                 \
 /* {  { ---------- lookUp_Id----------------------,  -A825MsgId--, -#Sig-, -DLC-, -Tx Period-, ------Signal Definition List-------------------------------- }, -A825_TX_SCHEDULE_FREQ-, Data, LastCount }, */    \
    {  { A825_MAINT_BUS_TX_MCU_PSN_STATUS_FLT_MAINT,  0x18530F00UL,    7U ,   8U ,       100U , G_A825_MAINT_BUS_TX_MCU_PSN_STATUS_FLT_MAINT_signal_list_t   }, A825_TX_SCHEDULE_A     ,  0LL, 0L        },       \
    {  { A825_MAINT_BUS_TX_MCU_SENSOR_DATA         ,  0x18530FFCUL,    4U ,   8U ,       100U , G_A825_MAINT_BUS_TX_MCU_SENSOR_DATA_signal_list_t            }, A825_TX_SCHEDULE_B     ,  0LL, 0L        },       \
};

/**************************** RECEIVE MESSAGE LISTS *****************************************************************************/
/* ARINC825 CONTROL BUS RX MESSAGE LIST */
A825RxTableEntry tA825CntrlBusRxMsgList[A825_CNTRL_BUS_RX_NUM_MSGS] =
{
/* { { ---------- lookUp_Id-----------, -A825MsgId--, -#Sig-, -DLC-, -StaleTime-, ------Signal Definition List--------------------- }, tData, LastCount, Stale}, */
   { { A825_CNTRL_BUS_RX_FIRST_MSG    , 0x08904C94UL,    1U ,   8U ,        30U , G_A825_CNTRL_BUS_RX_FIRST_MSG_signal_list_t       },   0LL,        0L,  true},
   { { A825_CNTRL_BUS_RX_FLAP_CMD     , 0x08530000UL,    8U ,   6U ,        30U , G_A825_CNTRL_BUS_RX_FLAP_CMD_signal_list_t        },   0LL,        0L,  true},
};

/* ARINC825 MAINTENANCE BUS RX MESSAGE LIST */
A825RxTableEntry tA825MaintBusRxMsgList[A825_MAINT_BUS_RX_NUM_MSGS] =
{
/* { { ---------- lookUp_Id-----------, -A825MsgId--, -#Sig-, -DLC-, -StaleTime-, ------Signal Definition List--------------------- }, tData, LastCount, Stale}, */
   { { A825_MAINT_BUS_RX_FIRST_MSG    , 0x18904C94UL,    1U ,   8U ,       250U , G_A825_MAINT_BUS_RX_FIRST_MSG_signal_list_t       },   0LL,        0L,  true},
   { { A825_MAINT_BUS_RX_RIG_CMD      , 0x1853F000UL,   10U ,   8U ,       250U , G_A825_MAINT_BUS_RX_RIG_CMD_signal_list_t         },   0LL,        0L,  true},
};


/* End of file */
