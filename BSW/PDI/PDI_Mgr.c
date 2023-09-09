/***************************************************************************************************
 * File name: PDI_Mgr.c
 *
 * Purpose: This module Provides interface for the information stored in the PDI table. Auto 
 * generated from PDI coder tool
 * 
 * Project Name: PDI Coder Tool
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
 * Adam Bouwens     09/14/2021        N/A    Refactored from ConfigMgr.c from previous AAP Program
 * Adam Bouwens     07/06/2022        N/A    Reworked for new PDI design concept
 * E0447863			08/01/2023
 ****************************************************************************************************/

/* INCLUDES *************************************************************************************/
#include "Defines.h"
#include "PDI_Mgr.h"

/* DEFINES **************************************************************************************/
#define NUM_PDIs (97U)  /* Number of PDIs defined in pdi.h as part of the tPdi_t structure */

/* TYPEDEFS *************************************************************************************/
typedef enum
{
    PDI_TYPE_bool_t = 0,
    PDI_TYPE_char16_t,
    PDI_TYPE_uchar16_t,
    PDI_TYPE_int16_t,
    PDI_TYPE_int32_t,
    PDI_TYPE_int64_t,
    PDI_TYPE_uint16_t,
    PDI_TYPE_uint32_t,
    PDI_TYPE_uint64_t,
    PDI_TYPE_float32_t,
    PDI_TYPE_float64_t,
    PDI_NUM_TYPES
} ePdiDataTypes_t;

typedef struct
{
    const bool_t* const pbPdi;
    const bool_t minVal;
    const bool_t maxVal;
    const uint16_t n;
} tPdiRange_bool_t;

typedef struct
{
    const char16_t* const pcPdi;
    const char16_t minVal;
    const char16_t maxVal;
    const uint16_t n;
} tPdiRange_char16_t;

typedef struct
{
    const uchar16_t* const pucPdi;
    const uchar16_t minVal;
    const uchar16_t maxVal;
    const uint16_t n;
} tPdiRange_uchar16_t;

typedef struct
{
    const int16_t* const ps16Pdi;
    const int16_t minVal;
    const int16_t maxVal;
    const uint16_t n;
} tPdiRange_int16_t;

typedef struct
{
    const int32_t* const ps32Pdi;
    const int32_t minVal;
    const int32_t maxVal;
    const uint16_t n;
} tPdiRange_int32_t;

typedef struct
{
    const int64_t* const ps64Pdi;
    const int64_t minVal;
    const int64_t maxVal;
    const uint16_t n;
} tPdiRange_int64_t;

typedef struct
{
    const uint16_t* const pu16Pdi;
    const uint16_t minVal;
    const uint16_t maxVal;
    const uint16_t n;
} tPdiRange_uint16_t;

typedef struct
{
    const uint32_t* const pu32Pdi;
    const uint32_t minVal;
    const uint32_t maxVal;
    const uint16_t n;
} tPdiRange_uint32_t;

typedef struct
{
    const uint64_t* const pu64Pdi;
    const uint64_t minVal;
    const uint64_t maxVal;
    const uint16_t n;
} tPdiRange_uint64_t;

typedef struct
{
    const float32_t* const pf32Pdi;
    const float32_t minVal;
    const float32_t maxVal;
    const uint16_t n;
} tPdiRange_float32_t;

typedef struct
{
    const float64_t* const pf64PdiData;
    const float64_t minVal;
    const float64_t maxVal;
    const uint16_t n;
} tPdiRange_float64_t;

typedef struct
{
    ePdiDataTypes_t eDataType;  /* Data type of the Parameter in enumerated form */
    void* const pvtPdiRange;    /* Void pointer to the PDI Range Type data for the associated parameter */
} tPdiRange_t;

/* CONSTANSTS ***********************************************************************************/
static const tPdiRange_bool_t tPdiRange_bSyncMonitorEnable = { &tPdi.bSyncMonitorEnable, false, true, (sizeof(tPdi.bSyncMonitorEnable) / sizeof(bool_t)) }; /* SYNC to MCU Enable Monitor */
static const tPdiRange_bool_t tPdiRange_bJamMonitorEnable = { &tPdi.bJamMonitorEnable, false, true, (sizeof(tPdi.bJamMonitorEnable) / sizeof(bool_t)) }; /* Jam Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16JamMonitorQuadLimit = { &tPdi.u16JamMonitorQuadLimit, 0U, 32767U, (sizeof(tPdi.u16JamMonitorQuadLimit) / sizeof(uint16_t)) }; /* Jam Monitor Quad Count Limit */
static const tPdiRange_uint16_t tPdiRange_u16JamMonitorCurrentLimit = { &tPdi.u16JamMonitorCurrentLimit, 1U, 4095U, (sizeof(tPdi.u16JamMonitorCurrentLimit) / sizeof(uint16_t)) }; /* Jam Monitor Current Limit */
static const tPdiRange_bool_t tPdiRange_bxChMonitorEnable = { &tPdi.bxChMonitorEnable, false, true, (sizeof(tPdi.bxChMonitorEnable) / sizeof(bool_t)) }; /* Cross Channel Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bxChCommMonitorEnable = { &tPdi.bxChCommMonitorEnable, false, true, (sizeof(tPdi.bxChCommMonitorEnable) / sizeof(bool_t)) }; /* Cross Channel Communication Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16xChCommMonitorFailLimit = { &tPdi.u16xChCommMonitorFailLimit, 0U, 50U, (sizeof(tPdi.u16xChCommMonitorFailLimit) / sizeof(uint16_t)) }; /* Cross Channel Communication SCI Maximum Retry */
static const tPdiRange_bool_t tPdiRange_bRigValueMonitorEnable = { &tPdi.bRigValueMonitorEnable, false, true, (sizeof(tPdi.bRigValueMonitorEnable) / sizeof(bool_t)) }; /* Rig Value Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bRigModeFaultMonitorEnable = { &tPdi.bRigModeFaultMonitorEnable, false, true, (sizeof(tPdi.bRigModeFaultMonitorEnable) / sizeof(bool_t)) }; /* Rig Mode Fault Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bBacklashFreePlayMonitorEnable = { &tPdi.bBacklashFreePlayMonitorEnable, false, true, (sizeof(tPdi.bBacklashFreePlayMonitorEnable) / sizeof(bool_t)) }; /* Backlash/free play Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16BacklashFreeplayMonitorQuadLimit = { &tPdi.u16BacklashFreeplayMonitorQuadLimit, 0U, 32767U, (sizeof(tPdi.u16BacklashFreeplayMonitorQuadLimit) / sizeof(uint16_t)) }; /* Backlash/free play Quad Count Limit */
static const tPdiRange_bool_t tPdiRange_bHallSensorSeqMonitorEnable = { &tPdi.bHallSensorSeqMonitorEnable, false, true, (sizeof(tPdi.bHallSensorSeqMonitorEnable) / sizeof(bool_t)) }; /* Hall Sensor Sequence Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16HallSensorSeqMonitorIllegalLimit = { &tPdi.u16HallSensorSeqMonitorIllegalLimit, 0U, 65535U, (sizeof(tPdi.u16HallSensorSeqMonitorIllegalLimit) / sizeof(uint16_t)) }; /* Hall Sensor Illegal Sequence Limit */
static const tPdiRange_bool_t tPdiRange_bInvalidHallSensorMonitorEnable = { &tPdi.bInvalidHallSensorMonitorEnable, false, true, (sizeof(tPdi.bInvalidHallSensorMonitorEnable) / sizeof(bool_t)) }; /* Invalid Hall Sensor Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bBrakeHoldMonitorEnable = { &tPdi.bBrakeHoldMonitorEnable, false, true, (sizeof(tPdi.bBrakeHoldMonitorEnable) / sizeof(bool_t)) }; /* Brake Hold Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bBrakeSwitchMonitorEnable = { &tPdi.bBrakeSwitchMonitorEnable, false, true, (sizeof(tPdi.bBrakeSwitchMonitorEnable) / sizeof(bool_t)) }; /* Brake Switch Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16BrakeSwitchMonitorBrkOffMin = { &tPdi.u16BrakeSwitchMonitorBrkOffMin, 1U, 4095U, (sizeof(tPdi.u16BrakeSwitchMonitorBrkOffMin) / sizeof(uint16_t)) }; /* Brake Switch Monitor Brake Off Min */
static const tPdiRange_uint16_t tPdiRange_u16BrakeSwitchMonitorBrkOffMax = { &tPdi.u16BrakeSwitchMonitorBrkOffMax, 1U, 4095U, (sizeof(tPdi.u16BrakeSwitchMonitorBrkOffMax) / sizeof(uint16_t)) }; /* Brake Switch Monitor Brake Off Max */
static const tPdiRange_uint16_t tPdiRange_u16BrakeSwitchMonitorBrkUpperMin = { &tPdi.u16BrakeSwitchMonitorBrkUpperMin, 1U, 4095U, (sizeof(tPdi.u16BrakeSwitchMonitorBrkUpperMin) / sizeof(uint16_t)) }; /* Brake Switch Monitor Brake Upper Limit Min */
static const tPdiRange_uint16_t tPdiRange_u16BrakeSwitchMonitorBrkUpperMax = { &tPdi.u16BrakeSwitchMonitorBrkUpperMax, 1U, 4095U, (sizeof(tPdi.u16BrakeSwitchMonitorBrkUpperMax) / sizeof(uint16_t)) }; /* Brake Switch Monitor Brake Upper Limit Max */
static const tPdiRange_uint16_t tPdiRange_u16BrakeSwitchMonitorBrkLowerMax = { &tPdi.u16BrakeSwitchMonitorBrkLowerMax, 1U, 4095U, (sizeof(tPdi.u16BrakeSwitchMonitorBrkLowerMax) / sizeof(uint16_t)) }; /* Brake Switch Monitor Brake Lower Limit Max */
static const tPdiRange_bool_t tPdiRange_bFpsuExcMonitorEnable = { &tPdi.bFpsuExcMonitorEnable, false, true, (sizeof(tPdi.bFpsuExcMonitorEnable) / sizeof(bool_t)) }; /* FPSU Position Sensor Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16FpsuExcMonitorSumIdeal = { &tPdi.u16FpsuExcMonitorSumIdeal, 1U, 4095U, (sizeof(tPdi.u16FpsuExcMonitorSumIdeal) / sizeof(uint16_t)) }; /* FPSU Position Sensor Monitor Sum Ideal */
static const tPdiRange_uint16_t tPdiRange_u16FpsuExcMonitorSumTolerance = { &tPdi.u16FpsuExcMonitorSumTolerance, 1U, 4095U, (sizeof(tPdi.u16FpsuExcMonitorSumTolerance) / sizeof(uint16_t)) }; /* FPSU Position Sensor Monitor Sum Tolerance */
static const tPdiRange_bool_t tPdiRange_bFpsuPosMonitorEnable = { &tPdi.bFpsuPosMonitorEnable, false, true, (sizeof(tPdi.bFpsuPosMonitorEnable) / sizeof(bool_t)) }; /* FPSU Position Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16FpsuPosMonitorDeltaThreshold = { &tPdi.u16FpsuPosMonitorDeltaThreshold, 1U, 4095U, (sizeof(tPdi.u16FpsuPosMonitorDeltaThreshold) / sizeof(uint16_t)) }; /* FPSU Position Monitor Delta Threshold */
static const tPdiRange_bool_t tPdiRange_bBridgeMonitorEnable = { &tPdi.bBridgeMonitorEnable, false, true, (sizeof(tPdi.bBridgeMonitorEnable) / sizeof(bool_t)) }; /* Bridge Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16BridgeMonitorBridgeNoneMin = { &tPdi.u16BridgeMonitorBridgeNoneMin, 1U, 4095U, (sizeof(tPdi.u16BridgeMonitorBridgeNoneMin) / sizeof(uint16_t)) }; /* Bridge Monitor None Switch Min */
static const tPdiRange_uint16_t tPdiRange_u16BridgeMonitorBridgeNoneMax = { &tPdi.u16BridgeMonitorBridgeNoneMax, 1U, 4095U, (sizeof(tPdi.u16BridgeMonitorBridgeNoneMax) / sizeof(uint16_t)) }; /* Bridge Monitor None Switch Max */
static const tPdiRange_uint16_t tPdiRange_u16BridgeMonitorBridgeUpperMin = { &tPdi.u16BridgeMonitorBridgeUpperMin, 1U, 4095U, (sizeof(tPdi.u16BridgeMonitorBridgeUpperMin) / sizeof(uint16_t)) }; /* Bridge Monitor Upper Switch Min */
static const tPdiRange_uint16_t tPdiRange_u16BridgeMonitorBridgeLowerMax = { &tPdi.u16BridgeMonitorBridgeLowerMax, 1U, 4095U, (sizeof(tPdi.u16BridgeMonitorBridgeLowerMax) / sizeof(uint16_t)) }; /* Bridge Monitor Lower Switch Max */
static const tPdiRange_bool_t tPdiRange_bProgPinMonitorEnable = { &tPdi.bProgPinMonitorEnable, false, true, (sizeof(tPdi.bProgPinMonitorEnable) / sizeof(bool_t)) }; /* Program Pin Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bRiggingCheckMonitorEnable = { &tPdi.bRiggingCheckMonitorEnable, false, true, (sizeof(tPdi.bRiggingCheckMonitorEnable) / sizeof(bool_t)) }; /* Rigging Check Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16RiggingCheckMonitorQuadDiff = { &tPdi.u16RiggingCheckMonitorQuadDiff, 0U, 32767U, (sizeof(tPdi.u16RiggingCheckMonitorQuadDiff) / sizeof(uint16_t)) }; /* Rigging Check Quad Difference Limit */
static const tPdiRange_bool_t tPdiRange_bROMCrcCheckMonitorEnable = { &tPdi.bROMCrcCheckMonitorEnable, false, true, (sizeof(tPdi.bROMCrcCheckMonitorEnable) / sizeof(bool_t)) }; /* ROM CRC Check Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bRAMCheckMonitorEnable = { &tPdi.bRAMCheckMonitorEnable, false, true, (sizeof(tPdi.bRAMCheckMonitorEnable) / sizeof(bool_t)) }; /* RAM Check Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bWDTimerPowerUpMonitorEnable = { &tPdi.bWDTimerPowerUpMonitorEnable, false, true, (sizeof(tPdi.bWDTimerPowerUpMonitorEnable) / sizeof(bool_t)) }; /* Watchdog Timer Powerup Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16WDTimerPowerUpMonitorMSec = { &tPdi.u16WDTimerPowerUpMonitorMSec, 0U, 65535U, (sizeof(tPdi.u16WDTimerPowerUpMonitorMSec) / sizeof(uint16_t)) }; /* Watchdog Timer Powerup Monitor Time */
static const tPdiRange_bool_t tPdiRange_bFrameOverrunMonitorEnable = { &tPdi.bFrameOverrunMonitorEnable, false, true, (sizeof(tPdi.bFrameOverrunMonitorEnable) / sizeof(bool_t)) }; /* Frame Overrun Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bSubframeIndexMonitorEnable = { &tPdi.bSubframeIndexMonitorEnable, false, true, (sizeof(tPdi.bSubframeIndexMonitorEnable) / sizeof(bool_t)) }; /* Subframe Index Fail Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bUndefinedINTMonitorEnable = { &tPdi.bUndefinedINTMonitorEnable, false, true, (sizeof(tPdi.bUndefinedINTMonitorEnable) / sizeof(bool_t)) }; /* Undefined Interrupt Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bStackOverrunMonitorEnable = { &tPdi.bStackOverrunMonitorEnable, false, true, (sizeof(tPdi.bStackOverrunMonitorEnable) / sizeof(bool_t)) }; /* Stack Overrun Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bOverCurrentMonitorEnable = { &tPdi.bOverCurrentMonitorEnable, false, true, (sizeof(tPdi.bOverCurrentMonitorEnable) / sizeof(bool_t)) }; /* Over Current Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bOverTempMonitorEnable = { &tPdi.bOverTempMonitorEnable, false, true, (sizeof(tPdi.bOverTempMonitorEnable) / sizeof(bool_t)) }; /* Over Temperature Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bSensorFusionBiasCheckMonitorEnable = { &tPdi.bSensorFusionBiasCheckMonitorEnable, false, true, (sizeof(tPdi.bSensorFusionBiasCheckMonitorEnable) / sizeof(bool_t)) }; /* Sensor Fusion Bias Check Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bSensorFusionMeasResidualMonitorEnable = { &tPdi.bSensorFusionMeasResidualMonitorEnable, false, true, (sizeof(tPdi.bSensorFusionMeasResidualMonitorEnable) / sizeof(bool_t)) }; /* Sensor Fusion Measurement Residual Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16InvalidHallSensorMonitorCountLimit = { &tPdi.u16InvalidHallSensorMonitorCountLimit, 0U, 65535U, (sizeof(tPdi.u16InvalidHallSensorMonitorCountLimit) / sizeof(uint16_t)) }; /* Invalid Hall Sensor Monitor Count Limit */
static const tPdiRange_bool_t tPdiRange_bA825CntrlBusPassiveMonitorEnable = { &tPdi.bA825CntrlBusPassiveMonitorEnable, false, true, (sizeof(tPdi.bA825CntrlBusPassiveMonitorEnable) / sizeof(bool_t)) }; /* ARINC825 Control Bus Passive Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bA825MaintBusPassiveMonitorEnable = { &tPdi.bA825MaintBusPassiveMonitorEnable, false, true, (sizeof(tPdi.bA825MaintBusPassiveMonitorEnable) / sizeof(bool_t)) }; /* ARINC825 Maintenance Bus Passive Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bA825CntrlBusOffMonitorEnable = { &tPdi.bA825CntrlBusOffMonitorEnable, false, true, (sizeof(tPdi.bA825CntrlBusOffMonitorEnable) / sizeof(bool_t)) }; /* ARINC825 Control Bus Off Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bA825MaintBusOffMonitorEnable = { &tPdi.bA825MaintBusOffMonitorEnable, false, true, (sizeof(tPdi.bA825MaintBusOffMonitorEnable) / sizeof(bool_t)) }; /* ARINC825 Maintenance Bus Off Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bA825CntrlBusCommsStaleMonitorEnable = { &tPdi.bA825CntrlBusCommsStaleMonitorEnable, false, true, (sizeof(tPdi.bA825CntrlBusCommsStaleMonitorEnable) / sizeof(bool_t)) }; /* ARINC825 Control Bus Comms Stale Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bA825MaintBusCommsStaleMonitorEnable = { &tPdi.bA825MaintBusCommsStaleMonitorEnable, false, true, (sizeof(tPdi.bA825MaintBusCommsStaleMonitorEnable) / sizeof(bool_t)) }; /* ARINC825 Maintenance Bus Comms Stale Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bDCBusMonitorEnable = { &tPdi.bDCBusMonitorEnable, false, true, (sizeof(tPdi.bDCBusMonitorEnable) / sizeof(bool_t)) }; /* DC Bus Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16DCBusMonitorVCLL = { &tPdi.u16DCBusMonitorVCLL, 1U, 4095U, (sizeof(tPdi.u16DCBusMonitorVCLL) / sizeof(uint16_t)) }; /* DC Bus Variable Camber Lower Limit */
static const tPdiRange_uint16_t tPdiRange_u16DCBusMonitorVCUL = { &tPdi.u16DCBusMonitorVCUL, 1U, 4095U, (sizeof(tPdi.u16DCBusMonitorVCUL) / sizeof(uint16_t)) }; /* DC Bus Variable Camber Upper Limit */
static const tPdiRange_uint16_t tPdiRange_u16DCBusMonitorHLLL = { &tPdi.u16DCBusMonitorHLLL, 1U, 4095U, (sizeof(tPdi.u16DCBusMonitorHLLL) / sizeof(uint16_t)) }; /* DC Bus High Lift Lower Limit */
static const tPdiRange_uint16_t tPdiRange_u16DCBusMonitorHLUL = { &tPdi.u16DCBusMonitorHLUL, 1U, 4095U, (sizeof(tPdi.u16DCBusMonitorHLUL) / sizeof(uint16_t)) }; /* DC Bus High Lift Upper Limit */
static const tPdiRange_bool_t tPdiRange_bp28VDCMonitorEnable = { &tPdi.bp28VDCMonitorEnable, false, true, (sizeof(tPdi.bp28VDCMonitorEnable) / sizeof(bool_t)) }; /* p28 VDC Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16p28VDCMonitorLL = { &tPdi.u16p28VDCMonitorLL, 1U, 4095U, (sizeof(tPdi.u16p28VDCMonitorLL) / sizeof(uint16_t)) }; /* p28 VDC Monitor Lower Limit */
static const tPdiRange_uint16_t tPdiRange_u16p28VDCMonitorUL = { &tPdi.u16p28VDCMonitorUL, 1U, 4095U, (sizeof(tPdi.u16p28VDCMonitorUL) / sizeof(uint16_t)) }; /* p28 VDC Monitor Upper Limit */
static const tPdiRange_bool_t tPdiRange_bp15VDCMonitorEnable = { &tPdi.bp15VDCMonitorEnable, false, true, (sizeof(tPdi.bp15VDCMonitorEnable) / sizeof(bool_t)) }; /* p15 VDC Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16p15VDCMonitorLL = { &tPdi.u16p15VDCMonitorLL, 1U, 4095U, (sizeof(tPdi.u16p15VDCMonitorLL) / sizeof(uint16_t)) }; /* p15 VDC Monitor Lower Limit */
static const tPdiRange_uint16_t tPdiRange_u16p15VDCMonitorUL = { &tPdi.u16p15VDCMonitorUL, 1U, 4095U, (sizeof(tPdi.u16p15VDCMonitorUL) / sizeof(uint16_t)) }; /* p15 VDC Monitor Upper Limit */
static const tPdiRange_bool_t tPdiRange_bn15VDCMonitorEnable = { &tPdi.bn15VDCMonitorEnable, false, true, (sizeof(tPdi.bn15VDCMonitorEnable) / sizeof(bool_t)) }; /* n15 VDC Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16n15VDCMonitorLL = { &tPdi.u16n15VDCMonitorLL, 1U, 4095U, (sizeof(tPdi.u16n15VDCMonitorLL) / sizeof(uint16_t)) }; /* n15 VDC Monitor Lower Limit */
static const tPdiRange_uint16_t tPdiRange_u16n15VDCMonitorUL = { &tPdi.u16n15VDCMonitorUL, 1U, 4095U, (sizeof(tPdi.u16n15VDCMonitorUL) / sizeof(uint16_t)) }; /* n15 VDC Monitor Upper Limit */
static const tPdiRange_bool_t tPdiRange_bp5VDCMonitorEnable = { &tPdi.bp5VDCMonitorEnable, false, true, (sizeof(tPdi.bp5VDCMonitorEnable) / sizeof(bool_t)) }; /* p5 VDC Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16p5VDCMonitorLL = { &tPdi.u16p5VDCMonitorLL, 1U, 4095U, (sizeof(tPdi.u16p5VDCMonitorLL) / sizeof(uint16_t)) }; /* p5 VDC Monitor Lower Limit */
static const tPdiRange_uint16_t tPdiRange_u16p5VDCMonitorUL = { &tPdi.u16p5VDCMonitorUL, 1U, 4095U, (sizeof(tPdi.u16p5VDCMonitorUL) / sizeof(uint16_t)) }; /* p5 VDC Monitor Upper Limit */
static const tPdiRange_bool_t tPdiRange_bGateDriverMonitorEnable = { &tPdi.bGateDriverMonitorEnable, false, true, (sizeof(tPdi.bGateDriverMonitorEnable) / sizeof(bool_t)) }; /* Gate Driver Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bPDICrcCheckMonitorEnable = { &tPdi.bPDICrcCheckMonitorEnable, false, true, (sizeof(tPdi.bPDICrcCheckMonitorEnable) / sizeof(bool_t)) }; /* PDI CRC Check Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bPDIRangeCheckMonitorEnable = { &tPdi.bPDIRangeCheckMonitorEnable, false, true, (sizeof(tPdi.bPDIRangeCheckMonitorEnable) / sizeof(bool_t)) }; /* PDI Range Check Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bMRAMCheckMonitorEnable = { &tPdi.bMRAMCheckMonitorEnable, false, true, (sizeof(tPdi.bMRAMCheckMonitorEnable) / sizeof(bool_t)) }; /* MRAM Check Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bGFDMonitorEnable = { &tPdi.bGFDMonitorEnable, false, true, (sizeof(tPdi.bGFDMonitorEnable) / sizeof(bool_t)) }; /* Ground Fault Detection Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bA825MaintBusFlapCmdSkewMonitorEnable = { &tPdi.bA825MaintBusFlapCmdSkewMonitorEnable, false, true, (sizeof(tPdi.bA825MaintBusFlapCmdSkewMonitorEnable) / sizeof(bool_t)) }; /* ARINC825 Maintenance Bus Flap Command Skew Monitor Enable */
static const tPdiRange_float32_t tPdiRange_f32A825MaintBusFlapSkewMaxDiffAllowed = { &tPdi.f32A825MaintBusFlapSkewMaxDiffAllowed, 0.0F, 1.0F, (sizeof(tPdi.f32A825MaintBusFlapSkewMaxDiffAllowed) / sizeof(float32_t)) }; /* ARINC825 Maintenance Bus Flap Command Skew Maximum Position Command Difference */
static const tPdiRange_bool_t tPdiRange_bPhNShortAsymMonitorEnable = { &tPdi.bPhNShortAsymMonitorEnable, false, true, (sizeof(tPdi.bPhNShortAsymMonitorEnable) / sizeof(bool_t)) }; /* Phase to Neutral Asymmetry Short Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16PhNShortAsymHLUL = { &tPdi.u16PhNShortAsymHLUL, 0U, 4095U, (sizeof(tPdi.u16PhNShortAsymHLUL) / sizeof(uint16_t)) }; /* Phase to Neutral Asymmetry Short High Lift Upper Limit */
static const tPdiRange_uint16_t tPdiRange_u16PhNShortAsymVCUL = { &tPdi.u16PhNShortAsymVCUL, 0U, 4095U, (sizeof(tPdi.u16PhNShortAsymVCUL) / sizeof(uint16_t)) }; /* Phase to Neutral Asymmetry Short Variable Camber Upper Limit */
static const tPdiRange_float32_t tPdiRange_f32UnbalCurIndexHLUL = { &tPdi.f32UnbalCurIndexHLUL, 0.0F, 1.0F, (sizeof(tPdi.f32UnbalCurIndexHLUL) / sizeof(float32_t)) }; /* Unbalance Current Index Upper Limit HL */
static const tPdiRange_bool_t tPdiRange_bOnePhOpenCktMonitorEnable = { &tPdi.bOnePhOpenCktMonitorEnable, false, true, (sizeof(tPdi.bOnePhOpenCktMonitorEnable) / sizeof(bool_t)) }; /* One Phase Open Circuit Monitor Enable */
static const tPdiRange_uint16_t tPdiRange_u16OnePhOpenCktLL = { &tPdi.u16OnePhOpenCktLL, 0U, 4095U, (sizeof(tPdi.u16OnePhOpenCktLL) / sizeof(uint16_t)) }; /* One Phase Open Circuit Lower Limit */
static const tPdiRange_float32_t tPdiRange_f32UnbalCurIndexVCUL = { &tPdi.f32UnbalCurIndexVCUL, 0.0F, 1.0F, (sizeof(tPdi.f32UnbalCurIndexVCUL) / sizeof(float32_t)) }; /* Unbalance Current Index Upper Limit VC */
static const tPdiRange_bool_t tPdiRange_bEncoderHealthMonitorEnable = { &tPdi.bEncoderHealthMonitorEnable, 0, 1, (sizeof(tPdi.bEncoderHealthMonitorEnable) / sizeof(bool_t)) }; /* Encoder Health Monitor Enable */
static const tPdiRange_bool_t tPdiRange_bIncoderRxReadyChkEnable = { &tPdi.bIncoderRxReadyChkEnable, 0, 1, (sizeof(tPdi.bIncoderRxReadyChkEnable) / sizeof(bool_t)) }; /* Incoder Rx Ready Chk Enable */
static const tPdiRange_uint16_t tPdiRange_u16IncoderRxReadyChkThreshold = { &tPdi.u16IncoderRxReadyChkThreshold, 0U, 32768U, (sizeof(tPdi.u16IncoderRxReadyChkThreshold) / sizeof(uint16_t)) }; /* Incoder Rx Ready Chk Threshold */
static const tPdiRange_uint16_t tPdiRange_u16IncoderRxReadyChkIncCount = { &tPdi.u16IncoderRxReadyChkIncCount, 0U, 32768U, (sizeof(tPdi.u16IncoderRxReadyChkIncCount) / sizeof(uint16_t)) }; /* Incoder Rx Ready Chk Inc Count */
static const tPdiRange_uint16_t tPdiRange_u16IncoderRxReadyChkDecCount = { &tPdi.u16IncoderRxReadyChkDecCount, 0U, 32768U, (sizeof(tPdi.u16IncoderRxReadyChkDecCount) / sizeof(uint16_t)) }; /* Incoder Rx Ready Chk Dec Count */
static const tPdiRange_bool_t tPdiRange_bIncoderPvInvalidChkEnable = { &tPdi.bIncoderPvInvalidChkEnable, 0, 1, (sizeof(tPdi.bIncoderPvInvalidChkEnable) / sizeof(bool_t)) }; /* Incoder Pv Invalid Chk Enable */
static const tPdiRange_uint16_t tPdiRange_u16IncoderPvInvalidChkThreshold = { &tPdi.u16IncoderPvInvalidChkThreshold, 0U, 32768U, (sizeof(tPdi.u16IncoderPvInvalidChkThreshold) / sizeof(uint16_t)) }; /* Incoder Pv Invalid Chk Threshold */
static const tPdiRange_uint16_t tPdiRange_u16IncoderPvInvalidChkIncCount = { &tPdi.u16IncoderPvInvalidChkIncCount, 0U, 32768U, (sizeof(tPdi.u16IncoderPvInvalidChkIncCount) / sizeof(uint16_t)) }; /* Incoder Pv Invalid Chk Inc Count */
static const tPdiRange_uint16_t tPdiRange_u16IncoderPvInvalidChkDecCount = { &tPdi.u16IncoderPvInvalidChkDecCount, 0U, 32768U, (sizeof(tPdi.u16IncoderPvInvalidChkDecCount) / sizeof(uint16_t)) }; /* Incoder Pv Invalid Chk Dec Count */
static const tPdiRange_bool_t tPdiRange_bIncoderCrcChkEnable = { &tPdi.bIncoderCrcChkEnable, 0, 1, (sizeof(tPdi.bIncoderCrcChkEnable) / sizeof(bool_t)) }; /* Incoder Crc Chk Enable */
static const tPdiRange_uint16_t tPdiRange_u16IncoderCrcChkThreshold = { &tPdi.u16IncoderCrcChkThreshold, 0U, 32768U, (sizeof(tPdi.u16IncoderCrcChkThreshold) / sizeof(uint16_t)) }; /* Incoder Crc Chk Threshold */
static const tPdiRange_uint16_t tPdiRange_u16IncoderCrcChkIncCount = { &tPdi.u16IncoderCrcChkIncCount, 0U, 32768U, (sizeof(tPdi.u16IncoderCrcChkIncCount) / sizeof(uint16_t)) }; /* Incoder Crc Chk Inc Count */
static const tPdiRange_uint16_t tPdiRange_u16IncoderCrcChkDecCount = { &tPdi.u16IncoderCrcChkDecCount, 0U, 32768U, (sizeof(tPdi.u16IncoderCrcChkDecCount) / sizeof(uint16_t)) }; /* Incoder Crc Chk Dec Count */
/* Range Data constants defined for each PDI */

/* PDI Range Data Pointer Array */
static const tPdiRange_t tPdiRange[NUM_PDIs] =  {
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bSyncMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bJamMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16JamMonitorQuadLimit },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16JamMonitorCurrentLimit },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bxChMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bxChCommMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16xChCommMonitorFailLimit },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bRigValueMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bRigModeFaultMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bBacklashFreePlayMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16BacklashFreeplayMonitorQuadLimit },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bHallSensorSeqMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16HallSensorSeqMonitorIllegalLimit },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bInvalidHallSensorMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bBrakeHoldMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bBrakeSwitchMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16BrakeSwitchMonitorBrkOffMin },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16BrakeSwitchMonitorBrkOffMax },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16BrakeSwitchMonitorBrkUpperMin },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16BrakeSwitchMonitorBrkUpperMax },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16BrakeSwitchMonitorBrkLowerMax },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bFpsuExcMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16FpsuExcMonitorSumIdeal },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16FpsuExcMonitorSumTolerance },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bFpsuPosMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16FpsuPosMonitorDeltaThreshold },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bBridgeMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16BridgeMonitorBridgeNoneMin },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16BridgeMonitorBridgeNoneMax },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16BridgeMonitorBridgeUpperMin },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16BridgeMonitorBridgeLowerMax },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bProgPinMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bRiggingCheckMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16RiggingCheckMonitorQuadDiff },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bROMCrcCheckMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bRAMCheckMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bWDTimerPowerUpMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16WDTimerPowerUpMonitorMSec },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bFrameOverrunMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bSubframeIndexMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bUndefinedINTMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bStackOverrunMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bOverCurrentMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bOverTempMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bSensorFusionBiasCheckMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bSensorFusionMeasResidualMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16InvalidHallSensorMonitorCountLimit },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bA825CntrlBusPassiveMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bA825MaintBusPassiveMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bA825CntrlBusOffMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bA825MaintBusOffMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bA825CntrlBusCommsStaleMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bA825MaintBusCommsStaleMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bDCBusMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16DCBusMonitorVCLL },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16DCBusMonitorVCUL },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16DCBusMonitorHLLL },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16DCBusMonitorHLUL },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bp28VDCMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16p28VDCMonitorLL },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16p28VDCMonitorUL },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bp15VDCMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16p15VDCMonitorLL },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16p15VDCMonitorUL },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bn15VDCMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16n15VDCMonitorLL },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16n15VDCMonitorUL },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bp5VDCMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16p5VDCMonitorLL },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16p5VDCMonitorUL },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bGateDriverMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bPDICrcCheckMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bPDIRangeCheckMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bMRAMCheckMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bGFDMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bA825MaintBusFlapCmdSkewMonitorEnable },
													{ PDI_TYPE_float32_t, (void*) &tPdiRange_f32A825MaintBusFlapSkewMaxDiffAllowed },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bPhNShortAsymMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16PhNShortAsymHLUL },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16PhNShortAsymVCUL },
													{ PDI_TYPE_float32_t, (void*) &tPdiRange_f32UnbalCurIndexHLUL },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bOnePhOpenCktMonitorEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16OnePhOpenCktLL },
													{ PDI_TYPE_float32_t, (void*) &tPdiRange_f32UnbalCurIndexVCUL },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bEncoderHealthMonitorEnable },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bIncoderRxReadyChkEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16IncoderRxReadyChkThreshold },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16IncoderRxReadyChkIncCount },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16IncoderRxReadyChkDecCount },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bIncoderPvInvalidChkEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16IncoderPvInvalidChkThreshold },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16IncoderPvInvalidChkIncCount },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16IncoderPvInvalidChkDecCount },
													{ PDI_TYPE_bool_t, (void*) &tPdiRange_bIncoderCrcChkEnable },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16IncoderCrcChkThreshold },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16IncoderCrcChkIncCount },
													{ PDI_TYPE_uint16_t, (void*) &tPdiRange_u16IncoderCrcChkDecCount },
                                                };

/* VARIABLES ************************************************************************************/

/* FUNCTION PROTOTYPES **************************************************************************/
static bool_t PDI_Mgr_RangeCheck_bool_t(tPdiRange_bool_t* const ptPdiRange);
static bool_t PDI_Mgr_RangeCheck_char16_t(tPdiRange_char16_t* const ptPdiRange);
static bool_t PDI_Mgr_RangeCheck_uchar16_t(tPdiRange_uchar16_t* const ptPdiRange);
static bool_t PDI_Mgr_RangeCheck_int16_t(tPdiRange_int16_t* const ptPdiRange);
static bool_t PDI_Mgr_RangeCheck_int32_t(tPdiRange_int32_t* const ptPdiRange);
static bool_t PDI_Mgr_RangeCheck_int64_t(tPdiRange_int64_t* const ptPdiRange);
static bool_t PDI_Mgr_RangeCheck_uint16_t(tPdiRange_uint16_t* const ptPdiRange);
static bool_t PDI_Mgr_RangeCheck_uint32_t(tPdiRange_uint32_t* const ptPdiRange);
static bool_t PDI_Mgr_RangeCheck_uint64_t(tPdiRange_uint64_t* const ptPdiRange);
static bool_t PDI_Mgr_RangeCheck_float32_t(tPdiRange_float32_t* const ptPdiRange);
static bool_t PDI_Mgr_RangeCheck_float64_t(tPdiRange_float64_t* const ptPdiRange);

/* FUNCTION DEFINITIONS *************************************************************************/

/*************************************************************************************************\
* Function: PDI_Mgr_RangeCheck
*
* Purpose:  Range check all PDIs
*
* Input(s):         none
* Output(s):        false if range check passed for every PDI
*                   true if range check failed at least one PDI range check.
**************************************************************************************************/
bool_t PDI_Mgr_RangeCheck(void)
{
    bool_t bRet = false;
    uint16_t i = 0U;
    ePdiDataTypes_t ePdiDataType;

    if(NUM_PDIs > 0U)
    {
        /* Number of PDIs is valid. Range check them */

        /* Loop through all configured PDIs as long as no error is detected in the range check. */
        do
        {
            /* Get the data type of the PDI */
            ePdiDataType = tPdiRange[i].eDataType;

            /* Decode the data type */
            switch (ePdiDataType)
            {
                case PDI_TYPE_bool_t:
                {
                    bRet = PDI_Mgr_RangeCheck_bool_t((tPdiRange_bool_t*) tPdiRange[i].pvtPdiRange);
                    break;
                }
                case PDI_TYPE_char16_t:
                {
                    bRet = PDI_Mgr_RangeCheck_char16_t((tPdiRange_char16_t*) tPdiRange[i].pvtPdiRange);
                    break;
                }
                case PDI_TYPE_uchar16_t:
                {
                    bRet = PDI_Mgr_RangeCheck_uchar16_t((tPdiRange_uchar16_t*) tPdiRange[i].pvtPdiRange);
                    break;
                }
                case PDI_TYPE_int16_t:
                {
                    bRet = PDI_Mgr_RangeCheck_int16_t((tPdiRange_int16_t*) tPdiRange[i].pvtPdiRange);
                    break;
                }
                case PDI_TYPE_int32_t:
                {
                    bRet = PDI_Mgr_RangeCheck_int32_t((tPdiRange_int32_t*) tPdiRange[i].pvtPdiRange);
                    break;
                }
                case PDI_TYPE_int64_t:
                {
                    bRet = PDI_Mgr_RangeCheck_int64_t((tPdiRange_int64_t*) tPdiRange[i].pvtPdiRange);
                    break;
                }
                case PDI_TYPE_uint16_t:
                {
                    bRet = PDI_Mgr_RangeCheck_uint16_t((tPdiRange_uint16_t*) tPdiRange[i].pvtPdiRange);
                    break;
                }
                case PDI_TYPE_uint32_t:
                {
                    bRet = PDI_Mgr_RangeCheck_uint32_t((tPdiRange_uint32_t*) tPdiRange[i].pvtPdiRange);
                    break;
                }
                case PDI_TYPE_uint64_t:
                {
                    bRet = PDI_Mgr_RangeCheck_uint64_t((tPdiRange_uint64_t*) tPdiRange[i].pvtPdiRange);
                    break;
                }
                case PDI_TYPE_float32_t:
                {
                    bRet = PDI_Mgr_RangeCheck_float32_t((tPdiRange_float32_t*) tPdiRange[i].pvtPdiRange);
                    break;
                }
                case PDI_TYPE_float64_t:
                {
                    bRet = PDI_Mgr_RangeCheck_float64_t((tPdiRange_float64_t*) tPdiRange[i].pvtPdiRange);
                    break;
                }
                default:
                {
                    /* Invalid Data Type. Fail the PDI check */
                    bRet = true;
                    break;
                }
            }
            i++;
        } while ((i < NUM_PDIs) && (bRet == false)); /* Loop while there are still PDI range entries to parse and there have been no range check failures */
    }
    else
    {
        /* Invalid number of PDIs. Declare error and exit. */
        bRet = true;
    }

    return bRet;
}

/*************************************************************************************************\
* Function: PDI_Mgr_RangeCheck_bool_t
*
* Purpose:  Range check a PDI of type bool_t
*
* Input(s):         ptPdiRange,  pointer to the PDI range bool_t data structure
* Output(s):        false if range check passed, true if range check failed.
**************************************************************************************************/
static bool_t PDI_Mgr_RangeCheck_bool_t(tPdiRange_bool_t* const ptPdiRange)
{
    bool_t bRet = false;        /* Return flag used to indicate pass/fail status of range check */
    uint16_t i = 0U;            /* Index used to loop thru all elements of PDI parameter */
    const bool_t* pPdiData;     /* Pointer to the PDI Data. Type must match that of the PDI */

    /* Null check and check for at least one element to range check */
    if(ptPdiRange != NULL)
    {
        if(ptPdiRange->n != 0U)
        {
            /* Setup the PDI Data pointer */
            pPdiData = ptPdiRange->pbPdi;
            /* Range check all elements of the parameter. */
            do
            {
                /* Null check the PDI Data pointer */
                if(pPdiData != NULL)
                {
                    if((*pPdiData < ptPdiRange->minVal) || (*pPdiData > ptPdiRange->maxVal))
                    {
                        /* Range check failed */
                        bRet = true;
                    }
                    i++;
                    pPdiData++; /* increment the data pointer to next element. */
                }
                else
                {
                    /* Null pointer check failed. Set to range check fail and exit. */
                    bRet = true;
                }
            } while((i < ptPdiRange->n) && (bRet != true)); /* Exit if an element failed the check or all elements have passed */
        }
        else
        {
            /* Invalid number of elements to range check */
            bRet = true;
        }
    }
    else
    {
        /* Null pointer or invalid size of parameter. Range check failed */
        bRet = true;
    }

    return bRet;
}

/*************************************************************************************************\
* Function: PDI_Mgr_RangeCheck_char16_t
*
* Purpose:  Range check a PDI of type char16_t
*
* Input(s):         ptPdiRange,  pointer to the PDI range char16_t data structure
* Output(s):        false if range check passed, true if range check failed.
**************************************************************************************************/
static bool_t PDI_Mgr_RangeCheck_char16_t(tPdiRange_char16_t* const ptPdiRange)
{
    bool_t bRet = false;        /* Return flag used to indicate pass/fail status of range check */
    uint16_t i = 0U;            /* Index used to loop thru all elements of PDI parameter */
    const char16_t* pPdiData;   /* Pointer to the PDI Data. Type must match that of the PDI */

    /* Null check and check for at least one element to range check */
    if(ptPdiRange != NULL)
    {
        if(ptPdiRange->n != 0U)
        {
            /* Setup the PDI Data pointer */
            pPdiData = ptPdiRange->pcPdi;
            /* Range check all elements of the parameter. */
            do
            {
                /* Null check the PDI Data pointer */
                if(pPdiData != NULL)
                {
                    if((*pPdiData < ptPdiRange->minVal) || (*pPdiData > ptPdiRange->maxVal))
                    {
                        /* Range check failed */
                        bRet = true;
                    }
                    i++;
                    pPdiData++; /* increment the data pointer to next element. */
                }
                else
                {
                    /* Null pointer check failed. Set to range check fail and exit. */
                    bRet = true;
                }
            } while((i < ptPdiRange->n) && (bRet != true)); /* Exit if an element failed the check or all elements have passed */
        }
        else
        {
            /* Invalid number of elements to range check */
            bRet = true;
        }
    }
    else
    {
        /* Null pointer or invalid size of parameter. Range check failed */
        bRet = true;
    }

    return bRet;
}

/*************************************************************************************************\
* Function: PDI_Mgr_RangeCheck_uchar16_t
*
* Purpose:  Range check a PDI of type uchar16_t
*
* Input(s):         ptPdiRange,  pointer to the PDI range uchar16_t data structure
* Output(s):        false if range check passed, true if range check failed.
**************************************************************************************************/
static bool_t PDI_Mgr_RangeCheck_uchar16_t(tPdiRange_uchar16_t* const ptPdiRange)
{
    bool_t bRet = false;        /* Return flag used to indicate pass/fail status of range check */
    uint16_t i = 0U;            /* Index used to loop thru all elements of PDI parameter */
    const uchar16_t* pPdiData;  /* Pointer to the PDI Data. Type must match that of the PDI */

    /* Null check and check for at least one element to range check */
    if(ptPdiRange != NULL)
    {
        if(ptPdiRange->n != 0U)
        {
            /* Setup the PDI Data pointer */
            pPdiData = ptPdiRange->pucPdi;
            /* Range check all elements of the parameter. */
            do
            {
                /* Null check the PDI Data pointer */
                if(pPdiData != NULL)
                {
                    if((*pPdiData < ptPdiRange->minVal) || (*pPdiData > ptPdiRange->maxVal))
                    {
                        /* Range check failed */
                        bRet = true;
                    }
                    i++;
                    pPdiData++; /* increment the data pointer to next element. */
                }
                else
                {
                    /* Null pointer check failed. Set to range check fail and exit. */
                    bRet = true;
                }
            } while((i < ptPdiRange->n) && (bRet != true)); /* Exit if an element failed the check or all elements have passed */
        }
        else
        {
            /* Invalid number of elements to range check */
            bRet = true;
        }
    }
    else
    {
        /* Null pointer or invalid size of parameter. Range check failed */
        bRet = true;
    }

    return bRet;
}

/*************************************************************************************************\
* Function: PDI_Mgr_RangeCheck_int16_t
*
* Purpose:  Range check a PDI of type int16_t
*
* Input(s):         ptPdiRange,  pointer to the PDI range int16_t data structure
* Output(s):        false if range check passed, true if range check failed.
**************************************************************************************************/
static bool_t PDI_Mgr_RangeCheck_int16_t(tPdiRange_int16_t* const ptPdiRange)
{
    bool_t bRet = false;        /* Return flag used to indicate pass/fail status of range check */
    uint16_t i = 0U;            /* Index used to loop thru all elements of PDI parameter */
    const int16_t* pPdiData;    /* Pointer to the PDI Data. Type must match that of the PDI */

    /* Null check and check for at least one element to range check */
    if(ptPdiRange != NULL)
    {
        if(ptPdiRange->n != 0U)
        {
            /* Setup the PDI Data pointer */
            pPdiData = ptPdiRange->ps16Pdi;
            /* Range check all elements of the parameter. */
            do
            {
                /* Null check the PDI Data pointer */
                if(pPdiData != NULL)
                {
                    if((*pPdiData < ptPdiRange->minVal) || (*pPdiData > ptPdiRange->maxVal))
                    {
                        /* Range check failed */
                        bRet = true;
                    }
                    i++;
                    pPdiData++; /* increment the data pointer to next element. */
                }
                else
                {
                    /* Null pointer check failed. Set to range check fail and exit. */
                    bRet = true;
                }
            } while((i < ptPdiRange->n) && (bRet != true)); /* Exit if an element failed the check or all elements have passed */
        }
        else
        {
            /* Invalid number of elements to range check */
            bRet = true;
        }
    }
    else
    {
        /* Null pointer or invalid size of parameter. Range check failed */
        bRet = true;
    }

    return bRet;
}

/*************************************************************************************************\
* Function: PDI_Mgr_RangeCheck_int32_t
*
* Purpose:  Range check a PDI of type int32_t
*
* Input(s):         ptPdiRange,  pointer to the PDI range int32_t data structure
* Output(s):        false if range check passed, true if range check failed.
**************************************************************************************************/
static bool_t PDI_Mgr_RangeCheck_int32_t(tPdiRange_int32_t* const ptPdiRange)
{
    bool_t bRet = false;        /* Return flag used to indicate pass/fail status of range check */
    uint16_t i = 0U;            /* Index used to loop thru all elements of PDI parameter */
    const int32_t* pPdiData;    /* Pointer to the PDI Data. Type must match that of the PDI */

    /* Null check and check for at least one element to range check */
    if(ptPdiRange != NULL)
    {
        if(ptPdiRange->n != 0U)
        {
            /* Setup the PDI Data pointer */
            pPdiData = ptPdiRange->ps32Pdi;
            /* Range check all elements of the parameter. */
            do
            {
                /* Null check the PDI Data pointer */
                if(pPdiData != NULL)
                {
                    if((*pPdiData < ptPdiRange->minVal) || (*pPdiData > ptPdiRange->maxVal))
                    {
                        /* Range check failed */
                        bRet = true;
                    }
                    i++;
                    pPdiData++; /* increment the data pointer to next element. */
                }
                else
                {
                    /* Null pointer check failed. Set to range check fail and exit. */
                    bRet = true;
                }
            } while((i < ptPdiRange->n) && (bRet != true)); /* Exit if an element failed the check or all elements have passed */
        }
        else
        {
            /* Invalid number of elements to range check */
            bRet = true;
        }
    }
    else
    {
        /* Null pointer or invalid size of parameter. Range check failed */
        bRet = true;
    }

    return bRet;
}

/*************************************************************************************************\
* Function: PDI_Mgr_RangeCheck_int64_t
*
* Purpose:  Range check a PDI of type int64_t
*
* Input(s):         ptPdiRange,  pointer to the PDI range int64_t data structure
* Output(s):        false if range check passed, true if range check failed.
**************************************************************************************************/
static bool_t PDI_Mgr_RangeCheck_int64_t(tPdiRange_int64_t* const ptPdiRange)
{
    bool_t bRet = false;        /* Return flag used to indicate pass/fail status of range check */
    uint16_t i = 0U;            /* Index used to loop thru all elements of PDI parameter */
    const int64_t* pPdiData;    /* Pointer to the PDI Data. Type must match that of the PDI */

    /* Null check and check for at least one element to range check */
    if(ptPdiRange != NULL)
    {
        if(ptPdiRange->n != 0U)
        {
            /* Setup the PDI Data pointer */
            pPdiData = ptPdiRange->ps64Pdi;
            /* Range check all elements of the parameter. */
            do
            {
                /* Null check the PDI Data pointer */
                if(pPdiData != NULL)
                {
                    if((*pPdiData < ptPdiRange->minVal) || (*pPdiData > ptPdiRange->maxVal))
                    {
                        /* Range check failed */
                        bRet = true;
                    }
                    i++;
                    pPdiData++; /* increment the data pointer to next element. */
                }
                else
                {
                    /* Null pointer check failed. Set to range check fail and exit. */
                    bRet = true;
                }
            } while((i < ptPdiRange->n) && (bRet != true)); /* Exit if an element failed the check or all elements have passed */
        }
        else
        {
            /* Invalid number of elements to range check */
            bRet = true;
        }
    }
    else
    {
        /* Null pointer or invalid size of parameter. Range check failed */
        bRet = true;
    }

    return bRet;
}

/*************************************************************************************************\
* Function: PDI_Mgr_RangeCheck_uint16_t
*
* Purpose:  Range check a PDI of type uint16_t
*
* Input(s):         ptPdiRange,  pointer to the PDI range uint16_t data structure
* Output(s):        false if range check passed, true if range check failed.
**************************************************************************************************/
static bool_t PDI_Mgr_RangeCheck_uint16_t(tPdiRange_uint16_t* const ptPdiRange)
{
    bool_t bRet = false;        /* Return flag used to indicate pass/fail status of range check */
    uint16_t i = 0U;            /* Index used to loop thru all elements of PDI parameter */
    const uint16_t* pPdiData;   /* Pointer to the PDI Data. Type must match that of the PDI */

    /* Null check and check for at least one element to range check */
    if(ptPdiRange != NULL)
    {
        if(ptPdiRange->n != 0U)
        {
            /* Setup the PDI Data pointer */
            pPdiData = ptPdiRange->pu16Pdi;
            /* Range check all elements of the parameter. */
            do
            {
                /* Null check the PDI Data pointer */
                if(pPdiData != NULL)
                {
                    if((*pPdiData < ptPdiRange->minVal) || (*pPdiData > ptPdiRange->maxVal))
                    {
                        /* Range check failed */
                        bRet = true;
                    }
                    i++;
                    pPdiData++; /* increment the data pointer to next element. */
                }
                else
                {
                    /* Null pointer check failed. Set to range check fail and exit. */
                    bRet = true;
                }
            } while((i < ptPdiRange->n) && (bRet != true)); /* Exit if an element failed the check or all elements have passed */
        }
        else
        {
            /* Invalid number of elements to range check */
            bRet = true;
        }
    }
    else
    {
        /* Null pointer or invalid size of parameter. Range check failed */
        bRet = true;
    }

    return bRet;
}

/*************************************************************************************************\
* Function: PDI_Mgr_RangeCheck_uint32_t
*
* Purpose:  Range check a PDI of type uint32_t
*
* Input(s):         ptPdiRange,  pointer to the PDI range uint32_t data structure
* Output(s):        false if range check passed, true if range check failed.
**************************************************************************************************/
static bool_t PDI_Mgr_RangeCheck_uint32_t(tPdiRange_uint32_t* const ptPdiRange)
{
    bool_t bRet = false;        /* Return flag used to indicate pass/fail status of range check */
    uint16_t i = 0U;            /* Index used to loop thru all elements of PDI parameter */
    const uint32_t* pPdiData;   /* Pointer to the PDI Data. Type must match that of the PDI */

    /* Null check and check for at least one element to range check */
    if(ptPdiRange != NULL)
    {
        if(ptPdiRange->n != 0U)
        {
            /* Setup the PDI Data pointer */
            pPdiData = ptPdiRange->pu32Pdi;
            /* Range check all elements of the parameter. */
            do
            {
                /* Null check the PDI Data pointer */
                if(pPdiData != NULL)
                {
                    if((*pPdiData < ptPdiRange->minVal) || (*pPdiData > ptPdiRange->maxVal))
                    {
                        /* Range check failed */
                        bRet = true;
                    }
                    i++;
                    pPdiData++; /* increment the data pointer to next element. */
                }
                else
                {
                    /* Null pointer check failed. Set to range check fail and exit. */
                    bRet = true;
                }
            } while((i < ptPdiRange->n) && (bRet != true)); /* Exit if an element failed the check or all elements have passed */
        }
        else
        {
            /* Invalid number of elements to range check */
            bRet = true;
        }
    }
    else
    {
        /* Null pointer or invalid size of parameter. Range check failed */
        bRet = true;
    }

    return bRet;
}

/*************************************************************************************************\
* Function: PDI_Mgr_RangeCheck_uint64_t
*
* Purpose:  Range check a PDI of type uint64_t
*
* Input(s):         ptPdiRange,  pointer to the PDI range uint64_t data structure
* Output(s):        false if range check passed, true if range check failed.
**************************************************************************************************/
static bool_t PDI_Mgr_RangeCheck_uint64_t(tPdiRange_uint64_t* const ptPdiRange)
{
    bool_t bRet = false;        /* Return flag used to indicate pass/fail status of range check */
    uint16_t i = 0U;            /* Index used to loop thru all elements of PDI parameter */
    const uint64_t* pPdiData;   /* Pointer to the PDI Data. Type must match that of the PDI */

    /* Null check and check for at least one element to range check */
    if(ptPdiRange != NULL)
    {
        if(ptPdiRange->n != 0U)
        {
            /* Setup the PDI Data pointer */
            pPdiData = ptPdiRange->pu64Pdi;
            /* Range check all elements of the parameter. */
            do
            {
                /* Null check the PDI Data pointer */
                if(pPdiData != NULL)
                {
                    if((*pPdiData < ptPdiRange->minVal) || (*pPdiData > ptPdiRange->maxVal))
                    {
                        /* Range check failed */
                        bRet = true;
                    }
                    i++;
                    pPdiData++; /* increment the data pointer to next element. */
                }
                else
                {
                    /* Null pointer check failed. Set to range check fail and exit. */
                    bRet = true;
                }
            } while((i < ptPdiRange->n) && (bRet != true)); /* Exit if an element failed the check or all elements have passed */
        }
        else
        {
            /* Invalid number of elements to range check */
            bRet = true;
        }
    }
    else
    {
        /* Null pointer or invalid size of parameter. Range check failed */
        bRet = true;
    }

    return bRet;
}

/*************************************************************************************************\
* Function: PDI_Mgr_RangeCheck_float32_t
*
* Purpose:  Range check a PDI of type float32_t
*
* Input(s):         ptPdiRange,  pointer to the PDI range float32_t data structure
* Output(s):        false if range check passed, true if range check failed.
**************************************************************************************************/
static bool_t PDI_Mgr_RangeCheck_float32_t(tPdiRange_float32_t* const ptPdiRange)
{
    bool_t bRet = false;        /* Return flag used to indicate pass/fail status of range check */
    uint16_t i = 0U;            /* Index used to loop thru all elements of PDI parameter */
    const float32_t* pPdiData;  /* Pointer to the PDI Data. Type must match that of the PDI */

    /* Null check and check for at least one element to range check */
    if(ptPdiRange != NULL)
    {
        if(ptPdiRange->n != 0U)
        {
            /* Setup the PDI Data pointer */
            pPdiData = ptPdiRange->pf32Pdi;
            /* Range check all elements of the parameter. */
            do
            {
                /* Null check the PDI Data pointer */
                if(pPdiData != NULL)
                {
                    if((*pPdiData < ptPdiRange->minVal) || (*pPdiData > ptPdiRange->maxVal))
                    {
                        /* Range check failed */
                        bRet = true;
                    }
                    i++;
                    pPdiData++; /* increment the data pointer to next element. */
                }
                else
                {
                    /* Null pointer check failed. Set to range check fail and exit. */
                    bRet = true;
                }
            } while((i < ptPdiRange->n) && (bRet != true)); /* Exit if an element failed the check or all elements have passed */
        }
        else
        {
            /* Invalid number of elements to range check */
            bRet = true;
        }
    }
    else
    {
        /* Null pointer or invalid size of parameter. Range check failed */
        bRet = true;
    }

    return bRet;
}

/*************************************************************************************************\
* Function: PDI_Mgr_RangeCheck_float64_t
*
* Purpose:  Range check a PDI of type float64_t
*
* Input(s):         ptPdiRange,  pointer to the PDI range float64_t data structure
* Output(s):        false if range check passed, true if range check failed.
**************************************************************************************************/
static bool_t PDI_Mgr_RangeCheck_float64_t(tPdiRange_float64_t* const ptPdiRange)
{
    bool_t bRet = false;        /* Return flag used to indicate pass/fail status of range check */
    uint16_t i = 0U;            /* Index used to loop thru all elements of PDI parameter */
    const float64_t* pPdiData;  /* Pointer to the PDI Data. Type must match that of the PDI */

    /* Null check and check for at least one element to range check */
    if(ptPdiRange != NULL)
    {
        if(ptPdiRange->n != 0U)
        {
            /* Setup the PDI Data pointer */
            pPdiData = ptPdiRange->pf64PdiData;
            /* Range check all elements of the parameter. */
            do
            {
                /* Null check the PDI Data pointer */
                if(pPdiData != NULL)
                {
                    if((*pPdiData < ptPdiRange->minVal) || (*pPdiData > ptPdiRange->maxVal))
                    {
                        /* Range check failed */
                        bRet = true;
                    }
                    i++;
                    pPdiData++; /* increment the data pointer to next element. */
                }
                else
                {
                    /* Null pointer check failed. Set to range check fail and exit. */
                    bRet = true;
                }
            } while((i < ptPdiRange->n) && (bRet != true)); /* Exit if an element failed the check or all elements have passed */
        }
        else
        {
            /* Invalid number of elements to range check */
            bRet = true;
        }
    }
    else
    {
        /* Null pointer or invalid size of parameter. Range check failed */
        bRet = true;
    }

    return bRet;
}

