/**************************************************************************************************
 * File name: pdi.h
 *
 * Purpose : The pdi.h file contains the declarations for the Parameter Data Items (PDIs). Auto generated from PDI Gen program
 *
 * Project Name: PDI Coder Tool
 * 
 * Copyright Notice:
 * All source code and data contained in this file is Proprietary and Confidential to
 * Eaton Aerospace, and must not be reproduced, transmitted, or disclosed; in whole or in part,
 * without the express written permission of Eaton Aerospace.
 * Copyright (c) 2022 - Eaton Aerospace, All Rights Reserved.
 *
 * Author                   Date          Ver#      CR#         Description
 * -------------------      --------      ----      ----        ---------------
 * Adam Bouwens             06/29/2022    0.1       NA          Initial          
 * E0447863					08/01/2023
 **************************************************************************************************/

#ifndef PDI_H_
#define PDI_H_

/******************** INCLUDES *******************************************************************/
#include "typedefs.h"


/******************** STRUCTS ********************************************************************/
/* PDI Structure Definition. PDIs may be arrays; however arrays are limited to the native data
 * types only. Types exceeding 64 bits are not permitted. */
typedef struct
{
	bool_t bSyncMonitorEnable;                                 	/* PDI 1:  bSyncMonitorEnable */
	bool_t bJamMonitorEnable;                                  	/* PDI 2:  bJamMonitorEnable */
	uint16_t u16JamMonitorQuadLimit;                           	/* PDI 3:  u16JamMonitorQuadLimit */
	uint16_t u16JamMonitorCurrentLimit;                        	/* PDI 4:  u16JamMonitorCurrentLimit */
	bool_t bxChMonitorEnable;                                  	/* PDI 5:  bxChMonitorEnable */
	bool_t bxChCommMonitorEnable;                              	/* PDI 6:  bxChCommMonitorEnable */
	uint16_t u16xChCommMonitorFailLimit;                       	/* PDI 7:  u16xChCommMonitorFailLimit */
	bool_t bRigValueMonitorEnable;                             	/* PDI 8:  bRigValueMonitorEnable */
	bool_t bRigModeFaultMonitorEnable;                         	/* PDI 9:  bRigModeFaultMonitorEnable */
	bool_t bBacklashFreePlayMonitorEnable;                     	/* PDI 10:  bBacklashFreePlayMonitorEnable */
	uint16_t u16BacklashFreeplayMonitorQuadLimit;              	/* PDI 11:  u16BacklashFreeplayMonitorQuadLimit */
	bool_t bHallSensorSeqMonitorEnable;                        	/* PDI 12:  bHallSensorSeqMonitorEnable */
	uint16_t u16HallSensorSeqMonitorIllegalLimit;              	/* PDI 13:  u16HallSensorSeqMonitorIllegalLimit */
	bool_t bInvalidHallSensorMonitorEnable;                    	/* PDI 14:  bInvalidHallSensorMonitorEnable */
	bool_t bBrakeHoldMonitorEnable;                            	/* PDI 15:  bBrakeHoldMonitorEnable */
	bool_t bBrakeSwitchMonitorEnable;                          	/* PDI 16:  bBrakeSwitchMonitorEnable */
	uint16_t u16BrakeSwitchMonitorBrkOffMin;                   	/* PDI 17:  u16BrakeSwitchMonitorBrkOffMin */
	uint16_t u16BrakeSwitchMonitorBrkOffMax;                   	/* PDI 18:  u16BrakeSwitchMonitorBrkOffMax */
	uint16_t u16BrakeSwitchMonitorBrkUpperMin;                 	/* PDI 19:  u16BrakeSwitchMonitorBrkUpperMin */
	uint16_t u16BrakeSwitchMonitorBrkUpperMax;                 	/* PDI 20:  u16BrakeSwitchMonitorBrkUpperMax */
	uint16_t u16BrakeSwitchMonitorBrkLowerMax;                 	/* PDI 21:  u16BrakeSwitchMonitorBrkLowerMax */
	bool_t bFpsuExcMonitorEnable;                              	/* PDI 22:  bFpsuExcMonitorEnable */
	uint16_t u16FpsuExcMonitorSumIdeal;                        	/* PDI 23:  u16FpsuExcMonitorSumIdeal */
	uint16_t u16FpsuExcMonitorSumTolerance;                    	/* PDI 24:  u16FpsuExcMonitorSumTolerance */
	bool_t bFpsuPosMonitorEnable;                              	/* PDI 25:  bFpsuPosMonitorEnable */
	uint16_t u16FpsuPosMonitorDeltaThreshold;                  	/* PDI 26:  u16FpsuPosMonitorDeltaThreshold */
	bool_t bBridgeMonitorEnable;                               	/* PDI 27:  bBridgeMonitorEnable */
	uint16_t u16BridgeMonitorBridgeNoneMin;                    	/* PDI 28:  u16BridgeMonitorBridgeNoneMin */
	uint16_t u16BridgeMonitorBridgeNoneMax;                    	/* PDI 29:  u16BridgeMonitorBridgeNoneMax */
	uint16_t u16BridgeMonitorBridgeUpperMin;                   	/* PDI 30:  u16BridgeMonitorBridgeUpperMin */
	uint16_t u16BridgeMonitorBridgeLowerMax;                   	/* PDI 31:  u16BridgeMonitorBridgeLowerMax */
	bool_t bProgPinMonitorEnable;                              	/* PDI 32:  bProgPinMonitorEnable */
	bool_t bRiggingCheckMonitorEnable;                         	/* PDI 33:  bRiggingCheckMonitorEnable */
	uint16_t u16RiggingCheckMonitorQuadDiff;                   	/* PDI 34:  u16RiggingCheckMonitorQuadDiff */
	bool_t bROMCrcCheckMonitorEnable;                          	/* PDI 35:  bROMCrcCheckMonitorEnable */
	bool_t bRAMCheckMonitorEnable;                             	/* PDI 36:  bRAMCheckMonitorEnable */
	bool_t bWDTimerPowerUpMonitorEnable;                       	/* PDI 37:  bWDTimerPowerUpMonitorEnable */
	uint16_t u16WDTimerPowerUpMonitorMSec;                     	/* PDI 38:  u16WDTimerPowerUpMonitorMSec */
	bool_t bFrameOverrunMonitorEnable;                         	/* PDI 39:  bFrameOverrunMonitorEnable */
	bool_t bSubframeIndexMonitorEnable;                        	/* PDI 40:  bSubframeIndexMonitorEnable */
	bool_t bUndefinedINTMonitorEnable;                         	/* PDI 41:  bUndefinedINTMonitorEnable */
	bool_t bStackOverrunMonitorEnable;                         	/* PDI 42:  bStackOverrunMonitorEnable */
	bool_t bOverCurrentMonitorEnable;                          	/* PDI 43:  bOverCurrentMonitorEnable */
	bool_t bOverTempMonitorEnable;                             	/* PDI 44:  bOverTempMonitorEnable */
	bool_t bSensorFusionBiasCheckMonitorEnable;                	/* PDI 45:  bSensorFusionBiasCheckMonitorEnable */
	bool_t bSensorFusionMeasResidualMonitorEnable;             	/* PDI 46:  bSensorFusionMeasResidualMonitorEnable */
	uint16_t u16InvalidHallSensorMonitorCountLimit;            	/* PDI 47:  u16InvalidHallSensorMonitorCountLimit */
	bool_t bA825CntrlBusPassiveMonitorEnable;                  	/* PDI 48:  bA825CntrlBusPassiveMonitorEnable */
	bool_t bA825MaintBusPassiveMonitorEnable;                  	/* PDI 49:  bA825MaintBusPassiveMonitorEnable */
	bool_t bA825CntrlBusOffMonitorEnable;                      	/* PDI 50:  bA825CntrlBusOffMonitorEnable */
	bool_t bA825MaintBusOffMonitorEnable;                      	/* PDI 51:  bA825MaintBusOffMonitorEnable */
	bool_t bA825CntrlBusCommsStaleMonitorEnable;               	/* PDI 52:  bA825CntrlBusCommsStaleMonitorEnable */
	bool_t bA825MaintBusCommsStaleMonitorEnable;               	/* PDI 53:  bA825MaintBusCommsStaleMonitorEnable */
	bool_t bDCBusMonitorEnable;                                	/* PDI 54:  bDCBusMonitorEnable */
	uint16_t u16DCBusMonitorVCLL;                              	/* PDI 55:  u16DCBusMonitorVCLL */
	uint16_t u16DCBusMonitorVCUL;                              	/* PDI 56:  u16DCBusMonitorVCUL */
	uint16_t u16DCBusMonitorHLLL;                              	/* PDI 57:  u16DCBusMonitorHLLL */
	uint16_t u16DCBusMonitorHLUL;                              	/* PDI 58:  u16DCBusMonitorHLUL */
	bool_t bp28VDCMonitorEnable;                               	/* PDI 59:  bp28VDCMonitorEnable */
	uint16_t u16p28VDCMonitorLL;                               	/* PDI 60:  u16p28VDCMonitorLL */
	uint16_t u16p28VDCMonitorUL;                               	/* PDI 61:  u16p28VDCMonitorUL */
	bool_t bp15VDCMonitorEnable;                               	/* PDI 62:  bp15VDCMonitorEnable */
	uint16_t u16p15VDCMonitorLL;                               	/* PDI 63:  u16p15VDCMonitorLL */
	uint16_t u16p15VDCMonitorUL;                               	/* PDI 64:  u16p15VDCMonitorUL */
	bool_t bn15VDCMonitorEnable;                               	/* PDI 65:  bn15VDCMonitorEnable */
	uint16_t u16n15VDCMonitorLL;                               	/* PDI 66:  u16n15VDCMonitorLL */
	uint16_t u16n15VDCMonitorUL;                               	/* PDI 67:  u16n15VDCMonitorUL */
	bool_t bp5VDCMonitorEnable;                                	/* PDI 68:  bp5VDCMonitorEnable */
	uint16_t u16p5VDCMonitorLL;                                	/* PDI 69:  u16p5VDCMonitorLL */
	uint16_t u16p5VDCMonitorUL;                                	/* PDI 70:  u16p5VDCMonitorUL */
	bool_t bGateDriverMonitorEnable;                           	/* PDI 71:  bGateDriverMonitorEnable */
	bool_t bPDICrcCheckMonitorEnable;                          	/* PDI 72:  bPDICrcCheckMonitorEnable */
	bool_t bPDIRangeCheckMonitorEnable;                        	/* PDI 73:  bPDIRangeCheckMonitorEnable */
	bool_t bMRAMCheckMonitorEnable;                            	/* PDI 74:  bMRAMCheckMonitorEnable */
	bool_t bGFDMonitorEnable;                                  	/* PDI 75:  bGFDMonitorEnable */
	bool_t bA825MaintBusFlapCmdSkewMonitorEnable;              	/* PDI 76:  bA825MaintBusFlapCmdSkewMonitorEnable */
	float32_t f32A825MaintBusFlapSkewMaxDiffAllowed;           	/* PDI 77:  f32A825MaintBusFlapSkewMaxDiffAllowed */
	bool_t bPhNShortAsymMonitorEnable;                         	/* PDI 78:  bPhNShortAsymMonitorEnable */
	uint16_t u16PhNShortAsymHLUL;                              	/* PDI 79:  u16PhNShortAsymHLUL */
	uint16_t u16PhNShortAsymVCUL;                              	/* PDI 80:  u16PhNShortAsymVCUL */
	float32_t f32UnbalCurIndexHLUL;                            	/* PDI 81:  f32UnbalCurIndexHLUL */
	bool_t bOnePhOpenCktMonitorEnable;                         	/* PDI 82:  bOnePhOpenCktMonitorEnable */
	uint16_t u16OnePhOpenCktLL;                                	/* PDI 83:  u16OnePhOpenCktLL */
	float32_t f32UnbalCurIndexVCUL;                            	/* PDI 84:  f32UnbalCurIndexVCUL */
	bool_t bEncoderHealthMonitorEnable;                        	/* PDI 85:  bEncoderHealthMonitorEnable */
	bool_t bIncoderRxReadyChkEnable;                           	/* PDI 86:  bIncoderRxReadyChkEnable */
	uint16_t u16IncoderRxReadyChkThreshold;                    	/* PDI 87:  u16IncoderRxReadyChkThreshold */
	uint16_t u16IncoderRxReadyChkIncCount;                     	/* PDI 88:  u16IncoderRxReadyChkIncCount */
	uint16_t u16IncoderRxReadyChkDecCount;                     	/* PDI 89:  u16IncoderRxReadyChkDecCount */
	bool_t bIncoderPvInvalidChkEnable;                         	/* PDI 90:  bIncoderPvInvalidChkEnable */
	uint16_t u16IncoderPvInvalidChkThreshold;                  	/* PDI 91:  u16IncoderPvInvalidChkThreshold */
	uint16_t u16IncoderPvInvalidChkIncCount;                   	/* PDI 92:  u16IncoderPvInvalidChkIncCount */
	uint16_t u16IncoderPvInvalidChkDecCount;                   	/* PDI 93:  u16IncoderPvInvalidChkDecCount */
	bool_t bIncoderCrcChkEnable;                               	/* PDI 94:  bIncoderCrcChkEnable */
	uint16_t u16IncoderCrcChkThreshold;                        	/* PDI 95:  u16IncoderCrcChkThreshold */
	uint16_t u16IncoderCrcChkIncCount;                         	/* PDI 96:  u16IncoderCrcChkIncCount */
	uint16_t u16IncoderCrcChkDecCount;                         	/* PDI 97:  u16IncoderCrcChkDecCount */
} tPdi_t;

extern const tPdi_t tPdi;

#endif /* PDI_H_*/
