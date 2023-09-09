/****************************************************************************************************
*  File name: encoder.h
*
*Purpose: Provides functionality involving Encoder
*
*  Copyright Notice:
*  All source code and data contained in this file is Proprietary and
*  Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
*  disclosed; in whole or in part, without the express written permission of
*  Eaton Aerospace.
*
*  Copyright (c) 2023 - Eaton Aerospace Group, All Rights Reserved.
*
*  Author            Date       CR#          Description
*  ------           ---------   ------     ---------------------------------------
*  Aniket Mahadik   04/05/2023    N/A        Ported to MCU
*
****************************************************************************************************/
#if defined(__SKEW_SNSR_ENCODER__)
#ifndef ENCODER_H__
#define ENCODER_H__

/*      Include Files
*/
#include "parameter.h"
#include "typedefs.h"
#include "gse.h"

/*      Public Type Definitions */
typedef struct
{
    float32_t f32Stroke;                    /* Actuator Stroke Position in inches from RVDT ADC Count */
    uint32_t u32Position;                   /* Position with applied offset */
    uint32_t u32RawPosition;                /* Raw position read from sensor */
    float32_t f32ScaledPosition;            /* Position scaled as a fractional value between 0 and 1 */
    float32_t f32ScaledPositionCalibrated;  /* Position after calibration equation is applied. */
    int16_t s16QuadCnt;                     /* Quad Count based on the Stroke calculated from RVDT count */
    uint16_t u16MemoryPad;                  /* Required to pad memory so CRC works */
} tEncoderCalcsStroke_t;

typedef struct
{
    float32_t f32FlapAngle;                 /* Flap Angle Degrees Stream-wise from RVDT ADC Count */
    uint32_t u32Position;                   /* Position with applied offset */
    uint32_t u32RawPosition;                /* Raw position read from sensor */
    float32_t f32ScaledPosition;            /* Position scaled as a fractional value between 0 and 1 */
    float32_t f32ScaledPositionCalibrated;  /* Position after calibration equation is applied. */
    int16_t s16QuadCnt;                     /* Quad Count based on the Stroke calculated from RVDT count */
    uint16_t u16MemoryPad;                  /* Required to pad memory so CRC works */
} tEncoderCalcsFlapAngle_t;

typedef struct
{
    tEncoderCalcsStroke_t tStroke;         /* Stroke Calculations */
    tEncoderCalcsFlapAngle_t tFlapAngle;   /* Flap Angle Calculations */
} tEncoderCalcs_t;

/* Incoder Inputs Raw Data Bits */
typedef struct
{
    uint32_t PD         : 17U;      /* Position Data Bits b00-b16 of 22-bit field are used with this sensor */
    uint16_t PD_UNUSED  :  5U;      /* Position Data Bits b17-b21 of 22-bit field are unused with this sensor */
    uint16_t ZPD        :  1U;      /* bit 22:  Zero Point Default */
    uint16_t PV         :  1U;      /* bit 23:  Position Valid status bit */
    uint16_t CRC        :  8U;      /* bits 24 to 31:  CRC-8 */
} tIncoderInputsRawBits_t;

/* Incoder Inputs Raw Union */
typedef union
{
    uint32_t u32Data;
    t32u16_t tu16;
    t32u8_t  tu8;
    tIncoderInputsRawBits_t bits;
} tIncoderInputsRaw_t;

typedef struct
{
    uint32_t u32Position;               /* 17-bit Positional Data represented by an unsigned 32-bit */
    tIncoderInputsRaw_t tRaw;           /* IncOder Raw Data */
    tIncoderInputsRaw_t tData;          /* Correctly formatted data based on SPI only able to read 32-bits and unaligned data */
} tIncoderInputs_t;

/* Encoder Calibration Data */
typedef struct
{
    uint32_t u32Offset;
    uint32_t u32RawPositionAtStartOfRig;
} tEncoderNvmCalibration_t;

/* Encoder Position Data per rig position */
typedef struct
{
    uint32_t u32Position;       /* Position data only */
    uint32_t u32PositionData;   /* bit packed data */
} tEncoderNvmPosData_t;

/* Encoder Data to be stored as part of NVM */
typedef struct
{
    tEncoderNvmCalibration_t    tEncoderCalibration;
    tEncoderNvmPosData_t        tEncoderPosData[NUM_RIG_POSITIONS];
} tEncoderNvmData_t;

/*      Public Defines
*/
#define ENCODER_CNT2FRAC   ((float32_t)(1.0F/131072.0F))    /* Conversion to 0 to 1 for 17-bit Incoder values. */

/* SPLINE INTERPOLATION 4th Order Polynomial defines used for
 * IncOder count to Flap Angle and IncOder count to Stroke Calculations */
#define NUM_POLYNOMIAL_COEFFS       4U
#define NUM_LOWER_UPPER_LIMITS      2U
#define NUM_POLYNOMIAL_EQUATIONS    16U

/*      Public Variable Declarations
*/
extern uint16_t u16IncoderRxNotReadyCntr;
extern uint16_t u16IncoderPositionNotValidCntr;
extern uint16_t u16IncoderCrc8CheckFailedCntr;
extern tIncoderInputs_t tIncoderInputs;

/*      Public ROM Constants
*/
extern const uint32_t u32EncoderIdealCnt[NUM_ACTUATOR_TYPES][NUM_RIG_POSITIONS];
extern const float32_t af32EncoderToStrokeCoeffsTable[NUM_ACTUATOR_TYPES][NUM_POLYNOMIAL_EQUATIONS][NUM_POLYNOMIAL_COEFFS];
extern const float32_t af32EncoderToFlapAngleCoeffsTable[NUM_ACTUATOR_TYPES][NUM_POLYNOMIAL_EQUATIONS][NUM_POLYNOMIAL_COEFFS];
extern const float32_t af32EncoderPolyBreaksTable[NUM_ACTUATOR_TYPES][NUM_POLYNOMIAL_EQUATIONS][NUM_LOWER_UPPER_LIMITS];

/*      Public Interface Function Prototypes
*/
void Encoder_Init(void);
void Encoder_Tx(void);
void Encoder_Rx(void);
uint16_t Encoder_GetEncoderNewDataFlag(void);
bool_t Encoder_CalibrateSensor(void);
void Encoder_GetScaledPositionFlapAngle(tEncoderCalcsFlapAngle_t* const ptEncoderCalcs);
void Encoder_GetScaledPositionStroke(tEncoderCalcsStroke_t* const ptEncoderCalcs);
void Encoder_SetNvmRigData(RIG_POSITIONS_T ePosition);
void Encoder_SetNvmPanelData(RIG_POSITIONS_T ePosition);
uint32_t Encoder_ApplyOffset(uint32_t u32PositionRaw);

#endif
#endif
/* end encoder.h*/

