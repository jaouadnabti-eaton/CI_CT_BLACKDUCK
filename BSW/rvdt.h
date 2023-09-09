/****************************************************************************************************
*  File name: rvdt.h
*
*  Purpose: Provides the API for the rvdt module.
*
*  Copyright Notice:
*  All source code and data contained in this file is Proprietary and
*  Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
*  disclosed; in whole or in part, without the express written permission of
*  Eaton Aerospace.
*
*  Copyright (c) 2022 - Eaton Aerospace Group, All Rights Reserved.
*
*  Author            Date       CR#        Description
*  ------           ---------   ------     ---------------------------------------
*  Adam Bouwens     12/07/2022  N/A        Ported to MCU
****************************************************************************************************/
#if defined(__SKEW_SNSR_RVDT__)
#ifndef RVDT_H__
#define RVDT_H__

/*    Include Files */
#include "parameter.h"
#include "timer.h"
#include "gse.h"

/*      Public Type Definitions */
typedef struct
{
    float32_t f32Stroke;                    /* Actuator Stroke Position in inches from RVDT ADC Count */
    float32_t f32ScaledPosition;            /* Position scaled as a fractional value between 0 and 1 */
    float32_t f32ScaledPositionCalibrated;  /* RVDT P and S value average after RVDT calibration equation is applied. */
    int16_t s16AdcAvgRaw;                   /* RVDT P and S value average used to calculate Stroke */
    int16_t s16QuadCnt;                     /* Quad Count based on the Stroke calculated from RVDT count */
} tRvdtCalcsStroke_t;

typedef struct
{
    float32_t f32FlapAngle;                 /* Flap Angle Degrees Stream-wise from RVDT ADC Count */
    float32_t f32ScaledPosition;            /* Position scaled as a fractional value between 0 and 1 */
    float32_t f32ScaledPositionCalibrated;  /* RVDT P and S value average after RVDT calibration equation is applied. */
    int16_t s16AdcAvgRaw;                   /* RVDT P and S value average used to calculate Stroke */
    int16_t s16QuadCnt;                     /* Quad Count based on the Flap Angle calculated from RVDT count */
} tRvdtCalcsFlapAngle_t;

typedef struct
{
    tRvdtCalcsStroke_t tStroke;         /* Stroke Calculations */
    tRvdtCalcsFlapAngle_t tFlapAngle;   /* Flap Angle Calculations */
} tRvdtCalcs_t;

/* Rvdt Calibration Data  */
typedef struct
{
    float32_t f32a0;
    float32_t f32a1;
    float32_t f32a2;
    float32_t f32a3;
} tRvdtNvmCalibration_t;

/* Rvdt Position Data per rig position */
typedef struct
{
    uint16_t u16RvdtPos;
    uint16_t u16RvdtSum;
    uint16_t u16RvdtAvg;
} tRvdtNvmPosData_t;

/* RVDT Data to be stored as part of NVM */
typedef struct
{
    tRvdtNvmCalibration_t tRvdtCalibration;
    tRvdtNvmPosData_t     tRvdtPosData[NUM_RIG_POSITIONS];
} tRvdtNvmData_t;

/*      Public Defines */
#define RVDT_CNT2FRAC   ((float32_t)(1.0F/4096.0F))    /* Conversion to 0 to 1 for 12-bit ADC values. */

/* SPLINE INTERPOLATION 4th Order Polynomial defines used for
 * RVDT count to Flap Angle and RVDT count to Stroke Calculations */
#define NUM_POLYNOMIAL_COEFFS       4U
#define NUM_LOWER_UPPER_LIMITS      2U
#define NUM_POLYNOMIAL_EQUATIONS    16U

/*      Public Variable Declarations */
extern tRvdtCalcs_t tRvdtCalcs;

//extern eRvdtCalibrateRigCompleteStates_t tRvdtCalibrateRigCompleteState;
extern bool RVDTEnable;
extern bool FindDC_Offset;
extern Timer_t AsymOffset_Timer;

/*      Public ROM Constants */
extern const float32_t af32RvdtToStrokeCoeffsTable[NUM_ACTUATOR_TYPES][NUM_POLYNOMIAL_EQUATIONS][NUM_POLYNOMIAL_COEFFS];
extern const float32_t af32RvdtToFlapAngleCoeffsTable[NUM_ACTUATOR_TYPES][NUM_POLYNOMIAL_EQUATIONS][NUM_POLYNOMIAL_COEFFS];
extern const float32_t af32RvdtPolyBreaksTable[NUM_ACTUATOR_TYPES][NUM_POLYNOMIAL_EQUATIONS][NUM_LOWER_UPPER_LIMITS];

/*      Public Interface Function Prototypes */

/*
        defgroup rvdt RVDT Driver
  
        brief The RVDT driver is an abstraction for exciting and monitoring the flap position
        sensor device.  The driver has services for sending the excitation signal (sine-wave)
        to the device as well as for measuring the flap position signals from the device.

        Excitation Signal

        The RVDT excitation signal is a sine wave, generated by means of a 12-bit serial DAC, 
        through one of the McBSP channels.  The MCU software is responsible for sending serial 
        messages to the DAC at the proper timing to produce a sine wave of one of two frequencies:
 
            - Primary channel: 3043 Hz, use divider at 77 for 30 MHz peripheral clock
            - Secondary channel: 2967 Hz, use divider of 79 for 30 MHz peripheral clock
 
        The McBSP will be used in FIFO mode, filling the FIFO when it empties, from the McBSP
        FIFO Transmit interrupt.  The sine wave function is stored in a 32-element lookup 
        table, taking advantage of the waveform symmetry, for use in filling the FIFO.

        When the excitation signal needs to be stopped, the Rvdt_SinewaveSend() routine will 
        set a flag to cause the interrupt handler to insert Zeros into the FIFO instead of the 
        sine wave values.  To start the sinewave again, Rvdt_SinewaveSend() may be used to re-enable
        the sinewave output.

        The MSB_OUT signal is synchronized to the serial port output by toggling the value whenever the
        FIFO is empty at a zero-crossing point of the sine wave.
 
        Measuring Flap Position

        After the data have been captured, the positions of the flaps may be computed in a manner depicted
        the figure shown here.
        image html rvdt-positioncomp.png   "Computing flap positions"
        The "normalize" operation shown in the figure is used to minimize the time-varying character 
        of the position signal, and may be computed by the following formula, where SumRES is the Sum 
        value stored with the rigged RES position.
        
        code
            PositionOUT = PositionIN * ( SumRES/SumIN )
        endcode

*/

void Rvdt_Init( void );
void Rvdt_SinewaveSend( bool enable );
float32_t Rvdt_CalcStrokeFromRvdt(eActuator_t tActuatorIndex, float32_t f32RvdtAdcCount);
float32_t Rvdt_CalcFlapAngleFromRvdt(eActuator_t tActuatorIndex, float32_t f32RvdtAdcCount);
bool_t Rvdt_CalibrateSensor(void);
float32_t Rvdt_ApplyCalibrationEquation(float32_t f32PositionScaled0to1);
void Rvdt_GetScaledPositionFlapAngle(tRvdtCalcsFlapAngle_t* const ptRvdtCalcs);
void Rvdt_GetScaledPositionStroke(tRvdtCalcsStroke_t* const ptRvdtCalcs);
void Rvdt_SetNvmRigData(RIG_POSITIONS_T ePosition);
void Rvdt_SetNvmPanelData(RIG_POSITIONS_T ePosition);

#endif
#endif
/* end rvdt.h*/
