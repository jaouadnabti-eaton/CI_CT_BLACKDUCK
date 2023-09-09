/****************************************************************************************************
*  File name: skewsensor.c
*
*  Purpose: Provides functionality involving Encoder
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
*  Adam Bouwens     06/01/2023  N/A        Initial version
*
****************************************************************************************************/

/*      Include Files
*/
#include "skewsensor.h"
#include "panel.h"

/*      Local Type Definitions
*/
/* Select the Polynomial variables based on the skew sensor used. */
#if defined (__SKEW_SNSR_RVDT__)
#define SKEW_SNSR_GET_SCALED_POSITION_FLAP_ANGLE    Rvdt_GetScaledPositionFlapAngle
#define SKEW_SNSR_GET_SCALED_POSITION_STROKE        Rvdt_GetScaledPositionStroke
#define F32_POLY_COEFFS_STROKE_TABLE                af32RvdtToStrokeCoeffsTable
#define F32_POLY_COEFFS_FLAP_ANGLE_TABLE            af32RvdtToFlapAngleCoeffsTable
#define F32_POLY_BREAKS_TABLE                       af32RvdtPolyBreaksTable
#define SKEW_SNSR_TYPE                              (false)
#define SKEW_SNSR_POSITION_COUNT                    ((uint16_t)tSkewSnsrCalcs.tStroke.s16AdcAvgRaw)
#elif defined (__SKEW_SNSR_ENCODER__)
#define SKEW_SNSR_GET_SCALED_POSITION_FLAP_ANGLE    Encoder_GetScaledPositionFlapAngle
#define SKEW_SNSR_GET_SCALED_POSITION_STROKE        Encoder_GetScaledPositionStroke
#define F32_POLY_COEFFS_STROKE_TABLE                af32EncoderToStrokeCoeffsTable
#define F32_POLY_COEFFS_FLAP_ANGLE_TABLE            af32EncoderToFlapAngleCoeffsTable
#define F32_POLY_BREAKS_TABLE                       af32EncoderPolyBreaksTable
#define SKEW_SNSR_TYPE                              (true)
#define SKEW_SNSR_POSITION_COUNT                    ((uint16_t)(tSkewSnsrCalcs.tStroke.u32Position >> 1U))
#endif

/*      Local Defines
*/

/*      Global Variables
*/
SKEW_SNSR_CALCS_T tSkewSnsrCalcs = { 0U };

/*      Local ROM Constants
*/

/*      Local Variable Declarations
*/

/*      Externs
 */

/*      Local Function Prototypes
*/

/*      Function Definitions
*/
/****************************************************************************************************
*  Function: SkewSnsr_GetSensorType
*  Purpose:
*  This function will return the sensor type per the following values for the SKEW_SNSR_TYPE signal
*  as part of the Sensor Data Messages.
*  0 = RVDT, 1 = IncOder
*
*  Global Inputs:
*  Global Outputs:
*  Input:
*  Output: bool_t bSensorType
****************************************************************************************************/
bool_t SkewSnsr_GetSensorType(void)
{
    bool_t bSensorType = SKEW_SNSR_TYPE;

    return bSensorType;
}

/****************************************************************************************************
*  Function: SkewSnsr_GetPositionCount
*  Purpose:
*  This function will return the position count associated with the configured skew sensor.
*
*  Global Inputs:
*  Global Outputs:
*  Input:
*  Output: u16Position:  Skew Sensor Position in Counts
****************************************************************************************************/
uint16_t SkewSnsr_GetPositionCount(void)
{
    uint16_t u16Position = SKEW_SNSR_POSITION_COUNT;

    return u16Position;
}

/****************************************************************************************************
*  Function: SkewSnsr_GetFlapAngle
*  Purpose:
*  This function will calculate and save off the Flap Angle based on the position feedback from the
*  Encoder.
*
*  Global Inputs:
*  Global Outputs:
*  Input:
*  Output:
****************************************************************************************************/
void SkewSnsr_GetFlapAngle(void)
{
    /* Get the scaled position information from the configured skew sensor */
    SKEW_SNSR_GET_SCALED_POSITION_FLAP_ANGLE(&tSkewSnsrCalcs.tFlapAngle);

    /* Calculate the Flap Angle Based on Position count */
    tSkewSnsrCalcs.tFlapAngle.f32FlapAngle = SkewSnsr_CalcFlapAngle(G_eActuatorNumber, tSkewSnsrCalcs.tFlapAngle.f32ScaledPositionCalibrated);

    return;
}

/****************************************************************************************************
*  Function: SkewSnsr_GetStroke
*  Purpose:
*  This function will calculate and save off the Stroke based on the position feedback from the
*  Encoder.
*
*  Global Inputs:
*  Global Outputs:
*  Input:
*  Output:
****************************************************************************************************/
void SkewSnsr_GetStroke(void)
{
    /* Get the scaled position information from the configured skew sensor */
    SKEW_SNSR_GET_SCALED_POSITION_STROKE(&tSkewSnsrCalcs.tStroke);

    /* Calculate the Flap Angle Based on Position count */
    tSkewSnsrCalcs.tStroke.f32Stroke = SkewSnsr_CalcStroke(G_eActuatorNumber, tSkewSnsrCalcs.tStroke.f32ScaledPosition);

    /* Calculate the quad count associated with the stroke */
    tSkewSnsrCalcs.tStroke.s16QuadCnt = Panel_ConvertStrokeToElectricalCycleCount(tSkewSnsrCalcs.tStroke.f32Stroke);

    return;
}

/****************************************************************************************************
*  Function: SkewSnsr_CalcStroke
*  Purpose:
*  This function implements the spline polynomial interpolation to compute
*  Stroke in Inches from the skew sensor input
*
*  Equation:
*  F(q) = (a_3,i * (q-q_LLi)^3) + (a_2,i * (q-q_LLi)^2) + (a_1,i * (q-q_LLi)) + a_0,i
*  where,
*  q is the ADC counts
*  q_LLi is the lower limit for the ith polynomial
*  F(q) is the calculated Stroke in Inches
*  a_n,i are the polynomial coefficients for the nth degree term and the ith polynomial
*  The ith polynomial should be used when q is between the lower and upper limits.
*
*  Global Inputs:
*  Global Outputs:
*  Input: tActuatorIndex A1,A2,A3,A4, f32PositionScaled0to1 representing the current position
*  input from the skew sensor.
*  Output:
*
*  Assumptions:
*  1) Caller has normalized the skew sensor input to a fractional value between 0 and
*  1 representing the full range of the skew sensor.
*  2) Caller has setup the pointers to the coefficients and breaks for the skew sensor being used.
****************************************************************************************************/
float32_t SkewSnsr_CalcStroke(eActuator_t tActuatorIndex, float32_t f32PositionScaled0to1)
{
    float32_t f32Stroke = NAN;      /* Initialize to NAN */
    float32_t f32LowerLimit;        /* lower limit used in poly equation from lookup table */
    bool_t bFound = false;          /* Flag to determine if equation found to use */
    float32_t f32x, f32x2, f32x3;   /* Polynomial multiples */
    float32_t f32PositionScaled = f32PositionScaled0to1; /* Save local copy to work with */

    /* Pointer to the array of Polynomial Coefficients [a3, a2, a1, a0]
     * selected for spline polynomial interpolation equation */
    const float32_t *paf32PolyCoeffs = NULL;
    float32_t f32DiffFromLowerLimit;

    /* Polynomial Term Temp Variable used in each term calculation for spline
     * piecewise polynomial interpolation equation */
    float32_t f32PolyTermTemp;

    /* Calculate the Stroke only for Valid Actuator Number and valid rvdt counts */
    if((tActuatorIndex < NUM_ACTUATOR_TYPES) && ((f32PositionScaled >= 0.0) && (f32PositionScaled < 1.0)))
    {
        uint16_t i = 0;
        do
        {
            /* Use the position to find the spline polynomial to apply.
             * When the Position is less than the Upper Limit of the increasing
             * order 16 index lookup table, then select that indice's polynomial values
             * for the final calculation.
             * OR, if went thru all indices and Position is higher than Upper Limit
             * of 16th polynomial, therefore set to the 16th polynomial rather
             * than inducing an error. */
            if((f32PositionScaled < F32_POLY_BREAKS_TABLE[tActuatorIndex][i][1]) || (i == (NUM_POLYNOMIAL_EQUATIONS - 1U)))
            {
                bFound = true;
                /* Select the array of polynomial coefficients to use in the
                 * final Stroke Calculation */
                paf32PolyCoeffs = &F32_POLY_COEFFS_STROKE_TABLE[tActuatorIndex][i][0];

                /* Set the Lower Limit for the equation */
                f32LowerLimit = F32_POLY_BREAKS_TABLE[tActuatorIndex][i][0];
            }
            i++;
        } while ((bFound == false) && (i < NUM_POLYNOMIAL_EQUATIONS));

        /* Calculate the stroke if the equation values were found and the
         * pointer to the coefficient array is not NULL */
        if((bFound == true) && (paf32PolyCoeffs != NULL))
        {
            f32DiffFromLowerLimit = f32PositionScaled - f32LowerLimit;
            /* a0 term */
            f32Stroke = paf32PolyCoeffs[3];  /* Init to a0 Term */
            /* a1 term */
            f32x = f32DiffFromLowerLimit;   /* X    */
            f32PolyTermTemp = paf32PolyCoeffs[2] * f32x;
            f32Stroke += f32PolyTermTemp;  /* Sum in the a1 Term */
            /* a2 term */
            f32x2 = f32x * f32x;   /* X^2  */
            f32PolyTermTemp = paf32PolyCoeffs[1] * f32x2;
            f32Stroke += f32PolyTermTemp; /* Sum in the a2 Term */
            /* a3 term */
            f32x3 = f32x2 * f32x; /* X^3  */
            f32PolyTermTemp = paf32PolyCoeffs[0] * f32x3;
            f32Stroke += f32PolyTermTemp;  /* Sum in the a3 Term */
        }
    }

    return(f32Stroke);
}

/****************************************************************************************************
*  Function: SkewSnsr_CalcFlapAngle
*  Purpose:
*  This function implements the spline polynomial interpolation to compute
*  Flap Angle in degrees from the skew sensor input
*
*  Equation:
*  F(q) = (a_3,i * (q-q_LLi)^3) + (a_2,i * (q-q_LLi)^2) + (a_1,i * (q-q_LLi)) + a_0,i
*  where,
*  q is the ADC counts
*  q_LLi is the lower limit for the ith polynomial
*  F(q) is the calculated Stroke in Inches
*  a_n,i are the polynomial coefficients for the nth degree term and the ith polynomial
*  The ith polynomial should be used when q is between the lower and upper limits.
*
*  Global Inputs:
*  Global Outputs:
*  Input: tActuatorIndex A1,A2,A3,A4, f32PositionScaled0to1 representing the current position
*  input from the skew sensor.
*  Output:
*
*  Assumptions:
*  1) Caller has normalized the skew sensor input to a fractional value between 0 and
*  1 representing the full range of the skew sensor.
*  2) Caller has setup the pointers to the coefficients and breaks for the skew sensor being used.
****************************************************************************************************/
float32_t SkewSnsr_CalcFlapAngle(eActuator_t tActuatorIndex, float32_t f32PositionScaled0to1)
{
    float32_t f32FlapAngle = NAN;   /* Initialize to NAN */
    float32_t f32LowerLimit;        /* lower limit used in poly equation from lookup table */
    bool_t bFound = false;          /* Flag to determine if equation found to use */
    float32_t f32x, f32x2, f32x3;   /* Polynomial multiples */
    float32_t f32PositionScaled = f32PositionScaled0to1; /* Save local copy to work with */

    /* Pointer to the array of Polynomial Coefficients [a3, a2, a1, a0]
     * selected for spline polynomial interpolation equation */
    const float32_t *paf32PolyCoeffs = NULL;
    float32_t f32DiffFromLowerLimit;

    /* Polynomial Term Temp Variable used in each term calculation for spline
     * piecewise polynomial interpolation equation */
    float32_t f32PolyTermTemp;

    /* Calculate the Flap Angle only for Valid Actuator Number and valid rvdt counts */
    if((tActuatorIndex < NUM_ACTUATOR_TYPES) && ((f32PositionScaled >= 0.0) && (f32PositionScaled < 1.0)))
    {
        uint16_t i = 0;
        do
        {
            /* Use the position to find the spline polynomial to apply.
             * When the Position is less than the Upper Limit of the increasing
             * order 16 index lookup table, then select that indice's polynomial values
             * for the final calculation.
             * OR, if went thru all indices and Position is higher than Upper Limit
             * of 16th polynomial, therefore set to the 16th polynomial rather
             * than inducing an error. */
            if((f32PositionScaled < F32_POLY_BREAKS_TABLE[tActuatorIndex][i][1]) || (i == (NUM_POLYNOMIAL_EQUATIONS - 1U)))
            {
                bFound = true;
                /* Select the array of polynomial coefficients to use in the
                 * final Flap Angle Calculation */
                paf32PolyCoeffs = &F32_POLY_COEFFS_FLAP_ANGLE_TABLE[tActuatorIndex][i][0];

                /* Set the Lower Limit for the equation */
                f32LowerLimit = F32_POLY_BREAKS_TABLE[tActuatorIndex][i][0];
            }
            i++;
        } while ((bFound == false) && (i < NUM_POLYNOMIAL_EQUATIONS));

        /* Calculate the flap angle if the equation values were found and the
         * pointer to the coefficient array is not NULL */
        if((bFound == true) && (paf32PolyCoeffs != NULL))
        {
            f32DiffFromLowerLimit = f32PositionScaled - f32LowerLimit;
            /* a0 term */
            f32FlapAngle = paf32PolyCoeffs[3];  /* Init to a0 Term */
            /* a1 term */
            f32x = f32DiffFromLowerLimit;   /* X    */
            f32PolyTermTemp = paf32PolyCoeffs[2] * f32x;
            f32FlapAngle += f32PolyTermTemp;  /* Sum in the a1 Term */
            /* a2 term */
            f32x2 = f32x * f32x;   /* X^2  */
            f32PolyTermTemp = paf32PolyCoeffs[1] * f32x2;
            f32FlapAngle += f32PolyTermTemp; /* Sum in the a2 Term */
            /* a3 term */
            f32x3 = f32x2 * f32x; /* X^3  */
            f32PolyTermTemp = paf32PolyCoeffs[0] * f32x3;
            f32FlapAngle += f32PolyTermTemp;  /* Sum in the a3 Term */
        }
    }

    return(f32FlapAngle);
}

/* end skewsensor.c*/

