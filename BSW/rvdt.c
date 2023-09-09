/****************************************************************************************************
*  File name: rvdt.c
*
*  Purpose: Provides the functionality involving the RVDTs (FPSUs)
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
/*      Include Files */
#include "F2837xD_device.h"
#include "gpio.h"
#include "rvdt.h"
#include "adc.h"
#include "nvm.h"
#include "spi.h"
//#include "gse.h"
#include "bitmonitors.h"
#include "panel.h"
#include "icc.h"

/*      Local Type Definitions */

/*      Local Defines */
#define FIFO_CONTROL    0x1000
#define DAC_LOW         719
#define DAC_HIGH        3309

#define MAX_FLAP_ANGLE  45
#define MIN_FLAP_ANGLE  -5

#define RET_LOW                 (-1 << 4)
#define RET_HIGH                (1 << 4)
#define POS1_LOW                (9 << 4)
#define POS1_HIGH               (11 << 4)
#define POS2_LOW                (25 << 4)
#define POS2_HIGH               (27 << 4)
#define POS3_LOW                (25 << 4)
#define POS3_HIGH               (27 << 4)
#define FULL_LOW                (35 << 4)
#define FULL_HIGH               (37 << 4)

#define SRGR1_CLK_DIV_BY_2       1U

#define RVDT_CALIBRATION_POLY_NUM_COEFFS    (4U)    /* 3rd order polynomial least squares line fitting has 4 coefficients */
#define RVDT_CALIBRATION_NUM_SAMPLES    ((uint16_t)(ESL - N1))  /* 13 rig position samples used for RVDT Calibration */
#define RVDT_CALIBRATION_SIZE_3X3           (3U)

/*      Global Variables */
tRvdtCalcs_t tRvdtCalcs = { 0 };

/*      Local Types */
typedef enum
{
    RVDT_CAL_INIT = 0,
    RVDT_CAL_CALC_VANDERMONDE_MATRIX,
    RVDT_CAL_CALC_XT_Y,
    RVDT_CAL_CALC_XT_X,
    RVDT_CAL_CALC_INV_XT_X_DETERMINANTS,
    RVDT_CAL_CALC_INV_XT_X_ADJUGATE,
    RVDT_CAL_CALC_INV_XT_X,
    RVDT_CAL_CALC_COEFFICIENTS,
    RVDT_CAL_COMPLETE,
    RVDT_CAL_NUM_STATES = RVDT_CAL_COMPLETE
} eRvdtCalibrationState;

/*      Local ROM Constants */
/* Rig RVDT Ideal ADC Count Table */
const int16_t s16RvdtIdealAdcCnt[NUM_ACTUATOR_TYPES][NUM_RIG_POSITIONS] =
{
  /*{   RSL,    N1,  ZERO,    P1,    P2,    P3,    P4,    P5,    P6,    P7,    P8,    P9,   P10,   P11,   ESL } */
    {    82,   181,   367,   708,   819,  1182,  1417,  1751,  1815,  2313,  2708,  3130,  3232,  3721,  3824 },  /* Actuator 1 */
    {    82,   181,   367,   708,   819,  1182,  1417,  1751,  1815,  2313,  2708,  3130,  3232,  3721,  3824 },  /* Actuator 2 */
    {    60,   165,   461,   740,   860,  1347,  1602,  1959,  2027,  2313,  2667,  3026,  3393,  3737,  3816 },  /* Actuator 3 */
    {    60,   165,   461,   740,   860,  1347,  1602,  1959,  2027,  2313,  2667,  3026,  3393,  3737,  3815 },  /* Actuator 4 */
};

/* Table of Ideal RVDT Fraction (0 to 1.0) converted from corresponding ADC Values per Actuator Number and Rig Position */
const float32_t f32RvdtIdealVal0to1[NUM_ACTUATOR_TYPES][NUM_RIG_POSITIONS] =
{
  /*{                RSL,                   N1,                 ZERO,                   P1,                   P2,                   P3,                   P4,                   P5,                   P6,                   P7,                   P8,                   P9,                  P10,                  P11,                  ESL } */
    { (82.0F * RVDT_CNT2FRAC),  (181.0F * RVDT_CNT2FRAC),  (367.0F * RVDT_CNT2FRAC),  (708.0F * RVDT_CNT2FRAC),  (819.0F * RVDT_CNT2FRAC), (1182.0F * RVDT_CNT2FRAC), (1417.0F * RVDT_CNT2FRAC), (1751.0F * RVDT_CNT2FRAC), (1815.0F * RVDT_CNT2FRAC), (2313.0F * RVDT_CNT2FRAC), (2708.0F * RVDT_CNT2FRAC), (3130.0F * RVDT_CNT2FRAC), (3232.0F * RVDT_CNT2FRAC), (3721.0F * RVDT_CNT2FRAC), (3824.0F * RVDT_CNT2FRAC) },  /* Actuator 1 */
    { (82.0F * RVDT_CNT2FRAC),  (181.0F * RVDT_CNT2FRAC),  (367.0F * RVDT_CNT2FRAC),  (708.0F * RVDT_CNT2FRAC),  (819.0F * RVDT_CNT2FRAC), (1182.0F * RVDT_CNT2FRAC), (1417.0F * RVDT_CNT2FRAC), (1751.0F * RVDT_CNT2FRAC), (1815.0F * RVDT_CNT2FRAC), (2313.0F * RVDT_CNT2FRAC), (2708.0F * RVDT_CNT2FRAC), (3130.0F * RVDT_CNT2FRAC), (3232.0F * RVDT_CNT2FRAC), (3721.0F * RVDT_CNT2FRAC), (3824.0F * RVDT_CNT2FRAC) },  /* Actuator 2 */
    { (60.0F * RVDT_CNT2FRAC),  (165.0F * RVDT_CNT2FRAC),  (461.0F * RVDT_CNT2FRAC),  (740.0F * RVDT_CNT2FRAC),  (860.0F * RVDT_CNT2FRAC), (1347.0F * RVDT_CNT2FRAC), (1602.0F * RVDT_CNT2FRAC), (1959.0F * RVDT_CNT2FRAC), (2027.0F * RVDT_CNT2FRAC), (2313.0F * RVDT_CNT2FRAC), (2667.0F * RVDT_CNT2FRAC), (3026.0F * RVDT_CNT2FRAC), (3393.0F * RVDT_CNT2FRAC), (3737.0F * RVDT_CNT2FRAC), (3816.0F * RVDT_CNT2FRAC) },  /* Actuator 3 */
    { (60.0F * RVDT_CNT2FRAC),  (165.0F * RVDT_CNT2FRAC),  (461.0F * RVDT_CNT2FRAC),  (740.0F * RVDT_CNT2FRAC),  (860.0F * RVDT_CNT2FRAC), (1347.0F * RVDT_CNT2FRAC), (1602.0F * RVDT_CNT2FRAC), (1959.0F * RVDT_CNT2FRAC), (2027.0F * RVDT_CNT2FRAC), (2313.0F * RVDT_CNT2FRAC), (2667.0F * RVDT_CNT2FRAC), (3026.0F * RVDT_CNT2FRAC), (3393.0F * RVDT_CNT2FRAC), (3737.0F * RVDT_CNT2FRAC), (3815.0F * RVDT_CNT2FRAC) },  /* Actuator 4 */
};


/****************************************************************************************************
* RVDT TO STROKE Spline Interpolation Polynomial Equation Constants.
*
* Coefficients a3, a2, a1, a0 used in the polynomials based on
* actuators 1, 2, 3, 4 */
const float32_t af32RvdtToStrokeCoeffsTable[NUM_ACTUATOR_TYPES][NUM_POLYNOMIAL_EQUATIONS][NUM_POLYNOMIAL_COEFFS] =
{
    {   /* A1 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {     -3.54498E+00,    6.22373E+00,    4.91371E+00,   -5.15000E-01  },
        {     -3.54498E+00,    5.96128E+00,    5.21442E+00,   -3.90000E-01  },
        {     -4.40575E+00,    5.70314E+00,    5.49755E+00,   -2.59970E-01  },
        {     -5.24498E+00,    5.10520E+00,    5.98651E+00,    0.00000E+00  },
        {     -4.60582E+00,    3.79263E+00,    6.72875E+00,    5.31860E-01  },
        {     -6.02266E+00,    3.41997E+00,    6.92327E+00,    7.16000E-01  },
        {     -4.14327E+00,    1.81964E+00,    7.38736E+00,    1.35186E+00  },
        {     -5.08411E+00,    1.10470E+00,    7.55556E+00,    1.78200E+00  },
        {     -4.17809E+00,   -1.37643E-01,    7.63433E+00,    2.40200E+00  },
        {     -4.05364E+00,   -3.34679E-01,    7.62690E+00,    2.52196E+00  },
        {     -3.55141E+00,   -1.81223E+00,    7.36605E+00,    3.43642E+00  },
        {     -3.37878E+00,   -2.83951E+00,    6.91753E+00,    4.12662E+00  },
        {     -2.81943E+00,   -3.88539E+00,    6.22365E+00,    4.80644E+00  },
        {     -3.35078E+00,   -4.09607E+00,    6.02485E+00,    4.95900E+00  },
        {     -3.12485E+00,   -5.29531E+00,    4.90445E+00,    5.61378E+00  },
        {     -3.12485E+00,   -5.52961E+00,    4.63391E+00,    5.73300E+00  }
    },
    {   /* A2 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {     -3.54498E+00,    6.22373E+00,    4.91371E+00,   -5.15000E-01  },
        {     -3.54498E+00,    5.96128E+00,    5.21442E+00,   -3.90000E-01  },
        {     -4.40575E+00,    5.70314E+00,    5.49755E+00,   -2.59970E-01  },
        {     -5.24498E+00,    5.10520E+00,    5.98651E+00,    0.00000E+00  },
        {     -4.60582E+00,    3.79263E+00,    6.72875E+00,    5.31860E-01  },
        {     -6.02266E+00,    3.41997E+00,    6.92327E+00,    7.16000E-01  },
        {     -4.14327E+00,    1.81964E+00,    7.38736E+00,    1.35186E+00  },
        {     -5.08411E+00,    1.10470E+00,    7.55556E+00,    1.78200E+00  },
        {     -4.17809E+00,   -1.37643E-01,    7.63433E+00,    2.40200E+00  },
        {     -4.05364E+00,   -3.34679E-01,    7.62690E+00,    2.52196E+00  },
        {     -3.55141E+00,   -1.81223E+00,    7.36605E+00,    3.43642E+00  },
        {     -3.37878E+00,   -2.83951E+00,    6.91753E+00,    4.12662E+00  },
        {     -2.81943E+00,   -3.88539E+00,    6.22365E+00,    4.80644E+00  },
        {     -3.35078E+00,   -4.09607E+00,    6.02485E+00,    4.95900E+00  },
        {     -3.12485E+00,   -5.29531E+00,    4.90445E+00,    5.61378E+00  },
        {     -3.12485E+00,   -5.52961E+00,    4.63391E+00,    5.73300E+00  }
    },
    {   /* A3 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {     -6.41613E-01,    3.82263E+00,    3.30938E+00,   -5.96000E-01  },
        {     -6.41613E-01,    3.71546E+00,    3.72908E+00,   -4.00000E-01  },
        {     -1.36624E+00,    3.66617E+00,    3.91809E+00,   -3.02090E-01  },
        {     -1.89180E+00,    3.36968E+00,    4.42705E+00,    0.00000E+00  },
        {     -2.07964E+00,    2.98362E+00,    4.85922E+00,    3.16140E-01  },
        {     -2.39928E+00,    2.80032E+00,    5.02916E+00,    4.61430E-01  },
        {     -2.59160E+00,    1.94573E+00,    5.59265E+00,    1.09399E+00  },
        {     -2.57673E+00,    1.46144E+00,    5.80488E+00,    1.44928E+00  },
        {     -2.38793E+00,    7.87819E-01,    6.00088E+00,    1.96452E+00  },
        {     -2.53790E+00,    6.68033E-01,    6.02523E+00,    2.06507E+00  },
        {     -2.39369E+00,    1.37565E-01,    6.08136E+00,    2.48725E+00  },
        {     -2.29613E+00,   -4.83837E-01,    6.05139E+00,    3.01297E+00  },
        {     -2.16280E+00,   -1.08700E+00,    5.91385E+00,    3.53759E+00  },
        {     -2.18379E+00,   -1.66859E+00,    5.66685E+00,    4.05739E+00  },
        {     -1.71659E+00,   -2.21865E+00,    5.34047E+00,    4.52013E+00  },
        {     -1.71659E+00,   -2.31768E+00,    5.25323E+00,    4.62200E+00  }
    },
    {   /* A4 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {     -5.21128E-01,    3.84509E+00,    3.29271E+00,   -5.95840E-01  },
        {     -5.21128E-01,    3.75776E+00,    3.71741E+00,   -4.00000E-01  },
        {     -1.42289E+00,    3.71748E+00,    3.90998E+00,   -3.01750E-01  },
        {     -1.96471E+00,    3.40870E+00,    4.42547E+00,    0.00000E+00  },
        {     -2.10270E+00,    3.00776E+00,    4.86194E+00,    3.16190E-01  },
        {     -2.46212E+00,    2.82242E+00,    5.03323E+00,    4.61580E-01  },
        {     -2.64969E+00,    1.94546E+00,    5.59931E+00,    1.09483E+00  },
        {     -2.58312E+00,    1.45031E+00,    5.81083E+00,    1.45052E+00  },
        {     -2.61919E+00,    7.75013E-01,    6.00475E+00,    1.96619E+00  },
        {     -2.50123E+00,    6.43627E-01,    6.02847E+00,    2.06680E+00  },
        {     -2.40498E+00,    1.20822E-01,    6.08173E+00,    2.48910E+00  },
        {     -2.26081E+00,   -5.03512E-01,    6.04862E+00,    3.01472E+00  },
        {     -2.16531E+00,   -1.09740E+00,    5.90844E+00,    3.53897E+00  },
        {     -2.05940E+00,   -1.67966E+00,    5.65952E+00,    4.05820E+00  },
        {     -2.22946E+00,   -2.19839E+00,    5.33391E+00,    4.52032E+00  },
        {     -2.22946E+00,   -2.32693E+00,    5.24694E+00,    4.62200E+00  }
    }
};

/****************************************************************************************************
* RVDT TO FLAP ANGLE Spline Interpolation Polynomial Equation Constants.
*
* Coefficients a3, a2, a1, a0 used in the polynomials based on
* actuators 1, 2, 3, 4 */
const float32_t af32RvdtToFlapAngleCoeffsTable[NUM_ACTUATOR_TYPES][NUM_POLYNOMIAL_EQUATIONS][NUM_POLYNOMIAL_COEFFS] =
{
    {   /* A1 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {     -4.68008E+01,    4.21146E+01,    3.88974E+01,   -3.99950E+00  },
        {     -4.68008E+01,    3.86497E+01,    4.08905E+01,   -3.01463E+00  },
        {     -3.37832E+01,    3.52417E+01,    4.26841E+01,   -2.00000E+00  },
        {     -3.89506E+01,    3.06567E+01,    4.56653E+01,    0.00000E+00  },
        {     -3.64120E+01,    2.09092E+01,    4.99668E+01,    4.00000E+00  },
        {     -3.10413E+01,    1.79632E+01,    5.10152E+01,    5.36208E+00  },
        {     -2.92025E+01,    9.71489E+00,    5.34667E+01,    1.00000E+01  },
        {     -2.38682E+01,    4.67586E+00,    5.42944E+01,    1.31019E+01  },
        {     -3.87525E+01,   -1.15651E+00,    5.45811E+01,    1.75424E+01  },
        {     -1.65362E+01,   -2.98405E+00,    5.45160E+01,    1.84000E+01  },
        {     -3.58101E+01,   -9.01153E+00,    5.30585E+01,    2.49500E+01  },
        {     -5.53881E+00,   -1.93699E+01,    5.03220E+01,    2.99500E+01  },
        {     -7.98317E+01,   -2.10844E+01,    4.61478E+01,    3.49300E+01  },
        {     -2.50575E+01,   -2.70496E+01,    4.49490E+01,    3.60651E+01  },
        {     -3.63361E+01,   -3.60177E+01,    3.74250E+01,    4.10000E+01  },
        {     -3.63361E+01,   -3.87421E+01,    3.55565E+01,    4.19123E+01  }
    },
    {   /* A2 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {     -4.68008E+01,    4.21146E+01,    3.88974E+01,   -3.99950E+00  },
        {     -4.68008E+01,    3.86497E+01,    4.08905E+01,   -3.01463E+00  },
        {     -3.37832E+01,    3.52417E+01,    4.26841E+01,   -2.00000E+00  },
        {     -3.89506E+01,    3.06567E+01,    4.56653E+01,    0.00000E+00  },
        {     -3.64120E+01,    2.09092E+01,    4.99668E+01,    4.00000E+00  },
        {     -3.10413E+01,    1.79632E+01,    5.10152E+01,    5.36208E+00  },
        {     -2.92025E+01,    9.71489E+00,    5.34667E+01,    1.00000E+01  },
        {     -2.38682E+01,    4.67586E+00,    5.42944E+01,    1.31019E+01  },
        {     -3.87525E+01,   -1.15651E+00,    5.45811E+01,    1.75424E+01  },
        {     -1.65362E+01,   -2.98405E+00,    5.45160E+01,    1.84000E+01  },
        {     -3.58101E+01,   -9.01153E+00,    5.30585E+01,    2.49500E+01  },
        {     -5.53881E+00,   -1.93699E+01,    5.03220E+01,    2.99500E+01  },
        {     -7.98317E+01,   -2.10844E+01,    4.61478E+01,    3.49300E+01  },
        {     -2.50575E+01,   -2.70496E+01,    4.49490E+01,    3.60651E+01  },
        {     -3.63361E+01,   -3.60177E+01,    3.74250E+01,    4.10000E+01  },
        {     -3.63361E+01,   -3.87421E+01,    3.55565E+01,    4.19123E+01  }
    },
    {   /* A3 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {      7.80111E+01,    9.66362E+00,    3.65612E+01,   -6.07758E+00  },
        {      7.80111E+01,    2.26942E+01,    3.83629E+01,   -3.99849E+00  },
        {     -5.37159E+01,    2.86867E+01,    3.96785E+01,   -3.00000E+00  },
        {     -8.97069E+00,    1.70298E+01,    4.29855E+01,    0.00000E+00  },
        {     -1.87216E+01,    1.51991E+01,    4.51778E+01,    3.00000E+00  },
        {      2.90449E+00,    1.35490E+01,    4.60224E+01,    4.34000E+00  },
        {     -4.71088E+01,    1.45835E+01,    4.93625E+01,    1.00000E+01  },
        {     -3.92431E+01,    5.78025E+00,    5.06310E+01,    1.31200E+01  },
        {      5.89075E+02,   -4.47888E+00,    5.07444E+01,    1.75500E+01  },
        {     -1.21835E+02,    2.50709E+01,    5.10887E+01,    1.84000E+01  },
        {      2.05208E+01,   -3.94825E-01,    5.28080E+01,    2.20400E+01  },
        {     -2.40771E+01,    4.93238E+00,    5.32006E+01,    2.66200E+01  },
        {     -7.31586E+00,   -1.39233E+00,    5.35106E+01,    3.13000E+01  },
        {     -2.25852E+01,   -3.35962E+00,    5.30847E+01,    3.60800E+01  },
        {     -3.69248E+00,   -9.04847E+00,    5.20429E+01,    4.05000E+01  },
        {     -3.69248E+00,   -9.26150E+00,    5.16908E+01,    4.14975E+01  }
    },
    {   /* A4 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {      7.67912E+01,    9.83760E+00,    3.65397E+01,   -6.08973E+00  },
        {      7.67912E+01,    2.27064E+01,    3.83576E+01,   -4.00452E+00  },
        {     -5.34465E+01,    2.86411E+01,    3.96804E+01,   -3.00000E+00  },
        {     -9.05609E+00,    1.70426E+01,    4.29850E+01,    0.00000E+00  },
        {     -1.86638E+01,    1.51945E+01,    4.51779E+01,    3.00000E+00  },
        {      2.90252E+00,    1.35495E+01,    4.60224E+01,    4.34000E+00  },
        {     -4.71076E+01,    1.45833E+01,    4.93626E+01,    1.00000E+01  },
        {     -3.92432E+01,    5.78029E+00,    5.06310E+01,    1.31200E+01  },
        {      5.89071E+02,   -4.47887E+00,    5.07444E+01,    1.75500E+01  },
        {     -1.21831E+02,    2.50707E+01,    5.10887E+01,    1.84000E+01  },
        {      2.05122E+01,   -3.94306E-01,    5.28080E+01,    2.20400E+01  },
        {     -2.40468E+01,    4.93067E+00,    5.32006E+01,    2.66200E+01  },
        {     -7.42517E+00,   -1.38606E+00,    5.35109E+01,    3.13000E+01  },
        {     -2.21404E+01,   -3.38274E+00,    5.30835E+01,    3.60800E+01  },
        {     -8.81040E+00,   -8.95954E+00,    5.20472E+01,    4.05000E+01  },
        {     -8.81040E+00,   -9.46750E+00,    5.16931E+01,    4.14969E+01  }
    }
};

/* Lower and Upper Limits use for selecting the Coefficients to use for both Flap Angle and Stroke equations */
const float32_t af32RvdtPolyBreaksTable[NUM_ACTUATOR_TYPES][NUM_POLYNOMIAL_EQUATIONS][NUM_LOWER_UPPER_LIMITS] =
{
    {   /* A1 Lower and Upper Limits */
        /*    Lower,     Upper  */
        {  -0.00454,   0.02013  },
        {   0.02013,   0.04441  },
        {   0.04441,   0.08965  },
        {   0.08965,   0.17306  },
        {   0.17306,   0.20003  },
        {   0.20003,   0.28861  },
        {   0.28861,   0.34613  },
        {   0.34613,   0.42758  },
        {   0.42758,   0.44330  },
        {   0.44330,   0.56480  },
        {   0.56480,   0.66122  },
        {   0.66122,   0.76440  },
        {   0.76440,   0.78931  },
        {   0.78931,   0.90861  },
        {   0.90861,   0.93360  },
        {   0.93360,   0.96152  }
    },
    {   /* A2 Lower and Upper Limits */
        /*    Lower,     Upper  */
        {  -0.00454,   0.02013  },
        {   0.02013,   0.04441  },
        {   0.04441,   0.08965  },
        {   0.08965,   0.17306  },
        {   0.17306,   0.20003  },
        {   0.20003,   0.28861  },
        {   0.28861,   0.34613  },
        {   0.34613,   0.42758  },
        {   0.42758,   0.44330  },
        {   0.44330,   0.56480  },
        {   0.56480,   0.66122  },
        {   0.66122,   0.76440  },
        {   0.76440,   0.78931  },
        {   0.78931,   0.90861  },
        {   0.90861,   0.93360  },
        {   0.93360,   0.96152  }
    },
    {   /* A3 Lower and Upper Limits */
        /*    Lower,     Upper  */
        {  -0.04086,   0.01482  },
        {   0.01482,   0.04042  },
        {   0.04042,   0.11276  },
        {   0.11276,   0.18078  },
        {   0.18078,   0.21017  },
        {   0.21017,   0.32889  },
        {   0.32889,   0.39118  },
        {   0.39118,   0.47833  },
        {   0.47833,   0.49505  },
        {   0.49505,   0.56472  },
        {   0.56472,   0.65125  },
        {   0.65125,   0.73881  },
        {   0.73881,   0.82845  },
        {   0.82845,   0.91241  },
        {   0.91241,   0.93164  },
        {   0.93164,   0.96312  }
    },
    {   /* A4 Lower and Upper Limits */
        /*    Lower,     Upper  */
        {  -0.04120,   0.01466  },
        {   0.01466,   0.04042  },
        {   0.04042,   0.11276  },
        {   0.11276,   0.18078  },
        {   0.18078,   0.21017  },
        {   0.21017,   0.32889  },
        {   0.32889,   0.39118  },
        {   0.39118,   0.47833  },
        {   0.47833,   0.49505  },
        {   0.49505,   0.56472  },
        {   0.56472,   0.65125  },
        {   0.65125,   0.73881  },
        {   0.73881,   0.82845  },
        {   0.82845,   0.91241  },
        {   0.91241,   0.93163  },
        {   0.93163,   0.96869  }
    }
};

/* Half Sine Wave Definition for RVDT Excitation Voltage */
const uint16_t u16RvdtValues[32] = {
    0x14a4,
    0x161f,
    0x178a,
    0x18e3,
    0x1a25,
    0x1b4f,
    0x1c5d,
    0x1d4c,
    0x1e1b,
    0x1ec7,
    0x1f4e,
    0x1fb0,
    0x1feb,
    0x1fff,
    0x1feb,
    0x1fb0,
    0x1f4e,
    0x1ec7,
    0x1e1b,
    0x1d4c,
    0x1c5d,
    0x1b4f,
    0x1a25,
    0x18e3,
    0x178a,
    0x161f,
    0x14a4,
    0x131e,
    0x1191,
    0x1000,
    0x1191,
    0x131e
};

/*      Local Variable Declarations */
bool RVDTEnable = false;     /* global flag for enabling or disabling the FPSU excitation signal*/

/*      Local Function Prototypes */
__interrupt void mcbspTxFifo_ISR(void);
int16 ConvertFPSU_AngleToFlapAngle(int32 fpsu);
void clkg_delay_loop(void); // Delay function used for CLKG initialization

/*      Function Definitions */

/*
    brief Configures serial port peripheral and interrupt for RVDT driver.

     Purpose:
        This routine is used to initialize the McBSP (Multichannel Buffered Serial Port) to send 
        serial data to produce a fullwave rectified sine wave using a DAC. It is also used to calculate 
        normalized position values to be used in flap position calculations.

    Global Data Referenced:
        #McbspbRegs
        #GpioMuxRegs 
        #GpioDataRegs
        #PieVectTable 
        #PieCtrlRegs
        #IER 
		#NormalizedP_Pos 
		#NormalizedS_Pos_In 
		#NormalizedS_Pos 
		#NormalizedP_Pos_In 
      
    return  void
    
    Preconditions and Assumptions:
        This function should be called only once, prior to scheduler loop.

*/
void Rvdt_Init( void )
{
    /* PATH(Rvdt_Init,A);*/

    /* shall place McBSP/Transmitter in reset */
    McbspbRegs.SPCR2.bit.FRST = 0;              /* Frame-sync logic*/
    McbspbRegs.SPCR2.bit.GRST = 0;              /* Sample-rate generator*/
    McbspbRegs.SPCR2.bit.XRST = 0;              /* Serial port transmitter*/

    /* shall set transmitter pins to operate as McBSP pins*/
    EALLOW;
#if !defined(DRV8312_DEV_KIT)
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = GPIO_MUX_TYPE_3;    /* set GPIO24 to MDXB */
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = GPIO_MUX_TYPE_3;    /* set GPIO26 to MCLKXB */
    GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = GPIO_ASYNC;        /* set GPIO26 to MCLKXB */
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = GPIO_MUX_TYPE_3;    /* set GPIO27 to MFXSB */
    GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 3;   /* set GPIO27 to MFXSB */
#endif
    EDIS;

    /* shall initialize MSB and clear it*/
    EALLOW;
    /* Setup GPIO_16 as MSB_OUT output */
    GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPADIR.bit.GPIO16 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = GPIO_DISABLE_PULLUP;   /* Disable the pullups */
    GpioCtrlRegs.GPAODR.bit.GPIO16 = GPIO_NORMAL_OUTPUT;    /* Configure as push-pull normal output */
    EDIS;
    MSB_OUT_clr();

    /* shall disable digital loopback mode*/
    McbspbRegs.SPCR1.bit.DLB = 0;

    /* shall enable clock stop mode*/
    McbspbRegs.SPCR1.bit.CLKSTP = 3; /* delay*/

    /* shall disable multi-channel transmit*/
    McbspbRegs.MCR2.bit.XMCM = 0;

    /* shall choose single phase for transmit frame*/
    McbspbRegs.XCR2.bit.XPHASE = 0;

    /* shall set transmit word length*/
    McbspbRegs.XCR1.bit.XWDLEN1 = 2; /* 16 bits*/

    /* shall set transmit frame length*/
    McbspbRegs.XCR1.bit.XFRLEN1 = 0; /* 1 word per frame*/

    /* shall disable TX frame-sync ignore function*/
    McbspbRegs.XCR2.bit.XFIG = 0;

    /* shell set transmit companding mode*/
    McbspbRegs.XCR2.bit.XCOMPAND = 0; /* None*/

    /* shall set transmit data delay*/
    McbspbRegs.XCR2.bit.XDATDLY = 1; /* 1-bit delay*/

    /* shall set transmit DXENA mode*/
    McbspbRegs.SPCR1.bit.DXENA = 0; /* No delay*/

    /* shall set transmit interrupt mode*/
    McbspbRegs.SPCR2.bit.XINTM = 0; /* Driven by XINT*/

    /* shell set transmit frame-sync mode*/
    McbspbRegs.PCR.bit.FSXM = 1;
    McbspbRegs.SRGR2.bit.FSGM = 1;

    /* shall set transmit frame-sync polarity*/
    McbspbRegs.PCR.bit.FSXP = 1; /* Active Low*/

	/* shall set SRG frame-sync period */
    /* Split the difference between desired frequencies for Left and Right Channels since the channel hasn't been established yet.
     * Call to reset the frequency will occur later after the channel has been set up.
     */
    McbspbRegs.SRGR2.bit.FPER = 252; /* 3000.5 Hz +/- 7.5 (((LSPCLK/CLKGDV)/3005.5)/64) - 1 = (((100MHz / 2) / 3005.5Hz) / 64) - 1 = 259 */

    /* shall set pulse width */
    McbspbRegs.SRGR1.bit.FWID = 0; /* Pulse width = 1*/

    /* shall set the transmit clock mode*/
    McbspbRegs.PCR.bit.CLKXM = 1;

    /* shall set the transmit clock polarity*/
    McbspbRegs.PCR.bit.CLKXP = 1; /* Falling Edge*/

    /* shall set the SRG clock divide-down value*/
    /* Clock is derived from LSPCLK which is set at 100MHz. IC supports max of 50MHz. Divide by 2 to get 50MHz */
    McbspbRegs.SRGR1.bit.CLKGDV = SRGR1_CLK_DIV_BY_2;   /* Set SRGR1 Clock Divide Down Register to Divide by 2 */

    /* shall set the SRG clock synchronization mode*/
    McbspbRegs.SRGR2.bit.GSYNC = 0;

    /* shall set the SRG clock mode*/
    McbspbRegs.PCR.bit.SCLKME = 0;      /* Internal clock*/
    McbspbRegs.SRGR2.bit.CLKSM = 1;     /* Internal clock*/

    /* shall enable transmit interrupt */
    McbspbRegs.MFFINT.bit.XINT = 1;

    /* shall take transmitter out of reset*/
    McbspbRegs.SPCR2.bit.GRST = 1;              /* Sample-rate generator*/
    clkg_delay_loop();
    McbspbRegs.SPCR2.bit.XRST = 1;              /* Serial port transmitter*/
    McbspbRegs.SPCR2.bit.FRST = 1;              /* Frame-sync logic*/

    /* shall initialize McBSP interrupt*/
    EALLOW;
    PieVectTable.MCBSPB_TX_INT = &mcbspTxFifo_ISR;
    EDIS;

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block

    PieCtrlRegs.PIEIER6.bit.INTx8 = 1;  /* PIE Group 6, INT8  McBSP Tx*/
    IER |= M_INT6;                      /* Enable CPU INT 6*/
    EINT;

    /* PATH(Rvdt_Init,D);*/
}

/*
    brief Starts or stops the Sinewave output to the DAC.

    Purpose:
        Starts or stops the Sinewave Output to the DAC by setting a flag
        to signal the ISR to send sine values or Zeros, depending on the flag setting.

     Global Data Referenced:
        #RVDTEnable
    
    return  void
    
    Preconditions and Assumptions:
    	none
*/
void Rvdt_SinewaveSend( bool enable )
{
    /* PATH(Rvdt_SinewaveSend,A);*/

    /* shall set the global flag RVDTEnable according to the passed in value*/
    RVDTEnable = enable;

    /* PATH(Rvdt_SinewaveSend,B);*/
}

/*
    brief Interrupt handler for FIFO Tx operation of McBSP

    Purpose:
        Interrupt handler for McBSP FIFO, called when FIFO status register (# of words
        remaining in FIFO) equals the FIFO interrupt level register (programmable, default 0).
       
        Sends next steps of excitation signal (sine wave) to the serial port FIFO,
        so that the port's FIFO does not underflow.
        
        The RVDT driver uses the FIFO interrupt to synchronize the MSB_OUT signal with the 
        half-sine wave output messages.  Note that the serial output words begin 2 steps
        away from the zero-crossing--this is to compensate for the lag in toggling the MSB_OUT
        signal.

    Global Data Referenced:
        #RVDTEnable
        #McbspbRegs
        #GPIO_DATA_REGS     
        #PieCtrlRegs
        #MSB_OUT_tgl
    
    return  void
    
    Preconditions and Assumptions:
         None.

*/
__interrupt void mcbspTxFifo_ISR( void )
{
    static uint16_t i = 0U;

    /* PATH(mcbspTxFifo_ISR,A); */

    /* shall send all zeros to the DAC if RVDTEnable is set to false*/
    if (RVDTEnable == false)
    {
        /* PATH(mcbspTxFifo_ISR,B); */
        McbspbRegs.DXR1.all = FIFO_CONTROL | 0;
    }
    /* shall generate the half-sine wave excitation signal if RVDTEnable is set to true*/
    else
    {
        McbspbRegs.DXR1.all = u16RvdtValues[i];
        /* Update next point of half sine wave */
        if(i < 32)
        {
            i++;
        }
        else
        {
            i = 0;
            MSB_OUT_tgl();
        }
    }

    PieCtrlRegs.PIEACK.bit.ACK6 = 1; /* Issue PIE ack*/

    /* PATH(mcbspTxFifo_ISR,F); */
}

/****************************************************************************************************
*  Function: Rvdt_CalibrateSensor
*  Purpose:
*  This function will calculate a 3rd order polynomial line of best fit on the RVDT data captured
*  during the rigging process. The coefficients a3, a2, a1, and a0 are calculated from this function.
*  The 3rd Order Polynomial:
*               y = ((a3 * x^3) + (a2 * x^2) + (a1 * x) + a0) where,
*               X is the RVDT ADC input converted to a fractional representation between 0 and 1, and
*               Y is the calibrated RVDT fractional representation between 0 and 1.
*               k is 3 the order of the polynomial
*  The Polynomial will be used on all normal mode RVDT to Stroke transfer functions to improve
*  accuracy of the sensor.
*
*   Yi = RVDT Ideal Values
*   Xi = Actual RVDT Ideal Values
*   n = number of samples used in polynomial calculation
*
*  3rd Order Polynomial Coefficient Equation:
*   [ A ] = [ X ] ^-1 * [ Y ], where
*      X is a 4 x 4 Matrix:
*      X = [  n             Sum(Xi)      Sum(Xi^2)    Sum(Xi^3)
*             Sum(Xi)       Sum(Xi^2)    Sum(Xi^3)    Sum(Xi^4)
*             Sum(Xi^2)     Sum(Xi^3)    Sum(Xi^4)    Sum(Xi^5)
*             Sum(Xi^3)     Sum(Xi^4)    Sum(Xi^5)    Sum(Xi^6)   ]
*      Y is a 4 x 1 Matrix:
*      Y = [  Sum(Yi)
*             Sum(Xi * Yi)
*             Sum(Xi^2 * Yi)
*             Sum(Xi^3 * Yi)
*      A is the resulting Coefficients in a 4 x 1 Matrix:
*      A = [  a0
*             a1
*             a2
*             a3 ]
*
*  The above is a Vandermonde Matrix and can be expressed by the following:
*  [ Y ] = [ X ] * [ A ], where
*
*  X = [    1       x1      ...     x1^k
*           1       x2      ...     x2^k
*           :       :       :       :
*           :       :       :       :
*           1       xn      ...     xn^k    ]  , and
*
*  Y = [    y1
*           y2
*           :
*           :
*           yn  ] , and
*
*  A = [    a0
*           a1
*           :
*           :
*           ak  ]
*
*  Notice that the highest exponent order is now k instead of 2 times k as before.
*
*  Premultiply both sides by the Transpose of X gives:
*
*  [ XT ] * [ Y ] = [ XT ] * [ X ] * [ A ]
*
*  Solving for A:
*
*  A = ([ XT ] * [ X ])^-1 * [XT] * [Y]
*
*  Notice that this solution requires the calculation of an inverse of the 4x4 matrix XT * X
*  The inverse of a 4x4 matrix is calculated by the following steps:
*  1) Calculate the Matrix of Minors
*  2) Calculate the Matrix of Cofactors
*  3) Calculate the Adjugate
*  4) Multiply by 1/Determinant
*
*  Global Inputs:
*  Global Outputs:
*  Input: none
*  Output: false = incomplete, true = calibration complete
****************************************************************************************************/
bool_t Rvdt_CalibrateSensor(void)
{
    bool_t bRet = false;

    static eRvdtCalibrationState tState = RVDT_CAL_INIT;
    static uint16_t i, j = RSL;
    uint16_t k = 0U;
    float32_t f32Xi = 0.0F;
    float32_t f32Yi = 0.0F;
    float32_t f32Temp = 0.0F;
    static float32_t f32Sum = 0.0F;
    float32_t f32mM[3U][3U];    /* Matrix of Minors for calculating determinants */
    static uint16_t iRow, iCol;
    uint16_t jRow, jCol;
    uint16_t kRow, kCol;
    float64_t f64Det1, f64Det2, f64Det3;
    float32_t f32DetXT_X_inv;
    float32_t f32a, f32b;
    float32_t f32Mult;
    float64_t f64a, f64b, f64c, f64d;

    /* Structure containing the Matrix used in the polynomial calculations. */
    static struct
    {
        float32_t f32mX[RVDT_CALIBRATION_NUM_SAMPLES][RVDT_CALIBRATION_POLY_NUM_COEFFS];   /* Matrix X (13x4):  RVDT Sample Values at the Rig Positions (rig samples N1 thru P11 only) */
        float32_t f32mXT[RVDT_CALIBRATION_POLY_NUM_COEFFS][RVDT_CALIBRATION_NUM_SAMPLES];  /* Matrix X Transposed (4x13):  The transposed X matrix */
        float32_t f32mY[RVDT_CALIBRATION_NUM_SAMPLES];          /* Matrix Y (13x1):  Ideal RVDT Matrix per rig position (samples N1 thru P11 only)  */
        float32_t f32mXT_Y[RVDT_CALIBRATION_POLY_NUM_COEFFS];   /* Matrix X Transformed (XT) multiplied by Matrix Y (4x1) */
        float32_t f32mXT_X[RVDT_CALIBRATION_POLY_NUM_COEFFS][RVDT_CALIBRATION_POLY_NUM_COEFFS]; /* Matrix X Transformed (XT) multiplied by Matrix X (4x4) */
        float32_t f32mA[RVDT_CALIBRATION_POLY_NUM_COEFFS];   /* Matrix A (4x1): The Polynomial Coefficient Matrix */
        float32_t f32Det[RVDT_CALIBRATION_POLY_NUM_COEFFS][RVDT_CALIBRATION_POLY_NUM_COEFFS];   /* Matrix of Minors (4x4): 3x3 Matrix of Minors Determinants Calculations */
        float32_t f32Adj[RVDT_CALIBRATION_POLY_NUM_COEFFS][RVDT_CALIBRATION_POLY_NUM_COEFFS];   /* Adjugate Matrix (4x4):  Adjugate of the Matrix of Minors */
        float32_t f32mXT_X_inv[RVDT_CALIBRATION_POLY_NUM_COEFFS][RVDT_CALIBRATION_POLY_NUM_COEFFS]; /* Matrix X Inverse (4x4):  Calculated inverse of matrix XT_X */
    } tPolyMatrixCalcs = {
                    .f32mX = { 0.0F },
                    .f32mXT = { 0.0F },
                    .f32mY = { 0.0F },
                    .f32mXT_Y = { 0.0F },
                    .f32mXT_X = { 0.0F },
                    .f32mA = { 0.0F },
                    .f32Det = { 0.0F },
                    .f32Adj = { 0.0F },
                    .f32mXT_X_inv = { 0.0F },
    };


    switch (tState)
    {
        case RVDT_CAL_INIT:
        {
            /* Initialize the variables used in the calculation. Must do for more than one channel */
            i = N1;
            /* Go to next state to calculate the means */
            tState = RVDT_CAL_CALC_VANDERMONDE_MATRIX;
            break;
        }
        case RVDT_CAL_CALC_VANDERMONDE_MATRIX:
        {
            /* Loop thru all rig position data (sample points) and build the Vandermonde Matrix [X] */
            if((i >= N1) && (i < ESL))
            {
                /* Set the Actual Value in fractional form. */
                f32Xi = (RVDT_CNT2FRAC * (float32_t)Nvm_Rigging_Temp.tSkewSnsr.tRvdtPosData[i].u16RvdtAvg);
                /* Set the Ideal Value in fractional form. */
                f32Yi = (float32_t)f32RvdtIdealVal0to1[G_eActuatorNumber][i];

                /* Build the X Matrix:  Vandermonde Matrix */
                j = i - N1; /* Set index for the sample matrix. Don't include rig samples that are not used in the calculations */
                tPolyMatrixCalcs.f32mX[j][0] = 1.0F;    /* 1 */
                f32Temp = f32Xi;
                tPolyMatrixCalcs.f32mX[j][1] = f32Temp;   /* Xi */
                f32Temp *= f32Xi;
                tPolyMatrixCalcs.f32mX[j][2] = f32Temp;   /* Xi^2 */
                f32Temp *= f32Xi;
                tPolyMatrixCalcs.f32mX[j][3] = f32Temp;   /* Xi^3 */

                /* Build the Y matrix loaded with the ideal RVDT values to map to */
                tPolyMatrixCalcs.f32mY[j] = f32Yi;
                /* Increment rig position index */
                i++;
            }
            else
            {
                /* Build the Transpose Y Matrix */
                for(iRow = 0U; iRow < RVDT_CALIBRATION_NUM_SAMPLES; iRow++)
                {
                    for(iCol = 0U; iCol < RVDT_CALIBRATION_POLY_NUM_COEFFS; iCol++)
                    {
                        /* Transpose */
                        tPolyMatrixCalcs.f32mXT[iCol][iRow] = tPolyMatrixCalcs.f32mX[iRow][iCol];
                    }
                }

                /* Init Indices for use in calculations */
                i = 0U;
                j = 0U;
                f32Sum = 0.0F;

                /* Go to next state to multiply both sides of the equation by the transpose of X. */
                tState = RVDT_CAL_CALC_XT_Y;
            }
            break;
        }
        case RVDT_CAL_CALC_XT_Y:
        {
            /* Calculate XT * Y Matrix */
            if(i < RVDT_CALIBRATION_POLY_NUM_COEFFS)
            {
                for(k = 0U; k < RVDT_CALIBRATION_NUM_SAMPLES; k++)
                {
                    f32a = tPolyMatrixCalcs.f32mXT[i][k];
                    f32b = tPolyMatrixCalcs.f32mY[k];
                    f32Mult = (f32a * f32b);
                    f32Sum += f32Mult;
                }

                /* Build the XT * Y Matrix */
                tPolyMatrixCalcs.f32mXT_Y[i] = f32Sum;
                f32Sum = 0U;    /* Reset Sum for next calc */
                i++;            /* Increment Row index for Matrix A [XT] */
            }
            else
            {
                /* Init Indices for use in calculations */
                i = 0U;
                j = 0U;
                f32Sum = 0.0F;

                /* Go to next state to compute XT * X Matrix */
                tState = RVDT_CAL_CALC_XT_X;
            }
            break;
        }
        case RVDT_CAL_CALC_XT_X:
        {
            if(i < RVDT_CALIBRATION_POLY_NUM_COEFFS)
            {
                if(j < RVDT_CALIBRATION_POLY_NUM_COEFFS)
                {
                    /* Calculate and sum the products of the row and column */
                    for(k = 0U; k < RVDT_CALIBRATION_NUM_SAMPLES; k++)
                    {
                        f32a = tPolyMatrixCalcs.f32mXT[i][k];
                        f32b = tPolyMatrixCalcs.f32mX[k][j];
                        f32Mult = (f32a * f32b);
                        f32Sum += f32Mult;
                    }
                    /* Build the XT * Y Matrix */
                    tPolyMatrixCalcs.f32mXT_X[i][j] = f32Sum;
                    f32Sum = 0.0F;    /* Reset Sum for next calc */

                    j++;    /* Increment Column index for Product Matrix [XT] */
                    /* Test if all columns have been processed */
                    if(j >= RVDT_CALIBRATION_POLY_NUM_COEFFS)
                    {
                        j = 0U; /* Reset Column index for next row to be processed */
                        i++;    /* Increment Row Index for Product Matrix [XT] */
                    }
                }
            }
            else
            {
                /* Go to next state to build the matrix of minors and calculate the determinants */
                iRow = 0U;
                iCol = 0U;

                /* Go to next state to build matrix of minors and calculate all the determinants. */
                tState = RVDT_CAL_CALC_INV_XT_X_DETERMINANTS;
            }
            break;
        }
        case RVDT_CAL_CALC_INV_XT_X_DETERMINANTS:
        {
            /* In order to calculate the inverse of matrix XT * X, first the matrix of minors and their
             * determinants must be calculated. After that the Adjugate matrix must be calculated and then finally
             * the inverse can be calculated. */

            /* Loop thru all samples of Matrix X to form the Matrix of Minors and Calculate their corresponding
             * Determinants and place into matrix form. */
            if((iRow < RVDT_CALIBRATION_POLY_NUM_COEFFS) && (iCol < RVDT_CALIBRATION_POLY_NUM_COEFFS))
            {
                kRow = 0U; /* Initialize the 3x3 Matrix of Minors Row Index */
                kCol = 0U; /* Initialize the 3x3 Matrix of Minors Column Index */

                /* generate the 3x3 matrix of minors for the associated element of the 4x4 matrix */
                for(jRow = 0U; jRow < RVDT_CALIBRATION_POLY_NUM_COEFFS; jRow++)
                {
                    for(jCol = 0U; jCol < RVDT_CALIBRATION_POLY_NUM_COEFFS; jCol++)
                    {
                        /* Skip the row and column of the index that the matrix of minor is being calculated for */
                        if((iRow != jRow) && (iCol != jCol))
                        {
                            if((kRow < RVDT_CALIBRATION_SIZE_3X3) && (kCol < RVDT_CALIBRATION_SIZE_3X3))
                            {
                                /* copy the elements from the 4x4 matrix to the 3x3 matrix of minor */
                                f32mM[kRow][kCol] = tPolyMatrixCalcs.f32mXT_X[jRow][jCol];

                                kCol++;
                                if(kCol >= RVDT_CALIBRATION_SIZE_3X3)
                                {
                                    /* reset the column index and go to next row */
                                    kCol = 0U;
                                    kRow++;
                                    if(kRow >= RVDT_CALIBRATION_SIZE_3X3)
                                    {
                                        /* Matrix of Minor is built. Calculate the determinant of it. */
                                        /* double float math for accuracy */

                                        /* Determinant 1 =  a11 * ((a22 * a33) - (a23 * a32)) */
                                        f64a = (float64_t) f32mM[1][1];
                                        f64d = (float64_t) f32mM[2][2];
                                        f64b = (float64_t) f32mM[1][2];
                                        f64c = (float64_t) f32mM[2][1];
                                        f64Det1 = (f64a * f64d);
                                        f64Det1 -= (f64b * f64c);
                                        f64Det1 *= (float64_t) f32mM[0][0];

                                        /* Determinant 2 = -a12 * ((a12 * a21) - (a23 * a31)) */
                                        f64a = (float64_t) f32mM[1][0];
                                        f64d = (float64_t) f32mM[2][2];
                                        f64b = (float64_t) f32mM[1][2];
                                        f64c = (float64_t) f32mM[2][0];
                                        f64Det2 = (f64a * f64d);
                                        f64Det2 -= (f64b * f64c);
                                        f64Det2 *= (float64_t) f32mM[0][1];
                                        f64Det2 *= -1.0F;

                                        /* Determinant 3 =  a13 * ((a21 * a32) - (a22 * a31)) */
                                        f64a = (float64_t) f32mM[1][0];
                                        f64d = (float64_t) f32mM[2][1];
                                        f64b = (float64_t) f32mM[1][1];
                                        f64c = (float64_t) f32mM[2][0];
                                        f64Det3 = (f64a * f64d);
                                        f64Det3 -= (f64b * f64c);
                                        f64Det3 *= (float64_t) f32mM[0][2];

                                        /* Matrix of Minor 3x3 Determinant = Determinant 1 + Determinant 2 + Determinant 3 */
                                        /* Cast back to single float */
                                        tPolyMatrixCalcs.f32Det[iRow][iCol] = (float32_t) (f64Det1 + f64Det2 + f64Det3);
                                    }
                                }
                            }
                        }
                    }
                }

                /* Increment 4x4 Matrix of Determinants Index */
                iCol++;
                if(iCol >= RVDT_CALIBRATION_POLY_NUM_COEFFS)
                {
                    iCol = 0U;
                    iRow++;
                    if(iRow >= RVDT_CALIBRATION_POLY_NUM_COEFFS)
                    {
                        tState = RVDT_CAL_CALC_INV_XT_X_ADJUGATE;
                    }
                }
            }

            break;
        }
        case RVDT_CAL_CALC_INV_XT_X_ADJUGATE:
        {
            /* Calculate the Adjugate of Matrix X */
            for(iCol = 0U; iCol < RVDT_CALIBRATION_POLY_NUM_COEFFS; iCol++)
            {
                for(iRow = 0U; iRow < RVDT_CALIBRATION_POLY_NUM_COEFFS; iRow++)
                {
                    /* Transpose the Determinant Matrix First */
                    tPolyMatrixCalcs.f32Adj[iRow][iCol] = tPolyMatrixCalcs.f32Det[iCol][iRow];
                    /* (-1)^(R+C)*f32Det:  Invert odd row plus column entries in the matrix. */
                    if(((iRow + iCol) % 2U) != 0U)
                    {
                        /* Invert the value */
                        tPolyMatrixCalcs.f32Adj[iRow][iCol] = -tPolyMatrixCalcs.f32Adj[iRow][iCol];
                    }
                }
            }

            tState = RVDT_CAL_CALC_INV_XT_X;

            break;
        }
        case RVDT_CAL_CALC_INV_XT_X:
        {
            /* Calculate the Determinant of the 4x4 Matrix XT * X */
            f32DetXT_X_inv = 0.0F;
            /* Determinant of 4x4 Matrix XT_X = (a11 * detm11) - (a12 * detm12) + (a13 * detm13) - (a14 * detm14) */
            f32DetXT_X_inv += (tPolyMatrixCalcs.f32mXT_X[0][0] * tPolyMatrixCalcs.f32Det[0][0]);
            f32DetXT_X_inv -= (tPolyMatrixCalcs.f32mXT_X[0][1] * tPolyMatrixCalcs.f32Det[0][1]);
            f32DetXT_X_inv += (tPolyMatrixCalcs.f32mXT_X[0][2] * tPolyMatrixCalcs.f32Det[0][2]);
            f32DetXT_X_inv -= (tPolyMatrixCalcs.f32mXT_X[0][3] * tPolyMatrixCalcs.f32Det[0][3]);

            /* Protect from diving by 0 */
            if(f32DetXT_X_inv != 0.0F)
            {
                /* Take the inverse of the determinant */
                f32DetXT_X_inv = (1 / f32DetXT_X_inv);
            }
            else
            {
                /* Set to 1 and continue processing and let the verification step catch that the
                 * calibration process fails. Setting to 1 will protect from dividing by 0 errors */
                f32DetXT_X_inv = 1.0F;
            }

            /* Calculate the Inverse of Matrix X */
            /* Inverse = ((1 / Determinant of X) * Matrix X Adjugate) */
            for(iRow = 0U; iRow < RVDT_CALIBRATION_POLY_NUM_COEFFS; iRow++)
            {
                for(iCol = 0U; iCol < RVDT_CALIBRATION_POLY_NUM_COEFFS; iCol++)
                {
                    /* Calculate each element for the Inverse X Matrix */
                    tPolyMatrixCalcs.f32mXT_X_inv[iRow][iCol] = (f32DetXT_X_inv * tPolyMatrixCalcs.f32Adj[iRow][iCol]);
                }
            }

            tState = RVDT_CAL_CALC_COEFFICIENTS;

            break;
        }
        case RVDT_CAL_CALC_COEFFICIENTS:
        {
            /* Calculate the 3rd Order Polynomial Coefficients */
            /* [ A ] = [ XT * X ] ^-1 * [ XT * Y ] */
            /* A11 = (Xinv11 * Y11) + (Xinv12 * Y21) + (Xinv13 * Y31) + (Xinv14 * Y41) */
            tPolyMatrixCalcs.f32mA[0] =  (tPolyMatrixCalcs.f32mXT_X_inv[0][0] * tPolyMatrixCalcs.f32mXT_Y[0]);
            tPolyMatrixCalcs.f32mA[0] += (tPolyMatrixCalcs.f32mXT_X_inv[0][1] * tPolyMatrixCalcs.f32mXT_Y[1]);
            tPolyMatrixCalcs.f32mA[0] += (tPolyMatrixCalcs.f32mXT_X_inv[0][2] * tPolyMatrixCalcs.f32mXT_Y[2]);
            tPolyMatrixCalcs.f32mA[0] += (tPolyMatrixCalcs.f32mXT_X_inv[0][3] * tPolyMatrixCalcs.f32mXT_Y[3]);

            /* A21 = (Xinv21 * Y11) + (Xinv22 * Y21) + (Xinv23 * Y31) + (Xinv24 * Y41) */
            tPolyMatrixCalcs.f32mA[1] =  (tPolyMatrixCalcs.f32mXT_X_inv[1][0] * tPolyMatrixCalcs.f32mXT_Y[0]);
            tPolyMatrixCalcs.f32mA[1] += (tPolyMatrixCalcs.f32mXT_X_inv[1][1] * tPolyMatrixCalcs.f32mXT_Y[1]);
            tPolyMatrixCalcs.f32mA[1] += (tPolyMatrixCalcs.f32mXT_X_inv[1][2] * tPolyMatrixCalcs.f32mXT_Y[2]);
            tPolyMatrixCalcs.f32mA[1] += (tPolyMatrixCalcs.f32mXT_X_inv[1][3] * tPolyMatrixCalcs.f32mXT_Y[3]);

            /* A31 = (Xinv31 * Y11) + (Xinv32 * Y21) + (Xinv33 * Y31) + (Xinv34 * Y41) */
            tPolyMatrixCalcs.f32mA[2] =  (tPolyMatrixCalcs.f32mXT_X_inv[2][0] * tPolyMatrixCalcs.f32mXT_Y[0]);
            tPolyMatrixCalcs.f32mA[2] += (tPolyMatrixCalcs.f32mXT_X_inv[2][1] * tPolyMatrixCalcs.f32mXT_Y[1]);
            tPolyMatrixCalcs.f32mA[2] += (tPolyMatrixCalcs.f32mXT_X_inv[2][2] * tPolyMatrixCalcs.f32mXT_Y[2]);
            tPolyMatrixCalcs.f32mA[2] += (tPolyMatrixCalcs.f32mXT_X_inv[2][3] * tPolyMatrixCalcs.f32mXT_Y[3]);

            /* A41 = (Xinv41 * Y11) + (Xinv42 * Y21) + (Xinv43 * Y31) + (Xinv44 * Y41) */
            tPolyMatrixCalcs.f32mA[3] =  (tPolyMatrixCalcs.f32mXT_X_inv[3][0] * tPolyMatrixCalcs.f32mXT_Y[0]);
            tPolyMatrixCalcs.f32mA[3] += (tPolyMatrixCalcs.f32mXT_X_inv[3][1] * tPolyMatrixCalcs.f32mXT_Y[1]);
            tPolyMatrixCalcs.f32mA[3] += (tPolyMatrixCalcs.f32mXT_X_inv[3][2] * tPolyMatrixCalcs.f32mXT_Y[2]);
            tPolyMatrixCalcs.f32mA[3] += (tPolyMatrixCalcs.f32mXT_X_inv[3][3] * tPolyMatrixCalcs.f32mXT_Y[3]);

            /* Store the 3rd Order Polynomial Coefficients into NVM */
            Nvm_Rigging_Temp.tSkewSnsr.tRvdtCalibration.f32a3 = tPolyMatrixCalcs.f32mA[3];
            Nvm_Rigging_Temp.tSkewSnsr.tRvdtCalibration.f32a2 = tPolyMatrixCalcs.f32mA[2];
            Nvm_Rigging_Temp.tSkewSnsr.tRvdtCalibration.f32a1 = tPolyMatrixCalcs.f32mA[1];
            Nvm_Rigging_Temp.tSkewSnsr.tRvdtCalibration.f32a0 = tPolyMatrixCalcs.f32mA[0];

            /* Done with Calibration of RVDT routine */
            tState = RVDT_CAL_COMPLETE;

            break;
        }
        case RVDT_CAL_COMPLETE:
        {
            /* set flag to indicate successful completion of RVDT Calibration Routine. */
            bRet = true;
            break;
        }
        default:
        {
            tState = RVDT_CAL_INIT;
            break;
        }
    }

    return bRet;
}

/****************************************************************************************************
*  Function: Rvdt_ApplyCalibrationEquation
*  Purpose:
*  This function will apply the 3rd order polynomial to the input RVDT ADC value and ensure the output
*  is within the tolerance of the ADC values before returning the updated value.
*
*  Global Inputs:
*  Global Outputs:
*  Input: f32PositionScaled0to1:  Scaled representation of 12-bit RVDT ADC position
*  Output: f32RvdtCalVal:  Calibrated and scaled representation of 12-bit RVDT ADC position
****************************************************************************************************/
float32_t Rvdt_ApplyCalibrationEquation(float32_t f32PositionScaled0to1)
{
    float32_t f32RvdtAdcVal = f32PositionScaled0to1;
    float32_t f32RvdtCalVal = 0.0F;
    float32_t f32PolyTermTemp;
    float32_t f32x, f32x2, f32x3;

    /* Adjust the position based on the equation resulting from the calibration process. */
    /* a0 term */
    f32RvdtCalVal = Nvm_Rigging_Temp.tSkewSnsr.tRvdtCalibration.f32a0;
    /* a1 term */
    f32x = f32RvdtAdcVal;   /* X    */
    f32PolyTermTemp = Nvm_Rigging_Temp.tSkewSnsr.tRvdtCalibration.f32a1 * f32x;
    f32RvdtCalVal += f32PolyTermTemp;
    /* a2 term */
    f32x2 = f32x * f32x;   /* X^2  */
    f32PolyTermTemp = Nvm_Rigging_Temp.tSkewSnsr.tRvdtCalibration.f32a2 * f32x2;
    f32RvdtCalVal += f32PolyTermTemp;
    /* a3 term */
    f32x3 = f32x2 * f32x; /* X^3  */
    f32PolyTermTemp = Nvm_Rigging_Temp.tSkewSnsr.tRvdtCalibration.f32a3 * f32x3;
    f32RvdtCalVal += f32PolyTermTemp;

    /* Check boundaries of the calibrated value */
    if(f32RvdtCalVal < 0.0F)
    {
        /* minimum value RVDT Calibrated Fractional Value is 0 */
        f32RvdtCalVal = 0.0F;
    }
    else if(f32RvdtCalVal > 1.0F)
    {
        /* Maximum value for RVDT Calibrated Fractional Value is 1.0 */
        f32RvdtCalVal = 1.0F;
    }

    return f32RvdtCalVal;
}

/****************************************************************************************************
*  Function: Rvdt_GetScaledPositionFlapAngle
*  Purpose:
*  This function will calculate and save the scaled position based on the Rvdt positional info
*  for the Stroke Transfer Function Calculations.
*
*  Global Inputs:
*  Global Outputs:
*  Input:
*  Output:
****************************************************************************************************/
void Rvdt_GetScaledPositionFlapAngle(tRvdtCalcsFlapAngle_t* const ptRvdtCalcs)
{
    /* NULL check the pointer */
    if(ptRvdtCalcs != NULL)
    {
        /* Average the RVDT Counts (Channel A and Channel B) */
        ptRvdtCalcs->s16AdcAvgRaw = (int16_t)((int32_t)Adc_Raw.val.u16_RVDT_POS);

        /* Convert the 12-bit positional data into a fraction between 0 and 1 to represent the position */
        ptRvdtCalcs->f32ScaledPosition = (float32_t)ptRvdtCalcs->s16AdcAvgRaw * RVDT_CNT2FRAC;

        /* Apply the calibration equation */
        ptRvdtCalcs->f32ScaledPositionCalibrated = Rvdt_ApplyCalibrationEquation(ptRvdtCalcs->f32ScaledPosition);
    }

    return;
}

/****************************************************************************************************
*  Function: Rvdt_GetScaledPositionStroke
*  Purpose:
*  This function will calculate and save the scaled position based on the Rvdt positional info
*  for the Stroke Transfer Function Calculations.
*
*  Global Inputs:
*  Global Outputs:
*  Input:
*  Output:
****************************************************************************************************/
void Rvdt_GetScaledPositionStroke(tRvdtCalcsStroke_t* const ptRvdtCalcs)
{
    /* NULL check the pointer */
    if(ptRvdtCalcs != NULL)
    {
        /* Average the RVDT Counts (Channel A and Channel B) */
        ptRvdtCalcs->s16AdcAvgRaw = (int16_t)((int32_t)Adc_Raw.val.u16_RVDT_POS);

        /* Convert the 12-bit positional data into a fraction between 0 and 1 to represent the position */
        ptRvdtCalcs->f32ScaledPosition = (float32_t)ptRvdtCalcs->s16AdcAvgRaw * RVDT_CNT2FRAC;

        /* Apply the calibration equation */
        ptRvdtCalcs->f32ScaledPositionCalibrated = Rvdt_ApplyCalibrationEquation(ptRvdtCalcs->f32ScaledPosition);
    }

    return;
}

///****************************************************************************************************
//*  Function: Rvdt_GetStroke
//*  Purpose:
//*  This function will calculate and save off the Onside Stroke based on the value of the RVDT. It
//*  also will convert the stroke to quad counts and save that value off.
//*
//*  Global Inputs: P_POS_raw, S_POS_IN_raw
//*  Global Outputs: tRvdtCalcsOnside.f32Stroke, tRvdtCalcsOnside.s16QuadStroke
//*  Input:
//*  Output:
//****************************************************************************************************/
//void Rvdt_GetStroke(void)
//{
//    /* Average the Onside RVDT Counts (Channel A and Channel B) */
//    tRvdtCalcs.tStroke.s16AdcAvgRaw = (int16_t)((int32_t)Adc_Raw.val.u16_RVDT_POS);
//
//    /* Apply the gain and offset and convert to floating point for the RVDT Calculation Function */
//    tRvdtCalcs.tStroke.f32ScaledPositionCalibrated = Rvdt_ApplyCalibrationEquation(tRvdtCalcs.tStroke.s16AdcAvgRaw);
//
//    /* Calculate the Stroke Based on RVDT count */
//    tRvdtCalcs.tStroke.f32Stroke = Rvdt_CalcStrokeFromRvdt(G_eActuatorNumber, tRvdtCalcs.tStroke.f32ScaledPositionCalibrated);
//
//    /* Calculate the quad count associated with the stroke */
//    tRvdtCalcs.tStroke.s16QuadCnt = Panel_ConvertStrokeToElectricalCycleCount(tRvdtCalcs.tStroke.f32Stroke);
//
//    return;
//}
//
///****************************************************************************************************
//*  Function: Rvdt_GetOnsideFlapAngle
//*  Purpose:
//*  This function will calculate and save off the Onside Flap Angle based on the value of the RVDT.
//*
//*  Global Inputs: P_POS, S_POS_IN
//*  Global Outputs: tRvdtCalcsOnside.f32FlapAngle
//*  Input:
//*  Output:
//****************************************************************************************************/
//void Rvdt_GetFlapAngle(void)
//{
//    /* Average the Onside RVDT Counts (Channel A and Channel B) */
//    tRvdtCalcs.tFlapAngle.s16RvdtAdcAvgRaw = (int16_t)((int32_t)Adc_Raw.val.u16_RVDT_POS);
//
//    /* Apply the gain and offset convert to floating point for the RVDT Calculation Function */
//    tRvdtCalcs.tFlapAngle.f32RvdtAdcAvgCalibrated = Rvdt_ApplyCalibrationEquation(tRvdtCalcs.tFlapAngle.s16RvdtAdcAvgRaw);
//
//    /* Calculate the Flap Angle Based on RVDT count */
//    tRvdtCalcs.tFlapAngle.f32FlapAngle = Rvdt_CalcFlapAngleFromRvdt(G_eActuatorNumber, tRvdtCalcs.tFlapAngle.f32RvdtAdcAvgCalibrated);
//
//    return;
//}

// CONDITIONS FOR CHECKING RVDT EXCITATION. MAY NEED TO USE IN CALC FUNCTIONS IN FUTURE
//    /* shall calculate onside flap angle if none of the monitors that terminate the sine wave excitation have tripped, and the +14V is at a safe level */
//    if ((LatchedFaults_Stat.bit.n14Volt_M != 0) || (LatchedFaults_Stat.bit.FPSU_L_Chnl_A_M != 0) ||
//        (LatchedFaults_Stat.bit.FPSU_L_Chnl_B_M != 0) || (LatchedFaults_Stat.bit.FPSU_R_Chnl_A_M != 0) ||
//        (LatchedFaults_Stat.bit.FPSU_R_Chnl_B_M != 0) || (v14V_MON <= 3208))

/****************************************************************************************************
*  Function: Rvdt_CalcStrokeFromRvdtPrimary
*  Purpose:
*  This function implements the spline polynomial interpolation to compute
*  Stroke in Inches from RVDT ADC Count.
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
*  Global Inputs: P_POS, S_POS_IN
*  Global Outputs:
*  Input: tActuatorIndex A1,A2,A3,A4, f32RvdtAdcCount in ADC Counts < 4096.0
*  Output:
****************************************************************************************************/
float32_t Rvdt_CalcStrokeFromRvdt(eActuator_t tActuatorIndex, float32_t f32RvdtAdcCount)
{
    float32_t f32Stroke = NAN;      /* Initialize to NAN */
    float32_t f32AdcLowerLimit;     /* lower limit used in poly equation from lookup table */
    bool_t bFound = false;          /* Flag to determine if equation found to use */
    float32_t f32x, f32x2, f32x3;   /* Polynomial multiples */
    float32_t f32RvdtAdcCntScaled = RVDT_CNT2FRAC * f32RvdtAdcCount; /* Scale the ADC between 0 and 1 */

    /* Pointer to the array of Polynomial Coefficients [a3, a2, a1, a0]
     * selected for spline polynomial interpolation equation */
    const float32_t *paf32PolyCoeffs = NULL;
    float32_t f32AdcDiffFromLowerLimit;

    /* Polynomial Term Temp Variable used in each term calculation for spline
     * piecewise polynomial interpolation equation */
    float32_t f32PolyTermTemp;

    /* Calculate the Stroke only for Valid Actuator Number and valid rvdt counts */
    if((tActuatorIndex < NUM_ACTUATOR_TYPES) && ((f32RvdtAdcCntScaled >= 0.0) && (f32RvdtAdcCntScaled < 1.0)))
    {
        uint16_t i = 0;
        do
        {
            /* Use the RVDT ADC Count to find the spline polynomial to apply.
             * When the RVDT ADC Count is less than the Upper Limit of the increasing
             * order 16 index lookup table, then select that indice's polynomial values
             * for the final calculation.
             * OR, if went thru all indices and ADC Count is higher than Upper Limit
             * of 16th polynomial, therefore set to the 16th polynomial rather
             * than inducing an error. */
            if((f32RvdtAdcCntScaled < af32RvdtPolyBreaksTable[tActuatorIndex][i][1]) || (i == (NUM_POLYNOMIAL_EQUATIONS - 1U)))
            {
                bFound = true;
                /* Select the array of polynomial coefficients to use in the
                 * final Stroke Calculation */
                paf32PolyCoeffs = &af32RvdtToStrokeCoeffsTable[tActuatorIndex][i][0];

                /* Set the Lower Limit of the RVDT ADC Count for the equation */
                f32AdcLowerLimit = af32RvdtPolyBreaksTable[tActuatorIndex][i][0];
            }
            i++;
        } while ((bFound == false) && (i < NUM_POLYNOMIAL_EQUATIONS));

        /* Calculate the stroke if the equation values were found and the
         * pointer to the coefficient array is not NULL */
        if((bFound == true) && (paf32PolyCoeffs != NULL))
        {
            f32AdcDiffFromLowerLimit = f32RvdtAdcCntScaled - f32AdcLowerLimit;
            /* a0 term */
            f32Stroke = paf32PolyCoeffs[3];  /* Init to a0 Term */
            /* a1 term */
            f32x = f32AdcDiffFromLowerLimit;   /* X    */
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
*  Function: Rvdt_CalcFlapAngleFromRvdt
*  Purpose:
*  This function implements the spline polynomial interpolation to compute
*  Flap Angle in degrees from RVDT ADC Count.
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
*  Global Inputs: P_POS, S_POS_IN
*  Global Outputs:
*  Input: tActuatorIndex A1,A2,A3,A4, f32RvdtAdcCount in ADC Counts < 4096.0
*  Output:
****************************************************************************************************/
float32_t Rvdt_CalcFlapAngleFromRvdt(eActuator_t tActuatorIndex, float32_t f32RvdtAdcCount)
{
    float32_t f32FlapAngle = NAN;   /* Initialize to NAN */
    float32_t f32AdcLowerLimit;     /* lower limit used in poly equation from lookup table */
    bool_t bFound = false;          /* Flag to determine if equation found to use */
    float32_t f32x, f32x2, f32x3;   /* Polynomial multiples */
    float32_t f32RvdtAdcCntScaled = RVDT_CNT2FRAC * f32RvdtAdcCount; /* Scale the ADC between 0 and 1 */

    /* Pointer to the array of Polynomial Coefficients [a3, a2, a1, a0]
     * selected for spline polynomial interpolation equation */
    const float32_t *paf32PolyCoeffs = NULL;
    float32_t f32AdcDiffFromLowerLimit;

    /* Polynomial Term Temp Variable used in each term calculation for spline
     * piecewise polynomial interpolation equation */
    float32_t f32PolyTermTemp;

    /* Calculate the Flap Angle only for Valid Actuator Number and valid rvdt counts */
    if((tActuatorIndex < NUM_ACTUATOR_TYPES) && ((f32RvdtAdcCntScaled >= 0.0) && (f32RvdtAdcCntScaled < 1.0)))
    {
        uint16_t i = 0;
        do
        {
            /* Use the RVDT ADC Count to find the spline polynomial to apply.
             * When the RVDT ADC Count is less than the Upper Limit of the increasing
             * order 16 index lookup table, then select that indice's polynomial values
             * for the final calculation.
             * OR, if went thru all indices and ADC Count is higher than Upper Limit
             * of 16th polynomial, therefore set to the 16th polynomial rather
             * than inducing an error. */
            if((f32RvdtAdcCntScaled < af32RvdtPolyBreaksTable[tActuatorIndex][i][1]) || (i == (NUM_POLYNOMIAL_EQUATIONS - 1U)))
            {
                bFound = true;
                /* Select the array of polynomial coefficients to use in the
                 * final Flap Angle Calculation */
                paf32PolyCoeffs = &af32RvdtToFlapAngleCoeffsTable[tActuatorIndex][i][0];

                /* Set the Lower Limit of the RVDT ADC Count for the equation */
                f32AdcLowerLimit = af32RvdtPolyBreaksTable[tActuatorIndex][i][0];
            }
            i++;
        } while ((bFound == false) && (i < NUM_POLYNOMIAL_EQUATIONS));

        /* Calculate the flap angle if the equation values were found and the
         * pointer to the coefficient array is not NULL */
        if((bFound == true) && (paf32PolyCoeffs != NULL))
        {
            f32AdcDiffFromLowerLimit = f32RvdtAdcCntScaled - f32AdcLowerLimit;
            /* a0 term */
            f32FlapAngle = paf32PolyCoeffs[3];  /* Init to a0 Term */
            /* a1 term */
            f32x = f32AdcDiffFromLowerLimit;   /* X    */
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

/****************************************************************************************************
*  Function: Rvdt_SetNvmRigData
*  Purpose:
*  This function saves the positional data associated with the rig position input for the RVDT as the
*  skew sensor.
*
*  Global Inputs: Adc_Averaged
*  Global Outputs: Nvm_Rigging_Temp
*  Input: ePosition:  Rig Position to save data for
*  Output: None
****************************************************************************************************/
void Rvdt_SetNvmRigData(RIG_POSITIONS_T ePosition)
{
    Nvm_Rigging_Temp.tSkewSnsr.tRvdtPosData[ePosition].u16RvdtPos = Adc_Averaged.val.u16_RVDT_POS;
    Nvm_Rigging_Temp.tSkewSnsr.tRvdtPosData[ePosition].u16RvdtSum = Adc_Averaged.val.u16_RVDT_SUM;
    Nvm_Rigging_Temp.tSkewSnsr.tRvdtPosData[ePosition].u16RvdtAvg = (uint16_t)((uint32_t)Adc_Averaged.val.u16_RVDT_POS);
}

/****************************************************************************************************
*  Function: Rvdt_SetNvmPanelData
*  Purpose:
*  This function saves the stroke and flap angle calculation data as part of the Panel data struct
*  in NVM
*
*  Global Inputs: tIncoderInputs
*  Global Outputs: Nvm_Rigging_Temp
*  Input: ePosition:  Rig Position to save data for
*  Output: None
****************************************************************************************************/
void Rvdt_SetNvmPanelData(RIG_POSITIONS_T ePosition)
{
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tFlapAngle.f32FlapAngle = tSkewSnsrCalcs.tFlapAngle.f32FlapAngle;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tFlapAngle.f32ScaledPosition = tSkewSnsrCalcs.tFlapAngle.f32ScaledPosition;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tFlapAngle.f32ScaledPositionCalibrated = tSkewSnsrCalcs.tFlapAngle.f32ScaledPositionCalibrated;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tFlapAngle.s16QuadCnt = tSkewSnsrCalcs.tFlapAngle.s16QuadCnt;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tFlapAngle.s16AdcAvgRaw = tSkewSnsrCalcs.tFlapAngle.s16AdcAvgRaw;

    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tStroke.f32Stroke = tSkewSnsrCalcs.tStroke.f32Stroke;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tStroke.f32ScaledPosition = tSkewSnsrCalcs.tStroke.f32ScaledPosition;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tStroke.f32ScaledPositionCalibrated = tSkewSnsrCalcs.tStroke.f32ScaledPositionCalibrated;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tStroke.s16QuadCnt = tSkewSnsrCalcs.tStroke.s16QuadCnt;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tStroke.s16AdcAvgRaw = tSkewSnsrCalcs.tStroke.s16AdcAvgRaw;
}
#endif
/* end rvdt.c*/

