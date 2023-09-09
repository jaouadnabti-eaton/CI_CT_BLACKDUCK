/****************************************************************************************************
*  File name: encoder.c
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
*  Aniket Mahadik   04/05/2023    N/A        Ported to MCU
*
****************************************************************************************************/

/*      Include Files
*/
#if defined(__SKEW_SNSR_ENCODER__)

#include "encoder.h"
#include "gpio.h"
#include "F2837xD_device.h"
#include "icc.h"
#include "nvm.h"

/*      Local Type Definitions
*/

/*      Local Defines
*/
#define SRGR1_CLK_DIV_BY_100       99U
#define CRC8_TABLE_SIZE            (256U)
#define CRC8_XOR_FINAL             (0U)
#define CRC8_BYTE_MASK             (0x00FFU)

/*      Global Variables
*/
//tEncoderCalcs_t tEncoderCalcs = { 0 };
uint16_t u16IncoderRxNotReadyCntr = 0U;
uint16_t u16IncoderPositionNotValidCntr = 0U;
uint16_t u16IncoderCrc8CheckFailedCntr = 0U;
tIncoderInputs_t tIncoderInputs = { 0 };

/*      Local ROM Constants
*/
/* Rig IncOder Ideal Count Table */
const uint32_t u32EncoderIdealCnt[NUM_ACTUATOR_TYPES][NUM_RIG_POSITIONS] =
{
  /*{   RSL,    N1,  ZERO,    P1,    P2,    P3,    P4,    P5,    P6,    P7,    P8,    P9,   P10,   P11,   ESL } */
    {    82,   181,   367,   708,   819,  1182,  1417,  1751,  1815,  2313,  2708,  3130,  3232,  3721,  3824 },  /* Actuator 1:  TBD */
    {    82,   181,   367,   708,   819,  1182,  1417,  1751,  1815,  2313,  2708,  3130,  3232,  3721,  3824 },  /* Actuator 2:  TBD */
    {  1754,  2560,  4839,  6982,  7908, 11648, 13610, 16355, 16882, 19077, 21803, 24561, 27385, 30030, 30636 },  /* Actuator 3 */
    {  1759,  2571,  4850,  6993,  7918, 11658, 13621, 16366, 16893, 19088, 21814, 24572, 27396, 30041, 30646 },  /* Actuator 4 */
};

/* Table of Ideal RVDT Fraction (0 to 1.0) converted from corresponding ADC Values per Actuator Number and Rig Position */
const float32_t f32EncoderIdealVal0to1[NUM_ACTUATOR_TYPES][NUM_RIG_POSITIONS] =
{
  /*{                RSL,                   N1,                 ZERO,                   P1,                   P2,                   P3,                   P4,                   P5,                   P6,                   P7,                   P8,                   P9,                  P10,                  P11,                  ESL } */
    { (82.0F * ENCODER_CNT2FRAC),  (181.0F * ENCODER_CNT2FRAC),  (367.0F * ENCODER_CNT2FRAC),  (708.0F * ENCODER_CNT2FRAC),  (819.0F * ENCODER_CNT2FRAC), (1182.0F * ENCODER_CNT2FRAC), (1417.0F * ENCODER_CNT2FRAC), (1751.0F * ENCODER_CNT2FRAC), (1815.0F * ENCODER_CNT2FRAC), (2313.0F * ENCODER_CNT2FRAC), (2708.0F * ENCODER_CNT2FRAC), (3130.0F * ENCODER_CNT2FRAC), (3232.0F * ENCODER_CNT2FRAC), (3721.0F * ENCODER_CNT2FRAC), (3824.0F * ENCODER_CNT2FRAC) },  /* Actuator 1 */
    { (82.0F * ENCODER_CNT2FRAC),  (181.0F * ENCODER_CNT2FRAC),  (367.0F * ENCODER_CNT2FRAC),  (708.0F * ENCODER_CNT2FRAC),  (819.0F * ENCODER_CNT2FRAC), (1182.0F * ENCODER_CNT2FRAC), (1417.0F * ENCODER_CNT2FRAC), (1751.0F * ENCODER_CNT2FRAC), (1815.0F * ENCODER_CNT2FRAC), (2313.0F * ENCODER_CNT2FRAC), (2708.0F * ENCODER_CNT2FRAC), (3130.0F * ENCODER_CNT2FRAC), (3232.0F * ENCODER_CNT2FRAC), (3721.0F * ENCODER_CNT2FRAC), (3824.0F * ENCODER_CNT2FRAC) },  /* Actuator 2 */
    { (60.0F * ENCODER_CNT2FRAC),  (165.0F * ENCODER_CNT2FRAC),  (461.0F * ENCODER_CNT2FRAC),  (740.0F * ENCODER_CNT2FRAC),  (860.0F * ENCODER_CNT2FRAC), (1347.0F * ENCODER_CNT2FRAC), (1602.0F * ENCODER_CNT2FRAC), (1959.0F * ENCODER_CNT2FRAC), (2027.0F * ENCODER_CNT2FRAC), (2313.0F * ENCODER_CNT2FRAC), (2667.0F * ENCODER_CNT2FRAC), (3026.0F * ENCODER_CNT2FRAC), (3393.0F * ENCODER_CNT2FRAC), (3737.0F * ENCODER_CNT2FRAC), (3816.0F * ENCODER_CNT2FRAC) },  /* Actuator 3 */
    { (60.0F * ENCODER_CNT2FRAC),  (165.0F * ENCODER_CNT2FRAC),  (461.0F * ENCODER_CNT2FRAC),  (740.0F * ENCODER_CNT2FRAC),  (860.0F * ENCODER_CNT2FRAC), (1347.0F * ENCODER_CNT2FRAC), (1602.0F * ENCODER_CNT2FRAC), (1959.0F * ENCODER_CNT2FRAC), (2027.0F * ENCODER_CNT2FRAC), (2313.0F * ENCODER_CNT2FRAC), (2667.0F * ENCODER_CNT2FRAC), (3026.0F * ENCODER_CNT2FRAC), (3393.0F * ENCODER_CNT2FRAC), (3737.0F * ENCODER_CNT2FRAC), (3815.0F * ENCODER_CNT2FRAC) },  /* Actuator 4 */
};

/****************************************************************************************************
* ENCODER TO STROKE Spline Interpolation Polynomial Equation Constants.
*
* Coefficients a3, a2, a1, a0 used in the polynomials based on
* actuators 1, 2, 3, 4 */
const float32_t af32EncoderToStrokeCoeffsTable[NUM_ACTUATOR_TYPES][NUM_POLYNOMIAL_EQUATIONS][NUM_POLYNOMIAL_COEFFS] =
{
    {   /* A1 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {     -2.55325E+02,    1.07738E+02,    2.04442E+01,   -5.15000E-01  },
        {     -2.55325E+02,    1.03195E+02,    2.16953E+01,   -3.90000E-01  },
        {     -3.17321E+02,    9.87264E+01,    2.28733E+01,   -2.59970E-01  },
        {     -3.77766E+02,    8.83755E+01,    2.49077E+01,    0.00000E+00  },
        {     -3.31731E+02,    6.56537E+01,    2.79959E+01,    5.31860E-01  },
        {     -4.33778E+02,    5.92028E+01,    2.88052E+01,    7.16000E-01  },
        {     -2.98416E+02,    3.14996E+01,    3.07361E+01,    1.35186E+00  },
        {     -3.66180E+02,    1.91233E+01,    3.14359E+01,    1.78200E+00  },
        {     -3.00924E+02,   -2.38272E+00,    3.17637E+01,    2.40200E+00  },
        {     -2.91960E+02,   -5.79360E+00,    3.17328E+01,    2.52196E+00  },
        {     -2.55788E+02,   -3.13714E+01,    3.06475E+01,    3.43642E+00  },
        {     -2.43355E+02,   -4.91544E+01,    2.87813E+01,    4.12662E+00  },
        {     -2.03067E+02,   -6.72597E+01,    2.58943E+01,    4.80644E+00  },
        {     -2.41337E+02,   -7.09066E+01,    2.50672E+01,    4.95900E+00  },
        {     -2.25065E+02,   -9.16666E+01,    2.04056E+01,    5.61378E+00  },
        {     -2.25065E+02,   -9.57225E+01,    1.92800E+01,    5.73300E+00  },
    },
    {   /* A2 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {     -2.55325E+02,    1.07738E+02,    2.04442E+01,   -5.15000E-01  },
        {     -2.55325E+02,    1.03195E+02,    2.16953E+01,   -3.90000E-01  },
        {     -3.17321E+02,    9.87264E+01,    2.28733E+01,   -2.59970E-01  },
        {     -3.77766E+02,    8.83755E+01,    2.49077E+01,    0.00000E+00  },
        {     -3.31731E+02,    6.56537E+01,    2.79959E+01,    5.31860E-01  },
        {     -4.33778E+02,    5.92028E+01,    2.88052E+01,    7.16000E-01  },
        {     -2.98416E+02,    3.14996E+01,    3.07361E+01,    1.35186E+00  },
        {     -3.66180E+02,    1.91233E+01,    3.14359E+01,    1.78200E+00  },
        {     -3.00924E+02,   -2.38272E+00,    3.17637E+01,    2.40200E+00  },
        {     -2.91960E+02,   -5.79360E+00,    3.17328E+01,    2.52196E+00  },
        {     -2.55788E+02,   -3.13714E+01,    3.06475E+01,    3.43642E+00  },
        {     -2.43355E+02,   -4.91544E+01,    2.87813E+01,    4.12662E+00  },
        {     -2.03067E+02,   -6.72597E+01,    2.58943E+01,    4.80644E+00  },
        {     -2.41337E+02,   -7.09066E+01,    2.50672E+01,    4.95900E+00  },
        {     -2.25065E+02,   -9.16666E+01,    2.04056E+01,    5.61378E+00  },
        {     -2.25065E+02,   -9.57225E+01,    1.92800E+01,    5.73300E+00  },
    },
    {   /* A3 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {     -4.62118E+01,    6.61732E+01,    1.37691E+01,   -5.96000E-01  },
        {     -4.62118E+01,    6.43179E+01,    1.55154E+01,   -4.00000E-01  },
        {     -9.84026E+01,    6.34647E+01,    1.63018E+01,   -3.02090E-01  },
        {     -1.36256E+02,    5.83322E+01,    1.84193E+01,    0.00000E+00  },
        {     -1.49784E+02,    5.16492E+01,    2.02175E+01,    3.16140E-01  },
        {     -1.72806E+02,    4.84760E+01,    2.09245E+01,    4.61430E-01  },
        {     -1.86658E+02,    3.36824E+01,    2.32690E+01,    1.09399E+00  },
        {     -1.85587E+02,    2.52989E+01,    2.41520E+01,    1.44928E+00  },
        {     -1.71989E+02,    1.36379E+01,    2.49675E+01,    1.96452E+00  },
        {     -1.82790E+02,    1.15643E+01,    2.50688E+01,    2.06507E+00  },
        {     -1.72404E+02,    2.38137E+00,    2.53023E+01,    2.48725E+00  },
        {     -1.65377E+02,   -8.37565E+00,    2.51776E+01,    3.01297E+00  },
        {     -1.55774E+02,   -1.88169E+01,    2.46054E+01,    3.53759E+00  },
        {     -1.57286E+02,   -2.88848E+01,    2.35777E+01,    4.05739E+00  },
        {     -1.23636E+02,   -3.84068E+01,    2.22198E+01,    4.52013E+00  },
        {     -1.23636E+02,   -4.01212E+01,    2.18568E+01,    4.62200E+00  }
    },
    {   /* A4 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {     -3.75339E+01,    6.65619E+01,    1.36998E+01,   -5.95840E-01  },
        {     -3.75339E+01,    6.50501E+01,    1.54668E+01,   -4.00000E-01  },
        {     -1.02482E+02,    6.43529E+01,    1.62680E+01,   -3.01750E-01  },
        {     -1.41507E+02,    5.90076E+01,    1.84128E+01,    0.00000E+00  },
        {     -1.51445E+02,    5.20670E+01,    2.02287E+01,    3.16190E-01  },
        {     -1.77333E+02,    4.88587E+01,    2.09414E+01,    4.61580E-01  },
        {     -1.90842E+02,    3.36776E+01,    2.32967E+01,    1.09483E+00  },
        {     -1.86048E+02,    2.51061E+01,    2.41768E+01,    1.45052E+00  },
        {     -1.88645E+02,    1.34162E+01,    2.49836E+01,    1.96619E+00  },
        {     -1.80150E+02,    1.11417E+01,    2.50823E+01,    2.06680E+00  },
        {     -1.73217E+02,    2.09153E+00,    2.53039E+01,    2.48910E+00  },
        {     -1.62834E+02,   -8.71624E+00,    2.51661E+01,    3.01472E+00  },
        {     -1.55955E+02,   -1.89969E+01,    2.45829E+01,    3.53897E+00  },
        {     -1.48327E+02,   -2.90765E+01,    2.35472E+01,    4.05820E+00  },
        {     -1.60575E+02,   -3.80561E+01,    2.21925E+01,    4.52032E+00  },
        {     -1.60575E+02,   -4.02812E+01,    2.18306E+01,    4.62200E+00  }
    }
};

///****************************************************************************************************
//* ENCODER TO FLAP ANGLE Spline Interpolation Polynomial Equation Constants.
//*
//* Coefficients a3, a2, a1, a0 used in the polynomials based on
//* actuators 1, 2, 3, 4 */
/* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * NEED TO REWORK THESE VALUES. HAVE NOT UPDATED THE TRANSFER FUNCTION SPREADSHEET FOR FLAP ANGLE YET
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 */
const float32_t af32EncoderToFlapAngleCoeffsTable[NUM_ACTUATOR_TYPES][NUM_POLYNOMIAL_EQUATIONS][NUM_POLYNOMIAL_COEFFS] =
{
    {   /* A1 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {     -2.55325E+02,    1.07738E+02,    2.04442E+01,   -5.15000E-01  },
        {     -2.55325E+02,    1.03195E+02,    2.16953E+01,   -3.90000E-01  },
        {     -3.17321E+02,    9.87264E+01,    2.28733E+01,   -2.59970E-01  },
        {     -3.77766E+02,    8.83755E+01,    2.49077E+01,    0.00000E+00  },
        {     -3.31731E+02,    6.56537E+01,    2.79959E+01,    5.31860E-01  },
        {     -4.33778E+02,    5.92028E+01,    2.88052E+01,    7.16000E-01  },
        {     -2.98416E+02,    3.14996E+01,    3.07361E+01,    1.35186E+00  },
        {     -3.66180E+02,    1.91233E+01,    3.14359E+01,    1.78200E+00  },
        {     -3.00924E+02,   -2.38272E+00,    3.17637E+01,    2.40200E+00  },
        {     -2.91960E+02,   -5.79360E+00,    3.17328E+01,    2.52196E+00  },
        {     -2.55788E+02,   -3.13714E+01,    3.06475E+01,    3.43642E+00  },
        {     -2.43355E+02,   -4.91544E+01,    2.87813E+01,    4.12662E+00  },
        {     -2.03067E+02,   -6.72597E+01,    2.58943E+01,    4.80644E+00  },
        {     -2.41337E+02,   -7.09066E+01,    2.50672E+01,    4.95900E+00  },
        {     -2.25065E+02,   -9.16666E+01,    2.04056E+01,    5.61378E+00  },
        {     -2.25065E+02,   -9.57225E+01,    1.92800E+01,    5.73300E+00  },
    },
    {   /* A2 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {     -2.55325E+02,    1.07738E+02,    2.04442E+01,   -5.15000E-01  },
        {     -2.55325E+02,    1.03195E+02,    2.16953E+01,   -3.90000E-01  },
        {     -3.17321E+02,    9.87264E+01,    2.28733E+01,   -2.59970E-01  },
        {     -3.77766E+02,    8.83755E+01,    2.49077E+01,    0.00000E+00  },
        {     -3.31731E+02,    6.56537E+01,    2.79959E+01,    5.31860E-01  },
        {     -4.33778E+02,    5.92028E+01,    2.88052E+01,    7.16000E-01  },
        {     -2.98416E+02,    3.14996E+01,    3.07361E+01,    1.35186E+00  },
        {     -3.66180E+02,    1.91233E+01,    3.14359E+01,    1.78200E+00  },
        {     -3.00924E+02,   -2.38272E+00,    3.17637E+01,    2.40200E+00  },
        {     -2.91960E+02,   -5.79360E+00,    3.17328E+01,    2.52196E+00  },
        {     -2.55788E+02,   -3.13714E+01,    3.06475E+01,    3.43642E+00  },
        {     -2.43355E+02,   -4.91544E+01,    2.87813E+01,    4.12662E+00  },
        {     -2.03067E+02,   -6.72597E+01,    2.58943E+01,    4.80644E+00  },
        {     -2.41337E+02,   -7.09066E+01,    2.50672E+01,    4.95900E+00  },
        {     -2.25065E+02,   -9.16666E+01,    2.04056E+01,    5.61378E+00  },
        {     -2.25065E+02,   -9.57225E+01,    1.92800E+01,    5.73300E+00  },
    },
    {   /* A3 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {     -4.62118E+01,    6.61732E+01,    1.37691E+01,   -5.96000E-01  },
        {     -4.62118E+01,    6.43179E+01,    1.55154E+01,   -4.00000E-01  },
        {     -9.84026E+01,    6.34647E+01,    1.63018E+01,   -3.02090E-01  },
        {     -1.36256E+02,    5.83322E+01,    1.84193E+01,    0.00000E+00  },
        {     -1.49784E+02,    5.16492E+01,    2.02175E+01,    3.16140E-01  },
        {     -1.72806E+02,    4.84760E+01,    2.09245E+01,    4.61430E-01  },
        {     -1.86658E+02,    3.36824E+01,    2.32690E+01,    1.09399E+00  },
        {     -1.85587E+02,    2.52989E+01,    2.41520E+01,    1.44928E+00  },
        {     -1.71989E+02,    1.36379E+01,    2.49675E+01,    1.96452E+00  },
        {     -1.82790E+02,    1.15643E+01,    2.50688E+01,    2.06507E+00  },
        {     -1.72404E+02,    2.38137E+00,    2.53023E+01,    2.48725E+00  },
        {     -1.65377E+02,   -8.37565E+00,    2.51776E+01,    3.01297E+00  },
        {     -1.55774E+02,   -1.88169E+01,    2.46054E+01,    3.53759E+00  },
        {     -1.57286E+02,   -2.88848E+01,    2.35777E+01,    4.05739E+00  },
        {     -1.23636E+02,   -3.84068E+01,    2.22198E+01,    4.52013E+00  },
        {     -1.23636E+02,   -4.01212E+01,    2.18568E+01,    4.62200E+00  }
    },
    {   /* A4 Coefficients */
        /*              a3,             a2,             a1,             a0  */
        {     -3.75339E+01,    6.65619E+01,    1.36998E+01,   -5.95840E-01  },
        {     -3.75339E+01,    6.50501E+01,    1.54668E+01,   -4.00000E-01  },
        {     -1.02482E+02,    6.43529E+01,    1.62680E+01,   -3.01750E-01  },
        {     -1.41507E+02,    5.90076E+01,    1.84128E+01,    0.00000E+00  },
        {     -1.51445E+02,    5.20670E+01,    2.02287E+01,    3.16190E-01  },
        {     -1.77333E+02,    4.88587E+01,    2.09414E+01,    4.61580E-01  },
        {     -1.90842E+02,    3.36776E+01,    2.32967E+01,    1.09483E+00  },
        {     -1.86048E+02,    2.51061E+01,    2.41768E+01,    1.45052E+00  },
        {     -1.88645E+02,    1.34162E+01,    2.49836E+01,    1.96619E+00  },
        {     -1.80150E+02,    1.11417E+01,    2.50823E+01,    2.06680E+00  },
        {     -1.73217E+02,    2.09153E+00,    2.53039E+01,    2.48910E+00  },
        {     -1.62834E+02,   -8.71624E+00,    2.51661E+01,    3.01472E+00  },
        {     -1.55955E+02,   -1.89969E+01,    2.45829E+01,    3.53897E+00  },
        {     -1.48327E+02,   -2.90765E+01,    2.35472E+01,    4.05820E+00  },
        {     -1.60575E+02,   -3.80561E+01,    2.21925E+01,    4.52032E+00  },
        {     -1.60575E+02,   -4.02812E+01,    2.18306E+01,    4.62200E+00  }
    }
};

/* Lower and Upper Limits use for selecting the Coefficients to use for both Flap Angle and Stroke equations */
const float32_t af32EncoderPolyBreaksTable[NUM_ACTUATOR_TYPES][NUM_POLYNOMIAL_EQUATIONS][NUM_LOWER_UPPER_LIMITS] =
{
    {   /* A1 Lower and Upper Limits */
        /*    Lower         Upper  */
        {  0.00000000,  0.00593142  },
        {  0.00593142,  0.01176533  },
        {  0.01176533,  0.02263852  },
        {  0.02263852,  0.04268775  },
        {  0.04268775,  0.04916983  },
        {  0.04916983,  0.07045818  },
        {  0.07045818,  0.08428258  },
        {  0.08428258,  0.10385950  },
        {  0.10385950,  0.10763772  },
        {  0.10763772,  0.13684007  },
        {  0.13684007,  0.16001430  },
        {  0.16001430,  0.18481379  },
        {  0.18481379,  0.19080020  },
        {  0.19080020,  0.21947379  },
        {  0.21947379,  0.22548077  },
        {  0.22548077,  0.23219128  }
    },
    {   /* A2 Lower and Upper Limits */
        /*    Lower         Upper  */
        {  0.00000000,  0.00593142  },
        {  0.00593142,  0.01176533  },
        {  0.01176533,  0.02263852  },
        {  0.02263852,  0.04268775  },
        {  0.04268775,  0.04916983  },
        {  0.04916983,  0.07045818  },
        {  0.07045818,  0.08428258  },
        {  0.08428258,  0.10385950  },
        {  0.10385950,  0.10763772  },
        {  0.10763772,  0.13684007  },
        {  0.13684007,  0.16001430  },
        {  0.16001430,  0.18481379  },
        {  0.18481379,  0.19080020  },
        {  0.19080020,  0.21947379  },
        {  0.21947379,  0.22548077  },
        {  0.22548077,  0.23219128  }
    },
    {   /* A3 Lower and Upper Limits */
        /*    Lower         Upper  */
        {  0.00000000,  0.01338214  },
        {  0.01338214,  0.01953636  },
        {  0.01953636,  0.03692242  },
        {  0.03692242,  0.05327172  },
        {  0.05327172,  0.06033331  },
        {  0.06033331,  0.08886931  },
        {  0.08886931,  0.10384061  },
        {  0.10384061,  0.12478494  },
        {  0.12478494,  0.12880381  },
        {  0.12880381,  0.14554956  },
        {  0.14554956,  0.16634769  },
        {  0.16634769,  0.18739300  },
        {  0.18739300,  0.20893672  },
        {  0.20893672,  0.22911661  },
        {  0.22911661,  0.23373875  },
        {  0.23373875,  0.24130389  }
    },
    {   /* A4 Lower and Upper Limits */
        /*    Lower         Upper  */
        {  0.00000000,  0.01342597  },
        {  0.01342597,  0.01961764  },
        {  0.01961764,  0.03700369  },
        {  0.03700369,  0.05335300  },
        {  0.05335300,  0.06041458  },
        {  0.06041458,  0.08895058  },
        {  0.08895058,  0.10392189  },
        {  0.10392189,  0.12486622  },
        {  0.12486622,  0.12888508  },
        {  0.12888508,  0.14563083  },
        {  0.14563083,  0.16642897  },
        {  0.16642897,  0.18747428  },
        {  0.18747428,  0.20901800  },
        {  0.20901800,  0.22919789  },
        {  0.22919789,  0.23381692  },
        {  0.23381692,  0.24272506  }
    }
};

/* CRC-8 Table with 0x97 Generator for IncOder data integrity check */
static const uint16_t u8Crc8Table[CRC8_TABLE_SIZE] =
{
    0x00U,  0x97U,  0xB9U,  0x2EU,  0xE5U,  0x72U,  0x5CU,  0xCBU,  0x5DU,  0xCAU,  0xE4U,  0x73U,  0xB8U,  0x2FU,  0x01U,
    0x96U,  0xBAU,  0x2DU,  0x03U,  0x94U,  0x5FU,  0xC8U,  0xE6U,  0x71U,  0xE7U,  0x70U,  0x5EU,  0xC9U,  0x02U,  0x95U,
    0xBBU,  0x2CU,  0xE3U,  0x74U,  0x5AU,  0xCDU,  0x06U,  0x91U,  0xBFU,  0x28U,  0xBEU,  0x29U,  0x07U,  0x90U,  0x5BU,
    0xCCU,  0xE2U,  0x75U,  0x59U,  0xCEU,  0xE0U,  0x77U,  0xBCU,  0x2BU,  0x05U,  0x92U,  0x04U,  0x93U,  0xBDU,  0x2AU,
    0xE1U,  0x76U,  0x58U,  0xCFU,  0x51U,  0xC6U,  0xE8U,  0x7FU,  0xB4U,  0x23U,  0x0DU,  0x9AU,  0x0CU,  0x9BU,  0xB5U,
    0x22U,  0xE9U,  0x7EU,  0x50U,  0xC7U,  0xEBU,  0x7CU,  0x52U,  0xC5U,  0x0EU,  0x99U,  0xB7U,  0x20U,  0xB6U,  0x21U,
    0x0FU,  0x98U,  0x53U,  0xC4U,  0xEAU,  0x7DU,  0xB2U,  0x25U,  0x0BU,  0x9CU,  0x57U,  0xC0U,  0xEEU,  0x79U,  0xEFU,
    0x78U,  0x56U,  0xC1U,  0x0AU,  0x9DU,  0xB3U,  0x24U,  0x08U,  0x9FU,  0xB1U,  0x26U,  0xEDU,  0x7AU,  0x54U,  0xC3U,
    0x55U,  0xC2U,  0xECU,  0x7BU,  0xB0U,  0x27U,  0x09U,  0x9EU,  0xA2U,  0x35U,  0x1BU,  0x8CU,  0x47U,  0xD0U,  0xFEU,
    0x69U,  0xFFU,  0x68U,  0x46U,  0xD1U,  0x1AU,  0x8DU,  0xA3U,  0x34U,  0x18U,  0x8FU,  0xA1U,  0x36U,  0xFDU,  0x6AU,
    0x44U,  0xD3U,  0x45U,  0xD2U,  0xFCU,  0x6BU,  0xA0U,  0x37U,  0x19U,  0x8EU,  0x41U,  0xD6U,  0xF8U,  0x6FU,  0xA4U,
    0x33U,  0x1DU,  0x8AU,  0x1CU,  0x8BU,  0xA5U,  0x32U,  0xF9U,  0x6EU,  0x40U,  0xD7U,  0xFBU,  0x6CU,  0x42U,  0xD5U,
    0x1EU,  0x89U,  0xA7U,  0x30U,  0xA6U,  0x31U,  0x1FU,  0x88U,  0x43U,  0xD4U,  0xFAU,  0x6DU,  0xF3U,  0x64U,  0x4AU,
    0xDDU,  0x16U,  0x81U,  0xAFU,  0x38U,  0xAEU,  0x39U,  0x17U,  0x80U,  0x4BU,  0xDCU,  0xF2U,  0x65U,  0x49U,  0xDEU,
    0xF0U,  0x67U,  0xACU,  0x3BU,  0x15U,  0x82U,  0x14U,  0x83U,  0xADU,  0x3AU,  0xF1U,  0x66U,  0x48U,  0xDFU,  0x10U,
    0x87U,  0xA9U,  0x3EU,  0xF5U,  0x62U,  0x4CU,  0xDBU,  0x4DU,  0xDAU,  0xF4U,  0x63U,  0xA8U,  0x3FU,  0x11U,  0x86U,
    0xAAU,  0x3DU,  0x13U,  0x84U,  0x4FU,  0xD8U,  0xF6U,  0x61U,  0xF7U,  0x60U,  0x4EU,  0xD9U,  0x12U,  0x85U,  0xABU,
    0xABU
};
/*      Local Variable Declarations
*/
static uint16_t u16NewData = 0U;

/*      Externs
 */

/*      Local Function Prototypes
*/
void Encoder_Reset(void);
bool_t Encoder_ReadData(uint32_t* const pu32Data);
bool_t Encoder_ValidateCrc8(void);
bool_t Encoder_CalcCrc8(const uint32_t u32Data);
bool_t Encoder_RunDataIntegrityChecks(void);
void Encoder_SetEncoderNewDataFlag(bool_t bNewData);
/*      Function Definitions
*/



/*
     Purpose:
        This routine is used to initialize the McBSP (Multichannel Buffered Serial Port) to send Serial Clock to
        Encoder and Receive Serial Data from Encoder

    Global Data Referenced:
        #McbspaRegs
        #GpioMuxRegs
        #GpioDataRegs

    return  void

    Preconditions and Assumptions:
        This function should be called only once, prior to scheduler loop.
*/
void Encoder_Init(void)
{
    EALLOW;

    /* Gpio22 as McBSP clock for Master */
    GpioCtrlRegs.GPAGMUX2.bit.GPIO22 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = GPIO_MUX_TYPE_2;      /* Set as MCLKXA (McBSP) clock */
    GpioCtrlRegs.GPADIR.bit.GPIO22 = GPIO_OUTPUT;           /* Set MCLKXA as an output */

    /* GPIO21 as McBSP RX DATA. */
    GpioCtrlRegs.GPAGMUX2.bit.GPIO21 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = GPIO_MUX_TYPE_2;      /* Set as MDRA */
    GpioCtrlRegs.GPADIR.bit.GPIO21 = GPIO_INPUT;            /* Set as an Input */
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = GPIO_ASYNC;          /* Set input as asynchronous */
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = GPIO_ENABLE_PULLUP;    /* Enable Pullup*/

    /* GPIO23 as McBSP FSXMA.
     * Can't do this with this HW version since GPIO23 is occupied by 270V_BUS_CNTL_CHA
     * and therefore requires a workaround for the FSXMA output. */

    /*GPIO53 as ENC_ZERO_SET_CNTL*/
    GpioCtrlRegs.GPBGMUX2.bit.GPIO53 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPBDIR.bit.GPIO53 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPBPUD.bit.GPIO53 = GPIO_DISABLE_PULLUP;   /* Disable the pullups */
    GpioCtrlRegs.GPBODR.bit.GPIO53 = GPIO_NORMAL_OUTPUT;    /* Configure as push-pull normal output */

    EDIS;

    /*Place Transmitter, Receiver & Sample Rate Generator in Reset*/
    McbspaRegs.SPCR2.bit.XRST = 0;                          /* Reset Transmitter */
    McbspaRegs.SPCR1.bit.RRST = 0;                          /* Reset Receiver */
    McbspaRegs.SPCR2.bit.GRST = 0;                          /* Reset Sample Generator */

    /*Programming McBSP registers for desired transmit operation*/
    McbspaRegs.SPCR1.bit.DLB = 0;                           /* shall disable digital loopback mode */
    McbspaRegs.MCR2.bit.XMCM = 0;                           /* Disbale Multi-channel selection mode */
    McbspaRegs.XCR2.bit.XPHASE = 0;                         /* Set to single phase frame */
    McbspaRegs.XCR1.bit.XWDLEN1 = 5;                        /* Set transmit Word Length to 32-bit */
    McbspaRegs.XCR1.bit.XFRLEN1 = 0;                        /* Set transmit Frame length to 1 */
    McbspaRegs.XCR2.bit.XCOMPAND = 0;                       /* Disable Transmit Companding mode */

    /*Programming McBSP registers for desired receive operation*/
    McbspaRegs.MCR1.bit.RMCM = 0;                           /* Disable Multi-channel selection mode */
    McbspaRegs.RCR2.bit.RPHASE = 0;                         /* Set to single phase frame */
    McbspaRegs.RCR1.bit.RWDLEN1 = 5;                        /* Set Receive Word Length to 32-bit */
    McbspaRegs.RCR1.bit.RFRLEN1 = 0;                        /* Set Receive Frame length to 1 */
    McbspaRegs.RCR2.bit.RCOMPAND = 0;                       /* Disable Receive Companding mode */
    McbspaRegs.SPCR1.bit.RJUST = 0;                         /* Set Receive sign extension to Right Justification, zeros fill msb */

    /* Program Register for SPI operation:  Program McBSP as SPI Master*/
    McbspaRegs.SPCR1.bit.CLKSTP = 2;                        /* Enable The clock stop mode without delay */
    McbspaRegs.PCR.bit.CLKXP = 1;                           /* High-Inactive state. Transmit data on falling edge of clock */
    McbspaRegs.PCR.bit.CLKRP = 0;                           /* Receive clock polarity - Receive data on falling edge of clock */
    McbspaRegs.PCR.bit.CLKXM = 1;                           /* Transmit clock mode - Set CLKX as an output signal (McBSP as Master) */
    McbspaRegs.PCR.bit.SCLKME = 0;                          /* Source of SRG - Clock generated by SRG is derived from CPU clock */
    McbspaRegs.SRGR2.bit.CLKSM = 1;

    /*Set the Sample Rate Generator divide down value*/
    /*Low speed clock is set 100MHz, Encoder IC support 100kHz to 2MHz
     * Divide by 100 to get 1MHz*/
    McbspaRegs.SRGR1.bit.CLKGDV = SRGR1_CLK_DIV_BY_100;

    /* Set transmit frame sync mode */
    /* FSXMA GPIO23 is occupied by the 270V_BUS_CNTL_CHA and therefore cannot set FSXM to true here */
    //McbspaRegs.PCR.bit.FSXM = 1;                          /* Transmit frame synchronization is supplied by Mcbsp */
    McbspaRegs.PCR.bit.FSXM = 0;                          /* Transmit frame synchronization is supplied by Mcbsp */
    McbspaRegs.SRGR2.bit.FSGM = 0;                        /* Mcbsp generates a Transmitter Frame Synchronization pulse when content of DXR are copied to XSR */
    McbspaRegs.PCR.bit.FSXP = 1;                          /* Transmit frame Polarity set to Active low */

    McbspaRegs.XCR2.bit.XDATDLY = 1;                /*Transmit data delay - One bit delay. 0 or 2 is not defined for SPI master operation*/
    McbspaRegs.RCR2.bit.RDATDLY = 1;                /*Receive data delay - One bit delay. 0 or 2 is not defined for SPI master operation*/

    /*Enable the sample rate generator*/
    /*Release sample rate generator from reset*/
    McbspaRegs.SPCR2.bit.GRST = 1;

    /* Wait for sample rate generator to stabilize*/
    clkg_delay_loop();

    /*Enable the transmitter and Receiver*/
    McbspaRegs.SPCR2.bit.XRST = 1;                  /* Release the Transmitter*/
    McbspaRegs.SPCR1.bit.RRST = 1;                  /* Release the receiver*/
    /*Enable Frame Synchronization Logic*/
    McbspaRegs.SPCR2.bit.FRST = 1;

    /* Clear the ZERO set point of the IncOder since requires ability to control the 28VDC output to the sensor
     * to set the ZERO position. Need to have ENC_ZERO_SET_CNTL_set() for a period of 5 seconds when sensor is first
     * powered by 28VDC to set the zero position  */
    ENC_ZERO_SET_CNTL_clr();

    /* Routine to read in first sensor reading such that Stroke and Flap Angle Calculations will have
     * position data to work with. */
    /* Transmit Dummy data onto SPI */
    McbspaRegs.DXR1.all = 0x55;
    McbspaRegs.DXR2.all = 0x55;

    /* Set the Frame Sychronization logic Reset to true since configured as an SPI Master via McBsp (per datasheet) */
    McbspaRegs.SPCR2.bit.FRST = 1U;

}

/*
     Purpose:
        This routine is used to reset the McBsp peripheral

    Global Data Referenced:
        #McbspaRegs

    return  void

    Preconditions and Assumptions:
        This function should be called prior to every McBsp SPI Master data transmit
        in order to reset the frame synchronization since the FSXM output cannot
        be used with this hardware design.
*/
void Encoder_Reset(void)
{
    /* Place Transmitter, Receiver & Sample Rate Generator in Reset */
    McbspaRegs.SPCR2.bit.XRST = 0;  /* Reset Transmitter */
    McbspaRegs.SPCR1.bit.RRST = 0;  /* Reset Receiver */
    McbspaRegs.SPCR2.bit.GRST = 0;  /* Reset Sample Generator */

    asm(" NOP"); /* statement between reset of Sample Generator and Releasing it from Reset */

    /* Release sample rate generator from reset */
    McbspaRegs.SPCR2.bit.GRST = 1;

//    /* Wait for two sample rate generator clock periods of Mcbsp to stabilize */
//    clkg_delay_loop();
//
//    /* Enable the transmitter and Receiver */
//    McbspaRegs.SPCR2.bit.XRST = 1;      /* Release the Transmitter */
//    McbspaRegs.SPCR1.bit.RRST = 1;      /* Release the receiver */
//    McbspaRegs.SPCR2.bit.FRST = 1;      /* Enable Frame Synchronization Logic */
}

/*
     Purpose:
        This routine is used to send Serial Clock to Encoder.

    Global Data Referenced:
        #McbspbRegs

    return  void

    Preconditions and Assumptions:
        This function should be called every time before reading the data from Encoder.
        Minimum 53 Micro Seconds time should be elapsed between two successive function calls.
        As frequency of McBSP clock is 1Mz, time required for 32 clock cycles is 32 micro seconds and
        21 micro seconds is time required for new data to be ready
*/
void Encoder_Tx(void)
{
    uint16_t DummyTxData1 = 0X55;
    uint16_t DummyTxData2 = 0x55;

    /* Re-Enable the transmitter and Receiver */
    McbspaRegs.SPCR2.bit.XRST = 1;      /* Release the Transmitter */
    McbspaRegs.SPCR1.bit.RRST = 1;      /* Release the receiver */
    McbspaRegs.SPCR2.bit.FRST = 1;      /* Enable Frame Synchronization Logic */

    asm(" NOP"); /* statement between releasing peripheral and then transmitting data */

//    Encoder_Reset();

    /*Encoder needs 32 falling edges of clock. In the Initialization routine the word length is set to 32.
    In order to send the Serial Clock to Encoder on MCLKXA (GPIO22), dummy data needs to be sent on MDXA,
    but as no GPIO is configured as MDXA there will be no output data.*/
    McbspaRegs.DXR1.all = DummyTxData1;
    McbspaRegs.DXR2.all = DummyTxData2;

    /* Set the Frame Sychronization logic Reset to true since configured as an SPI Master via McBsp (per datasheet) */
    McbspaRegs.SPCR2.bit.FRST = 1U;
}

/*
     Purpose:
        This routine is used to receive Serial data from Encoder.

    Global Data Referenced:
        #McbspbRegs
        ICC_xData.tData.tIcc_3.u16NewData
        ICC_xData.tData.tIcc_2.u32IncoderInputsRaw
        tIncoderInputs.tRaw.u32Data
        tIncoderInputs.tData.u32Data
        tIncoderInputs.u32Position

    return  void

    Preconditions and Assumptions:
        This function should be called every time data is to be read from Encoder.
        Before calling this function, Encoder_Tx() should be called.

        Assumes Encoder_RunDataIntegrityChecks function updates tIncoderInputs.tData.u32Data
*/
void Encoder_Rx(void)
{
    bool_t bNewData = false;
    bool_t bDataValid = false;
    uint32_t u32Data = 0U;

    /* Reading the sensor is different depending upon the configuration of the software for the various
     * rigs that are supported:
     * VCR2:  Normal configuration, encoder per MCU channel
     * SITS:  Abnormal configuration, single encoder per actuator. */
#if !defined(__SKEW_SNSR_ENCODER_1PER_ACTUATOR__)
    /* Normal configuration has one Encoder per channel, read the data */
    bNewData = Encoder_ReadData(&u32Data);
#else
    /* SITS Rig configuration has a single IncOder per actuator as a limitation of mechanical setup.
     * Channel B will get the IncOder positional information directly from Channel A which is
     * transmitted over the ICC bus. */
    if(G_eChId == CHANNEL_B)
    {
        /* Channel B read IncOder data from Channel A ICC data */
        /* Read the bNewData flag from Channel A */
        bNewData = (bool_t)ICC_xData.tData.tIcc_3.u16IncoderNewData;
        /* Read raw positional data from Channel A */
        u32Data = ICC_xData.tData.tIcc_2.u32IncoderInputsRaw;
    }
    else
    {
        /* Channel A read the IncOder data */
        bNewData = Encoder_ReadData(&u32Data);
    }
#endif
    /* Save off the new data flag to send over ICC */
    Encoder_SetEncoderNewDataFlag(bNewData);

    /* If there is new data read from the sensor then run the integrity checks prior to using data */
    if(bNewData == true)
    {
        /* Service the RX Not Ready Error Counter */
        if((u16IncoderRxNotReadyCntr != 0U) && (u16IncoderRxNotReadyCntr < tPdi.u16IncoderRxReadyChkThreshold))
        {
            /* Decrement RX Not Ready counter and avoid rolling past zero */
            if(tPdi.u16IncoderRxReadyChkDecCount > u16IncoderRxNotReadyCntr)
            {
                /* Set count to zero to avoid rolling past zero. */
                u16IncoderRxNotReadyCntr = 0U;
            }
            else
            {
                /* Decrement RX Not Ready counter by configured count for every successful check when the counter is positive. */
                u16IncoderRxNotReadyCntr = u16IncoderRxNotReadyCntr - tPdi.u16IncoderRxReadyChkDecCount;
            }
        }

        /* Save the raw data */
        tIncoderInputs.tRaw.u32Data = u32Data;

        /* Run the integrity checks on the IncOder data */
        bDataValid = Encoder_RunDataIntegrityChecks();
        if(bDataValid == true)
        {
            /* All checks passed. Store the 17-bit (22-bit field) positional data which will be used
             * to calculate stroke and angle later */
            tIncoderInputs.u32Position = (uint32_t)tIncoderInputs.tData.bits.PD;

#if defined(__SKEW_SNSR_ENCODER_SITS_RIG__)
            /* take the 2's complement to format the counting to increase while extending due to the mounting
             * being opposite of what it should be. */
            tIncoderInputs.u32Position = (~tIncoderInputs.u32Position) + 1U;
#endif
        }
    }
    else
    {
        /* No new data from the sensor. */
        /* Incoder status is not ready, increment the associated error counter */
        u16IncoderRxNotReadyCntr += tPdi.u16IncoderRxReadyChkIncCount;;
        if(u16IncoderRxNotReadyCntr > tPdi.u16IncoderRxReadyChkThreshold)
        {
            u16IncoderRxNotReadyCntr = tPdi.u16IncoderRxReadyChkThreshold;
        }
    }

    /* Reset the IncOder to get ready for next Ready Cycle. Required b/c of incompatible
     * interface between SSI IncOder and McBSP SPI Master Mode */
    Encoder_Reset();
}

/*
     Purpose:
        This routine reads the raw data from the IncOder.

    Global Data Referenced:
        #McbspbRegs

    return  bool_t true:  new data read; false = McBSP was not ready to read

    Preconditions and Assumptions:
*/
bool_t Encoder_ReadData(uint32_t* const pu32Data)
{
    bool_t bNewData = false;

    /* Check if the McBSP is ready with new Incoder data to process */
    if ( McbspaRegs.SPCR1.bit.RRDY == 1 )               /* Read data only if receiver is ready */
    {
        /* McBSP SPI has received new data. Store data in temps and then conduct integrity checks */
        *pu32Data = McbspaRegs.DRR2.all;      /* Read DRR2 before reading DRR1 */
        *pu32Data = (*pu32Data << 16U);       /* Shift word 1 into place */
        *pu32Data |= McbspaRegs.DRR1.all;     /* RRDY is cleared once the data is read from DRR1 */

        bNewData = true;    /* New data was read in from the sensor, flag as new */
    }

    return bNewData;
}

/*
     Purpose:
        This routine runs data integrity checks on the data read from the IncOder.

        CRC8 integrity check
        Position Validity check from the IncOder

    Global Data Referenced:
        #McbspbRegs
        tIncoderInputs.tRaw.u32Data

    return  bool_t true:  Integrity Checks Passed; false = Integrity Checks Failed

    Preconditions and Assumptions:
    tIncoderInputs.tRaw.u32Data has been updated prior to call.
    tIncoderInputs.tData.u32Data has been updated in Encoder_ValidateCrc8 function call
*/
bool_t Encoder_RunDataIntegrityChecks(void)
{
    bool_t bChecksPassed = false; /* Init the integrity checks to failed */

    /* Conduct the CRC8 Integrity Check on the data */
    bChecksPassed = Encoder_ValidateCrc8();
    if(bChecksPassed == true)
    {
        /* CRC8 check passed */
        /* Service the CRC Error Counter */
        if((u16IncoderCrc8CheckFailedCntr != 0U) && (u16IncoderCrc8CheckFailedCntr < tPdi.u16IncoderCrcChkThreshold))
        {
            /* Decrement CRC counter and avoid rolling past zero */
            if(tPdi.u16IncoderCrcChkDecCount > u16IncoderCrc8CheckFailedCntr)
            {
                /* Set count to zero to avoid rolling past zero. */
                u16IncoderCrc8CheckFailedCntr = 0U;
            }
            else
            {
                /* Decrement CRC counter by configured count for every successful check when the counter is positive. */
                u16IncoderCrc8CheckFailedCntr = u16IncoderCrc8CheckFailedCntr - tPdi.u16IncoderCrcChkDecCount;
            }
        }

        /* Verify the position is valid according to the IncOder sensor */
        if(tIncoderInputs.tData.bits.PV == true)
        {
            /* CRC8 is valid and Position is Valid. */

            /* Service the Incoder Position Not Valid Error Counter */
            if((u16IncoderPositionNotValidCntr != 0U) && (u16IncoderPositionNotValidCntr < tPdi.u16IncoderPvInvalidChkThreshold))
            {
                /* Decrement Position Invalid counter and avoid rolling past zero. */
                if(tPdi.u16IncoderPvInvalidChkDecCount > u16IncoderPositionNotValidCntr)
                {
                    /* Set count to zero to avoid rolling past zero. */
                    u16IncoderPositionNotValidCntr = 0U;
                }
                else
                {
                    /* Decrement Position Invalid counter by configured count for every successful check when the counter is positive. */
                    u16IncoderPositionNotValidCntr = u16IncoderPositionNotValidCntr - tPdi.u16IncoderPvInvalidChkDecCount;
                }
            }

            /* Flag the check as passed */
            bChecksPassed = true;
        }
        else
        {
            /* Failed the Position Valid check */
            /* Position is invalid, increment the Not Valid counter for the monitor */
            u16IncoderPositionNotValidCntr += tPdi.u16IncoderPvInvalidChkIncCount;
            if(u16IncoderPositionNotValidCntr > tPdi.u16IncoderPvInvalidChkThreshold)
            {
                u16IncoderPositionNotValidCntr = tPdi.u16IncoderPvInvalidChkThreshold;
            }
            /* Flag the check as failed */
            bChecksPassed = false;
        }
    }
    else
    {
        /* Invalid CRC, Increment CRC fail counter for the monitor */
        u16IncoderCrc8CheckFailedCntr += tPdi.u16IncoderCrcChkIncCount;
        if(u16IncoderCrc8CheckFailedCntr > tPdi.u16IncoderCrcChkThreshold)
        {
            u16IncoderCrc8CheckFailedCntr = tPdi.u16IncoderCrcChkThreshold;
        }
        /* Flag the check as failed */
        bChecksPassed = false;
    }

    return bChecksPassed;
}

/*
     Purpose:
        Validate the CRC8 from the IncOder against the CRC8 calculated from the data.

    Global Data Referenced:
    tIncoderInputs.tRaw.u32Data

    return  bool_t true:  CRC8 Validated, false:  CRC8 Invalid

    Preconditions and Assumptions:
    tIncoderInputs.tRaw.u32Data has been updated prior to call.
*/
bool_t Encoder_ValidateCrc8(void)
{
    bool_t bRet = false;   /* Initialize the validation check to false */

    /* SPI / McBSP is incompatible with the SSI version of the incOder sensor. The bits that are read in
     * are off by 1 bit value where the first bit is read as the incative high state rather than bit 31 of
     * the sensor data. Therefore the data needs to be shifted left by 1 and the lsb is unknown. Set the lsb
     * to both 0 and 1 and run the crc check. If a match is found then that is the value of the lsb. This is
     * a band-aid until a version of the sensor is acquired that communicates properly with one of the avail
     * peripherals on this DSP
     */
    /* Copy the raw data to working variable */
    tIncoderInputs.tData.u32Data = tIncoderInputs.tRaw.u32Data;

    /* Calculate the CRC of the Incoder Data */
    bRet = Encoder_CalcCrc8(tIncoderInputs.tData.u32Data);

    /* If CRC check hasn't passed then try scenario that is bit shifted by 1 */
    if(bRet != true)
    {
        /* Copy the raw data to working variable */
        tIncoderInputs.tData.u32Data = tIncoderInputs.tRaw.u32Data;

        /* Shift the data by 1 to position the bits correctly. lsb becomes 0 */
        tIncoderInputs.tData.u32Data <<= 1U;
        bRet = Encoder_CalcCrc8(tIncoderInputs.tData.u32Data);
        if(bRet != true)
        {
            /* CRC check failed, check the other case */
            /* Calculate the CRC of the Incoder Data with bit 0 a value of 1 */
            tIncoderInputs.tData.u32Data |= 0x00000001;
            bRet = Encoder_CalcCrc8(tIncoderInputs.tData.u32Data);
        }
    }

    return bRet;
}

/*
     Purpose:
        Calculate the CRC8 on the IncOder data and validate it against the CRC8 embedded in the data.

    Global Data Referenced:

    return  bool_t true:  CRC8 Validated, false:  CRC8 Invalid

    Preconditions and Assumptions:
*/
bool_t Encoder_CalcCrc8(const uint32_t u32Data)
{
    bool_t bRet = false;
    tIncoderInputsRaw_t tIncoderData = { u32Data };
    uint16_t u16Crc8 = 0U;  /* Init CRC to 0 */
    uint16_t u16Index = 0U; /* Index to lookup table */

    /* XOR in the first byte */
    u16Index = (tIncoderData.tu8.u8_2 ^ u16Crc8) & CRC8_BYTE_MASK;
    /* Get the current CRC value via the lookup table */
    u16Crc8 = u8Crc8Table[u16Index];

    /* XOR in the second byte */
    u16Index = (tIncoderData.tu8.u8_1 ^ u16Crc8) & CRC8_BYTE_MASK;
    /* Get the current CRC value via the lookup table */
    u16Crc8 = u8Crc8Table[u16Index];

    /* XOR in the 3rd and final byte */
    u16Index = (tIncoderData.tu8.u8_0 ^ u16Crc8) & CRC8_BYTE_MASK;
    /* Get the current CRC value via the lookup table */
    u16Crc8 = u8Crc8Table[u16Index];

    u16Crc8 ^= CRC8_XOR_FINAL;  /* do the final XOR */
    u16Crc8 &= CRC8_BYTE_MASK;  /* Ensure only 8-bits of data */

    /* Validate the calculated CRC to the provided CRC read from the IncOder */
    if(u16Crc8 == tIncoderData.bits.CRC)
    {
        /* CRCs match, return true for validation check */
        bRet = true;
    }

    return bRet;
}

void Encoder_SetEncoderNewDataFlag(bool_t bNewData)
{
    u16NewData = (uint16_t)bNewData;
}

uint16_t Encoder_GetEncoderNewDataFlag(void)
{
    return u16NewData;
}

/****************************************************************************************************
*  Function: Encoder_CalibrateSensor
*  Purpose:  Placeholder function for calibration routine of Encoder skew sensor for 2 reasons:
*  1.  Common logic with RVDT Skew Sensor
*  2.  In case there is a need to calibrate the sensor down the road.
*
*  Global Inputs:
*  Global Outputs:
*  Input: none
*  Output: false = incomplete, true = calibration complete
****************************************************************************************************/
bool_t Encoder_CalibrateSensor(void)
{
    bool_t bRet = true;

    return bRet;
}

/****************************************************************************************************
*  Function: Encoder_ApplyOffset
*  Purpose:
*  This function will apply the offset as saved off during the rigging process.
*
*  Global Inputs:
*  Global Outputs:
*  Input: u32PositionRaw:  Position of 17-bit Encoder
*  Output: u32Position:  Offset applied position of 17-bit Encoder
****************************************************************************************************/
uint32_t Encoder_ApplyOffset(uint32_t u32PositionRaw)
{
    uint32_t u32Position = u32PositionRaw;

    /* Adjust by the offset as saved during the rigging process */
    u32Position += Nvm_Rigging_Temp.tSkewSnsr.tEncoderCalibration.u32Offset;

    return u32Position;
}

/****************************************************************************************************
*  Function: Encoder_ApplyCalibrationEquation
*  Purpose:
*  This function will apply the calibration equation to the scaled position value. Placeholder for
*  now as may be needed in future.
*
*  Global Inputs:
*  Global Outputs:
*  Input: f32PositionScaled0to1:  Scaled representation of 17-bit Encoder position
*  Output: f32RvdtCalVal:  Calibrated and scaled representation of 17-bit Encoder position
****************************************************************************************************/
float32_t Encoder_ApplyCalibrationEquation(float32_t f32PositionScaled0to1)
{
    float32_t f32PosVal = f32PositionScaled0to1;

    return f32PosVal;
}

/****************************************************************************************************
*  Function: Encoder_GetScaledPositionFlapAngle
*  Purpose:
*  This function will calculate and save the scaled position based on the encoder positional info
*  for the Stroke Transfer Function Calculations.
*  Encoder.
*
*  Global Inputs:
*  Global Outputs:
*  Input:
*  Output:
****************************************************************************************************/
void Encoder_GetScaledPositionFlapAngle(tEncoderCalcsFlapAngle_t* const ptEncoderCalcs)
{
    /* NULL check the pointer */
    if(ptEncoderCalcs != NULL)
    {
        /* Get the latest position data from the sensor and store a snapshot for debugging if needed */
        ptEncoderCalcs->u32RawPosition = tIncoderInputs.u32Position;

        /* Apply the offset as saved off during the rigging process */
        ptEncoderCalcs->u32Position = Encoder_ApplyOffset(ptEncoderCalcs->u32RawPosition);

        /* Convert the 17-bit positional data into a fraction between 0 and 1 to represent the position */
        ptEncoderCalcs->f32ScaledPosition = ((float32_t)ptEncoderCalcs->u32Position * ENCODER_CNT2FRAC);

        /* Apply the calibration equation */
        ptEncoderCalcs->f32ScaledPositionCalibrated = ptEncoderCalcs->f32ScaledPosition;
    }

    return;
}

/****************************************************************************************************
*  Function: Encoder_GetScaledPositionStroke
*  Purpose:
*  This function will calculate and save the scaled position based on the encoder positional info
*  for the Stroke Transfer Function Calculations.
*  Encoder.
*
*  Global Inputs:
*  Global Outputs:
*  Input:
*  Output:
****************************************************************************************************/
void Encoder_GetScaledPositionStroke(tEncoderCalcsStroke_t* const ptEncoderCalcs)
{
    /* NULL check the pointer */
    if(ptEncoderCalcs != NULL)
    {
        /* Get the latest position data from the sensor and store a snapshot for debugging if needed */
        ptEncoderCalcs->u32RawPosition = tIncoderInputs.u32Position;

        /* Apply the offset as saved off during the rigging process */
        ptEncoderCalcs->u32Position = Encoder_ApplyOffset(ptEncoderCalcs->u32RawPosition);

        /* Convert the 17-bit positional data into a fraction between 0 and 1 to represent the position */
        ptEncoderCalcs->f32ScaledPosition = ((float32_t)ptEncoderCalcs->u32Position * ENCODER_CNT2FRAC);

        /* Apply the calibration equation */
        ptEncoderCalcs->f32ScaledPositionCalibrated = ptEncoderCalcs->f32ScaledPosition;
    }

    return;
}

///****************************************************************************************************
//*  Function: Encoder_GetFlapAngle
//*  Purpose:
//*  This function will calculate and save off the Flap Angle based on the position feedback from the
//*  Encoder.
//*
//*  Global Inputs:
//*  Global Outputs:
//*  Input:
//*  Output:
//****************************************************************************************************/
//void Encoder_GetFlapAngle(void)
//{
//
//    tEncoderCalcs.tStroke.u32RawPosition = tIncoderInputs.u32Position;
//
//    /* Apply the gain and offset convert to floating point for the RVDT Calculation Function */
//    //tEncoderCalcs.tFlapAngle.f32RvdtAdcAvgCalibrated = Rvdt_ApplyCalibrationEquation(tRvdtCalcs.tFlapAngle.s16RvdtAdcAvgRaw);
//
//    /* Calculate the Flap Angle Based on Position count */
//    tEncoderCalcs.tFlapAngle.f32FlapAngle = Encoder_CalcFlapAngleFromIncoder(G_eActuatorNumber, tEncoderCalcs.tFlapAngle.u32RawPosition);
//
//    return;
//}
//
///****************************************************************************************************
//*  Function: Encoder_GetStroke
//*  Purpose:
//*  This function will calculate and save off the Stroke based on the position feedback from the
//*  Encoder.
//*
//*  Global Inputs:
//*  Global Outputs:
//*  Input:
//*  Output:
//****************************************************************************************************/
//void Encoder_GetStroke(void)
//{
//    /* Get the latest position data from the sensor and store a snapshot for debugging if needed */
//    tEncoderCalcs.tStroke.u32RawPosition = tIncoderInputs.u32Position;
//    /* Convert the 17-bit positional data into a fraction between 0 and 1 to represent the position */
//    tEncoderCalcs.tStroke.f32ScaledPosition = ((float32_t)tEncoderCalcs.tStroke.u32RawPosition * INCODER_CNT2FRAC);
//
//    /* Apply the gain and offset convert to floating point for the RVDT Calculation Function */
//    //tEncoderCalcs.tStroke.f32RvdtAdcAvgCalibrated = Rvdt_ApplyCalibrationEquation(tRvdtCalcs.tFlapAngle.s16RvdtAdcAvgRaw);
//
//    /* Calculate the Flap Angle Based on Position count */
//    tEncoderCalcs.tStroke.f32Stroke = Encoder_CalcStrokeFromIncoder(G_eActuatorNumber, (float32_t)tEncoderCalcs.tStroke.f32ScaledPosition);
//
//    /* Calculate the quad count associated with the stroke */
//    tEncoderCalcs.tStroke.s16QuadCnt = Panel_ConvertStrokeToElectricalCycleCount(tEncoderCalcs.tStroke.f32Stroke);
//
//    return;
//}

/****************************************************************************************************
*  Function: Encoder_SetNvmRigData
*  Purpose:
*  This function saves the positional data associated with the rig position input for the Encoder as
*  the skew sensor.
*
*  Global Inputs: tIncoderInputs
*  Global Outputs: Nvm_Rigging_Temp
*  Input: ePosition:  Rig Position to save data for
*  Output: None
****************************************************************************************************/
void Encoder_SetNvmRigData(RIG_POSITIONS_T ePosition)
{
    Nvm_Rigging_Temp.tSkewSnsr.tEncoderPosData[ePosition].u32Position = tIncoderInputs.u32Position;
    Nvm_Rigging_Temp.tSkewSnsr.tEncoderPosData[ePosition].u32PositionData = tIncoderInputs.tData.u32Data;
}

/****************************************************************************************************
*  Function: Encoder_SetNvmPanelData
*  Purpose:
*  This function saves the stroke and flap angle calculation data as part of the Panel data struct
*  in NVM
*
*  Global Inputs: tIncoderInputs
*  Global Outputs: Nvm_Rigging_Temp
*  Input: ePosition:  Rig Position to save data for
*  Output: None
****************************************************************************************************/
void Encoder_SetNvmPanelData(RIG_POSITIONS_T ePosition)
{
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tFlapAngle.f32FlapAngle = tSkewSnsrCalcs.tFlapAngle.f32FlapAngle;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tFlapAngle.f32ScaledPosition = tSkewSnsrCalcs.tFlapAngle.f32ScaledPosition;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tFlapAngle.f32ScaledPositionCalibrated = tSkewSnsrCalcs.tFlapAngle.f32ScaledPositionCalibrated;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tFlapAngle.s16QuadCnt = tSkewSnsrCalcs.tFlapAngle.s16QuadCnt;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tFlapAngle.u32RawPosition = tSkewSnsrCalcs.tFlapAngle.u32RawPosition;

    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tStroke.f32Stroke = tSkewSnsrCalcs.tStroke.f32Stroke;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tStroke.f32ScaledPosition = tSkewSnsrCalcs.tStroke.f32ScaledPosition;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tStroke.f32ScaledPositionCalibrated = tSkewSnsrCalcs.tStroke.f32ScaledPositionCalibrated;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tStroke.s16QuadCnt = tSkewSnsrCalcs.tStroke.s16QuadCnt;
    Nvm_Rigging_Temp.tPanel[ePosition].tSkewSnsrCalcs.tStroke.u32RawPosition = tSkewSnsrCalcs.tStroke.u32RawPosition;
}
/* end encoder.c*/

#endif
