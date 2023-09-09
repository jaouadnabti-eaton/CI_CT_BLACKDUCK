/****************************************************************************************************
*  File name: adc.h
*
*  Purpose: Interface file for Analog/Digital Converter Driver
*      This file describes the Public Interface for the ADC driver.  The interface provides
*      routines necessary to initialize the ADC hardware, and to capture and calibrate the
*      readings.
*
*  Copyright Notice:
*  All source code and data contained in this file is Proprietary and
*  Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
*  disclosed; in whole or in part, without the express written permission of
*  Eaton Aerospace.
*
*  Copyright (c) 2022 - Eaton Aerospace Group, All Rights Reserved.
*
*  Author            Date       CR#          Description
*  ------           ---------   ------     ---------------------------------------
*  Adam Bouwens     12/07/2022  N/A        Ported to MCU
*
****************************************************************************************************/

#ifndef ADC_H__
#define ADC_H__

/*      Include Files */
#include "F2837xD_device.h"
#include "hsm.h"

/**
  defgroup adc Analog Inputs
  
  brief The Analog Inputs Driver is an abstraction for capturing readings from analog input signals.

    The ADC driver provides facilities for initializing the ADC hardware, capturing readings
    from analog input signals, and calibrating those readings based on two well-known inputs.
  
    Calibration is performed by feeding two known reference values into two ADC channels and
    calculating a calibration gain and offset to compensate for the input readings from the other
    channels.  This is possible because the channel-to-channel errors are small.  The achievable
    accuracy using calibration is largely dependent on the accuracy of the known references fed
    into the ADC.  The best possible accuracy achievable is limited by the channel-to-channel gain
    and offset errors of the ADC. 
  
    The calibration process involves four basic steps:
    -# Read the known reference values input channels
    -# Calculate the calibration gain
    -# Calculate the calibration offset
    -# Cycle through all remaining channels applying the calibration

    After the readings have been calibrated, they are available for direct use by drivers,  Additionally,
    the ADC driver provides a facility for averaging a series of ADC samples over time, to reduce the
    effect of signal noise.  The algorithm used is a running average over the most recent samples, so
    that the average is updated after every new sample.
    
*/
#define ADC_CHANNELS            16
#define ADC_CURRENT_CHANNELS    3
#define ADC_MA_WINDOW_LENGTH    400
#define ADC_MA_Multiplier       0.0025F /*Window length of MA filter is 400. 0.0025 = 1/400*/

/*      Public Type Definitions */

/* Structure for individual names of ADC Input Signals */

/* Structure for individual names of ADC Input Signals */
typedef struct
{
    uint16_t    u16_I_PHA_CHA;                /*   ADCIND0  I_PHA_CHx  */
    uint16_t    u16_I_PHB_CHA;                /*   ADCIND1  I_PHB_CHx  */
    uint16_t    u16_I_PHC_CHA;                /*   ADCIND2  I_PHC_CHx  */
    uint16_t    u16_MOTOR_PHASEx_I;         /*Motor phase peak current among windings*/
    uint16_t    u16_5V_SENSE;                 /*   ADCINC2  5v SENSE   */
    uint16_t    u16_POSITIVE_15V_SENSE;       /*   ADCINC3  +15v SENSE */
    uint16_t    u16_NEGATIVE_15V_SENSE;       /*   ADCINC4 -15v SENSE  */
    uint16_t    u16_MOTOR_TEMP_PHA;           /*   ADCIND0  I_PHA_CHx*/
    uint16_t    u16_MOTOR_TEMP_PHB;           /*   ADCIND1  I_PHB_CHx*/
    uint16_t    u16_MOTOR_TEMP_PHC;           /*   ADCIND2  I_PHC_CHx*/
    uint16_t    u16_RVDT_POS;                 /*   ADCINA1  P_POS */
    uint16_t    u16_RVDT_SUM;                 /*   ADCINA0  P_SUM */
    uint16_t    u16_DC_BUS_VSENSE;            /*   SPI ADC-DC Bus Sense 270V(HL Mode) or 28V (VC Mode)*/
    uint16_t    u16_28VBUS_VSENSE;
    uint16_t    u16_IBRK_SENSE;
} tu16_Adc;

/* Structure for individual currents */
typedef struct
{
    uint16_t u16_I_PHA;
    uint16_t u16_I_PHB;
    uint16_t u16_I_PHC;
}tu16_Current;

/*Structure for Mov Average Sum*/
typedef struct
{
    uint32_t u32_SUM_PHA;
    uint32_t u32_SUM_PHB;
    uint32_t u32_SUM_PHC;
}tu32_CurrSum;

/* Structure used for storing ADC raw results */
typedef union
{                 /* Store results from ADC readings (Q0): */
    uint16_t    ch[ADC_CHANNELS];
    tu16_Adc    val;
} Adc_Raw_t;

/* Structure used for storing ADC averages */
typedef union
{                 /* Store results from ADC readings (Q0): */
    uint16_t    ch[ADC_CHANNELS];
    tu16_Adc    val;
} Adc_Averaged_t;

/* Structure for individual names of ADC Filtered Signals */
typedef struct
{
    uint32_t    u32_I_PHA_CHA;                /*   ADCIND0  I_PHA_CHx  */
    uint32_t    u32_I_PHB_CHA;                /*   ADCIND1  I_PHB_CHx  */
    uint32_t    u32_I_PHC_CHA;                /*   ADCIND2  I_PHC_CHx  */
    uint32_t    u32_MOTOR_PHASEx_I;           /*   Peak ADC Cnt from the 3 motor currents A, B, C */
    uint32_t    u32_5V_SENSE;                 /*   ADCINC2  5v SENSE   */
    uint32_t    u32_POSITIVE_15V_SENSE;       /*   ADCINC3  +15v SENSE */
    uint32_t    u32_NEGATIVE_15V_SENSE;       /*   ADCINC4 -15v SENSE  */
    uint32_t    u32_MOTOR_TEMP_PHA;           /*   ADCIND0  I_PHA_CHx  */
    uint32_t    u32_MOTOR_TEMP_PHB;           /*   ADCIND1  I_PHB_CHx  */
    uint32_t    u32_MOTOR_TEMP_PHC;           /*   ADCIND2  I_PHC_CHx  */
    uint32_t    u32_RVDT_POS;                 /*   A1  P_POS */
    uint32_t    u32_RVDT_SUM;                 /*   A0  P_SUM */
    uint32_t    u32_DC_BUS_VSENSE;            /*   SPI ADC-DC Bus Sense 270V(HL Mode) or 28V (VC Mode)*/
    uint32_t    u32_28VBUS_VSENSE;
    uint32_t    u32_IBRK_SENSE;
} tu32_Adc;

typedef union
{
    uint32_t    ch[ADC_CHANNELS];
    tu32_Adc    val;
} Adc_Filtered_t;

typedef struct
{
    float32_t    f32_I_PHA_CHA;               /*   ADCIND0  I_PHA_CHx */
    float32_t    f32_I_PHB_CHA;               /*   ADCIND1  I_PHB_CHx */
    float32_t    f32_I_PHC_CHA;               /*   ADCIND2  I_PHC_CHx */
    float32_t    f32_MOTOR_PHASEx_I;          /*   Peak current from the 3 motor currents A, B, C */
    float32_t    f32_5V_SENSE;                /*   ADCINC2  5v SENSE */
    float32_t    f32_POSITIVE_15V_SENSE;      /*   ADCINC3  +15v SENSE */
    float32_t    f32_NEGATIVE_15V_SENSE;      /*   ADCINC4 -15v SENSE */
    float32_t    f32_MOTOR_TEMP_PHA;          /*   ADCIND0  I_PHA_CHx*/
    float32_t    f32_MOTOR_TEMP_PHB;          /*   ADCIND1  I_PHB_CHx*/
    float32_t    f32_MOTOR_TEMP_PHC;          /*   ADCIND2  I_PHC_CHx*/
} tf32_Adc;

/* Structure used for storing unit values after calculation for Bus Volts,Wind.Temp.,Phase Currents*/
typedef union
{                 /* Store Unit results after calculation*/
   float32_t    ch[9];
   tf32_Adc    val;
} Adc_UnitVal_t;

/* Structure used for storing Filtered (Moving Average) Currents*/
typedef union
{
    uint16_t        ch[ADC_CURRENT_CHANNELS];
    tu16_Current    val;
} Adc_MovAveraged_t;

/* Structure used for storing Moving Average Sum*/
typedef union
{
    uint32_t        ch[ADC_CURRENT_CHANNELS];
    tu32_CurrSum    val;
} Adc_MovAvgSum_t;


/*      Public Variable Declarations */

/* Storage for raw results */
extern Adc_Raw_t Adc_Raw;

/* Storage for averaged values */
extern Adc_Averaged_t Adc_Averaged;

/* Storage for intermediate filtered averaged values */
extern Adc_Filtered_t Adc_Filtered;

extern Adc_UnitVal_t Adc_f32UnitValRaw;
extern Adc_UnitVal_t Adc_f32UnitValAvg;
extern Adc_MovAveraged_t Adc_MovAveraged;

/*      Public ROM Constants */

/*      Public Defines */
#define RIG_CAL_LIMIT          2457    /* 1.80V = 6.77A (w/ tare) = 5.25A (w/o tare) */
#define CURRENT_LIMIT_NORMAL   3003    /* 2.20V = 8.25A (w/ tare) = 6.75A (w/o tare) */
#define CURRENT_LIMIT_REDUCE   2702    /* 1.98V (reduced performance current limit (24V) */

#define I_PHA_MOV_AVG                (Adc_MovAveraged.val.u16_I_PHA)
#define I_PHB_MOV_AVG                (Adc_MovAveraged.val.u16_I_PHB)
#define I_PHC_MOV_AVG                (Adc_MovAveraged.val.u16_I_PHC)
#define VBUS270_VSENSE_SIG_value      270.0F    // in volts
#define VBUS28_VSENSE_SIG_value       28.0F     // in volts
#define IBRK_SENSE_SIG_value          2.75F     // in volts

#define VBUS270_VSENSE_tol_value      0.013F    // in volts
#define VBUS28_VSENSE_tol_value       0.024F    // in volts
#define IBRK_SENSE_tol_value          0.005F    // in volts

#define VBUS270_VSENSE_TFUNC    (VBUS270_VSENSE_SIG_value/((1.62F/3.3F)*4095.0F))  // tolerance +-13mv
#define VBUS28_VSENSE_TFUNC     (VBUS28_VSENSE_SIG_value/((2.38F/3.3F)*4095.0F))  // tolerance +-24mv
#define IBRK_SENSE_TFUNC        (IBRK_SENSE_SIG_value/((2.75F/3.3F)*4095.0F))  // tolerance +-5mv

#define VBUS270_VSENSE_UNITV        (VBUS270_VSENSE_TFUNC * Adc_Averaged.val.u16_DC_BUS_VSENSE)
#define VBUS270_VSENSE_UNITV_raw    (VBUS270_VSENSE_TFUNC * Adc_Raw.val.u16_DC_BUS_VSENSE)
#define VBUS28_VSENSE_UNITV         (VBUS28_VSENSE_TFUNC * Adc_Averaged.val.u16_28VBUS_VSENSE)
#define VBUS28_VSENSE_UNITV_raw     (VBUS28_VSENSE_TFUNC * Adc_Raw.val.u16_28VBUS_VSENSE)
#define IBRK_SENSE_UNITV            (IBRK_SENSE_TFUNC * Adc_Averaged.val.u16_IBRK_SENSE)
#define IBRK_SENSE_UNITV_raw        (IBRK_SENSE_TFUNC * Adc_Raw.val.u16_IBRK_SENSE)

#define VBUS270_VSENSE_SIG_tol_value      (VBUS270_VSENSE_TFUNC*((0.013F/3.3F)*4095.0F))  // in volts
#define VBUS28_VSENSE_SIG_tol_value       (VBUS28_VSENSE_TFUNC*((0.024F/3.3F)*4095.0F))   // in volts
#define IBRK_SENSE_SIG_tol_value          (IBRK_SENSE_TFUNC*((0.005F/3.3F)*4095.0F))      // in volts

#define VBUS270_VSENSE_SIG_value_max      (VBUS270_VSENSE_UNITV+VBUS270_VSENSE_SIG_tol_value)   // in volts
#define VBUS28_VSENSE_SIG_value_max       (VBUS28_VSENSE_UNITV+VBUS28_VSENSE_SIG_tol_value)     // in volts
#define IBRK_SENSE_SIG_value_max          (IBRK_SENSE_UNITV+IBRK_SENSE_SIG_tol_value)           // in volts

#define VBUS270_VSENSE_SIG_value_min      (VBUS270_VSENSE_UNITV-VBUS270_VSENSE_SIG_tol_value)   // in volts
#define VBUS28_VSENSE_SIG_value_min       (VBUS28_VSENSE_UNITV-VBUS28_VSENSE_SIG_tol_value)     // in volts
#define IBRK_SENSE_SIG_value_min          (IBRK_SENSE_UNITV-IBRK_SENSE_SIG_tol_value)           // in volts

/* name Raw Analog Input Macros */
/* Use these macros to easily access the proper ADC Result Register for the raw sampled value. */
#define V5_SENSE_rslt                 (AdccResultRegs.ADCRESULT2)       /* ADC input for +5V_AN_CH */
#define POSITIVE_V15_SENSE_rslt       (AdccResultRegs.ADCRESULT3)       /* ADC input for +15V_AN_CH */
#define NEGATIVE_V15_SENSE_rslt       (AdccResultRegs.ADCRESULT4)       /* ADC input for -15V_AN_CH */

#define I_PHA_CHA_rslt                (AdcdResultRegs.ADCRESULT0)
#define I_PHB_CHA_rslt                (AdcdResultRegs.ADCRESULT1)
#define I_PHC_CHA_rslt                (AdcdResultRegs.ADCRESULT2)

#define MOTOR_TEMP_PHA_rslt           (AdcdResultRegs.ADCRESULT4)
#define MOTOR_TEMP_PHB_rslt           (AdcaResultRegs.ADCRESULT14)
#define MOTOR_TEMP_PHC_rslt           (AdcaResultRegs.ADCRESULT15)

#define MOTOR_MAX_POSITIVE_TEMP_LIMIT (120.0f) /* Max +ve Winding Temp for Monitors*/
#define MOTOR_FAULT_RESET_TEMP_LIMIT  (110.0f) /* Fault reset +ve Winding Temp for Monitors*/

#define RVDT_POS_rslt           (AdcaResultRegs.ADCRESULT1)      /* RVDT primary position signal */
#define RVDT_SUM_rslt           (AdcaResultRegs.ADCRESULT0)      /* RVDT primary sum signal */

#define ADC_AVG_SAMPLES 32
#define ADC_AVG_SHIFT   5
#define ADC_PBIT_SHIFT  3
#define ADC_CONSTANT_COUNT (2047U) /* ADC constant value for -ve to +ve Amp range*/

/*      Public Interface Function Prototypes */

void Adc_Init( void );
void Adc_Sample( void );
void Adc_GetResults( void );
void Adc_Average( STATE_ID );
void Adc_GetResults_motor_phases( void );
void Adc_Sample_motor_phase_currents( void );
void Adc_CalcTempfromADCcount(void);
void Adc_CalcUnitvalforVolts(void);
void Adc_CalcUnitvalforPhases(void);
void Adc_GetMovingAverage(void);



#endif
/* end adc.h */

