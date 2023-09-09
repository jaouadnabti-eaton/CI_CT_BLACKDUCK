/****************************************************************************************************
*  File name: ASW_BSW_I.h
*
*  Purpose: ASW to BSW Interface Definition
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


#ifndef ASW_BSW_I_H_
#define ASW_BSW_I_H_

/* Required for Simulink Import functionality so settings are the same as actual target */
#ifndef __TI_COMPILER_VERSION__
#define _FLASH
#define CPU1
#define __NO_STROKE_LIMIT__
#define __WCTA__
#define N_HSIT
#define NDRV8312_DEV_KIT
#define __cregister
#define __interrupt
#define interrupt
#endif


/******************** INCLUDES *************************/
//#include <stdint.h>
//#include "rtwtypes.h" /* for uint16_t etc */
#include "typedefs.h"
//#include "adc.h"

/*      Public Type Definitions */
typedef struct 
{
    float32_t    f32_I_PHA_CHA;               /*   ADCIND0  I_PHA_CHx */
    float32_t    f32_I_PHB_CHA;               /*   ADCIND1  I_PHB_CHx */
    float32_t    f32_I_PHC_CHA;               /*   ADCIND2  I_PHC_CHx */
    float32_t    f32_5V_SENSE;                /*   ADCINC2  5v SENSE */
    float32_t    f32_POSITIVE_15V_SENSE;      /*   ADCINC3  +15v SENSE */
    float32_t    f32_NEGATIVE_15V_SENSE;      /*   ADCINC4 -15v SENSE */
    float32_t    f32_MOTOR_TEMP_PHA;          /*   ADCIND0  I_PHA_CHx*/
    float32_t    f32_MOTOR_TEMP_PHB;          /*   ADCIND1  I_PHB_CHx*/
    float32_t    f32_MOTOR_TEMP_PHC;          /*   ADCIND2  I_PHC_CHx*/
    float32_t    f32_I_AVR_MAX;               /* Average Current Amplitude in Amperes */
    float32_t    f32SpeedRef;                 /* Speed Reference for Speed PID Calculations */
    float32_t    f32SpeedFdb;                 /* Speed Feedback for Speed PID Calculations */
} tAnalogInputs_t;

typedef struct
{
    eLruId_t    eLruId;                 /* G_eLruId:  LRU Identification */
    eActId_t    eChId;                  /* G_eChId:   Channel Identification */
    eActuator_t eActuatorNumber;        /* G_eActuatorNumber:  Actuator Number */
    uint16_t    u16SCU_MCU_ENABLE;      /* SCU_MCU_ENABLE */
    bool_t      bCHX_STATUS;            /* Cross-side CH_STATUS */
    bool_t      bVcModeCmd;             /* VC Mode Command from ARINC825 Interface */
    bool_t      bSensorFusionEnableCmd; /* Sensor Fusion Enable Command from ARINC825 Interface */
	int16_t     ResetIntegralGain;
    uint16_t    xChannelEnable_M;
} tDiscreteInputs_t;

typedef struct
{
    float32_t f32StrokeRvdt;                 /* Actuator Stroke Position in inches from RVDT ADC Count */
    float32_t f32FlapAngleRvdt;              /* Flap Angle Degrees Stream-wise from RVDT ADC Count */
    float32_t f32StrokeQuad;                 /* Actuator Stroke Position in inches from Quad Count */
    float32_t f32FlapAngleQuad;              /* Flap Angle Degrees Stream-wise from Quad Count */
    float32_t f32StrokeFused;                /* Last copy of Stroke Fused used for Initial Position input (NVM) in Sensor Fusion. */
    int16_t   s16QuadPosition;               /* Quad Count position of the actuator */
    int16_t   s16QuadCntRvdt;                /* Quad Count based on the Stroke calculated from RVDT count */
} tPanelInputs_t;

typedef struct
{
    tAnalogInputs_t     tAnalogInputData;
    tDiscreteInputs_t   tDiscreteInputData;
    tPanelInputs_t      tPanelInputData;
} BSW_DATA_t;

typedef struct 
{
    uint16_t  ANALOG_OUT_A;           /* Analog Output A */                      
    uint16_t  ANALOG_OUT_B;           /* Analog Output B */ 
    float32_t f32SpeedPidOutput;      /* Speed PID Output in % PWM Duty Cycle On */
} tAnalogOutputs_t;

typedef struct
{
    bool_t bVC_MODE;
    bool_t b270V_BUS_CNTL;
    bool_t b28V_BUS_CNTL;
    bool_t bINRUSH_CTR;
    bool_t bBUS_SW_ENABLE;
} tDiscreteOutputs_t;

typedef struct
{
    float32_t f32StrokeFused;                /* Actuator Stroke Position Estimate in inches from Sensor Fusion algo*/
    float32_t f32BiasFused;                  /* Estimated Bias from Sensor Fusion */
    float32_t f32QuadCntResidualFused;       /* Measurement Residual for Motor Quad Counts from Sensor Fusion */
    float32_t f32SkewResidualFused;          /* Measurement Residual for Skew Sensor in Stroke inches from Sensor Fusion */
    int16_t   s16QuadFused;                  /* Motor Position Estimate in Quad Counts converted from Stroke Position Estimate */
    uint16_t  u16MemoryPad;                  /* Required to pad memory so CRC works */
} tPanelOutputs_t;

typedef struct
{
    tAnalogOutputs_t    tAnalogOutputData;
    tDiscreteOutputs_t  tDiscreteOutputData;
    tPanelOutputs_t     tPanelOutputData;
} ASW_DATA_t;

/*      Public Variable Declarations */
extern BSW_DATA_t G_BSW_DATA;
extern ASW_DATA_t G_ASW_DATA;

/*      Public Interface Function Prototypes */
extern void ASW_getBSWdata_Wrapper(BSW_DATA_t *Bus);
extern void ASW_setBSWdata_Wrapper(const ASW_DATA_t *Bus);

#endif /* ASW_BSW_I_H_ */
