/****************************************************************************************************
*  File name: adc.c
*
*  Purpose: Implements ADC Driver functions.
*      The file implements routines necessary to initialize the ADC hardware,
*      capture analog signal inputs, and calibrate those signals against two of
*      the channel readings from well-known signal inputs.
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

/*      Include Files */
/* DSP281x Include File, includes register definition file */
#include "adc.h"
#include "mcu.h"
#include "gpio.h"
#include "parameter.h"
#include "spi.h"

/*      Local Type Definitions
*/

/*      Local Defines */

/*--------------------------------------------------------
 Mapping of ADC channels to result registers:*/

#define     A0       ADCRESULT0
#define     A1       ADCRESULT1
#define     A2       ADCRESULT2
#define     A3       ADCRESULT3
#define     A4       ADCRESULT4
#define     A5       ADCRESULT5
#define     A6       ADCRESULT6
#define     A7       ADCRESULT7
#define     B0       ADCRESULT8
#define     B1       ADCRESULT9
#define     B2       ADCRESULT10
#define     B3       ADCRESULT11
#define     B4       ADCRESULT12
#define     B5       ADCRESULT13
#define     B6       ADCRESULT14
#define     B7       ADCRESULT15


/* ADC start parameters */
/* ADC module clock = SYSCLK/8 = 200MHz/8 = 25MHz. ADC_CKPS = value of 8 which is 0x1110 */
#define ADC_CKPS   0xE
/* Sample + Hold width in ADC module periods = 270ns / 5ns = 54 SYSCLKs = 53 + 1 */
//#define ADC_SHCLK  53U
#define ADC_SHCLK  511U /* Max S+H Cycle Count */

/* ADC SOCPRIORITY defines */
#define ADC_SOCPRIORITY_ALL_HIGH_PRIORITY       0x10U

/* ADCPWDNZ ADC Power Down Register defines */
#define ADC_ADCPWDNZ_POWERDOWN  0U
#define ADC_ADCPWDNZ_POWERUP    1U

#define ADC_POWERUP_DELAY_1MS (1000L)

/* Software SOC defines */
#define ADCA_SOC_FRC_ALL (0xC003U) /* ADCA, SOC0, 1, 14, 15    */
#define ADCC_SOC_FRC_ALL (0x001CU) /* ADCC, SOC2, 3, 4       */
#define ADCD_SOC_FRC_ALL (0x0017U) /* ADCD, SOC0, 1, 2, 4 */
#define ADCD_SOC_FRC_PHASE_CURRENT (0x0007U)

#define V5_SENSE_TRANSFER_FUNC             (0.00146526f)/*((5.0f)  / ((0.8333)*(4095))*/
/* V5_SENSE_TRANSFER_FUNC details
 * 5V_SENSE signal sensing
 * 0v to 5v is mapped to 0v to +2.5v range available
 * 0.8333 = 2.50v / 3.0v
 * ADCC can measure 0v to +3v
 *
 * */

#define POSITIVE_V15_SENSE_TRANSFER_FUNC  (0.00422654f) /*((15.0f) / (((2.60f)/(3.0f))*(4095)))*/
/* POSITIVE_V15_SENSE_TRANSFER_FUNC details
 * V15_SENSE signal sensing
 * 0v to 15v is mapped to 0v to +2.60v range available
 * 0.8666 = 2.60v/3.0f
 * ADCC can measure 0v to +3v
 *
 * */

#define NEGATIVE_V15_SENSE_TRANSFER_FUNC   (0.00437809f) /*((15.0f) / (((2.51f)/(3.0f))*(4095)))*/
/* NEGATIVE_V15_SENSE_TRANSFER_FUNC details
 * V15_SENSE signal sensing
 * 0v to 15v is mapped to 0v to +2.51v range available
 * 0.8366 = 2.51v/3.0v
 * ADCC can measure 0v to +3v
 *
 * */

#define MOTOR_PHASE_TEMP_VOLTAGE_DIVIDER_FACTOR (0.00122095f) /*(((3.0f) / (4095))* (1.6666f))  1.6666f converts 2.7v to 4.5v range*/

#define MOTOR_PHASE_CURRENT_TRANSFER_FUNC (0.00282726f) /*0.00282726f Amps/ADC count * (ADC-2047 Cnts) = Amps*/
/*  Phase current transfer function details
 *  Applicable for Motor Phase A,B,C current measurement
 * -5Amp to +5 Amp range available
 *  for -5Amp - analog voltage 0.84V - Adc count 1145
 *  for +5Amp - analog voltage 2.16V - Adc count 2949
 *  Transfer Function
 *  ADC_I_Read = (((((ADC count*Vref)/ADC max count)-1.5)/Diff. amplifier gain)/AMC1300gain)/R-shunt)
 *  Vref- 3v, Reference voltage 1.5v, Diff. amplifier gain 0.806,AMC1300gain 8.2,R-shunt 0.02
 *  Equation further simplified to 0.00554228015244007110595703125 Amps/Cnt * (ADC-2047 Cnts) = Amps
 * */


/* RVDTs need to be inverted for some channels due to cabling and HW inversion. This software inversion
 * is applied for the R&D VCR2 project and should not be carried over as part of future designs.
 * The RVDTs have a range of 0.042VDC to 2.81VDC for both VA and VB signals. The ADC has a range of
 * 0VDC to 3.0VDC. Therefore, when inverting an offset needs to be subtracted off to appropriately
 * scale the signal in the proper range:
 *
 * Voltage Offset = (3VDC - 2.81VDC) - (0.042VDC - 0VDC) = 0.19VDC - 0.042VDC = 0.148VDC
 * ADC Offset = (0.148VDC / 3.0VDC) * 4095 = 202 Counts
 */
#define ADC_RVDT_INVERT_OFFSET  (202U)
#define ADC_MAX_VAL_12_BIT      (0x0FFFU)

/*      Global Variables */
Adc_Raw_t Adc_Raw =
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
Adc_Averaged_t Adc_Averaged =
{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
Adc_Filtered_t Adc_Filtered =
{0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL};
Adc_UnitVal_t Adc_f32UnitValRaw =
{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
Adc_UnitVal_t Adc_f32UnitValAvg =
{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
Adc_MovAveraged_t Adc_MovAveraged =
{0U, 0U, 0U};

/*      Local ROM Constants */

/*      Local Variable Declarations */
/*Due to limitation of compiler-
 * Ia_Arr[ADC_MA_WINDOW_LENGTH] = { 0 } syntax initializes only first element of array
 * Therefore initializing individual elements of array */

//Array holding 400 samples for Ia
static uint16_t Ia_Arr[ADC_MA_WINDOW_LENGTH] =
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//Array holding 400 samples for Ib
static uint16_t Ib_Arr[ADC_MA_WINDOW_LENGTH] =
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//Array holding 400 samples for Ic
static uint16_t Ic_Arr[ADC_MA_WINDOW_LENGTH] =
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/*      Local Function Prototypes */
static uint16_t Adc_MovingAverage(uint16_t* ptrArrWindow, uint32_t* ptrMovAvgSum, uint16_t index, uint16_t CurrSample);



/*      Function Definitions */

/*
    brief Initialization routine for ADC driver.

    Purpose:
        Configures ADC registers for proper operation of the driver.

    Global Data Referenced:
        #AdcRegs
    
    return void
    
    Preconditions and Assumptions:
        This routine should be called only once, prior to the scheduler loop.

    remarks
    Pull calibrated TRIM values from OTP as part of the configuration to ensure
    ADCs are calibrated

*/
void Adc_Init( void )
{
    EALLOW;

    /* PATH(Adc_Init,A); */

    /* shall configure ADC clock to 25MHz */
    AdcaRegs.ADCCTL2.bit.PRESCALE = ADC_CKPS;
    AdccRegs.ADCCTL2.bit.PRESCALE = ADC_CKPS;
    AdcdRegs.ADCCTL2.bit.PRESCALE = ADC_CKPS;

    /* Datasheet refers to calling this function for getting TRIM values from OTP memory. */
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    /* Set ALL SOCs to High Priority. Arbitration done by SOC# */
    AdcaRegs.ADCSOCPRICTL.bit.SOCPRIORITY = ADC_SOCPRIORITY_ALL_HIGH_PRIORITY;    /* Set to high priority for ADCA SOCs */
    AdccRegs.ADCSOCPRICTL.bit.SOCPRIORITY = ADC_SOCPRIORITY_ALL_HIGH_PRIORITY;    /* Set to high priority for ADCC SOCs */
    AdcdRegs.ADCSOCPRICTL.bit.SOCPRIORITY = ADC_SOCPRIORITY_ALL_HIGH_PRIORITY;    /* Set to high priority for ADCD SOCs */

    /************* SOC0 CONFIGURATION *****************************************************/
    /* SOC0:  Channel Enable */
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = ADC_CHANNEL_0; /* Set SOC0 to convert ADCINA0:  RVDT_SUM    */
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = ADC_CHANNEL_0; /* Set SOC0 to convert ADCIND0:  I_PHA_CHA   */

    /* SOC0:  Conversion Timing */
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = ADC_SHCLK; /* Set Sample + Hold Clock Cycles for ADCA SOC0  */
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = ADC_SHCLK; /* Set Sample + Hold Clock Cycles for ADCD SOC0  */

    /************* SOC1 CONFIGURATION *****************************************************/
    /* SOC1:  Channel Enable */
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = ADC_CHANNEL_1; /* Set SOC1 to convert ADCINA1:  RVDT_POS */
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = ADC_CHANNEL_1; /* Set SOC1 to convert ADCIND1:  I_PHA_CHB   */

    /* SOC1:  Conversion Timing */
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = ADC_SHCLK; /* Set Sample + Hold Clock Cycles for ADCA SOC1  */
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = ADC_SHCLK; /* Set Sample + Hold Clock Cycles for ADCD SOC1  */

    /************* SOC2 CONFIGURATION *****************************************************/
    /* SOC2:  Channel Enable */
    AdccRegs.ADCSOC2CTL.bit.CHSEL = ADC_CHANNEL_2; /* Set SOC2 to convert ADCINC2:  5v SENSE    */
    AdcdRegs.ADCSOC2CTL.bit.CHSEL = ADC_CHANNEL_2; /* Set SOC2 to convert ADCIND2:  I_PHA_CHC */

    /* SOC2:  Conversion Timing */
    AdccRegs.ADCSOC2CTL.bit.ACQPS = ADC_SHCLK; /* Set Sample + Hold Clock Cycles for ADCC SOC2  */
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = ADC_SHCLK; /* Set Sample + Hold Clock Cycles for ADCD SOC2  */

    /************* SOC3 CONFIGURATION *****************************************************/
    /* SOC3:  Channel Enable */
    AdccRegs.ADCSOC3CTL.bit.CHSEL = ADC_CHANNEL_3; /* Set SOC3 to convert ADCINC3:  15v SENSE   */

    /* SOC3:  Conversion Timing */
    AdccRegs.ADCSOC3CTL.bit.ACQPS = ADC_SHCLK; /* Set Sample + Hold Clock Cycles for ADCC SOC3  */

    /************* SOC4 CONFIGURATION *****************************************************/
    /* SOC4:  Channel Enable */
    AdccRegs.ADCSOC4CTL.bit.CHSEL = ADC_CHANNEL_4; /* Set SOC4 to convert ADCINC4:  -15v SENSE    */
    AdcdRegs.ADCSOC4CTL.bit.CHSEL = ADC_CHANNEL_4; /* Set SOC4 to convert ADCIND4:  MOTOR_TEMP_PHA */

    /* SOC4:  Conversion Timing */
    AdccRegs.ADCSOC4CTL.bit.ACQPS = ADC_SHCLK; /* Set Sample + Hold Clock Cycles for ADCC SOC4  */
    AdcdRegs.ADCSOC4CTL.bit.ACQPS = ADC_SHCLK; /* Set Sample + Hold Clock Cycles for ADCD SOC4  */

    /************* SOC14 CONFIGURATION *****************************************************/
    /* SOC14:  Channel Enable */
    AdcaRegs.ADCSOC14CTL.bit.CHSEL = ADC_CHANNEL_14; /* Set SOC14 to convert ADCINC14:  MOTOR_TEMP_PHB */

    /* SOC14:  Conversion Timing */
    AdcaRegs.ADCSOC14CTL.bit.ACQPS = ADC_SHCLK; /* Set Sample + Hold Clock Cycles for ADCC SOC14 */

    /************* SOC15 CONFIGURATION *****************************************************/
    /* SOC15:  Channel Enable */
    AdcaRegs.ADCSOC15CTL.bit.CHSEL = ADC_CHANNEL_15; /* Set SOC15 to convert ADCINC15:  MOTOR_TEMP_PHC */

    /* SOC15:  Conversion Timing */
    AdcaRegs.ADCSOC15CTL.bit.ACQPS = ADC_SHCLK; /* Set Sample + Hold Clock Cycles for ADCC SOC15  */
    /************* SOC CONFIGURATION END***********************************************************/

    /* Power Up the ADCs */
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = ADC_ADCPWDNZ_POWERUP; /* Power up ADC A */
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = ADC_ADCPWDNZ_POWERUP; /* Power up ADC C */
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = ADC_ADCPWDNZ_POWERUP; /* Power up ADC D */

    EDIS;

    /* Delay to allow time for ADCs to power up */
    DELAY_US(ADC_POWERUP_DELAY_1MS);        /* Delay after powering up ADC */

    /* PATH(Adc_Init,B); */
}


/*
    brief Triggers the ADC to sample and convert all analog inputs.

    Purpose:
        Signals the ADC hardware to sample and convert a sequence of all 16 
        analog inputs.  The hardware can perform an entire 16-channel 
        conversion sequence without software interaction.

    Global Data Referenced:
        #AdcRegs
    
    return void
    
    Preconditions and Assumptions:
        This routine is intended to be called regularly by the scheduler.  

    remarks
        The time needed for the hardware to complete the conversions is 
        dependent on the ADC clock setting, but should be well within the time 
        of a single 8 kHz scheduler frame.  Total time for converting 16 
        channels across 5 different SOCs is:
    
            T = 5 * ((tLAT + (1 + ACQPS)) * SYSCLK)
            T = 5 * ((84 + 54) * 1/200M) = 3,450ns
            0.125ms / 0.000003450ms = 36times larger
    
        where tLAT is the time in SYSCLK cycles it takes for the data to
        be latched into the ADCRESULT register after the S + H time and
        ACQPS is the acquisition window width, in SYS clock cycles.

*/
void Adc_Sample( void )
{
    /* PATH(Adc_Sample,A); */

    /* Force SOCs for all configured SOCs */
    AdcaRegs.ADCSOCFRC1.all = ADCA_SOC_FRC_ALL;
    AdccRegs.ADCSOCFRC1.all = ADCC_SOC_FRC_ALL;
    AdcdRegs.ADCSOCFRC1.all = ADCD_SOC_FRC_ALL & (~ADCD_SOC_FRC_PHASE_CURRENT);

    /* PATH(Adc_Sample,B); */
}

/*
    brief Triggers the ADC to sample and convert all analog inputs.

    Purpose:
        Signals the ADC hardware to sample and convert a sequence of Phase signal
        analog inputs.

    Global Data Referenced:
        #AdcRegs

    return void

    Preconditions and Assumptions:
        This routine is intended to be called @0.000125 by the scheduler.
*/
void Adc_Sample_motor_phase_currents(void)
{
    AdcdRegs.ADCSOCFRC1.all = ((ADCD_SOC_FRC_ALL) & (ADCD_SOC_FRC_PHASE_CURRENT));
}
/*
    brief Retrieves the analog input results and stores into local memory

    Purpose:  Get the results of the ADC conversions and store them to
    local memory.


     Global Data Referenced:
        #Adc_Calibrated
    
    return  void
    
    Preconditions and Assumptions:
        This routine is intended to be called regularly by the scheduler.

    remarks
            
*/
void Adc_GetResults( void )
{
    Adc_Raw.val.u16_5V_SENSE = V5_SENSE_rslt;
    Adc_Raw.val.u16_POSITIVE_15V_SENSE = POSITIVE_V15_SENSE_rslt;
    Adc_Raw.val.u16_NEGATIVE_15V_SENSE = NEGATIVE_V15_SENSE_rslt;
    Adc_Raw.val.u16_MOTOR_TEMP_PHA = MOTOR_TEMP_PHA_rslt;
    Adc_Raw.val.u16_MOTOR_TEMP_PHB = MOTOR_TEMP_PHB_rslt;
    Adc_Raw.val.u16_MOTOR_TEMP_PHC = MOTOR_TEMP_PHC_rslt;
    Adc_Raw.val.u16_RVDT_SUM = RVDT_SUM_rslt;
    Adc_Raw.val.u16_RVDT_POS = RVDT_POS_rslt;
    Adc_Raw.val.u16_DC_BUS_VSENSE = SpiRxBuf[nCS_270VBUS_VSENSE];
    Adc_Raw.val.u16_28VBUS_VSENSE = SpiRxBuf[nCS_28VBUS_VSENSE];
    Adc_Raw.val.u16_IBRK_SENSE = SpiRxBuf[nCS_IBRK_SENSE];
}


/*
    brief converts the ADC measured voltage in temperature using polynomial equation

    Purpose:  Get the results of the ADC conversions and store them to
    local memory.


     Global Data Referenced:
        #Adc_Averaged

    return  temp. in degree C

    Preconditions and Assumptions:
        This routine is intended to be called regularly by the scheduler.

    remarks
      equation = -0.9008x4 + 9.9189x3 - 36.987x2 + 83.975x + 11.024
*/
void Adc_CalcTempfromADCcount(void)
{
    static const float32_t f32TempPolyCoeffs[] = {11.024f,83.975f,-36.987f,9.9189f, -0.9008f};
    uint16_t i;
    float32_t f32x, f32x2, f32x3,f32x4;   /* Polynomial multiples */
    float32_t f32TermXstageVal = NAN;
    float32_t f32Temperature[3] = { NAN, NAN, NAN };
    uint16_t u16AverageAdc[] = {  Adc_Averaged.val.u16_MOTOR_TEMP_PHA,
                                  Adc_Averaged.val.u16_MOTOR_TEMP_PHB,
                                  Adc_Averaged.val.u16_MOTOR_TEMP_PHC  };

    for (i = 0; i < 3; i++)
    {
         /* Convert adc count to engineering units */
         f32x = ((float32_t)u16AverageAdc[i]) * MOTOR_PHASE_TEMP_VOLTAGE_DIVIDER_FACTOR;

         /* Run through the polynomial function */
         f32Temperature[i] = f32TempPolyCoeffs[0];

         f32TermXstageVal = f32TempPolyCoeffs[1] * f32x;
         f32Temperature[i] += f32TermXstageVal;

         f32x2 =f32x*f32x;
         f32TermXstageVal = f32TempPolyCoeffs[2] * f32x2;
         f32Temperature[i] += f32TermXstageVal;

         f32x3 = f32x2*f32x;
         f32TermXstageVal = f32TempPolyCoeffs[3] * f32x3;
         f32Temperature[i] += f32TermXstageVal;

         f32x4 = f32x3*f32x;
         f32TermXstageVal = f32TempPolyCoeffs[4] * f32x4;
         f32Temperature[i] += f32TermXstageVal;
    }

    /* Save off the temperatures to global engineering units structure */
    Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHA = f32Temperature[0];
    Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHB = f32Temperature[1];
    Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHC = f32Temperature[2];
}

/*
    brief stores the Unit value for phase currents & Volts from Adc Average value

    Purpose:  store the unit value in Adc_f32UnitVal for monitor purpose


     Global Data Referenced:
        #Adc_Averaged

    return  None

    Preconditions and Assumptions:
        This routine is intended to be called regularly by the scheduler.

    remarks

*/

void Adc_CalcUnitvalforPhases(void)
{
    uint16_t i;
    int16_t s16Temp;

    /* Convert the Raw and Average phase currents to engineering units.
     * NOTE:  this code relies on all 3 phase currents and maximum current to be
     * contiguous in their respective structures and begin at index 0. */
    for (i = 0; i < 4; i++)
    {
        /* Convert the Raw values */
        s16Temp = (int16_t)Adc_Raw.ch[i];
        s16Temp -= (int16_t)ADC_CONSTANT_COUNT; /* part of transfer function:  Average */
        Adc_f32UnitValRaw.ch[i] = (float32_t)s16Temp * MOTOR_PHASE_CURRENT_TRANSFER_FUNC;

        /* Convert the Average values */
		s16Temp = (int16_t)Adc_Averaged.ch[i];
		s16Temp -= (int16_t)ADC_CONSTANT_COUNT; /* part of transfer function:  Average */
        Adc_f32UnitValAvg.ch[i] = (float32_t)s16Temp * MOTOR_PHASE_CURRENT_TRANSFER_FUNC;
    }
}

/*
    brief: stores the Unit value for Volts from Adc Average value

    Purpose:  store the unit value in Adc_f32UnitVal for monitor purpose


     Global Data Referenced:
        #Adc_Averaged

    return  None

    Preconditions and Assumptions:
        This routine is intended to be called regularly by the scheduler.

    remarks

*/
void Adc_CalcUnitvalforVolts(void)
{
    /* copy volts after ADC averaged value converted to unit val*/
    Adc_f32UnitValAvg.val.f32_5V_SENSE = (float32_t)Adc_Averaged.val.u16_5V_SENSE * V5_SENSE_TRANSFER_FUNC;
    Adc_f32UnitValAvg.val.f32_POSITIVE_15V_SENSE = (float32_t)Adc_Averaged.val.u16_POSITIVE_15V_SENSE * POSITIVE_V15_SENSE_TRANSFER_FUNC;
    Adc_f32UnitValAvg.val.f32_NEGATIVE_15V_SENSE = (float32_t)Adc_Averaged.val.u16_NEGATIVE_15V_SENSE * NEGATIVE_V15_SENSE_TRANSFER_FUNC;
}

/*
    brief Retrieves the analog input results and stores into local memory

    Purpose:  Get the results of the ADC conversions and store them to
    local memory.


     Global Data Referenced:
        #Adc_Calibrated

    return  void

    Preconditions and Assumptions:
        This routine is intended to be called regularly by the scheduler.

    remarks

*/
void Adc_GetResults_motor_phases(void)
{
    Adc_Raw.val.u16_I_PHA_CHA = (I_PHA_CHA_rslt < 2047) ? (2047+(2047-I_PHA_CHA_rslt)):(I_PHA_CHA_rslt); /*converts the -ve values to +ve side*/
    Adc_Raw.val.u16_I_PHB_CHA=  (I_PHB_CHA_rslt < 2047) ? (2047+(2047-I_PHB_CHA_rslt)):(I_PHB_CHA_rslt); /*converts the -ve values to +ve side*/
    Adc_Raw.val.u16_I_PHC_CHA = (I_PHC_CHA_rslt < 2047) ? (2047+(2047-I_PHC_CHA_rslt)):(I_PHC_CHA_rslt); /*converts the -ve values to +ve side*/

    /*Saves the highest sample from a,b,c phase */
    Adc_Raw.val.u16_MOTOR_PHASEx_I = (Adc_Raw.val.u16_I_PHA_CHA > Adc_Raw.val.u16_I_PHB_CHA) ?
                                     ((Adc_Raw.val.u16_I_PHA_CHA > Adc_Raw.val.u16_I_PHC_CHA)?(Adc_Raw.val.u16_I_PHA_CHA):
                                      (Adc_Raw.val.u16_I_PHC_CHA))                           :
                                     ((Adc_Raw.val.u16_I_PHB_CHA > Adc_Raw.val.u16_I_PHC_CHA)?(Adc_Raw.val.u16_I_PHB_CHA):(Adc_Raw.val.u16_I_PHC_CHA));
}

/*
     Averaging routine for analog input readings.

     Purpose:
        Averages calibrated ADC readings to compensate for signal noise errors.
        Uses calibrated readings as input so that gain and offset errors are 
        already filtered out of the readings.

     Global Data Referenced:
        #Adc_Raw
        #Adc_Averaged
        #Adc_Filtered
    
    return  void
    
    Preconditions and Assumptions:
        This routine is intended to be called regularly by the scheduler, after
        the Adc_GetResults() routine.  Use of this function is not mandatory.

    remarks
        The averaging formula used is an approximate running average, based on 
        a first-order-lag filter algorithm.  The algorithm used is equivalent 
        to the equation shown here:
        
            New = Old + t(Sample - Old)
        
         where  t controls the "time constant" of the filter.  If  t is kept as 
         a (inverse) power of 2, then the multiply operation can be a simple 
         bitwise shift.
        
        Note that when the computation was implemented as a single C statement, 
        it did not work correctly.
*/
void Adc_Average( STATE_ID state )
{
    int16 i;
    uint32_t raw32, mult, new;
    int32_t diff;

    /* PATH(Adc_Average,A); */

    for (i = 0; i < ADC_CHANNELS; i++)
    {
        /* PATH(Adc_Average,B); */
        raw32 = ((Uint32)(Adc_Raw.ch[i]) & 0x00000FFF);
        mult = (raw32 << 16);
        diff = (int32)(mult) - (int32)(Adc_Filtered.ch[i]);

        if (state == BOOT_MODE)
        {
            /* PATH(Adc_Average,C); */
            diff >>= ADC_PBIT_SHIFT;
        }
        else
        {
            /* PATH(Adc_Average,D); */
            diff >>= ADC_AVG_SHIFT;
        }

        new = (Uint32)(diff + Adc_Filtered.ch[i]);

        if ((new & 0xF0000000) != 0)
        {
            /* PATH(Adc_Average,E); */
            new = 0;
        }

        Adc_Filtered.ch[i] = new;
        Adc_Averaged.ch[i] = (uint16_t) (Adc_Filtered.ch[i] >> 16UL);
    }

    /* PATH(Adc_Average,F); */
}


/*************************************************************************************************\
* Function: Adc_GetMovingAverage
*
* Purpose:            This function perform the Moving Average filtering on raw currents (Ia,Ib,Ic)
* Input(s):           None
* Output(s):          None
* Global Variables    Adc_Raw
* Referenced:         Adc_MovAveraged
*
* Note:               The function performs the Moving Average filtering on currents.
*                     These filtered currents are used for detecting motor winding faults.
*                     This function is called every 125 microseconds.
\*************************************************************************************************/
void Adc_GetMovingAverage(void)
{

    static Adc_MovAvgSum_t Adc_MovAvgSum = {0U, 0U, 0U};        //Structure holding Sum of 400 samples of sliding window for 3 currents
    /*Array of Pointers pointing to starting addresses of three Arrays containing the 400 samples of sliding Window */
    static uint16_t *ptrArrWindow[ADC_CURRENT_CHANNELS] = {(uint16_t*)&Ia_Arr, (uint16_t*)&Ib_Arr, (uint16_t*)&Ic_Arr};
    uint16_t i;                                                    //ADC current channel being processed
    static uint16_t index = 0;                                    //Element of the array being processed (being replaced by raw value)


    for (i = 0; i < ADC_CURRENT_CHANNELS; i++)
    {
        /*Store the Moving Average values in Global variables */
         Adc_MovAveraged.ch[i] = Adc_MovingAverage(ptrArrWindow[i], (uint32_t*)&Adc_MovAvgSum.ch[i], index, Adc_Raw.ch[i]);
    }

    /*Update the element of the array that needs to be replaced by raw values in next function all*/
    index++;
     if (index >= ADC_MA_WINDOW_LENGTH)
     {
         index = 0;
     }
}

/*************************************************************************************************\
* Function: Adc_MovingAverage
*
* Purpose:          Calculates the Moving Average
* Input(s):         ptrArray
*                   ptrSum
*                   pos
*                   CurrSample
* Output(s):        Adc_Moving_Averge
* Note:             This is a Moving Average Filter using sliding window method, the window length is 400 Samples.
\*************************************************************************************************/

uint16_t Adc_MovingAverage(uint16_t* ptrArrWindow, uint32_t* ptrMovAvgSum, uint16_t index, uint16_t CurrSample)
{
    uint16_t MovingAverage = 0;
    /*Update the Sum of 400 samples
     * NewSum =  Old Sum + Current Sample - value of element of Array being replaced + Current Sample*/
    *ptrMovAvgSum = (*ptrMovAvgSum + CurrSample) - *(ptrArrWindow+index);

   /*Replace the element of the array with Current Sample */
   *(ptrArrWindow+index) = CurrSample;

   /*Calculate moving average by dividing the NewSum by the ADC WINDOW LENGTH (400)
    * Moving Average = Sum/400
    * Moving Average = Sum * 0.0025*/
   /*Take local variable*/
   MovingAverage = (uint16_t)((*ptrMovAvgSum) * ADC_MA_Multiplier);
   return (MovingAverage);
}

/* end adc.c */

