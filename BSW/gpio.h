/****************************************************************************************************
*  File name: gpio.h
*
*  Purpose: Interface file for GPIO (general purpose I/O) Driver
*  This file describes the Public Interface for the GPIO driver.  The interface provides
*  routines necessary to capture discrete inputs from input ports, and macros for access
*  to the GPIO signals.
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

#ifndef GPIO_H__
#define GPIO_H__

/*      Include Files
*/
//#include "dsp281x.h"   /* DSP281x Include File, includes register definition file*/
#include "F2837xD_device.h"
#include "parameter.h"

/*      Public Type Definitions
*/
/* ingroup gpio*/
typedef struct
{
    uint16_t bStart:1;   /* Start Pushbutton */
    uint16_t bStop:1;    /* Stop Pushbutton */
    uint16_t rsvd1:14;   /* 15:2 RESERVED */
} tStartStopBits_t;

typedef union
{
    uint16_t  all;
    tStartStopBits_t  bit;
} tStartStopWord_t;

typedef enum
{
    PHA_HS_nRESET = 0,
    PHA_LS_nRESET,
    PHB_HS_nRESET,
    PHB_LS_nRESET,
    PHC_HS_nRESET,
    PHC_LS_nRESET,
    nCS_270VBUS_VSENSE,
    nCS_28VBUS_VSENSE,
    nCS_IBRK_SENSE,
    LED1,
    LED2,
    LED3,
    ENC_ZERO_SET_CNTL = 12,
    INVALID_DEMUX_OUTPUT
} DemuxOutput_t ;

typedef enum
{
    PHA_HS_nFAULT = 0,
    PHA_LS_nFAULT,
    PHB_HS_nFAULT,
    PHB_LS_nFAULT,
    PHC_HS_nFAULT,
    PHC_LS_nFAULT,
    PHA_HS_READY,
    PHA_LS_READY,
    PHB_HS_READY,
    PHB_LS_READY,
    PHC_HS_READY,
    PHC_LS_READY,
    TOTAL_MUX_INPUT,
    INVALID_MUX_INPUT
} MuxInput_t;

typedef struct
{                 //bits description
    uint16_t PHA_HS_nFAULT:1;
    uint16_t PHA_LS_nFAULT:1;
    uint16_t PHB_HS_nFAULT:1;
    uint16_t PHB_LS_nFAULT:1;
    uint16_t PHC_HS_nFAULT:1;
    uint16_t PHC_LS_nFAULT:1;
    uint16_t PHA_HS_READY:1;
    uint16_t PHA_LS_READY:1;
    uint16_t PHB_HS_READY:1;
    uint16_t PHB_LS_READY:1;
    uint16_t PHC_HS_READY:1;
    uint16_t PHC_LS_READY:1;
    uint16_t reserved: 4;
}tMuxInputs_Bits;

typedef union
{
    uint16_t all;
    tMuxInputs_Bits bit;
}tMuxInputs;

/*      Public Variable Declarations
*/

extern t32ByteBits G_tGpioInputs;

/*      Public ROM Constants
*/

/*      Public Defines
*/
/* MCU PANEL   ACT_ID      PRGM3   PRGM4   PRGM5   PRGM6  |  VALUE
* -----------------------------------------------------------------
*  LIB         L           0       1       0       1      |    5
*  LIB         R           0       1       1       0      |    6
*  RIB         L           1       0       0       1      |    9
*  RIB         R           1       0       1       0      |   10
*  LOB         L           1       0       1       1      |   11
*  LOB         R           1       1       0       0      |   12
*  ROB         L           1       1       0       1      |   13
*  ROB         R           1       1       1       0      |   14    */
#define MCU_LIB_L   (5U)
#define MCU_LIB_R   (6U)
#define MCU_RIB_L   (9U)
#define MCU_RIB_R   (10U)
#define MCU_LOB_L   (11U)
#define MCU_LOB_R   (12U)
#define MCU_ROB_L   (13U)
#define MCU_ROB_R   (14U)

#define DEBOUNCED_INPUTS_MAX    32      /* Allows for up to 32 discrete inputs*/
#define DEBOUNCE_TIME           6       /* in ms, assuming Gpio_CaptureDiscrete() is called every 1 ms*/

/* 5V reference supply min is 4.95V. At 4.95V the 5V_SENSE is 2.475V.
 * 2.475V/3V * 4095 = 3378 ADC Cnts. */
#define PRGM_PIN_READ_LEVEL		3378

#define START_GPIO_PIN      9U      /* Start pushbutton GPIO number */
#define STOP_GPIO_PIN       7U      /* Stop pushbutton GPIO number */

/* External Interrupt Edge Detection Polarity Definitions */
#define XINT_POLARITY_POS_EDGE      1U
#define XINT_POLARITY_NEG_EDGE      2U
#define XINT_POLARITY_BOTH_EDGES    3U

/* ECAP ECTTL1 Definitions*/
#define ECAP_DISABLE_PRESCALER  0U
#define ECAP_FREE_SOFT_FREE_RUN 3U

/* ECAP Capture Load Enable Definitions */
#define ECAP_CAP_LOAD_DISABLE       0U  /* Disables CAP1-4 register load at capture event time */
#define ECAP_CAP_LOAD_ENABLE        1U  /* Enables CAP1-4 register load at capture event time */

/* ECAP Edge Detection Polarity Definitions */
#define ECAP_POLARITY_RISING_EDGE   0U
#define ECAP_POLARITY_FALLING_EDGE  1U

/* ECAP Counter Reset on Capture Event Definitions */
#define ECAP_NO_RESET_CTR_ON_CAP_EVENT  0U
#define ECAP_RESET_CTR_ON_CAP_EVENT     1U

/* ECAP ECCTL2 Definitions */

/* ECAP CONT_ONESHT Continuous or One-Shot Mode definitions */
#define ECAP_CONTINUOUS_MODE    0U
#define ECAP_ONESHOT_MODE       1U

/* ECAP STOP_WRAP Stop / Wrap after Capture Event definitions */
#define ECAP_STOP_WRAP_AFTER_CAP1_EVENT     0U
#define ECAP_STOP_WRAP_AFTER_CAP2_EVENT     1U
#define ECAP_STOP_WRAP_AFTER_CAP3_EVENT     2U
#define ECAP_STOP_WRAP_AFTER_CAP4_EVENT     3U

/* ECAP TSCTRSTOP Time Stamp Counter Stop Definitions */
#define ECAP_TSCTRSTOP_STOP     0U
#define ECAP_TSCTRSTOP_START    1U

/* ECAP SYNCI_EN Sync-In Select mode definitions */
#define ECAP_SYNCCI_EN_DISABLE  0U
#define ECAP_SYNCCI_EN_ENABLE   1U

/* ECAP SYNCO_SEL Sync-Out Select Definitions */
#define ECAP_SYNCO_SEL_SYNC_IN  0U
#define ECAP_SYNCO_SEL_PRD      1U
#define ECAP_SYCNO_SEL_DISABLE  2U

/* ECAP CAP_APWM Capture or Output Compare PWM Operating Mode Select Definitions */
#define ECAP_CAP_APWM_CAPTURE_MODE  0U
#define ECAP_CAP_APWM_APWM_MODE     1U

/* ECAP ECEINT Definitions */

/* ECAP CEVT / CTROVF / CTR_EQ_PRD / CTR_EQ_CMP Interrupt Definitions */
#define ECAP_INTERRUPT_DISABLE  0U
#define ECAP_INTERRUPT_ENABLE   1U

/* Defines for I/O Types */

/* GPyMUXn GPIO MUX Defines */
#define GPIO_MUX_TYPE_0         0U
#define GPIO_MUX_TYPE_1         1U
#define GPIO_MUX_TYPE_2         2U
#define GPIO_MUX_TYPE_3         3U

/* GPyGMUXn GPIO MUX Group Defines */
#define GPIO_MUX_GRP_0          0U
#define GPIO_MUX_GRP_1          1U
#define GPIO_MUX_GRP_2          2U
#define GPIO_MUX_GRP_3          3U

/* GPyDIR GPIO Direction Defines */
#define GPIO_INPUT              0U
#define GPIO_OUTPUT             1U

/* GPyPUD GPIO Pull-Up Disable Defines */
#define GPIO_ENABLE_PULLUP      0U
#define GPIO_DISABLE_PULLUP     1U

/* GPyODR GPIO Open Drain Defines */
#define GPIO_NORMAL_OUTPUT      0U
#define GPIO_OPEN_DRAIN_OUTPUT  1U

/* GPyQSEL GPIO Qualifier Select Defines */
#define GPIO_SYNC               0U
#define GPIO_QUAL3              1U
#define GPIO_QUAL6              2U
#define GPIO_ASYNC              3U

/* GP Output Macros 
  GP Output Macros: Gets
Writing to the DAT register sets the corresponding state of any I/O signal that is configured as an
output.
 
When using the GPxDAT register to change the level of an output pin, you
should be cautious not to accidentally change the level of another pin.  The problem can occur if
another I/O port A signal changes the level between the read and the write
stage of the instruction. If this happens, the signal level that changed is
overwritten by the original value read during the read stage of the instruction.
You can avoid this scenario by using the GPxSET, GPxCLEAR, and GPxTOGGLE registers instead.
*/
/* REWORK FOR NEW HARDWARE */
//#define MTR_DIR         GpioDataRegs.GPBDAT.bit.GPIOB10


#define DFT_SCI_C_TX_EN     GpioDataRegs.GPADAT.bit.GPIO20

/*Update for GPD LITE MCU*/
#define MSB_OUT             GpioDataRegs.GPADAT.bit.GPIO16
#define BRAKE_CNTL_CHA      GpioDataRegs.GPADAT.bit.GPIO17
#define SHDN_BRK_CTRL_CHA   GpioDataRegs.GPBDAT.bit.GPIO54
#define CH_STATUS           GpioDataRegs.GPADAT.bit.GPIO30
#define NS_CNTL_PHA_CHA     GpioDataRegs.GPADAT.bit.GPIO14
#define NS_CNTL_PHC_CHA     GpioDataRegs.GPADAT.bit.GPIO15
#define BUS_SW_ENABLE       GpioDataRegs.GPEDAT.bit.GPIO133
#define VC_MODE_CHA         GpioDataRegs.GPADAT.bit.GPIO28
#define v270_BUS_CNTL_CHA   GpioDataRegs.GPADAT.bit.GPIO23
#define PHA_HS_CNTL_CHA     GpioDataRegs.GPADAT.bit.GPIO0
#define PHA_LS_CNTL_CHA     GpioDataRegs.GPADAT.bit.GPIO1
#define PHB_HS_CNTL_CHA     GpioDataRegs.GPADAT.bit.GPIO2
#define PHB_LS_CNTL_CHA     GpioDataRegs.GPADAT.bit.GPIO3
#define PHC_HS_CNTL_CHA     GpioDataRegs.GPADAT.bit.GPIO4
#define PHC_LS_CNTL_CHA     GpioDataRegs.GPADAT.bit.GPIO5
#define GD_RESET            GpioDataRegs.GPBDAT.bit.GPIO36
#define ENC_ZERO_SET_CNTL   GpioDataRegs.GPBDAT.bit.GPIO53
#define CS_270VBUS_VSENSE   GpioDataRegs.GPCDAT.bit.GPIO91
#define CS_28VBUS_VSENSE    GpioDataRegs.GPCDAT.bit.GPIO93
#define CS_IBRK_SENSE       GpioDataRegs.GPCDAT.bit.GPIO94
#define v28_BUS_CNTL_CHA    GpioDataRegs.GPDDAT.bit.GPIO99
#define INRUSH_CTR_CHA      GpioDataRegs.GPBDAT.bit.GPIO59

/* GP Output Macros: Sets
 If the corresponding pin is configured as an output, then writing a 1 to that bit 
 in the set register will pull the corresponding pin high.  Writing a 0 to a bit has no effect.
*/
/* REWORK FOR NEW HARDWARE */
#if defined(DRV8312_DEV_KIT)
#define LED3_set()            GpioDataRegs.GPBSET.bit.GPIO34 = 1U;  /* Set output high for LED2 */
#define LED10_set()           GpioDataRegs.GPBSET.bit.GPIO40 = 1U;  /* Set output high for LED10 */
#define TEST_OUTPUT_GPIO17_set() GpioDataRegs.GPASET.bit.GPIO17 = 1U; /* Set output high for GPIO17 */
#endif

#define MSB_OUT_set()         GpioDataRegs.GPASET.bit.GPIO16 = 1
#define PHA_HS_CNTL_CHA_set()     GpioDataRegs.GPASET.bit.GPIO4 = 1
#define PHB_HS_CNTL_CHA_set()     GpioDataRegs.GPASET.bit.GPIO2 = 1
#define PHC_HS_CNTL_CHA_set()     GpioDataRegs.GPASET.bit.GPIO0 = 1
#define PHA_LS_CNTL_CHA_set()     GpioDataRegs.GPASET.bit.GPIO5 = 1
#define PHB_LS_CNTL_CHA_set()     GpioDataRegs.GPASET.bit.GPIO3 = 1
#define PHC_LS_CNTL_CHA_set()     GpioDataRegs.GPASET.bit.GPIO1 = 1
#define DFT_SCI_C_TX_EN_set()     GpioDataRegs.GPASET.bit.GPIO20 = 1
/*Update for GPD LITE MCU*/
#define NS_CNTL_PHA_CHA_set() GpioDataRegs.GPASET.bit.GPIO14 = 1
#define NS_CNTL_PHC_CHA_set() GpioDataRegs.GPASET.bit.GPIO15 = 1
#define BRAKE_CNTL_set()      GpioDataRegs.GPASET.bit.GPIO17 = 1
#define SHDN_BRK_CTRL_set()   GpioDataRegs.GPBSET.bit.GPIO54 = 1
#define CH_STATUS_set()       GpioDataRegs.GPASET.bit.GPIO30 = 1
#define LED1_set()                GpioDataRegs.GPCSET.bit.GPIO60 = 1
#define nCS_270VBUS_VSENSE_set()  GpioDataRegs.GPCSET.bit.GPIO91 = 1
#define nCS_28VBUS_VSENSE_set()   GpioDataRegs.GPCSET.bit.GPIO93 = 1
#define nCS_IBRK_SENSE_set()      GpioDataRegs.GPCSET.bit.GPIO94 = 1
#define GD_RESET_set()            GpioDataRegs.GPBSET.bit.GPIO36 = 1
#define ENC_ZERO_SET_CNTL_set()   GpioDataRegs.GPBSET.bit.GPIO53 = 1

/*  GP Output Macros: Clears
The clear registers are write-only registers that read back 0. If the corresponding 
pin is configured as an output, then writing a 1 to that bit in the clear register 
will pull the corresponding pin low. Writing a 0 to a bit has no effect.
*/
/* REWORK FOR NEW HARDWARE */
#if defined(DRV8312_DEV_KIT)
#define LED3_clr()            GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1U;  /* Clear output low for LED2 */
#define LED10_clr()           GpioDataRegs.GPBCLEAR.bit.GPIO40 = 1U;  /* Clear output low for LED10 */
#define TEST_OUTPUT_GPIO17_clr() GpioDataRegs.GPACLEAR.bit.GPIO17 = 1U; /* Set output high for GPIO17 */
#endif

#if defined(DRV8312_DEV_KIT)
#define TEST_OUTPUT_GPIO31_clr() GpioDataRegs.GPACLEAR.bit.GPIO31 = 1U; /* Set output high for GPIO17 */
#endif

#define MSB_OUT_clr()         GpioDataRegs.GPACLEAR.bit.GPIO16 = 1
#define PHA_HS_CNTL_CHA_clr()     GpioDataRegs.GPACLEAR.bit.GPIO4 = 1
#define PHB_HS_CNTL_CHA_clr()     GpioDataRegs.GPACLEAR.bit.GPIO2 = 1
#define PHC_HS_CNTL_CHA_clr()     GpioDataRegs.GPACLEAR.bit.GPIO0 = 1
#define PHA_LS_CNTL_CHA_clr()     GpioDataRegs.GPACLEAR.bit.GPIO5 = 1
#define PHB_LS_CNTL_CHA_clr()     GpioDataRegs.GPACLEAR.bit.GPIO3 = 1
#define PHC_LS_CNTL_CHA_clr()     GpioDataRegs.GPACLEAR.bit.GPIO1 = 1
#define DFT_SCI_C_TX_EN_clr()     GpioDataRegs.GPACLEAR.bit.GPIO20 = 1
/*Update for GPD LITE MCU*/
#define NS_CNTL_PHA_CHA_clr() GpioDataRegs.GPACLEAR.bit.GPIO14 = 1
#define NS_CNTL_PHC_CHA_clr() GpioDataRegs.GPACLEAR.bit.GPIO15 = 1
#define BRAKE_CNTL_clr()      GpioDataRegs.GPACLEAR.bit.GPIO17 = 1
#define SHDN_BRK_CTRL_clr()   GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1
#define CH_STATUS_clr()       GpioDataRegs.GPACLEAR.bit.GPIO30 = 1
#define LED1_clr()                GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1
#define nCS_270VBUS_VSENSE_clr()  GpioDataRegs.GPCCLEAR.bit.GPIO91 = 1
#define nCS_28VBUS_VSENSE_clr()   GpioDataRegs.GPCCLEAR.bit.GPIO93 = 1
#define nCS_IBRK_SENSE_clr()      GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1
#define GD_RESET_clr()            GpioDataRegs.GPBCLEAR.bit.GPIO36 = 1
#define ENC_ZERO_SET_CNTL_clr()   GpioDataRegs.GPBCLEAR.bit.GPIO53 = 1
/*  GP Output Macros: Toggles
The toggle registers are write-only registers that read back 0. If the corresponding 
pin is configured as an output, then writing a 1 to that bit in the toggle register 
will pull the corresponding pin in the opposite direction. That is, if the output pin 
is low, writing a 1 to the corresponding bit in the toggle register will pull the pin 
high. Likewise, if the output pin is high, writing a 1 to the corresponding bit in 
the toggle register will pull the pin low. Writing a 0 to a bit has no effect.
*/
/* REWORK FOR NEW HARDWARE */
#if defined(DRV8312_DEV_KIT)
#define LED3_tgl()            GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1U;  /* Toggle output for LED2 */
#define LED10_tgl()           GpioDataRegs.GPBTOGGLE.bit.GPIO40 = 1U;  /* Toggle output for LED10 */
#define TEST_OUTPUT_GPIO17_tgl() GpioDataRegs.GPATOGGLE.bit.GPIO17 = 1U; /* Toggle output high for GPIO17 */
#endif

#if defined(DRV8312_DEV_KIT)
#define TEST_OUTPUT_GPIO31_tgl() GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1U; /* Set output high for GPIO17 */
#endif

#define MSB_OUT_tgl()       GpioDataRegs.GPATOGGLE.bit.GPIO16 = 1
#define PHA_HS_CNTL_CHA_tgl()     GpioDataRegs.GPATOGGLE.bit.GPIO4 = 1
#define PHB_HS_CNTL_CHA_tgl()     GpioDataRegs.GPATOGGLE.bit.GPIO2 = 1
#define PHC_HS_CNTL_CHA_tgl()     GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1
#define PHA_LS_CNTL_CHA_tgl()     GpioDataRegs.GPATOGGLE.bit.GPIO5 = 1
#define PHB_LS_CNTL_CHA_tgl()     GpioDataRegs.GPATOGGLE.bit.GPIO3 = 1
#define PHC_LS_CNTL_CHA_tgl()     GpioDataRegs.GPATOGGLE.bit.GPIO1 = 1
/*Commenting out the as in the MCU HW DFT_SCI_A_TX_EN will be replaced by SCI_C_TX_EN*/
#if defined(DRV8312_DEV_KIT)
#define DFT_SCI_A_TX_EN_tgl() GpioDataRegs.GPATOGGLE.bit.GPIO28 = 1
#endif
/*Update for GPD LITE MCU*/
#define NS_CNTL_PHA_CHA_tgl() GpioDataRegs.GPATOGGLE.bit.GPIO14 = 1
#define NS_CNTL_PHC_CHA_tgl() GpioDataRegs.GPATOGGLE.bit.GPIO15 = 1
#define BRAKE_CNTL_tgl()      GpioDataRegs.GPATOGGLE.bit.GPIO17 = 1
#define SHDN_BRK_CTRL_tgl()   GpioDataRegs.GPBTOGGLE.bit.GPIO54 = 1
#define CH_STATUS_tgl()       GpioDataRegs.GPATOGGLE.bit.GPIO30 = 1
#define LED1_tgl()            GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1
/*  GP Input Macros
These macros provide access to Discrete Inputs after debounce processing.  The Gpio_Capture() routine
sets up the bit arrangement within the #G_tGpioInputs word, and performs the debounce logic.
*/
/* GP Input Macros RAW Data */
#define PRGM1_RAW               GpioDataRegs.GPCDAT.bit.GPIO64   /* GPIO64 */
#define PRGM2_RAW               GpioDataRegs.GPCDAT.bit.GPIO65   /* GPIO65 */
#define PRGM3_RAW               GpioDataRegs.GPCDAT.bit.GPIO66   /* GPIO66 */
#define PRGM4_RAW               GpioDataRegs.GPCDAT.bit.GPIO67   /* GPIO67 */
#define PRGM5_RAW               GpioDataRegs.GPCDAT.bit.GPIO68   /* GPIO68 */
#define PRGM6_RAW               GpioDataRegs.GPCDAT.bit.GPIO84   /* GPIO84 */
#define RIG_N_RAW               GpioDataRegs.GPADAT.bit.GPIO29   /* GPIO29 */
#define CHX_STATUS_RAW          GpioDataRegs.GPBDAT.bit.GPIO32   /* GPIO32 */
#define GD_FAULT_RAW            GpioDataRegs.GPBDAT.bit.GPIO33   /* GPIO33 */
#define GD_READY_RAW            GpioDataRegs.GPBDAT.bit.GPIO35   /* GPIO35 */
#define MTR_EN_CHA_RAW          GpioDataRegs.GPADAT.bit.GPIO25   /* GPIO25 */
#define GFD_I_SENSE_RAW         GpioDataRegs.GPBDAT.bit.GPIO57   /* GPIO57 */
#define nFAULT_BRK_HS_OUT_RAW   GpioDataRegs.GPBDAT.bit.GPIO58   /* GPIO58 */


#define PRGM1               (G_tGpioInputs.bit.b0)          /* GPIO64 */
#define PRGM2               (G_tGpioInputs.bit.b1)          /* GPIO65 */
#define PRGM3               (G_tGpioInputs.bit.b2)          /* GPIO66 */
#define PRGM4               (G_tGpioInputs.bit.b3)          /* GPIO67 */
#define PRGM5               (G_tGpioInputs.bit.b4)          /* GPIO68 */
#define PRGM6               (G_tGpioInputs.bit.b5)          /* GPIO84 */
#define RIG_N               (G_tGpioInputs.bit.b6)          /* GPIO29 */
#define CHX_STATUS          (G_tGpioInputs.bit.b7)          /* GPIO32 */
#define GD_FAULT            (G_tGpioInputs.bit.b8)          /* GPIO33 */
#define GD_READY            (G_tGpioInputs.bit.b9)          /* GPIO35 */
#define MTR_EN_CHA          (G_tGpioInputs.bit.b10)         /* GPIO25 */
#define CHA_HS1             (G_tGpioInputs.bit.b11)         /* GPIO6  */
#define CHA_HS2             (G_tGpioInputs.bit.b12)         /* GPIO7  */
#define CHA_HS3             (G_tGpioInputs.bit.b13)         /* GPIO8  */
#define CHB_ICC_HS1         (G_tGpioInputs.bit.b14)         /* GPIO9  */
#define CHB_ICC_HS2         (G_tGpioInputs.bit.b15)         /* GPIO10 */
#define CHB_ICC_HS3         (G_tGpioInputs.bit.b16)         /* GPIO11 */
#define GFD_I_SENSE         (G_tGpioInputs.bit.b17)         /* GPIO57 */
#define nFAULT_BRK_HS_OUT   (G_tGpioInputs.bit.b18)         /* GPIO58 */

#if defined(DRV8312_DEV_KIT)
/* 2837xD Development Kit Hardware */
#define CHA_HS1_RAW            GpioDataRegs.GPADAT.bit.GPIO24    /* HALL A */
#define CHA_HS2_RAW            GpioDataRegs.GPADAT.bit.GPIO25    /* HALL B */
#define CHA_HS3_RAW            GpioDataRegs.GPADAT.bit.GPIO26    /* HALL C */
#define CHB_ICC_HS1_RAW        CHA_HS1_RAW  /* Secondary (Cross) Side Hall Effect Sensor 1. Doesn't exist on Dev Kit. Set to HS1 */
#define CHB_ICC_HS2_RAW        CHA_HS2_RAW  /* Secondary (Cross) Side Hall Effect Sensor 2. Doesn't exist on Dev Kit. Set to HS2 */
#define CHB_ICC_HS3_RAW        CHA_HS3_RAW  /* Secondary (Cross) Side Hall Effect Sensor 3. Doesn't exist on Dev Kit. Set to HS3 */

#define START_RAW                 GpioDataRegs.GPADAT.bit.GPIO9
#define STOP_RAW                  GpioDataRegs.GPADAT.bit.GPIO7
#define LED3_RAW                  GpioDataRegs.GPBDAT.bit.GPIO34
#define LED10_RAW                 GpioDataRegs.GPBDAT.bit.GPIO40
#else
    /* MCU HW */
#define CHA_HS1_RAW                 GpioDataRegs.GPADAT.bit.GPIO6   /* On-Side Hall Effect Sensor 1 */
#define CHA_HS2_RAW                 GpioDataRegs.GPADAT.bit.GPIO7   /* On-Side Hall Effect Sensor 2 */
#define CHA_HS3_RAW                 GpioDataRegs.GPADAT.bit.GPIO8   /* On-Side Hall Effect Sensor 3 */
#define CHB_ICC_HS1_RAW             GpioDataRegs.GPADAT.bit.GPIO9   /* Cross-Side Hall Effect Sensor 1 */
#define CHB_ICC_HS2_RAW             GpioDataRegs.GPADAT.bit.GPIO10  /* Cross-Side Hall Effect Sensor 2 */
#define CHB_ICC_HS3_RAW             GpioDataRegs.GPADAT.bit.GPIO11  /* Cross-Side Hall Effect Sensor 3 */
#define HALL_MASK                   (0x01C0)    /* HallGpio.2-0 = GPIO8-GPIO6 */
#define HALL_SHIFT                  (6)         /* Shift GPIO8-6 down to bits 2-0 */
#endif

extern bool bGFD_I_SENSE_ISR;

/* Monitor 0x50 GPIO Defines */
//#define GPIO_A_BIT50_mux()      GpioMuxRegs.GPAMUX.all = 0x0740
//#define GPIO_A_BIT50_dir()      GpioMuxRegs.GPADIR.all = 0x003F

/* Monitor 0x25 GPIO Defines */
//#define GPIO_A_BIT25_mux()      GpioMuxRegs.GPAMUX.all = 0x073F
//#define GPIO_A_BIT25_dir()      GpioMuxRegs.GPADIR.all = 0x0040

/* GPIO A Normal settings post BIT tests */
//#define GPIO_A_mux()            GpioMuxRegs.GPAMUX.all = 0x077F
//#define GPIO_A_dir()            GpioMuxRegs.GPADIR.all = 0x0000

/**
  defgroup gpio GPIO Driver
  
  brief The GPIO driver is an abstraction for capturing discrete input signals and accessing
    discrete output signals.  Debounce logic is included for discrete inputs.
    
    Input discretes are captured to a single bitmapped word that can be accessed through macros
    defined in gpio.h.  Output discretes may be set/cleared/toggled through macros that are
    defined in gpio.h.

*/


/*      Public Interface Function Prototypes
*/

void Gpio_Init( void );
void Gpio_Capture( void );
void Gpio_EstablishChannel(void);
bool_t Gpio_SetChannel(bool_t bGetFromNvm);
bool_t Gpio_SetPanelType(bool_t bGetFromNvm);
__interrupt void GFD_I_SENSE_ISR( void );

#endif
/* end gpio.h*/

