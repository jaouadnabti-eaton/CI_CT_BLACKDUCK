/****************************************************************************************************
*  File name: gpio.c
*
*  Purpose: Provides facilities for accessing discrete inputs and outputs.
*    This file contains the API definition for the GPIO facilities.  Debounce
*    logic is included for all digital inputs, within the Gpio_Capture() routine.
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

/*      Include Files
*/
#include "F2837xD_device.h"
//#include "F2837xD_Examples.h"
#include "gpio.h"
#include "actuation.h"
#include "parameter.h"
#include "timer.h"
#include "mcu.h"
#include "nvm.h"
#include "motor.h"
#include "spi.h"
#include "adc.h"
//#include "panel.h"
#include "bitmonitors.h"
#include "A825_MsgInfo.h"
/*      Local Type Definitions
*/

/*      Local Defines
*/

/*      Global Variables
*/
bool G_bChannelA = true;                    /* Channel A Boolean Identification. */
eChId_t G_eChId = INVALID_CHANNEL_ID;       /* Channel Identification */
eActId_t G_eActId = INVALID_ACTUATOR_ID;    /* LRU Identification */
eLruId_t G_eLruId = INVALID_LRU;            /* Actuator Identification */
eActuator_t G_eActuatorNumber = INVALID_ACTUATOR;   /* Actuator Number */
eActuator_t G_eActuatorNumberX = INVALID_ACTUATOR;  /* Actuator Number for Opposite Side MCU */

/* 32-bit structure holding all stable discrete input values */
t32ByteBits G_tGpioInputs;
/*Initializing all the Mux input to 1 as PHx_xS_nFAULT & PHx_xS_READY will be 1 when there is no Fault in Gate Drivers*/
tMuxInputs MuxInputs = {0x0FFFU};

/*      Local ROM Constants
*/

/* The debounce mask sets certain bits in the "discrete input" word for debouncing.
   This debounce mask is not debouncing X_ENBL_IN or 200_MS_MON signals even though
   documentation says that they should be debounced
*/
#define DEBOUNCE_MASK		0x7FF00E7DL
#define	FSL_DEBOUNCE_MASK	0x7FF00010L

#define GFD_I_SENSE_PIN     57U         /* GFD_I_SENSE GPIO number */

/*      Local Variable Declarations
*/
/* Debounce counters are used to time debouncing for discrete inputs.
   When the count exceeds DEBOUNCE_TIME, the signal is considered stable.
*/
unsigned int DebounceCount[DEBOUNCED_INPUTS_MAX];
bool xSideBootStatus = false;
Timer_t SyncTimeoutTimer = TIMER_DEFAULTS;

bool bGFD_I_SENSE_ISR = True;

/* 32-bit word holding intermediate discrete input values, for debouncing purposes*/
unsigned long Debounced = 0; 

/*      Local Function Prototypes
*/
void Gpio_ChannelSync(void);
void Gpio_SetActuatorNumber(void);

#ifdef CPU1
/*      Function Definitions
*/

/**
    Initialization routine for GPIO driver.

    Purpose:
        Configures GPIO pins for proper operation of the signals, configuring pins
        as either input or output.

    Parameters:
        None.

    Global Data Referenced:
    
    return  void
    
    Preconditions and Assumptions:
        This routine should be called only once, prior to the scheduler loop.
        This routine should be called prior to other peripheral "Init" routines, 
        so that the peripherals may override the GPIO pin settings defined herein, 
        if necessary.
*/
void Gpio_Init( void )
{
    volatile uint32_t *gpioBaseAddr;
    uint16_t regOffset;
    uint16_t i;
	G_tGpioInputs.all = 0UL; /* Init Global Inputs to zero */

    EALLOW;

    /* Unlock all pins for configuration */
    GpioCtrlRegs.GPALOCK.all = 0x00000000;
    GpioCtrlRegs.GPBLOCK.all = 0x00000000;
    GpioCtrlRegs.GPCLOCK.all = 0x00000000;
    GpioCtrlRegs.GPDLOCK.all = 0x00000000;
    GpioCtrlRegs.GPELOCK.all = 0x00000000;
    GpioCtrlRegs.GPFLOCK.all = 0x00000000;

    /* Fill all registers with zeros. Writing to each register separately
     * for six GPIO modules would make this function *very* long. */
    gpioBaseAddr = (uint32_t *)&GpioCtrlRegs;
    for (regOffset = 0; regOffset < sizeof(GpioCtrlRegs)/2; regOffset++)
    {
        /* Hack to avoid enabling pull-ups on all pins. GPyPUD is offset
         * /0x0C in each register group of 0x40 words. Since this is a
         * 32-bit pointer, the addresses must be divided by 2. */
        if (regOffset % (0x40/2) != (0x0C/2))
        {
            gpioBaseAddr[regOffset] = 0x00000000;
        }
    }

    /* Clear our the data registers on init */
    gpioBaseAddr = (Uint32 *)&GpioDataRegs;
    for (regOffset = 0; regOffset < sizeof(GpioDataRegs)/2; regOffset++)
    {
        gpioBaseAddr[regOffset] = 0x00000000;
    }
#if defined(DRV8312_DEV_KIT)
    /* Setup GPIO17 as an output used for Debug and Timing purposes. was 17 */
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPADIR.bit.GPIO31 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPAPUD.bit.GPIO31 = GPIO_DISABLE_PULLUP;   /* Disable the pullups */
    GpioCtrlRegs.GPAODR.bit.GPIO31 = GPIO_NORMAL_OUTPUT;    /* Configure as push-pull normal output */
    TEST_OUTPUT_GPIO31_clr();                               /* Initialize the GP17 Output */
#endif

#if defined(DRV8312_DEV_KIT)
    /* Setup GPIO_31 to control LED3 on Control Card */
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = GPIO_DISABLE_PULLUP;   /* Disable the pullups */
    GpioCtrlRegs.GPBODR.bit.GPIO34 = GPIO_NORMAL_OUTPUT;    /* Configure as push-pull normal output */
    LED3_set();                                             /* Initialize LED3 */

    /* Setup GPIO_40 to control LED10 on DRV8312 Card */
    GpioCtrlRegs.GPBGMUX1.bit.GPIO40 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPBDIR.bit.GPIO40 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPBPUD.bit.GPIO40 = GPIO_DISABLE_PULLUP;   /* Disable the pullups */
    GpioCtrlRegs.GPBODR.bit.GPIO40 = GPIO_NORMAL_OUTPUT;    /* Configure as push-pull normal output */
    LED10_clr();                                            /* Initialize LED10 */

    /* Setup GPIO17 as an output used for Debug and Timing purposes. was 17 */
    GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPADIR.bit.GPIO17 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = GPIO_DISABLE_PULLUP;   /* Disable the pullups */
    GpioCtrlRegs.GPAODR.bit.GPIO17 = GPIO_NORMAL_OUTPUT;    /* Configure as push-pull normal output */
    TEST_OUTPUT_GPIO17_clr();                               /* Initialize the GP17 Output */

    /* looks like SPI/CAN isolator is the only pins to connect to for SCI configuration.
     * GPIO17 is the only output here and cannot be configured to SCI => SPI/CAN isolator cannot be used for SCI
     * GPIO22:  SCITXDB (O) :  STATUS LED:  J5P8
     * CPIO23:  SCIRXDB (I) :  QEPI:  J4P3
     */

    /* Make GPIO20 an input for testing purposes */
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = GPIO_ENABLE_PULLUP;    /* Enable pullup on GPIO20 */
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = GPIO_MUX_TYPE_0;      /* GPIO20 = GPIO20 */
    GpioCtrlRegs.GPADIR.bit.GPIO20 = GPIO_INPUT;            /* GPIO20 = input */

#else
    /* MCU HW */
    /* PRGM_1, PRGM_2, PRGM_3, PRGM_4, PRGM_5, PRGM_6 Configuration */
    /* Setup GPIO_64 as PRGM_1 input */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO64 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPCMUX1.bit.GPIO64 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPCDIR.bit.GPIO64 = GPIO_INPUT;            /* Set as an input */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = GPIO_ASYNC;          /* Set input as asynchronous */

    /* Setup GPIO_65 as PRGM_2 input */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPCDIR.bit.GPIO65 = GPIO_INPUT;            /* Set as an input */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = GPIO_ASYNC;          /* Set input as asynchronous */

    /* Setup GPIO_66 as PRGM_3 input */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO66 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPCDIR.bit.GPIO66 = GPIO_INPUT;            /* Set as an input */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO66 = GPIO_ASYNC;          /* Set input as asynchronous */

    /* Setup GPIO_67 as PRGM_4 input */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO67 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPCDIR.bit.GPIO67 = GPIO_INPUT;            /* Set as an input */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO67 = GPIO_ASYNC;          /* Set input as asynchronous */

    /* Setup GPIO_68 as PRGM_5 input */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO68 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPCMUX1.bit.GPIO68 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPCDIR.bit.GPIO68 = GPIO_INPUT;            /* Set as an input */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO68 = GPIO_ASYNC;          /* Set input as asynchronous */

    /* Setup GPIO_84 as PRGM_6 input */
    GpioCtrlRegs.GPCGMUX2.bit.GPIO84 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPCMUX2.bit.GPIO84 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPCDIR.bit.GPIO84 = GPIO_INPUT;            /* Set as an input */
    GpioCtrlRegs.GPCQSEL2.bit.GPIO84 = GPIO_ASYNC;          /* Set input as asynchronous */

	/*Configure GPIO17(EPWM9B) as BRAKE_CNTL*/
    GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = GPIO_MUX_GRP_1;      /*Select group 1*/
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = GPIO_MUX_TYPE_1;      /*Set as EPWM9B pin*/
    GpioCtrlRegs.GPADIR.bit.GPIO17 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = GPIO_DISABLE_PULLUP;   /*Disable pull-up on GPIO17 (EPWM9B)*/
    GpioCtrlRegs.GPAODR.bit.GPIO17 = GPIO_NORMAL_OUTPUT;    /*Configure GPIO17 (EPWM9B) as push-pull normal output*/
    BRAKE_CNTL_clr();                                       /*Setting BRAKE_CNTL to 0 so that no current is drawn by brake circuit in GPD-Lite HW*/

    /*Configure GPIO30 as CH_STATUS*/
    GpioCtrlRegs.GPAGMUX2.bit.GPIO30 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPADIR.bit.GPIO30 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = GPIO_DISABLE_PULLUP;   /*Disable pull-up on GPIO30*/
    GpioCtrlRegs.GPAODR.bit.GPIO30 = GPIO_NORMAL_OUTPUT;    /*Configure GPIO30 as push-pull normal output*/

    /*Configure GPIO59 as INRUSH_CTR*/
    GpioCtrlRegs.GPBGMUX2.bit.GPIO59 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPBDIR.bit.GPIO59 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPBPUD.bit.GPIO59 = GPIO_DISABLE_PULLUP;   /*Disable pull-up on GPIO59*/
    GpioCtrlRegs.GPBODR.bit.GPIO59 = GPIO_NORMAL_OUTPUT;    /*Configure GPIO59 as push-pull normal output*/

    /*Configure GPIO54 as SHDN_BRK_CTRL*/
    GpioCtrlRegs.GPBGMUX2.bit.GPIO54 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPBDIR.bit.GPIO54 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = GPIO_DISABLE_PULLUP;   /*Disable pull-up on GPIO54*/
    GpioCtrlRegs.GPBODR.bit.GPIO54 = GPIO_NORMAL_OUTPUT;    /*Configure GPIO54 as push-pull normal output*/

    /*Configure GPIO35 as GD_READY*/
    GpioCtrlRegs.GPBGMUX1.bit.GPIO35 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPBDIR.bit.GPIO35 = GPIO_INPUT;            /* Set as an input */
    GpioCtrlRegs.GPBQSEL1.bit.GPIO35 = GPIO_ASYNC;          /* Set input as asynchronous */

    /*Configure GPIO36 as GD_RESET*/
    GpioCtrlRegs.GPBGMUX1.bit.GPIO36 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBGMUX1.bit.GPIO36 = GPIO_MUX_TYPE_0;     /* Set as GPIO pin type */
    GpioCtrlRegs.GPBDIR.bit.GPIO36 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPBPUD.bit.GPIO36 = GPIO_DISABLE_PULLUP;   /* Disable pull-up on GPIO36*/
    GpioCtrlRegs.GPBODR.bit.GPIO36 = GPIO_NORMAL_OUTPUT;    /* Configure GPIO36 as push-pull normal output*/
    GD_RESET_set();                                         /* Set the GD_RESET output high for normal operation of gate drivers */

    /*Configure GPIO53 as ENC_ZERO_SET_CNTL*/
    GpioCtrlRegs.GPBGMUX2.bit.GPIO53 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPBDIR.bit.GPIO53 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPBPUD.bit.GPIO53 = GPIO_DISABLE_PULLUP;   /*Disable pull-up on GPIO53*/
    GpioCtrlRegs.GPBODR.bit.GPIO53 = GPIO_NORMAL_OUTPUT;    /*Configure GPIO53 as push-pull normal output*/

    /*Configure GPIO33 as GD_FAULT*/
    GpioCtrlRegs.GPBGMUX1.bit.GPIO33 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = GPIO_INPUT;            /* Set as an input */
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = GPIO_ASYNC;          /* Set input as asynchronous */

    /*Configure GPIO90 as A18*/
    GpioCtrlRegs.GPCGMUX2.bit.GPIO90 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPCMUX2.bit.GPIO90 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPCDIR.bit.GPIO90 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPCPUD.bit.GPIO90 = GPIO_DISABLE_PULLUP;   /*Disable pull-up on GPIO90*/
    GpioCtrlRegs.GPCODR.bit.GPIO90 = GPIO_NORMAL_OUTPUT;    /*Configure GPIO90 as push-pull normal output*/

    /*Configure GPIO91 as nCS_270VBUS_VSENSE*/
    GpioCtrlRegs.GPCGMUX2.bit.GPIO91 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPCMUX2.bit.GPIO91 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPCDIR.bit.GPIO91 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPCPUD.bit.GPIO91 = GPIO_DISABLE_PULLUP;   /*Disable pull-up on GPIO91*/
    GpioCtrlRegs.GPCODR.bit.GPIO91 = GPIO_NORMAL_OUTPUT;    /*Configure GPIO91 as push-pull normal output*/

    /*Configure GPIO93 as nCS_28VBUS_VSENSE*/
    GpioCtrlRegs.GPCGMUX2.bit.GPIO93 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPCMUX2.bit.GPIO93 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPCDIR.bit.GPIO93 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPCPUD.bit.GPIO93 = GPIO_DISABLE_PULLUP;   /*Disable pull-up on GPIO93*/
    GpioCtrlRegs.GPCODR.bit.GPIO93 = GPIO_NORMAL_OUTPUT;    /*Configure GPIO93 as push-pull normal output*/

    /*Configure GPIO94 as nCS_IBRK_SENSE*/
    GpioCtrlRegs.GPCGMUX2.bit.GPIO94 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPCMUX2.bit.GPIO94 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPCDIR.bit.GPIO94 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPCPUD.bit.GPIO94 = GPIO_DISABLE_PULLUP;   /*Disable pull-up on GPIO94*/
    GpioCtrlRegs.GPCODR.bit.GPIO94 = GPIO_NORMAL_OUTPUT;    /*Configure GPIO94 as push-pull normal output*/

    /*Configure GPIO60 as LED1*/
    GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPBDIR.bit.GPIO60 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = GPIO_DISABLE_PULLUP;   /*Disable pull-up on GPIO60*/
    GpioCtrlRegs.GPBODR.bit.GPIO60 = GPIO_NORMAL_OUTPUT;    /*Configure GPIO60 as push-pull normal output*/

    /*Configure GPIO61 for watchdog*/
    GpioCtrlRegs.GPBGMUX2.bit.GPIO61 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPBPUD.bit.GPIO61 = GPIO_DISABLE_PULLUP;   /*Disable pull-up on GPIO60*/
    GpioCtrlRegs.GPBODR.bit.GPIO61 = GPIO_NORMAL_OUTPUT;    /*Configure GPIO60 as push-pull normal output*/

    /* map the XINT1 interrupts to the GFD_I_SENSE ISR*/
    PieVectTable.XINT1_INT = &GFD_I_SENSE_ISR;
    /* Setup GPIO_57 as GFD_I_SENSE */
    GpioCtrlRegs.GPBGMUX2.bit.GPIO57 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPBDIR.bit.GPIO57 = GPIO_INPUT;            /* Set as an input */
    GpioCtrlRegs.GPBQSEL2.bit.GPIO57 = GPIO_ASYNC;          /* Set input as asynchronous */
    /* Set INPUT_XBAR_REGS for XINT1 for GPIO connected to detect GFD_I_SENSE low */
    InputXbarRegs.INPUT4SELECT = GFD_I_SENSE_PIN;    /* Configure INPUT 4 (XINT1) source of Input X-BAR detect GFD_I_SENSE low */
    /* Configure the External Interrupt Edge Detection for XINT1 */
    /* Only monitor for falling edges for GFD_I_SENSE signal. */
    XintRegs.XINT1CR.bit.POLARITY = XINT_POLARITY_NEG_EDGE; /* Falling edges generate interrupt */
    XintRegs.XINT1CR.bit.ENABLE = INTERRUPT_ENABLE;         /* Enable XINT1 */

    /* Setup GPIO_32 as CHX_STATUS (CHB_ICC_STATUS)*/
    GpioCtrlRegs.GPBGMUX1.bit.GPIO32 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = GPIO_INPUT;            /* Set as an input */
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = GPIO_ASYNC;          /* Set input as asynchronous */
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = GPIO_ENABLE_PULLUP;    /*Disable Pull-up*/

    /* Setup GPIO_29 as RIG_N */
    GpioCtrlRegs.GPAGMUX2.bit.GPIO29 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPADIR.bit.GPIO29 = GPIO_INPUT;            /* Set as an input */
    GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = GPIO_ASYNC;          /* Set input as asynchronous */
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = GPIO_ENABLE_PULLUP;    /*Enable Pull-up as RIG_N is not pulled up in Hardware*/

    /* Setup GPIO_58 as nFAULT_RK_HS_OUT */
    GpioCtrlRegs.GPBGMUX2.bit.GPIO58 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPBDIR.bit.GPIO58 = GPIO_INPUT;            /* Set as an input */
    GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = GPIO_ASYNC;          /* Set input as asynchronous */

#if defined(_HSIT)
    /*---------------------- CONFIGURE UPPER AND LOWER GATES DRIVER PINS ----------------------------*/
    /* Disable pull-up functionality for GPIO0/1 (Phase C) gate pins */
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO0 (EPWM1A) */
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO1 (EPWM1B) */
    /* Disable pull-up functionality for GPIO2/3 (Phase B) gate pins */
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO2 (EPWM2A) */
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO3 (EPWM2B) */
    /* Disable pull-up functionality for GPIO4/5 (Phase A) gate pins */
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO4 (EPWM3A) */
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO5 (EPWM3B) */

    /* Set GPIO0/1 (Phase C) gate pins as Push Pull Outputs */
    GpioCtrlRegs.GPADIR.bit.GPIO0 = GPIO_OUTPUT;            /* Configure GPIO0 (EPWM1A) as an output */
    GpioCtrlRegs.GPADIR.bit.GPIO1 = GPIO_OUTPUT;            /* Configure GPIO1 (EPWM1B) as an output */
    GpioCtrlRegs.GPAODR.bit.GPIO0 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO0 (EPWM1A) as push-pull normal output */
    GpioCtrlRegs.GPAODR.bit.GPIO1 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO0 (EPWM1B) as push-pull normal output */
    /* Set GPIO2/3 (Phase B) gate pins as Push Pull Outputs */
    GpioCtrlRegs.GPADIR.bit.GPIO2 = GPIO_OUTPUT;            /* Configure GPIO2 (EPWM2A) as an output */
    GpioCtrlRegs.GPADIR.bit.GPIO3 = GPIO_OUTPUT;            /* Configure GPIO3 (EPWM2B) as an output */
    GpioCtrlRegs.GPAODR.bit.GPIO2 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO2 (EPWM2A) as push-pull normal output */
    GpioCtrlRegs.GPAODR.bit.GPIO3 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO3 (EPWM2B) as push-pull normal output */
    /* Set GPIO4/5 (Phase A) gate pins as Push Pull Outputs */
    GpioCtrlRegs.GPADIR.bit.GPIO4 = GPIO_OUTPUT;            /* Configure GPIO4 (EPWM3A) as an output */
    GpioCtrlRegs.GPADIR.bit.GPIO5 = GPIO_OUTPUT;            /* Configure GPIO5 (EPWM3B) as an output */
    GpioCtrlRegs.GPAODR.bit.GPIO4 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO4 (EPWM3A) as push-pull normal output */
    GpioCtrlRegs.GPAODR.bit.GPIO5 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO5 (EPWM3B) as push-pull normal output */

    /* Configure GPIO0/1 (Phase C) to EPWM1A/1B */
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = GPIO_MUX_TYPE_0;       /* Configure GPIO0 as GPIO */
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = GPIO_MUX_TYPE_0;       /* Configure GPIO1 as GPIO */
    /* Configure GPIO2/3 (Phase B) to EPWM2A/2B */
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = GPIO_MUX_TYPE_0;       /* Configure GPIO2 as GPIO */
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = GPIO_MUX_TYPE_0;       /* Configure GPIO3 as GPIO */
    /* Configure GPIO4/5 (Phase A) to EPWM3A/3B */
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = GPIO_MUX_TYPE_0;       /* Configure GPIO4 as GPIO */
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = GPIO_MUX_TYPE_0;       /* Configure GPIO5 as GPIO */

    /*---------------------- CONFIGURE BRAKE and RESET pin Functionality  ----------------------------*/
    /*Brake functionality is deferred for MCU SW.
     * Brake functionality will be implemented in future but the GPIO's will change.
     */
    #if 0
    /* Configure U_BRAKE_N Output */
    GpioCtrlRegs.GPCGMUX2.bit.GPIO90 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPCMUX2.bit.GPIO90 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPCDIR.bit.GPIO90 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPCPUD.bit.GPIO90 = GPIO_DISABLE_PULLUP;   /* Disable the pullups */
    GpioCtrlRegs.GPCODR.bit.GPIO90 = GPIO_NORMAL_OUTPUT;    /* Configure as push-pull normal output */

    /* Configure PWM_BRK_N Output */
    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = GPIO_MUX_TYPE_0;       /* Configure as GPIO */
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO14 (EPWM8A) */
    GpioCtrlRegs.GPADIR.bit.GPIO14 = GPIO_OUTPUT;            /* Configure GPIO14 (EPWM8A) as an output */
    GpioCtrlRegs.GPAODR.bit.GPIO14 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO14 (EPWM8A) as push-pull normal output */

    /* Configure PWM_RST_N Output */
    GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = GPIO_MUX_TYPE_0;       /* Configure GPIO15 as GPIO */
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO15 (EPWM8B) */
    GpioCtrlRegs.GPADIR.bit.GPIO15 = GPIO_OUTPUT;            /* Configure GPIO15 (EPWM8B) as an output */
    GpioCtrlRegs.GPAODR.bit.GPIO15 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO15 (EPWM8B) as push-pull normal output */
    #endif

#endif
    /* Configure NS_CNTL_PHA_CHA Output */
    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = GPIO_MUX_TYPE_0;       /* Configure as GPIO */
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO14 */
    GpioCtrlRegs.GPADIR.bit.GPIO14 = GPIO_OUTPUT;            /* Configure GPIO14 as an output */
    GpioCtrlRegs.GPAODR.bit.GPIO14 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO14 as push-pull normal output */

    /* Configure NS_CNTL_PHC_CHA Output */
    GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = GPIO_MUX_TYPE_0;       /* Configure GPIO15 as GPIO */
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = GPIO_DISABLE_PULLUP;    /* Disable pull-up on GPIO15 */
    GpioCtrlRegs.GPADIR.bit.GPIO15 = GPIO_OUTPUT;            /* Configure GPIO15 as an output */
    GpioCtrlRegs.GPAODR.bit.GPIO15 = GPIO_NORMAL_OUTPUT;     /* Configure GPIO15 as push-pull normal output */

    /* Setup GPIO_25 as SCU_MCU_ENABLE Input */
    GpioCtrlRegs.GPAGMUX2.bit.GPIO25 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPADIR.bit.GPIO25 = GPIO_INPUT;            /* Set as an input */
    GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = GPIO_ASYNC;          /* Set input as asynchronous */
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = GPIO_DISABLE_PULLUP;   /* Disable pullup */

    /* Setup GPIO23 as 270V_BUS_CNTL_CHA Output */
    GpioCtrlRegs.GPAGMUX2.bit.GPIO23 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX2.bit.GPIO23  = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPADIR.bit.GPIO23   = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPAPUD.bit.GPIO23   = GPIO_DISABLE_PULLUP;   /* Disable the pullups */
    GpioCtrlRegs.GPAODR.bit.GPIO23   = GPIO_NORMAL_OUTPUT;    /* Configure as push-pull normal output */

    /* Setup GPIO_28 as VC_MODE_CHA Output */
    GpioCtrlRegs.GPAGMUX2.bit.GPIO28 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX2.bit.GPIO28  = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPADIR.bit.GPIO28   = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPAPUD.bit.GPIO28   = GPIO_DISABLE_PULLUP;   /* Disable the pullups */
    GpioCtrlRegs.GPAODR.bit.GPIO28   = GPIO_NORMAL_OUTPUT;    /* Configure as push-pull normal output */

    /* Setup GPIO_99 28V_BUS_CNTL_CHA   Output */
     GpioCtrlRegs.GPDGMUX1.bit.GPIO99 = GPIO_MUX_GRP_0;      /* Select group 0 */
     GpioCtrlRegs.GPDMUX1.bit.GPIO99  = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
     GpioCtrlRegs.GPDDIR.bit.GPIO99   = GPIO_OUTPUT;           /* Set as an output */
     GpioCtrlRegs.GPDPUD.bit.GPIO99   = GPIO_DISABLE_PULLUP;   /* Disable the pullups */
     GpioCtrlRegs.GPDODR.bit.GPIO99   = GPIO_NORMAL_OUTPUT;    /* Configure as push-pull normal output */

     /* Setup GPIO_133 BUS_SW_ENABLE   Output */
      GpioCtrlRegs.GPEGMUX1.bit.GPIO133 = GPIO_MUX_GRP_0;      /* Select group 0 */
      GpioCtrlRegs.GPEMUX1.bit.GPIO133  = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
      GpioCtrlRegs.GPEDIR.bit.GPIO133   = GPIO_OUTPUT;           /* Set as an output */
      GpioCtrlRegs.GPEPUD.bit.GPIO133   = GPIO_DISABLE_PULLUP;   /* Disable the pullups */
      GpioCtrlRegs.GPEODR.bit.GPIO133   = GPIO_NORMAL_OUTPUT;    /* Configure as push-pull normal output */
#endif

    EDIS;
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
    /* Enable CPU INT1 which is connected to XINT1 INT: */
    IER |= M_INT1;   /* Enable CPU INT 1*/
    EINT;
//	GpioDataRegs.GPFCLEAR.all = 0x000F;
//    MTR_DIR_clr();
//    b485_ENBL_set();
//    b200MS_STRT_clr();
//    TST_TX_clr();       /* Make sure that TP1 is low for use throughout the software.*/
//    GpioDataRegs.GPBCLEAR.bit.GPIOB7 = 1;
//    GpioDataRegs.GPDCLEAR.bit.GPIOD5 = 1;
//
    /* shall reset all debounce timers to 0.*/
    for (i = 0; i < DEBOUNCED_INPUTS_MAX; i++)
    {
        /*PATH(Gpio_Init,F);*/
        DebounceCount[i] = 0;
    }

    /* shall call Gpio_Capture to initialize G_tGpioInputs variable*/
    Gpio_Capture();
    /*PATH(Gpio_Init,G);*/
}


/**
    Captures discrete inputs for GPIO driver.

    Purpose:
        Captures discrete inputs for the GPIO driver.  Includes debouncing logic 
        on all inputs.  Results are copied to the global G_tGpioInputs word.

    Parameters:
        None.

    Global Data Referenced:
        GpioDataRegs 
        G_tGpioInputs
        Debounced  
        DebounceCount
    
    Return  void
    
    Preconditions and Assumptions:
        This routine must be called on a 1 ms timebase to ensure proper debounce timing.

    Remarks
    This routine implements debouncing by first capturing all discretes to a 
    single 32-bit word, then performing a debounce check on each bit 
    individually.  DebounceCount is an array of counters for the debouncing, 
    with one entry for each discrete bit.  When a bit's DebounceCount entry 
    exceeds the defined maximum count, the discrete value is defined as stable, 
    and copied to the #Debounced word.
    
    The maximum debounce count is set by the DEBOUNCE_TIME macro, and is 
    configured to allow a debounce time of 6 ms.  To maintain this timing, it 
    is important that Gpio_Capture() be called on a 1 ms timebase.
    
    Individual discretes may be selected or deselected for debounce logic by 
    use of the DEBOUNCE_MASK macro.  If the DEBOUNCE_MASK bit is set, 
    debouncing for that bit is enabled.  If the DEBOUNCE_MASK bit is unset, the
    debounce logic is skipped for that bit.
*/
void Gpio_Capture( void )
{
    int i;
    t32ByteBits newDiscretes;
    register unsigned int *pcount;
    register unsigned long *pbit;
    static unsigned long bitmask[DEBOUNCED_INPUTS_MAX] = /* declared static to put init at boot*/
    {
        0x00000001, 0x00000002, 0x00000004, 0x00000008,
        0x00000010, 0x00000020, 0x00000040, 0x00000080,
        0x00000100, 0x00000200, 0x00000400, 0x00000800,
        0x00001000, 0x00002000, 0x00004000, 0x00008000,
        0x00010000, 0x00020000, 0x00040000, 0x00080000,
        0x00100000, 0x00200000, 0x00400000, 0x00800000,
        0x01000000, 0x02000000, 0x04000000, 0x08000000,
        0x10000000, 0x20000000, 0x40000000, 0x80000000
    };

    /*PATH(Gpio_Capture,A);*/

    /* Refer to the GP Input Macros in gpio.h for specific bit placement
        in the newDiscretes word.
       shall capture and pack GPIO input discretes into a 32 bit word*/
    newDiscretes.all = 0UL; /* Initialize */
    newDiscretes.bit.b0  = PRGM1_RAW;                /* GPIO64 */
    newDiscretes.bit.b1  = PRGM2_RAW;                /* GPIO65 */
    newDiscretes.bit.b2  = PRGM3_RAW;                /* GPIO66 */
    newDiscretes.bit.b3  = PRGM4_RAW;                /* GPIO67 */
    newDiscretes.bit.b4  = PRGM5_RAW;                /* GPIO68 */
    newDiscretes.bit.b5  = PRGM6_RAW;                /* GPIO84 */
    newDiscretes.bit.b6  = RIG_N_RAW;                /* GPIO29 */
    newDiscretes.bit.b7  = CHX_STATUS_RAW;           /* GPIO32 */
    newDiscretes.bit.b8  = GD_FAULT_RAW;             /* GPIO33 */
    newDiscretes.bit.b9  = GD_READY_RAW;             /* GPIO35 */
    newDiscretes.bit.b10 = MTR_EN_CHA_RAW;           /* GPIO25 */
    newDiscretes.bit.b11 = CHA_HS1_RAW;              /* GPIO6  */
    newDiscretes.bit.b12 = CHA_HS2_RAW;              /* GPIO7  */
    newDiscretes.bit.b13 = CHA_HS3_RAW;              /* GPIO8  */
    newDiscretes.bit.b14 = CHB_ICC_HS1_RAW;          /* GPIO9  */
    newDiscretes.bit.b15 = CHB_ICC_HS2_RAW;          /* GPIO10 */
    newDiscretes.bit.b16 = CHB_ICC_HS3_RAW;          /* GPIO11 */
    newDiscretes.bit.b17 = GFD_I_SENSE_RAW;          /* GPIO57 */
    newDiscretes.bit.b18 = nFAULT_BRK_HS_OUT_RAW;    /* GPIO58 */

    /* shall maintain an internal "Debounced"  word which is to contain the debounced
        discretes for debounceable inputs and the raw discretes for those
        which are not.
       shall update the "Debounced" word with non-debounced discretes directly.*/
    Debounced = ((Debounced & DEBOUNCE_MASK) | (newDiscretes.all & ~DEBOUNCE_MASK));

    /* shall identify all changes in newDiscretes compared to Debounced inputs*/;
    newDiscretes.all ^= Debounced;

    /* shall mask non-debounced bits*/
    newDiscretes.all &= DEBOUNCE_MASK;

    /* shall loop through all discretes to check debouncing, skipping unchanged bits*/
    pcount = DebounceCount;              /* pointer to counter array*/
    pbit = bitmask;                      /* pointer to bitmask array*/

    for (i = 0; i < DEBOUNCED_INPUTS_MAX; i++, pcount++, pbit++)
    {
		/*PATH(Gpio_Capture,B);*/

        if (newDiscretes.all & *pbit)        /* check bit i*/
        {
            /*PATH(Gpio_Capture,C);*/

            if (*pcount < DEBOUNCE_TIME) /* check if counter exceeds limit*/
            {
                /*PATH(Gpio_Capture,F);*/

                /* shall increment the timer for that discrete if any debounceable input
                   discrete has been different than the "Debounced" word for < 6ms.*/
                (*pcount)++;             /* increment counter*/
            }
            else
            {
                /*PATH(Gpio_Capture,G);*/

                /* shall update the "Debounced" word and reset the timer to 0 if any
                   debounceable input discrete has been different from the "Debounced"
                   word for >= timeDelay (ms).*/
                Debounced ^= *pbit;      /* toggle bit i*/
                *pcount = 0;             /* reset counter*/
            }
        }
        else
        {
            /*PATH(Gpio_Capture,H);*/

            /*  shall reset the the 6ms timer for that discrete to 0 if any debounceable input discrete
                is the same as the "Debounced" word.*/
            *pcount = 0;
        }
    }

    /* shall snapshot the internal "Debounced" word to the externally available
        "G_tGpioInputs" word*/
    G_tGpioInputs.all = Debounced;

    /*PATH(Gpio_Capture,I);*/
}

/*
    Purpose:
        This function determines what channel the software is executing 
        on based on the PRGM pins.
    
    Parameters:
        None.

    Global Data Referenced:
        GpioDataRegs
        PrimarySide
		Nvm_Rigging
		v14V_MON_raw
		PRGM_PIN_READ_LEVEL
		TIMER_ONESHOT
		TIMER_10ms
		TIMER_DEFAULTS

*/
void Gpio_EstablishChannel(void)
{
	Timer_t v5V_DelayTimer = TIMER_DEFAULTS;
	bool_t bValidChannelCfg = false;
	bool_t bValidPanelCfg = false;
	bool_t bChannelCheckNvm = false;
	bool_t bPanelCheckNvm = false;


#if defined(DRV8312_DEV_KIT) /* 2837xD Development Kit Hardware */
    /* Hardcode since there are no PRGM pin inputs on dev kit. */
    G_bChannelA = true;
    G_eActId = LEFT_ACTUATOR;
    G_eLruId = MCU_LIB_LRU;
#else /* MCU Hardware */

    /*PATH(Gpio_EstablishChannel,A);*/
	Timer_SetTime(&v5V_DelayTimer, TIMER_ONESHOT, TIMER_10ms);

	while ((Adc_Raw.val.u16_5V_SENSE <= PRGM_PIN_READ_LEVEL) && (Timer_IsExpired(&v5V_DelayTimer) == false))
	{
		/*PATH(Gpio_EstablishChannel,B);*/
		Adc_Sample();
	    DELAY_US(4L); /* Delay to allow time for conversion (690 cycles = 3.45us) before reading ADC */
	    Adc_GetResults();
		Gpio_Capture();
	}

	/* Check if the 5V supply is healthy */
	if (Adc_Raw.val.u16_5V_SENSE > PRGM_PIN_READ_LEVEL)
	{
	    /* Healthy voltage to read PRGM pins directly */

        /* Read PRGM pins directly to attempt to set the channel */
        bValidChannelCfg = Gpio_SetChannel(false);
        /* Check if the configuration was valid */
        if(bValidChannelCfg != true)
        {
            /* Invalid PRGM pin configuration. Read NVM to attempt to set the channel */
            bChannelCheckNvm = true;
        }

        /* Read PRGM pins directly to attempt to set the panel */
        bValidPanelCfg = Gpio_SetPanelType(false);
        /* Check if the configuration was valid */
        if(bValidPanelCfg != true)
        {
            /* Invalid PRGM pin configuration. Read NVM to attempt to set the panel */
            bPanelCheckNvm = true;
        }
	}
	else
	{
	    /* Unhealthy voltage level to read PRGM pins. Set to read from NVM for both
	     * Channel and Panel configurations */
	    bChannelCheckNvm = true;
	    bPanelCheckNvm = true;
	}

	if(bChannelCheckNvm == true)
	{
        /* Read NVM to attempt to set the channel */
        bValidChannelCfg = Gpio_SetChannel(true);
	}
	if(bPanelCheckNvm == true)
	{
        /* Read NVM to attempt to set the panel */
        bValidPanelCfg = Gpio_SetPanelType(true);
	}

    /* Check if either the channel or panel configuration was invalid */
    if((bValidChannelCfg != true) || (bValidPanelCfg != true))
    {
        /* Was not able to read the PGRM pins directly, there was at least one invalid configuration with the PRGM pins,
         * or the NVM configuration for the channel and/or panel was invalid.
         * Run fault monitor 0x57 to trip the appropriate fault */
        Bit_LatchedMonitors(0x57, true);
    }

    /* Set the Actuator Number */
    Gpio_SetActuatorNumber();
#endif

    /*PATH(Gpio_EstablishChannel,M);*/
}

/****************************************************************************************************
*  Function: Gpio_SetChannel
*  Purpose: Decode the program pins dedicated to channel.
*  Global Inputs: None
*  Global Outputs: G_bChannelA, G_eChId
*  Input: bGetFromNvm:  true = read program pins out of NVM, false = read raw values.
*  Output: bValidCfg:  true = valid configuration, false = invalid configuration
****************************************************************************************************/
bool_t Gpio_SetChannel(bool_t bGetFromNvm)
{
    bool_t bValidCfg = false;
    tPrgm_t tPrgm = { 0 };

    /* PRGM_1 | PRGM_2 | CHANNEL
     * -------------------------
     *   0    |   0    | INVALID
     *   1    |   0    | A
     *   0    |   1    | B
     *   1    |   1    | INVALID
     */

    /* Check whether to retrieve from NVM or not */
    if(bGetFromNvm == true)
    {
        /* Use values in NVM to set the G_eChId */
        tPrgm.bit.bPRGM1 = tNvm.tData.Nvm_Rigging.tPrgm.bit.bPRGM1;
        tPrgm.bit.bPRGM2 = tNvm.tData.Nvm_Rigging.tPrgm.bit.bPRGM2;
    }
    else
    {
        /* Read the PRGM pins to set the G_eChId */
        tPrgm.bit.bPRGM1 = PRGM1_RAW;
        tPrgm.bit.bPRGM2 = PRGM2_RAW;
    }

    if ((tPrgm.bit.bPRGM1 != false) && (tPrgm.bit.bPRGM2 == false)) /* MCU Channel A */
    {
        G_bChannelA = true;
        G_eChId = CHANNEL_A;
        bValidCfg = true;
    }
    else if ((tPrgm.bit.bPRGM1 == false) && (tPrgm.bit.bPRGM2 != false)) /* MCU Channel B */
    {
        G_bChannelA = false;
        G_eChId = CHANNEL_B;
        bValidCfg = true;
    }
    else
    {
        /* Invalid configuration for channel PRGM pins */
        bValidCfg = false;
    }

    return bValidCfg;
}

/****************************************************************************************************
*  Function: Gpio_SetPanelType
*  Purpose: Decode the panel type and set the Global variable for panel type
*  Global Inputs: None
*  Global Outputs: G_eLruId
*  Input: bGetFromNvm:  true = read program pins out of NVM, false = read raw values.
*  Output: bValidCfg:  true = valid configuration, false = invalid configuration
****************************************************************************************************/
bool_t Gpio_SetPanelType(bool_t bGetFromNvm)
{
    bool_t bValidCfg = false;
    t16ByteBits tPrgm = { 0 };

    /* Check whether to retrieve from NVM or not */
    if(bGetFromNvm == true)
    {
        /* Use values in NVM to set the G_eLruId */
        tPrgm.bit.b0 = tNvm.tData.Nvm_Rigging.tPrgm.bit.bPRGM6;
        tPrgm.bit.b1 = tNvm.tData.Nvm_Rigging.tPrgm.bit.bPRGM5;
        tPrgm.bit.b2 = tNvm.tData.Nvm_Rigging.tPrgm.bit.bPRGM4;
        tPrgm.bit.b3 = tNvm.tData.Nvm_Rigging.tPrgm.bit.bPRGM3;
    }
    else
    {
        /* Read the PRGM pins to set the G_eLruId */
        tPrgm.bit.b0 = PRGM6_RAW;
        tPrgm.bit.b1 = PRGM5_RAW;
        tPrgm.bit.b2 = PRGM4_RAW;
        tPrgm.bit.b3 = PRGM3_RAW;
    }

    switch (tPrgm.all)
    {
        case MCU_LIB_L:
        {
            G_eLruId = MCU_LIB_LRU;
            G_eActId = LEFT_ACTUATOR;
            bValidCfg = true;
            break;
        }
        case MCU_LIB_R:
        {
            G_eLruId = MCU_LIB_LRU;
            G_eActId = RIGHT_ACTUATOR;
            bValidCfg = true;
            break;
        }
        case MCU_RIB_L:
        {
            G_eLruId = MCU_RIB_LRU;
            G_eActId = LEFT_ACTUATOR;
            bValidCfg = true;
            break;
        }
        case MCU_RIB_R:
        {
            G_eLruId = MCU_RIB_LRU;
            G_eActId = RIGHT_ACTUATOR;
            bValidCfg = true;
            break;
        }
        case MCU_LOB_L:
        {
            G_eLruId = MCU_LOB_LRU;
            G_eActId = LEFT_ACTUATOR;
            bValidCfg = true;
            break;
        }
        case MCU_LOB_R:
        {
            G_eLruId = MCU_LOB_LRU;
            G_eActId = RIGHT_ACTUATOR;
            bValidCfg = true;
            break;
        }
        case MCU_ROB_L:
        {
            G_eLruId = MCU_ROB_LRU;
            G_eActId = LEFT_ACTUATOR;
            bValidCfg = true;
            break;
        }
        case MCU_ROB_R:
        {
            G_eLruId = MCU_ROB_LRU;
            G_eActId = RIGHT_ACTUATOR;
            bValidCfg = true;
            break;
        }
        default:
        {
            /* Error */
            G_eLruId = INVALID_LRU;
            G_eActId = INVALID_ACTUATOR_ID;
            bValidCfg = false;
            break;
        }
    }

    return bValidCfg;
}

/****************************************************************************************************
*  Function: Gpio_SetActuatorNumber
*  Purpose: Decode the panel type and set the Global variable for panel type
*  Global Inputs: None
*  Global Outputs: G_eLruId
*  Input:
*  Output:
****************************************************************************************************/
void Gpio_SetActuatorNumber(void)
{

    /*         ON-SIDE          |    CROSS-SIDE           */
    /*  PANEL |  LEFT  | RIGHT  | PANEL |  LEFT  | RIGHT
     * --------------------------------------------------
     *   RIB  |   A1   |  A2    |  RIB  |   A2   |  A1
     *   ROB  |   A3   |  A4    |  ROB  |   A4   |  A3
     *   LIB  |   A2   |  A1    |  LIB  |   A1   |  A2
     *   LOB  |   A4   |  A3    |  LOB  |   A3   |  A4
     */

    /* Decode the LRU ID to set the proper actuator number for the onside
     * channel and cross-side channel */
    switch(G_eLruId)
    {
        case MCU_LIB_LRU:
        {
            if(G_eActId == RIGHT_ACTUATOR)
            {
                G_eActuatorNumber = ACTUATOR_1;
                G_eActuatorNumberX = ACTUATOR_2;
            }
            else if(G_eActId == LEFT_ACTUATOR)
            {
                G_eActuatorNumber = ACTUATOR_2;
                G_eActuatorNumberX = ACTUATOR_1;
            }
            break;
        }
        case MCU_RIB_LRU:
        {
            if(G_eActId == RIGHT_ACTUATOR)
            {
                G_eActuatorNumber = ACTUATOR_2;
                G_eActuatorNumberX = ACTUATOR_1;
            }
            else if(G_eActId == LEFT_ACTUATOR)
            {
                G_eActuatorNumber = ACTUATOR_1;
                G_eActuatorNumberX = ACTUATOR_2;
            }
            break;
        }
        case MCU_LOB_LRU:
        {
            if(G_eActId == RIGHT_ACTUATOR)
            {
                G_eActuatorNumber = ACTUATOR_3;
                G_eActuatorNumberX = ACTUATOR_4;
            }
            else if(G_eActId == LEFT_ACTUATOR)
            {
                G_eActuatorNumber = ACTUATOR_4;
                G_eActuatorNumberX = ACTUATOR_3;
            }
            break;
        }
        case MCU_ROB_LRU:
        {
            if(G_eActId == RIGHT_ACTUATOR)
            {
                G_eActuatorNumber = ACTUATOR_4;
                G_eActuatorNumberX = ACTUATOR_3;
            }
            else if(G_eActId == LEFT_ACTUATOR)
            {
                G_eActuatorNumber = ACTUATOR_3;
                G_eActuatorNumberX = ACTUATOR_4;
            }
            break;
        }
        default:
        {
            /* ERROR */
            G_eActuatorNumber = INVALID_ACTUATOR;
            G_eActuatorNumberX = INVALID_ACTUATOR;
            break;
        }
    }

    return;
}


/* REWORK THIS LATER */
//
//GPIO_EnableUnbondedIOPullupsFor176Pin - Enable pullups for the unbonded
//                                        GPIOs on the 176PTP package:
//                                        GPIOs     Grp Bits
//                                        95-132    C   31
//                                                  D   31:0
//                                                  E   4:0
//                                        134-168   E   31:6
//                                                  F   8:0
//
void GPIO_EnableUnbondedIOPullupsFor176Pin()
{
    EALLOW;
    GpioCtrlRegs.GPCPUD.all = ~0x80000000;  //GPIO 95
    GpioCtrlRegs.GPDPUD.all = ~0xFFFFFFF7;  //GPIOs 96-127
    GpioCtrlRegs.GPEPUD.all = ~0xFFFFFFDF;  //GPIOs 128-159 except for 133
    GpioCtrlRegs.GPFPUD.all = ~0x000001FF;  //GPIOs 160-168
    EDIS;
}

/* REWORK THIS LATER */
//
// GPIO_EnableUnbondedIOPullupsFor100Pin - Enable pullups for the unbonded
//                                         GPIOs on the 100PZ package:
//                                         GPIOs     Grp Bits
//                                         0-1       A   1:0
//                                         5-9       A   9:5
//                                         22-40     A   31:22
//                                                   B   8:0
//                                         44-57     B   25:12
//                                         67-68     C   4:3
//                                         74-77     C   13:10
//                                         79-83     C   19:15
//                                         93-168    C   31:29
//                                                   D   31:0
//                                                   E   31:0
//                                                   F   8:0
//
void GPIO_EnableUnbondedIOPullupsFor100Pin()
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.all = ~0xFFC003E3;  //GPIOs 0-1, 5-9, 22-31
    GpioCtrlRegs.GPBPUD.all = ~0x03FFF1FF;  //GPIOs 32-40, 44-57
    GpioCtrlRegs.GPCPUD.all = ~0xE10FBC18;  //GPIOs 67-68, 74-77, 79-83, 93-95
    GpioCtrlRegs.GPDPUD.all = ~0xFFFFFFF7;  //GPIOs 96-127
    GpioCtrlRegs.GPEPUD.all = ~0xFFFFFFFF;  //GPIOs 128-159
    GpioCtrlRegs.GPFPUD.all = ~0x000001FF;  //GPIOs 160-168
    EDIS;
}

/* REWORK THIS LATER */
//
// GPIO_EnableUnbondedIOPullups - InitSysCtrl would call this function
//                                this takes care of enabling IO pullups.
//
void GPIO_EnableUnbondedIOPullups()
{
    //
    //bits 8-10 have pin count
    //
    unsigned char pin_count = ((DevCfgRegs.PARTIDL.all & 0x00000700) >> 8) ;

    //
    //5 = 100 pin
    //6 = 176 pin
    //7 = 337 pin
    //
    if(pin_count == 5)
    {
        GPIO_EnableUnbondedIOPullupsFor100Pin();
    }
    else if (pin_count == 6)
    {
        GPIO_EnableUnbondedIOPullupsFor176Pin();
    }
    else
    {
        //do nothing - this is 337 pin package
    }
}

/**
    brief Interrupt handler for GFD_I_SENSE signal Input.

     Purpose:
        Interrupt handler GFD_I_SENSE external interrupt.

     Global Data Referenced:

    return  void

     Preconditions and Assumptions:
        Is not to be called by any other function, only by GFD_I_SENSE falling edges.
*/
__interrupt void GFD_I_SENSE_ISR( void )
{

    bGFD_I_SENSE_ISR = False;
    Bit_CriticalMonitors(0x7D, true);    // GFD_I_SENSE monitor = 0x7D
    /* Acknowledge interrupt */
    PieCtrlRegs.PIEACK.bit.ACK1 = TRUE;
}


#endif //CPU1

/* end of file */
