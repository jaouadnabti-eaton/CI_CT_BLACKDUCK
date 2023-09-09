/****************************************************************************************************
*  File name: spi.c
*
*  Purpose: This file implements the interface for the SPI Serial Port.  The interface
*  provides routines necessary to intialize the driver and send a message on the channel.
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
#include "spi.h"
#include "gpio.h"
//#include "nvm.h"
//#include "bit.h"
//#include "bitmonitors.h"
////#include "panel.h"
//#include "motor.h"
//#include "rvdt.h"

/*      Local Type Definitions */

/*      Local Defines */


//#define SPI_BEGIN               0xBB /* Beginning SPI Tx Character */
//#define SPI_END                 0xEE /* Ending SPI Tx Character */
//#define MAX_FIFO_COUNT 16 /*maximum capacity of the FIFO */

//void SPI_Transmit(Uint16 a);

/*      Global Variables */
//Uint16 SpiTxBuf[SPI_TX_MSG_SIZE] = {0};  /* the global transmit buffer for the SPI interface */
Uint16 SpiRxBuf[SPI_RX_MSG_SIZE] = {0};  /* the global receive buffer for the SPI interface */
uint16_t adcCh = nCS_270VBUS_VSENSE;
//Uint16 msgCnt = 0;

//bool CrossSideSpiDone = false;
//bool SpiMessageValid = false;
//Uint16 xLatestFaultCode = 0;
//Uint16 xLatestFaultData = 0;
//Timer_t FaultSpiTimer = TIMER_DEFAULTS;
//bool SpiMessageReady = false;
//Uint16 spiHrtBt = 0;
//bool SpiTxStatus = true;

//RIG_STATUS xSideRigStatus = NOT_RIGGED;
//Uint16 xReportedFaultCode = 0;

//SPI_MSG *SpiOutMsg = {0};

/*      Local ROM Constants */

/*      Local Variable Declarations */

/*      Local Function Prototypes */

/*      Function Definitions */

/***************************************************************************************************
 * Function: Spi_Init
 * Purpose: Initialize SPI serial port, including setting the port pins properly
 *          and configuring the bit rate, etc.
 * Global Inputs(s): SpiaRegs
 * Global Outputs(s): SpiaRegs
 * Inputs: None
 * Output: None
 ****************************************************************************************************/
void Spi_Init(void)
{
    /* PATH(Spi_Init,A); */

    /* Re-configure the DSP pins for SPI functionality. */
    EALLOW;

    /* GPIO55 is SPISOMIA (SPI_A_SOMI). */
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;                  // Enable pull-up
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 3;                // Asynch input
    GpioCtrlRegs.GPBGMUX2.bit.GPIO55 = GPIO_MUX_GRP_0;   // SPISOMIA
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = GPIO_MUX_TYPE_1;   // SPISOMIA

    /* GPIO56 is SPICLKA (SPI_A_CLK). */
    GpioCtrlRegs.GPBPUD.bit.GPIO56 = 0;                  // Enable pull-up
    GpioCtrlRegs.GPBQSEL2.bit.GPIO56 = 3;                // Asynch input
    GpioCtrlRegs.GPBGMUX2.bit.GPIO56 = GPIO_MUX_GRP_0;   // SPICLKA
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = GPIO_MUX_TYPE_1;   // SPICLKA


    /*Configure GPIO91 as nCS_270VBUS_VSENSE*/
    GpioCtrlRegs.GPCGMUX2.bit.GPIO91 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPCMUX2.bit.GPIO91 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPCDIR.bit.GPIO91 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPCPUD.bit.GPIO91 = GPIO_DISABLE_PULLUP;   /* Disable the pullups */
    GpioCtrlRegs.GPCODR.bit.GPIO91 = GPIO_NORMAL_OUTPUT;    /* Configure as push-pull normal output */

    /*Configure GPIO93 as nCS_28VBUS_VSENSE*/
    GpioCtrlRegs.GPCGMUX2.bit.GPIO93 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPCMUX2.bit.GPIO93 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPCDIR.bit.GPIO93 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPCPUD.bit.GPIO93 = GPIO_DISABLE_PULLUP;   /* Disable the pullups */
    GpioCtrlRegs.GPCODR.bit.GPIO93 = GPIO_NORMAL_OUTPUT;    /* Configure as push-pull normal output */

    /*Configure GPIO94 as nCS_IBRK_SENSE*/
    GpioCtrlRegs.GPCGMUX2.bit.GPIO94 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPCMUX2.bit.GPIO94 = GPIO_MUX_TYPE_0;      /* Set as GPIO pin type */
    GpioCtrlRegs.GPCDIR.bit.GPIO94 = GPIO_OUTPUT;           /* Set as an output */
    GpioCtrlRegs.GPCPUD.bit.GPIO94 = GPIO_DISABLE_PULLUP;   /* Disable the pullups */
    GpioCtrlRegs.GPCODR.bit.GPIO94 = GPIO_NORMAL_OUTPUT;    /* Configure as push-pull normal output */

    EDIS;
    /* shall initialize the SPI registers such that the SPI peripheral will be:
           - 781.250 kHz
           - Clock is active high
           - no loopback
           - 8 bit words
           - no clock delay */
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;    /* Reset on */
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;   /* Clock is Active High */
#if defined(_HSIT)
    SpiaRegs.SPICCR.bit.SPILBK = 1;        /* loopback */
#endif
    SpiaRegs.SPICCR.bit.SPICHAR = (16-1);     /* 16-bit character */
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;  /* Set as SPI Master */
    SpiaRegs.SPICTL.bit.TALK = 0;           // enable receive mode
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
    SpiaRegs.SPICTL.bit.SPIINTENA = 1;
    /* Set the SPI Bit Rate. The register is limited to 7 bits.
     * Set to maximum register setting to obtain slowest baud rate closets to that of VLJ which
     * was 468.75kHz */
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 127;//127; /* SPI Baud Rate = (100 MHz (LSPCLK) / (127 + 1)) = 781.250 kHz */
    SpiaRegs.SPIFFCT.bit.TXDLY = 0;

    SpiaRegs.SPIFFTX.bit.SPIFFENA = 0;      /* Enable FIFO Transmission */
    SpiaRegs.SPIFFTX.bit.SPIRST = 0;        /* Reset SPI transmit and receive channels */
    SpiaRegs.SPIFFTX.bit.TXFIFO = 0;        /* Reset Tx FIFO pointer to zero */
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;   /* Reset Rx FIFO pointer to zero */

    SpiaRegs.SPIFFTX.bit.SPIRST = 1;        /* FIFO can resume transmit or receive */
    SpiaRegs.SPIFFTX.bit.TXFIFO = 1;        /* Re-enable Transmit FIFO operation */
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;   /* Re-enable FIFO Reception */

//    SpiaRegs.SPIPRI.bit.TRIWIRE = 1;
    SpiaRegs.SPIPRI.bit.FREE = 1;           /* Breakpoints don't disturb transmission */          /* Transmit enabled */
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;     /* Take SPI out of reset */

    /* PATH(Spi_Init,E); */
}  /* end Spi_Init */

/***************************************************************************************************
 * Function: Spi_RxMessage
 * Purpose: Gets the recieved SPI message from the SPI Receive Buffer
 * Global Inputs(s): SpiRxBuf, adcCh
 * Global Outputs(s): SpiRxBuf
 * Inputs: None
 * Output: SpiReceived
 ****************************************************************************************************/
bool Spi_RxMessage()
{
    bool SpiReceived = False;
    Uint16  spi_temp = 0;
    /* PATH(Spi_RxMessage,A); */
    if(SpiaRegs.SPISTS.bit.INT_FLAG ==1)
    {
        spi_temp = SpiaRegs.SPIRXBUF;
        SpiRxBuf[adcCh] = (Uint16)spi_temp >> 2;
        SpiReceived = True;
    }
    else
    {
        SpiReceived = False;
    }
    return SpiReceived;
} /* end Spi_RxMessage */

/***************************************************************************************************
 * Function: SPI_SetCS
 * Purpose: Sets the Chip Select low and initiate handshake.
 * Global Inputs(s): adcCh
 * Global Outputs(s): None
 * Inputs: None
 * Output: None
 ****************************************************************************************************/
void SPI_SetCS()
{

    switch(adcCh)
    {
        case nCS_270VBUS_VSENSE:
            nCS_270VBUS_VSENSE_clr();
        	break;
        case nCS_28VBUS_VSENSE:
            nCS_28VBUS_VSENSE_clr();
        	break;
        case nCS_IBRK_SENSE:
            nCS_IBRK_SENSE_clr();
        	break;
        default:
        	break;
    }

#if defined(SPI_ADC_TEST)
    Spib_send();                    // for ADC test and will be removed
#endif

    // Initiate SPI Transmission
    uint16_t dummy = 0xBBEE;
    SpiaRegs.SPITXBUF = dummy;      // Send Dummy data to begin conversion

}

/***************************************************************************************************
 * Function: SPI_ClearCS
 * Purpose: Clears the Chip Select.
 * Global Inputs(s): None
 * Global Outputs(s): None
 * Inputs: None
 * Output: None
 ****************************************************************************************************/
void SPI_ClearCS()
{

    switch(adcCh)
    {
        case nCS_270VBUS_VSENSE:
            nCS_270VBUS_VSENSE_set();
        	break;
        case nCS_28VBUS_VSENSE:
            nCS_28VBUS_VSENSE_set();
        	break;
        case nCS_IBRK_SENSE:
            nCS_IBRK_SENSE_set();
        	break;
        default:
        	break;
    }

}

/***************************************************************************************************
 * Function: SPI_Capture
 * Purpose: Capture the ADC counts received on SPI.
 * Global Inputs(s): adcCh
 * Global Outputs(s): None
 * Inputs: None
 * Output: None
 ****************************************************************************************************/
void SPI_Capture()
{

    switch(adcCh)
    {
        case nCS_270VBUS_VSENSE:
            Spi_RxMessage();
            break;
        case nCS_28VBUS_VSENSE:
            Spi_RxMessage();
            break;
        case nCS_IBRK_SENSE:
            Spi_RxMessage();
            break;
        default:
            break;
    }
    adcCh++;
    if(adcCh>nCS_IBRK_SENSE)
    {
        adcCh = nCS_270VBUS_VSENSE;
    }


    SPI_ClearCS();
}

/***************************************************************************************************
 * Function: SPI_LoopbackTest
 * Purpose: Test SPI port for Loopback.
 * Global Inputs(s): None
 * Global Outputs(s): None
 * Inputs: None
 * Output: None
 ****************************************************************************************************/
void SPI_LoopbackTest()
{
    uint16_t txdummy = 0xBBEE;
    uint16_t rxdummy;
    SpiaRegs.SPICCR.bit.SPILBK = 1; // Enable Loopback
    SpiaRegs.SPICTL.bit.TALK = 1;   // Enable to transmit
    SpiaRegs.SPITXBUF = txdummy;      // Send Dummy data to begin conversion
    if(SpiaRegs.SPISTS.bit.INT_FLAG ==1)
    {
        rxdummy = SpiaRegs.SPIRXBUF;
        if(rxdummy != txdummy)
        {
            // Test failed
            // Log Monitor
        }
        else
        {
            // Test pass
            // do nothing
        }
    }
    else
    {
        // Test failed to establish connection
    }
    SpiaRegs.SPICCR.bit.SPILBK = 0; // Disable Loopback
    SpiaRegs.SPICTL.bit.TALK = 0;   // Disable to transmit
}
/* end spi.c */

// for SPI ADC test only and will be removed
#if defined(SPI_ADC_TEST)
void Spia_Init(void)
{
    /* PATH(Spi_Init,A); */

    /* Re-configure the DSP pins for SPI functionality. */
    EALLOW;
    /* GPIO25 is SPISOMIB (SPI_B_SIMO). */
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;                  // Enable pull-up
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3;                // Asynch input
    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = GPIO_MUX_GRP_3;   // SPISOMIB
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = GPIO_MUX_TYPE_3;   // SPISOMIB

    /* GPIO26 is SPICLKB (SPI_B_CLK). */
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;                  // Enable pull-up
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3;                // Asynch input
    GpioCtrlRegs.GPCGMUX1.bit.GPIO65 = GPIO_MUX_GRP_3;   // SPICLKB
    GpioCtrlRegs.GPCMUX1.bit.GPIO65 = GPIO_MUX_TYPE_3;   // SPICLKB

    /* GPIO26 is SPISTEB (SPI_B_STE). */
    GpioCtrlRegs.GPCPUD.bit.GPIO66 = 0;                  // Enable pull-up
    GpioCtrlRegs.GPCQSEL1.bit.GPIO66 = 3;                // Asynch input
    GpioCtrlRegs.GPCGMUX1.bit.GPIO66 = GPIO_MUX_GRP_3;   // SPISTEB
    GpioCtrlRegs.GPCMUX1.bit.GPIO66 = GPIO_MUX_TYPE_3;   // SPISTEB
    EDIS;
    /* shall initialize the SPI registers such that the SPI peripheral will be:
           - 781.250 kHz
           - Clock is active high
           - no loopback
           - 8 bit words
           - no clock delay */
    SpibRegs.SPICCR.bit.SPISWRESET = 0;    /* Reset on */
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0;   /* Clock is Active High */
    SpibRegs.SPICCR.bit.SPICHAR = (16-1);     /* 16-bit character */
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 0;  /* Set as SPI Master */
    SpibRegs.SPICTL.bit.TALK = 1;           // enable transmit path
    SpibRegs.SPICTL.bit.CLK_PHASE = 0;
    SpibRegs.SPICTL.bit.SPIINTENA = 1;
    /* Set the SPI Bit Rate. The register is limited to 7 bits.
     * Set to maximum register setting to obtain slowest baud rate closets to that of VLJ which
     * was 468.75kHz */
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 127;//127; /* SPI Baud Rate = (100 MHz (LSPCLK) / (127 + 1)) = 781.250 kHz */
    SpibRegs.SPIFFCT.bit.TXDLY = 0;

    SpibRegs.SPIFFTX.bit.SPIFFENA = 0;      /* Enable FIFO Transmission */
    SpibRegs.SPIFFTX.bit.SPIRST = 0;        /* Reset SPI transmit and receive channels */
    SpibRegs.SPIFFTX.bit.TXFIFO = 0;        /* Reset Tx FIFO pointer to zero */
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;   /* Reset Rx FIFO pointer to zero */

    SpibRegs.SPIFFTX.bit.SPIRST = 1;        /* FIFO can resume transmit or receive */
    SpibRegs.SPIFFTX.bit.TXFIFO = 1;        /* Re-enable Transmit FIFO operation */
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1;   /* Re-enable FIFO Reception */

    SpibRegs.SPIPRI.bit.TRIWIRE = 1;
    SpibRegs.SPIPRI.bit.FREE = 1;           /* Breakpoints don't disturb transmission */          /* Transmit enabled */
    SpibRegs.SPICCR.bit.SPISWRESET = 1;     /* Take SPI out of reset */

    /* PATH(Spi_Init,E); */
}  /* end Spi_Init */


void Spib_send()
{
    uint16_t dummy;
    static uint16_t data = 0;        // begin conversion
    SpibRegs.SPITXBUF = data++;
    if (data>0xFFFF)
    {data = 0;}
//    while(SpibRegs.SPISTS.bit.INT_FLAG !=1)
//    {} // Wait until data rx’d
    dummy = SpibRegs.SPIRXBUF;     // clear data
}
#endif

/* end spi.c */

