/****************************************************************************************************
*  File name: DFT.c
*
*  Purpose: The DFT.c file contains the functionality related to interfacing with the Serial DFT
*          GSE over the RS485bus on SCI
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

/****************************************** INCLUDES *********************************************/
//#include "Defines.h"
//#include "Cpu.h"
//#include "SysCtrl.h"
//#include "PieCtrl.h"
//#include "PieVect.h"
//#include "F2837xD_sci.h"
#include "DFT.h"
#include "Gpio.h"
#include "panel.h"
#include "actuation.h"
#include "adc.h"
#include "mcu.h"
#include "bit.h"
#include "nvm.h"
#include "motor.h"
#include "spi.h"
#include "rvdt.h"
//#include "Assembly.h"
//#include "TimerApi.h"
#include "csci.h"
#include "ASW_BSW_I.h"
#include "crc.h"
#include "bitmonitors.h"

/******************************************* DEFINES *********************************************/
/* Definition of SCI register based on hardware */
#if defined(DRV8312_DEV_KIT)
    /* Development kit is on SCI_A */
    #define DftSciRegs  SciaRegs
#else
    /* MCU Hardware is on SCI_A */
    #define DftSciRegs  ScicRegs
#endif

/* Transmit and receive maximum buffer sizes  */
/*   NOTE: both values must be a power of 2   */
#define SERIAL_TX_BUF_SIZE    0x80U
#define SERIAL_RX_BUF_SIZE    0x80U

/* For FIFO Operation define the size of the buffers in hardware */
#define SERIAL_TX_FIFO_SIZE     16U
#define SERIAL_RX_FIFO_SIZE     16U

//#define DFT_TX_BUF_SIZE         (sizeof(tDftMsgBuf_t))   /* maximum size needed for the DFT transmit buffer to hold the largest defined transmit message */
#define DFT_RX_BUF_SIZE         32      /* maximum size needed for the DFT interface serial receive buffer (unstuffed) */
#define DFT_MAX_FIFO_COUNT      8  /* DFT max FIFO set to 8 to hold 2 subframes worth of data only. Specifically designed to send 4 bytes + any DLE and ETX one subframe at a time.*/

#define DFT_LFT_CHN_ID          0x60 /* left channel value for the Channel Data Identifier data word in the DFT message */
#define DFT_RT_CHN_ID           0x61 /* right channel value for the Channel Data Identifier data word in the DFT message */

//#define RS485_RXTXLINE_SET_DELAY (20)
#define RS485_RXTXLINE_SET_DELAY (150)

/* Macros for incrementing circular buffer pointers */
/* NOTE: since we assume a buffer size that is a power of two,  */
/* bitwise anding an index value with the buffer size minus one */
/* provides an efficient way to wrap an index value back to 0   */
/* when it becomes equal to the buffer size. */

/*Justification for Coding Standard Deviation:
 *    Function like macro is the preferred solution here
 *    since SERIAL_TX_BUF_NEXT is a pre-compiled value
 *    An exception to MISRA Rule 18.4 is required.
 */
/*lint -e(961) # intended as is */
#define SERIAL_TX_BUF_NEXT(A)    (((A)+1U) & (SERIAL_TX_BUF_SIZE - 1U))

/*Justification for Coding Standard Deviation:
 *    Function like macro is the preferred solution here
 *    since SERIAL_TX_BUF_NEXT is a pre-compiled value
 *    An exception to MISRA Rule 18.4 is required.
 */
/*lint -e(961) # intended as is */
#define SERIAL_RX_BUF_NEXT(A)    (((A)+1U) & (SERIAL_RX_BUF_SIZE - 1U))

#define BLD_CRC_MESSAGE_SIZE        64
#define BLD_CHARS_PER_LONG          8U
#define BLD_UPPER_NIBBLE_SHIFT      28U
#define BLD_BITS_PER_NIBBLE         4U
#define DFT_MSG_SIZE                (sizeof(tDftMsg_t) * 2U)    /* Message size in bytes */
#define DFT_PKT_SIZE                0xAA   /* Unused: value for the Data Packet Size data word in DFT mode */

/* Transmit buffer */
//static uint16_t DFT_TxBuf[DFT_TX_BUF_SIZE] = {0}; /* the DFT transmit buffer for the DFT interface */
static tDftMsgBuf_t DFT_TxBuf = {0}; /* the DFT transmit buffer for the DFT interface */
static uint16_t DFT_TxIndex = 0;                  /* index into the DFT transmit buffer */
//static tDftMsg_t const* DFT_Msg = (tDftMsg_t const*)DFT_TxBuf;

/* Receive buffer */
static uint16_t DFT_RxBuf[DFT_RX_BUF_SIZE] = {0}; /* the DFT receive buffer */
static uint16_t DFT_RxIndex = 0;                  /* index into the DFT receive buffer */
static uint16_t DFT_RxNumBytes = 0;               /* count of the number of received bytes in the DFT buffer */
static uint16_t DFT_MAX_PERIOD = 80;              /* 80*0.125 = 10ms */
static uint16_t DFT_Period = 0;                   /* DFT_Period*0.125 = ms of DFT_MAX_PERIOD elapsed */
static uint16_t u16DFTIndex = 0;
///* DFT TX Message Time Period */
//static Timer_t DFT_TimeoutTimer = TIMER_DEFAULTS;

uint32_t G_Address[2]={0};

#if 0
/* Local Function Prototypes */
extern bool TransmitLeft;
extern bool TransmitRight;
extern bool TransmitBoth;
extern TRANSMITTING_MSG Transmitting;
extern Uint16 nvmidx;
extern bool TxMostRecentNvm;
extern bool TxNextRecentNvm;
extern bool TxAllNvm;
#endif
SCI_STATUS DFT_RxMsg(void);
bool DFT_TxBufferEmpty(void);
//SCI_STATUS DFT_SendMessage(void)
bool_t DFT_UpdateDataToTx(void);

/*************************************************************************************************\
* Function: DFT_Inint
*
* Purpose:  Serial DFT communications initialization function.
*
* Global Input(s):  None
* Global Output(s): None
* Input(s):         None
* Output(s):        None
*
\*************************************************************************************************/
void DFT_Init(void)
{
    /* GPIO init for SCI:(For Evaluation Board)*/
    /* Justification for Coding Standard Deviation:
     *    Assembler code needs to be executed.
     *    An exception to MISRA Rule 2.1 is required.
     */
    /*lint -e(586) # intended as is */
    EALLOW;
#if defined(DRV8312_DEV_KIT)
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0U;         // Enable pull-up for GPIO28 (SCIRXDA)
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0U;         // Enable pull-up for GPIO29 (SCITXDA)

    /* Configure SCI-A pins using GPIO regs*/
    // This specifies which of the possible GPIO pins will be SCI functional pins.

    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1U;        // Configure GPIO28 for SCIRXDA operation
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1U;        // Configure GPIO29 for SCITXDA operation  /*end*/

    GpioCtrlRegs.GPADIR.bit.GPIO28 = 1U;         // GPIO28 : Outpin
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 0U;         // GPIO29 : In pin


    /* GPIO7: TXENB and GPIO8: RXENB*/

    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0U;          // Enable pull-up for GPIO7 (TXENB)

    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0U;          // Enable pull-up for GPIO8 (RXENB)


    /* Configure SCI-A pins using GPIO regs*/
    // This specifies which of the possible GPIO pins will be SCI functional pins.

    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0U;
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0U;


    GpioCtrlRegs.GPADIR.bit.GPIO7 = 1U;         // GPIO7 : Out pin
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1U;         // GPIO8 : Out pin


    GpioDataRegs.GPADAT.bit.GPIO7 = 1U;         // GPIO7 :1 TXENB(TX enabled)
//    TimerApi_SleepTimeInNs((uint32_t)RS485_RXTXLINE_SET_DELAY);
    GpioDataRegs.GPADAT.bit.GPIO8 = 0U;         // GPIO8 :1 RXENB will be connected to ~RE, disable Rx line
//    TimerApi_SleepTimeInNs((uint32_t)RS485_RXTXLINE_SET_DELAY);


    /* Set qualification for selected pins to asynch only */
    // Inputs are synchronized to SYSCLKOUT by default.
    // This will select asynch (no qualification) for the selected pins.

    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3U;        // Asynch input GPIO28 (SCIRXDA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3U;        // Asynch input GPIO28 (SCIRXDA)

    PieVectTable.SCIA_RX_INT = &DFT_RxSrvc;

#else
    /* MCU HW */

    /* Configure SCI_C for Serial DFT Bus */
    /* SCI_C:  DFT:  RS485_EICAS Bus */

    /* SCI Peripheral is Group 0 for GPIO62 and GPIO63 */
    GpioCtrlRegs.GPBGMUX2.bit.GPIO62 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPBGMUX2.bit.GPIO63 = GPIO_MUX_GRP_0;      /* Select group 0 */

    /* SCI Peripheral is MUX1 for GPIO62 and GPIO63 */
    GpioCtrlRegs.GPBMUX2.bit.GPIO62 = GPIO_MUX_TYPE_1;      /* Configure GPIO62 for SCIRXDC operation */
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = GPIO_MUX_TYPE_1;      /* Configure GPIO63 for SCITXDC operation */

    /* Enable Pullups on TX and RX Pins */
    GpioCtrlRegs.GPBPUD.bit.GPIO62 = GPIO_ENABLE_PULLUP;    /* Enable pull-up for GPIO62 (SCIRXDC) */
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = GPIO_ENABLE_PULLUP;    /* Enable pull-up for GPIO63 (SCITXDC) */

    /* Set the Pin Directions for TX and RX pins */
    GpioCtrlRegs.GPBDIR.bit.GPIO62 = GPIO_INPUT;           /* GPIO62 : SCI_C_RX */
    GpioCtrlRegs.GPBDIR.bit.GPIO63 = GPIO_OUTPUT;            /* GPIO63 : SCI_C_TX  */

    /* Set qualification for selected pins to asynch only */
    /* Inputs are synchronized to SYSCLKOUT by default.
     * This will select asynch (no qualification) for the selected pins.
     */
    GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = GPIO_ASYNC;          /* Asynch input GPIO62 (SCIRXDC) */
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = GPIO_ASYNC;          /* Asynch input GPIO63 (SCITXDC) */

    /* Configure the GPIO20 as SCI_C_TX_EN Serial Tx Enable Line */
    GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = GPIO_MUX_GRP_0;      /* Select group 0 */
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = GPIO_MUX_TYPE_0;      /* Select as GPIO */
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = GPIO_ENABLE_PULLUP;    /* Enable pull-up for GPIO20 (TXENB) */
    GpioCtrlRegs.GPADIR.bit.GPIO20 = GPIO_OUTPUT;           /* Set as Output */
    GpioCtrlRegs.GPAODR.bit.GPIO20 = GPIO_NORMAL_OUTPUT;    /* Configure as normal output */


    PieVectTable.SCIC_RX_INT = &DFT_RxSrvc;

#endif
    EDIS;

    /************************ Configure the SCI *******************************************************/
    /* Setup serial parameters */
    DftSciRegs.SCICCR.bit.STOPBITS      = 0U;     /* Set to one stop bit */
    DftSciRegs.SCICCR.bit.PARITY        = 0U;     /* Odd Parity */
    DftSciRegs.SCICCR.bit.PARITYENA     = 1U;     /* Enable Parity */
    DftSciRegs.SCICCR.bit.SCICHAR       = SCICCHAR_VALUE;  /* Set to 8 bit Character Length */
    DftSciRegs.SCICCR.bit.ADDRIDLE_MODE = 0U;
    /* Set BAUD Rate Registers. LSPCLK is 100MHz as setup during InitSysCtrl clock setup routine.
     * 460,800 is BAUD RATE required as set by BRR registers SCIHBAUD and SCILBAUD. */
    DftSciRegs.SCIHBAUD.all = SCICHBAUD_VALUE;
    DftSciRegs.SCILBAUD.all = SCICLBAUD_VALUE;
    /* NOTE: The order in which the Tx/Rx FIFO's are reset wrt to the overall port */
    /* reset seems to be significant; care should be taken when reordering the     */
    /* initialization sequence so that proper and expected operation is preserved. */
    /* Put SCI port in reset */
    DftSciRegs.SCICTL1.bit.SWRESET = 0U;
    /* Reset the SCI Tx/Rx channels and the Tx FIFO; enable the transmit   */
    /* FIFO interrupt when FIFO is empty by setting the SCIRST, SCIFFENA,  */
    /* and TXFFIENA bits in the SCI FIFO transmit register                 */
    DftSciRegs.SCIFFTX.bit.SCIRST   = 1U;
    DftSciRegs.SCIFFTX.bit.SCIFFENA = 1U; /* Enable SCI FIFO Enhancements option */
    DftSciRegs.SCIFFTX.bit.TXFFIENA = 1U;
    DftSciRegs.SCIFFTX.bit.TXFFIL = 0U;   /* Set Tx FIFO Fill to 0 */


    /* Reset Rx FIFO; enable Rx FIFO to interrupt when FIFO has 1 word */
    DftSciRegs.SCIFFRX.bit.RXFFIENA = 1U; /* Enable Receive FIFO Interrupt Enable */
    DftSciRegs.SCIFFRX.bit.RXFFIL   = 1U; /* Set to interrupt at one word received */
    /* Enable the transmitter and receiver */
    DftSciRegs.SCICTL1.bit.TXENA = 1U;    /* Enable Transmitter */
    DftSciRegs.SCICTL1.bit.RXENA = 1U;    /* Enable Receiver */
    /* Enable Tx/Rx interrupts at the SCI */
    //DftSciRegs.SCICTL2.bit.TXINTENA   = 1U;   /* Enable the Transmit Interrupt */
    DftSciRegs.SCICTL2.bit.TXINTENA   = 0U;   /* Disable the Transmit Interrupt */
    DftSciRegs.SCICTL2.bit.RXBKINTENA = 1U;   /* Enable Receiver Buffer / Break Interrupt */
    //DftSciRegs.SCICTL2.bit.RXBKINTENA = 0U;   /* Disable Receiver Buffer / Break Interrupt */
    /* Remove SCI from reset */
    DftSciRegs.SCICTL1.bit.SWRESET = 1U;
    /* Remove Tx/Rx FIFOs from reset */
    DftSciRegs.SCIFFTX.bit.TXFIFORESET = 1U;
    DftSciRegs.SCIFFRX.bit.RXFIFORESET = 1U;
    DftSciRegs.SCIFFRX.bit.RXFFINTCLR = 1U;
    DftSciRegs.SCIFFTX.bit.TXFFINTCLR = 1U;

    /* Enable loopback for testing purposes only */

    DftSciRegs.SCIFFCT.all = 0x0; /* Disable Auto Baud */

#if defined(DRV8312_DEV_KIT)
    /* Enable SCI-A Rx interrupt (SCIRXINTA) */
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE Group 9, INT1

    /* Enable CPU level 9 INT */
    IER |= M_INT9;
#else
    /* SCI_C:  DFT:  RS485_EICAS Bus */
    /* Enable SCI-C Rx interrupt (SCIRXINTC) */
    PieCtrlRegs.PIEIER8.bit.INTx5 = 1U;

    /* Enable CPU level 8 INT */
    IER |= M_INT8;
#endif
    //DFT_InitializeTimers();

    /* shall initialize the DFT message timeout timer */
    EINT;
//    DFT_EnableRxLine();  /* Enable the TEST bus transceiver for Receive and not Transmit to Start */

}

//static uint16_t u16TxIsrCnt = 0U;
//static uint16_t u16TxBufCnt = 0U;
//static uint16_t u16TxFifoCnt = 0U;
///*************************************************************************************************\
//* Function: DFT_TxIsr
//*
//* Purpose:  Serial transmit interrupt handler.
//*
//* Global Input(s):  DFT_Tx_Buffer, DFT_Tx_Buffer_Head, DFT_Tx_Buffer_Tail
//* Global Output(s): DFT_Tx_Buffer_Head
//* Input(s):         None
//* Output(s):        None
//*
//\*************************************************************************************************/
//interrupt void DFT_TxIsr(void)
//{
//    uint16_t i;
//    uint16_t u16NumToSend = 0U;
//
//    /* Is there data to transmit? */
//    if (G_serial_tx_buffer_head != G_serial_tx_buffer_tail)
//    {
//        /* Calculate number of words ready in the buffer that can be transmitted */
//        if(G_serial_tx_buffer_tail >= G_serial_tx_buffer_head)
//        {
//            /* Tail has not wrapped around yet. Normal Calculation for number to send */
//            u16NumToSend = G_serial_tx_buffer_tail - G_serial_tx_buffer_head;
//        }
//        else
//        {
//            /* Tail wrapped around. Use buffer size to avoid negative difference */
//            u16NumToSend = (SERIAL_TX_BUF_SIZE + G_serial_tx_buffer_tail - G_serial_tx_buffer_head);
//        }
//
//        /* Check if number of words to send is greater than the FIFO size. */
//        if(u16NumToSend >= SERIAL_TX_FIFO_SIZE)
//        {
//            /* Set number to send to FIFO size to avoid dropping words to send */
//            u16NumToSend = SERIAL_TX_FIFO_SIZE;
//        }
//        /* Load the Serial TX Buffer with up to FIFO Size entries for transmission */
//        for(i = 0U; i < u16NumToSend; i++)
//        {
//            /* Transmit the data. Load the SCITXFUF */
//            DftSciRegs.SCITXBUF.all = G_serial_tx_buffer[G_serial_tx_buffer_head];
//            /* Update transmit buffer head */
//            G_serial_tx_buffer_head = SERIAL_TX_BUF_NEXT(G_serial_tx_buffer_head);
//            u16TxFifoCnt++;
//        }
//    }
//    else
//    {
//        /* Disable SCI-A Tx interrupt (SCITXINTA) */
//        PieCtrlRegs.PIEIER9.bit.INTx2 = 0U;
//    }
//
//    u16TxIsrCnt++;
//
//    /* Clear interrupt source */
//    DftSciRegs.SCIFFTX.bit.TXFFINTCLR = 1U;
//
//    /* Acknowledge interrupt */
//    PieCtrlRegs.PIEACK.bit.ACK9 = TRUE;
//
//}


///*************************************************************************************************\
//* Function: DFT_RxIsr
//*
//* Purpose:  Serial receive interrupt handler.
//*
//* Global Input(s):  G_serial_rx_buffer, G_serial_rx_buffer_head, G_serial_rx_buffer_tail
//* Global Output(s): G_serial_rx_buffer, G_serial_rx_buffer_tail
//* Input(s):         None
//* Output(s):        None
//*
//\*************************************************************************************************/
//interrupt void DFT_RxIsr(void)
//{
//    uint16_t newtail;
//
//    newtail = SERIAL_RX_BUF_NEXT(G_serial_rx_buffer_tail);
//
//    /* Is there room for 1 more byte in the buffer? */
//    if (newtail != G_serial_rx_buffer_head)
//    {
//        /* Get the received data from the lower 8-bits of the RXBUF register */
//        G_serial_rx_buffer[G_serial_rx_buffer_tail] = (uint16_t)DftSciRegs.SCIRXBUF.bit.SAR;
//        G_serial_rx_buffer_tail = newtail;
//    }
//
//    /* Clear overflow flag */
//    DftSciRegs.SCIFFRX.bit.RXFFOVRCLR = 1U;
//
//    /* Clear interrupt source */
//    DftSciRegs.SCIFFRX.bit.RXFFINTCLR = 1U;
//
//    /* Acknowledge interrupt */
//    PieCtrlRegs.PIEACK.bit.ACK9 = TRUE;
//
//}

///*************************************************************************************************\
//* Function: DFT_TxBufferEmpty
//*
//* Purpose:  Query transmit buffer status function.
//*
//* Global Input(s):  None
//* Global Output(s): None
//* Input(s):         None
//* Output(s):        DFT_TxBufferEmptyStatus
//*                   TRUE if the transmit buffer and shift register are both empty
//*                   FALSE is returned if the transmit buffer or transmit shift register
//*                   contains any data remaining to be transmitted.
//*
//*
//\*************************************************************************************************/
//bool DFT_TxBufferEmpty(void)
//{
//    bool serial_txbufferemptystatus = FALSE;
//
//    /* If our buffer is empty (it is not empty if head and tail differ) and
//     * if Tx buffer and shift register is empty, the FIFO is empty, and the TXBUF is Ready to reload
//     * then the TX FIFO operation is complete and return TRUE */
//    if ((DftSciRegs.SCICTL2.bit.TXEMPTY == 1U) && (G_serial_tx_buffer_head == G_serial_tx_buffer_tail) &&
//        (DftSciRegs.SCICTL2.bit.TXRDY == 1U) && (DftSciRegs.SCIFFTX.bit.TXFFST == 0U))
//    {
//        serial_txbufferemptystatus = TRUE;
//    }
//
//    return serial_txbufferemptystatus;
//}

/*************************************************************************************************\
* Function: DFT_DisableRxLine
*
* Purpose:  Disable Serial RX line
*
* Global Input(s):  None
* Global Output(s): None
* Input(s):         None
* Output(s):        None
*                   
*
\*************************************************************************************************/
void DFT_DisableRxLine(void)
{

#if defined(DRV8312_DEV_KIT)
    DftSciRegs.SCICTL1.bit.RXENA = 0U;            /* Disable the Receiver so we don't receive the loopbacked Tx  */
    DftSciRegs.SCICTL1.bit.TXENA = 1U;            /* Enable Transmitter */
#else /* MCU HW */

    ScicRegs.SCICTL1.bit.RXENA = 0U;            /* Disable the Receiver so we don't receive the loopbacked Tx  */
    DFT_SCI_C_TX_EN_set();        /* Enable the SCI_C_TX_EN to enable the chip for Transmission */
#endif

}

/*************************************************************************************************\
* Function: DFT_EnableRxLine
*
* Purpose:  Enable Serial RX line
*
* Global Input(s):  None
* Global Output(s): None
* Input(s):         None
* Output(s):        None
*                   
*
\*************************************************************************************************/
void DFT_EnableRxLine(void)
{
#if defined(DRV8312_DEV_KIT)
//    TimerApi_SleepTimeInNs((uint32_t)RS485_RXTXLINE_SET_DELAY);
    DftSciRegs.SCICTL1.bit.TXENA = 0U;            /* Disable Transmitter */
    DftSciRegs.SCICTL1.bit.RXENA = 1U;            /* Enable the Receiver for Rx comms  */
#else /* MCU HW */

    DFT_SCI_C_TX_EN_clr();        /* Disable the SCI_C_TX_EN to enable the chip for Receive */
    ScicRegs.SCICTL1.bit.RXENA = 1U;            /* Enable the Receiver for Rx comms  */

#endif

}

/*************************************************************************************************\
* Function: DFT_TxMsg
*
* Purpose:  Transmit DFT message onto SCI RS485 bus
*
\*************************************************************************************************/
SCI_STATUS DFT_TxMsg(Uint16 length)
{
    static Uint16 checksum = 0;
    SCI_STATUS status = TX_IN_PROGRESS;
    uint16_t u16SubIndex = 0U;  /* used to send in 32-bit chunks */
    t32Data_t t32Data = {0};
    uint16_t u16CharToSend = 0U;
    uint16_t i = 0;



    /* loop through the message buffer */
    /*shall verify there is still room in the FIFO for at least 2 bytes, otherwise exit */
    //while ((DFT_TxIndex < length) && (DftSciRegs.SCIFFTX.bit.TXFFST <= MAX_STUFFED_FIFO_COUNT))
    if ((DFT_TxIndex < length) && (DftSciRegs.SCIFFTX.bit.TXFFST <= DFT_MAX_FIFO_COUNT))
    {
        /* PATH(DFT_TxMsg,K); */
        /* Update the 4 bytes of data to be sent out next */
        DFT_UpdateDataToTx();

        /* shall check the value of the byte transmitted for a match with the Data Link Escape
           character. If one is found, another DLE character will be immediately afterwards
           (as long as the current word is not Word 00 or Word 01) */

        /* Send out 4 at a time */
        for (i = 0U; i < 4; i++)
        {
            //DftSciRegs.SCITXBUF.all = DFT_TxBuf[DFT_TxIndex];
            u16SubIndex = (DFT_TxIndex / 4U); /* Get the sub-index we are on to send the correct 32-bit word */
            t32Data.u32Data = DFT_TxBuf.t32[u16SubIndex].u32Data;

            /* decode which byte to send */
            switch (i)
            {
                case 0:
                {
                    u16CharToSend = t32Data.t8.byte0;
                    break;
                }
                case 1:
                {
                    u16CharToSend = t32Data.t8.byte1;
                    break;
                }
                case 2:
                {
                    u16CharToSend = t32Data.t8.byte2;
                    break;
                }
                case 3:
                {
                    u16CharToSend = t32Data.t8.byte3;
                    break;
                }
            }

            /* Send the byte onto the SCI bus */
            DftSciRegs.SCITXBUF.all = u16CharToSend;

            if (((u16CharToSend & 0xFF) == DLE_CHAR) && (DFT_TxIndex > 1))
            {
                /* PATH(DFT_TxMsg,B); */
                /* since the buffer value matched DLE_CHAR, just assign #define */
                /* insert an escape character if the current buffer value matches hex 10 */
                DftSciRegs.SCITXBUF.all = DLE_CHAR;
            }

            /* add byte value to the checksum, but not if it is a beginning DLE */
            if (DFT_TxIndex != 0)
            {
                /* PATH(DFT_TxMsg,D); */
                checksum += u16CharToSend;
            }

            DFT_TxIndex++;
        }
    }

    /* send out the checksum, if there's room in the FIFO */
    if ((DFT_TxIndex == length) && (DftSciRegs.SCIFFTX.bit.TXFFST <= MAX_STUFFED_FIFO_COUNT))
    {
        /* PATH(DFT_TxMsg,E); */
        /*shall calculate the message checksum by taking the 2's complement of the summation */
        checksum = -checksum;

        /*shall check to see if the checksum matches the value of the data link escape character
               and insert another DLE character immediately afterwards if a match is found */
        if ((checksum & 0x00FF) == DLE_CHAR)
        {
            /* PATH(DFT_TxMsg,F); */
            /* since the checksum value matched DLE_CHAR, just assign DLE_CHAR to the buffer */
            DftSciRegs.SCITXBUF.all = DLE_CHAR;
            /* followup with another data link escape character */
            DftSciRegs.SCITXBUF.all = DLE_CHAR;
        }
        else
        {
            /* PATH(DFT_TxMsg,G); */
            DftSciRegs.SCITXBUF.all = (checksum & 0x00FF);
        }

        DFT_TxIndex++;
    }

    /*shall send the closing data link escape character once all of the message data words
        have been transmitted. */
    if ((DFT_TxIndex == (length + 1)) && (DftSciRegs.SCIFFTX.bit.TXFFST < MAX_FIFO_COUNT))
    {
        /* PATH(DFT_TxMsg,H); */
        DftSciRegs.SCITXBUF.all = DLE_CHAR;
        DFT_TxIndex++;
    }

    /*shall send the end of transmission character following the closing data link escape character */
    if ((DFT_TxIndex == (length + 2)) && (DftSciRegs.SCIFFTX.bit.TXFFST < MAX_FIFO_COUNT))
    {
        /* PATH(DFT_TxMsg,I); */
        DftSciRegs.SCITXBUF.all = EOTX_CHAR;
        checksum = 0;
        status = TX_DONE;
#if 0
        Transmitting.NVM_Data = false;
        Transmitting.IBIT_Message = false;
#endif
        DFT_Tx_cmd = false;
        DFT_TxIndex = 0;
    }
    /* PATH(DFT_TxMsg,J); */

    return (status);
}

/*************************************************************************************************\
* Function: DFT_RxSrvc
*
* Purpose:  Service to receive DFT messages on the SCI RS485 Bus
*
\*************************************************************************************************/
interrupt void DFT_RxSrvc(void)
{
    SCI_STATUS status;
    DFT_ADDRESS *selectMsg;

    status = DFT_RxMsg();

    /*shall check for indication that there is a complete message in the receive buffer */
    if (status == VALID_RX_MSG)
    {
        /* PATH(DFT_RxSrvc,C); */
        /* parse the received data from the DFT GSE */
        Uint16 Rxbuf[13] = {0}; /* buffer for unstuffing the received message */
        int16 chksum;                        /* used for verifying the checksum */
        Uint16 x, y;                         /* loop variables */

        /* PATH(DFT_RxSrvc,A); */

        /*shall remove any stuffed data link escape characters within the received message */
        for (x = 1, y = 1, Rxbuf[0] = DLE_CHAR; y < DFT_RxNumBytes; x++, y++)
        {
            /* PATH(DFT_RxSrvc,K); */
            Rxbuf[x] = DFT_RxBuf[y];

            /* if the current character is not Word 01 of the message and two consecutive words
               contain the data link escape character, skip past the stuffed escape character */

            if (( y != 1) && ((DFT_RxBuf[y] & 0xFF) == DLE_CHAR) && ((DFT_RxBuf[y + 1] & 0xFF) == DLE_CHAR))
            {
                /* PATH(DFT_RxSrvc,B); */
                y++;
            }
        }

        /*shall clear out the global number of received bytes */
        DFT_RxNumBytes = 0;
        /*shall typecast the buffer to the select message type */
        selectMsg = (DFT_ADDRESS *)Rxbuf;

        /*shall validate the message by adding the message checksum to message data words (DLE, EOTX, and "stuffed" characters excluded) */
        /*   and verifying the result is 0 */
//        chksum = selectMsg->checksum + selectMsg->ChanDataID + selectMsg->DataPktSize + selectMsg->FSCUChanID +
//                selectMsg->Addbyte0 + selectMsg->Addbyte1 + selectMsg->Addbyte2 + selectMsg->Addbyte3 + selectMsg->AddID;
        chksum = selectMsg->checksum + selectMsg->Addbyte0 + selectMsg->Addbyte1 + selectMsg->Addbyte2 +
                selectMsg->Addbyte3 + selectMsg->AddID;

        /* does the checksum indicate valid received data? */
        if ((chksum & 0xFF) == 0x00)
        {
            /* PATH(ProcessCommandMsg,C); */
            /*shall verify that this message was meant for this MCU */
            switch(selectMsg->AddID)
            {
            case 0:
                G_Address[0] = selectMsg->Addbyte3;
                G_Address[0] = (G_Address[0] << 8) | selectMsg->Addbyte2;
                G_Address[0] = (G_Address[0] << 8) | selectMsg->Addbyte1;
                G_Address[0] = (G_Address[0] << 8) | selectMsg->Addbyte0;
                break;
            case 1:
                G_Address[1] = selectMsg->Addbyte3;
                G_Address[1] = (G_Address[1] << 8) | selectMsg->Addbyte2;
                G_Address[1] = (G_Address[1] << 8) | selectMsg->Addbyte1;
                G_Address[1] = (G_Address[1] << 8) | selectMsg->Addbyte0;
                break;
            default:
                break;
            }
            /* PATH(DFT_TxMsg,A); */
            /* Disable the Transceiver Receive Functionality to Enable the Transmit function. */
            DFT_Tx_cmd = true;
            DFT_Tx_block = false;
        }
    }
    /* Clear overflow flag */
    DftSciRegs.SCIFFRX.bit.RXFFOVRCLR = 1U;

    /* Clear interrupt source */
    DftSciRegs.SCIFFRX.bit.RXFFINTCLR = 1U;

#if defined(DRV8312_DEV_KIT)
    /* Acknowledge interrupt */
    PieCtrlRegs.PIEACK.bit.ACK9 = TRUE;
#else
    /* Acknowledge interrupt */
    PieCtrlRegs.PIEACK.bit.ACK8 = TRUE;
#endif

}

/*************************************************************************************************\
* Function: DFT_RxSrvc
*
* Purpose:  Function to receive a DFT message off the SCI RS485 Bus
*
\*************************************************************************************************/
SCI_STATUS DFT_RxMsg(void)
{
    Uint16 x, rxwords;
    SCI_STATUS status = RX_IN_PROGRESS;
    static Uint16 DLE_CHAR_Num = 0;
    Uint16 temp = 0;
    static Uint16 prevtemp = 0;
    static SCI_STATUS flag = INVALID_RX_MSG;    /* PATH(DFT_RxMsg,A); */

    /*shall check that the receive FIFO byte count is greater than 0 for SCI-A*/

    if (DftSciRegs.SCIFFRX.bit.RXFFST > 0)
    {
        /* PATH(DFT_RxMsg,B); */
        /*shall read the FIFO byte count */

        rxwords = DftSciRegs.SCIFFRX.bit.RXFFST;

        for (x = 0; x < rxwords; x++)
        {
            /* PATH(DFT_RxMsg,C); */
            /*shall transfer a number of bytes from the FIFO to the global Rx buffer
              equal to the byte count read */

            temp = DftSciRegs.SCIRXBUF.all;

            if (((temp & 0xFF) == DLE_CHAR) && (prevtemp != DLE_CHAR))
            {
                flag = RX_IN_PROGRESS;
            }
            else if (((temp & 0xFF) == EOTX_CHAR) && (prevtemp == DLE_CHAR))
            {
                flag = VALID_RX_MSG;
            }

            prevtemp = temp;
            if(flag == RX_IN_PROGRESS)
            {
                DFT_RxBuf[DFT_RxIndex] = temp;
                /*shall increment the global received byte count */

                DFT_RxNumBytes++;

                /* If the current word being processed contains the DLE character, increment the
                   number of data link escape character found. If the current word being processed
                   contains the End Of Transmission character and an even number of DLE characters
                   have been processed, the end of a message has been reached. */

                if ((DFT_RxBuf[DFT_RxIndex] & 0xFF) == DLE_CHAR)
                {
                   /* PATH(DFT_RxMsg,D); */
                    DLE_CHAR_Num++;
                }
                else  if (((DFT_RxBuf[DFT_RxIndex] & 0xFF) == EOTX_CHAR) && ((DLE_CHAR_Num & 0x01) == 0))
                {
                    /* PATH(DFT_RxMsg,E); */
                    /*The end of the messages has been reached so set the return status as VALID_RX_MSG & reset # of DLEs found. */

                    status = VALID_RX_MSG;
                    DLE_CHAR_Num = 0;
                    x = rxwords;        /* set x equal to rxwords to break out of the loop */
                }

                if ((status == VALID_RX_MSG) || (DFT_RxIndex == (DFT_RX_BUF_SIZE - 1)))
                {
                     /* PATH(DFT_RxMsg,F); */
                      DFT_RxIndex = 0;
                }
                else
                {
                    /* PATH(DFT_RxMsg,G); */
                    DFT_RxIndex++;
                }
            }
            else if(flag == VALID_RX_MSG)
            {
                DFT_RxBuf[DFT_RxIndex] = temp;
                status = VALID_RX_MSG;
                flag = INVALID_RX_MSG;
                DLE_CHAR_Num = 0;
                x = rxwords;        /* set x equal to rxwords to break out of the loop */
                DFT_RxIndex = 0;
//                /* Re-enable SCI-A Tx interrupt (SCITXINTA) */
//                PieCtrlRegs.PIEIER9.bit.INTx2 = 1U;
            }
            else
            {
                if(temp == 0x25)
                {
                    DFT_Tx_block = true;
                }
                else
                {
                    DFT_Tx_block = false;
                }
                DLE_CHAR_Num = 0;
                x = rxwords;        /* set x equal to rxwords to break out of the loop */
                DFT_RxIndex = 0;
            }
        }
    }

    /* PATH(DFT_RxMsg,H); */
    return status;
}

/*************************************************************************************************\
* Function: DFT_TxBufferEmpty
*
* Purpose:  Query transmit buffer status function.
*
* Global Input(s):  None
* Global Output(s): None
* Input(s):         None
* Output(s):        DFT_TxBufferEmptyStatus
*                   TRUE if the transmit buffer and shift register are both empty
*                   FALSE is returned if the transmit buffer or transmit shift register
*                   contains any data remaining to be transmitted.
*
**************************************************************************************************/
bool DFT_TxBufferEmpty(void)
{
    bool serial_txbufferemptystatus = FALSE;

    /* If our buffer is empty (it is not empty if head and tail differ) and
     * if Tx buffer and shift register is empty, the FIFO is empty, and the TXBUF is Ready to reload
     * then the TX FIFO operation is complete and return TRUE */
    if ((DftSciRegs.SCICTL2.bit.TXEMPTY == 1U) && (DftSciRegs.SCICTL2.bit.TXRDY == 1U) && (DftSciRegs.SCIFFTX.bit.TXFFST == 0U))
    {
        serial_txbufferemptystatus = TRUE;
    }

    return serial_txbufferemptystatus;
}

/*
    brief Update the maintenance communication interface on a periodic basis.

     Purpose:
        This routine is responsible for transmitting the DFT message onto the bus. A new
        DFT message is created when the timer expires.

    return  void

    Global Data Referenced:

    Preconditions and Assumptions:

*/
void DFT_TxSrvc(void)
{
    // DFT_MAX_PERIOD*0.125 = 80*0.125 = 10ms
    // Tx time : (DFT_MSG_SIZE/4)*0.125 = 67 * 0.125 = 8.325ms
    // Rx time : (DFT_MAX_PERIOD-(DFT_MSG_SIZE/4))*0.125 = 13*0.125 = 1.625ms
    if(DFT_Period >= DFT_MAX_PERIOD)
    {
        DFT_Tx_cmd = true;
        DFT_Period = 0;
        u16DFTIndex = 0;
    }
    if (DFT_Tx_cmd == true)
    {
        DFT_DisableRxLine();
        DFT_TxMsg(DFT_MSG_SIZE);
    }
    else
    {
        if(DFT_TxBufferEmpty() == TRUE)
        {
            DFT_EnableRxLine();
        }
    }
    DFT_Period++;

//    /* Clear interrupt source */
//    DftSciRegs.SCIFFTX.bit.TXFFINTCLR = 1U;
//
//    /* Acknowledge interrupt */
//    PieCtrlRegs.PIEACK.bit.ACK9 = TRUE;

//    static bool_t bSendMsg = false;
//    static SCI_STATUS status = TX_DONE;
//
//    /* Begin transmission of a new DFT Message when DFT timer expires */
//    //if(Timer_IsExpired(&DFT_TimeoutTimer) == true)
//    if(status == TX_DONE)
//    {
//        bSendMsg = true;        /* start sending a new DFT packet out */
//        status = TX_IN_PROGRESS;
//        //DFT_CreateMessage();    /* Create the new DFT Message to Transmit */
//        //Timer_SetTime(&DFT_TimeoutTimer, TIMER_ONESHOT, TIMER_10ms);       /* Init DFT timer */
//    }
//
//    /* Align the start of DFT transmission with the DFT Timeout. Send until the
//     * entire message has been sent out on the bus and then reset for next message. */
//    if(bSendMsg == true)
//    {
//        status = DFT_TxMsg(DFT_MSG_SIZE);
//
//        /* Align the message transmission with the next DFT period */
//        if(status == TX_DONE)
//        {
//            bSendMsg = false;
//        }
//    }
}

/*
    brief Create a new DFT message to transmit

     Purpose:
        This routine can be called to create a message containing the standard
        maintenance interface update information (DFT).

     Preconditions and Assumptions:

*/
bool_t DFT_UpdateDataToTx(void)
{
    bool_t bDone = false;

    /* Disable Interrupts to ensure integrity of data */
    DINT;

    switch (u16DFTIndex)
    {
        case 0:
        {   /* DWord 0 */
            DFT_TxBuf.tData.tDft_0.BeginDLE = DLE_CHAR;
            DFT_TxBuf.tData.tDft_0.DataPktSize = DFT_MSG_SIZE;
            /* shall set the channel and MCU ID data words based on the PrimarySide flag */
            if (G_bChannelA == true)
            {
//                 DFT_TxBuf.tData.tDft_0.ChanDataID = DFT_LFT_CHN_ID;
                 DFT_TxBuf.tData.tDft_0.bMcuChannelA = 1U;
                 DFT_TxBuf.tData.tDft_0.bMcuChannelB = 0U;
            }
            else if (G_bChannelA == false)
            {
//                 DFT_TxBuf.tData.tDft_0.ChanDataID = DFT_RT_CHN_ID;
                 DFT_TxBuf.tData.tDft_0.bMcuChannelA = 0U;
                 DFT_TxBuf.tData.tDft_0.bMcuChannelB = 1U;
            }
            DFT_TxBuf.tData.tDft_0.bFasStatusFlapFail = (uint16_t)bFlapFail;
            DFT_TxBuf.tData.tDft_0.bFasStatusNotAvailable = (uint16_t)bFasNotAvailable;
            DFT_TxBuf.tData.tDft_0.bFasStatusRigInProcess = (uint16_t)bRigInProcess;
            DFT_TxBuf.tData.tDft_0.bFasStatusGseConnected = (uint16_t)bGseConnected;
            DFT_TxBuf.tData.tDft_0.bFasStatusFlapPositionFail = (uint16_t)bFlapPosMiscompare;
            DFT_TxBuf.tData.tDft_0.bValidCmd = (uint16_t)tPanel.bValidCmd;
            break;
        }
        case 1:
        {   /* DWord 1 */
            DFT_TxBuf.tData.tDft_1.u32OfpCrc = CSCI_APP_HEADER.CRC;
            break;
        }
        case 2:
        {
            DFT_TxBuf.tData.tDft_2.u32PdiCrc = CSCI_PDI_HEADER.CRC;
            break;
        }
        case 3:
        {
            DFT_TxBuf.tData.tDft_3.u32BldCrc = CSCI_BLD_HEADER.CRC;
            break;
        }
        case 4:
        {
            DFT_TxBuf.tData.tDft_4.u16OfpSwVersion_Build = CSCI_APP_HEADER.tSwVersion.u16Build;
            DFT_TxBuf.tData.tDft_4.u16OfpSwVersion_Revision = CSCI_APP_HEADER.tSwVersion.u16Revision;
            DFT_TxBuf.tData.tDft_4.u16OfpSwVersion_Minor = CSCI_APP_HEADER.tSwVersion.u16Minor;
            DFT_TxBuf.tData.tDft_4.u16OfpSwVersion_Major = CSCI_APP_HEADER.tSwVersion.u16Major;
            break;
        }
        case 5:
        {
            DFT_TxBuf.tData.tDft_5.u32BldSwVersion_Build = CSCI_BLD_HEADER.tSwVersion.u16Build;
            DFT_TxBuf.tData.tDft_5.u32BldSwVersion_Revision = CSCI_BLD_HEADER.tSwVersion.u16Revision;
            DFT_TxBuf.tData.tDft_5.u32BldSwVersion_Minor = CSCI_BLD_HEADER.tSwVersion.u16Minor;
            DFT_TxBuf.tData.tDft_5.u32BldSwVersion_Major = CSCI_BLD_HEADER.tSwVersion.u16Major;
            break;
        }
        case 6:
        {
            DFT_TxBuf.tData.tDft_6.u32PdiSwVersion_Build = CSCI_PDI_HEADER.tSwVersion.u16Build;
            DFT_TxBuf.tData.tDft_6.u32PdiSwVersion_Revision = CSCI_PDI_HEADER.tSwVersion.u16Revision;
            DFT_TxBuf.tData.tDft_6.u32PdiSwVersion_Minor = CSCI_PDI_HEADER.tSwVersion.u16Minor;
            DFT_TxBuf.tData.tDft_6.u32PdiSwVersion_Major = CSCI_PDI_HEADER.tSwVersion.u16Major;
            break;
        }
        case 7:
        {
            DFT_TxBuf.tData.tDft_7.LatchedFaultsStatb0 = (uint16_t) (LatchedFaults_Stat.all & 0x000000FFUL);
            DFT_TxBuf.tData.tDft_7.LatchedFaultsStatb1 = (uint16_t)((LatchedFaults_Stat.all & 0x0000FF00UL) >> 8UL);
            DFT_TxBuf.tData.tDft_7.LatchedFaultsStatb2 = (uint16_t)((LatchedFaults_Stat.all & 0x00FF0000UL) >> 16UL);
            DFT_TxBuf.tData.tDft_7.LatchedFaultsStatb3 = (uint16_t)((LatchedFaults_Stat.all & 0xFF000000UL) >> 24UL);
            break;
        }
        case 8:
        {
            DFT_TxBuf.tData.tDft_8.CriticalFaultsStatb0 = (uint16_t) (CriticalFaults_Stat.all & 0x000000FFUL);
            DFT_TxBuf.tData.tDft_8.CriticalFaultsStatb1 = (uint16_t)((CriticalFaults_Stat.all & 0x0000FF00UL) >> 8UL);
            DFT_TxBuf.tData.tDft_8.CriticalFaultsStatb2 = (uint16_t)((CriticalFaults_Stat.all & 0x00FF0000UL) >> 16UL);
            DFT_TxBuf.tData.tDft_8.McuMode = McuGetState();
            DFT_TxBuf.tData.tDft_8.u16Nvm_StateMode = Nvm_State.Mode;
            break;
        }
        case 9:
        {
            DFT_TxBuf.tData.tDft_9.u16InhibitsStat = Inhibits_Stat.all;
            DFT_TxBuf.tData.tDft_9.u16LatchedWarnings = LatchedWarnings_Stat.all;
            break;
        }
        case 10:
        {
            DFT_TxBuf.tData.tDft_10.FaultCode = ReportedFaultCode;
            DFT_TxBuf.tData.tDft_10.FaultData = ReportedFaultData;
            DFT_TxBuf.tData.tDft_10.tFasLruStatus.bit.bMcuFail = (Uint16)bMcuFail;
            DFT_TxBuf.tData.tDft_10.tFasLruStatus.bit.bScuFail = (Uint16)bScuFail;
            DFT_TxBuf.tData.tDft_10.tFasLruStatus.bit.bFlaFail = (Uint16)bFlaFail;
            DFT_TxBuf.tData.tDft_10.tFasLruStatus.bit.bFpsuFail = (Uint16)bFpsuFail;
            DFT_TxBuf.tData.tDft_10.tFasLruStatus.bit.bMotorFail = (Uint16)bMotorFail;
            DFT_TxBuf.tData.tDft_10.tFasLruStatus.bit.bFlapJam = (Uint16)bFlapJam;
            break;
        }
        case 11:
        {
            DFT_TxBuf.tData.tDft_11.f32FlapAngleSkewSnsr = tSkewSnsrCalcs.tFlapAngle.f32FlapAngle;
            break;
        }
        case 12:
        {
            DFT_TxBuf.tData.tDft_12.f32StrokeSkewSnsr = tSkewSnsrCalcs.tStroke.f32Stroke;
            break;
        }
        case 13:
        {
            DFT_TxBuf.tData.tDft_13.f32StrokeQuad = tPanel.f32StrokeQuad;
            break;
        }
        case 14:
        {
            DFT_TxBuf.tData.tDft_14.f32BiasFused = G_ASW_DATA.tPanelOutputData.f32BiasFused;
            break;
        }
        case 15:
        {
            DFT_TxBuf.tData.tDft_15.f32QuadCntResidualFused = G_ASW_DATA.tPanelOutputData.f32QuadCntResidualFused;
            break;
        }
        case 16:
        {
            DFT_TxBuf.tData.tDft_16.f32SkewResidualFused = G_ASW_DATA.tPanelOutputData.f32SkewResidualFused;
            break;
        }
        case 17:
        {
            DFT_TxBuf.tData.tDft_17.f32StopPosition = MotorCmd.f32StopPosition;
            break;
        }
        case 18:
        {
            DFT_TxBuf.tData.tDft_18.f32StrokeScaledPositionCalibrated = tSkewSnsrCalcs.tStroke.f32ScaledPositionCalibrated;
            break;
        }
        case 19:
        {
            /*Commenting out as  GPIO14 (EPM8A) is used for other purpose in MCU software*/
            /*DFT_TxBuf.tData.tDft_19.u16PWM_RST_N_CMPA = EPwm8Regs.CMPA.bit.CMPA;*/
#if defined(__SKEW_SNSR_RVDT__)
            DFT_TxBuf.tData.tDft_19.f32RvdtCalCoeffA1 = Nvm_Rigging_Temp.tSkewSnsr.tRvdtCalibration.f32a1;
#endif
            break;
        }
        case 20:
        {
#if defined(__SKEW_SNSR_RVDT__)
            DFT_TxBuf.tData.tDft_20.f32RvdtCalCoeffA0 = Nvm_Rigging_Temp.tSkewSnsr.tRvdtCalibration.f32a0;
#endif
            break;
        }
        case 21:
        {
#if defined(__SKEW_SNSR_RVDT__)
            DFT_TxBuf.tData.tDft_21.f32RvdtCalCoeffA3 = Nvm_Rigging_Temp.tSkewSnsr.tRvdtCalibration.f32a3;
#endif
            break;
        }
        case 22:
        {
#if defined(__SKEW_SNSR_RVDT__)
            DFT_TxBuf.tData.tDft_22.f32RvdtCalCoeffA2 = Nvm_Rigging_Temp.tSkewSnsr.tRvdtCalibration.f32a2;
#endif
            break;
        }
        case 23:
        {
            DFT_TxBuf.tData.tDft_23.f32PositionCmd = tPanel.f32PositionCmd;
            break;
        }
        case 24:
        {
            DFT_TxBuf.tData.tDft_24.f32SpeedCmd = tPanel.f32SpeedCmd;
            break;
        }
        case 25:
        {
            DFT_TxBuf.tData.tDft_25.f32PositionFdbk = G_ASW_DATA.tPanelOutputData.f32StrokeFused;
            break;
        }
        case 26:
        {
            DFT_TxBuf.tData.tDft_26.f32Speed = tSpeed.Speed;
            break;
        }
        case 27:
        {
            DFT_TxBuf.tData.tDft_27.G_tGpioInputs = G_tGpioInputs.all;
            break;
        }
        case 28:
        {
            DFT_TxBuf.tData.tDft_28.f32MotorSpeed = MotorCmd.MotorSpeed;
            break;
        }
        case 29:
        {
            DFT_TxBuf.tData.tDft_29.s32SpeedRpm = tSpeed.SpeedRpm;
            break;
        }
        case 30:
        {
            DFT_TxBuf.tData.tDft_30.s16QuadFused = G_ASW_DATA.tPanelOutputData.s16QuadFused;
#if defined(__SKEW_SNSR_RVDT__)
            DFT_TxBuf.tData.tDft_30.s16StrokeRvdtAdcAvgRaw = tRvdtCalcs.tStroke.s16AdcAvgRaw;
#endif
            break;
        }
        case 31:
        {
            DFT_TxBuf.tData.tDft_31.u16MotorCmdLastBadHallState = (uint16_t) MotorCmd.LastBadHallState;
            DFT_TxBuf.tData.tDft_31.G_ACTUATOR_ID = (G_eActuatorNumber & 0x0F);
            DFT_TxBuf.tData.tDft_31.CurrentBusState = CurrentBusState;
            DFT_TxBuf.tData.tDft_31.G_eLruId = G_eLruId;
            DFT_TxBuf.tData.tDft_31.MotorRunning = MotorCmd.MotorRunning;
            DFT_TxBuf.tData.tDft_31.BrakeActivated = MotorCmd.BrakeActivated;
            DFT_TxBuf.tData.tDft_31.MotorStart = MotorCmd.MotorStart;
            DFT_TxBuf.tData.tDft_31.MotorStop = MotorCmd.MotorStop;
#if defined(__SKEW_SNSR_RVDT__)
            DFT_TxBuf.tData.tDft_31.RVDTEnable = (uint16_t)RVDTEnable;
#endif
            DFT_TxBuf.tData.tDft_31.u16MotorCmdIllegalHalls = MotorCmd.IllegalHalls;
            break;
        }
        case 32:
        {
            DFT_TxBuf.tData.tDft_32.u16Hall1HallGpio = tHall.HallGpio;
            DFT_TxBuf.tData.tDft_32.s16Hall1Revolutions = tHall.Revolutions;
            break;
        }
        case 33:
        {
            DFT_TxBuf.tData.tDft_33.s16Hall1Direction = tHall.Direction;
            DFT_TxBuf.tData.tDft_33.s16Hall1Position = tHall.Position;
            break;
        }
        case 34:
        {
            DFT_TxBuf.tData.tDft_34.s16PositionCmdQuad = tPanel.s16PositionCmdQuad;
            DFT_TxBuf.tData.tDft_34.s16StopPosition = MotorCmd.StopPosition;
            break;
        }
        case 35:
        {
            DFT_TxBuf.tData.tDft_35.s16MotorDirection = MotorCmd.MotorDirection;
            DFT_TxBuf.tData.tDft_35.u16BrakeDutyCycle = MotorCmd.u16BrakeDutyCycle;
            break;
        }
        case 36:
        {
            DFT_TxBuf.tData.tDft_36.u16RVDT_POS_raw = Adc_Raw.val.u16_RVDT_POS;
            DFT_TxBuf.tData.tDft_36.u16RVDT_SUM_raw = Adc_Raw.val.u16_RVDT_SUM;
            break;
        }
        case 37:
        {
            DFT_TxBuf.tData.tDft_37.s16QuadCntSkewSnsr = tSkewSnsrCalcs.tStroke.s16QuadCnt;
            DFT_TxBuf.tData.tDft_37.u16NVM_Rig_PRGM_Pins = tNvm.tData.Nvm_Rigging.tPrgm.all;
            break;
        }
        case 38:
        {
            DFT_TxBuf.tData.tDft_38.u16Nvm_State_Pointer = tNvm.tData.Nvm_State_Pointer;
            DFT_TxBuf.tData.tDft_38.u16Nvm_RiggingCRC = tNvm.tData.Nvm_Rigging.crc;
            break;
        }
        case 39:
        {
            DFT_TxBuf.tData.tDft_39.u16Pwm1CmtnPointer = tPwm.CmtnPointer;
            DFT_TxBuf.tData.tDft_39.s16Pwm1DutyFunc = tPwm.DutyFunc;
            break;
        }
        case 40:
        {
            DFT_TxBuf.tData.tDft_40.f32G_SpeedPidOutput = G_ASW_DATA.tAnalogOutputData.f32SpeedPidOutput;
            break;
        }
        case 41:
        {
            DFT_TxBuf.tData.tDft_41.u16WDRS = tNvm.tData.WDRS;
            DFT_TxBuf.tData.tDft_41.u16WdrsCounter = tNvm.tData.WDRS_Counter;
            break;
        }
        case 42:
        {
            DFT_TxBuf.tData.tDft_42.tWctaFrame0_u16Avg = tWctaFrameTime[0].u16Avg;
            DFT_TxBuf.tData.tDft_42.tWctaFrame1_u16Avg = tWctaFrameTime[1].u16Avg;
            break;
        }
        case 43:
        {
            DFT_TxBuf.tData.tDft_43.tWctaFrame2_u16Avg = tWctaFrameTime[2].u16Avg;
            DFT_TxBuf.tData.tDft_43.tWctaFrame3_u16Avg = tWctaFrameTime[3].u16Avg;
            break;
        }
        case 44:
        {
            DFT_TxBuf.tData.tDft_44.tWctaFrame4_u16Avg = tWctaFrameTime[4].u16Avg;
            DFT_TxBuf.tData.tDft_44.tWctaFrame5_u16Avg = tWctaFrameTime[5].u16Avg;
            break;
        }
        case 45:
        {
            DFT_TxBuf.tData.tDft_45.tWctaFrame6_u16Avg = tWctaFrameTime[6].u16Avg;
            DFT_TxBuf.tData.tDft_45.tWctaFrame7_u16Avg = tWctaFrameTime[7].u16Avg;
            break;
        }
        case 46:
        {
            DFT_TxBuf.tData.tDft_46.tWctaFrameAll_u16Avg = tWctaFrameTime[8].u16Avg;
            DFT_TxBuf.tData.tDft_46.tWctaFrame0_u16Max = tWctaFrameTime[0].u16Max;
            break;
        }
        case 47:
        {
            DFT_TxBuf.tData.tDft_47.tWctaFrame1_u16Max = tWctaFrameTime[1].u16Max;
            DFT_TxBuf.tData.tDft_47.tWctaFrame2_u16Max = tWctaFrameTime[2].u16Max;
            break;
        }
        case 48:
        {
            DFT_TxBuf.tData.tDft_48.tWctaFrame3_u16Max = tWctaFrameTime[3].u16Max;
            DFT_TxBuf.tData.tDft_48.tWctaFrame4_u16Max = tWctaFrameTime[4].u16Max;
            break;
        }
        case 49:
        {
            DFT_TxBuf.tData.tDft_49.tWctaFrame5_u16Max = tWctaFrameTime[5].u16Max;
            DFT_TxBuf.tData.tDft_49.tWctaFrame6_u16Max = tWctaFrameTime[6].u16Max;
            break;
        }
        case 50:
        {
            DFT_TxBuf.tData.tDft_50.tWctaFrame7_u16Max = tWctaFrameTime[7].u16Max;
            DFT_TxBuf.tData.tDft_50.tWctaFrameAll_u16Max = tWctaFrameTime[8].u16Max;
            break;
        }
        case 51:
        {
            DFT_TxBuf.tData.tDft_51.G_address0 = 0;
            break;
        }
        case 52:
        {
            DFT_TxBuf.tData.tDft_52.G_address1 = 0;
            break;
        }
        case 53:
        {
            DFT_TxBuf.tData.tDft_53.f32_5V_SENSE = Adc_f32UnitValAvg.val.f32_5V_SENSE;
            break;
        }
        case 54:
        {
            DFT_TxBuf.tData.tDft_54.f32_28VBUS_VSENSE = VBUS28_VSENSE_UNITV_raw;
            break;
        }
        case 55:
        {
            DFT_TxBuf.tData.tDft_55.f32_POSITIVE_15V_SENSE = Adc_f32UnitValAvg.val.f32_POSITIVE_15V_SENSE;
            break;
        }
        case 56:
        {
            DFT_TxBuf.tData.tDft_56.f32_NEGATIVE_15V_SENSE = Adc_f32UnitValAvg.val.f32_NEGATIVE_15V_SENSE;
            break;
        }
        case 57:
        {
            DFT_TxBuf.tData.tDft_57.f32_DC_BUS_VSENSE = VBUS270_VSENSE_UNITV_raw;
            break;
        }
        case 58:
        {
            DFT_TxBuf.tData.tDft_58.f32_I_PHA_CHA = Adc_f32UnitValAvg.val.f32_I_PHA_CHA;
            break;
        }
        case 59:
        {
            DFT_TxBuf.tData.tDft_59.f32_I_PHB_CHA = Adc_f32UnitValAvg.val.f32_I_PHB_CHA;
            break;
        }
        case 60:
        {
            DFT_TxBuf.tData.tDft_60.f32_I_PHC_CHA = Adc_f32UnitValAvg.val.f32_I_PHC_CHA;
            break;
        }
        case 61:
        {
            DFT_TxBuf.tData.tDft_61.f32_MOTOR_TEMP_PHA = Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHA;
            break;
        }
        case 62:
        {
            DFT_TxBuf.tData.tDft_62.f32_MOTOR_TEMP_PHB = Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHB;
            break;
        }
        case 63:
        {
            DFT_TxBuf.tData.tDft_63.f32_MOTOR_TEMP_PHC = Adc_f32UnitValAvg.val.f32_MOTOR_TEMP_PHC;
            break;
        }
        case 64:
        {
            DFT_TxBuf.tData.tDft_64.f32_Peak_I = Adc_f32UnitValAvg.val.f32_MOTOR_PHASEx_I;
            break;
        }
        case 65:
        {
            DFT_TxBuf.tData.tDft_65.u16NVMfaultData = Nvm_State.faultData;
            DFT_TxBuf.tData.tDft_65.u16NVMfaultId = Nvm_State.faultId;
            DFT_TxBuf.tData.tDft_65.u16MSB_OUT = MSB_OUT;
            DFT_TxBuf.tData.tDft_65.bBRAKE_CNTL_CHA = BRAKE_CNTL_CHA;
            DFT_TxBuf.tData.tDft_65.bSHDN_BRK_CTRL_CHA = SHDN_BRK_CTRL_CHA;
            DFT_TxBuf.tData.tDft_65.bCHA_STATUS = CH_STATUS;
            DFT_TxBuf.tData.tDft_65.bNS_CNTL_PHA_CHA = NS_CNTL_PHA_CHA;
            DFT_TxBuf.tData.tDft_65.bNS_CNTL_PHC_CHA = NS_CNTL_PHC_CHA;
            DFT_TxBuf.tData.tDft_65.bBUS_SW_ENABLE = BUS_SW_ENABLE;
            DFT_TxBuf.tData.tDft_65.bVC_MODE_CHA = VC_MODE_CHA;
            DFT_TxBuf.tData.tDft_65.b270V_BUS_CNTL_CHA = v270_BUS_CNTL_CHA;
            DFT_TxBuf.tData.tDft_65.bGD_RESET = GD_RESET;
            DFT_TxBuf.tData.tDft_65.bENC_ZERO_SET_CNTL = ENC_ZERO_SET_CNTL;
            DFT_TxBuf.tData.tDft_65.bnCS_270VBUS_VSENSE = CS_270VBUS_VSENSE;
            DFT_TxBuf.tData.tDft_65.bnCS_28VBUS_VSENSE = CS_28VBUS_VSENSE;
            DFT_TxBuf.tData.tDft_65.bnCS_IBRK_SENSE = CS_IBRK_SENSE;
            DFT_TxBuf.tData.tDft_65.b28V_BUS_CNTL_CHA = v28_BUS_CNTL_CHA;
            DFT_TxBuf.tData.tDft_65.bINRUSH_CTR_CHA = INRUSH_CTR_CHA;
            break;
        }
        case 66:
        {
            DFT_TxBuf.tData.tDft_66.bPHA_HS_CNTL_CHA = PHA_HS_CNTL_CHA;
            DFT_TxBuf.tData.tDft_66.bPHA_LS_CNTL_CHA = PHA_LS_CNTL_CHA;
            DFT_TxBuf.tData.tDft_66.bPHB_HS_CNTL_CHA = PHB_HS_CNTL_CHA;
            DFT_TxBuf.tData.tDft_66.bPHB_LS_CNTL_CHA = PHB_LS_CNTL_CHA;
            DFT_TxBuf.tData.tDft_66.bPHC_HS_CNTL_CHA = PHC_HS_CNTL_CHA;
            DFT_TxBuf.tData.tDft_66.bPHC_LS_CNTL_CHA = PHC_LS_CNTL_CHA;
            DFT_TxBuf.tData.tDft_66.u16NVMrigstatus = Nvm_State.rigstatus;
            DFT_TxBuf.tData.tDft_66.bVcModeCmd = tPanel.bVcModeCmd;
            DFT_TxBuf.tData.tDft_66.bSensorFusionEnableCmd = tPanel.bSensorFusionEnableCmd;
            DFT_TxBuf.tData.tDft_66.CurrentBusSubState = CurrentBusSubState;
            DFT_TxBuf.tData.tDft_66.Reserved1 = 0;
#if defined(__SKEW_SNSR_ENCODER__)
            DFT_TxBuf.tData.tDft_66.u16IncoderPositionNotValidCntr = u16IncoderPositionNotValidCntr;
#endif
            break;
        }
        case 67:
        {
            DFT_TxBuf.tData.tDft_67.f32imbCurrent = G_f32imbal_index;
            break;
        }
        case 68:
        {
            DFT_TxBuf.tData.tDft_68.BitmonitorsIllegalHalls = BitmonitorsIllegalHalls;
            DFT_TxBuf.tData.tDft_68.BadTransitions = MotorCmd.BadTransitions;
            break;
        }
        case 69:
        {
#if defined(__SKEW_SNSR_ENCODER__)
            DFT_TxBuf.tData.tDft_69.u32IncoderPosition = tIncoderInputs.u32Position;
#endif
            break;
        }
        case 70:
        {
#if defined(__SKEW_SNSR_ENCODER__)
            DFT_TxBuf.tData.tDft_70.u16IncoderCrc8CheckFailedCntr = u16IncoderCrc8CheckFailedCntr;
            DFT_TxBuf.tData.tDft_70.u16IncoderRxNotReadyCntr = u16IncoderRxNotReadyCntr;
#endif
            break;
        }
        case 71:
        {
#if defined(__SKEW_SNSR_ENCODER__)
            DFT_TxBuf.tData.tDft_71.u32IncoderPositionOffset = Nvm_Rigging_Temp.tSkewSnsr.tEncoderCalibration.u32Offset;
#endif
            break;
        }
        case 72:
        {
            DFT_TxBuf.tData.tDft_72.u32OneMsTimer = OneMsTimer;
            break;
        }
        /* End:  Cannot have more than 80 32-bit words */
        default:
        {
            /* Invalid Index */
            bDone = true;
            break;
        }
    }

    /* Enable Interrupts */
    EINT;
    u16DFTIndex++;
//    /* If the last word of data is being sent out then reset the index to 0, otherwise increment */
//    if((bDone == true)&&(DFT_Period == 0))
//    {
//        u16DFTIndex = 0U;
//    }
//    else
//    {
//        /* increment the data index as there is more data to send */
//        u16DFTIndex++;
//    }

    return bDone;
}
