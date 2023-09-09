/***********************************************************************
    File name : icc.c

    Purpose: The icc.c file implements functionality related to sci driver,
             required to communicate between two MCUs over Inter Channel
             Communication (ICC).

    Author                  Date            CR#         Description
    Bhaveek Dhaigude        12/22/2022      NA          Initial Draft
*************************************************************************/

/***************************Included Files*******************************/
#include "icc.h"
#include "gpio.h"
#include "crc.h"
#include "adc.h"
#include "mcu.h"
#include "nvm.h"
//#include "bit.h"
#include "bitmonitors.h"

#define ICC_RX_BUF_SIZE         32      /* maximum size needed for the ICC interface serial receive buffer (unstuffed) */
#define ICC_MAX_FIFO_COUNT      8       /* ICC max FIFO set to 8 to hold 2 subframes worth of data only. Specifically designed to send 4 bytes + any DLE and ETX one subframe at a time.*/
#define ICC_MAX_PERIOD          8       /* ICC_MAX_PERIOD*0.125 = 8*0.125 = 1ms */

/* Definition of SCI register based on hardware */
#if defined(DRV8312_DEV_KIT)
    /* Development kit is on SCI_A */
    #define IccSciRegs  ScicRegs
#else
    /* MCU Hardware is on SCI_A */
    #define IccSciRegs  SciaRegs
#endif

/**************************Global Variables*******************************/
static uint16_t ICC_RxBuf[ICC_MSG_SIZE] = {0};
static tSciIccMsgBuf_t ICC_TxBuf = {0};
static uint16_t ICC_TxIndex = 0;
static uint16_t ICC_RxIndex = 0;
static uint16_t ICC_tempRxIndex = 0;
static uint16_t u16IccIndex = 0;
static bool_t ICC_error_flag = false;
tSciIccMsgBuf_t ICC_xData = {0};
uint16_t ICC_InvalidCRCcounter = 0;
uint16_t ICC_TimeOutCounter = 0;

/****************************************************************
 * Function: ICC_GpioInit
 * Purpose: Initialize the serial communication GPIO for ICC.
 ****************************************************************/
void ICC_GpioInit(void)
{
    /*GPIO initialization for SCI*/
     EALLOW;
     GpioCtrlRegs.GPBGMUX1.bit.GPIO43 = GPIO_MUX_GRP_3;          /*Select group 3 */
     GpioCtrlRegs.GPBMUX1.bit.GPIO43 = GPIO_MUX_TYPE_3;          /*Set as SCIRXDA pin type 3*/
     GpioCtrlRegs.GPBPUD.bit.GPIO43 = GPIO_ENABLE_PULLUP;        /*Enable the pull up*/
     GpioCtrlRegs.GPBDIR.bit.GPIO43 = GPIO_INPUT;                /*Set pin as a input pin*/
     GpioCtrlRegs.GPBQSEL1.bit.GPIO43 = GPIO_ASYNC;              /*Asynch input GPIO43 (SCIRXDA)*/

     GpioCtrlRegs.GPBGMUX1.bit.GPIO42 = GPIO_MUX_GRP_3;          /*Select group 3 */
     GpioCtrlRegs.GPBMUX1.bit.GPIO42 = GPIO_MUX_TYPE_3;          /*Set as SCITXDA pin type 3*/
     GpioCtrlRegs.GPBPUD.bit.GPIO42 = GPIO_ENABLE_PULLUP;        /*Enable the pull up*/
     GpioCtrlRegs.GPBDIR.bit.GPIO42 = GPIO_OUTPUT;               /*Set pin as a output pin*/
     GpioCtrlRegs.GPBQSEL1.bit.GPIO42 = GPIO_ASYNC;              /*Asynch input GPIO42 (SCITXDA)*/

     EDIS;
}

/****************************************************************
 * Function: Icc_Init
 * Purpose: Initialize the serial communication function for ICC.
 ****************************************************************/
void Icc_Init(void)
{
    IccSciRegs.SCICCR.bit.STOPBITS        = 0U;               /*One Stop Bit*/
    IccSciRegs.SCICCR.bit.PARITY          = 0U;               /*Odd Parity*/
    IccSciRegs.SCICCR.bit.PARITYENA       = 1U;               /*Enable Parity*/
    IccSciRegs.SCICCR.bit.SCICHAR         = SCIACHAR_VALUE;   /*SCI character Length is 8 bits*/
    IccSciRegs.SCICCR.bit.ADDRIDLE_MODE   = 0U;               /*Idle-Line mode protocol selected*/

    IccSciRegs.SCIHBAUD.all       = SCIAHBAUD_VALUE;
    IccSciRegs.SCILBAUD.all       = SCIALBAUD_VALUE;

    IccSciRegs.SCICTL1.bit.SWRESET        = 0U;       /*Initialize SCI state machine & operating flags to the reset condition*/

    IccSciRegs.SCIFFTX.bit.SCIRST         = 1U;       /*Enable Transmit and Receive operation for FIFO*/
    IccSciRegs.SCIFFTX.bit.SCIFFENA       = 1U;       /*Enable FIFO enhancement*/

    IccSciRegs.SCIFFTX.bit.TXFFIENA       = 1U;       /*Disable Transmit FIFO interrupt enable*/
    IccSciRegs.SCIFFTX.bit.TXFFIL         = 0U;       /*Set the interrupt Trigger to 0 word*/

    IccSciRegs.SCIFFRX.bit.RXFFIENA       = 1U;       /*Enable receive FIFO interrupt*/
    IccSciRegs.SCIFFRX.bit.RXFFIL         = 1U;       /*Set the interrupt trigger to 1 word*/

    IccSciRegs.SCICTL1.bit.TXENA          = 1U;       /* Enable SCI Transmitter */
    IccSciRegs.SCICTL1.bit.RXENA          = 1U;       /* Enable SCI Receiver */

    IccSciRegs.SCICTL2.bit.TXINTENA       = 0U;       /*Disable TXRDY interrupt*/
    IccSciRegs.SCICTL2.bit.RXBKINTENA     = 0U;       /* Enable Receiver Buffer / Break Interrupt */

    IccSciRegs.SCICTL1.bit.SWRESET        = 1U;        /*Re-enabling the SCI from reset*/

    IccSciRegs.SCIFFTX.bit.TXFIFORESET    = 1U;       /*Re-Enable Transmit FIFO operation*/
    IccSciRegs.SCIFFRX.bit.RXFIFORESET    = 1U;       /*Re-Enable receive FIFO operation*/
    IccSciRegs.SCIFFRX.bit.RXFFINTCLR     = 1U;       /*Clear RXFFINT flag*/
    IccSciRegs.SCIFFTX.bit.TXFFINTCLR     = 1U;       /*Clear TXFFINT flag*/

    IccSciRegs.SCIFFCT.all                = 0x0;      /* Disable Auto Baud detection*/

    /* shall initialize the ICC message timeout timer */
    EINT;
}

/*************************************************************************************************************
Function: Icc_UpdateMsg

Purpose: Update ICC frame to transmit.

*************************************************************************************************************/

void Icc_UpdateMsg (void)
{
    bool_t bDone = false;

    /* Disable Interrupts to ensure integrity of data */
    DINT;

    switch (u16IccIndex)
    {

    /* As a place holder dummy hard coded values assigned for ICC-Frame fields.
     * To be updated as and when respective variables are created.*/

        case 0:
        {   /* DWord 0 */
            ICC_TxBuf.tData.tIcc_0.BeginDLE = DLE_CHAR;

            if(G_bChannelA)
            {
                ICC_TxBuf.tData.tIcc_0.CHStatus.bit.CHID = True;
            }
            else
            {
                ICC_TxBuf.tData.tIcc_0.CHStatus.bit.CHID = False;
            }

            ICC_TxBuf.tData.tIcc_0.CHStatus.bit.FLA_fail = bFlaFail;
            ICC_TxBuf.tData.tIcc_0.CHStatus.bit.FPSU_fail = bFpsuFail;
            ICC_TxBuf.tData.tIcc_0.CHStatus.bit.motor_fail = bMotorFail;
            ICC_TxBuf.tData.tIcc_0.CHStatus.bit.flap_jam = bFlapJam;
            ICC_TxBuf.tData.tIcc_0.CHStatus.bit.rig_status = (Uint16)Nvm_State.rigstatus;
            ICC_TxBuf.tData.tIcc_0.CHStatus.bit.faultStatus = (Uint16)OnSideFault;
            break;
        }
        case 1:
        {   /* DWord 1 */
            ICC_TxBuf.tData.tIcc_1.f32_MOTOR_PHASEx_I = Adc_f32UnitValAvg.val.f32_MOTOR_PHASEx_I;
            break;
        }
        case 2:
        {   /* DWord 2 */
#if defined (__SKEW_SNSR_ENCODER__)
            ICC_TxBuf.tData.tIcc_2.u32IncoderInputsRaw = tIncoderInputs.tRaw.u32Data;
#endif
            break;
        }
        case 3:
        {   /* DWord 3 */
#if defined (__SKEW_SNSR_ENCODER__)
            ICC_TxBuf.tData.tIcc_3.u16IncoderNewData = Encoder_GetEncoderNewDataFlag();
            ICC_TxBuf.tData.tIcc_3.bAvailable = 0U;
#endif
            break;
        }
        case 4:
        {
            ICC_TxBuf.tData.tIcc_4.FaultCode = ReportedFaultCode;
            ICC_TxBuf.tData.tIcc_4.FaultData = ReportedFaultData;
            break;
        }
        case 5:
        {
            ICC_TxBuf.tData.tIcc_5.rvdt_pos_in = Adc_Averaged.val.u16_RVDT_POS;
            ICC_TxBuf.tData.tIcc_5.crc = CRC16Api_GetCRC((const uint16_t *)&ICC_TxBuf, ICC_MAX_SIZE);
            break;
        }
        case 6:
        {
            // reserved
            break;
        }
        case 7:
        {
            // reserved
            break;
        }
        default:
        {
            /* Invalid Index */
            bDone = true;
            break;
        }
    }
    /* Enable Interrupts */
    EINT;

    /* If the last word of data is being sent out then reset the index to 0, otherwise increment */
    if(bDone == true)
    {
        u16IccIndex = 0U;
    }
    else
    {
        /* increment the data index as there is more data to send */
        u16IccIndex++;
    }

  if (ICC_error_flag == true) /*Reinitialize the SCI state machine*/
  {
      Icc_Init();
      ICC_error_flag = false;
  }

}

/*************************************************************************************************************
Function: Icc_TxSrvc

Purpose: Transmit service routine for ICC

*************************************************************************************************************/
void Icc_TxSrvc(void)
{
    // ICC_MAX_PERIOD*0.125 = 8*0.125 = 1ms
    if(u16IccIndex >= ICC_MAX_PERIOD)
    {
        ICC_TxIndex = 0;
        u16IccIndex = 0;
    }
    Icc_TxMsg(ICC_MSG_SIZE);
}

/*************************************************************************************************************
Function: Icc_TxMsg

Purpose: Transmit data on ICC bus

*************************************************************************************************************/
void Icc_TxMsg (Uint16 length)
{
    SCI_STATUS status = TX_IN_PROGRESS;
    uint16_t u16SubIndex = 0U;  /* used to send in 32-bit chunks */
    t32Data_t t32Data = {0};
    uint16_t u16CharToSend = 0U;
    uint16_t i = 0;
    Icc_UpdateMsg();
    if ((ICC_TxIndex < length) && (IccSciRegs.SCIFFTX.bit.TXFFST <= ICC_MAX_FIFO_COUNT))
    {
        for (i = 0U; i < 4; i++)
        {
            u16SubIndex = (ICC_TxIndex / 4U); /* Get the sub-index we are on to send the correct 32-bit word */
            t32Data.u32Data = ICC_TxBuf.t32[u16SubIndex].u32Data;
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
            IccSciRegs.SCITXBUF.all = u16CharToSend;

            ICC_TxIndex++;
        }
    }

    /*shall send the closing data link escape character once all of the message data words
        have been transmitted. */
    if ((ICC_TxIndex == length) && (IccSciRegs.SCIFFTX.bit.TXFFST < MAX_FIFO_COUNT))
    {
        /* PATH(ICC_TxMsg,H); */
        IccSciRegs.SCITXBUF.all = DLE_CHAR;
        ICC_TxIndex++;
    }

    /*shall send the end of transmission character following the closing data link escape character */
    if ((ICC_TxIndex == (length + 1)) && (IccSciRegs.SCIFFTX.bit.TXFFST < MAX_FIFO_COUNT))
    {
        /* PATH(Icc_TxMsg,I); */
        IccSciRegs.SCITXBUF.all = EOTX_CHAR;
        ICC_TxIndex++;
        status = TX_DONE;
    }
    /* PATH(Icc_TxMsg,J); */

}

/*************************************************************************************************\
* Function: Icc_RxMsg
*
* Purpose:  Function to receive a ICC message off the ICC Bus
*
\*************************************************************************************************/
SCI_STATUS Icc_RxMsg(void)
{
    Uint16 x, rxwords;
    tSciIccMsgBuf_t *selectMsg;
    SCI_STATUS status = RX_IN_PROGRESS;
    Uint16 temp = 0;
    static Uint16 prevtemp = 0;
    static SCI_STATUS flag = INVALID_RX_MSG;
    uint16_t crc = 0;
    /*shall check that the receive FIFO byte count is greater than 0 for SCI-A*/

    if (IccSciRegs.SCIFFRX.bit.RXFFST > 0)
    {
        /* PATH(ICC_RxMsg,B); */
        /*shall read the FIFO byte count */

        rxwords = SciaRegs.SCIFFRX.bit.RXFFST;
        
        ICC_TimeOutCounter = 0;                 /*monitor 0x07 ICC timeout*/

        for (x = 0; x < rxwords; x++)
        {
            /* PATH(ICC_RxMsg,C); */
            /*shall transfer a number of bytes from the FIFO to the global Rx buffer
              equal to the byte count read */

            temp = SciaRegs.SCIRXBUF.all;

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
                if((ICC_tempRxIndex % 2) == 0)
                {
                    ICC_RxBuf[ICC_RxIndex] = temp;
                }
                else
                {
                    ICC_RxBuf[ICC_RxIndex] = (temp << 8) | ICC_RxBuf[ICC_RxIndex];
                    ICC_RxIndex++;
                }


                if ((status == VALID_RX_MSG) || (ICC_RxIndex == (ICC_RX_BUF_SIZE - 1)))
                {
                     /* PATH(ICC_RxMsg,F); */
                    ICC_RxIndex = 0;
                    ICC_tempRxIndex = 0;
                }
                else
                {
                    /* PATH(ICC_RxMsg,G); */
                    ICC_tempRxIndex++;
                }
            }
            else if(flag == VALID_RX_MSG)
            {
                ICC_RxBuf[ICC_RxIndex] = temp;
                status = VALID_RX_MSG;
                /*shall typecast the buffer to the select message type */
                selectMsg = (tSciIccMsgBuf_t *)ICC_RxBuf;

                crc = CRC16Api_GetCRC((const uint16_t *)selectMsg, ICC_MAX_SIZE);

                /* does the checksum indicate valid received data? */
                if (crc == selectMsg->tData.tIcc_5.crc)
                {
                    ICC_xData = *selectMsg;

                    if (ICC_InvalidCRCcounter != 0)
                    {
                        ICC_InvalidCRCcounter--; /*CRC matched, decrement the counter*/
                    }
                }
                else
                {
                    if (ICC_InvalidCRCcounter < ICC_INVALID_CRC_COUNTER)
                    {
                        ICC_InvalidCRCcounter++; /*CRC mismatched, increment the counter*/
                    }
                }
                flag = INVALID_RX_MSG;
                x = rxwords;        /* set x equal to rxwords to break out of the loop */
                ICC_RxIndex = 0;
                ICC_tempRxIndex= 0; /*CRC error was observed before frequently */
            }
        }
    }

 /* Observed RXERROR bit set of SCIRXT register and it is due to either PE or FE or BRKDT error
  * BRKDT- A break detect condition occurs (the SCIRXD is low for ten bit periods following a missing stop bit).
    This action sets the BRKDT flag bit (SCIRXST, bit 5) and initiates an interrupt.
    Clear SCICTL1.SWRESET bit after a receiver break detect (BRKDT flag, bit SCIRXST, bit 5).

    FE- The SCI sets this bit when an expected stop bit is not found. Only
    the first stop bit is checked. The missing stop bit indicates that
    synchronization with the start bit has been lost and that the character
    is incorrectly framed. The FE bit is reset by a clearing of the SW
    RESET bit or by a system reset.

    PE- sets when parity mismatch
    Ref - TRM 19.14.2.2
    */
    else if ((IccSciRegs.SCIRXST.bit.FE == 1U) ||
             (IccSciRegs.SCIRXST.bit.PE == 1U) ||
             (IccSciRegs.SCIRXST.bit.BRKDT == 1U))
    {
        ICC_error_flag = true;
    }

    /* PATH(ICC_RxMsg,H); */
    return status;
}

void Icc_SetEncoderNewData(uint16_t u16NewData)
{
    ICC_TxBuf.tData.tIcc_3.u16IncoderNewData = u16NewData;
}

