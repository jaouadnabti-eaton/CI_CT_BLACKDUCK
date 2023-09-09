/***************************************************************************************************
 * File name: A825Drvr.c
 *
 * Purpose : Low-Level Interface to CAN peripheral on TMS320F28379D Microcontroller
 *
 * Copyright Notice:
 * All source code and data contained in this file is Proprietary and
 * Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
 * disclosed; in whole or in part, without the express written permission of
 * Eaton Aerospace.
 *
 * Copyright (c) 2020 - Eaton Aerospace Group, All Rights Reserved.
 *
 * Author            Date       CR#              Description
 * ------          ---------    ------        -------------------------------------
 * Adam Bouwens     12/07/2022  NA             Port to MCU
 ****************************************************************************************************/
#include "parameter.h"
#include "Defines.h"
//#include "F28x_Project.h"
#include "hw_memmap.h"
//#include "Assembly.h"
//#include "PieVect.h"
//#include "PieCtrl.h"
//#include "Cpu.h"
//#include "Gpio.h"
#include "A825Drvr.h"
//#include "sys_common.h"
//#include "device.h"
//#include "F2837xD_device.h"
//#include "stdbool.h"
//#include "Utils.h"
#define NUMCANMBOXES             32
#define U32ZERO                  (0x00000000UL)
#define U16ZERO                  (0U)
#define U32ONES                  (0xFFFFFFFFUL)
#define CLEAR_ACK                (0xFFFFFFFFUL)
#define LOOP_ESCAPE_COUNTDOWN    (20000000UL)
#define TXCOUNT  100
#define MSG_DATA_LENGTH    8
#define TX_MSG_OBJ_ID    32
#define RX_MSG_OBJ_ID    1

/* IFxCMD Configuration Bits for Receive Mailbox Transfer Setup Definition */
/* IFxCMD.bit.Control   = 1U: Transfer message control bits from object to IFx */
/* IFxCMD.bit.Mask      = 1U: Transfer the Mask from object to IFx */
/* IFxCMD.bit.Arb       = 1U: Transfer Arbitration bits from object to IFx */
/* IFxCMD.bit.TxRqst    = 1U: Reset NewDat bit in the message object when data transferred */
/* IFxCMD.bit.DATA_A    = 1U: Transfer DATA_A to IFx */
/* IFxCMD.bit.DATA_B    = 1U: Transfer DATA_B to IFx */
#define RX_MBOX_IFCMD_CONFIG    (0x00770000U)

/* Set Baud rate for CAN Channels */
#define CAN_BAUD_500K   500             /* Set CAN-A and CAN-B to 500kbps */
//#define CANA_BAUD       CAN_BAUD_500K   /* Set CAN-A for 500kbps */
//#define CANB_BAUD       CAN_BAUD_500K   /* Set CAN-B for 500kbps */

//static volatile MBOX *g_ptr_SendMailBox;

typedef enum
{
    READ_ONLY,
    READ_WRITE
} ACCESS_STATUS;

typedef struct
{
    CANMBOXESNUMBERS tMailboxNumber;
    uint32_t    u32Mask;
    uint32_t    u32Arb;
} MAILBOX_CONFIGURATION_T;

/*
 * Function prototypes.
 */
ERRORCODES A825Drvr_SetupMailboxes(volatile struct CAN_REGS* const ptCanRegs, const A825_BUS_TYPE_T eBusType);
ERRORCODES A825Drvr_GetCANMessage(volatile struct CAN_REGS* const ptCanRegs, uint32_t objID);


A825Drvr_Data_t tA825Drvr_Data[NUM_CAN_CHANNELS] =
{
 { &CanaRegs, { 0 }, { 0 } },
 { &CanbRegs, { 0 }, { 0 } }
};

static const MAILBOX_CONFIGURATION_T G_CAN_Mailbox_Config[NUMCANMBOXES] =
{
 {MBOX01, CANMBOX01MASK, CANMBOX01ARB}, {MBOX02, CANMBOX02MASK, CANMBOX02ARB}, {MBOX03, CANMBOX03MASK, CANMBOX03ARB}, {MBOX04, CANMBOX04MASK, CANMBOX04ARB},
 {MBOX05, CANMBOX05MASK, CANMBOX05ARB}, {MBOX06, CANMBOX06MASK, CANMBOX06ARB}, {MBOX07, CANMBOX07MASK, CANMBOX07ARB}, {MBOX08, CANMBOX08MASK, CANMBOX08ARB},
 {MBOX09, CANMBOX09MASK, CANMBOX09ARB}, {MBOX10, CANMBOX10MASK, CANMBOX10ARB}, {MBOX11, CANMBOX11MASK, CANMBOX11ARB}, {MBOX12, CANMBOX12MASK, CANMBOX12ARB},
 {MBOX13, CANMBOX13MASK, CANMBOX13ARB}, {MBOX14, CANMBOX14MASK, CANMBOX14ARB}, {MBOX15, CANMBOX15MASK, CANMBOX15ARB}, {MBOX16, CANMBOX16MASK, CANMBOX16ARB},
 {MBOX17, CANMBOX17MASK, CANMBOX17ARB}, {MBOX18, CANMBOX18MASK, CANMBOX18ARB}, {MBOX19, CANMBOX19MASK, CANMBOX19ARB}, {MBOX20, CANMBOX20MASK, CANMBOX20ARB},
 {MBOX21, CANMBOX21MASK, CANMBOX21ARB}, {MBOX22, CANMBOX22MASK, CANMBOX22ARB}, {MBOX23, CANMBOX23MASK, CANMBOX23ARB}, {MBOX24, CANMBOX24MASK, CANMBOX24ARB},
 {MBOX25, CANMBOX25MASK, CANMBOX25ARB}, {MBOX26, CANMBOX26MASK, CANMBOX26ARB}, {MBOX27, CANMBOX27MASK, CANMBOX27ARB}, {MBOX28, CANMBOX28MASK, CANMBOX28ARB},
 {MBOX29, CANMBOX29MASK, CANMBOX29ARB}, {MBOX30, CANMBOX30MASK, CANMBOX30ARB}, {MBOX31, CANMBOX31MASK, CANMBOX31ARB}, {MBOX32, CANMBOX32MASK, CANMBOX32ARB}
};

/***************************************************************************************************
 * Function: A825Drvr_SetupMailboxes
 * Purpose: Setup all 32 Mailboxes (objects) in RAM used for Transmitting and Receiving CAN frames
 * Global Inputs(s):
 *
 * Global Outputs(s):
 *
 * Inputs:
 * Output:
 ****************************************************************************************************/
ERRORCODES A825Drvr_SetupMailboxes(volatile struct CAN_REGS* const ptCanRegs, const A825_BUS_TYPE_T eBusType)
{
    ERRORCODES retval = FAILURE;
    uint16_t u16MailboxIndex = 0U;
    uint16_t u16ExitCounter = A825_WAIT_COUNTER;
    union CAN_IF1CMD_REG CAN_IF1CMD_SHADOW; /* IF1CMD needs to be written in single 32-bit write */
    MSGID_REG_MGR tMsgId;   /* Temp Message ID used for initializing the Destination LRU ID */

    if(ptCanRegs != NULL)
    {
        tMsgId.all = 0UL; /* Initialize */

        /* Configure all Mailboxes */
        for (u16MailboxIndex = 0U; u16MailboxIndex < NUMCANMBOXES; u16MailboxIndex++)
        {
            /* Wait for busy bit to clear. */
            while((ptCanRegs->CAN_IF1CMD.bit.Busy) && (u16ExitCounter != 0U))
            {
                u16ExitCounter = u16ExitCounter - 1U; /* Protect from endless loop */
            }
            if(u16ExitCounter == 0U)
            {
                retval = TIMEOUT;
                break;
            }
            else
            {
                /* Clear registers and init shadow buffer to program the message object. */
                ptCanRegs->CAN_IF1MCTL.all = 0U;
                ptCanRegs->CAN_IF1MSK.all = 0U;
                ptCanRegs->CAN_IF1ARB.all = 0U;
                CAN_IF1CMD_SHADOW.all = 0U;

                /* CONFIGURE MASK CONTROL (MCTL) REGISTER */
                /* Setup to use Masks (MUST SET MsgVal to 1 AFTER UMask is set if using masks */
                ptCanRegs->CAN_IF1MCTL.bit.UMask = 1U;
                /* Set the Data Length Code to 8 for all mailboxes. */
                ptCanRegs->CAN_IF1MCTL.bit.DLC = 8U;
                /* Setup for single transfer. FIFO function is disabled. */
                ptCanRegs->CAN_IF1MCTL.bit.EoB = 1U;

                /* CONFIGURE MASK (MSK) REGISTER */
                /* Set the MASK ID used for Filtering */
                ptCanRegs->CAN_IF1MSK.all = G_CAN_Mailbox_Config[u16MailboxIndex].u32Mask;

                /* CONFIGURE ARBITRATION (ARB) REGISTER */
                /* Enable/Disable the Mailbox, set ID type, Direction, and ID used for filtering RX/TX */
                tMsgId.all = G_CAN_Mailbox_Config[u16MailboxIndex].u32Arb;

                /* Adjust Mailbox to receive only the messages in the LCC associated with this bus (CNTRL/MAINT busses) */
                if(eBusType == A825_CONTROL_BUS)
                {
                    /* Set SRCID to filter on SCU LRU ID */
                    tMsgId.a825_o2m.SRCID = SCU_LRU;
                    /* Set LCC to filter for NOC messages */
                    tMsgId.a825_o2m.LCC = LCC_NOC;
                }
                else if(eBusType == A825_MAINTENANCE_BUS)
                {
                    /* Set SRCID to filter on GSE ID */
                    tMsgId.a825_o2m.SRCID = GSE_ID;
                    /* Set LCC to filter for TMC messages */
                    tMsgId.a825_o2m.LCC = LCC_TMC;
                }

                /* Adjust Mailbox to receive only the messages destined for this MCU Type (DSTID) */
                tMsgId.a825_o2m.DSTID = G_eLruId; /* Set the Destination LRU ID as the configured MCU type */
                /* Adjust Mailbox to receive only the messages destined for this MCU CHANNEL A/B (CHANID) */
                tMsgId.a825_o2m.RCI = G_eChId; /* Set the Channel ID as the configured Channel Identifier (A/B) */
                /* ACTID is masked off by the MASK but set it here for the filter in case the masking ever changes */
                /* Adjust Mailbox to receive only the messages destined for this MCU ACTUATOR (ACTID) */
                tMsgId.a825_o2m.ACTID = G_eActId; /* Set the Actuator ID as the configured Actuator Identifier (L/R) */

                /* Save the ARBITRATION (ARB) settings to the register. */
                ptCanRegs->CAN_IF1ARB.all = tMsgId.all;

                /* CONFIGURE COMMAND (CMD) REGISTER */
                /* Set the Control, Mask, and Arb bit so that they get transferred to the Message object. */
                CAN_IF1CMD_SHADOW.bit.Control = 1U;
                CAN_IF1CMD_SHADOW.bit.Arb = 1U;
                CAN_IF1CMD_SHADOW.bit.Mask = 1U;
                CAN_IF1CMD_SHADOW.bit.DIR = 1U;

                /* Set the MAILBOX number in RAM to transfer the data to */
                CAN_IF1CMD_SHADOW.bit.MSG_NUM = G_CAN_Mailbox_Config[u16MailboxIndex].tMailboxNumber;

                /* Write the IF1CMD register. Initiates transfer of data from IF1 to the RAM object */
                ptCanRegs->CAN_IF1CMD.all = CAN_IF1CMD_SHADOW.all;

                retval = STAT_SUCCESS;
            }
        }
    }
    else
    {
        /* Null Pointer Error */
        retval = NULLPTR;
    }

    return retval;
}

/***************************************************************************************************
 * Function: A825Drvr_GetCANMessage
 * Purpose: Check for new Messages in the Receive Mailboxes.
 * Global Inputs(s):
 *
 * Global Outputs(s): None
 *
 * Inputs: Mailbox ID to check if new data
 * Output: True = New Message received at mailbox, False, No new message
 ****************************************************************************************************/
ERRORCODES A825Drvr_GetCANMessage(volatile struct CAN_REGS* const ptCanRegs, uint32_t objID)
{
    ERRORCODES retval = FAILURE;
    union CAN_IF2CMD_REG CAN_IF2CMD_SHADOW;
    uint16_t u16ExitCounter = A825_WAIT_COUNTER;

    if(ptCanRegs != NULL)
    {
        /* Message Data A, Data B, and control values to be transferred to IF2 Registers on request */
        CAN_IF2CMD_SHADOW.all = RX_MBOX_IFCMD_CONFIG; /* IF2CMD Register to Receive a Mailbox */
        CAN_IF2CMD_SHADOW.bit.MSG_NUM = objID; /* Setup the Mailbox to Read from */

        /* Transfer the Receive message object Contents to the IF register. */
        ptCanRegs->CAN_IF2CMD.all = CAN_IF2CMD_SHADOW.all;

        /* Wait for busy bit to clear. */
        while((ptCanRegs->CAN_IF2CMD.bit.Busy) && (u16ExitCounter != 0U))
        {
            u16ExitCounter = u16ExitCounter - 1U; /* Protect from endless loop */
        }
        if(u16ExitCounter == 0U)
        {
            retval = TIMEOUT;
        }
        else
        {
            /* Check if the mailbox has a new message */
            if(ptCanRegs->CAN_IF2MCTL.bit.NewDat == 1U)
            {
                retval = STAT_SUCCESS;
            }
            else
            {
                retval = RX_NO_NEW_MESSAGE;
            }
        }
    }

    return(retval);
}


/***************************************************************************************************
 * Function: A825Drvr_Init
 * Purpose: Initialize CAN for A825
 * Global Inputs(s):
 *      CAN_FilterValue[]                       Array of CAN acceptance filters
 *      CAN_MaskValue[]                         Array of CAN Local-Acceptance Masks
 *
 * Global Outputs(s): None
 *
 * Inputs: None
 * Output: None
 ****************************************************************************************************/
ERRORCODES A825Drvr_Init(A825Drvr_Data_t* const ptDrvrData, const A825_BUS_TYPE_T eBusType)
{   /* ARINC 825 Driver Initialization Function */
    ERRORCODES retval = FAILURE;
    bool_t bValidCanChnl = false;
    union CAN_BTR_REG tCAN_BTR_REG = { 0 };
    union CAN_CTL_REG tCAN_CTL_REG = { 0 };

    if(ptDrvrData != NULL)
    {
        if(ptDrvrData->ptCanRegs == &CanaRegs)
        {
            EALLOW; /* This is needed to write to EALLOW protected registers*/

            #if defined(DRV8312_DEV_KIT)
            /* 2837xD Development Kit Hardware */

            /* Setup GPIO used for CAN A peripheral use */
            GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = GPIO_MUX_GRP_0;     /* Select group 0 */
            GpioCtrlRegs.GPAPUD.bit.GPIO31 = GPIO_ENABLE_PULLUP;   /* Enable pullup on GPIO31 */
            GpioCtrlRegs.GPAMUX2.bit.GPIO31 = GPIO_MUX_TYPE_1;     /* GPIO31 = CANTXA */

            GpioCtrlRegs.GPAGMUX2.bit.GPIO30 = GPIO_MUX_GRP_0;     /* Select group 0 */
            GpioCtrlRegs.GPAPUD.bit.GPIO30 = GPIO_ENABLE_PULLUP;   /* Enable pullup on GPIO30 */
            GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = GPIO_ASYNC;         /* Asynch input */
            GpioCtrlRegs.GPAMUX2.bit.GPIO30 = GPIO_MUX_TYPE_1;     /* GPIO30 = CANRXA */

            #else
            /* MCU HW */

            /* Setup GPIO used for CAN A peripheral use */
            GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = GPIO_MUX_GRP_0;     /* Select group 0 */
            GpioCtrlRegs.GPAPUD.bit.GPIO19 = GPIO_ENABLE_PULLUP;   /* Enable pullup on GPIO19 */
            GpioCtrlRegs.GPAMUX2.bit.GPIO19 = GPIO_MUX_TYPE_3;     /* GPIO19 = CANTXA */

            GpioCtrlRegs.GPAGMUX2.bit.GPIO18 = GPIO_MUX_GRP_0;     /* Select group 0 */
            GpioCtrlRegs.GPAPUD.bit.GPIO18 = GPIO_ENABLE_PULLUP;   /* Enable pullup on GPIO18 */
            GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = GPIO_ASYNC;         /* Asynch input */
            GpioCtrlRegs.GPAMUX2.bit.GPIO18 = GPIO_MUX_TYPE_3;     /* GPIO18 = CANRXA */
            #endif
            bValidCanChnl = true;
        }
        else if(ptDrvrData->ptCanRegs == &CanbRegs)
        {
            EALLOW; /* This is needed to write to EALLOW protected registers*/

            /* MCU HW */

            /* Setup GPIO used for CAN A peripheral use */
            GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = GPIO_MUX_GRP_0;     /* Select group 0 */
            GpioCtrlRegs.GPAPUD.bit.GPIO12 = GPIO_ENABLE_PULLUP;   /* Enable pullup on GPIO20 */
            GpioCtrlRegs.GPAMUX1.bit.GPIO12 = GPIO_MUX_TYPE_2;     /* GPIO20 = CANTXB */

            GpioCtrlRegs.GPAGMUX1.bit.GPIO13 = GPIO_MUX_GRP_0;     /* Select group 0 */
            GpioCtrlRegs.GPAPUD.bit.GPIO13 = GPIO_ENABLE_PULLUP;   /* Enable pullup on GPIO21 */
            GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = GPIO_ASYNC;         /* Asynch input */
            GpioCtrlRegs.GPAMUX1.bit.GPIO13 = GPIO_MUX_TYPE_2;     /* GPIO18 = CANRXB */

            bValidCanChnl = true;
        }
        else
        {
            /* Error */
            retval = INVALID_CAN_CHNL;
        }

        /* Continue setting up the driver for the CAN channel if the channel is valid */
        if(bValidCanChnl == true)
        {
            /* Place CAN controller in init state. This will allow the message object RAM to be programmed. */
            ptDrvrData->ptCanRegs->CAN_CTL.bit.Init = 1U;
            ptDrvrData->ptCanRegs->CAN_CTL.bit.SWR = 1U;

            /* Setup CAN to be clocked off the SYSCLKOUT */
            ClkCfgRegs.CLKSRCCTL2.bit.CANABCLKSEL = 0;

            /* Save off CAN_CTL register to update for Bit Timing values */
            tCAN_CTL_REG.all = ptDrvrData->ptCanRegs->CAN_CTL.all;
            ptDrvrData->ptCanRegs->CAN_CTL.bit.CCE = 1U;   /* Set Configuration Change Bit to 1 to enable changing Bit Timing Settings */

            /* Setup the Baud Rate and Bit Timing:  Baud Rate Prescaler, TSeg1, TSeg2, SJW */
            /* Set to 500kBAUD:  400 cycles in 1 bit time at 500kBAUD:  2000/500 = 400
             * To set to 16Tq in a bit time set BRP to 25:  400/25 = 16Tq*/
            tCAN_BTR_REG.bit.BRPE = 0;      /* Set Baud Rate Prescaler Extension to 0. */
            tCAN_BTR_REG.bit.BRP = 24;      /* Set Baud Rate Prescaler to 25 */
            /* Total of 16 Tq for a bit time:  1 (Sync Segment) + 11 (Propagation Delay and Phase Segment 1) + 4 (Phase Segment 2) = 16 Tq */
            /* A825 Requirements:
             * - Sync Segment shall be 1Tq
             * - Sampling point shall be >= 75% Bit Time
             * - SJW shall be less than either Phase Segment 1 or Phase Segment 2
             * - SJW shall be between 1Tq and 4Tq
             */
            /* Sampling Point Calculation:
             * Sampling Point is at end of Phase Segment 1. This is at position 1 + 11 Tq = 12Tq
             * Total Tq in a bit time is 16Tq
             * => Sample Point = 12Tq / 16Tq = 75% */
            tCAN_BTR_REG.bit.TSEG1 = 10;    /* Set Propagation Delay plus Phase Segment 1 to 11 Tq */
            tCAN_BTR_REG.bit.TSEG2 = 3;     /* Set Phase Segment 2 to 4 Tq */
            tCAN_BTR_REG.bit.SJW = 3;       /* Set Synchronization Jump Width to 4Tq */

            /* Save off the BTR settings to the actual register */
            ptDrvrData->ptCanRegs->CAN_BTR.all = tCAN_BTR_REG.all;
            //ptCanRegs->CAN_BTR.all = 5735;

            /* Restore the saved CAN Control register. */
            ptDrvrData->ptCanRegs->CAN_CTL.all = tCAN_CTL_REG.all;

            /* Initialize the mailbox objects used for CAN Tx and Rx */
            retval = A825Drvr_SetupMailboxes(ptDrvrData->ptCanRegs, eBusType);

            /* Done initializing, enable CAN for operation. */
            ptDrvrData->ptCanRegs->CAN_CTL.bit.Init = 0;

            EDIS;   /* This is needed to disable write to EALLOW protected registers */
        }
    }
    else
    {
        /* Null Pointer Error */
        retval = NULLPTR;
    }

    return retval;
} /* ARINC 825 Driver Initialization Function */


/***************************************************************************************************
 * Function: A825Drvr_ReceiveRxDataService
 * Purpose: Process A825 receive message FIFO
 * Global Inputs(s):
 *      A825Drvr_DataRxFifo.fifo_avl            Number of available receive FIFO entries
 *      A825Drvr_DataRxFifo.fifo_in             Marks bottom of A825 FIFO
 * Global Outputs(s):
 *      A825Drvr_DataRxFifo.MSG[]               A825 receive message FIFO
 *      A825Drvr_DataRxFifo.fifo_avl            Number of available receive FIFO entries
 *      A825Drvr_DataRxFifo.fifo_in             Marks bottom of A825 FIFO
 *      A825Drvr_DataRxFifo.fifo_overflow       Indicates A825 FIFO overflow
 * Inputs: None
 * Output: None
 ****************************************************************************************************/
void A825Drvr_ReceiveRxDataService(A825Drvr_Data_t* const ptDrvrData)
{   /* ARINC 825 Driver Receive Data Service Function */
    ERRORCODES retval = FAILURE;
    uint16_t mailbox_number = 1U;

    if (ptDrvrData != NULL)
    {
        /* Loop through each CAN receive mailbox to check if a message is received */
        for(mailbox_number = 1U; mailbox_number <= A825_RX_NUMBER_OF_MAILBOXES; mailbox_number++)
        {
            retval = A825Drvr_GetCANMessage(ptDrvrData->ptCanRegs, mailbox_number);
            if (retval == STAT_SUCCESS)
            {
                /* Mailbox has received a message */
                if (ptDrvrData->tRx.tFifo.fifo_avl >= MAX_A825DRVR_RCV_FIFO_SIZE)
                {   /* Somehow the fifo count has exceeded allowed maximum */
                    ptDrvrData->tRx.tFifo.fifo_overflow = true;
                }   /* Somehow the fifo count has exceeded allowed maximum */
                else
                {   /* Normal Fifo operation */
                    /* defensive code for tFifo.MSG array */
                    if(ptDrvrData->tRx.tFifo.fifo_in < MAX_A825DRVR_RCV_FIFO_SIZE)
                    {
                        ptDrvrData->tRx.tFifo.MSG[ptDrvrData->tRx.tFifo.fifo_in].MSGID.all = ptDrvrData->ptCanRegs->CAN_IF2ARB.all & MSGID_29BIT_MASK;
                        ptDrvrData->tRx.tFifo.MSG[ptDrvrData->tRx.tFifo.fifo_in].DATA_A.all = ptDrvrData->ptCanRegs->CAN_IF2DATA.all;
                        ptDrvrData->tRx.tFifo.MSG[ptDrvrData->tRx.tFifo.fifo_in].DATA_B.all = ptDrvrData->ptCanRegs->CAN_IF2DATB.all;
                        ptDrvrData->tRx.tFifo.MSG[ptDrvrData->tRx.tFifo.fifo_in].DLC = ptDrvrData->ptCanRegs->CAN_IF2MCTL.bit.DLC;
                    }
                    ptDrvrData->tRx.tFifo.fifo_avl++;
                    ptDrvrData->tRx.tFifo.fifo_in++;
                    if (ptDrvrData->tRx.tFifo.fifo_in >= MAX_A825DRVR_RCV_FIFO_SIZE)
                    {
                        ptDrvrData->tRx.tFifo.fifo_in = 0U;
                    }
                } /* Normal Fifo operation */
            } /* Mailbox has received a message */
        }
    }

} /* ARINC 825 Driver Receive Data Service Function */


/***************************************************************************************************
 * Function: A825Drvr_SendTxData
 * Purpose: Send A825 message
 * Global Input(s):
 *      g_ptr_SendMailBox       Transmit CAN mailbox
 * Global Outputs(s): None
 * Inputs: A825Mgr_MSG_t txMsg-----------A825 message to transmit
 * Output: ERRORCODES return code--see include file
 ****************************************************************************************************/
ERRORCODES A825Drvr_SendTxData(A825Drvr_Data_t* const ptDrvrData, const MSG_MGR_t *const ptr_a825_msg_to_send)
{   /* ARINC 825 Driver Send Transmit Data function */
    ERRORCODES retval = STAT_SUCCESS;
    uint16_t u16ExitCounter = A825_WAIT_COUNTER;
    union CAN_IF1CMD_REG CAN_IF1CMD_SHADOW;

    if(ptDrvrData != NULL)
    {
        /* Check if still waiting on previous transmitted message to finish. */
        if(ptDrvrData->ptCanRegs->CAN_IF1MCTL.bit.TxRqst != 0)
        {
            /* If previous message didn't finish transmitting flag as a failure and
             * continue to transmit a new message. */
            retval = TX_FAILURE;
        }

        /* Previous Message was successfully transmitted. Can schedule next message to go out. */
        if ((ptr_a825_msg_to_send != (MSG_MGR_t *)NULL))
        {
            /* Message Pointer Check is Ok. Proceed with transmission on to bus */
            /* Use Shadow variable for IF1CMD. IF1CMD should be written to in single 32-bit write. */
            /* Wait for busy bit to clear. */
            while((ptDrvrData->ptCanRegs->CAN_IF1CMD.bit.Busy) && (u16ExitCounter != 0U))
            {
                u16ExitCounter = u16ExitCounter - 1U; /* Protect from endless loop */
            }

            /* Check if Exit Counter Protection timer expired */
            if(u16ExitCounter != 0U)
            {
                /* CAN Hardware ready for new message. Ok to send new data */
                /* Write CAN Frame data to transfer into ARB, DATA-A, and DATA-B interface registers */
                ptDrvrData->ptCanRegs->CAN_IF1ARB.bit.ID = ptr_a825_msg_to_send->MSGID.all;
                ptDrvrData->ptCanRegs->CAN_IF1DATA.all =  ptr_a825_msg_to_send->DATA_A.all;
                ptDrvrData->ptCanRegs->CAN_IF1DATB.all =  ptr_a825_msg_to_send->DATA_B.all;
                ptDrvrData->ptCanRegs->CAN_IF1MCTL.bit.DLC = ptr_a825_msg_to_send->DLC;

                /* Set Direction to write and set DATA-A/DATA-B to be transfered to message object */
                CAN_IF1CMD_SHADOW.all = 0UL;
                CAN_IF1CMD_SHADOW.bit.DIR = 1UL;
                CAN_IF1CMD_SHADOW.bit.Arb = 1UL;
                CAN_IF1CMD_SHADOW.bit.DATA_A = 1UL;
                CAN_IF1CMD_SHADOW.bit.DATA_B = 1UL;
                ptDrvrData->ptCanRegs->CAN_IF1ARB.bit.Dir = 1UL;

                /* Set Tx Request Bit */
                CAN_IF1CMD_SHADOW.bit.TXRQST = 1UL;

                /* Transfer the message object to the message object specified by objID. */
                CAN_IF1CMD_SHADOW.bit.MSG_NUM = A825_TX_MAILBOX_NUMBER;
                ptDrvrData->ptCanRegs->CAN_IF1CMD.all = CAN_IF1CMD_SHADOW.all;

            }
            else
            {
                /* Busy timer elapsed. Flag as a Timeout Error. */
                retval = TIMEOUT;
            }
        }
        else
        {
            /* Invalid Message Pointer. Return */
            retval = NULLPTR;
        }
    }
    else
    {
        /* Invalid Driver Pointer. Return */
        retval = NULLPTR;
    }

    return(retval);
} /* ARINC 825 Driver Send Transmit Data function */


/***************************************************************************************************
 * Function: A825Drvr_ReadRxFifo
 * Purpose: Read A825 Receive FIFO
 * Global Inputs(s): A825Drvr_DataRxFifo[fifo_out] Top of A825 receive FIFO
 * Global Outputs(s): None
 * Inputs: MSG_MGR_t *ptr_msg      Pointer to ARINC 825 message storage
 * Output: None
 ****************************************************************************************************/
void A825Drvr_ReadRxFifo(A825Drvr_Data_t* const ptDrvrData, MSG_MGR_t *const ptr_msg)
{
    if ((ptDrvrData != NULL) && (ptr_msg != NULL))
    {
        ptr_msg->MSGID.all  = ptDrvrData->tRx.tFifo.MSG[ptDrvrData->tRx.tFifo.fifo_out].MSGID.all;
        ptr_msg->DATA_A.all = ptDrvrData->tRx.tFifo.MSG[ptDrvrData->tRx.tFifo.fifo_out].DATA_A.all;
        ptr_msg->DATA_B.all = ptDrvrData->tRx.tFifo.MSG[ptDrvrData->tRx.tFifo.fifo_out].DATA_B.all;
        ptr_msg->DLC = ptDrvrData->tRx.tFifo.MSG[ptDrvrData->tRx.tFifo.fifo_out].DLC;

        /* Re-initialize the fifo slot */
        ptDrvrData->tRx.tFifo.MSG[ptDrvrData->tRx.tFifo.fifo_out].MSGID.all  = 0UL;
        ptDrvrData->tRx.tFifo.MSG[ptDrvrData->tRx.tFifo.fifo_out].DATA_A.all = 0UL;
        ptDrvrData->tRx.tFifo.MSG[ptDrvrData->tRx.tFifo.fifo_out].DATA_B.all = 0UL;
        ptDrvrData->tRx.tFifo.MSG[ptDrvrData->tRx.tFifo.fifo_out].DLC        = 0U;

        /* Update the position to pop the message from. Built in initialization if max position met */
        ptDrvrData->tRx.tFifo.fifo_out = (ptDrvrData->tRx.tFifo.fifo_out + 1U) % MAX_A825DRVR_RCV_FIFO_SIZE;
        /* Update the number of messages remaining by decrementing one */
        ptDrvrData->tRx.tFifo.fifo_avl--;
    }
}

/***************************************************************************************************
 * Function: A825Drvr_Disable
 * Purpose: Disable the communication on the CAN bus
 * Global Inputs(s): None
 * Global Outputs(s): GpioCtrlRegs
 * Inputs: None
 * Output: None
 *
 * Notes: fifo_out wraps around to start of FIFO
 ****************************************************************************************************/
//void A825Drvr_Disable()
//{
//    /*Justification for Coding Standard Deviation:
//     *    keyword 'asm' is deprecated
//     *    An exception to MISRA Rules 2.1 is required.
//     */
//    /*lint -e{586} # intended as is */
//    EALLOW;    //lint !e586
//    GpioCtrlRegs.GPAPUD.bit.GPIO30   = 1U; /* Enable pull-up for GPIO30 (CANRXA)    */
//    GpioCtrlRegs.GPAPUD.bit.GPIO31   = 1U; /* Enable pull-up for GPIO31 (CANTXA)    */
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 0U; /* Asynch qual for GPIO30 (CANRXA)       */
//    GpioCtrlRegs.GPAMUX2.bit.GPIO30  = 0U; /* Configure GPIO30 for CANRXA operation */
//    GpioCtrlRegs.GPAMUX2.bit.GPIO31  = 0U; /* Configure GPIO31 for CANTXA operation */
//    GpioCtrlRegs.GPBPUD.bit.GPIO39   = 1U;
//    GpioCtrlRegs.GPBMUX1.bit.GPIO39  = 1U;
//
//    /*Justification for Coding Standard Deviation:
//     *    keyword 'asm' is deprecated
//     *    An exception to MISRA Rules 2.1 is required.
//     */
//    /*lint -e{586} # intended as is */
//    EDIS;    //lint !e586
//}
