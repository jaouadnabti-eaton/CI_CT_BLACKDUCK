/****************************************************************************************************
*  File name: A825Mgr.c
*
*  Purpose: Provides API to ARINC 825 functionality.
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
*  Adam Bouwens     12/07/2022  N/A        Port to MCU
*
****************************************************************************************************/
#include "A825Mgr.h"
#include "A825Drvr.h"
#include "A825_MsgInfo.h"
//#include "Exec.h"
//#include "ExecTbl.h"
//#include "FqscFlightMgr.h"
//#include "Utils.h"
//#include "ExecApi.h"
#include "F2837xD_device.h"
#include "Defines.h"
#include "F2837xD_GlobalPrototypes.h"
//#include "F2837xD_Gpio_defines.h"

/* ARINC825 BUS INITIALIZATION */
A825_Bus_t tA825_Bus[NUM_CAN_CHANNELS] =
{
    { /* Control Bus (CAN Channel A) Bus Initializer */
      A825_CONTROL_BUS,
      { A825_CNTRL_BUS_TX_NUM_MSGS, tA825CntrlBusTxMsgList, A825_TX_SCHEDULE_A, 0U, 0U, 0U },
      { A825_CNTRL_BUS_RX_NUM_MSGS, tA825CntrlBusRxMsgList },
      A825_SRVC_COUNTER_INIT, 0UL, 0UL, false, &tA825Drvr_Data[CAN_CHANNEL_A]
    },
    { /* Maintenance Bus (CAN Channel B) Bus Initializer */
      A825_MAINTENANCE_BUS,
      { A825_MAINT_BUS_TX_NUM_MSGS, tA825MaintBusTxMsgList, A825_TX_SCHEDULE_A, 0U, 0U, 0U },
      { A825_MAINT_BUS_RX_NUM_MSGS, tA825MaintBusRxMsgList },
      A825_SRVC_COUNTER_INIT, 0UL, 0UL, false, &tA825Drvr_Data[CAN_CHANNEL_B]
    }
};

/* Function Prototypes */
static void A825Mgr_TxSyncToTimeslot(A825_Bus_t* const ptA825Bus);
static void A825Mgr_DetRxStaleStatus(A825_Bus_t* const ptA825Bus);
static void A825Mgr_SetRxMsgPayload(A825_Bus_t* const ptA825Bus, const MSG_MGR_t *const ptr_a825_rx_message);
static void A825Mgr_SendA825Data(A825_Bus_t* const ptA825Bus);
static void A825Mgr_SetSignalValueInPayload(uint64_t *const ptr_payload,const uint64_t new_value, const Signal_Info *const ptr_signal);
static float32_t A825Mgr_ScaleUpAndRound_BNR(const float32_t f32Value, const float32_t f32InverseResolution);
static float32_t Utils_ConTwosCompBitToFloat(uint32_t value_in_twos_comp, uint32_t number_of_bits);

/****************************************************************************************************
*  Function: A825Mgr_Init
*  Purpose: Initialize ARINC 825 Manager data tables.
*  Global Inputs:  G_A825_Table_All_Messages, G_A825_Table_All_Messages
*  Global Outputs: G_A825_TxRx_Count_All_Msg, G_RxMsgStaleStatus, G_RxMsgEverRecvd
*  Input: None
*  Output: retval error code
****************************************************************************************************/
ERRORCODES A825Mgr_Init( void )
{
    uint16_t i = 0U;
    uint16_t index;
    ERRORCODES retval = FAILURE;

    /* Initialize the number of CAN channels that are configured. */
    do
    {
        /* Init the Transmit message list data */
        if(tA825_Bus[i].tTx.ptMsgList != NULL)
        {
            tA825_Bus[i].tTx.u16TxMsgIdx = 0U;
            /* Init TX Message Variables */
            for (index = 0U; index < tA825_Bus[i].tTx.u16NumMsgs; index++)
            {
                tA825_Bus[i].tTx.ptMsgList[index].tData.u64 = 0x0ULL;
                tA825_Bus[i].tTx.ptMsgList[index].u32LastCount = 0UL;
            }
        }
        /* Init the Receive message list data */
        if(tA825_Bus[i].tRx.ptMsgList != NULL)
        {
            /* Init RX Message Variables */
            for (index = 0U; index < tA825_Bus[i].tRx.u16NumMsgs; index++)
            {
                tA825_Bus[i].tRx.ptMsgList[index].tData.u64 = 0x0ULL;
                tA825_Bus[i].tRx.ptMsgList[index].u32LastCount = 0UL;
            }
        }

        /* Initialize the CAN Driver for the ARINC825 Channel */
        retval = A825Drvr_Init(tA825_Bus[i].ptDrvrData, tA825_Bus[i].tBusType);

        i++;    /* Increment CAN Channel Index */
    } while((i < NUM_CAN_CHANNELS) && (retval == STAT_SUCCESS)); /* Break out of loop if all chnls serviced or an error on any */

    return retval;
}

/****************************************************************************************************
*  Function: A825Mgr_SendA825Data
*  Purpose: Send any scheduled outgoing ARINC 825 messages.
*  Global Inputs:  G_A825_Table_All_Messages, G_A825_TxRx_Count_All_Msg, G_A825_Table_All_Messages,
*               G_A825_Table_All_Messages, G_A825_TxRx_Count_All_Msg
*  Global Outputs: G_A825_TxRx_Count_All_Msg, G_a825_tx_msg_idx
*  Input: None
*  Output: None
****************************************************************************************************/
static void A825Mgr_SendA825Data(A825_Bus_t* const ptA825Bus)
{
    MSG_MGR_t msg_to_tx;
    MSGID_REG_MGR     tMsgId;
    uint32_t          tx_rate;
    ERRORCODES retval = FAILURE;
    A825TxTableEntry* ptTxMsg = NULL;
    uint16_t*         pu16TxMsgIdx = NULL;

    if(ptA825Bus != NULL)
    {
        /* Decode TX Schedule to Service to update Schedule Indices */
        switch (ptA825Bus->tTx.tSchedule)
        {
            case A825_TX_SCHEDULE_A:
            {
                ptA825Bus->tTx.tSchedule = A825_TX_SCHEDULE_A;
                pu16TxMsgIdx = &ptA825Bus->tTx.u16TxMsgIdx_A;
                break;
            }
            case A825_TX_SCHEDULE_B:
            {
                ptA825Bus->tTx.tSchedule = A825_TX_SCHEDULE_B;
                pu16TxMsgIdx = &ptA825Bus->tTx.u16TxMsgIdx_B;
                break;
            }
            default:
            {
                ptA825Bus->tTx.tSchedule = A825_TX_SCHEDULE_A;
                pu16TxMsgIdx = &ptA825Bus->tTx.u16TxMsgIdx_A;
                break;
            }
        }

        if(pu16TxMsgIdx != NULL)
        {
            /* Find next message for the TX Schedule being serviced */
            while (ptA825Bus->tTx.ptMsgList[*pu16TxMsgIdx].tSchedule != ptA825Bus->tTx.tSchedule)
            {
                *pu16TxMsgIdx = *pu16TxMsgIdx + 1U;
                /* Boundary check */
                if (*pu16TxMsgIdx == ptA825Bus->tTx.u16NumMsgs)
                {
                    *pu16TxMsgIdx = 0U;
                }
            }
        }

        /* Set index to Schedule Index to service */
        ptA825Bus->tTx.u16TxMsgIdx = *pu16TxMsgIdx;
        ptTxMsg = &ptA825Bus->tTx.ptMsgList[*pu16TxMsgIdx];

        /*Justification for Coding Standard Deviation:
         *   Type conversions
         *    An exception to MISRA Rule 10.1 is required.
         */
        /*lint -e{960} # intended as is */
        /* Get the configured Transmit period of the message */
        //tx_rate = (uint32_t)(ptA825BusTx->ptMsgList[*pu16TxMsgIdx].tMsg.tRate.tx_period_ms / A825_SRVC_TIME_MILLISEC);
        /* Multiply by 8 as the service is ran at 8kHz and period is in 1kHz */
        tx_rate = (uint32_t)(ptTxMsg->tMsg.tRate.tx_period_ms << 3U);

        /* Check to see if the message is ready to transmit */
        if (((ptA825Bus->u32A825ServiceCounter - ptTxMsg->u32LastCount)) >= tx_rate)
        {
            /* Setup the ARINC825 Message ID */
            tMsgId.all = ptTxMsg->tMsg.MsgId.all;
            //tMsgId.a825_o2m.DSTID = SCU_LRU;   /* Set Destination ID Field */
            tMsgId.a825_o2m.SRCID = G_eLruId;   /* Set Source ID Field */
            tMsgId.a825_o2m.ACTID = G_eActId;   /* Set Actuator ID Field */
            tMsgId.a825_o2m.RCI = G_eChId;      /* Set RCI Field */

            /* Save the ARINC825 Message to the Message Object Type to Transmit on the Bus */
            msg_to_tx.MSGID.all = tMsgId.all;
            msg_to_tx.DLC = ptTxMsg->tMsg.data_length_in_bytes;
            /* Format the data as Big_Endian on the CAN Bus */
            msg_to_tx.DATA_B.byte.u8_7 = ptTxMsg->tData.tu8.u8_0;
            msg_to_tx.DATA_B.byte.u8_6 = ptTxMsg->tData.tu8.u8_1;
            msg_to_tx.DATA_B.byte.u8_5 = ptTxMsg->tData.tu8.u8_2;
            msg_to_tx.DATA_B.byte.u8_4 = ptTxMsg->tData.tu8.u8_3;
            msg_to_tx.DATA_A.byte.u8_3 = ptTxMsg->tData.tu8.u8_4;
            msg_to_tx.DATA_A.byte.u8_2 = ptTxMsg->tData.tu8.u8_5;
            msg_to_tx.DATA_A.byte.u8_1 = ptTxMsg->tData.tu8.u8_6;
            msg_to_tx.DATA_A.byte.u8_0 = ptTxMsg->tData.tu8.u8_7;

            /* Send the ARINC825 Message */
            retval = A825Drvr_SendTxData(ptA825Bus->ptDrvrData, &msg_to_tx);

            /* Check if there was a problem transmitting the message. */
            if (retval != STAT_SUCCESS)
            {
                /* If there was a problem sending out on the bus increment general Tx error counter */
                ptA825Bus->ptDrvrData->tTx.u32ErrCnt++;
                /* Check if the previous message failed to transmit */
                if(retval == TX_FAILURE)
                {
                    /* Increment transmit failure counter */
                    ptA825Bus->ptDrvrData->tTx.u16TxFailureCnt++;
                }
                /* Check if the error was a null pointer */
                else if(retval == NULLPTR)
                {
                    /* Increment null pointer counter */
                    ptA825Bus->ptDrvrData->tTx.u16NullPtrCnt++;
                }
                /* Check if the CAN hardware timed out */
                else if(retval == TIMEOUT)
                {
                    /* Increment timeout counter */
                    ptA825Bus->ptDrvrData->tTx.u16TimeoutCnt++;
                }
            }

            /* Save off Last Count associated with transmission of the message */
            ptTxMsg->u32LastCount = ptA825Bus->u32A825ServiceCounter;

            do
            {
                /* Update to next index for schedule being serviced */
                *pu16TxMsgIdx = *pu16TxMsgIdx + 1U;
                /* Boundary check */
                if (*pu16TxMsgIdx == ptA825Bus->tTx.u16NumMsgs)
                {
                    *pu16TxMsgIdx = 0U;
                }
            } while (ptA825Bus->tTx.ptMsgList[*pu16TxMsgIdx].tSchedule != ptA825Bus->tTx.tSchedule);
        }
    }
}

/****************************************************************************************************
*  Function: A825Mgr_TxSrvc
*  Purpose: Schedule and Transmit A825 CAN frames onto the bus.
*
*  Global Inputs:
*  Global Outputs:
*
*  Input: None
*  Output: None
****************************************************************************************************/
void A825Mgr_TxSrvc(A825_Bus_t* const ptA825Bus)
{
    bool bScheduleTxMsgs = false;
    uint32_t u32Timeslot = 0U;
    uint32_t u32TimeslotTemp = 0U;

    if (ptA825Bus != NULL)
    {
        /* Service the counter used for scheduling TX Messages and determining stale status for RX Messages */
        ptA825Bus->u32A825ServiceCounter = ptA825Bus->u32A825ServiceCounter + 1UL;

        /* Only the Control Bus will adjust to the SCU message inputs since the Maint Bus is for debugging
         * only and may not have the SCU messages coming as inputs. */
        if(ptA825Bus->tBusType == A825_CONTROL_BUS)
        {
            /* Monitor and adjust the Counter to align the A825 TX messages to the proper time slot based upon the node ID */
            A825Mgr_TxSyncToTimeslot(ptA825Bus);
        }

        /* Counter is based on 8kHz (0.125ms) timer. 8 counts per millisecond
         * CAN frame is a 10ms base where that all nodes share. Each node is permitted to transmit
         * at specific 1ms intervals in this frame.
         */

        /* Convert to timeslot base counter. 20 A825 message timeslots in 10ms Frame. (20/10ms = 0.5ms timeslot)
         * 80 counts in 10ms Frame @8kHz. Get which 1/8th millisecond counter we're on
         * Divide 1/8th millisecond counter by 4 to convert to which timeslot we're currently on */
        u32Timeslot = ((ptA825Bus->u32A825ServiceCounter % A825_COUNTS_PER_PERIOD) >> 2UL);
        u32TimeslotTemp = ptA825Bus->u32A825ServiceCounter % A825_COUNTS_PER_TIMESLOT; /* Remainder of divide by 4 gives 8th millisecond increment that we're on */

        /* Check if this MCU Channel (node) has the bus. If so, service the TX messages. */
        if(u32TimeslotTemp == 0UL)
        {
            if(u32Timeslot == 1UL)
            {
                /* Schedule A Time Slot */
                ptA825Bus->tTx.tSchedule = A825_TX_SCHEDULE_A;
                /* Tx Messages to go out on the bus */
                bScheduleTxMsgs = true;
            }
            else if(u32Timeslot == 2UL)
            {
                /* Schedule B Time Slot */
                ptA825Bus->tTx.tSchedule = A825_TX_SCHEDULE_B;
                /* Tx Messages to go out on the bus */
                bScheduleTxMsgs = true;
            }
            else
            {
                bScheduleTxMsgs = false;
            }

            if(bScheduleTxMsgs == true)
            {
                /* Tx Messages to go out on the bus */
                A825Mgr_SendA825Data(ptA825Bus);
            }
        }
    }
}

/****************************************************************************************************
*  Function: A825Mgr_TxSyncToTimeslot
*  Purpose: Monitor and Adjust the main counter used for scheduling the A825 CAN frames
*
*  Global Inputs: G_A825_Rx_Count_All_Msg, G_A825_Tx_Count_All_Msg, G_A825_Table_Tx_Messages
*                   G_RxMsgStaleStatus, G_u32A825ServiceCounter
*  Global Outputs: G_A825_Rx_Count_All_Msg, G_A825_Tx_Count_All_Msg, G_A825_Table_Tx_Messages
*                   G_RxMsgStaleStatus, G_u32A825ServiceCounter
*
*  Input: None
*  Output: None
****************************************************************************************************/
void A825Mgr_TxSyncToTimeslot(A825_Bus_t* const ptA825Bus)
{
    static bool bSyncWasStale = true;
    /* SCU -> MCUx:  POSITION COMMAND (A825_CNTRL_BUS_RX_FLAP_CMD) message. */
    uint32_t u32ScuPsnCmdCnt;
    /* MCUx -> SCU:  POSITION FEEDBACK (A825_CNTRL_BUS_TX_MCU_PSN_STATUS_FLT_MAINT) message */
    uint32_t u32McuPsnFdbkCnt;
    /* Period for the MCU POSITION FEEEDBACK message */
    uint32_t u32McuPsnFdbkPeriod;
    /* Tentatively next scheduled MCU POSITION FEEDBACK message */
    uint32_t u32McuPsnFdbkCntNext = 0UL;
    uint32_t u32Diff = 0UL;
    uint32_t u32IdealTimeslotCnt = 0UL;
    uint16_t u16Index;

    /* IMPORTANT NOTE:
     * Each MCU filters for the SCU message destined for it's node ID through the Mailbox setup. Therefore,
     * the counters associated with the SCU message is the one that was sent specifically to this node. The
     * other SCU messages for other nodes are not received. This is important in determining when to respond
     * to the SCU. Notice that the MCUs first response depends on the channel L,R. The Left channel will
     * respond in the first timeslot following the SCU message, and the Right channel will respond in the third
     * timeslot following the SCU message. This is true for all MCUs no matter the IB/OB designation.
     */
    if(ptA825Bus != NULL)
    {
        /* SCU -> MCUx:  Count of Last Received POSITION COMMAND (A825_CNTRL_BUS_RX_FLAP_CMD) message. */
        u32ScuPsnCmdCnt = ptA825Bus->tRx.ptMsgList[A825_CNTRL_BUS_RX_FLAP_CMD].u32LastCount;
        /* MCUx -> SCU:  Count of Last Sent POSITION FEEDBACK (A825_CNTRL_BUS_TX_MCU_PSN_STATUS_FLT_MAINT) message */
        u32McuPsnFdbkCnt = ptA825Bus->tTx.ptMsgList[A825_CNTRL_BUS_TX_MCU_PSN_STATUS_FLT_MAINT].u32LastCount;
        /* Period for the MCU POSITION FEEEDBACK message */
        u32McuPsnFdbkPeriod = (uint32_t)(ptA825Bus->tTx.ptMsgList[A825_CNTRL_BUS_TX_MCU_PSN_STATUS_FLT_MAINT].tMsg.tRate.tx_period_ms << 3U);

        /* Check the A825_CNTRL_BUS_RX_FLAP_CMD for stale status */
        if(ptA825Bus->tRx.ptMsgList[A825_CNTRL_BUS_RX_FLAP_CMD].bMsgStaleStatus == false)
        {   /* SCU Message is NOT STALE */

            /* Can only monitor alignment error if the SCU POSITION COMMAND message
             * is current (not stale data) and has been received at least one time. */

            /* Protection to only run the adjustment once time per SCU period */
            if(u32ScuPsnCmdCnt != ptA825Bus->u32PositionCmdCntPrev)
            {   /* New SCU Message to process timeslot */
                /* Fresh SCU message arrived to calculate error and adjust if necessary.*/
                /* Calculate the period of the latest SCU message */
                u32Diff = u32ScuPsnCmdCnt - ptA825Bus->u32PositionCmdCntPrev;

                /* If SCU was stale then disregard the period and run the alignment calculation since this is
                 * either the first SCU message received or the first one received after coming online
                 * again after a long enough period to declare stale [offline]. This logic protects against
                 * math errors that may arise as a result of counter rollovers due to asynchronous SCU message
                 * arriving at particular counter times that are problematic.
                 * If the SCU messages are up-to-date (not stale) then make sure the periodicity of the
                 * SCU messages is within the specified limit (SCU Period is 10ms +/- 10%)
                 * such that we aren't adjusting the alignment to invalid SCU periods.
                 * This prevents bus flooding in case the SCU messages are coming at unexpected periods. */
                if((bSyncWasStale == true) || ((bSyncWasStale == false) &&
                    ((u32Diff <= A825_ALIGN_TIMESLOT_TO_SCU_MSG_PERIOD_MAX) && (u32Diff >= A825_ALIGN_TIMESLOT_TO_SCU_MSG_PERIOD_MIN))))
                {   /* SCU Message was stale OR SCU Message is within specified period limits to calculate and adjust timeslot. */
                    /* SCU message is not stale and period is within specified limits, or this is the first
                     * SCU message received after a stale condition. */

                    /* POSITION COMMAND message is up-to-date and we are in the latest 10ms CAN
                     * frame where the timeslot for the MCU needs to be checked against it's ideal
                     * timeslot and adjusted and adjusted if necessary .*/

                    /* CALCULATE THE IDEAL TIMESLOT FOR THE NODE TO COMMUNICATE */
                    /* SCU_LIB      Timeslot  #0:  0.0ms:  Cnt:  0
                     * MCU_LIB_CHL  Timeslot  #1:  0.5ms:  Cnt:  4
                     * MCU_LIB_CHL  Timeslot  #2:  1.0ms:  Cnt:  8
                     * MCU_LIB_CHR  Timeslot  #3:  1.5ms:  Cnt: 12
                     * MCU_LIB_CHR  Timeslot  #4:  2.0ms:  Cnt: 16
                     * SCU_RIB      Timeslot  #5:  2.5ms:  Cnt: 20
                     * MCU_RIB_CHL  Timeslot  #6:  3.0ms:  Cnt: 24
                     * MCU_RIB_CHL  Timeslot  #7:  3.5ms:  Cnt: 28
                     * MCU_RIB_CHR  Timeslot  #8:  4.0ms:  Cnt: 32
                     * MCU_RIB_CHR  Timeslot  #9:  4.5ms:  Cnt: 36
                     * SCU_LOB      Timeslot #10:  5.0ms:  Cnt: 40
                     * MCU_LOB_CHL  Timeslot #11:  5.5ms:  Cnt: 44
                     * MCU_LOB_CHL  Timeslot #12:  6.0ms:  Cnt: 48
                     * MCU_LOB_CHR  Timeslot #13:  6.5ms:  Cnt: 52
                     * MCU_LOB_CHR  Timeslot #14:  7.0ms:  Cnt: 56
                     * SCU_ROB      Timeslot #15:  7.5ms:  Cnt: 60
                     * MCU_ROB_CHL  Timeslot #16:  8.0ms:  Cnt: 64
                     * MCU_ROB_CHL  Timeslot #17:  8.5ms:  Cnt: 68
                     * MCU_ROB_CHR  Timeslot #18:  9.0ms:  Cnt: 72
                     * MCU_ROB_CHR  Timeslot #19:  9.5ms:  Cnt: 76
                     *
                     * End of 10ms A825 frame and all 4 SCU XYB and MCUs have responded.
                     *
                     * MCU CH_L Ideal Timeslot is first after SCU (1):  in ms is 0.5ms; 0.5/0.125 =  4
                     * MCU CH_L Ideal Timeslot in Counts is:  4 (for first timeslot to communicate)
                     * MCU CH_R Ideal Timeslot is third after SCU (3):  in ms is 1.5ms; 1.5/0.125 = 12
                     * MCU CH_R Ideal Timeslot in Counts is: 12 (for first timeslot to communicate)
                     * Ideal Timeslot = (CH_ID x 2) + 1,  CH_L = 1, CH_R = 3
                     * Ideal Timeslot in Counts = Ideal Timeslot x 4
                     * Therefore, Ideal Timeslot in Counts = ((CH_ID x 2) + 1) x 4   */

                    /* Calculate the target timeslot in 1/8th millisecond counts
                     * Ideal Timeslot in Counts = ((CH_ID x 2) + 1) x 4  */
                    u32IdealTimeslotCnt = ((G_eActId << 1U) + 1U) << 2UL;

                    /* Calculate the next POSITION FEEDBACK message count based on last message count */
                    u32McuPsnFdbkCntNext = u32McuPsnFdbkCnt + u32McuPsnFdbkPeriod;
                    /* Normalize the count to the 10ms frame beginning with new SCU POSITION COMMAND message */
                    u32McuPsnFdbkCntNext = u32McuPsnFdbkCntNext - u32ScuPsnCmdCnt;

                    /* Check if timing needs adjusting. If alignment is outside of 0.25ms, then adjust */
                    if((u32McuPsnFdbkCntNext - u32IdealTimeslotCnt) > A825_ALIGN_TIMESLOT_IDEAL_TOLERANCE)
                    {   /* Timeslot needs to be adjusted */
                        /* Timing needs adjustment. Determine if we need to advance or retard the timing. */
                        t64Debug.tu8.u8_3++;
                        if(u32McuPsnFdbkCntNext > u32IdealTimeslotCnt)
                        {   /* Advance the timing to meet Ideal Timeslot */
                            t64Debug.tu8.u8_4++;
                            /* Need to Advance the timing. Calculate the difference */
                            u32Diff = (u32McuPsnFdbkCntNext - u32IdealTimeslotCnt);
                            /* Advance the timing by the difference */
                            ptA825Bus->u32A825ServiceCounter = ptA825Bus->u32A825ServiceCounter + u32Diff;
                            /* Calculate the new 1/8th ms division to run the TX Scheduler on */
                            ptA825Bus->u32EighthMillisecondTarget = u32Diff % A825_COUNTS_PER_TIMESLOT;
                            /* Update the RX counters used for detecting stale status */
                            for (u16Index = 0U; u16Index < ptA825Bus->tRx.u16NumMsgs; u16Index++)
                            {   /* Update Receive Counters used to detect Stale Status */
                                ptA825Bus->tRx.ptMsgList[u16Index].u32LastCount = ptA825Bus->tRx.ptMsgList[u16Index].u32LastCount + u32Diff;
                            }   /* Update Receive Counters used to detect Stale Status */
                        }   /* Advance the timing to meet Ideal Timeslot */
                        else if (u32McuPsnFdbkCntNext < u32IdealTimeslotCnt)
                        {   /* Retard the timing to meet Ideal Timeslot */
                            t64Debug.tu8.u8_5++;
                            /* Need to Retard the Timing. Calculate the difference */
                            u32Diff = (u32IdealTimeslotCnt - u32McuPsnFdbkCntNext);
                            /* Retard the timing by the difference */
                            ptA825Bus->u32A825ServiceCounter = ptA825Bus->u32A825ServiceCounter - u32Diff;
                            /* Calculate the new 1/8th ms division to run the TX Scheduler on */
                            ptA825Bus->u32EighthMillisecondTarget = u32Diff % A825_COUNTS_PER_TIMESLOT;
                            /* Update the RX counters used for detecting stale status */
                            for (u16Index = 0U; u16Index < ptA825Bus->tRx.u16NumMsgs; u16Index++)
                            {   /* Update Receive Counters used to detect Stale Status */
                                ptA825Bus->tRx.ptMsgList[u16Index].u32LastCount = ptA825Bus->tRx.ptMsgList[u16Index].u32LastCount - u32Diff;
                            }   /* Update Receive Counters used to detect Stale Status */
                        }   /* Retard the timing to meet Ideal Timeslot */
                    }   /* Timeslot needs to be adjusted */
                }   /* SCU Message was stale OR SCU Message is within specified period limits to calculate and adjust timeslot. */
            }   /* New SCU Message to process timeslot */
            bSyncWasStale = false;
        }   /* SCU Message is NOT STALE */
        else
        {   /* SCU Message is Stale */
            /* SCU Message is currently time-out (stale) set flag for one-shot when it comes back online
             * to do the alignment again. */
            bSyncWasStale = true;
        }   /* SCU Message is Stale */

        /* Save copy of new SCU POSITION COMMAND counter to prevent multiple adjustments for the
         * CAN frame we are adjusting. One shot. */
        ptA825Bus->u32PositionCmdCntPrev = u32ScuPsnCmdCnt;
    }
}

/****************************************************************************************************
*  Function: A825Mgr_RxSrvc
*  Purpose: Read and process any incoming ARINC 825 messages.
*
*  Global Inputs: G_nvmd_state, G_normal_sched_tran_occ, G_maint_sched_tran_occ,
*                 G_tot_mass_calc_status, G_A825_transmission_start
*  Global Outputs: G_normal_sched_tran_occ, G_maint_sched_tran_occ, G_A825_transmission_start
*
*  Input: None
*  Output: None
****************************************************************************************************/
void A825Mgr_RxSrvc(A825_Bus_t* const ptA825Bus)
{
    MSG_MGR_t  a825_rx_message;

    if(ptA825Bus != NULL)
    {
        /* Get the received messages from the hardware (mailboxes) and store to Rx FIFO */
        A825Drvr_ReceiveRxDataService(ptA825Bus->ptDrvrData);

        /* At 8kHz service rate and 500kBAUD, there can be no more than a single message received
         * per function call. Can change to an "if" rather than a "while" for this design; however,
         * not changing for now to leave more portable in case timing and num mailboxes is changed. */

        /* Process all received messages in the Rx FIFO */
        while(ptA825Bus->ptDrvrData->tRx.tFifo.fifo_avl > 0U)
        {
            /* Read from the Rx FIFO */
            A825Drvr_ReadRxFifo(ptA825Bus->ptDrvrData, &a825_rx_message);
            /* Process the received message */
            A825Mgr_SetRxMsgPayload(ptA825Bus, &a825_rx_message);
        }

        /* Determine if the configured receive signals are timed out or not */
        A825Mgr_DetRxStaleStatus(ptA825Bus);
    }
}


/****************************************************************************************************
*  Function: A825Mgr_IsRxMsgStale
*  Purpose: Get stale status for ARINC 825 receive message data.
*  Global Inputs: G_RxMsgStaleStatus
*  Global Outputs: None
*  Input: A825_MSG_ID_LOOKUP_TABLE msg_id  //A825 message ID
*  Output: None
****************************************************************************************************/
bool A825Mgr_IsRxMsgStale(A825_Bus_Rx_t* const ptA825Rx, const uint16_t msg_id)
{
    bool stale = false;;

    if (msg_id < ptA825Rx->u16NumMsgs)
    {
        stale = ptA825Rx->ptMsgList[msg_id].bMsgStaleStatus;
    }

    return stale;
}

/****************************************************************************************************
*  Function: A825Mgr_DetRxStaleStatus
*  Purpose: Determine if ARINC 825 received data is stale.
*  Global Inputs:  G_A825_TxRx_Count_All_Msg, G_rx_track_counter,
*  Global Outputs: G_rx_track_counter, G_RxMsgStaleStatus
*  Input: None
*  Output: None
****************************************************************************************************/
static void A825Mgr_DetRxStaleStatus(A825_Bus_t* const ptA825Bus)
{
    uint16_t index;
    uint32_t stale_period = 0U;

    if(ptA825Bus != NULL)
    {
        for (index = 0U; index < ptA825Bus->tRx.u16NumMsgs; index++)
        {
            /* If message was fresh, determine if it is now stale. Once stale, a receipt of new message will reset */
            if(ptA825Bus->tRx.ptMsgList[index].bMsgStaleStatus == false)
            {
                /* Get the stale time configuration for each individual message. */
                /* Multiply by 8 as the service is ran at 8kHz and period is in 1kHz */
                stale_period = (uint32_t)(ptA825Bus->tRx.ptMsgList[index].tMsg.tRate.rx_stale_time_ms << 3U);

                if ((ptA825Bus->u32A825ServiceCounter - ptA825Bus->tRx.ptMsgList[index].u32LastCount) > stale_period)
                {
                    ptA825Bus->tRx.ptMsgList[index].bMsgStaleStatus = true;
                }
            }
        }
    }
}


/****************************************************************************************************
*  Function: A825Mgr_SetRxMsgPayload
*  Purpose: Store the payload of an A825 message into the A825 received payload table.
*  Global Inputs:  G_rx_track_counter,
*  Global Outputs:
*  Input:
*  Output: None
****************************************************************************************************/
static void A825Mgr_SetRxMsgPayload(A825_Bus_t* const ptA825Bus, const MSG_MGR_t *const ptr_a825_rx_message)
{
    bool  msg_id_found = false;
    uint16_t   index;

    if ((ptA825Bus != NULL) && (ptr_a825_rx_message != NULL))
    {
        for (index = 0U; ((index < ptA825Bus->tRx.u16NumMsgs) && (msg_id_found == false)); index++)
        {
            if ((ptA825Bus->tRx.ptMsgList[index].tMsg.MsgId.all & A825_MSG_ID_CHECK_FILTER) ==
                (ptr_a825_rx_message->MSGID.all & A825_MSG_ID_CHECK_FILTER))//CR1208

            {
                msg_id_found = true;

                /* Ensure the DLC received is what was expected. Throw out if not */
                if(ptr_a825_rx_message->DLC == ptA825Bus->tRx.ptMsgList[index].tMsg.data_length_in_bytes)
                {
                    /* Set the payload */
                    /* CAN Bus Data is formatted in Big-Endian, store it to local memory format */
                    ptA825Bus->tRx.ptMsgList[index].tData.tu8.u8_0 = ptr_a825_rx_message->DATA_B.byte.u8_7;
                    ptA825Bus->tRx.ptMsgList[index].tData.tu8.u8_1 = ptr_a825_rx_message->DATA_B.byte.u8_6;
                    ptA825Bus->tRx.ptMsgList[index].tData.tu8.u8_2 = ptr_a825_rx_message->DATA_B.byte.u8_5;
                    ptA825Bus->tRx.ptMsgList[index].tData.tu8.u8_3 = ptr_a825_rx_message->DATA_B.byte.u8_4;
                    ptA825Bus->tRx.ptMsgList[index].tData.tu8.u8_4 = ptr_a825_rx_message->DATA_A.byte.u8_3;
                    ptA825Bus->tRx.ptMsgList[index].tData.tu8.u8_5 = ptr_a825_rx_message->DATA_A.byte.u8_2;
                    ptA825Bus->tRx.ptMsgList[index].tData.tu8.u8_6 = ptr_a825_rx_message->DATA_A.byte.u8_1;
                    ptA825Bus->tRx.ptMsgList[index].tData.tu8.u8_7 = ptr_a825_rx_message->DATA_A.byte.u8_0;

                    /* Save off the service counter used to detect stale status */
                    ptA825Bus->tRx.ptMsgList[index].u32LastCount = ptA825Bus->u32A825ServiceCounter;

                    /* Clear Message Stale Status as message has been received */
                    ptA825Bus->tRx.ptMsgList[index].bMsgStaleStatus = false;
                }
            }
        }
    }
}

/****************************************************************************************************
*  Function: A825Mgr_SetSignalValueInPayload
*  Purpose: Update an entry in the "G_A825_Payload_tbl_msgs" table.
*  Global Inputs: None
*  Global Outputs: None
*  Input:
*  uint64_t *const ptr_payload          //Pointer to G_A825_Payload_Tbl_Msgs[] entry
*  const uint32_t new_value             //Value to update table entry
*  const Signal_Info *const ptr_signal  //Pointer to table entry's Signal_Info data
*  Output: None
****************************************************************************************************/
static void A825Mgr_SetSignalValueInPayload(uint64_t *const ptr_payload, const uint64_t new_value, const Signal_Info *const ptr_signal)
{
    uint64_t u64Mask = 0ULL;
    uint64_t u64NewValueAlignedToStart = new_value;

    if((ptr_signal->startPos + ptr_signal->sizeOfSignal) <= LONG_LONG_SIZE)
    {
        /* Cannot shift 64 bits or mask becomes zero. If signal size is 64-bit then
         * set to full 1's for the mask, otherwise set the mask based upon the size
         * of the signal. */
        if(ptr_signal->sizeOfSignal != LONG_LONG_SIZE)
        {
            /* Setup a Mask for the bits in this signal */
            u64Mask = ((1ULL << ptr_signal->sizeOfSignal) - 1ULL);
            u64Mask = u64Mask << ptr_signal->startPos;
        }
        else
        {
            /* Set mask to all 1's for 64-bit signal size */
            u64Mask = UNSIGNED_64BIT_MASK;
        }

        /* Align the Signal value to the start bit in the 64-bit signal */
        u64NewValueAlignedToStart = u64NewValueAlignedToStart << ptr_signal->startPos;

        /* Clear the signal value within the payload. */
        *ptr_payload &= ~u64Mask;
        /* Set the signal value within the payload. */
        *ptr_payload |= (u64Mask & u64NewValueAlignedToStart);
    }
}

/****************************************************************************************************
*  Function: A825Mgr_SetSignal
*  Purpose: Store value into "G_A825_Payload_Tbl_Msgs".
*  Global Inputs: G_A825_Table_All_Messages, G_A825_Payload_Tbl_Msgs
*  Global Outputs: None
*  Input:
*  const uint16_t lookup_id     // A825 message ID
*  const SIGNAL_ENTRY_INDEX signal_index        // Signal info index
*  Argument List ...                            // Value to store in the type native to the SIGNAL
*  Output: None
****************************************************************************************************/
void A825Mgr_SetSignal(A825_Bus_Tx_t* const ptA825Tx, const uint16_t lookup_id, const SIGNAL_ENTRY_INDEX signal_index, ...)
{
    A825TxTableEntry     *ptr_a825_message;
    const Signal_Info    *ptr_signal;
    uint64_t             *ptr_payload;
    t64Types_t           t64Data = { 0 };
    bool_t               bValidType = false;
    va_list              vaList;

    va_start(vaList, signal_index); /* Start the list at the signal_index argument */

    if ((ptA825Tx != NULL) && (lookup_id < ptA825Tx->u16NumMsgs))
    {
        ptr_a825_message = &ptA825Tx->ptMsgList[lookup_id];
        ptr_payload      = &ptr_a825_message->tData.u64;

        /*Justification for Coding Standard Deviation:
         *    pointer arithmetic is required for A825 signal message processing
         *    An exception to MISRA Rules 17.4 is required.
         */
        /*lint -e{960} # intended as is */
        ptr_signal = &ptr_a825_message->tMsg.ptr_signalInfoTable[(uint16_t)signal_index];

        if(ptr_signal != NULL)
        {
            switch (ptr_signal->signalType)
            {
                case SIGNAL_NODATA:
                case SIGNAL_ENUM:
                case SIGNAL_CHAR:
                case SIGNAL_UCHAR:
                case SIGNAL_ACHAR:
                {
                    break;
                }
                case SIGNAL_SHORT:
                case SIGNAL_USHORT:
                {
                    t64Data.tu16.u16_0 = va_arg(vaList, uint16_t);
                    bValidType = true;
                    break;
                }
                case SIGNAL_LONG:
                case SIGNAL_ULONG:
                {
                    t64Data.tu32.u32_0 = va_arg(vaList, uint32_t);
                    bValidType = true;
                    break;
                }
                case SIGNAL_FLOAT:
                {
                    t64Data.tf32.f32_0 = va_arg(vaList, float32_t);
                    bValidType = true;
                    break;
                }
                case SIGNAL_LONG64:
                case SIGNAL_ULONG64:
                {
                    t64Data.u64 = va_arg(vaList, uint64_t);
                    bValidType = true;
                    break;
                }
                case SIGNAL_DOUBLE:
                {
                    t64Data.f64 = va_arg(vaList, float64_t);
                    bValidType = true;
                    break;
                }
                case SIGNAL_OPAQUE:
                {
                    if(ptr_signal->sizeOfSignal <= 16U)
                    {
                        t64Data.tu16.u16_0 = va_arg(vaList, uint16_t);
                        bValidType = true;
                    }
                    else if((ptr_signal->sizeOfSignal > 16U) && (ptr_signal->sizeOfSignal <= 32U))
                    {
                        t64Data.tu32.u32_0 = va_arg(vaList, uint32_t);
                        bValidType = true;
                    }
                    else if(ptr_signal->sizeOfSignal <= 64U)
                    {
                        t64Data.u64 = va_arg(vaList, uint64_t);
                        bValidType = true;
                    }

                    break;
                }
                case SIGNAL_BOOL:
                {
                    t64Data.tu16.u16_0 = va_arg(vaList, uint16_t);
                    /* Don't allow any numbers other than 0 and 1 for bool */
                    if(t64Data.tu16.u16_0 > 0U)
                    {
                        t64Data.tu16.u16_0 = 1U;
                    }
                    bValidType = true;
                    break;
                }
                case SIGNAL_BCD:
                {
                    break;
                }
                case SIGNAL_UBNR:
                {
                    float32_t f32Temp;
                    uint64_t u64MaxValue;

                    f32Temp = va_arg(vaList, float32_t);    /* Signal Value input is a float32_t */

                    /* Scale and Round the Signal to BNR type */
                    f32Temp = A825Mgr_ScaleUpAndRound_BNR(f32Temp, ptr_signal->f32InverseResolution);

                    /* Calculate the maximum magnitude for positive numbers for the size of the signal */
                    u64MaxValue = ((1ULL << (uint64_t)ptr_signal->sizeOfSignal) - 1ULL);

                    t64Data.ts32.s32_0 = (int32_t)f32Temp;

                    /* Check if the Scaled up value fits within the size of the signal */
                    if(t64Data.u64 <= u64MaxValue)
                    {
                        bValidType = true; /* Value is valid and within limits, value is held in t64Data.u64 */
                    }

                    break;
                }
                case SIGNAL_BNR:
                {
                    float32_t f32Temp;
                    uint64_t u64MaxValue;
                    t64Types_t t64DataTemp = { 0 };

                    /* Max size for type BNR is 31 bits, otherwise no point and use different type */
                    if(ptr_signal->sizeOfSignal < 32U)
                    {
                        f32Temp = va_arg(vaList, float32_t);    /* Signal Value input is a float32_t */

                        /* Scale and Round the Signal to BNR type */
                        f32Temp = A825Mgr_ScaleUpAndRound_BNR(f32Temp, ptr_signal->f32InverseResolution);

                        /* Temporarily save off value */
                        t64DataTemp.s64 = (int64_t) f32Temp;    /* Cast to signed 64 to maintain sign bit scalability */
                        t64Data.u64 = t64DataTemp.u64;    /* Save off final value and check validity prior to using */

                        /* Calculate max limit and check within limit for negative and positive numbers respectively. */
                        if (f32Temp < 0.0F)
                        {
                            /* Calculate the maximum magnitude for negative numbers for the size of the signal */
                            u64MaxValue = (1ULL << ((uint64_t)ptr_signal->sizeOfSignal - 1ULL));

                            /* Get magnitude of the negative number to check if within scale of signal */
                            t64DataTemp.s64 = (0LL - t64DataTemp.s64);
                        }
                        else
                        {
                            /* Calculate the maximum magnitude for positive numbers for the size of the signal */
                            u64MaxValue = ((1ULL << ((uint64_t)ptr_signal->sizeOfSignal - 1ULL)) - 1ULL);
                        }

                        /* Check if the Scaled up value fits within the size of the signal */
                        if(t64DataTemp.u64 <= u64MaxValue)
                        {
                            bValidType = true; /* Value is valid and within limits, value is held in t64Data.u64 */
                        }
                    }

                    break;
                }
                default:
                {
                    break;
                }
            }
        }

        if(bValidType == true)
        {
            A825Mgr_SetSignalValueInPayload(ptr_payload, t64Data.u64, ptr_signal);
        }
    }

    va_end(vaList); /* End the Argument List */
}

/****************************************************************************************************
*  Function: A825Mgr_ScaleUpAndRound_BNR
*  Purpose: Scale the Floating Point number up to the inverse resolution specified and round to the
*  nearest integer for BNR / UBNR type signals
*  Global Inputs:
*  Global Outputs: None
*  Input:
*  const float32_t f32Value                         // Value to Scale
*  const float32_t f32InverseResolution             // Scaling Factor
*  Output: None
****************************************************************************************************/
float32_t A825Mgr_ScaleUpAndRound_BNR(const float32_t f32Value, const float32_t f32InverseResolution)
{
    float32_t f32ReturnValue;
    float32_t f32Diff;
    int32_t   s32RoundedDownValue;

    /* Scale the BNR value per the Inverse Resolution provided by the SIGNAL definition and input
     * into this function call. */
    f32ReturnValue = (f32Value * f32InverseResolution); /* Scale to the resolution specified */

    s32RoundedDownValue = (int32_t)f32ReturnValue;   /* Save to Signed integer which is rounded down */
    /* Calculate the magnitude of the difference used for rounding to nearest integer later */
    if (f32ReturnValue > (float32_t)s32RoundedDownValue)
    {
        /* Positive Values difference is the real minus the rounded down */
        f32Diff = f32ReturnValue - (float32_t)s32RoundedDownValue;
    }
    else
    {
        /* Negative Values difference is rounded down minus the real */
        f32Diff = (float32_t)s32RoundedDownValue - f32ReturnValue;
    }

    /* Round to the nearest whole integer number. */
    f32ReturnValue = (float32_t)s32RoundedDownValue;
    /* Round up if past half way between whole numbers */
    if (f32Diff > ROUND_UP_LIMIT)
    {
        /* Round up */
        f32ReturnValue = f32ReturnValue + 1.0F;
    }

    return f32ReturnValue;
}

/****************************************************************************************************
*  Function: A825Mgr_GetSignal_BNR
*  Purpose: Returns the lower 32 bit word of the payload corresponding to the specified message id.
*  Global Inputs: G_A825_Payload_Tbl_Msgs
*  Global Outputs:
*  Input: const uint16_t lookup_id     // A825 message ID
*         const SIGNAL_ENTRY_INDEX signal_index,       // Signal index
*         float_t *const ptr_val                       // Location of value to store payload data
*  Output: None
****************************************************************************************************/
void A825Mgr_GetSignal_BNR(A825_Bus_Rx_t* const ptA825Rx,
                           const uint16_t lookup_id,
                           const SIGNAL_ENTRY_INDEX signal_index,
                           float32_t *const ptr_val)
{
    uint64_t payload;
    const A825RxTableEntry *ptr_a825_message;
    const Signal_Info    *ptr_signal;
    uint32_t u32Val = 0UL;
    uint64_t u64Mask = 0ULL;

    //All receive messages have payload within 32 bit, so return the first thirty two bits
    //of the value.
    //Can be expanded later.
    if ((ptA825Rx != NULL) && (ptr_val != NULL))
    {
        if (lookup_id < ptA825Rx->u16NumMsgs)
        {
            ptr_a825_message = &ptA825Rx->ptMsgList[lookup_id];

            /*Justification for Coding Standard Deviation:
             *    pointer arithmetic is required for A825 signal message processing
             *    An exception to MISRA Rules 17.4 is required.
             */
            /*lint -e{960} # intended as is */
            ptr_signal = &ptr_a825_message->tMsg.ptr_signalInfoTable[(uint16_t)signal_index];
            payload  = ptr_a825_message->tData.u64;

            u64Mask = (1ULL << ptr_signal->sizeOfSignal) - 1ULL;
            u32Val = (uint32_t) ((payload >> ptr_signal->startPos) & u64Mask);
            //u32Val = (uint32_t)((payload>>ptr_signal->startPos) & (uint64_t)((1<<ptr_signal->sizeOfSignal)-1));

            /* Check if Signed or Unsigned BNR Type. */
            if(ptr_signal->signalType == SIGNAL_BNR)
            {
                /* Signed BNR has to do extra math to convert properly to float */
                /* Convert the signal to floating point value */
                *ptr_val = Utils_ConTwosCompBitToFloat(u32Val, ptr_signal->sizeOfSignal);
            }
            else
            {
                /* Unsigned BNR type simply casts the unsigned value over to float */
                *ptr_val = (float32_t)u32Val;
            }

            /* Scale the float according to the signal resolution */
            *ptr_val = (*ptr_val * ptr_signal->f32Resolution);
        }
    }
}

/****************************************************************************************************
*  Function: A825Mgr_GetSignal_BOOL
*  Purpose: Returns the status of the bit corresponding to the specified signal location.
*  Global Inputs: G_A825_Table_All_Messages, G_A825_Payload_Tbl_Msgs
*  Global Outputs:
*  Input:
*    const uint16_t lookup_id     //A825 message ID
*    const SIGNAL_ENTRY_INDEX signal_index        //Index of the signal in the Info Table
*    bool_e *const ptr_is_set                  //Pointer to signal set flag
*  Output: None
****************************************************************************************************/
void A825Mgr_GetSignal_BOOL(A825_Bus_Rx_t* const ptA825Rx,
                            const uint16_t lookup_id,
                            const SIGNAL_ENTRY_INDEX signal_index,
                            bool_t *const ptr_is_set)
{
    const A825RxTableEntry *ptr_a825_message;
    const Signal_Info    *ptr_signal;
    uint64_t             payload;
    uint64_t             mask = UNSIGNED_64BIT_MASK;
    uint64_t             start_position_mask;
    uint64_t             end_position_mask;


    if ((ptA825Rx != NULL) && (ptr_is_set != NULL))
    {
        if (lookup_id < ptA825Rx->u16NumMsgs)
        {
            ptr_a825_message = &ptA825Rx->ptMsgList[lookup_id];

            /*Justification for Coding Standard Deviation:
             *    pointer arithmetic is required for A825 signal message processing
             *    An exception to MISRA Rules 17.4 is required.
             */
            /*lint -e{960} # intended as is */
            ptr_signal = &ptr_a825_message->tMsg.ptr_signalInfoTable[(uint16_t)signal_index];
            payload    = ptr_a825_message->tData.u64;

            start_position_mask = ~(mask << ptr_signal->startPos);

            //If shifting by 64 bit, the compiler sets the mask to all F's. But the desired value is 0,
            //so set the mask to zero if shifting by 64 bits.

            if ((ptr_signal->startPos + ptr_signal->sizeOfSignal) < LONG_LONG_SIZE)
            {
                end_position_mask = (mask << (ptr_signal->startPos + ptr_signal->sizeOfSignal));
            }
            else
            {
                end_position_mask = 0U;
            }

            payload &= ~(start_position_mask | end_position_mask);

            if (payload > 0ULL)
            {
                *ptr_is_set = true;
            }
            else
            {
                *ptr_is_set = false;
            }
        }
    }

}

/****************************************************************************************************
*  Function: A825Mgr_GetSignal_OPAQUE
*  Purpose: Returns the value of the opaque signal corresponding to the specified signal location.
*  Global Inputs: G_A825_Table_All_Messages, G_A825_Payload_Tbl_Msgs
*  Global Outputs:
*  Input:
*    const uint16_t lookup_id     //A825 message ID
*    const SIGNAL_ENTRY_INDEX signal_index        //Index of the signal in the Info Table
*    bool_e *const pt64Val                  // Pointer to t64Types signal to set
*  Output: None
****************************************************************************************************/
void A825Mgr_GetSignal_OPAQUE(A825_Bus_Rx_t* const ptA825Rx,
                            const uint16_t lookup_id,
                            const SIGNAL_ENTRY_INDEX signal_index,
                            t64Types_t *const pt64Val)
{
    const A825RxTableEntry *ptr_a825_message;
    const Signal_Info    *ptr_signal;
    uint64_t             u64Signal;
    uint64_t             u64Mask = UNSIGNED_64BIT_MASK;

    if ((ptA825Rx != NULL) && (pt64Val != NULL))
    {
        if (lookup_id < ptA825Rx->u16NumMsgs)
        {
            ptr_a825_message = &ptA825Rx->ptMsgList[lookup_id];

            /*Justification for Coding Standard Deviation:
             *    pointer arithmetic is required for A825 signal message processing
             *    An exception to MISRA Rules 17.4 is required.
             */
            /*lint -e{960} # intended as is */
            ptr_signal = &ptr_a825_message->tMsg.ptr_signalInfoTable[(uint16_t)signal_index];
            u64Signal  = ptr_a825_message->tData.u64;

            /* Generate a mask the size of the signal */
            u64Mask = (1ULL << ptr_signal->sizeOfSignal) - 1ULL;
            /* Shift the signal to the zero position and save off only the signal itself by masking. */
            u64Signal = ((u64Signal >> ptr_signal->startPos) & u64Mask);
            /* Save the signal */
            pt64Val->u64 = u64Signal;
        }
    }
}

/*************************************************************************************************
Function: Utils_ConTwosCompBitToFloat
Purpose: This function converts a two's complement representation of a specified number of bits
to a floating point number.
Note:  Limited to 31 bits to get accurate results since 1 bit is always used for sign. Conversion
to float requires the additional bit for sign.

Global Inputs(s): None
Global Outputs(s): None

Inputs: uint32_t value_in_twos_comp, uint32_t number_of_bits

Output: float32_t, float representation of two's complement number.
**************************************************************************************************/
float32_t Utils_ConTwosCompBitToFloat(uint32_t value_in_twos_comp, uint32_t number_of_bits)
{
    float32_t ret_value = 0.0F;
    uint32_t value = value_in_twos_comp;
    uint32_t u32Mask = 0UL;
    uint32_t u32SignBit = 0UL;

    /* Set bit to look at sign */
    u32SignBit = (1UL << (number_of_bits - 1UL));

    /* If negative number */
    if((value_in_twos_comp & u32SignBit) > 0U)
    {
        /* Create the mask used for the float conversion */
        u32Mask = (1UL << (number_of_bits - 1U)) - 1;

        value = (~(value_in_twos_comp)) & u32Mask;
        value = value + 1U;
        ret_value = (float32_t)value;
        ret_value *= -1.0F;
    }
    else
    {
        ret_value = (float32_t)value_in_twos_comp;
    }
    return ret_value;
}
