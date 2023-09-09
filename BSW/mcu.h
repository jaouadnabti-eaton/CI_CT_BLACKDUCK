/****************************************************************************************************
*  File name: mcu.h
*
*  Purpose: Interface for the MCU state manager.
*  This file describes the public interface to the derived hierarchical state machine
*  for the MCU software.
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

#ifndef McuH__
#define McuH__

/*      Include Files
*/
#include "parameter.h"
#include "gse.h"
#include "hsm.h"
#include "panel.h"

/*      Local Function Prototypes
*/

/*
  MCU Mode Control
  
  brief The MCU Mode Control module implements an instance of an HSM, which handles transitions 
     from state to state as well as any operational tasks that each state must perform.  
     These are tasks that fall outside the normal actions that are handled by the scheduler.
     The Mode Control module is an event driven state machine that incorporates
     a single linked-list of all possible system states.  The list is iterated through
     until a handler is found that is associated with the event.  This handler can then 
     take any necessary action to resolve the event, including transitioning to another 
     state.  Shown below is the governing state chart for the system.
*/


#define POWER_DOWN_LIMIT        2
#define POWER_DOWN_THRESHOLD	2048 /* 1.5V */
#define POWER_DOWN_BOOTMODE		5

#define POWER_RECOVER_LIMIT		10
#define POWER_RECOVER_THRESHOLD	2321 /* 1.7V */

//#define MODE_CMD_ARINC825  (tPanel.bVcModeCmd) /* link mode command received over ARINC825*/
#define BUS_HL_MODE (false) /* define for HL Mode value bVcModeCmd=0 is HL Mode linked to bVcModeCmd received over ARINC825 */
#define BUS_VC_MODE (true) /* define for VL Mode value bVcModeCmd=1 is VL Mode linked to bVcModeCmd received over ARINC825 */

/*      Public Type Definitions
*/
/* DC BUS Mode state handling */
typedef enum
{
    BUS_IDLE_STATE, /**/
    HIGH_LIFT_STATE,
    VC_BUS_STATE,
    BUS_FAILURE_STATE
}DC_BUS_STATE;

/*DC BUS sub state handling*/
typedef enum
{
    SUBSTATE_HIGHLIFT_BUS_TRANSITION,
    SUBSTATE_HIGH_LIFT_EXECUTE,
    SUBSTATE_HIGH_LIFT_WAIT_STATE,
    SUBSTATE_VC_BUS_STATE_TRANSITION,
    SUBSTATE_VC_BUS_WAIT_STATE,
    SUBSTATE_VC_BUS_DISCHARGE_STATE,
    SUBSTATE_VC_BUS_EXECUTE
}DC_BUS_SUB_STATE;
/*      Public Variable Declarations
*/

extern DC_BUS_STATE CurrentBusState;
extern DC_BUS_SUB_STATE CurrentBusSubState;
extern STATE_ID G_eMcuMode;         /* The MCU state machine object*/
extern bool bInhibitFlap;            /* Whether or not the flap should be inhibited in the current state or mode*/
extern bool G_bChannelA;            /* Represents whether this MCU is for the primary side or not.  Set during Rig mode.*/

extern bool bCommandActionComplete;
extern bool bCommandActionResetFlg;
extern bool bFlapFail;
extern bool bFasNotAvailable;
extern bool bRigInProcess;
extern bool bGseConnected;
extern bool bFlapPosMiscompare;

extern bool bMcuFail;
extern bool bMcuWarning;
extern bool bScuFail;
extern bool bFlaFail;
extern bool bFpsuFail;
extern bool bMotorFail;
extern bool bFlapJam;

extern bool SystemStopFlg;
extern bool bRigModeFlg;

extern bool ENTER_CAL_MODE;

/*  brief Integer counter representing the total number of power cycles over unit lifetime
 */
extern Uint16 u16McuBootCount;
extern Timer_t t28VdcMonitorDelayTimer;
extern Timer_t tDcBusMonitorDelayTimer;

/*      Public ROM Constants
*/

/*      Public Defines
*/

/*      Public Interface Function Prototypes
*/
STATE_ID McuGetState(void);
void McuEntryEventHandler(STATE_ID);
STATE_ID McuInitModeHandler(Evt const *);
STATE_ID McuBootModeHandler(Evt const *);
STATE_ID McuRigModeHandler(Evt const *);
STATE_ID McuTestModeHandler(Evt const *);
STATE_ID McuIdleModeHandler(Evt const *);
STATE_ID McuRunModeHandler(Evt const *);
STATE_ID McuFasFailHandler(Evt const *);

/* ingroup scheduler*/
void RigModeScheduler(void);

/* ingroup scheduler*/
bool PBitScheduler(void);

/* ingroup scheduler*/
void TestModeScheduler(void);
void MCU_BusStateMachineProcessState(void);
void MCU_BusOutputInit(void);
void openNeutralSwitch(void);
void MCU_BusSetVcModeCmd(bool_t bVcModeCommand);
bool_t MCU_BusGetVcModeCmd(void);
#endif

/* end mcu.h*/
