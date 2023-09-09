/****************************************************************************************************
*  File name: nvm.c
*
*  Purpose: Defines NVM functionality for MCU
*
*  Copyright Notice:
*  All source code and data contained in this file is Proprietary and
*  Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
*  disclosed; in whole or in part, without the express written permission of
*  Eaton Aerospace.
*
*  Copyright (c) 2022 - Eaton Aerospace Group, All Rights Reserved.
*
 * Author            Date       CR#              Description
 * ------          ---------    ------        -------------------------------------
 * Adam Bouwens     12/07/2022  NA             Port to MCU
****************************************************************************************************/

/*      Include Files
*/
#include "nvm.h"
#include "motor.h"
//#include "panel.h"
#include "adc.h"
#include "gpio.h"
#include "mcu.h"

/*      Local Type Definitions
*/

/*      Local Defines
*/
#define EMIF_DISABLE_ACCESS_PROTECTION  (0x0U)     /* EMIF Disable Access Protection */
#define EMIF_LOCK_EMIF                  (0x1U)     /* EMIF access protection and master select fields lock bit */
#define EMIF_COMMIT_EMIF                (0x1U)     /* EMIF access protection and master select permanent lock */
#define EMIF_ASYNC_CR_SS_DISABLE        (0x0U)     /* EMIF Asynchronous Configuration Register, set for normal mode. Disable Select Strobe mode */
#define EMIF_ASYNC_CR_SS_ENABLE         (0x1U)
#define EMIF_ASYNC_CR_EW_DISABLE        (0x0U)     /* EMIF Asynchronous Configuration Register, Disable Extended Wait */

#if defined(_HSIT)
#define TEST_PASS 0xABCDABCD
#define TEST_FAIL 0xDEADDEAD
#define ASRAM_CS2_START_ADDR 0x100000
#define ASRAM_CS2_SIZE 0x40000
#endif

//#define NVM_CMD1_OFFSET         0x5555
//#define NVM_CMD2_OFFSET         0x2AAA
//
#define min(a,b)                ((a) < (b) ? (a) : (b))

/*      Global Variables
*/

/* Locate the NVM Data in MRAM */
#pragma DATA_SECTION (tNvm, "MRAM_NVM_DATA")
tNmvData_t tNvm;    /* Definition of the NVM Data that is stored as part of MRAM */

/* Locate the Fault Record Table in MRAM */
#pragma DATA_SECTION (tFault, "MRAM_FAULT_TBL")
tFaultData_t tFault;

/*      Local ROM Constants
*/

/* brief Array of 'zeros' for writing to the NVM during an 'erase' operation*/
const uint16_t u16Zeros[NVM_MAXWRITESIZE] =
{
     0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
     0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
     0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
     0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U
};

/*      Local Variable Declarations
*/

/* Default data for local mirror data structures */
const Nvm_State_t       Default_NVM_State = NVM_STATE_DEFAULTS;
const Uint32            Default_NVM_RunTimeCount = 0;

/****************************************/
/*  Local Mirror RAM copies of NVM Data */
/****************************************/
/* An instance of the State data structure to maintain State data from NVM in RAM*/
Nvm_State_t     Current_NVM_State = NVM_STATE_DEFAULTS;
/* An instance of the Current State Pointer to maintain it from NVM in RAM*/
Uint16          Current_NVM_State_Pointer = 0;
/* An instance of the Run Time Counter to maintain it data from NVM in RAM*/
Uint32          Current_NVM_RunTimeCount = 0;

Nvm_Rigging_t Nvm_Rigging_Temp = { 0 };  /* Working copy. Save to NVM [tNvm] when rigging complete */

Nvm_State_t Nvm_State = NVM_STATE_DEFAULTS;         /* Copy of NVM on startup. */
Nvm_State_t Nvm_State_Temp = NVM_STATE_DEFAULTS;    /* Working copy. Save to Nvm_State when written to NVM [tNvm] */

Nvm_State_t Nvm_Powerdown_State = NVM_STATE_DEFAULTS;

Uint16 BoardSerialNumber = 0;

bool NVM_ClearInProgress = false;
bool NVM_FormatComplete = false;    /* Used in Test Mode */
bool NVM_StateInfoCleared = false;  /* Used in Test Mode */
bool NVM_RigInfoCleared = false;    /* Used in Test Mode */
bool NVM_AllDataSent = false;       /* Used in Test Mode */
bool Nvm_PowerDown = false;

#if defined(_HSIT)
//#pragma DATA_SECTION(u16NvmBuffer, "MRAM")
uint16_t errCountGlobal = 0U;
uint32_t testStatusGlobal;
uint32_t i;
//#pragma SET_DATA_SECTION("MRAM_S1")
uint16_t u16NvmBuffer[1024];
//#pragma SET_DATA_SECTION()
#endif

/*      Local Function Prototypes
*/
void Nvm_InitMram(void);

#if defined(_HSIT)
void Nvm_TestMram(void);
uint16_t readWriteMem(uint32_t startAddr, uint32_t memSize);
uint16_t walkMemData(uint32_t startAddr, uint32_t memSize);
uint16_t walkMemAddr(uint32_t startAddr, uint32_t addrSize);
uint16_t accessMemData(uint32_t startAddr, uint32_t sizeToCheck);
#endif

/*      Function Definitions
*/

/****************************************************************************************************
*  Function: Nvm_InitMram
*  Purpose: Initialize the MRAM Peripheral that is used for NVM.
*
*  Global Inputs:
*  Global Outputs:
*
*  Input: None
*  Output: None
****************************************************************************************************/
void Nvm_InitMram(void)
{

    EALLOW;

    /* Set Clock Frequency for EMIF1CLK */
    ClkCfgRegs.PERCLKDIVSEL.bit.EMIF1CLKDIV = 0U;    /* Set EMIF1CLK to run at full speed */

    /* Set CPU1 as Master for MRAM connected on EMIF1 */
    Emif1ConfigRegs.EMIF1MSEL.all = MSEL_EMIF1_CPU1;
    /* Disable Access Protection. (CPU_FETCH/CPU_WR/DMA_WR) */
    Emif1ConfigRegs.EMIF1ACCPROT0.all = EMIF_DISABLE_ACCESS_PROTECTION;
    /* Commit the configuration related to protection. Till this bit remains set, contents of EMIF1ACCPROT0 register can't be changed.*/
    Emif1ConfigRegs.EMIF1COMMIT.bit.COMMIT_EMIF1 = EMIF_COMMIT_EMIF;
    /* Lock the EMIF1 Configuration so that EMIF1COMMIT cannot be changed anymore. */
    Emif1ConfigRegs.EMIF1LOCK.bit.LOCK_EMIF1 = EMIF_LOCK_EMIF;


    /************************************************************************************/
    /* Setup EMIF Chip Select, Write Enable, and Output Enable Pins */
    /************************************************************************************/
    /* Setup GPIO_34 as EM1CS2 */
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_31 as EM1WE */
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_37 as EM1OE */
    GpioCtrlRegs.GPBGMUX1.bit.GPIO37 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPBMUX1.bit.GPIO37 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */

    /************************************************************************************/
    /* Setup EMIF Address Pins */
    /************************************************************************************/
    /* Setup GPIO_92 as EM1BA1 for MRAM A0 */
    GpioCtrlRegs.GPCGMUX2.bit.GPIO92 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX2.bit.GPIO92 = GPIO_MUX_TYPE_3;      /* Set as EMIF pin type */
    /* Setup GPIO_38 as EM1A0 for MRAM A1 */
    GpioCtrlRegs.GPBGMUX1.bit.GPIO38 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPBMUX1.bit.GPIO38 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_39 as EM1A1 for MRAM A2 */
    GpioCtrlRegs.GPBGMUX1.bit.GPIO39 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_40 as EM1A2 for MRAM A3 */
    GpioCtrlRegs.GPBGMUX1.bit.GPIO40 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_41 as EM1A3 for MRAM A4 */
    GpioCtrlRegs.GPBGMUX1.bit.GPIO41 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_44 as EM1A4 for MRAM A5 */
    GpioCtrlRegs.GPBGMUX1.bit.GPIO44 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPBMUX1.bit.GPIO44 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_45 as EM1A5 for MRAM A6 */
    GpioCtrlRegs.GPBGMUX1.bit.GPIO45 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPBMUX1.bit.GPIO45 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_46 as EM1A6 for MRAM A7 */
    GpioCtrlRegs.GPBGMUX1.bit.GPIO46 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPBMUX1.bit.GPIO46 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_47 as EM1A7 for MRAM A8 */
    GpioCtrlRegs.GPBGMUX1.bit.GPIO47 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPBMUX1.bit.GPIO47 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_48 as EM1A8 for MRAM A9 */
    GpioCtrlRegs.GPBGMUX2.bit.GPIO48 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPBMUX2.bit.GPIO48 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_49 as EM1A9 for MRAM A10 */
    GpioCtrlRegs.GPBGMUX2.bit.GPIO49 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPBMUX2.bit.GPIO49 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_50 as EM1A10 for MRAM A11 */
    /* commented below GPIO as it used for DC BUS switching, to be replaced with as per HW new schematic*/
    GpioCtrlRegs.GPBGMUX2.bit.GPIO50 = GPIO_MUX_GRP_0;     /*  Select group 0       */
    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = GPIO_MUX_TYPE_2;     /*  Set as EMIF pin type */
    /* Setup GPIO_51 as EM1A11 for MRAM A12 */
    GpioCtrlRegs.GPBGMUX2.bit.GPIO51 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_52 as EM1A12 for MRAM A13 */
    GpioCtrlRegs.GPBGMUX2.bit.GPIO52 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_86 as EM1A13 for MRAM A14 */
    GpioCtrlRegs.GPCGMUX2.bit.GPIO86 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX2.bit.GPIO86 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_87 as EM1A14 for MRAM A15 */
    GpioCtrlRegs.GPCGMUX2.bit.GPIO87 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX2.bit.GPIO87 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_88 as EM1A15 for MRAM A16 */
    GpioCtrlRegs.GPCGMUX2.bit.GPIO88 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX2.bit.GPIO88 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    /* Setup GPIO_89 as EM1A16 for MRAM A17 */
    GpioCtrlRegs.GPCGMUX2.bit.GPIO89 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX2.bit.GPIO89 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */


    /************************************************************************************/
    /* Setup EMIF Data Pins */
    /************************************************************************************/
    /* Setup GPIO_85 as EM1D0 for MRAM DQL0 */
    GpioCtrlRegs.GPCGMUX2.bit.GPIO85 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX2.bit.GPIO85 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL2.bit.GPIO85 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO85 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_83 as EM1D1 for MRAM DQL1 */
    GpioCtrlRegs.GPCGMUX2.bit.GPIO83 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX2.bit.GPIO83 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL2.bit.GPIO83 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO83 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_82 as EM1D2 for MRAM DQL2 */
    GpioCtrlRegs.GPCGMUX2.bit.GPIO82 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX2.bit.GPIO82 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL2.bit.GPIO82 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO82 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_81 as EM1D3 for MRAM DQL3 */
    GpioCtrlRegs.GPCGMUX2.bit.GPIO81 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX2.bit.GPIO81 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL2.bit.GPIO81 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO81 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_80 as EM1D4 for MRAM DQL4 */
    GpioCtrlRegs.GPCGMUX2.bit.GPIO80 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX2.bit.GPIO80 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL2.bit.GPIO80 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO80 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_79 as EM1D5 for MRAM DQL5 */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO79 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX1.bit.GPIO79 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO79 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO79 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_78 as EM1D6 for MRAM DQL6 */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO78 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX1.bit.GPIO78 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO78 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO78 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_77 as EM1D7 for MRAM DQL7 */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO77 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX1.bit.GPIO77 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO77 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO77 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_76 as EM1D8 for MRAM DQL8 */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO76 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX1.bit.GPIO76 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO76 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO76 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_75 as EM1D9 for MRAM DQL9 */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO75 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX1.bit.GPIO75 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO75 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO75 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_74 as EM1D10 for MRAM DQL10 */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO74 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX1.bit.GPIO74 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO74 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO74 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_73 as EM1D11 for MRAM DQL11 */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO73 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX1.bit.GPIO73 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO73 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO73 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_72 as EM1D12 for MRAM DQL12 */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO72 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX1.bit.GPIO72 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO72 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO72 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_71 as EM1D13 for MRAM DQL13 */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO71 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX1.bit.GPIO71 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO71 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO71 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_70 as EM1D14 for MRAM DQL14 */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO70 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX1.bit.GPIO70 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO70 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO70 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */
    /* Setup GPIO_69 as EM1D15 for MRAM DQL15 */
    GpioCtrlRegs.GPCGMUX1.bit.GPIO69 = GPIO_MUX_GRP_0;      /* Select group 0       */
    GpioCtrlRegs.GPCMUX1.bit.GPIO69 = GPIO_MUX_TYPE_2;      /* Set as EMIF pin type */
    GpioCtrlRegs.GPCQSEL1.bit.GPIO69 = GPIO_ASYNC;          /* Set as asynchronous type */
    GpioCtrlRegs.GPCPUD.bit.GPIO69 = GPIO_ENABLE_PULLUP;    /* Enable pull-up resistor */


    /************************************************************************************/
    /* Setup EMIF Asynchronous Communication Configuration */
    /************************************************************************************/
    /* Set to normal mode rather than strobe select. CS line is not strobed, just an enable */
    Emif1Regs.ASYNC_CS2_CR.bit.SS = EMIF_ASYNC_CR_SS_DISABLE;
    /* Disable Extended Wait Mode */
    Emif1Regs.ASYNC_CS2_CR.bit.EW = EMIF_ASYNC_CR_EW_DISABLE;
    /* Set MRAM interface on EMIF1 to 16-bit */
    Emif1Regs.ASYNC_CS2_CR.bit.ASIZE = EMIF_ASYNC_ASIZE_16;

    /* Set Asynchronous Memory Access Timing Characteristics */
    Emif1Regs.ASYNC_CS2_CR.bit.R_SETUP = 0;         /* Set Read Setup Time:    0ns   */
    Emif1Regs.ASYNC_CS2_CR.bit.R_STROBE = 7;        /* Set Read Strobe Time:  40ns   */
    Emif1Regs.ASYNC_CS2_CR.bit.R_HOLD = 1;          /* Set Read Hold Time:    10ns   */
    Emif1Regs.ASYNC_CS2_CR.bit.W_SETUP = 0;         /* Set Write Setup Time:   0ns   */
    Emif1Regs.ASYNC_CS2_CR.bit.W_STROBE = 7;        /* Set Write Strobe Time: 40ns   */
    Emif1Regs.ASYNC_CS2_CR.bit.W_HOLD = 0;          /* Set Write Hold Time:    0ns   */
    Emif1Regs.ASYNC_CS2_CR.bit.TA = 2;              /* Set Turn-Around Time:  15ns   */

    EDIS;

    /* Run the MRAM Check Monitor */
    Bit_CriticalMonitors(0x7C, true);

#if defined(_HSIT)
    Nvm_TestMram();
#endif

    return;
}

#if defined(_HSIT)
void Nvm_TestMram(void)
{
    uint16_t errCountLocal;

    DisableDog();

    //
    // Checks basic RD/WR access to CS2 space.
    //
    errCountLocal = readWriteMem(ASRAM_CS2_START_ADDR, ASRAM_CS2_SIZE/2);
    errCountGlobal = errCountGlobal + errCountLocal;

    //
    // Address walk checks. (Tested for Memory with address width of 16bit)
    //
    errCountLocal = walkMemAddr(ASRAM_CS2_START_ADDR, 16);
    errCountGlobal = errCountGlobal + errCountLocal;

    //
    // Data walk checks.
    //
    errCountLocal = walkMemData(ASRAM_CS2_START_ADDR, ASRAM_CS2_SIZE/2);
    errCountGlobal = errCountGlobal + errCountLocal;

    //
    // Data size checks.
    //
    errCountLocal = accessMemData(ASRAM_CS2_START_ADDR, 4);
    errCountGlobal = errCountGlobal + errCountLocal;

    if(errCountGlobal == 0x0)
    {
        testStatusGlobal = TEST_PASS;
    }

    if(testStatusGlobal != TEST_PASS)
    {
        while (1);
    }

    EnableDog();
    return;
}

//
// Read Write Memory - This function performs simple read/write word accesses
// to memory.
//
uint16_t readWriteMem(uint32_t startAddr, uint32_t memSize)
{
    uint32_t memReadData;
    uint32_t memWriteData;
    uint32_t *memPtr;
    uint32_t i;

    memPtr = (uint32_t *)startAddr;
    //
    // Write data to memory.
    //
    memWriteData = 0x01234567;
    for(i = 0; i < memSize; i++)
    {
        *memPtr++ = memWriteData;
        memWriteData += 0x11111111;
    }

    //
    // Verify data written to memory.
    //
    memWriteData = 0x01234567;
    memPtr = (uint32_t *)startAddr;
    for(i = 0; i < memSize; i++)
    {
        memReadData = *memPtr;
        if(memReadData != memWriteData)
        {
            return(1);
        }
        memPtr++;
        memWriteData += 0x11111111;
    }
    return(0);
}

//uint16_t readWriteMem(uint32_t startAddr, uint32_t memSize)
//{
//    uint16_t memReadData;
//    uint16_t memWriteData;
//    uint16_t *memPtr;
//    uint32_t i;
//
//    memPtr = (uint16_t *)startAddr;
//
//    memSize = 0x40000;
//    //
//    // Write data to memory.
//    //
//    memWriteData = 0x0123;
//    for(i = 0; i < memSize; i++)
//    {
//        *memPtr++ = memWriteData;
//        memWriteData += 0x1111;
//    }
//
//    //
//    // Verify data written to memory.
//    //
//    memWriteData = 0x0123;
//    memPtr = (uint16_t *)startAddr;
//    for(i = 0; i < memSize; i++)
//    {
//        memReadData = *memPtr;
//        if(memReadData != memWriteData)
//        {
//            //return(1);
//            u32ErrorCnt++;
//        }
//        memPtr++;
//        memWriteData += 0x1111;
//    }
//    return(0);
//}

//
// Walk Memory Data - This function performs a walking 0 & 1 on data lines
// for SRAM RD & WR.
//
uint16_t walkMemData(uint32_t startAddr, uint32_t memSize)
{
    uint32_t sramReadData;
    uint32_t sramWriteData;
    uint32_t i;
    uint32_t k;
    uint32_t m;
    uint32_t *memPtr;
    uint32_t *memPtrIter;

    memPtr = (uint32_t *)startAddr;

    for(i = 0; i < memSize; i = i + 64)
    {
        for(m = 0; m < 2; m++)
        {
            //
            // Write loop.
            //
            memPtrIter = memPtr;
            sramWriteData = 0x01;
            for(k = 0; k < 32; k++)
            {
                if(m == 0)
                {
                    *memPtrIter++ = sramWriteData;
                }
                else
                {
                    *memPtrIter++ = ~sramWriteData;
                }
                sramWriteData = sramWriteData << 1;
            }

            //
            // Read loop.
            //
            memPtrIter = memPtr;
            sramWriteData = 0x01;
            for(k = 0; k < 32; k++)
            {
                sramReadData = *memPtrIter;
                if(m == 1)
                {
                    sramReadData = ~sramReadData;
                }
                if(sramReadData != sramWriteData)
                {
                    return(1);
                }
                memPtrIter++;
                sramWriteData = sramWriteData << 1;
            }
        }
        memPtr = memPtrIter;
    }
    return(0);
}

//
// Walk Memory Addresses - This function performs a toggle on each address bit.
//
uint16_t walkMemAddr(uint32_t startAddr, uint32_t addrSize)
{
    uint32_t sramReadData;
    uint32_t sramWriteData;
    uint32_t k;
    uint32_t xshift;
    uint32_t *memPtr;
    uint32_t *memPtrIter;

    memPtr = (uint32_t *)startAddr;

    //
    // Write loop.
    //
    xshift = 0x00000001;
    sramWriteData = 0x00;
    for(k = 0; k < addrSize; k++)
    {
        memPtrIter = (uint32_t *)(startAddr + xshift);
        *memPtrIter = sramWriteData++;
        xshift = xshift << 1;
    }

    //
    // Read loop.
    //
    memPtrIter = memPtr;
    xshift = 0x00000001;
    sramWriteData = 0x00;
    for(k = 0; k < addrSize; k++)
    {
        memPtrIter = (uint32_t *)(startAddr + xshift);
        sramReadData = *memPtrIter;
        if(sramReadData != sramWriteData)
        {
            return(1);
        }
        xshift = xshift << 1;
        sramWriteData++;
    }
    return(0);
}

//
// Access Memory Data - This function performs different data type
// (HALFWORD/WORD) access.
//
uint16_t accessMemData(uint32_t startAddr, uint32_t sizeToCheck)
{
    uint16_t memRdShort;
    uint32_t memRdLong;
    uint16_t memWrShort;
    uint32_t memWrLong;
    uint32_t i;
    uint16_t *memPtrShort;
    uint32_t *memPtrLong;

    //
    // Write short data.
    //
    memPtrShort = (uint16_t *)startAddr;
    memWrShort = 0x0605;

    for(i = 0; i < 2; i++)
    {
        *memPtrShort++ = memWrShort;
        memWrShort += 0x0202;
    }

    //
    // Write long data.
    //
    memPtrLong = (uint32_t *)memPtrShort;
    memWrLong = 0x0C0B0A09;
    for(i = 0; i < 2; i++)
    {
        *memPtrLong++ = memWrLong;
        memWrLong += 0x04040404;
    }

    //
    // Read short data.
    //
    memPtrShort = (uint16_t *)startAddr;
    memWrShort = 0x0605;
    for(i = 0; i < 6; i++)
    {
        memRdShort = *memPtrShort;
        if(memRdShort != memWrShort)
        {
            return(1);
        }
        memPtrShort++;
        memWrShort += 0x0202;
    }

    //
    // Read long data.
    //
    memPtrLong = (uint32_t *)startAddr;
    memWrLong = 0x08070605;
    for(i = 0; i < 3; i++)
    {
        memRdLong = *memPtrLong;
        if(memRdLong != memWrLong)
        {
            return(1);
        }
        memPtrLong++;
        memWrLong += 0x04040404;
    }
    return(0);
}
#endif

/*
    brief Calls functions to initialize the local RAM Mirrors of the stored RAM data.

     Purpose:
        Calls functions to initialize the local RAM Mirrors of the stored RAM data.

    return  void

     Preconditions and Assumptions:
        none

*/
void Nvm_Init(void)
{
    /*  PATH(Nvm_Init,A); */

#if !defined(DRV8312_DEV_KIT)
    /* MCU HW:  MRAM only available on MCU hardware */
    Nvm_InitMram();
#endif


    /*shall call functions to initialize the local RAM mirror versions of State,
    Run Time Counter and DSP Health state Data from NVM*/
    Nvm_InitReadState();

    /* shall retrieve rigging information from NVM */
    Nvm_ReadRigging(&Nvm_Rigging_Temp);

    /* shall retrieve status information from NVM, including last position */
    Nvm_ReadState(&Nvm_State);

    /* Overwrite the critical position data with the powerdown data since there are
     * no stay-up caps to get the POWER_DOWN State stored into MRAM using
     * the legacy VLJ interface and operations. Rework how the powerdown state records
     * are stored in NVM at some point. */
    Nvm_State.quadposition = tNvm.tData.Nvm_Powerdown_State.quadposition;
    Nvm_State.tPanel = tNvm.tData.Nvm_Powerdown_State.tPanel;

    /* Initialize the Sensor Fusion Position Estimate with the last known Skew Sensor Based Stroke */
    G_ASW_DATA.tPanelOutputData.f32StrokeFused = Nvm_State.tPanel.tSkewSnsrCalcs.tStroke.f32Stroke;

    /* Need to mask off the upper byte that contains the Left/Right side information */
    Nvm_State.Mode &= 0x00FF;

    /* shall retrieve run time count from NVM */
    Nvm_State.RunTimeCount = tNvm.tData.Nvm_RunTimeCount;

    /*  PATH(Nvm_Init,B); */
}

/*
    brief Reads the most recently stored State information from NVM
       into the local NVM RAM mirror

     Purpose:
        Reads the most recently stored State information from NVM into the
        Current State local RAM mirror.

    return  void

     Preconditions and Assumptions:
        none

*/
void Nvm_InitReadState( void )
{
    /*  PATH(Nvm_InitReadState,A); */

    /* shall Read the most recent State Pointer into the Local RAM mirror*/
    Nvm_InitReadCurrentStatePointer();

    /* shall Read the Current state from NVM into the local RAM mirror*/
    Nvm_GetState(Current_NVM_State_Pointer, &Current_NVM_State);
    /*  PATH(Nvm_InitReadState,B); */
}

/*
    brief Initial read of Current State Pointer from NVM into local RAM mirror

    Purpose:
        Local copy of NVM_State_Pointer

    return  void

    Preconditions and Assumptions:
        none

*/
void Nvm_InitReadCurrentStatePointer( void )
{
    /* shall set the local RAM mirror variable the non-zero pointer value (or zero
      if they were all zero).*/
    Current_NVM_State_Pointer = tNvm.tData.Nvm_State_Pointer;
}


/*
    brief Retrieves Current State Pointer value from local RAM copy

    Purpose:
        Returns the local RAM copy of the Current State Pointer value for use by external
        components to read state records from NVM at the most recent location.

    return  Uint16      Value of most recently-used index into State partiiton of NVM.

    Preconditions and Assumptions:
        None.

*/
Uint16 Nvm_GetCurrentStatePointer( void )
{
    /* PATH(Nvm_GetCurrentStatePointer,A); */
    return (Current_NVM_State_Pointer);
}

/*
    brief Clears the entire NVM Memory Space, one block at a time.

    Purpose:
        Zeros out all of NVM memory by writing zeros, one NVM page at a time.  Page written
        is given by the addr argument.  After a page-write is successful, the return
        value gives the address of the next page of NVM to be erased.  Completion is
        signified by a return value of 0xffffffff.

    param[in] addr      Address of page in NVM to be erased

    return  Uint32  The address argument to use in the next call to Nvm_Format().
                    Returns '0xFFFFFFFF' when all NVM has been erased.

    Preconditions and Assumptions:
        This routine is to be called only during maintenance operations.  This routine is
        designed for invocation in the test-mode scheduler, using the return value of a
        previous call to Nvm_Format() as an argument.

*/
Uint32 Nvm_Format( Uint32 addr )
{
    bool success = false;
    Uint32 retval;

    /* PATH(Nvm_Format,A); */

    /* shall write all zeros to the page (128 16-bit words) given by the addr argument*/
    success = Nvm_Erase( addr, NVM_MAXWRITESIZE );

    /* shall return address of next page to be erased if write was successful, and
       return given argument otherwise*/
    if (success == true)
    {
        /* PATH(Nvm_Format,B); */
        retval = addr + NVM_MAXWRITESIZE;

        /* shall return '0xffffffff' to signify completion if 'next page address' is
           outside NVM range*/
        if (retval >= NVM_SIZE)
        {
            /* PATH(Nvm_Format,C); */
            retval = 0xFFFFFFFF;
            Current_NVM_State_Pointer = 0;
			Nvm_State = Default_NVM_State;
			u16McuBootCount = 1;
			Nvm_State.bootcount = 1;
        }
    }
    else
    {
        /* PATH(Nvm_Format,D); */
        retval = addr;
    }

    /* PATH(Nvm_Format,E); */
    return retval;
}


/*
    brief Reads rigging data from the Rigging partition in NVM.

    Purpose:
        Reads rigging data from the local mirrored copy of NVM and 
        fills given Rigging data structure.

    param[in]  rig   Pointer to rigging data structure to be written to from NVM
    
    Preconditions and Assumptions:
         NVM Init must have been called to fill in the local mirror copy

*/
void Nvm_ReadRigging(Nvm_Rigging_t *rig)
{
    /* PATH(Nvm_ReadRigging,A); */
	
	/* Check if rig parameter is not a null pointer */
    if (rig)
    {
        /* PATH(Nvm_ReadRigging,B); */

        /* shall return the local mirrored copy of the rigging data*/
        *rig = tNvm.tData.Nvm_Rigging;
    }

    /* PATH(Nvm_ReadRigging,C); */
}


/*
    brief Writes rigging data to the rigging partition in NVM.

    Purpose:
        Writes given data to the rigging partition of the NVM,

    param[in]  rig   Pointer to rigging data structure to be written to NVM

    retval true     Successfully wrote rigging data to NVM
    retval false    Rigging data not written

    Preconditions and Assumptions:
        None.

    Non-Compliance: This function exceeds 3 nesting levels.
    Justification: The algorithm is clear enough that this amount of nesting is not excessively complex.

*/
bool_t Nvm_WriteRigging( Nvm_Rigging_t *rig )
{
    bool_t result = false;
    static uint16_t index = 0;
    static uint16_t CRC16 = 0;
    uint16_t* pu16Src = (uint16_t*) rig;
    uint16_t* pu16Dst = (uint16_t*) &tNvm.tData.Nvm_Rigging;


    if((pu16Src != NULL) && (pu16Dst != NULL))
    {
        /* compute a portion of the Rigging CRC each call until the full structure has
         * gone through the CRC algorithm. */
        if (index < (NVM_RIGSIZE - 1))
        {
            /* Setup the source and destination pointers */
            pu16Src += index;
            pu16Dst += index;
            /* Copy the Rig data word to NVM */
            *pu16Dst = *pu16Src;

            /* Update the CRC based on the word just written to NVM */
            CRC16 = (CRC16 ^ (*pu16Dst));

            if (CRC16 & 0x0001)
            {
                CRC16 = (CRC16 >> 1);
                CRC16 = (CRC16 ^ BIT_RIGTEST_CRC_CODE);
            }
            else
            {
                CRC16 = (CRC16 >> 1);
            }
            /* increment index after CRC computation is done on the byte */
            index++;
        }
        else
        {
            /* CRC calculation is complete. Write the Rigging data to NVM along with the CRC */
            tNvm.tData.Nvm_Rigging.crc = CRC16;

            /* Reset the data used to calculate the CRC */
            CRC16 = 0;
            index = 0;
            result = true;
        }
    }

    return (result);
}


/*
    brief Restores the Rigging portion of NVM to its defaults, one page at a time.

    Purpose:
        Zeros out the rigging partition of NVM

    param[in] void

    return  false clear in process
    return  true rig structure clear complete

    Preconditions and Assumptions:
        This routine is to be called only during maintenance operations and on a cyclic
        basis only when the clear is to take place.
*/
bool_t Nvm_ClearRigging( void )
{
    bool_t result = false;
    static uint16_t index = 0;
    uint16_t u16WordsToClear = NVM_RIG_WORDS_TO_CLEAR_PER_CYCLE;
    uint16_t* pu16Dst = (uint16_t*) &tNvm.tData.Nvm_Rigging;

    if(pu16Dst != NULL)
    {
        /* Clear Rigging Structure in chunks of NVM_RIG_WORDS_TO_CLEAR_PER_CYCLE */
        while (u16WordsToClear > 0U)
        {
            u16WordsToClear--;

            /* compute a portion of the Rigging CRC each call until the full structure has
             * gone through the CRC algorithm. */
            if (index < NVM_RIGSIZE)
            {
                /* Setup the source and destination pointers */
                pu16Dst += index;
                /* Copy the Rig data word to NVM */
                *pu16Dst = 0;

                /* increment index after CRC computation is done on the byte */
                index++;
            }
            else
            {
                /* Clear NVM process is complete. Reset the data for next clear if necessary. */
                index = 0;
                u16WordsToClear = 0U;
                result = true;
            }
        }
    }

    return (result);
}


/*
    Purpose: This function reads the Fault/State Record from NVM at the given index

    param[in]  index  Specifies the index of the state you'd like to get
    param[out] state  Pointer to State data structure to be filled in with specified State

    Global Variables Referenced:
       	#NVM_STATEBASE

    return
    	none

    Preconditions and Assumptions:
        none

*/
void Nvm_GetState( Uint16 index, Nvm_State_t *state )
{

    /* PATH(Nvm_GetState,A); */

	if (state)
	{
	    /* index should be Current_NVM_State_Pointer at this point, legacy.
	     *  Should check index for array bounds, but don't need this with MRAM,
	     *  just keeping interface the same to get working and then can eliminate
	     *  all unnecessary function calls and intermediate work and rather access
	     *  MRAM variables directly */
	    if(index < NUMBER_OF_FAULT_RECORDS)
	    {
	        *state = tFault.tRecord[index].Nvm_State;
	    }
	}

    /* PATH(Nvm_GetState,C); */
}


/*
    brief Reads current State data from the local NVM State mirror.

    Purpose:
        Reads rigging data from the local mirrored copy of NVM and
        fills given Rigging data structure.

    param[in]  state   Pointer to state data structure to be written to from NVM

    Preconditions and Assumptions:
        NVM Init must have been called to fill in the local mirror copy

*/
void Nvm_ReadState( Nvm_State_t *state )
{
    /* PATH(Nvm_ReadState,A); */

	if (state)
	{
		/* PATH(Nvm_ReadState,C); */

		/* shall return the local mirrored copy of the State data*/
	    *state = Current_NVM_State;
	}

	/* PATH(Nvm_ReadState,B); */
}


/*
    brief Writes State data to a rotating State buffer in NVM.

    Purpose:
        Writes given data to the "next" State address in the State partition of the NVM.

    param[in]  state   Pointer to state data structure to be written to NVM

    retval true     Successfully wrote state data to EEPROM's page buffer.
    retval false    State data not written; EEPROM was busy in a write cycle.

    Preconditions and Assumptions:
        None.

*/
bool Nvm_WriteState( Nvm_State_t *state )
{
    bool result = false;

    /* PATH(Nvm_WriteState,A); */

    if (state)
    {
    	/* PATH(Nvm_WriteState,H); */

        Current_NVM_State_Pointer++;

        /* shall wrap the pointer back to the beginning of the Buffer if at the end*/
        if (Current_NVM_State_Pointer >= MAX_STATE_RECORDS)
        {
            /* PATH(Nvm_WriteState,D); */
            Current_NVM_State_Pointer = 0;
        }

        /* shall latch the passed in state data to local mirrored copy */
        Current_NVM_State = *state;

        /* shall write the new State Pointer into NVM */
        tNvm.tData.Nvm_State_Pointer = Current_NVM_State_Pointer;
        /* Save the new State Record to the Fault Table */
        tFault.tRecord[Current_NVM_State_Pointer].Nvm_State = Current_NVM_State;

        result = true;
	}

    /* PATH(Nvm_WriteState,G); */
    return (result);
} /* end of Nvm_WriteState */


///*
//
//    Purpose:
//        This function checks whether there is a write command pending to commit
//        a State record to the NVM.
//
//    param[in]
//         none
//
//
//    Global Variables Referenced:
//       	State_Write_Pending
//
//    return
//    	true if a State record Write command is pending; false otherwise
//
//*/
//bool Nvm_StateWritePending( void )
//{
//    /* PATH(Nvm_StateWritePending,A); */
//
//    return (State_Write_Pending);
//}

// WILL NEED THIS LATER:  FUNCTION DELETES ALL FAULT RECORDS AND CURRENT STATE POINTER AND
// RUNTIMECOUNT
///*
//    brief Clears the entire State partition of NVM, one block at a time.
//
//    Purpose:
//        Writes zeros into the State partition of NVM, one NVM page at a time.  Page written
//        is given by the  addr argument, where  addr=0 signals the function to first
//        clear the StatePointer.  After a page-write is successful, the return
//        value gives the address of the next page of NVM to be erased.  Completion is
//        signified by a return value of 0xffffffff.
//
//    param[in] addr      Address of page in the State Partition to be erased
//
//    return  Uint32  The address argument to use in the next call to Nvm_ClearState().
//                    Returns '0xFFFFFFFF' when all of the State Partition has been erased.
//
//    Preconditions and Assumptions:
//        This routine is to be called only during maintenance operations.  This routine is
//        designed for invocation in the test-mode scheduler, using the return value of a
//        previous call to Nvm_ClearState() as an argument.  The erase procedure begins
//        by invoking the function with an argument of zero, to clear the StatePointer.
//
//*/
//Uint32 Nvm_ClearState( Uint32 addr )
//{
//    bool success = false;
//    Uint32 retval;
//
//    /* PATH(Nvm_Clear_State,A); */
//
//    if (addr == 0)
//    {
//        /* PATH(Nvm_Clear_State,B); */
//
//        /* shall set the local Mirror RAM copy of the NVM State Pointer to zero if 'addr' argument is zero*/
//        Current_NVM_State_Pointer = 0;
//        Current_NVM_State_Pointer_Index = ((NVM_CURRENTPOINTERSIZE - 1) << 1);
//
//        /* shall mirror the cleared NVM State Pointer from RAM to NVM if 'addr' argument is zero*/
//        success = Nvm_WriteCurrentStatePointer(Current_NVM_State_Pointer);
//
//        if (success == true)
//        {
//            /* PATH(Nvm_Clear_State,C); */
//            addr = NVM_STATEBASE - NVM_MAXWRITESIZE;
//        }
//    }
//    else
//    {
//        /* PATH(Nvm_Clear_State,D); */
//        /* shall Write zeros to the local Mirror RAM copy of the State partition of NVM*/
//        Current_NVM_State = Default_NVM_State;
//
//        /* shall write all zeros to the page (128 bytes) given by the addr argument*/
//        success = Nvm_Erase( addr, NVM_MAXWRITESIZE );
//    }
//
//    /* shall return address of next page to be erased if write was successful, and
//       return given argument otherwise*/
//    if (success == true)
//    {
//        /* PATH(Nvm_Clear_State,E); */
//        retval = addr + NVM_MAXWRITESIZE;
//
//        /* shall return '0xffffffff' to signify completion if 'next page address' is
//           outside NVM range*/
//        if (retval >= NVM_SIZE)
//        {
//            /* PATH(Nvm_Clear_State,F); */
//            retval = 0xFFFFFFFF;
//			Nvm_State = Default_NVM_State;
//			McuBootCount = 1;
//			Nvm_State.bootcount = 1;
//        }
//    }
//    else
//    {
//        /* PATH(Nvm_Clear_State,G); */
//        retval = addr;
//    }
//
//    /* PATH(Nvm_Clear_State,H); */
//    return retval;
//} /* end of Nvm_Clear_State */

/*
    Purpose: This function retrieves the most recently stored State record that
    contains a reported fault.

    param[out] state  Pointer to State data structure to be filled in with last failed State

    Global Variables Referenced:
    	Current_NVM_State_Pointer
    	Default_NVM_State

    retval Uint16  Index of the returned state...0 indicates no failed states exist

    Preconditions and Assumptions:
        none

*/
int16 Nvm_GetLastFailState( Nvm_State_t *state )
{
    int16 index;
    int16 ret = -1;

    /* PATH(Nvm_GetLastFailState,A); */
    if (state)
    {
	    /* PATH(Nvm_GetLastFailState,G); */

	    /* shall start at the current state, finding the most recent state with a non-zero Fault ID*/
	    index = Current_NVM_State_Pointer;

	    do
	    {
	        /* PATH(Nvm_GetLastFailState,B); */

		    Nvm_GetState(index,state);

	        /* shall wrap to the end of the buffer if the first buffer has no fault.*/
	        if (index == 0)
	        {
	            /* PATH(Nvm_GetLastFailState,C); */
	            index = MAX_STATE_RECORDS;
	        }

	        index--;

	      /* Loop until you either find a fault or have traversed all of NVM*/
	    } while (((*state).faultId == 0) && (index != Current_NVM_State_Pointer));

	    /* shall return the index of the most recent Fail State record,
	       or -1 if no fault records were found.*/
	    if ((*state).faultId == 0)
	    {
	        /* PATH(Nvm_GetLastFailState,D); */
	        /* No Fails Found...clear the state and return -1*/
	        *state = Default_NVM_State;
	        ret = -1;
	    }
	  	else
	    {
	        /* PATH(Nvm_GetLastFailState,E); */
	        /* Return Index of the last fail*/
	  	    index++;
	        if (index >= MAX_STATE_RECORDS)
	        {
	        	/* PATH(Nvm_GetLastFailState,H); */
	        	index = 0;
	      	}
	        ret = index;
	    }
    }
    /* PATH(Nvm_GetLastFailState,F); */
    return (ret);
} /* end of Nvm_GetLastFailState */


/*
    Purpose: This function retrieves the second-most recently stored State
    record that contains a reported fault.

    param[out] state  Pointer to State data structure to be filled in with next
    most recent failed State

    Global Variables Referenced:
    	Current_NVM_State_Pointer
    	Default_NVM_State

    retval Uint16  Index of the returned state...0 indicates no other failed states exist

    Preconditions and Assumptions:
        none

*/
int16 Nvm_GetNextToLastFailState( Nvm_State_t *state )
{
    int16 index;
    int16 ret = -1;

    /* PATH(Nvm_GetNextToLastFailState,A); */

    if (state)
    {

    	/* PATH(Nvm_GetNextToLastFailState,H); */

	    /* shall first find the most recent fail state*/
	    index = Nvm_GetLastFailState(state);

	    if (index == -1)
	    {
	        /* shall return -1 if no states with fault information exist*/
	        /* PATH(Nvm_GetNextToLastFailState,B); */
	        *state = Default_NVM_State;
	        ret = -1;
	    }
	    else
	    {
	        /* shall start at the state previous to the last fail state,
	           finding the next most recent state with a non-zero Fault ID*/
	        /* PATH(Nvm_GetNextToLastFailState,C); */
            if (index == 0)
            {
                /* PATH(Nvm_GetNextToLastFailState,J); */
                index = MAX_STATE_RECORDS;
            }
	        index--;

	        do
	        {
	            /* PATH(Nvm_GetNextToLastFailState,D); */
	            Nvm_GetState(index,state);

	            /* shall wrap to the end of the buffer if the first buffer has no fault.*/
	            if (index == 0)
	            {
	                /* PATH(Nvm_GetNextToLastFailState,E); */
	                index = MAX_STATE_RECORDS;
	            }
	            index--;

	        /* Loop until you either find a fault or have traversed all of NVM*/
	        } while (((*state).faultId == 0) && (index != Current_NVM_State_Pointer));

	        /* shall return the index of the next to most recent Fail State record,
	           or 0 if no other fault records were found.*/
			if ((*state).faultId == 0)
	        {
	            /* PATH(Nvm_GetNextToLastFailState,I); */
	            /* No Fails Found...clear the state and return 0*/
	            *state = Default_NVM_State;
	            ret = -1;
	        }
	        else
	        {
	            /* PATH(Nvm_GetNextToLastFailState,F); */
	            /* Return Index of the last fail*/
                index++;
                if (index >= MAX_STATE_RECORDS)
                {
                    /* PATH(Nvm_GetNextToLastFailState,K); */
                    index = 0;
                }
                ret = index;
	        }
	    }
    } /* end of if state */

    /* PATH(Nvm_GetNextToLastFailState,G); */
    return (ret);
} /* end of Nvm_GetNextToLastFailState */


/*
    brief Writes zeros to erase the given address in the NVM.

    Purpose:
        This API is usable to "erase" the given address in the NVM.  The EEPROM part has no
        block- or chip-erase function, meaning that erases must be done be writing zeros
        to each address individually.  A single write-cycle operation can operate on no
        more than 128 bytes at a time.

    param[in]  offset  Start offset to be erased in the NVM.
    param[in]  len     Length of section (in 16-bit words) to be erased.

    retval true     Successfully erased NVM segment.
    retval false    NVM Segment not erased; EEPROM was busy in a write cycle.

    Preconditions and Assumptions:
        No more than 128 bytes should be addressed at any one time, or there is risk of frame overrun
        while waiting for the write cycle to complete.

*/
bool Nvm_Erase( unsigned long offset, unsigned int len )
{
    bool status = false;
    uint16_t *ptr;
    uint16_t *nvmptr;
    uint16_t *endptr;
    uint16_t minlen;

    /* truncate length if offset + len is not in the range of NVM */
    if (((offset + len) > NVM_SIZE) && (offset <= NVM_SIZE))
    {
        /* PATH(Nvm_Erase,H); */
        len = (NVM_SIZE - offset);
    }

    /* shall write 'blanks' to the given offset to erase them if not busy*/
    if ((offset <= NVM_SIZE) && ((offset + len) <= NVM_SIZE))
    {
        /* PATH(Nvm_Erase,F); */
//        Nvm_Write(offset, blanks, len);

        /* shall limit the amount of data to be written to the NVM maximum write size*/
        minlen = min(len, NVM_MAXWRITESIZE);

        ptr = (uint16_t *) u16Zeros;         /* Start of the Input buffer*/
        endptr = (ptr + minlen);             /* End of the Input Buffer*/
        nvmptr = (uint16_t *) (NVM_BASEADDR + offset);    /* Start of NVM write area*/

        while (ptr < endptr)
        {
            *nvmptr = *ptr;
            nvmptr++;
            ptr++;
        }

        status = true;
    }
    else
    {
        /* PATH(Nvm_Erase,G); */
        status = false;
    }


    /* shall return true of the write was successful, false if not.*/
    /* PATH(Nvm_Erase,E); */
    return status;
}

/*
    brief Stores position information into NVM data structure for rigging, for the on-side motor.

    Purpose:
        Stores position information into NVM Rigging data structure. Data must be written to
        the NVM in a separate step.

    param[in] position Nominal position of FSL, used to indicate location of position storage

    Global Variables Referenced:
        #Nvm_Rigging
        #tHall

    return  void

*/
void Nvm_SetPosOnside(RIG_POSITIONS_T position)
{
    /* PATH(Nvm_SetPosOnside,A); */
    if(position < NUM_RIG_POSITIONS)
    {
        Nvm_Rigging_Temp.quad[position].onside = tHall.Position;

        /* Save NVM Rigging data related to the configured Skew Sensor */
        SKEW_SNSR_SET_NVM_RIG_DATA(position);

        /* Save Sensor Fusion Data */
        Nvm_Rigging_Temp.tPanel[position].tSf.f32StrokeFused = G_ASW_DATA.tPanelOutputData.f32StrokeFused;
        Nvm_Rigging_Temp.tPanel[position].tSf.f32BiasFused = G_ASW_DATA.tPanelOutputData.f32BiasFused;
        Nvm_Rigging_Temp.tPanel[position].tSf.f32QuadCntResidualFused = G_ASW_DATA.tPanelOutputData.f32QuadCntResidualFused;
        Nvm_Rigging_Temp.tPanel[position].tSf.f32SkewResidualFused = G_ASW_DATA.tPanelOutputData.f32SkewResidualFused;
        Nvm_Rigging_Temp.tPanel[position].tSf.s16QuadFused = G_ASW_DATA.tPanelOutputData.s16QuadFused;

        /* Save NVM Panel data related to the configured Skew Sensor */
        SKEW_SNSR_SET_NVM_PANEL_DATA(position);

    }
} /* end of Nvm_SetPosOnside */

#if defined(__HALLX_CONFIGURED)
/*
    brief Stores position information into NVM data structure for rigging, for the cross-side motor.

    Purpose:
        Stores position information into NVM Rigging data structure. Data must be written to
        the NVM in a separate step.

    param[in] position Nominal position of FSL, used to indicate location of position storage

    Global Variables Referenced:
        #Nvm_Rigging
        #tHallx

    return  void

*/
void Nvm_SetPosXside(RIG_POSITIONS_T position)
{
    /* PATH(Nvm_SetPosOnside,A); */
    if(position < NUM_RIG_POSITIONS)
    {
        Nvm_Rigging_Temp.quad[position].xside = tHallx.Position;
    }
} /* end of Nvm_SetPosXside */
#endif

/*
    brief Retrieves position information from the NVM data structure for rigging, for the on-side motor.

    Purpose:
        Accesses position information from NVM Rigging data structure. Data must be retrieved from
        the NVM in a separate (prior) step.

    param[in] position Nominal position of FSL, used to indicate location of position storage

    Global Variables Referenced:
        #Nvm_Rigging

    return  int  Rigged Hall-sensor count of the given flap position

*/
int Nvm_GetPosOnside(RIG_POSITIONS_T position)
{
    int16_t retPos = 0;

    if(position < NUM_RIG_POSITIONS)
    {
        /* shall retrieve position from NVM */
        retPos = tNvm.tData.Nvm_Rigging.quad[position].onside;
    }

    return (retPos);
} /* end of Nvm_GetPosOnside */

#if defined(__HALLX_CONFIGURED)
/*
    brief Retrieves position information from the NVM data structure for rigging, for the cross-side motor.

    Purpose:
        Accesses position information from NVM Rigging data structure. Data must be retrieved from
        the NVM in a separate (prior) step.

    param[in] position Nominal position of FSL, used to indicate location of position storage

    Global Variables Referenced:
        #Nvm_Rigging

    return  int  Rigged Hall-sensor count of the given flap position

*/
int Nvm_GetPosXside(RIG_POSITIONS_T position)
{
    int16_t retPos = 0;

    if(position < NUM_RIG_POSITIONS)
    {
        /* shall retrieve position from NVM */
        retPos = tNvm.tData.Nvm_Rigging.quad[position].xside;
    }

    return (retPos);
} /* end of Nvm_GetPosXside */
#endif

/*

    Purpose:
        If the Nvm_State data structure has been erased, Nvm_ValidateRigStatus
        sets Nvm_State.rigstatus to NOT_RIGGED.

    param[in]
		none

    Global Variables Referenced:
       	Nvm_Rigging
		Nvm_State.rigstatus
		NOT_RIGGED RIG_STATUS

    return
    	none

*/
void Nvm_ValidateRigStatus(void)
{
    Uint16 i, sum;
    Uint16 * rigCheck;

    /* PATH(Nvm_ValidateRigStatus,A); */

    /* We need to determine if the State data structure still has a RIG_VERIFIED/RIG_COMPLETE status, and if
     the rig data structure has been erased, in this condition we want to default the system to NOT_RIGGED*/
    if (Nvm_State.rigstatus != NOT_RIGGED)
    {
        /* PATH(Nvm_ValidateRigStatus,B); */
        rigCheck = (Uint16 *)&tNvm.tData.Nvm_Rigging;

        for (i = 0, sum = 0; i < sizeof(Nvm_Rigging_t); i++)
        {
            /* PATH(Nvm_ValidateRigStatus,C); */
            sum += (* rigCheck);
            rigCheck++;
        }

        if (sum == 0)
        {
            /* PATH(Nvm_ValidateRigStatus,D); */
            Nvm_State.rigstatus = NOT_RIGGED;
        }
    }

    /* PATH(Nvm_ValidateRigStatus,E); */
}


/*

    Purpose:
        This function manages the availability of the flaps based on the rig status passed to it.

    param[in] RIG_STATUS status - Rig status of the channel.

    Global Variables Referenced:
        RIG_STATUS
        RigInProcess
  		FAS_NotAvail
  		InhibitFlap

    return
    	none

*/
void Nvm_RigStatusSetup(RIG_STATUS status)
{
    /* PATH(Nvm_RigStatusSetup,A); */

    switch (status)
    {
        case RIG_COMPLETE:
            /* PATH(Nvm_RigStatusSetup,B); */
            bRigInProcess = true;
            bFasNotAvailable = true;
            break;
        case RIG_VERIFIED:
            /* PATH(Nvm_RigStatusSetup,C); */
            bRigInProcess = false;
            bFasNotAvailable = false;
            break;
        case NOT_RIGGED:
            /* PATH(Nvm_RigStatusSetup,D); */
        default:
            /* PATH(Nvm_RigStatusSetup,E); */
            bFasNotAvailable = true;
            bInhibitFlap = true;
            break;
    }

    /* PATH(Nvm_RigStatusSetup,F); */
}


/*

    Purpose:
        This function writes out the NVM state records and manages the
         writing of the Powerdown State record as well.

    param[in] None.

    Global Variables Referenced:
        StatusWriteTimer
        Nvm_State_Temp
  		Nvm_PowerDown
  		Nvm_Powerdown_State

    return
    	none

*/
void Nvm_WriteOutStateInformation(void)
{
	/* PATH(Nvm_WriteOutStateInformation,A); */

	/* Write the first part of the State data structure if necessary */
    //if ((Nvm_BootModePwrDwn == false) && (OneMsTimer >= POWER_DOWN_BOOTMODE))
    if (OneMsTimer >= POWER_DOWN_BOOTMODE)
    {
        /* PATH(Nvm_WriteOutStateInformation,B); */

        /* REMOVED POWERDOWN RECORD SINCE NO HOLD-UP CAPS. CAN ADD BACK IN LATER WHEN WE CAN DETECT A POWERDOWN */
//		/* StatusWriteTimer has expired, meaning only the Powerdown record
//		    should be written, ignore the other records in the queue */
//		else if ((Timer_IsExpired(&StatusWriteTimer) == true) && (Nvm_PowerDown == false))
//		{
//			/* PATH(Nvm_WriteOutStateInformation,D); */
//			Nvm_WriteState(&Nvm_Powerdown_State);
//			Nvm_PowerDown = true;       /* One-shot state write on powerdown */
//		}
    }

	/* PATH(Nvm_WriteOutStateInformation,E); */
}


/* end nvm.c*/
