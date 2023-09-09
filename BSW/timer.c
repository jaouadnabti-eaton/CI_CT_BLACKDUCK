/****************************************************************************************************
*  File name: timer.c
*
*  Purpose: Implements timer driver.
*  This example configures CPU Timer0 and increments a counter each time the timer asserts an
*  interrupt.
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

//#include "dsp281x.h"   	/* DSP281x Include File, includes register definition file*/
#include "parameter.h"	/* Parameters file include Sensor parameters */
#include "timer.h"	

/*      Local Type Definitions
*/

/*      Local Defines
*/

/*      Global Variables
*/
/* brief Up-Down counter used as timebase for the ref scheduler "Scheduler".*/

/* The master counter is set up to pulse at a timebase of 8 kHz.  The scheduler then uses that to schedule\n
 its eight timeslices, each on a 1 kHz timebase.*/

int32 Timer_MasterFrameCount = 0;
Uint16 Timer_OneMinTimer = 0;
#if defined (__WCTA__)
/* u16New, u16Min, u16Max, u16Avg, u32Filter */
tWctaFrameTime_t tWctaFrameTime[SIZE_OF_WCTA_ARRAY] =
{
   { 0U, 0xFFFFU, 0U, 0U, 0UL},{ 0U, 0xFFFFU, 0U, 0U, 0UL},{ 0U, 0xFFFFU, 0U, 0U, 0UL},{ 0U, 0xFFFFU, 0U, 0U, 0UL},
   { 0U, 0xFFFFU, 0U, 0U, 0UL},{ 0U, 0xFFFFU, 0U, 0U, 0UL},{ 0U, 0xFFFFU, 0U, 0U, 0UL},{ 0U, 0xFFFFU, 0U, 0U, 0UL},
   { 0U, 0xFFFFU, 0U, 0U, 0UL}
};
#endif

/*      Local ROM Constants
*/

/*      Local Variable Declarations
*/

/*      Local Function Prototypes
*/
__interrupt void Timer_timer0_isr(void);

#if defined (__WCTA__)
uint16_t Wcta_GetResult(void);
void Wcta_CalculateAverage(uint16_t u16Subframe);
void Wcta_CheckMin(uint16_t u16Subframe);
void Wcta_CheckMax(uint16_t u16Subframe);
#endif

/*      Function Definitions
*/

/*
    brief Initialize timer hardware.

    Purpose:
        Configures Timer hardware for the scheduler base, and sets up ISR.

    Global Data Referenced:
        #PieVectTable   \n
        #PieCtrlRegs    \n
        #CpuTimer0      \n
        #CpuTimer2Regs 
    
    return void
    
    Preconditions and Assumptions:
        This function should be called only once, prior to the scheduler loop.

*/
void Timer_Init(void)
{
    uint32_t u32Period = 0U;

    /* PATH(Timer_Init,A); */
    
    /* shall assign Timer 0 ISR function pointer to interrupt vector table at the Timer 0 entry */
    EALLOW;
    PieVectTable.TIMER0_INT = &Timer_timer0_isr;
    EDIS;

    /* shall use the default initialization for CPU Timer 0*/
    //dsp281x_InitCpuTimers();   /* Initialize the Cpu Timers (CPU Timer 0)*/
    /* REWORK */
    InitCpuTimers();    /* Init CPU Timers to a known state */

    /* shall configure CPU Timer 0 to trigger its ISR at a 125 usecond period (8 kHz) for the scheduler*/
    //dsp281x_ConfigCpuTimer(&CpuTimer0, SYSTEM_FREQUENCY, 125);

    /* SETUP TIMER 0 */
    /* shall initialize and configure CPU Timer 0 to be 8kHz (125us) timer with ISR */
    u32Period = ((((SYSTEM_FREQUENCY * HZ_TO_KHZ_MULTIPLIER) * KHZ_TO_MHZ_MULTIPLIER) / TIMER0_FREQUENCY) - 1U);    /* Timer 0 Period. */
    CpuTimer0Regs.PRD.all  = u32Period;     /* Set Period to 8kHz Equivalent */
    CpuTimer0Regs.TPR.bit.PSC = 0U;         /* Use SYSCLKOUT (Divide by 1) */
    CpuTimer0Regs.TPR.bit.TDDR = 0U;        /* No divide down. Use SYSCLKOUT (Divide by 1) */
    CpuTimer0Regs.TPRH.all = 0U;            /* Init High Prescaler and Divide to 0 */
    CpuTimer0Regs.TCR.bit.TSS = 1U;         /* Hold the Timer in Stop */
    CpuTimer0Regs.TCR.bit.TRB = 1U;         /* Reload all counter register with period value: */
    CpuTimer0Regs.TCR.bit.SOFT = 0U;        /* Stop Timer upon SW Breakpoint in Emulation */
    CpuTimer0Regs.TCR.bit.FREE = 0U;        /* Stop Timer upon SW Breakpoint */
    CpuTimer0Regs.TCR.bit.TIE = INTERRUPT_ENABLE;   /* Enable Interrupt for TIMER0 */
    CpuTimer0Regs.TCR.bit.TSS = 0U;         /* Start the Timer */

    /* shall enable TINT0 in the PIE: Group 1 interrupt 7 */
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    /* shall enable CPU INT1 which is connected to CPU-Timer 0 through PIE Group 1 */
    IER |= M_INT1;

#if defined (__WCTA__)
    /* SETUP TIMER 1 */
    /* shall initialize and configure CPU Timer 1 to be a free running 32 bit timer, incrementing at 10MHz rate */
    CpuTimer1Regs.PRD.all  = 0xFFFFFFFF;    /* Initialize timer period to maximum */
    CpuTimer1Regs.TPR.bit.PSC = 0U;         /* Prescaler divide by 1 (SYSCLKOUT) */
    CpuTimer1Regs.TPR.bit.TDDR = (WCTA_COUNTER_DIVIDER - 1);    /* Divide Down Register. One less than the actual divider desired. 200MHz/20 = 10 MHz*/
    CpuTimer1Regs.TPRH.all = 0U;            /* Init High Prescaler and Divide to 0 */
    CpuTimer1Regs.TCR.bit.TSS = 1U;         /* Hold the Timer in Stop */
    CpuTimer1Regs.TCR.bit.TRB = 1U;         /* Reload all counter register with period value: */
    CpuTimer1Regs.TCR.bit.SOFT = 0U;        /* Stop Timer upon SW Breakpoint in Emulation */
    CpuTimer1Regs.TCR.bit.FREE = 0U;        /* Stop Timer upon SW Breakpoint */
    CpuTimer1Regs.TCR.bit.TIE = INTERRUPT_DISABLE;   /* Disable Timer 1 Interrupt */
    CpuTimer1Regs.TCR.bit.TSS = 0U;         /* Start up the timer */

#endif

    /* SETUP TIMER 2 */
    /* shall initialize and configure CPU Timer 2 to be a free running 32 bit timer, incrementing at SYSCLOCKOUT/4 rate */
    CpuTimer2Regs.PRD.all  = 0xFFFFFFFF;    /* Initialize timer period to maximum */
    CpuTimer2Regs.TPR.bit.PSC = 0U;         /* Prescaler divide by 1 (SYSCLKOUT) */
    CpuTimer2Regs.TPR.bit.TDDR = (COUNTER_DIVIDER - 1);    /* Divide Down Register. One less than the actual divider desired. 200MHz/10 = 20 MHz*/
    CpuTimer2Regs.TPRH.all = 0U;            /* Init High Prescaler and Divide to 0 */
    CpuTimer2Regs.TCR.bit.TSS = 1U;         /* Hold the Timer in Stop */
    CpuTimer2Regs.TCR.bit.TRB = 1U;         /* Reload all counter register with period value: */
    CpuTimer2Regs.TCR.bit.SOFT = 1U;        /* Timer Free Run */
    CpuTimer2Regs.TCR.bit.FREE = 1U;        /* Timer Free Run */
    CpuTimer2Regs.TCR.bit.TIE = INTERRUPT_DISABLE;   /* Disable Timer 2 Interrupt */
    CpuTimer2Regs.TCR.bit.TSS = 0U;         /* Start up the timer */

    /* shall enable global Interrupts and higher priority real-time debug events: */
    EINT;
    ERTM;

    /* PATH(Timer_Init,B); */
} 


/*
    brief Interrupt handler for timer0.

    Purpose:
        Handles timer0 interrupt and increments master frame counter.

    Global Data Referenced:
        #Timer_MasterFrameCount \n
        #PieCtrlRegs
    
    return void
    
    Preconditions and Assumptions:
        None.

*/
__interrupt void Timer_timer0_isr(void)
{
    /* PATH(Timer_timer0_isr,A); */

    /* shall increment #Timer_MasterFrameCount for the scheduler */
	if (Timer_MasterFrameCount <= 1)
	{
		/* PATH(Timer_timer0_isr,D); */
    	Timer_MasterFrameCount++;
	}

    /* shall increment the one millisecond timer */
    if (subframe == 7)
    {
        /* PATH(Timer_timer0_isr,B); */
        OneMsTimer++;
    }
    FrameTimer++;

    /* shall reset this interrupt so that more interrupts can be received*/
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    /* PATH(Timer_timer0_isr,C); */
}

/*
	Brief:
		Resets passed timer.

	Purpose:
		The Timer_ResetTimer function will reset all the elements of the
		given timer structure.

	Parameter[in]:
		timer - Pointer to timer to be queried.
	
	Parameter[out]:
		timer - Pointer to timer to be reset.

	Return:
		bool = true - Timer was reset successfully.
		bool = false - Timer was not reset because it was a NULL pointer.

	Preconditions and Assumptions:
		None.

*/
bool Timer_ResetTimer( Timer_t *timer )
{
    bool status = false;

    /* PATH(Timer_ResetTimer,A); */ 
    
	/* shall check that timer pointer is not NULL */
    if (timer)
    {
        /* PATH(Timer_ResetTimer,B); */

        timer->flags = TIMER_NOTSET;
        timer->duration = 0;
        timer->starttime = 0;
        timer->expired = false; 

        status = true;
    }

    /* PATH(Timer_ResetTimer,C); */
    return (status);
}


/*
	Brief:
		Tells if a timer has been set yet.

	Purpose:
		The Timer_IsSet function will return true if the instance of given
		timer is set to one-shot or periodic timer.

	Parameter[in]:
		timer - Pointer to timer to be queried.

	Return:
		bool = true - Timer is set.
		bool = false - Timer is not set.

	Preconditions and Assumptions:
		None.

*/
bool Timer_IsSet( Timer_t *timer )
{
    bool status = false;

    /* PATH(Timer_IsSet,A); */

	/* shall check that timer pointer is not NULL */
    if (timer)
    {
        /* PATH(Timer_IsSet,B); */

        if (timer->flags != TIMER_NOTSET)
        {
            /* PATH(Timer_IsSet,C); */
            status = true;
        }
    }

    /* PATH(Timer_IsSet,D); */
    return (status);
}


/*
    brief Reports whether timer is expired.

    Purpose:
        Compares current time to start time of given timer index, reports
        whether or not timer has expired.

    param[in] timer  pointer to timer to be queried

    retval true    timer has expired
    retval false   timer has not expired, or timer pointer is NULL
    
     Preconditions and Assumptions:
        Timer_Init() must be called prior to calling this routine, because these
            general timers depend on the free-running hardware timer.

    remarks
        If the given timer was configured with TIMER_PERIODIC, this routine will
        return true only once per period.

*/
bool Timer_IsExpired( Timer_t *timer )
{
    bool status = false;
    Uint32 remainder = 0;

    /* PATH(Timer_IsExpired,A); */
    
    /* shall check timer pointer argument for validity; skip to return false if invalid */
    if (timer)
    {
        /* PATH(Timer_IsExpired,B); */
        
        if (timer->flags != TIMER_NOTSET)
        {
            /* PATH(Timer_IsExpired,C); */

            if (timer->expired == true)
            {
                /* PATH(Timer_IsExpired,D); */
                status = true;
            }
	        /* shall check new timer status against hardware free-running counter */
	        else if ((timer->expired == false) && (Timer_GetTime(timer, &remainder)))
	        {
	            /* PATH(Timer_IsExpired,E); */

	            /* shall return true status only when timer has expired; false otherwise */
				if (remainder <= 0)
				{
					/* PATH(Timer_IsExpired,F); */
					status = true;
					timer->expired = true;
				}
	        }
        }
    }

    /* PATH(Timer_IsExpired,G); */
    return (status);
}


/*
    brief Reports status of timer.

    Purpose:

    param[in]  timer       pointer to timer to be queried
    param[out] remainder   pointer to integer storage for remaining time since start

    retval true    function executed successfully; remainder contains a valid value
    retval false   function unsuccessful; pointer arguments are NULL
    
    Preconditions and Assumptions:
        Timer_Init() must be called prior to calling this routine, because these
            general timers depend on the free-running hardware timer.

    remarks
        No counter overflow logic is necessary when comparing with hardware free-running
        counter because we use 32-bit unsigned quantities, then compare relative times, not
        absolute times. \n
        The hardware timer used is a free-running decrementer, timed based on constants configured
        in parameter.h. \n
        For "one-shot" timers, remainder returns with zero if timer has expired.
        For "periodic" timers, remainder returns with zero once per period, and the timer's
        starttime field is reloaded with the current time value, adjusted for elapsed
        time since the timer's period expired.

*/
bool Timer_GetTime( Timer_t *timer, Uint32 *remainder )
{
    bool status = false; 		/*variable to hold the status */
    Uint32 elapsed = 0, now = 0;/* variables hold the elapsed time and the present time */

    /* PATH(Timer_GetTime,A); */

    /* shall check pointer arguments for validity; skip to return false if invalid */
    if (timer && remainder)
    {
        /* PATH(Timer_GetTime,B); */
        /* shall obtain total elapsed time in terms of the free-running counter */
        /* note: subtraction looks backward because hardware timer is a decrementer */
        now = ReadCpuTimer2Counter();
        elapsed = timer->starttime - now;

        /* shall compare elapsed time to the set duration of the timer */
        if (elapsed < timer->duration)
        {
            /* PATH(Timer_GetTime,C);*/
            *remainder = timer->duration - elapsed;
        }
        else
        {
            /* PATH(Timer_GetTime,D); */
            *remainder = 0L;

            /* shall reset start time if timer is periodic*/
            if (timer->flags & TIMER_PERIODIC)
            {
                /* PATH(Timer_GetTime,E); */
                timer->starttime = now - (elapsed - timer->duration);
                timer->expired = false;
            }
        }

        status = true;
    }

    /* PATH(Timer_GetTime,F); */
    return (status);
}


/*
   brief Sets duration and start time for timer.

    Purpose:
    
    param[in]  timer       pointer to timer to be queried
    param[in]  flags       configures timer to be one-shot or periodic.  One-shot is default.
    param[in]  duration    time interval to be loaded to timer object.  Should be nonzero.

    retval true    timer was successfully initialized and started
    retval false   Timer_SetTime() was not successful, or timer pointer is NULL
    
    Preconditions and Assumptions:
        Timer_Init() must be called prior to calling Timer_SetTime, because these
            general timers depend on the free-running hardware timer.

    remarks
        The flags field may be used to configure the timer as either a one-shot (default) or
        a periodic timer.  For a periodic timer, the Timer_t::starttime field is automatically
        reset when the expiry is checked.

        Because a zero-duration timer makes no sense, the function will return false (unsucessful)
        if a zero value is passed for duration.

*/
bool Timer_SetTime( Timer_t *timer, Uint16 flags, Uint32 duration )
{
    bool status = false;

    /* PATH(Timer_SetTime,A); */
     
    /* shall check pointer argument for validity; skip to return false if invalid */
    if (timer)
    {
        /* PATH(Timer_SetTime,B); */
        
        /* shall ensure that duration value is nonzero; skip to return false if zero */
        if (duration != 0)
        {
            /* PATH(Timer_SetTime,C); */

            /* shall configure given timer object with given duration and flags */
            /* shall configure given timer object with current time count from hardware */
            timer->flags = flags;
            timer->duration = duration;
            timer->starttime = ReadCpuTimer2Counter();
            timer->expired = false;

            status = true;
        }
    }
    
    /* PATH(Timer_SetTime,D); */
    return (status);
}

#if defined (__WCTA__)
void Wcta_ResetTimer(void)
{
    EALLOW;
    CpuTimer1Regs.TCR.bit.TRB = 1;
    CpuTimer1Regs.TCR.bit.TSS = 0;
    EDIS;
}

uint16_t Wcta_GetResult(void)
{
    uint32_t u32Time = CpuTimer1Regs.TIM.all;

    u32Time = ~u32Time;
    u32Time &= 0x0000FFFF;

    return ((uint16_t) u32Time);
}

void Wcta_StoreResult(uint16_t u16Subframe)
{
    tWctaFrameTime[u16Subframe].u16New = Wcta_GetResult();

    Wcta_CalculateAverage(u16Subframe);
    Wcta_CheckMin(u16Subframe);
    Wcta_CheckMax(u16Subframe);

    return;
}

void Wcta_CalculateAverage(uint16_t u16Subframe)
{
    uint32_t u32Temp = 0;
    int32_t  s32Diff = 0;

    u32Temp = (((uint32_t) tWctaFrameTime[u16Subframe].u16New) << 16UL);   /* scale up the latest reading */
    s32Diff = (int32_t)u32Temp - (int32_t)tWctaFrameTime[u16Subframe].u32Filter;       /* Calculate difference between new value and filtered value */
    s32Diff >>= 5UL;                                        /* scale down the difference */
    tWctaFrameTime[u16Subframe].u32Filter = (uint32_t)(tWctaFrameTime[u16Subframe].u32Filter + s32Diff);    /* sum in the difference of the new value */
    tWctaFrameTime[u16Subframe].u16Avg = (uint16_t) (tWctaFrameTime[u16Subframe].u32Filter >> 16UL);        /* scale back down */

    return;
}

void Wcta_CheckMin(uint16_t u16Subframe)
{
    if(tWctaFrameTime[u16Subframe].u16New < tWctaFrameTime[u16Subframe].u16Min)
    {
        tWctaFrameTime[u16Subframe].u16Min = tWctaFrameTime[u16Subframe].u16New;
    }

    return;
}

void Wcta_CheckMax(uint16_t u16Subframe)
{
    if(tWctaFrameTime[u16Subframe].u16New > tWctaFrameTime[u16Subframe].u16Max)
    {
        tWctaFrameTime[u16Subframe].u16Max = tWctaFrameTime[u16Subframe].u16New;
    }

    return;
}

#endif
/* end timer.c */

