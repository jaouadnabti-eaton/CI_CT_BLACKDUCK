/****************************************************************************************************
*  File name: timer.h
*
*  Purpose: Interface for Timer driver
*  This file describes the Public Interface for the CPU Timer driver.  The interface provides
*  routines necessary for setting up the base CPU timer for Scheduler control.
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
#ifndef TIMER_H__
#define TIMER_H__

/* Include Files */

//#include "dsp281x_regs.h"	/* DSP281x Include File, includes register definition file*/
#include "F2837xD_device.h"
#include "F2837xD_cputimervars.h"
#include "parameter.h" 		/* Parameters file include Sensor parameters */
/*
  defgroup timer Timers
  
  brief The Timer driver is a container for low-level timer control, and for a generic timer service.

    The Timer driver provides two classes of service:
        -# initializes CPU timer hardware and launches hardware-driven timer interrupts
        -# generic facility for long-duration (>1ms) counter/timers

    The generic timer facility is designed to be used in a distributed manner.  No facility
    is provided for interrupt or callback due to expiry of long-duration timers.  Users should
    instantiate the Timer_t object, then use the Timer_SetTime() call to configure and start
    the timer running.  Users will poll the timer using either Timer_IsExpired() or Timer_GetTime()
    to determine whether the timer is expired or to determine how much time is left before expiry.

    The generic timer facility depends on the free-running counter configured and serviced by the
    CPU timer hardware, so Timer_Init() must be called before any calls to Timer_SetTime().

    Timers may be one-shot (default) or periodic in nature.  One-shot timers have a single duration, after
    which the remainder field in Timer_GetTime() will always return 0, and Timer_IsExpired() will always
    return true (until the hardware counter rolls over).  In other words, the "expired" boolean is a
    step function in time.

    Periodic timers are automatically reset after they expire, so that they will repeatedly expire after
    the duration time has elapsed.  The Timer_IsExpired() API will return true just once per period.
    In other words, the "expired" boolean is a periodic pulse function in time.
*/

/* Public Type Definitions*/ 
#define TIMER0_FREQUENCY    8000U      /* Timer 0 is set to 8kHz */

/* brief Container object for generic timers.*/
typedef struct
{
    Uint16  flags;          /*16-bit value to contain configuration flags for the timer*/
    Uint32  duration;       /*32-bit value specifying time interval from start to expiry.*/
    Uint32  starttime;      /*32-bit value containing snapshot of CpuTimer2 hardware at timer start.*/
    bool    expired;        /*boolean value indicating if timer has expired or not.*/
} Timer_t;

#if defined (__WCTA__)
#define NUMBER_OF_SUBFRAMES          (8U)
#define SIZE_OF_WCTA_ARRAY           (NUMBER_OF_SUBFRAMES + 1)
#define WCTA_ALL_SUBFRAMES_INDEX     (NUMBER_OF_SUBFRAMES)

typedef struct
{
    uint16_t u16New;
    uint16_t u16Min;
    uint16_t u16Max;
    uint16_t u16Avg;
    uint32_t u32Filter;
} tWctaFrameTime_t;
#endif

/*    Public Variable Declarations*/

extern int32 Timer_MasterFrameCount;
extern Uint16 Timer_OneMinTimer;
extern Uint32 OneMsTimer;
extern Uint32 FrameTimer;
extern Uint16 subframe;            /*Scheduler subframe index; declared in main.c*/
//extern Timer_t tStartupTimer;
#if defined (__WCTA__)
extern tWctaFrameTime_t tWctaFrameTime[SIZE_OF_WCTA_ARRAY];
#endif

/*      Public ROM Constants
*/

/*      Public Defines
*/

/* name Timer Configuration Flags*/
/* Use these flags with the Timer_SetTime() API to configure a timer.*/

#define TIMER_NOTSET    0x0000
#define TIMER_ONESHOT   0x0001    /*Configures timer to be "one-shot" (default).*/
#define TIMER_PERIODIC  0x0002    /*Configures timer to be "periodic".*/

/* name Timer Duration Value Macros */
/* These macros provide a timebase-dependent way to load a real-time value into a generic timer.*/
/*Expected syntax would be something like Timer_SetTime(&timer, NULL, TIMER_100ms); where timer is/
 an instance of the Timer_t structure.*/

#define TIMER_1ms       ((SYSTEM_FREQUENCY / COUNTER_DIVIDER) * 1000UL) /*20,000 */
#define TIMER_5ms       5UL * TIMER_1ms
#define TIMER_10ms      10UL * TIMER_1ms
#define TIMER_11ms      11UL * TIMER_1ms
#define TIMER_20ms      20UL * TIMER_1ms
#define TIMER_30ms      30UL * TIMER_1ms
#define TIMER_50ms      50UL * TIMER_1ms
#define TIMER_100ms     100UL * TIMER_1ms
#define TIMER_200ms     200UL * TIMER_1ms
#define TIMER_300ms     300UL * TIMER_1ms
#define TIMER_400ms     400UL * TIMER_1ms
#define TIMER_500ms     500UL * TIMER_1ms
#define TIMER_750ms     750UL * TIMER_1ms
#define TIMER_1000ms    1000UL * TIMER_1ms
#define TIMER_1500ms    1500UL * TIMER_1ms
#define TIMER_1900ms    1900UL * TIMER_1ms
#define TIMER_1s        1000UL * TIMER_1ms
#define TIMER_2s        2UL * TIMER_1s
#define TIMER_3s        3UL * TIMER_1s
#define TIMER_4s        4UL * TIMER_1s
#define TIMER_5s        5UL * TIMER_1s
#define TIMER_8s        8UL * TIMER_1s
#define TIMER_10s       10UL * TIMER_1s
#define TIMER_11s       11UL * TIMER_1s
#define TIMER_30s       30UL * TIMER_1s
#define TIMER_40s       40UL * TIMER_1s
#define TIMER_31s       31UL * TIMER_1s
#define TIMER_1min      60UL * TIMER_1s
#define TIMER_120s      120UL * TIMER_1s


/*Default Initializer for the Timer_t Object*/
#define TIMER_DEFAULTS  { TIMER_NOTSET, 0UL, 0UL, false }

/*      Public Interface Function Prototypes
*/

void Timer_Init(void);
bool Timer_IsExpired(Timer_t *);
bool Timer_GetTime(Timer_t *, Uint32 *);
bool Timer_SetTime(Timer_t *, Uint16, Uint32);
bool Timer_ResetTimer(Timer_t *);
bool Timer_IsSet(Timer_t *);
#if defined (__WCTA__)
void Wcta_ResetTimer(void);
void Wcta_StoreResult(uint16_t u16Subframe);
#endif

#endif
/*end timer.h*/

