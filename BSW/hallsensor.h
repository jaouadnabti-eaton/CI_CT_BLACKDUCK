/****************************************************************************************************
*  File name: hallsensor.h
*
*  Purpose: Interface for Hall-effect sensor driver
*  This file describes the Public Interface for the Hall Sensor driver.  The interface provides
*  routines necessary to monitoring the position and speed of the motor through its magnetic
*  sensors.
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
#ifndef HALLSENSOR_H__
#define HALLSENSOR_H__

/*      Include Files
*/
//#include "dsp281x_regs.h"
#include "F2837xD_device.h"
#include "IQmathLib.h"
#include "parameter.h"

/**
  defgroup hallsensor Hall Sensor Driver
  
  brief The Hall Sensors driver is an abstraction for monitoring the position and speed
        of the motor through its magnetc--Hall-effect--sensors.  Included in this driver
        is access to the cross-channel Hall sensors as well.
*/


/*      Public Type Definitions
*/

/**
 *  brief The Hall Sensor Driver object
 *
 *  The Hall Sensor object is a container for all variables and methods relating to the
 *  Hall sensor inputs.  Includes history provisions for a debouncing algorithms, a running
 *  edge count, and a running count of motor revolutions.
*/ 

enum hallState_e
{
	GOOD_HALLS = 1,
	ALL_ZEROS = 0,
	ALL_ONES = 7
};
typedef enum hallState_e HallState_t;

typedef struct
{
    uint16_t bHall1:1;   /* 0 Hall 1 */
    uint16_t bHall2:1;   /* 1 Hall 2 */
    uint16_t bHall3:1;   /* 2 Hall 3 */
    uint16_t rsvd1:13;  /* 15:3 RESERVED */
} tHallBits_t;

typedef union
{
    uint16_t  all;      /* 16-bit Unsigned Integer bit packed */
    tHallBits_t  bit;   /* bit field */
} tHallWord_t;

typedef struct 
{
    /* Variable: Number of Hall Capture Interrupts*/ 
    Uint16 CaptureCount;
    /* Variable: Timestamp of most recent Hall Capture*/ 
    Uint32 TimeStamp;
    /* Variable: Most recent logic level on CAP/GPIO*/ 
    Uint16 HallGpio;
    /* Variable: Buffer of previous logic level on CAP/GPIO*/ 
    Uint16 HallGpioBuffer;
    /* Variable: Double buffer of two logic levels on CAP/GPIO previous*/ 
    Uint16 HallGpioDblBuffer;
    /* Variable: ECAP flags, indicating which XINT detected the edge*/
    tHallWord_t tXintFlag;
    /* Parameter: Running counter, with a revolution defined as 1-cycle of the 6 hall states*/ 
    int16 Revolutions;
    /* Parameter: Quadrature-based flap position, incremented for CW hall sensor events, decremented for CCW events*/ 
    int16 Position;
    /* Variable: Used for increment and decrement of quadrature count*/ 
    int16 Direction;
    /* Variable: Used for increment and decrement of quadrature count*/ 
    bool Armed;
} Hall_t;


/* An instance of the hall sensor driver object for the on-side motor/flap*/ 
extern Hall_t tHall;

#if defined(__HALLX_CONFIGURED)
/* An instance of the hall sensor driver object to represent the cross-side motor/flap*/ 
extern Hall_t tHallx;
#endif
extern int16 hallStartPosition;

/*      Public Variable Declarations
*/

/*      Public ROM Constants
*/

/*      Public Defines
*/
#if defined(DRV8312_DEV_KIT)
/* 2837xD Development Kit Hardware */

#define CHA_HS1_GPIO_PIN     24  /* HALL A is on GPIO Pin 24 */
#define CHA_HS2_GPIO_PIN     25  /* HALL B is on GPIO Pin 25 */
#define CHA_HS3_GPIO_PIN     26  /* HALL C is on GPIO Pin 26 */
    #if defined(__HALLX_CONFIGURED)
    #define CHB_ICC_HS1_GPIO_PIN     CHA_HS1_GPIO_PIN  /* Cross-Side Hall Effect Sensor 1 doesn't exist. map to Hall A */
    #define CHB_ICC_HS2_GPIO_PIN     CHA_HS2_GPIO_PIN  /* Cross-Side Hall Effect Sensor 2 doesn't exist. map to Hall B */
    #endif

#else
/* MCU HW */
#define CHA_HS1_GPIO_PIN         6   /* On-Side Hall Effect Sensor 1:  GPIO6 */
#define CHA_HS2_GPIO_PIN         7   /* On-Side Hall Effect Sensor 2:  GPIO7 */
#define CHA_HS3_GPIO_PIN         8   /* On-Side Hall Effect Sensor 3:  GPIO8 */
    #if defined(__HALLX_CONFIGURED)
    #define CHB_ICC_HS1_GPIO_PIN     9   /* Cross-Side Hall Effect Sensor 1:  GPIO9 */
    #define CHB_ICC_HS2_GPIO_PIN     10  /* Cross-Side Hall Effect Sensor 2:  GPIO10 */
    #define CHB_ICC_HS3_GPIO_PIN     11  /* Cross-Side Hall Effect Sensor 2:  GPIO11 */
    #endif
#endif

#define HALL_ECAP_ISR_ENABLE_MASK   (0x0007U)       /* INTx1, INTx2, and INTx3 enable mask */
#define HALL_ECAP_ISR_DISABLE_MASK  (0xFFF8U)       /* INTx1, INTx2, and INTx3 disable mask */
#define HALL_TIMER_INIT_VALUE       (0xFFFFFFFFU)   /*Timer used to capture hall timings in initialized with this value */

/* Default Initializer for the Hall Object*/ 
#define EMB_MCU_HALL    { 0,    /* CaptureCount      */ \
                          0,    /* TimeStamp         */ \
                          0,    /* HallGpio          */ \
                          0,    /* HallGpioBuffer    */ \
                          0,    /* HallGpioDblBuffer */ \
                          0,    /* CapFlag           */ \
                          0,    /* Revolutions       */ \
                          0,    /* Position          */ \
                          0,    /* Direction         */ \
                          false /* Armed             */ }

/* Target Independent Default Initializer Hall Object*/ 
#define HALL_DEFAULTS EMB_MCU_HALL



/*      Public Interface Function Prototypes
*/
void Hall_Read( Hall_t *p );
#if !defined(__HALLX_CONFIGURED)
void Hall_Init( Hall_t *on );
#else
void Hall_Init( Hall_t *on, Hall_t *cross );
void Hallx_Read( Hall_t *p );
#endif

void Hall_ComputePosition( Hall_t *p );

__interrupt void Hall_1_ISR( void );
__interrupt void Hall_2_ISR( void );
__interrupt void Hall_3_ISR( void );
#if defined(__HALLX_CONFIGURED)
__interrupt void Hallx_1_ISR( void );
__interrupt void Hallx_2_ISR( void );
__interrupt void Hallx_3_ISR( void );
#endif

#endif
/* end hallsensor.h*/ 

