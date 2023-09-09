/**************************************************************************************************
 * File name: typedefs.h
 *
 * Purpose : This is the header file for common type defines.
 *
 * Copyright Notice:
 * All source code and data contained in this file is Proprietary and
 * Confidential to Eaton Aerospace, and must not be reproduced, transmitted, or
 * disclosed; in whole or in part, without the express written permission of
 * Eaton Aerospace.
 *
 * Copyright (c) 2022 - Eaton Aerospace Group, All Rights Reserved.
 *
 * Author            Date        CR#         Description
 * ------       ------------  ------       ---------------------------------------
 * Adam Bouwens   12/07/2022    N/A        Port to MCU
 ******************************************************************************/
#ifndef TYPEDEFS_H
#define TYPEDEFS_H

//#include "stdint.h"

/******************** TYPES ********************/
/* Generic types
 * Standard ANSI-C types as recommended by MISRA 1.1
 * Although C99 introduces _Bool to the C language, _Bool will be eventually deprecated in favor of bool.
 * Per stdbool.h, _Bool is defined as unsigned char for this processor. Define here explicitly.
 * Using stdint.h as the rts2800_fpu32.lib is currently being used. When this is converted from an
 * R&D project over to cert we will eliminate the library and re-enter these integer types here. */
typedef unsigned char         bool_t;
//typedef _Bool                   bool_t;
#define DEFINED_TYPEDEF_FOR_bool_t_
typedef char                    char16_t;
#define DEFINED_TYPEDEF_FOR_char16_t_
typedef unsigned char           uchar16_t;
#define DEFINED_TYPEDEF_FOR_uchar16_t_
typedef short int               int16_t;
#define DEFINED_TYPEDEF_FOR_int16_t_
typedef long int                int32_t;
#define DEFINED_TYPEDEF_FOR_int32_t_
typedef long long int           int64_t;
#define DEFINED_TYPEDEF_FOR_int64_t_
typedef unsigned short int      uint16_t;
#define DEFINED_TYPEDEF_FOR_uint16_t_
typedef unsigned long int       uint32_t;
#define DEFINED_TYPEDEF_FOR_uint32_t_
typedef unsigned long long int  uint64_t;
#define DEFINED_TYPEDEF_FOR_uint64_t_
typedef float                   float32_t;
#define DEFINED_TYPEDEF_FOR_float32_t_
typedef long double             float64_t;
#define DEFINED_TYPEDEF_FOR_float64_t_

//typedef volatile uint32_t    REG32;

/******************** DEFINES ******************/
#define BITS_PER_WORD          16U
#define BYTES_PER_WORD         (BITS_PER_WORD / 8U)
#define NUM_BYTES_PER_FLOAT    (4U)
#define NUM_BYTES_PER_U32      (4U)
/******************** ENUMS ********************/
/* Boolean */

typedef enum
{
    False,
    True
} Boolean_e;

typedef enum
{
    DISABLE,
    ENABLE      /* Do not expand this enum */
} SWITCH;

typedef enum
{
    DISABLED,
    ENABLED      /* Do not expand this enum */
} SWITCHED;

typedef enum
{
    CHANNEL_A = 0,
    CHANNEL_B,
    NUM_CHANNELS,
    INVALID_CHANNEL_ID = NUM_CHANNELS
} eChId_t;

typedef enum
{
    LEFT_ACTUATOR = 0,
    RIGHT_ACTUATOR,
    INVALID_ACTUATOR_ID
} eActId_t;

typedef enum
{
    ONSIDE = 0,
    XSIDE,
    NUMBER_OF_SIDES
} eSide_t;

typedef enum
{
    SCU_LRU = 0,    /* SYNC CONTROLLER LRU */
    MCU_LIB_LRU,    /* LEFT INBOARD MOTOR CONTROL UNIT LRU ADDRESS */
    MCU_RIB_LRU,    /* RIGHT INBOARD MOTOR CONTROL UNIT LRU ADDRESS*/
    MCU_LOB_LRU,    /* LEFT OUTBOARD MOTOR CONTROL UNIT LRU ADDRESS*/
    MCU_ROB_LRU,    /* RIGHT OUTBOARD MOTOR CONTROL UNIT LRU ADDRESS*/
    INVALID_LRU,    /* NO LRU ADDRESSES FROM THIS POINT ON */
    GSE_ID = 15U    /* GSE ID IS 0xF */
} eLruId_t;  /* FLAP PANEL CONTROL SYSTEM LRU TYPES */

typedef enum
{
    LIB_PANEL = 0,      /* LEFT INBOARD PANEL */
    RIB_PANEL,          /* RIGHT INBOARD PANEL */
    LOB_PANEL,          /* LEFT OUTBOARD PANEL */
    ROB_PANEL,          /* RIGHT OUTBOARD PANEL */
    NUM_PANELS
} ePanel_t;  /* FLAP PANEL CONTROL SYSTEM LRU TYPES */

typedef enum
{
    ACTUATOR_1 = 0,      /* INBOARD PANEL RIGHT ACTUATOR */
    ACTUATOR_2,          /* INBOARD PANEL LEFT ACTUATOR */
    ACTUATOR_3,          /* OUTBOARD PANEL RIGHT ACTUATOR */
    ACTUATOR_4,          /* OUTBOARD PANEL LEFT ACTUATOR*/
    NUM_ACTUATOR_TYPES,
    INVALID_ACTUATOR = NUM_ACTUATOR_TYPES
} eActuator_t;  /* FLAP PANEL CONTROL SYSTEM ACTUATOR TYPES */

/* 32-bit Bit Description */
typedef struct
{
    uint16_t b0:1;
    uint16_t b1:1;
    uint16_t b2:1;
    uint16_t b3:1;
    uint16_t b4:1;
    uint16_t b5:1;
    uint16_t b6:1;
    uint16_t b7:1;
    uint16_t b8:1;
    uint16_t b9:1;
    uint16_t b10:1;
    uint16_t b11:1;
    uint16_t b12:1;
    uint16_t b13:1;
    uint16_t b14:1;
    uint16_t b15:1;
} t16Bits;

typedef union
{
    uint16_t  all;
    t16Bits   bit;
} t16ByteBits;

/* 32-bit Bit Description */
typedef struct
{
    uint16_t b0:1;
    uint16_t b1:1;
    uint16_t b2:1;
    uint16_t b3:1;
    uint16_t b4:1;
    uint16_t b5:1;
    uint16_t b6:1;
    uint16_t b7:1;
    uint16_t b8:1;
    uint16_t b9:1;
    uint16_t b10:1;
    uint16_t b11:1;
    uint16_t b12:1;
    uint16_t b13:1;
    uint16_t b14:1;
    uint16_t b15:1;
    uint16_t b16:1;
    uint16_t b17:1;
    uint16_t b18:1;
    uint16_t b19:1;
    uint16_t b20:1;
    uint16_t b21:1;
    uint16_t b22:1;
    uint16_t b23:1;
    uint16_t b24:1;
    uint16_t b25:1;
    uint16_t b26:1;
    uint16_t b27:1;
    uint16_t b28:1;
    uint16_t b29:1;
    uint16_t b30:1;
    uint16_t b31:1;
} t32Bits;

typedef union
{
    uint32_t  all;
    t32Bits   bit;
} t32ByteBits;

typedef struct
{
    uint16_t    Normal_Op_State:1;       /* b0  */
    uint16_t    Spare_b01:1;             /* b1  */
    uint16_t    PBIT_Passed:1;           /* b2  */
    uint16_t    Ground_Maint_State:1;    /* b3  */
    uint16_t    SCU_Bus_Input_Fault:1;   /* b4  */
    uint16_t    MCU_Engaged:1;           /* b5  */
    uint16_t    MCU_Control_Valid:1;     /* b6  */
    uint16_t    SCU_Bus_Valid:1;         /* b7  */
    uint16_t    Spare_b08:1;             /* b8  */
    uint16_t    Flap_In_Motion:1;        /* b9  */
    uint16_t    Brake_Engaged:1;         /* b10 */
    uint16_t    Rig_Status:2;            /* b11 */
    uint16_t    Spare_b13:1;             /* b13 */
    uint16_t    Spare_b14:1;             /* b14 */
    uint16_t    Spare_b15:1;             /* b15 */
} tMcuStatusBits;

typedef struct
{
    uint16_t    Normal_Op_State:1;       /* b0  */
    uint16_t    Spare_b01:1;             /* b1  */
    uint16_t    PBIT_Passed:1;           /* b2  */
    uint16_t    Ground_Maint_State:1;    /* b3  */
    uint16_t    Maint_Bus_Input_Fault:1; /* b4  */
    uint16_t    MCU_Engaged:1;           /* b5  */
    uint16_t    MCU_Control_Valid:1;     /* b6  */
    uint16_t    Maint_Bus_Valid:1;       /* b7  */
    uint16_t    Spare_b08:1;             /* b8  */
    uint16_t    Flap_In_Motion:1;        /* b9  */
    uint16_t    Brake_Engaged:1;         /* b10 */
    uint16_t    Rig_Status:2;            /* b11 */
    uint16_t    Rig_ESL_Rigged:1;        /* b13 */
    uint16_t    Rig_RSL_Rigged:1;        /* b14 */
    uint16_t    Rig_Verify_Ready:1;      /* b15 */
} tRigStatusBits;

typedef union
{
    uint16_t  all;
    tMcuStatusBits  bit;        /* Normal Mode Status Bit Definition */
    tRigStatusBits  bRig;       /* Maintenance Mode Rig Status Bit Definition */
    //tTestStatusBits bTest;    /* Maintenance Mode Test Status Bit Definition */
} tMcuStatus_t;

typedef struct
{
    uint16_t    Sensor_Fusion_Covariance:1;   /* b0  */
    uint16_t    Spare_b01:1;                  /* b1  */
    uint16_t    Sensor_Fault:1;               /* b2  */
    uint16_t    Flap_Jam_Active:1;            /* b3  */
    uint16_t    Spare_04:1;                   /* b4  */
    uint16_t    Spare_05:1;                   /* b5  */
    uint16_t    Spare_06:1;                   /* b6  */
    uint16_t    Spare_07:1;                   /* b7  */
    uint16_t    Spare_08:1;                   /* b8  */
    uint16_t    Motor_Brake_Fault:1;          /* b9  */
    uint16_t    Spare_b10:1;                  /* b10 */
    uint16_t    Spare_b11:1;                  /* b11 */
    uint16_t    Spare_b12:1;                  /* b12 */
    uint16_t    Spare_b13:1;                  /* b13 */
    uint16_t    Spare_b14:1;                  /* b14 */
    uint16_t    Spare_b15:1;                  /* b15 */
} tMcuFaultBits;

typedef union
{
    uint16_t  all;
    tMcuFaultBits  bit;
} tMcuFault_t;

typedef struct
{
    uint16_t    Spare_b00:1;                /* b0  */
    uint16_t    Spare_b01:1;                /* b1  */
    uint16_t    RVDT_Position_Fault:1;      /* b2  */
    uint16_t    RVDT_Excitation_Fault:1;    /* b3  */
    uint16_t    Hall_Sensor_Fault:1;        /* b4  */
    uint16_t    Brake_Hold_Fault:1;         /* b5  */
    uint16_t    Brake_Drive_Fault:1;        /* b6  */
    uint16_t    Spare_b07:1;                /* b7  */
    uint16_t    Spare_b08:1;                /* b8  */
    uint16_t    Spare_b09:1;                /* b9  */
    uint16_t    Spare_b10:1;                /* b10 */
    uint16_t    Spare_b11:1;                /* b11 */
    uint16_t    Spare_b12:1;                /* b12 */
    uint16_t    Spare_b13:1;                /* b13 */
    uint16_t    Spare_b14:1;                /* b14 */
    uint16_t    Spare_b15:1;                /* b15 */
} tMcuMaintBits;

typedef struct
{
    uint16_t    Spare_b00:1;                /* b0  */
    uint16_t    Spare_b01:1;                /* b1  */
    uint16_t    RVDT_Position_Fault:1;      /* b2  */
    uint16_t    RVDT_Excitation_Fault:1;    /* b3  */
    uint16_t    Hall_Sensor_Fault:1;        /* b4  */
    uint16_t    Brake_Hold_Fault:1;         /* b5  */
    uint16_t    Brake_Drive_Fault:1;        /* b6  */
    uint16_t    Spare_b07:1;                /* b7  */
    uint16_t    Fault_Code:8;               /* b8-b15: Fault Code (8-bit)   */
} tRigMaintBits;

typedef union
{
    uint16_t  all;
    tMcuMaintBits  bit;
    tRigMaintBits  bRig;
    //tTestMaintBits bTest;
} tMcuMaint_t;

typedef struct
{
    uint16_t  u8_0 : 8U;      /* byte 0 */
    uint16_t  u8_1 : 8U;      /* byte 1 */
    uint16_t  u8_2 : 8U;      /* byte 2 */
    uint16_t  u8_3 : 8U;      /* byte 3 */
} t32u8_t;

/* Unsignd 16-bit values within a 32-bity memory address */
typedef struct
{
    uint16_t u16_0;
    uint16_t u16_1;
} t32u16_t;

/* Unsigned 8-bit values within a 64-bit memory address */
typedef struct
{
    uint16_t  u8_0 : 8U;      /* byte 0 */
    uint16_t  u8_1 : 8U;      /* byte 1 */
    uint16_t  u8_2 : 8U;      /* byte 2 */
    uint16_t  u8_3 : 8U;      /* byte 3 */
    uint16_t  u8_4 : 8U;      /* byte 4 */
    uint16_t  u8_5 : 8U;      /* byte 5 */
    uint16_t  u8_6 : 8U;      /* byte 6 */
    uint16_t  u8_7 : 8U;      /* byte 7 */
} t64u8_t;

/* Unsigned 16-bit values within a 64-bit memory address */
typedef struct
{
    uint16_t u16_0;
    uint16_t u16_1;
    uint16_t u16_2;
    uint16_t u16_3;
} t64u16_t;

/* Signed 16-bit values within a 64-bit memory address */
typedef struct
{
    int16_t s16_0;
    int16_t s16_1;
    int16_t s16_2;
    int16_t s16_3;
} t64s16_t;

/* Unsigned 32-bit values within a 64-bit memory address */
typedef struct
{
    uint32_t u32_0;
    uint32_t u32_1;
} t64u32_t;

/* Signed 32-bit values within a 64-bit memory address */
typedef struct
{
    int32_t s32_0;
    int32_t s32_1;
} t64s32_t;

/* Float 32-bit values within a 64-bit memory address */
typedef struct
{
    float32_t f32_0;
    float32_t f32_1;
} t64f32_t;

/* 64-bit Address Types Union */
typedef union
{
    uint64_t        u64;    /* uint64_t         */
    int64_t         s64;    /* int64_t          */
    float64_t       f64;    /* float64_t        */
    t64u32_t        tu32;   /* uint32_t types   */
    t64s32_t        ts32;   /* int32_t types    */
    t64f32_t        tf32;   /* float32_t types  */
    t64u16_t        tu16;   /* uint16_t types   */
    t64s16_t        ts16;   /* int16_t types    */
    t64u8_t         tu8;    /* uint8_t types    */
} t64Types_t;

typedef struct
{
    uint16_t u16Major;
    uint16_t u16Minor;
    uint16_t u16Revision;
    uint16_t u16Build;
} tSwVersion_t;

#endif
