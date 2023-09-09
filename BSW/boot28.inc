;******************************************************************************
;* BOOT  v5.2.6
;* Copyright (c) 2022 - Eaton Aerospace Group, All Rights Reserved.
;******************************************************************************

;****************************************************************************
;* BOOT.ASM
;*
;* THIS IS THE INITAL BOOT ROUTINE FOR C28xx C++ PROGRAMS.
;* IT MUST BE LINKED AND LOADED WITH ALL C++ PROGRAMS.
;* 
;* THIS MODULE PERFORMS THE FOLLOWING ACTIONS:
;*   1) SET RESET VECTOR TO POINT AT _C_INT00
;*   2) ALLOCATES THE STACK AND INITIALIZES THE STACK POINTER
;*   3) SET UP PROPER STATUS
;*   4) PERFORMS AUTO-INITIALIZATION
;*   5) CALLS INITALIZATION ROUTINES FOR FILE SCOPE CONSTRUCTION
;*   6) CALLS THE FUNCTION MAIN TO START THE C++ PROGRAM
;*   7) CALLS THE STANDARD EXIT ROUTINE
;*
;* THIS MODULE DEFINES THE FOLLOWING GLOBAL SYMBOLS:
;*   1) __stack     STACK MEMORY AREA
;*   2) _c_int00    BOOT ROUTINE
;*
;****************************************************************************
CONST_COPY  .set 0

    .global  _c_int00, cinit, pinit
    .global  _main
    .global  __stack

****************************************************************************
* Declare the stack.  Size is determined by the linker option -stack.  The *
* default value is 1K words.                                               *
****************************************************************************
__stack:    .usect  ".stack",0

****************************************************************************
*  INITIALIZE RESET VECTOR TO POINT AT _c_int00                            *
****************************************************************************
    .sect .reset
    .long _c_int00

    .text
****************************************************************************
* FUNCTION DEF : _c_int00                                                  *
*                                                                          *
****************************************************************************

_c_int00:   .asmfunc
****************************************************************************
*  INITIALIZE STACK POINTER.                                               *
****************************************************************************
    MOV     SP,#__stack     ; set to beginning of stack space

****************************************************************************
* INITIALIZE STATUS BIT FIELDS *NOT* INITIALIZED AT RESET                  *
****************************************************************************
    SPM 0           ; set product shift to 0

****************************************************************************
* SET C28x MODES                                                           *
****************************************************************************
        C28OBJ                          ; select C28x object mode
        C28ADDR                         ; clear the addressing mode
        C28MAP                          ; set block M0 and M1 mode

        .if .TMS320C2800_FPU32
        SETFLG            RNDF32=1      ; Enable rounding in FPU32 mode.
        .endif
****************************************************************************
* SETTING THESE STATUS BITS/REGISTER TO RESET VALUES.  IF YOU RUN          *
* _c_int00 FROM RESET, YOU CAN REMOVE THIS CODE                            *
****************************************************************************
    CLRC    PAGE0           ; use stack addressing mode
        MOVW    DP,#0           ; initialize DP to point at low 64K
    CLRC    OVM                 ; turn off overflow mode

    ASP             ; ensure SP is aligned

****************************************************************************
* SET THE EALLOW BIT BEFORE THE CINIT TABLE IS COPIED.                     *
****************************************************************************
    EALLOW

****************************************************************************
*  IF cinit IS NOT -1, PROCESS CINIT INITIALIZATION TABLE              *
****************************************************************************
    MOV AL,#cinit
    MOV AH,#hi16(cinit)
    ADDB    ACC,#1
    B   DO_PINIT,EQ     ; if cinit < 0 (-1) no init tables

****************************************************************************
*  PROCESS CINIT INITIALIZATION TABLE.  TABLE IS IN PROGRAM MEMORY IN THE  *
*  FOLLOWING FORMAT:                                                       *
*                                                                          *
*       .word  <length of init data in words>                              *
*       .word  or .long <address of variable to initialize>                *
*       .word  <init data>                                                 *
*       .word  ...                                                         *
*                                                                          *
*  If the variable's address is greater than 65535 (located in 'far'       *
*  memory), then the address field of the cinit record will be 32-bits     *
*  instead of the default 16-bits. The length value is negated to tag      *
*  cinit records for those variables located in far memory.                *
*                                                                          *
*  The init table is terminated with a zero length                         *
*                                                                          *
****************************************************************************
    MOVL    XAR7,#cinit     ; point XAR7 at start of table  
    CLRC    TC              ; reset TC bit used as far flag 
    B   START, UNC      ; jump to start processing
LOOP:
    MOVB    AH,#0               ; zero out upper addr bits
    PREAD   AL,*XAR7        ; load address of variable to be inited
    ADDB    XAR7,#1         ; point to initialization data
        B   GET_DATA,NTC            ; get data if variable is not far 
    CLRC    TC              ; reset TC bit used as far flag 
    PREAD   AH,*XAR7            ; otherwise, get hi bits of 22-bit addr
    ADDB    XAR7,#1
GET_DATA:   
    MOVL    XAR6,ACC            ; address
    RPT AR1         ; repeat length + 1 times
||  PREAD   *XAR6++,*XAR7       ; copy data from table to memory
    
    MOVL    ACC,XAR7        ; using ACC as temp, point XAR7 to 
    ADD     ACC,AR1         ; next cinit record since PREAD 
    ADDB    ACC,#1          ; doesn't change value of XAR7. 
    MOVL    XAR7,ACC    
START:
    PREAD   AL,*XAR7        ; load length
    B   GET_ADDR,GEQ            ; a length < 0 denotes far data  
        NEG     AL              ; negate value to get real length   
    SETC    TC              ; flag that the address field is 32-bits
GET_ADDR:   
    MOVZ    AR1,AL              ; copy length value to loop register
        ADDB    XAR7,#1         ; point to address field
    BANZ    LOOP,AR1--      ; if (length-- != 0) continue 

****************************************************************************
*  IF pinit IS NOT -1, PROCESS CONSTRUCTOR ROUTINES                        *
****************************************************************************
DO_PINIT:           

****************************************************************************
* CLEAR THE EALLOW BIT AFTER THE CINIT TABLE IS COPIED.                    *
****************************************************************************
    EDIS

    MOV AL,#pinit
    MOV AH,#hi16(pinit)
    ADDB    ACC,#1
    B   DONE_INIT,EQ        ; if pinit < 0 (-1) no pinit table

****************************************************************************
*  PROCESS PINIT SECTION. TABLE CONSISTS OF CONSTRUCTOR ROUTINE ADDRESSES  *
*  THAT POINT TO C++ INITIALIZATION FUNCTIONS. THESE ROUTINES MUST EXECUTE *
*  BEFORE MAIN IS CALLED                                                   *
****************************************************************************
    MOVL    XAR7,#pinit         ; set up pointer to initialization table
    B   START_PINIT, UNC        ; jump to start processing

LOOP_PINIT: 
    ADDB    XAR7,#1         ; point to next table entry 
    PUSH    XAR7            ; save off table entry address 
    MOVL    XAR7,ACC        ; load constructor routine address
    LCR     *XAR7           ; execute routine
    POP XAR7            ; reload table entry address 
    
START_PINIT:
    PREAD   AL,*XAR7            ; unpack constructor routine address
    ADDB    XAR7,#1
    PREAD   AH,*XAR7
    TEST    ACC         ; test for table end condition (zero)
    B   LOOP_PINIT,NEQ      ; process table entry if not zero


****************************************************************************
*  COPY CONSTANTS TO DATA MEMORY, IF NECESSARY                             *
****************************************************************************
DONE_INIT:

****************************************************************************
*  CALL USER'S PROGRAM                                                     *
****************************************************************************
    LCR     _main
    .endasmfunc
