/*
###########################################################################

 FILE:	IQmathLib.h

 TITLE:	IQ Math library functions definitions.

###########################################################################

 Ver  | dd-mmm-yyyy |  Who  | Description of changes
 =====|=============|=======|==============================================
  1.3 | 19 Nov 2001 | A. T. | Original Release.
 -----|-------------|-------|----------------------------------------------
  1.4 | 17 May 2002 | A. T. | Added new functions and support for
      |             |       | intrinsics IQmpy, IQxmpy, IQsat.
 -----|-------------|-------|----------------------------------------------
  1.4a| 12 Jun 2002 | A. T. | Fixed problem with _IQ() operation on
      |             |       | variables.
 -----|-------------|-------|----------------------------------------------
  1.4b| 18 Jun 2002 | A. T. | Fixed bug with _IQtoIQN() and _IQNtoIQ()
      |             |       | operations.
 -----|-------------|-------|----------------------------------------------
  1.5 | 19 Nov 2008 | K. D. | Removed unused IQMath macros
 -----|-------------|-------|----------------------------------------------
  1.6 | 18 Aug 2009 | K. D. | Updated per PR6590

########################################################################### 
*/

/* Don't allow multiple instances of definitions */
#ifndef _IQMATHLIB_H
#define _IQMATHLIB_H

//Define system Math Type
// Select Floating Math Type for 2837xD Delfino Types
#define MATH_TYPE 1 /* Floating Point Processor */

/* Select global Q value of 24 for all functions */
#define   GLOBAL_Q       24 	

/* Make all _iq types 32 bits */
typedef   long    _iq;
typedef   long    _iq18;
typedef   long    _iq13;
typedef   long    _iq8;

/* Conversion factors */
#define   _IQ24(A)      (long) (A * 16777216.0L)
#define   _IQ18(A)      (long) (A * 262144.0L)
#define   _IQ13(A)      (long) (A * 8192.0L)
#define   _IQ8(A)       (long) (A * 256.0L)

/* Default conversion factor (IQ24) */
#define   _IQ(A)  _IQ24(A)

/* Macro definition for assy call to saturation check */
#define   _IQsat(A, Pos, Neg)  __IQsat(A, Pos, Neg)

/* Macros for conversione */
#define   _IQ18toIQ(A)  ((long) A << (GLOBAL_Q - 18))
#define   _IQtoIQ15(A)  ((long) A >> (GLOBAL_Q - 15))

/* Macro definition for assy calls to multiply functions */ 
#define   _IQ18mpy(A,B)  __IQmpy(A,B,18)
#define   _IQ8mpy(A,B)   __IQmpy(A,B,8)
#define   _IQmpy(A,B)    __IQmpy(A,B,GLOBAL_Q)

/* Macro definition for assy calls to divide functions */ 
#define   _IQdiv(A,B)  _IQ24div(A,B)

/* extern definitions to make functions accessible*/
extern    long _IQ24div(long A, long B);
extern    long _IQ18div(long A, long B);
extern    long _IQ13div(long A, long B);
extern    long _IQ8div(long A, long B);
extern    long _IQ24int(long A);
extern    long _IQ13int(long A);
extern    long _IQ8int(long A);


#define   _IQint(A)  _IQ24int(A)

#endif  /* No more (_IQMATHLIB_H) */

