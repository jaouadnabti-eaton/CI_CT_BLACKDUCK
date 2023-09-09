;/*
;    File name: stackpointer.asm
;
;    Purpose: Assembly file that returns the address of the stack pointer
;
;	 Copyright 2022, Eaton Aerospace.
;    All rights reserved.
;*/
		;	.include "path_macro.asm"
	.global  _stack_pointer ; beginning _stack_pointer
;PATHA: .string "stack_pointer_A",0	 
;PATHB: .string "stack_pointer_B",0
        .text
_stack_pointer:
;	PATH	PATHA			;Added for Path coverage analysis 
	mov	ACC,SP              ; Load the accumulator with the current address of the stack pointer
;	PATH	PATHB			;Added for Path coverage analysis
	lretr			        ; Return the address of the stack pointer
		.end				; ending _stack_pointer
