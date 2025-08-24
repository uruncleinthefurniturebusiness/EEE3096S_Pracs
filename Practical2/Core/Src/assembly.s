/*
 * assembly.s
 *
 */
 
 @ DO NOT EDIT
	.syntax unified
    .text
    .global ASM_Main
    .thumb_func

@ DO NOT EDIT
vectors:
	.word 0x20002000
	.word ASM_Main + 1

@ DO NOT EDIT label ASM_Main
ASM_Main:

	@ Some code is given below for you to start with
	LDR R0, RCC_BASE  		@ Enable clock for GPIOA and B by setting bit 17 and 18 in RCC_AHBENR
	LDR R1, [R0, #0x14]
	LDR R2, AHBENR_GPIOAB	@ AHBENR_GPIOAB is defined under LITERALS at the end of the code
	ORRS R1, R1, R2
	STR R1, [R0, #0x14]

	LDR R0, GPIOA_BASE		@ Enable pull-up resistors for pushbuttons
	MOVS R1, #0b01010101
	STR R1, [R0, #0x0C]
	LDR R1, GPIOB_BASE  	@ Set pins connected to LEDs to outputs
	LDR R2, MODER_OUTPUT
	STR R2, [R1, #0]
	MOVS R2, #0         	@ NOTE: R2 will be dedicated to holding the value on the LEDs

@ TODO: Add code, labels and logic for button checks and LED patterns


main_loop:
	LDR R3, GPIOA_BASE
	LDR R3, [R3, #0x10]

	STR R2, [R1, #0x14] @ updates LEDs while no button is pressed

	LDR R5, =3
	MOV R7, R3
	ANDS R7, R5 @
	CMP R7, #0 @ compares R3 and 2 (switch 1), Z flag updated
	BEQ fast_increment

	LDR R5, =1
	MOV R7, R3
	ANDS R7, R5 @
	CMP R7, #0 @ compares R3 and 2 (switch 1), Z flag updated
	BEQ fast_delay_2

	LDR R5, =2 @ loads the value of 1 into R5
	MOV R7, R3
	ANDS R7, R5 @
	CMP R7, #0 @ compares R3 and 0 (switch 0), Z flag updated
	BEQ fast_delay


	LDR R5, =4 @ SWITCH 2
	MOV R7, R3
	ANDS R7, R5 @
	CMP R7, #0 @ compares R3 and 4 (switch 2), Z flag updated


	LDR R5, =8 @ SWITCH 3
	MOV R7, R3
	ANDS R7, R5 @
	CMP R7, #0 @ compares R3 and 8 (switch 3), Z flag updated

	LDR R4, LONG_DELAY_CNT
	B delay_1

fast_delay:
	LDR R4, SHORT_DELAY_CNT
	B delay_1

fast_delay_2:
	LDR R4, LONG_DELAY_CNT
	B delay_2

delay_1:
	SUBS R4, #1
	BNE delay_1

	ADDS R2, #1
	B main_loop

delay_2:
	SUBS R4, #1
	BNE delay_2

	ADDS R2, #2
	B main_loop

fast_increment:
	LDR R4, SHORT_DELAY_CNT
	SUBS R4, #1
	BNE delay_2

	ADDS R2, #2
	B main_loop

@ LITERALS; DO NOT EDIT
	.align
RCC_BASE: 			.word 0x40021000
AHBENR_GPIOAB: 		.word 0b1100000000000000000
GPIOA_BASE:  		.word 0x48000000
GPIOB_BASE:  		.word 0x48000400
MODER_OUTPUT: 		.word 0x5555

@ TODO: Add your own values for these delays
LONG_DELAY_CNT: 	.word 0x155CC0
SHORT_DELAY_CNT: 	.word 0x0927C0
