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

	LDR R3, GPIOA_BASE @ initializes input register to R3
	LDR R3, [R3, #0x10] @ loads the address of the inout register with offset to R3

	STR R2, [R1, #0x14] @ updates LEDs while no button is pressed

	@ For when SW0 and SW1 are pressed togehter, this will cause fast +2
	LDR R5, =3
	MOV R7, R3 @ makes a copy of R5 into R7
	ANDS R7, R5 @ logic AND between R7 and R5, update Z flag
	CMP R7, #0 @ compares R7 and 0
	BEQ fast_increment @ branches to the function which increments by 2 and decreases the delay

	@ For when SW0 is held, causing default time +2
	LDR R5, =1
	MOV R7, R3
	ANDS R7, R5
	CMP R7, #0
	BEQ long_delay

	@ For when SW1 is held, causing fast +1
	LDR R5, =2
	MOV R7, R3
	ANDS R7, R5
	CMP R7, #0
	BEQ fast_delay

	@ For when SW2 is held, causing 0xAA to be displayed
	LDR R5, =4
	MOV R7, R3
	ANDS R7, R5
	CMP R7, #0
	BEQ sw2_press

	@ For when SW3 is held, causing the pattern to freeze
	LDR R5, =8
	MOV R7, R3
	ANDS R7, R5
	CMP R7, #0
	BEQ sw3_press

	LDR R4, LONG_DELAY_CNT @ default delay value
	B delay_1

fast_delay:
	LDR R4, SHORT_DELAY_CNT @ loads the short delay value into R4
	B delay_1

long_delay:
	LDR R4, LONG_DELAY_CNT @ loads the long delay value into R4
	B delay_2

delay_1:
	SUBS R4, #1 @ subtracts 1 from R4
	BNE delay_1 @ once R4 = 0, Z flag = 1 and exits loop

	ADDS R2, #1 @ increments R2 by 1
	B main_loop

delay_2:
	SUBS R4, #1
	BNE delay_2

	ADDS R2, #2 @ increments R2 by 2
	B main_loop

fast_increment:
	LDR R4, SHORT_DELAY_CNT
	SUBS R4, #1
	BNE delay_2

	ADDS R2, #2
	B main_loop

sw2_press:  @ This just forces the 0b101010 into R2, then once SW2 depressed continues counting as normal.
			@ It works by just branching into this event and remaining there until the button is depressed,
			@ and it also sets the LED register to be 0xAA, so it can continue counting from there
	LDR R2, = 0xAA
	STR R2, [R1, #0x14]

sw3_press:
	@
	STR R2, [R1, #0x14] @Stores the current LED pattern located at address [R1+0x14] into the register R2
	B main_loop

LDR R4, LONG_DELAY_CNT
B delay_1

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
