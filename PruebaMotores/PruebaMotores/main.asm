;
; ParteB.asm
;
; Created: 02/11/2023 0:07:09
; Author : tomas
;



.include "m328pdef.inc"

.cseg 
.org 0x00
  jmp	main


.org INT_VECTORS_SIZE

main:
			
  ;Inicializo stack pointer
  ldi r16, HIGH(RAMEND)
  out SPH, r16
  ldi r16, LOW(RAMEND)
  out SPL, r16

  call configPuertos
  call configPWM

  ldI r17, 0X00
  ldi	r16, 0x80
  
  sts	OCR1BH, r17
  sts	OCR1BL, r17
    
  sts	OCR1AH, r17
  sts	OCR1AL, r16

  loop:
  rjmp loop



configPuertos:
  ldi	r16, (1 << PB2) | (1 << PB1)
  out	DDRB, r16
ret


configPWM:
  ldi	r16, (0 << WGM11) | (1 << WGM10) | (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0)
  sts	TCCR1A, r16

  ldi	r16, (0 << WGM13) | (0 << WGM12) | (1 << CS12) | (0 << CS11) | (0 << CS10)
  sts	TCCR1B, r16

ret