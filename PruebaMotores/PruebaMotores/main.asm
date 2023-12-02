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

  loop:
  rjmp loop



configPuertos:
  ldi	r16, (1 << PB2) | (1 << PB1)
  out	DDRB, r16
  
  ldi	r16, (1 << PD5) | (1 << PD6)
  out	DDRD, r16
  
  /*
  cbi PORTB, PB2
  cbi PORTB, PB1
  
  cbi PORTD, PD5
  sbi PORTD, PD6
  */
ret


configPWM:


  ;Configuración timer 1
  ldi	r16, (0 << WGM11) | (1 << WGM10) | (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0)
  sts	TCCR1A, r16

  ldi	r16, (0 << WGM13) | (0 << WGM12) | (1 << CS12) | (0 << CS11) | (0 << CS10)
  sts	TCCR1B, r16

  ldI r16, 0X00

  sts	OCR1BH, r16
  sts	OCR1AH, r16
  
  ldI r16, 0X00
  
  sts	OCR1BL, r16
  sts	OCR1AL, r16
  
  ;Configuración timer 0
  ldi	r16, (0 << WGM01) | (1 << WGM00) | (1 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0)
  out	TCCR0A, r16

  ldi	r16, (0 << WGM02) | (1 << CS02) | (0 << CS01) | (0 << CS00)
  out	TCCR0B, r16
  
  ldI r16, 25
  out	OCR0B, r16
  
  ldI r16, 00
  out	OCR0A, r16
 
 
  //SI DESCOMENTO LA CONFIGURACIÓN DEL TIMER 0 DEJA DE ANDAR EL TIMER 1
  //CUANDO CONFIGURO EL TIMER 0 SE ACTIVA LA SALIDA B DEL TIMER 1
ret