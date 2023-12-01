.include "m328pdef.inc"

.equ VAR_SUAVE_VALOR = 5
.equ VAR_FUERTE_VALOR = 30 
.equ VEL_MAX = 80
.equ VEL_MAX_EXTREMOS = 255
.equ VEL_REVERSA = 25

.equ VAL_IZQ_2 = PC0 //0x01  ; 0 0 0 0 0 0 0 1 
.equ VAL_IZQ_1 = PC1 //0x02  ; 0 0 0 0 0 0 1 0   
.equ VAL_CENTRO = PC2 //0x04 ; 0 0 0 0 0 1 0 0
.equ VAL_DER_1 = PC3 //0x08  ; 0 0 0 0 1 0 0 0 
.equ VAL_DER_2 = PC4 //0x10  ; 0 0 0 1 0 0 0 0  

.equ BLANCO = 1
.equ NEGRO = 0

.def VEL_MOTOR_DER = r19
.def VEL_MOTOR_IZQ = r20
.def VEL_MOTOR_DER_AUX = r21
.def VEL_MOTOR_IZQ_AUX = r22
.def VAR_SUAVE = r16   ;cuando el giro debe ser fuerte la vel sube en un 10%
.def VAR_FUERTE = r17  ;cuando el giro debe ser suave la vel sube en un 2%
.def VAL_LEIDO = r18
.def VEL_MOTOR_REV_DER = R23
.def VEL_MOTOR_REV_IZQ = R24

.cseg 
.org 0x00
  jmp	main

.org INT_VECTORS_SIZE

main:

  ldi r16, HIGH(RAMEND) ;Se inicializa el stack pointer
  out SPH, r16
  ldi r16, LOW(RAMEND)
  out SPL, r16

  ldi VAR_SUAVE, VAR_SUAVE_VALOR
  ldi VAR_FUERTE, VAR_FUERTE_VALOR
  
  call config_puertos
  call configPWM

  wait:
    sbic PINB, PB0 ; Espera a que se pulse el botón
		rjmp wait

  //call delay_inicial

  //call encenderLed
  ldi VEL_MOTOR_DER, VEL_MAX
  ldi VEL_MOTOR_IZQ, VEL_MAX
  call actualizarVelocidad 


  loop:
    in VAL_LEIDO, PINC    ;leo valores de los sensores y dependiendo de que leyeron giro o sigo en linea recta
	
    sbic PINC, VAL_IZQ_2
    call doblar_izq_fuerte

    sbic PINC, VAL_DER_2
    call doblar_der_fuerte

    sbic PINC, VAL_IZQ_1
    call doblar_izq_suave

    sbic PINC, VAL_DER_1
    call doblar_der_suave
	
    sbic PINC, VAL_CENTRO
    call sigo_centro

    call actualizarVelocidad 

  rjmp loop

sigo_centro: ;voy a toda vel 
  call apagarLed
  ldi VEL_MOTOR_DER, VEL_MAX
  ldi VEL_MOTOR_IZQ, VEL_MAX
  ldi VEL_MOTOR_REV_DER, 0X00
  ldi VEL_MOTOR_REV_IZQ, 0X00
ret

doblar_izq_fuerte:
  call encenderLed
  /*
  ldi VEL_MOTOR_DER, VEL_MAX_EXTREMOS
  ldi VEL_MOTOR_IZQ, 0 
  ldi VEL_MOTOR_REV_IZQ, VEL_REVERSA
*/
  mov VEL_MOTOR_DER_AUX,VEL_MOTOR_DER

  cpi VEL_MOTOR_DER, VEL_MAX ; 1° veo si VEL_MOTOR_DER = 255
        breq red_fuerte_vel_izquierda ;si VEL_MOTOR_DER = 255 debo reducir la vel_motor_izq

  add VEL_MOTOR_DER_AUX, VAR_FUERTE
  cpi VEL_MOTOR_DER_AUX, VEL_MAX ; ¿vel_motor_der = vel_motor_der + var_fuerte > 255 (vel max)?
        brlo subo_fuerte_vel_derecha ;salta si la suma es menor

  cpi VEL_MOTOR_DER, VEL_MAX
        brsh mot_der_vel_max
   ret

  subo_fuerte_vel_derecha:
    add VEL_MOTOR_DER, VAR_FUERTE
	ret

  red_fuerte_vel_izquierda:
    cpi VEL_MOTOR_IZQ, VAR_FUERTE_VALOR
          brsh decre_fuerte_vel_mot_izq        ;si la vel izq es mayor a la var suave, puedo decrementar su valor
    cpi VEL_MOTOR_IZQ, VAR_FUERTE_VALOR
          brlo mot_izq_rev_der_max
    ret

  decre_fuerte_vel_mot_izq:
     sub VEL_MOTOR_IZQ,VAR_FUERTE 
  ret

  mot_izq_rev_der_max:
     ldi VEL_MOTOR_IZQ, 0x00
     ldi VEL_MOTOR_REV_IZQ, VEL_REVERSA
     ldi VEL_MOTOR_DER, VEL_MAX_EXTREMOS
  ret

  
ret             

doblar_izq_suave:
  call apagarLed
/*  ldi VEL_MOTOR_DER, VEL_MAX
  ldi VEL_MOTOR_IZQ, 0 
  */

  cpi VEL_MOTOR_IZQ, VEL_REVERSA
  breq desactivar_giro_extremo_izq

  rjmp fin_desactivar_giro_extremo_izq

  desactivar_giro_extremo_izq:
    ldi VEL_MOTOR_REV_IZQ, 0x00
    ldi VEL_MOTOR_REV_DER, 0x00
    
    ldi VEL_MOTOR_IZQ, 0x00
    ldi VEL_MOTOR_DER, VEL_MAX
  ret

  fin_desactivar_giro_extremo_izq:

  mov VEL_MOTOR_DER_AUX,VEL_MOTOR_DER

  cpi VEL_MOTOR_DER, VEL_MAX ; 1° veo si VEL_MOTOR_DER = 255
        breq  red_suave_vel_izquierda;si VEL_MOTOR_DER = 255 debo reducir la vel_motor_izq

  add VEL_MOTOR_DER_AUX, VAR_SUAVE
  cpi VEL_MOTOR_DER_AUX, VEL_MAX ; ¿vel_motor_der = vel_motor_der + var_fuerte > 255 (vel max)?
        brlo subo_suave_vel_derecha ;salta si la suma es menor

  cpi VEL_MOTOR_DER, VEL_MAX
        brsh mot_der_vel_max
  ret

  subo_suave_vel_derecha:
    add VEL_MOTOR_DER, VAR_SUAVE
	ret

  red_suave_vel_izquierda:
    cpi VEL_MOTOR_IZQ, VAR_SUAVE_VALOR
          brsh decre_suave_vel_mot_izq        ;si la vel izq es mayor a la var suave, puedo decrementar su valor
    cpi VEL_MOTOR_IZQ, VAR_SUAVE_VALOR
          brlo mot_izq_vel_nula
    ret

  decre_suave_vel_mot_izq:
     sub VEL_MOTOR_IZQ,VAR_SUAVE 
  ret
  
  mot_izq_vel_nula:
     ldi VEL_MOTOR_IZQ, 0x00
  ret

ret             

mot_der_vel_max:
     ldi VEL_MOTOR_DER, VEL_MAX
   ret

doblar_der_fuerte:
  call encenderLed
  /*ldi VEL_MOTOR_DER, 0
  ldi VEL_MOTOR_IZQ, VEL_MAX_EXTREMOS
  ldi VEL_MOTOR_REV_DER, VEL_REVERSA
*/
  mov VEL_MOTOR_IZQ_AUX,VEL_MOTOR_IZQ

  cpi VEL_MOTOR_IZQ, VEL_MAX ; 1° veo si VEL_MOTOR_IZQ = 255
        breq red_fuerte_vel_derecha ;si VEL_MOTOR_IZQ = 255 debo reducir la vel_motor_der

  add VEL_MOTOR_IZQ_AUX, VAR_FUERTE
  cpi VEL_MOTOR_IZQ_AUX, VEL_MAX ; ¿vel_motor_izq = vel_motor_izq + var_fuerte > 255 (vel max)?
        brlo subo_fuerte_vel_izquierda ;salta si la suma es menor

  cpi VEL_MOTOR_IZQ, VEL_MAX
        brsh mot_izq_vel_max
  ret

  subo_fuerte_vel_izquierda:
    add VEL_MOTOR_IZQ, VAR_FUERTE
	ret

  red_fuerte_vel_derecha:
    cpi VEL_MOTOR_DER, VAR_FUERTE_VALOR
          brsh decre_fuerte_vel_mot_der       ;si la vel der es mayor a la var suave, puedo decrementar su valor
    cpi VEL_MOTOR_DER, VAR_FUERTE_VALOR
          brlo mot_der_rev_izq_max
    ret

  decre_fuerte_vel_mot_der:
     sub VEL_MOTOR_DER,VAR_FUERTE 
  ret

  mot_der_rev_izq_max:
     ldi VEL_MOTOR_DER, 0x00
     ldi VEL_MOTOR_REV_DER, VEL_REVERSA
     ldi VEL_MOTOR_IZQ, VEL_MAX_EXTREMOS
   ret

ret

doblar_der_suave:
  call apagarLed
/*
  ldi VEL_MOTOR_DER, 0
  ldi VEL_MOTOR_IZQ, VEL_MAX
  */

  cpi VEL_MOTOR_DER, VEL_REVERSA
  breq desactivar_giro_extremo_der

  rjmp fin_desactivar_giro_extremo_der

  desactivar_giro_extremo_der:
    ldi VEL_MOTOR_REV_IZQ, 0x00
    ldi VEL_MOTOR_REV_DER, 0x00
    
    ldi VEL_MOTOR_DER, 0x00
    ldi VEL_MOTOR_IZQ, VEL_MAX
  ret

  fin_desactivar_giro_extremo_der:

  mov VEL_MOTOR_IZQ_AUX,VEL_MOTOR_IZQ

  cpi VEL_MOTOR_IZQ, VEL_MAX ; 1° veo si VEL_MOTOR_IZQ = 255
        breq red_suave_vel_derecha ;si VEL_MOTOR_IZQ = 255 debo reducir la vel_motor_der

  add VEL_MOTOR_IZQ_AUX, VAR_SUAVE
  cpi VEL_MOTOR_IZQ_AUX, VEL_MAX ; ¿vel_motor_izq = vel_motor_izq + var_fuerte > 255 (vel max)?
        brlo subo_suave_vel_izquierda ;salta si la suma es menor

  cpi VEL_MOTOR_IZQ, VEL_MAX
        brsh mot_izq_vel_max
  ret

  subo_suave_vel_izquierda:
    add VEL_MOTOR_IZQ, VAR_SUAVE
  ret

  red_suave_vel_derecha:
    cpi VEL_MOTOR_DER, VAR_SUAVE_VALOR
          brsh decre_suave_vel_mot_der        ;si la vel der es mayor a la var suave, puedo decrementar su valor
    cpi VEL_MOTOR_DER, VAR_SUAVE_VALOR
          brlo mot_der_vel_nula
    ret

  decre_suave_vel_mot_der:
     sub VEL_MOTOR_DER,VAR_SUAVE 
  ret

  mot_der_vel_nula:
     ldi VEL_MOTOR_DER, 0x00     
   ret
ret  



mot_izq_vel_max:
     ldi VEL_MOTOR_IZQ, VEL_MAX
ret



detectar_lineas: ;si sucede que val_centro = 0 ---> la pista es blanca con raya negra
                 ;si sucede que val_centro = 1 ---> la pista es negra con raya blanca
				 ;USAR REG COMO FLAGS 
                ret

actualizarVelocidad:
  sts	OCR1BL, VEL_MOTOR_DER
  sts	OCR1AL, VEL_MOTOR_IZQ
  out	OCR0A, VEL_MOTOR_REV_DER
  out	OCR0B, VEL_MOTOR_REV_IZQ
ret

delay_inicial:  ;cinco seg 
  ldi r18, 0xFA
  ldi r19, 0xFA
  ldi r20, 0x9E
  L1: 
  dec r20
  brne L1
  dec r19
  brne L1
  dec r18
  brne L1
ret

config_puertos:
  push r16

  ldi r16, 0x00
  out DDRC,r16
              
  ldi r16, (0 << PB0) | (1 << PB1) | (1 << PB2)
  out DDRB,r16

              
  ldi r16,(1 << PD7) | (1 << PD5) | (1 << PD6)
  out DDRD,r16 ; 1 led conectado a PD7
  
  cbi PORTD, PD5 //Sacar para usar reversa
  cbi PORTD, PD6

  pop r16
ret


configPWM:
  push r16

  ;Configuración timer 1
  ldi	r16, (0 << WGM11) | (1 << WGM10) | (1 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0)
  sts	TCCR1A, r16

  ldi	r16, (0 << WGM13) | (0 << WGM12) | (1 << CS12) | (0 << CS11) | (0 << CS10)
  sts	TCCR1B, r16

  ldI r16, 0X00
  
  sts	OCR1BH, r16
  sts	OCR1BL, r16
    
  sts	OCR1AH, r16
  sts	OCR1AL, r16

  ;Configuración timer 0
  ldi	r16, (0 << WGM01) | (1 << WGM00) | (1 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0)
  out	TCCR0A, r16

  ldi	r16, (0 << WGM02) | (1 << CS02) | (0 << CS01) | (0 << CS00)
  out	TCCR0B, r16
  
  ldI r16, 0X00

  out	OCR0B, r16
  out	OCR0A, r16

  pop r16
ret

encenderLed:
  sbi PORTD, PD7
ret

apagarLed:
  cbi PORTD, PD7
ret