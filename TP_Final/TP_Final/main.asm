;
; TP3.asm
;
; Created: 22/11/2023 19:44:43
; Author : tomas
;

//Agregar inicio de programa

/*
  ldi r16, HIGH(RAMEND)
  out SPH, r16
  ldi r16, LOW(RAMEND)
  out SPL, r16
*/

.equ VAL_IZQ_2 = 0x01  ; 0 0 0 0 0 0 0 1 
.equ VAL_IZQ_1 = 0x02  ; 0 0 0 0 0 0 1 0   
.equ VAL_CENTRO = 0x04 ; 0 0 0 0 0 1 0 0
.equ VAL_DER_1 = 0x08  ; 0 0 0 0 1 0 0 0 
.equ VAL_DER_2 = 0x10  ; 0 0 0 1 0 0 0 0  

.equ BLANCO = 1
.equ NEGRO = 0

.equ VEL_MAX = 0xff
.def VEL_MOTOR_DER = r19
.def VEL_MOTOR_IZQ = r20
.def VEL_MOTOR_DER_AUX = r21
.def VEL_MOTOR_IZQ_AUX = r22
.def VAR_SUAVE = r16   ;cuando el giro debe ser fuerte la vel sube en un 10%
.def VAR_FUERTE = r17  ;cuando el giro debe ser suave la vel sube en un 2%
.def VAL_LEIDO = r18

ldi VAR_SUAVE, 0x05
ldi VAR_FUERTE, 25

loop:
  in VAL_LEIDO, PINC    ;leo valores de los sensores y dependiendo de que leyeron giro o sigo en linea recta
	
  cpi VAL_LEIDO, VAL_CENTRO
	breq sigo_centro
	
  cpi VAL_LEIDO, VAL_IZQ_2
	breq doblar_izq_fuerte
	
  cpi VAL_LEIDO, VAL_IZQ_1
	breq doblar_izq_suave
	
  cpi VAL_LEIDO, VAL_DER_2
	breq doblar_der_fuerte
	
  cpi VAL_LEIDO, VAL_DER_1
	breq doblar_der_suave
	
  rjmp loop

sigo_centro: ;voy a toda vel (255)
  ldi VEL_MOTOR_DER, VEL_MAX
  ldi VEL_MOTOR_IZQ, VEL_MAX		       
ret

doblar_izq_fuerte:

  mov VEL_MOTOR_DER_AUX,VEL_MOTOR_DER

  cpi VEL_MOTOR_DER, VEL_MAX ; 1° veo si VEL_MOTOR_DER = 255
        breq red_fuerte_vel_izquierda ;si VEL_MOTOR_DER = 255 debo reducir la vel_motor_izq

  add VEL_MOTOR_DER_AUX, VAR_FUERTE
  cpi VEL_MOTOR_DER_AUX, VEL_MAX ; ¿vel_motor_der = vel_motor_der + var_fuerte > 255 (vel max)?
        brlo subo_fuerte_vel_derecha ;salta si la suma es menor

  cpi VEL_MOTOR_DER, VEL_MAX
        brsh fijo_vel_derecha
   ret

  subo_fuerte_vel_derecha:
    add VEL_MOTOR_DER, VAR_FUERTE
	ret

  red_fuerte_vel_izquierda:
    cpi VEL_MOTOR_IZQ, VAR_FUERTE_VALOR
          brsh decre_fuerte_vel_mot_izq        ;si la vel izq es mayor a la var suave, puedo decrementar su valor
    cpi VEL_MOTOR_IZQ, VAR_FUERTE_VALOR
          brlo fijo_vel_mot_izq_f
    ret

   fijo_vel_derecha:
     ldi VEL_MOTOR_DER, VEL_MAX
   ret

   fijo_vel_mot_izq_f:
     ldi VEL_MOTOR_IZQ, 0x00 
   ret

   decre_fuerte_vel_mot_izq:
     sub VEL_MOTOR_IZQ,VAR_FUERTE
   ret             

doblar_izq_suave:

  mov VEL_MOTOR_DER_AUX,VEL_MOTOR_DER

  cpi VEL_MOTOR_DER, VEL_MAX ; 1° veo si VEL_MOTOR_DER = 255
        breq red_suave_vel_izquierda ;si VEL_MOTOR_DER = 255 debo reducir la vel_motor_izq

  add VEL_MOTOR_DER_AUX, VAR_SUAVE
  cpi VEL_MOTOR_DER_AUX, VEL_MAX ; ¿vel_motor_der = vel_motor_der + var_fuerte > 255 (vel max)?
        brlo subo_suave_vel_derecha ;salta si la suma es menor

  cpi VEL_MOTOR_DER, VEL_MAX
        brsh fijo_vel_derecha
  ret

  subo_suave_vel_derecha:
    add VEL_MOTOR_DER, VAR_SUAVE
	ret

  red_suave_vel_izquierda:
    cpi VEL_MOTOR_IZQ, VAR_SUAVE_VALOR
          brsh decre_suave_vel_mot_izq        ;si la vel izq es mayor a la var suave, puedo decrementar su valor
    cpi VEL_MOTOR_IZQ, VAR_SUAVE_VALOR
          brlo fijo_vel_mot_izq_f
    ret

   decre_suave_vel_mot_izq:
     sub VEL_MOTOR_IZQ,VAR_SUAVE
   ret             

			          
doblar_der_fuerte:

  mov VEL_MOTOR_IZQ_AUX,VEL_MOTOR_IZQ

  cpi VEL_MOTOR_IZQ, VEL_MAX ; 1° veo si VEL_MOTOR_IZQ = 255
        breq red_fuerte_vel_derecha ;si VEL_MOTOR_IZQ = 255 debo reducir la vel_motor_der

  add VEL_MOTOR_IZQ_AUX, VAR_FUERTE
  cpi VEL_MOTOR_IZQ_AUX, VEL_MAX ; ¿vel_motor_izq = vel_motor_izq + var_fuerte > 255 (vel max)?
        brlo subo_fuerte_vel_izquierda ;salta si la suma es menor

  cpi VEL_MOTOR_IZQ, VEL_MAX
        brsh fijo_vel_izquierda
  ret

  subo_fuerte_vel_izquierda:
    add VEL_MOTOR_IZQ, VAR_FUERTE
	ret

  red_fuerte_vel_derecha:
    cpi VEL_MOTOR_DER, VAR_FUERTE_VALOR
          brsh decre_fuerte_vel_mot_der       ;si la vel der es mayor a la var suave, puedo decrementar su valor
    cpi VEL_MOTOR_DER, VAR_FUERTE_VALOR
          brlo fijo_vel_mot_der_f
    ret

   fijo_vel_izquierda:
     ldi VEL_MOTOR_IZQ, VEL_MAX
   ret

   fijo_vel_mot_der_f:
     ldi VEL_MOTOR_DER, 0x00 
   ret

   decre_fuerte_vel_mot_der:
     sub VEL_MOTOR_DER,VAR_FUERTE
   ret

doblar_der_suave:

  mov VEL_MOTOR_IZQ_AUX,VEL_MOTOR_IZQ

  cpi VEL_MOTOR_IZQ, VEL_MAX ; 1° veo si VEL_MOTOR_IZQ = 255
        breq red_suave_vel_derecha ;si VEL_MOTOR_IZQ = 255 debo reducir la vel_motor_der

  add VEL_MOTOR_IZQ_AUX, VAR_SUAVE
  cpi VEL_MOTOR_IZQ_AUX, VEL_MAX ; ¿vel_motor_izq = vel_motor_izq + var_fuerte > 255 (vel max)?
        brlo subo_suave_vel_izquierda ;salta si la suma es menor

  cpi VEL_MOTOR_IZQ, VEL_MAX
        brsh fijo_vel_izquierda
  ret

  subo_suave_vel_izquierda:
    add VEL_MOTOR_IZQ, VAR_SUAVE
  ret

  red_suave_vel_derecha:
    cpi VEL_MOTOR_DER, VAR_SUAVE_VALOR
          brsh decre_suave_vel_mot_der        ;si la vel der es mayor a la var suave, puedo decrementar su valor
    cpi VEL_MOTOR_DER, VAR_SUAVE_VALOR
          brlo fijo_vel_mot_der_f
    ret

   decre_suave_vel_mot_der:
     sub VEL_MOTOR_DER,VAR_SUAVE
   ret  



detectar_lineas: ;si sucede que val_centro = 0 ---> la pista es blanca con raya negra
                 ;si sucede que val_centro = 1 ---> la pista es negra con raya blanca
				 ;USAR REG COMO FLAGS 
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
  ldi r16, 0x00
  out DDRC,r16
              
  ldi r16,0x06
  out DDRB,r16
              
  ldi r16,0x01 ; Rpull-up interna para pulsador
  out PORTB, r16

              
  ldi r16,0b00001000
  out DDRD,r16 ; 1 led conectado a PD7

ret


configPWM:
  ldi	r16, (0 << WGM11) | (1 << WGM10) | (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0)
  sts	TCCR1A, r16

  ldi	r16, (0 << WGM13) | (0 << WGM12) | (1 << CS12) | (0 << CS11) | (0 << CS10)
  sts	TCCR1B, r16

ret
