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

doblar_izq_fuerte: ;en mot der subo la vel en un 10%
                   ;en mot izq bajo la vel en un 10%
  adc VEL_MOTOR_DER_AUX, VAR_FUERTE
	
  cpi VEL_MOTOR_DER_AUX, VEL_MAX ; ESTOY EN VEL MAX(255)?
	brsh fijo_vel_der ; si la vel der se pasa de 255 -> fijo su valor
	
  cpi VEL_MOTOR_DER, VEL_MAX
	brne inc_vel_der
  ret
  
  inc_vel_der:
    add VEL_MOTOR_DER, VAR_FUERTE
	ret
    
  fijo_vel_der: ;subo la vel del mot derecho
    ldi VEL_MOTOR_DER, VEL_MAX
    sub VEL_MOTOR_DER_AUX, VAR_FUERTE //                                  ACTUALIZAR VALOR AUXILIAR Y SALIR AL SETEAR VEL MAX
    cpi VEL_MOTOR_IZQ, 25 ; 25= var_fuerte
    brsh dec_vel_mot_izq ;si la vel izq es mayor a la var fuerte, puedo decrementar su valor
    cpi VEL_MOTOR_IZQ, 25 ; 25= var_fuerte
    brlo fijo_vel_mot_izq
  ret
  
  fijo_vel_mot_izq:
    ldi VEL_MOTOR_IZQ, 0x00 
  ret
  
  dec_vel_mot_izq:
    sub VEL_MOTOR_IZQ, VAR_FUERTE
ret             

doblar_izq_suave: ;en mot der subo la vel en un 2%
                   ;en mot izq bajo la vel en un 2%
  add VEL_MOTOR_DER_AUX, VAR_SUAVE
	
  cpi VEL_MOTOR_DER_AUX, VEL_MAX ; ESTOY EN VEL MAX(255)?
	brsh fijo_vel_der_s ; si la vel der se pasa de 255 -> fijo su valor
	
  cpi VEL_MOTOR_DER, VEL_MAX
	brne inc_vel_der_s
  ret
  
  inc_vel_der_s:
    add VEL_MOTOR_DER, VAR_SUAVE
  ret
   
   fijo_vel_der_s: ;subo la vel del mot derecho
    ldi VEL_MOTOR_DER, VEL_MAX
		sub VEL_MOTOR_DER_AUX, VAR_SUAVE
		cpi VEL_MOTOR_IZQ, 5
		brsh dec_vel_mot_izq_s ;si la vel izq es mayor a la var suave, puedo decrementar su valor
		cpi VEL_MOTOR_IZQ, 5
		brlo fijo_vel_mot_izq_s
	ret
  
  fijo_vel_mot_izq_s:
    ldi VEL_MOTOR_IZQ, 0x00 
  ret
  
  dec_vel_mot_izq_s:
    sub VEL_MOTOR_IZQ, VAR_SUAVE
  ret             

			          
doblar_der_fuerte: ;en mot izq subo la vel en un 10%
                   ;en mot der bajo la vel en un 10%
  adc VEL_MOTOR_IZQ_AUX, VAR_FUERTE
	
  cpi VEL_MOTOR_IZQ_AUX, VEL_MAX ; ESTOY EN VEL MAX(255)?
	brsh fijo_vel_izq ; si la vel der se pasa de 255 -> fijo su valor
  
  cpi VEL_MOTOR_IZQ, VEL_MAX
	brne inc_vel_izq
  ret
  
  inc_vel_izq:
    add VEL_MOTOR_IZQ, VAR_FUERTE
  ret
  
  fijo_vel_izq: ;subo la vel del mot izquierdo
    ldi VEL_MOTOR_IZQ, VEL_MAX
    sub VEL_MOTOR_IZQ_AUX, VAR_FUERTE
    cpi VEL_MOTOR_DER, 25 ; 25= var_fuerte
    brsh dec_vel_mot_der ;si la vel der es mayor a la var fuerte, puedo decrementar su valor
    cpi VEL_MOTOR_DER, 25 ; 25= var_fuerte
    brlo fijo_vel_mot_der
  ret
  
  fijo_vel_mot_der:
    ldi VEL_MOTOR_DER, 0x00 
  ret
  
  dec_vel_mot_der:
    sub VEL_MOTOR_DER,VAR_FUERTE
ret  

doblar_der_suave: ;en mot izq subo la vel en un 2%
                   ;en mot der bajo la vel en un 2%
  add VEL_MOTOR_IZQ_AUX, VAR_SUAVE
  
  cpi VEL_MOTOR_IZQ_AUX, VEL_MAX ; ESTOY EN VEL MAX(255)?
  brsh fijo_vel_izq_s ; si la vel iz se pasa de 255 -> fijo su valor
  
  cpi VEL_MOTOR_IZQ, VEL_MAX
  brne inc_vel_izq_s
  ret
  
  inc_vel_izq_s:
    add VEL_MOTOR_IZQ, VAR_SUAVE
  ret
  
  fijo_vel_izq_s: ;subo la vel del mot izquierdo
    ldi VEL_MOTOR_IZQ, VEL_MAX
    sub VEL_MOTOR_IZQ_AUX, VAR_SUAVE
		cpi VEL_MOTOR_DER, 5
		brsh dec_vel_mot_der_s ;si la vel izq es mayor a la var suave, puedo decrementar su valor
		cpi VEL_MOTOR_DER, 5
		brlo fijo_vel_mot_der_s
		ret
  fijo_vel_mot_der_s:
    ldi VEL_MOTOR_DER, 0x00 
		ret
  dec_vel_mot_der_s:
    sub VEL_MOTOR_DER, VAR_SUAVE
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