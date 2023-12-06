.include "m328pdef.inc"

.equ VAR_SUAVE = 1            ;Paso con el que varían las velocidades en doblar suave
.equ VAR_FUERTE = 2           ;Paso con el que varían las velocidades en doblar fuerte
.equ VEL_MAX =  80            ;Velocidad máxima, puede variar entre 0 y 255
.equ VEL_MAX_EXTREMOS = 200   ;Velocidad máxima de las ruedas exteriores cuando se necesita un giro brusco, puede variar entre 0 y 255
.equ VEL_REVERSA = 20         ;Velocidad de las ruedas interiores cuando se necesita un giro brusco y se activa la reversa, puede variar entre 0 y 255
.equ VEL_MIN = 30             ;Velocidad cuando se necesita ir a velocidad mínima, puede variar entre 0 y 255

.equ VAL_IZQ_2 = PC0 //0x01  ; 0 0 0 0 0 0 0 1 
.equ VAL_IZQ_1 = PC1 //0x02  ; 0 0 0 0 0 0 1 0   
.equ VAL_CENTRO = PC2 //0x04 ; 0 0 0 0 0 1 0 0
.equ VAL_DER_1 = PC3 //0x08  ; 0 0 0 0 1 0 0 0 
.equ VAL_DER_2 = PC4 //0x10  ; 0 0 0 1 0 0 0 0  

.equ BLANCO = 1
.equ NEGRO = 0

.def PISTA_BLA_LINEA_NEG = r16 ;Flag para decidir entre pista de fondo negro, linea blanca o inversa
.def REG_TEMP = r17
.def VAL_LEIDO = r18
.def VEL_MOTOR_DER = r19
.def VEL_MOTOR_IZQ = r20
.def VEL_MOTOR_DER_AUX = r21
.def VEL_MOTOR_IZQ_AUX = r22
.def VEL_MOTOR_REV_DER = r23
.def VEL_MOTOR_REV_IZQ = r24
.def CONTADOR_TIMER = r25

.cseg 
.org 0x00
  jmp	main
.org 0x0012
	rjmp handler_interrup_overflow

.org INT_VECTORS_SIZE

main:

  ldi REG_TEMP, HIGH(RAMEND) ;Se inicializa el stack pointer
  out SPH, REG_TEMP
  ldi REG_TEMP, LOW(RAMEND)
  out SPL, REG_TEMP

  call config_puertos
  call configPWM

  wait: ;Espera a que se pulse el botón
    sbic PINB, PB0 
		rjmp wait
  call elegir_tipo_de_pista
  
  call delay_inicial
  
  ldi VEL_MOTOR_DER, VEL_MAX
  ldi VEL_MOTOR_IZQ, VEL_MAX
  call actualizarVelocidad 

  ldi CONTADOR_TIMER, 0x00
  SEI

  loop: ;Se leen valores de los sensores y se decide la reacción

    in VAL_LEIDO, PINC ;Lectura de sensores
    call acomodar_valores_a_tipo_de_pista

    mov REG_TEMP, VAL_LEIDO ;Comprobación del caso "curva de 90º"
    andi REG_TEMP, 0b11111
    cpi REG_TEMP, 0b11111
    brmi continuacion_loop

    call curva_recta

    continuacion_loop:

    sbrc VAL_LEIDO, VAL_IZQ_2
    call doblar_izq_fuerte

    sbrc VAL_LEIDO, VAL_DER_2
    call doblar_der_fuerte

    sbrc VAL_LEIDO, VAL_IZQ_1
    call doblar_izq_suave

    sbrc VAL_LEIDO, VAL_DER_1
    call doblar_der_suave
	
    sbrc VAL_LEIDO, VAL_CENTRO
    call sigo_centro

    call actualizarVelocidad 

  rjmp loop

sigo_centro: ;Ambos motores a velocidad máxima

  ldi CONTADOR_TIMER, 0x00

  ldi VEL_MOTOR_DER, VEL_MAX
  ldi VEL_MOTOR_IZQ, VEL_MAX
  ldi VEL_MOTOR_REV_DER, 0X00
  ldi VEL_MOTOR_REV_IZQ, 0X00
ret

doblar_izq_fuerte:

  ldi CONTADOR_TIMER, 0x00

  mov VEL_MOTOR_DER_AUX,VEL_MOTOR_DER

  cpi VEL_MOTOR_DER, VEL_MAX
  breq red_fuerte_vel_izquierda ;Si VEL_MOTOR_DER = VEL_MAX se reduce la velocidad del motor izquierdo

  ldi REG_TEMP, VAR_FUERTE
  add VEL_MOTOR_DER_AUX, REG_TEMP

  cpi VEL_MOTOR_DER_AUX, VEL_MAX 
  brlo subo_fuerte_vel_derecha ;Si VEL_MOTOR_DER + VAR_FUERTE < VEL_MAX se aumenta la velocidad del motor derecho

  cpi VEL_MOTOR_DER, VEL_MAX
  brsh mot_der_vel_max ;Si VEL_MOTOR_DER + VAR_FUERTE > VEL_MAX se aumenta la velocidad del motor derecho a VEL_MAX
  rjmp final_doblar_izq_fuerte

  subo_fuerte_vel_derecha:
    ldi REG_TEMP, VAR_FUERTE
    add VEL_MOTOR_DER, REG_TEMP
	rjmp final_doblar_izq_fuerte

  red_fuerte_vel_izquierda:
    cpi VEL_MOTOR_IZQ, VAR_FUERTE
    brsh decre_fuerte_vel_mot_izq
    
    cpi VEL_MOTOR_IZQ, VAR_FUERTE
    brlo mot_izq_rev_der_max ;Si VEL_MOTOR_IZQ - VAR_FUERTE < 0 => motor derecho a VEL_MAX Y motor izquierdo en reversa
  rjmp final_doblar_izq_fuerte

  decre_fuerte_vel_mot_izq:
    ldi REG_TEMP, VAR_FUERTE
    sub VEL_MOTOR_IZQ, REG_TEMP 
  rjmp final_doblar_izq_fuerte

  mot_izq_rev_der_max:
     ldi VEL_MOTOR_IZQ, 0x00
     ldi VEL_MOTOR_REV_IZQ, VEL_REVERSA
     ldi VEL_MOTOR_DER, VEL_MAX_EXTREMOS
  rjmp final_doblar_izq_fuerte

  final_doblar_izq_fuerte:
ret             

doblar_izq_suave:

  ldi CONTADOR_TIMER, 0x00

  cpi VEL_MOTOR_REV_IZQ, VEL_REVERSA
  breq desactivar_giro_extremo_izq ;Se desactiva la reversa si está activada

  rjmp fin_desactivar_giro_extremo_izq

  desactivar_giro_extremo_izq:
    ldi VEL_MOTOR_REV_IZQ, 0x00
    ldi VEL_MOTOR_REV_DER, 0x00
    
    ldi VEL_MOTOR_IZQ, 0x00
    ldi VEL_MOTOR_DER, VEL_MAX
  rjmp final_doblar_izq_suave

  fin_desactivar_giro_extremo_izq:

  mov VEL_MOTOR_DER_AUX,VEL_MOTOR_DER

  cpi VEL_MOTOR_DER, VEL_MAX 
  breq  red_suave_vel_izquierda ;Si VEL_MOTOR_DER = VEL_MAX se reduce la velocidad del motor izquierdo

  ldi REG_TEMP, VAR_SUAVE
  add VEL_MOTOR_DER_AUX, REG_TEMP

  cpi VEL_MOTOR_DER_AUX, VEL_MAX 
  brlo subo_suave_vel_derecha ;Si VEL_MOTOR_DER + VAR_SUAVE < VEL_MAX se aumenta la velocidad del motor derecho

  cpi VEL_MOTOR_DER, VEL_MAX
  brsh mot_der_vel_max  ;Si VEL_MOTOR_DER + VAR_SUAVE > VEL_MAX => motor derecho a VEL_MAX Y motor izquierdo a velocidad nula
  
  rjmp final_doblar_izq_suave

  subo_suave_vel_derecha:
    ldi REG_TEMP, VAR_SUAVE
    add VEL_MOTOR_DER, REG_TEMP
	rjmp final_doblar_izq_suave

  red_suave_vel_izquierda:
    cpi VEL_MOTOR_IZQ, VAR_SUAVE
    brsh decre_suave_vel_mot_izq
    
    cpi VEL_MOTOR_IZQ, VAR_SUAVE
    brlo mot_izq_vel_nula
  rjmp final_doblar_izq_suave

  decre_suave_vel_mot_izq:
    ldi REG_TEMP, VAR_SUAVE
    sub VEL_MOTOR_IZQ, REG_TEMP 
  rjmp final_doblar_izq_suave
  
  mot_izq_vel_nula:
    ldi VEL_MOTOR_IZQ, 0x00
  rjmp final_doblar_izq_suave

  final_doblar_izq_suave:
ret             

mot_der_vel_max:
  ldi VEL_MOTOR_DER, VEL_MAX
ret

doblar_der_fuerte:

  ldi CONTADOR_TIMER, 0x00

  mov VEL_MOTOR_IZQ_AUX,VEL_MOTOR_IZQ

  cpi VEL_MOTOR_IZQ, VEL_MAX 
  breq red_fuerte_vel_derecha ;Si VEL_MOTOR_IZQ = VEL_MAX se reduce la velocidad del motor derecho

  ldi REG_TEMP, VAR_FUERTE
  add VEL_MOTOR_IZQ_AUX, REG_TEMP
  
  cpi VEL_MOTOR_IZQ_AUX, VEL_MAX
  brlo subo_fuerte_vel_izquierda ;Si VEL_MOTOR_IZQ + VAR_FUERTE < VEL_MAX se aumenta la velocidad del motor izquierdo

  cpi VEL_MOTOR_IZQ, VEL_MAX
  brsh mot_izq_vel_max  ;Si VEL_MOTOR_IZQ + VAR_FUERTE > VEL_MAX se aumenta la velocidad del motor izquierdo a VEL_MAX
  
  rjmp final_doblar_der_fuerte

  subo_fuerte_vel_izquierda:
    ldi REG_TEMP, VAR_FUERTE
    add VEL_MOTOR_IZQ, REG_TEMP
	rjmp final_doblar_der_fuerte

  red_fuerte_vel_derecha:
    cpi VEL_MOTOR_DER, VAR_FUERTE
    brsh decre_fuerte_vel_mot_der
    
    cpi VEL_MOTOR_DER, VAR_FUERTE
    brlo mot_der_rev_izq_max ;Si VEL_MOTOR_DER - VAR_FUERTE < 0 => motor izquierdo a VEL_MAX Y motor derecho en reversa
  rjmp final_doblar_der_fuerte

  decre_fuerte_vel_mot_der:
    ldi REG_TEMP, VAR_FUERTE
    sub VEL_MOTOR_DER, REG_TEMP 
  ret

  mot_der_rev_izq_max:
    ldi VEL_MOTOR_DER, 0x00
    ldi VEL_MOTOR_REV_DER, VEL_REVERSA
    ldi VEL_MOTOR_IZQ, VEL_MAX_EXTREMOS
  rjmp final_doblar_der_fuerte

  final_doblar_der_fuerte:
ret

doblar_der_suave:

  ldi CONTADOR_TIMER, 0x00

  cpi VEL_MOTOR_REV_DER, VEL_REVERSA
  breq desactivar_giro_extremo_der  ;Se desactiva la reversa si está activada

  rjmp fin_desactivar_giro_extremo_der

  desactivar_giro_extremo_der:
    ldi VEL_MOTOR_REV_IZQ, 0x00
    ldi VEL_MOTOR_REV_DER, 0x00
    
    ldi VEL_MOTOR_DER, 0x00
    ldi VEL_MOTOR_IZQ, VEL_MAX
  rjmp final_doblar_der_suave

  fin_desactivar_giro_extremo_der:

  mov VEL_MOTOR_IZQ_AUX,VEL_MOTOR_IZQ

  cpi VEL_MOTOR_IZQ, VEL_MAX 
  breq red_suave_vel_derecha ;Si VEL_MOTOR_IZQ = VEL_MAX se reduce la velocidad del motor derecho

  ldi REG_TEMP, VAR_SUAVE
  add VEL_MOTOR_IZQ_AUX, REG_TEMP

  cpi VEL_MOTOR_IZQ_AUX, VEL_MAX
  brlo subo_suave_vel_izquierda ;Si VEL_MOTOR_IZQ + VAR_SUAVE < VEL_MAX se aumenta la velocidad del motor izquierdo

  cpi VEL_MOTOR_IZQ, VEL_MAX
  brsh mot_izq_vel_max ;Si VEL_MOTOR_IZQ + VAR_SUAVE > VEL_MAX => motor izquierdo a VEL_MAX Y motor derecho a velocidad nula
  
  rjmp final_doblar_der_suave

  subo_suave_vel_izquierda:
    ldi REG_TEMP, VAR_SUAVE
    add VEL_MOTOR_IZQ, REG_TEMP
  rjmp final_doblar_der_suave

  red_suave_vel_derecha:
    cpi VEL_MOTOR_DER, VAR_SUAVE
    brsh decre_suave_vel_mot_der
    
    cpi VEL_MOTOR_DER, VAR_SUAVE
    brlo mot_der_vel_nula
  rjmp final_doblar_der_suave

  decre_suave_vel_mot_der:
    ldi REG_TEMP, VAR_SUAVE
    sub VEL_MOTOR_DER,REG_TEMP 
  rjmp final_doblar_der_suave

  mot_der_vel_nula:
    ldi VEL_MOTOR_DER, 0x00     
  rjmp final_doblar_der_suave
  
  final_doblar_der_suave:
ret  

mot_izq_vel_max:
     ldi VEL_MOTOR_IZQ, VEL_MAX
ret

actualizarVelocidad:
  sts	OCR1BL, VEL_MOTOR_DER
  sts	OCR1AL, VEL_MOTOR_IZQ
  out	OCR0B, VEL_MOTOR_REV_DER
  out	OCR0A, VEL_MOTOR_REV_IZQ
ret

delay_inicial: ;Delay de 5 segundos
  push r16
  push r17
  push r18
  push r19

  ldi r16, 0x00
  ldi r17, 0x00
  ldi	r18, 0x00
  ldi	r19, 0x00

inicioDelay:
  inc	r16
  brne inicioDelay

  ldi	r16, 0x00
  inc	r17
  brne inicioDelay

  ldi	r16, 0x00
  ldi	r17, 0x00
  inc	r18
  brne inicioDelay

  ldi	r18, 115
  inc	r19
  cpi	r19, 2
  brne inicioDelay

  pop r19
  pop r18
  pop r17
  pop r16

ret


config_puertos:

  ldi REG_TEMP, 0x00
  out DDRC,REG_TEMP

  ldi REG_TEMP, (0 << PB0) | (1 << PB1) | (1 << PB2)
  out DDRB,REG_TEMP

              
  ldi REG_TEMP,(1 << PD7) | (1 << PD5) | (1 << PD6)
  out DDRD,REG_TEMP ;
  
ret


configPWM:
  
  ;Configuración timer 1
  ldi	REG_TEMP, (0 << WGM11) | (1 << WGM10) | (1 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0)
  sts	TCCR1A, REG_TEMP

  ldi	REG_TEMP, (0 << WGM13) | (0 << WGM12) | (1 << CS12) | (0 << CS11) | (0 << CS10)
  sts	TCCR1B, REG_TEMP

  ldI REG_TEMP, 0X00
  
  sts	OCR1BH, REG_TEMP
  sts	OCR1BL, REG_TEMP
    
  sts	OCR1AH, REG_TEMP
  sts	OCR1AL, REG_TEMP

  ;Configuración timer 0
  ldi	REG_TEMP, (0 << WGM01) | (1 << WGM00) | (1 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0)
  out	TCCR0A, REG_TEMP

  ldi	REG_TEMP, (0 << WGM02) | (1 << CS02) | (0 << CS01) | (0 << CS00)
  out	TCCR0B, REG_TEMP
  
  ldI REG_TEMP, 0X00

  out	OCR0B, REG_TEMP
  out	OCR0A, REG_TEMP

  ;Configuración timer 2

  ldi	REG_TEMP, (1 << CS02) | (1 << CS01) | (1 << CS00)
  sts	TCCR2B, REG_TEMP

  ldi	REG_TEMP, (1 << TOIE2)
  sts	TIMSK2, REG_TEMP

ret

encenderLed:
  sbi PORTD, PD7
ret

apagarLed:
  cbi PORTD, PD7
ret

elegir_tipo_de_pista: ;Decide el tipo de pista evaluando los sensores mas alejados
  in REG_TEMP, PINC
  andi REG_TEMP, 0b10001
  
  ldi PISTA_BLA_LINEA_NEG, 0x00

  tst REG_TEMP
  breq fin_elegir_tipo_de_pista
  
  ldi PISTA_BLA_LINEA_NEG, 0xff
  
  fin_elegir_tipo_de_pista:
ret

curva_recta:
;Se ponen los motores a velocidad mínima, se espera a leer dos líneas perpendiculares a la dirección del auto y 
;se realiza un giro cerrado hacia el lado correcto

  ldi VEL_MOTOR_DER, VEL_MIN
  ldi VEL_MOTOR_IZQ, VEL_MIN
  ldi VEL_MOTOR_REV_DER, 0X00
  ldi VEL_MOTOR_REV_IZQ, 0X00

  call actualizarVelocidad

  in VAL_LEIDO, PINC
  call acomodar_valores_a_tipo_de_pista

  andi VAL_LEIDO, 0b10001
  
  tst VAL_LEIDO
  breq fin_primera_linea ;Espera a que termine de leerse la primer línea perpendicular

  rjmp curva_recta

  fin_primera_linea:

  in VAL_LEIDO, PINC
  call acomodar_valores_a_tipo_de_pista

  andi VAL_LEIDO, 0b10001
  
  cpi VAL_LEIDO, 0b10001
  breq inicio_segunda_linea ;Espera a que se comienze a leer la segunda línea perpendicular

  rjmp fin_primera_linea

  inicio_segunda_linea:
  
  in VAL_LEIDO, PINC
  call acomodar_valores_a_tipo_de_pista

  andi VAL_LEIDO, 0b10001
  
  tst VAL_LEIDO
  breq fin_segunda_linea ;Espera a que termine de leerse la segunda línea perpendicular

  rjmp inicio_segunda_linea

  fin_segunda_linea:
  
  in VAL_LEIDO, PINC
  call acomodar_valores_a_tipo_de_pista

  andi VAL_LEIDO, 0b10001
  
  tst VAL_LEIDO
  breq fin_segunda_linea ;Espera a que se lea el inicio de la curva de 90º

  
  inicio_curva: ;Se leen los sensores de los extremos, según cual se encienda se decide la dirección de giro
    
  sbrc VAL_LEIDO, VAL_IZQ_2
  call giro_cerrado_izq

  sbrc VAL_LEIDO, VAL_DER_2
  call giro_cerrado_der

  call actualizarVelocidad
  
  fin_lectura_lateral:
  
  in VAL_LEIDO, PINC
  call acomodar_valores_a_tipo_de_pista

  andi VAL_LEIDO, 0b10001
  
  tst VAL_LEIDO
  breq espera_leer_centro ;Espera a que el sensor extremo que se había activado se desactive

  rjmp fin_lectura_lateral

  espera_leer_centro:
  
  in VAL_LEIDO, PINC
  call acomodar_valores_a_tipo_de_pista

  sbrs VAL_LEIDO, VAL_CENTRO
  rjmp espera_leer_centro ;Continúa realizando el giro cerrado hasta que se activa el sensor central

ret

giro_cerrado_izq:
  ldi VEL_MOTOR_IZQ, 0x00
  ldi VEL_MOTOR_REV_IZQ, VEL_REVERSA
  ldi VEL_MOTOR_DER, VEL_MAX_EXTREMOS
ret

giro_cerrado_der:
  ldi VEL_MOTOR_DER, 0x00
  ldi VEL_MOTOR_REV_DER, VEL_REVERSA
  ldi VEL_MOTOR_IZQ, VEL_MAX_EXTREMOS
ret

acomodar_valores_a_tipo_de_pista:
  tst PISTA_BLA_LINEA_NEG
  breq fin_acomodar_a_tipo_de_pista

  com VAL_LEIDO

  fin_acomodar_a_tipo_de_pista:
ret

handler_interrup_overflow:
;Cuando el contador alcanza el valor 120 (dos segundos sin que ningún sensor se active) el auto se frena y se enciende el led

  push REG_TEMP
  in REG_TEMP, SREG

  inc CONTADOR_TIMER

  cpi CONTADOR_TIMER, 120
  brmi final_handler_interrup_overflow

  ldi VEL_MOTOR_DER, 0x00
  ldi VEL_MOTOR_IZQ, 0x00
  ldi VEL_MOTOR_REV_DER, 0x00
  ldi VEL_MOTOR_REV_IZQ, 0x00

  call actualizarVelocidad
  call encenderLed

  loop_infinito:
  rjmp loop_infinito

  final_handler_interrup_overflow:
  
  out SREG, REG_TEMP
  pop REG_TEMP
reti