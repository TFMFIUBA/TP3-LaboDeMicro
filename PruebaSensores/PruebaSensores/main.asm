;
; PruebaSensores.asm
;
; Created: 24/11/2023 18:36:23
; Author : tomas
;

.equ VAL_IZQ_2 = 0x01  ; 0 0 0 0 0 0 0 1 
.equ VAL_IZQ_1 = 0x02  ; 0 0 0 0 0 0 1 0   
.equ VAL_CENTRO = 0x04 ; 0 0 0 0 0 1 0 0
.equ VAL_DER_1 = 0x08  ; 0 0 0 0 1 0 0 0 
.equ VAL_DER_2 = 0x10  ; 0 0 0 1 0 0 0 0  



.def VAL_LEIDO = r18

;config puertos
ldi r16,0x00
out DDRC,r16 ;cinco sensores en PORTC

ldi r16,0b00001000
out DDRD,r16 ; 1 led conectado a PD7

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
     


doblar_izq_fuerte: 
  ;sbi PORTD, 7      
  ret             
			 
doblar_izq_suave: 
  ;sbi PORTD, 7      
  ret             

sigo_centro: 
  ;sbi PORTD, 7       
ret	          

doblar_der_suave: 
;sbi PORTD, 7      
ret

doblar_der_fuerte:
 sbi PORTD, 7      
ret  

