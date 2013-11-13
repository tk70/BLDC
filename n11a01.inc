; *------------------------------*
; |   Hardware related config    |
; *-----------------------------*

; clock frequency [MHz]
.equ CLOCK_MHZ = 16

.equ CONTROL_METHOD = 2					; 1 - RC pulse, 2 - I2C (coming soon)

.equ I2C_SLAVE_ADDRESS = 4

; if rc pulse used
.equ RCP_CHANNEL = 0

.equ PWM_SIDE = 1					; which FET side does the PWM; 1 - low, 2 - high

; pins
; phase R
.equ RL_PIN	=	PB5				; low gate mosfet
.equ RL_PORT =		PORTB
.equ RL_DDR =		DDRB

.equ RH_PIN	=	PD0				; high gate mosfet
.equ RH_PORT =		PORTD
.equ RH_DDR =		DDRD

.equ R_COMP_CHANNEL	= 0				; phase comparator channel (port C)


; phase S
.equ SL_PIN =		PD3
.equ SL_PORT =		PORTD
.equ SL_DDR =		DDRD

.equ SH_PIN =		PB1
.equ SH_PORT =		PORTB
.equ SH_DDR =		DDRB

.equ S_COMP_CHANNEL	= 1


; phase T
.equ TL_PIN	=	PD4
.equ TL_PORT =		PORTD
.equ TL_DDR =		DDRD

.equ TH_PIN	=	PB2
.equ TH_PORT =		PORTB
.equ TH_DDR =		DDRB

.equ T_COMP_CHANNEL	= 2

.equ LED_DEBUG = 1

; for debug
.equ LED0_DDR	=	DDRB
.equ LED0_PORT	=	PORTB
.equ LED0_PIN	=	PB3

.equ LED1_DDR	=	DDRB
.equ LED1_PORT	=	PORTB
.equ LED1_PIN	=	PB4

;.equ LED5_DDR	=	DDRB
;.equ LED5_PORT	=	PORTB
;.equ LED5_PIN	=	PB3