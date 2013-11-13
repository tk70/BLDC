; *------------------------------*
; |   Hardware related config    |
; *-----------------------------*

; clock frequency [MHz]
.equ CLOCK_MHZ = 16

.equ CONTROL_METHOD = 2								; 1 - RC pulse, 2 - I2C (coming soon)

.equ I2C_SLAVE_ADDRESS = 4

; if rc pulse used
.equ RCP_CHANNEL = 0

.equ PWM_SIDE = 1

; pins
; phase R
.equ RL_PIN	=	PB1									; low gate mosfet
.equ RL_PORT =	PORTB
.equ RL_DDR =	DDRB

.equ RH_PIN	=	PB0									; high gate mosfet
.equ RH_PORT =	PORTB
.equ RH_DDR =	DDRB

.equ R_COMP_CHANNEL	= 0								; phase comparator channel (port C)


; phase S
.equ SL_PIN =	PB2
.equ SL_PORT =	PORTB
.equ SL_DDR =	DDRB

.equ SH_PIN =	PB4
.equ SH_PORT =	PORTB
.equ SH_DDR =	DDRB

.equ S_COMP_CHANNEL	= 1


; phase T
.equ TL_PIN	=	PB3
.equ TL_PORT =	PORTB
.equ TL_DDR =	DDRB

.equ TH_PIN	=	PB5
.equ TH_PORT =	PORTB
.equ TH_DDR =	DDRB

.equ T_COMP_CHANNEL	= 2

.equ LED_DBG = 1

; for debug
.equ LED0_DDR	=	DDRC
.equ LED0_PORT	=	PORTC
.equ LED0_PIN	=	PC3

.equ LED1_DDR	=	DDRC
.equ LED1_PORT	=	PORTC
.equ LED1_PIN	=	PC4

.equ LED2_DDR	=	DDRD
.equ LED2_PORT	=	PORTD
.equ LED2_PIN	=	PD5

.equ LED3_DDR	=	DDRD
.equ LED3_PORT	=	PORTD
.equ LED3_PIN	=	PD4

.equ LED4_DDR	=	DDRD
.equ LED4_PORT	=	PORTD
.equ LED4_PIN	=	PD3

.equ LED5_DDR	=	DDRC
.equ LED5_PORT	=	PORTC
.equ LED5_PIN	=	PC5

.equ LED6_DDR	=	DDRD
.equ LED6_PORT	=	PORTD
.equ LED6_PIN	=	PD1

.equ LED7_DDR	=	DDRD
.equ LED7_PORT	=	PORTD
.equ LED7_PIN	=	PD0