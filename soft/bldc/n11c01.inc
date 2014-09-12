; *------------------------------*
; |   Hardware related config    |
; *-----------------------------*

; clock frequency [Hz]
.equ CLOCK_HZ = 16000000

; Input signal. 1 - RC pulse, 2 - I2C
.equ CONTROL_METHOD = 2	

; If I2C chosen as control method
.equ I2C_SLAVE_ADDRESS = 4				; Base slave address of the ESC
.equ I2C_PROTOCOL = 2					; 1 - basic, 2 - extended
.equ I2C_ADDR_PINS = 1					; Does the board have pins shifting the I2C address?

.if I2C_ADDR_PINS == 1

 .equ I2C_ADDR0_P = PD5
 .equ I2C_ADDR0_PORT = PORTD
 .equ I2C_ADDR0_PIN = PIND
 .equ I2C_ADDR0_DDR = DDRD			

 .equ I2C_ADDR1_P = PD7
 .equ I2C_ADDR1_PORT = PORTD
 .equ I2C_ADDR1_PIN = PIND
 .equ I2C_ADDR1_DDR = DDRD

 .equ I2C_ADDR_PULL_UPS = 1				; Pull-up to Vcc internally?

.endif


; If RC Pulse chosen as control method
.equ RCP_CHANNEL = 0

.equ PWM_SIDE = 1					; which FET side does the PWM; 1 - low, 2 - high

; pins
; phase R
.equ RL_PIN	=	PD4				; low gate mosfet
.equ RL_PORT =		PORTD
.equ RL_DDR =		DDRD
.equ RL_INVERTING =	0

.equ RH_PIN	=	PD1				; high gate mosfet
.equ RH_PORT =		PORTD
.equ RH_DDR =		DDRD
.equ RH_INVERTING =	1

.equ R_COMP_CHANNEL	= 2				; phase comparator channel (port C)

; phase S
.equ SL_PIN =		PD3
.equ SL_PORT =		PORTD
.equ SL_DDR =		DDRD
.equ SL_INVERTING =	0

.equ SH_PIN =		PB1
.equ SH_PORT =		PORTB
.equ SH_DDR =		DDRB
.equ SH_INVERTING =	1

.equ S_COMP_CHANNEL	= 1

; phase T
.equ TL_PIN	=	PB2
.equ TL_PORT =		PORTB
.equ TL_DDR =		DDRB
.equ TL_INVERTING =	0

.equ TH_PIN	=	PB3
.equ TH_PORT =		PORTB
.equ TH_DDR =		DDRB
.equ TH_INVERTING =	1

.equ T_COMP_CHANNEL	= 0

.equ LED_DEBUG = 1

; for debug
.equ LED0_DDR	=	DDRB
.equ LED0_PORT	=	PORTB
.equ LED0_PIN	=	PB0

.equ LED1_DDR	=	DDRB
.equ LED1_PORT	=	PORTB
.equ LED1_PIN	=	PB4

;.equ LED5_DDR	=	DDRB
;.equ LED5_PORT	=	PORTB
;.equ LED5_PIN	=	PB3