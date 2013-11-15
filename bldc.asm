;
;	Electronic Speed Controller for a BLDC motor.
;	
;	Software written for ATMega8 microcontroller
;	
;	by tk
;	v 0.22
;	3.09.2013


; This software is still in Alpha version
; 
; The todo list:
; - I2C protocols
; - proper active rectification (unfinished yet)
; - governor mode (unfinished yet)
; - reverse
; - speed-current limit
; - hardware checks
; - beeps (unfinished yet)

.include "n11a01.inc"
;.include "mybldc.inc"
.include "bldc.inc"

.equ TIMER_PRESCALER = 1
.equ TICKS_PER_US = CLOCK_MHZ/TIMER_PRESCALER
.equ TICKS_PER_MS = TICKS_PER_US * 1000
.equ TICKS_PER_SEC = TICKS_PER_MS * 1000

.equ COM_T_RPM_MIN = 1000000*10*TICKS_PER_US/MIN_RPM
.equ COM_T_RPM_MAX = 1000000*10*TICKS_PER_US/MAX_RPM
.equ MIN_RPS = MIN_RPM / 60
.equ MAX_RPS = MAX_RPM / 60
.equ GOV_MAX_RPS = GOVERNOR_MAX_RPM / 60
.equ SPEED_THR_1 = 1000000*10*TICKS_PER_US/RPM_THR_1

.if CONTROL_METHOD == 1
 .equ POWER_RANGE = (RCP_MAX_THR - RCP_MIN_THR)*TICKS_PER_US
.elif CONTROL_METHOD == 2
 .equ POWER_RANGE = 255
.endif

.equ POWER_RANGE_L = low(POWER_RANGE)
.equ POWER_RANGE_H = high(POWER_RANGE)



.equ GOV_SPEED_CONST = POWER_RANGE * TICKS_PER_SEC / GOV_MAX_RPS / 6 ; 6 - 6 commutations per cycle
;.equ STARTUP_PULL_POWER_L = low(STARTUP_PULL_POWER*TICKS_PER_US)
;.equ STARTUP_PULL_POWER_H = high(STARTUP_PULL_POWER*TICKS_PER_US)
.equ STARTUP_MAX_POWER_L = low(POWER_RANGE*STARTUP_MAX_POWER/100)
.equ STARTUP_MAX_POWER_H = high(POWER_RANGE*STARTUP_MAX_POWER/100)
.equ STARTUP_MIN_POWER_L = low(POWER_RANGE*STARTUP_MIN_POWER/100)
.equ STARTUP_MIN_POWER_H = high(POWER_RANGE*STARTUP_MIN_POWER/100)
.equ STARTUP_COM_TIMEOUT = 1000000*10*TICKS_PER_US/STARTUP_MIN_RPM

.equ COM_DELAY = 128 - TIMING_ADVANCE

.if RCP_CHANNEL == 0
	.equ MCUCR_VAL = 1<<ISC00
	.equ GICR_VAL = 1<<INT0
	.equ RCP_DIR_REG = DDRD
	.equ RCP_PIN_REG = PIND
	.equ RCP_PIN = PD2
.else
.if RCP_CHANNEL == 1
	.equ MCUCR_VAL = 1<<ISC10
	.equ GICR_VAL = 1<<INT1
	.equ RCP_DIR_REG = DDRD
	.equ RCP_PIN_REG = PIND
	.equ RCP_PIN = PD3
.else
	.error	"Invalid constant: RCP_CHANNEL. Please select 0 (external interrupt 0) or 1 (external interrupt 1)"
.endif
.endif

; Beeper on state in one cycle time [us]
.equ BEEP_ON_TIME = 20

; Beeper sound frequencies [Hz]
.equ BPF_C6 =	1047
.equ BPF_Db6 =	1108
.equ BPF_D6 =	1175
.equ BPF_Eb6 =	1245
.equ BPF_E6 =	1319
.equ BPF_F6 =	1397
.equ BPF_Gb6 =	1480
.equ BPF_G6 =	1568
.equ BPF_Ab6 =	1661
.equ BPF_A6 =	1760
.equ BPF_Hb6 =	1856
.equ BPF_H6 =	1976
.equ BPF_C7 =	2093

; *------------------*
; |     Registers    |
; *------------------*

; RC pulse registers
.def rcp_timeout = r6
.def rcp_fail_count = r7
.def rcp_time_l = r8			
.def rcp_time_h = r9

; Sigma-delta modulator registers
.def sdm_factor_l = r10
.def sdm_factor_h = r11
.def sdm_err_l = r12
.def sdm_err_h = r13

; General purpose registers
.def AL = r0
.def AH = r1
.def BL = r2
.def BH = r3
.def CL = r16
.def CH = r17
.def DL = r18
.def DH = r19

; General purpose registers in interrupt code
.def IL = r4
.def IH = r5
.def JL = r20
.def JH = r21
.def JE = r22

; General flag registers
.def flags = r23					; pwm generator state flag
	.equ SDM_ACTIVE = 0				; is pwm wave being generated? no (0), yes (1)
	.equ SDM_STATE = 1				; set once 16bit timer1 overflows	
	.equ TIMER_READY = 2				; ready flag A
	.equ BRAKED = 3					; set if motor is braked
	.equ STARTUP = 4				; set if motor is in the starting mode
	.equ RUN = 5					; set if motor is in the running mode
	.equ TIMER_COMMUTATE = 6			; should timer call the next commutation?
	.equ TIMER_START_ZC_SCAN = 7

.def flags2 = r24
	.equ CALC_IN_PROGRESS = 0			; 1 if calcuations are in progress
	.equ BEEPER = 2
	.equ BEEP_CYCLE = 3
	.equ RCP_AWAITING_L = 4				; set if waiting for falling RC signal edge
	.equ RCP_CHECK_TIMEOUT = 5
	.equ SIGNAL_READY = 6
	.equ SIGNAL_ERROR = 7

.def flags3 = r25
	.equ UPDATE_RPM = 0
	.equ CYCLE_DONE = 1
	

; *------------------*
; |       RAM        |
; *------------------*
.dseg
.org SRAM_START
	startup_counter:	.byte	1		; number of successful cycles in startup mode
	startup_attempts:	.byte	1
	startup_forced_com_num:	.byte	1
	timing_angle:		.byte	1		; current angle between zero cross detection and commutation switch
	com_length_l:		.byte	1		; commutation length
	com_length_h:		.byte	1
	com_length_e:		.byte	1
	previous_zc_time_l:	.byte	1		; time of the previous zero cross detection
	previous_zc_time_h:	.byte	1
	previous_zc_time_e:	.byte	1
	cycle_length_l:		.byte	1		; averaged commutation length
	cycle_length_h:		.byte	1
	cycle_length_e:		.byte	1
	rpm_l:			.byte	1
	rpm_h:			.byte	1
	i2c_rpm_h:		.byte	1		
	zc_timeout_l:		.byte	1		; time of comparator scan activation
	zc_timeout_h:		.byte	1
	zc_timeout_e:		.byte	1
	power_signal_l:		.byte	1		; length of rc pulse
	power_signal_h:		.byte	1
	next_comm_call_addr_l:	.byte	1		; function call pointer to the next commutation
	next_comm_call_addr_h:	.byte	1
	beep_time_l:		.byte	1
	beep_time_h:		.byte	1
	beep_cycle_time_l:	.byte	1
	beep_cycle_time_h:	.byte	1
	signal_cnt:		.byte	1
	loop_led_cnt:		.byte	1
	tcnt1e:			.byte	1		; extended timer byte
	ocr1ae:			.byte	1		; timer compare output extended byte
	st_sreg:		.byte	1
	st_xl:			.byte	1
	st_xh:			.byte	1
	st_yl:			.byte	1
	st_yh:			.byte	1
	st_al:			.byte	1
	st_ah:			.byte	1
	st_bl:			.byte	1
	st_bh:			.byte	1	
	st_cl:			.byte	1
	st_ch:			.byte	1

	RAM_END:		.byte	1

.cseg
.org 0

; *------------------*
; |  Interrupt table |
; *------------------*

		rjmp	reset				; RESET
		.if CONTROL_METHOD == 1			; if RC pulse as signal source
		.if RCP_CHANNEL == 0
		 rjmp	rcp_int				; INT0
		 rjmp	inv_int				; INT1
		.else
		 rjmp	inv_int				; INT0
		 rjmp	rcp_int				; INT1
		.endif
		.else					; if no RC pulse
		 nop
		 rjmp	inv_int
		.endif
		rjmp	inv_int				; TIMER2 COMP
		rjmp	t2_ovf_int			; TIMER2 OVF
		rjmp	inv_int				; TIMER1 CAPT
		rjmp	timer_ocr1a_int			; TIMER1 COMPA
		rjmp	beep_ocr1b_int			; TIMER1 COMPB
		rjmp	timer_t1ovf_int			; TIMER1 OVF
		rjmp	sdm_sample_int			; TIMER0 OVF
		nop					; SPI, STC Serial Transfer Complete
		nop					; USART, RXC
		nop					; USART, UDRE
		nop					; USART, TXC
		nop					; ADC
		rjmp	inv_int				; EE_RDY
		rjmp	run_acomp_int			; ANA_COMP
		.if	CONTROL_METHOD == 2		; if I2C as signal source
		 rjmp	i2c_int				; TWI
		.else
		 nop
		.endif
		nop					; SPM_RDY
inv_int:
stop:		cli					; if unhandled interrupt occurs by mistake,
		rcall	motor_free_run			; at least we will know about it
stloop:		nop
		rjmp	stloop

; *------------------*
; |      General     |
; *------------------*

.macro flash_led
		push	XL
		ldi	XL, 0
		out	TCNT2, XL
		;led_dbg	2, 1
		pop	XL
.endmacro

.macro set_pin	.if @2					; if high state
		 sbi	@0, @1
		.else
		 cbi	@0, @1
		.endif
.endmacro

; args: led number, state (0/1)
.macro led	.if @0 == 0
		 .ifdef LED0_PORT
		  set_pin	LED0_PORT, LED0_PIN, @1
		 .endif
		.elif @0 == 1
		 .ifdef LED1_PORT
		  set_pin	LED1_PORT, LED1_PIN, @1
		 .endif
		.elif @0 == 2
		 .ifdef LED2_PORT
		  set_pin	LED2_PORT, LED2_PIN, @1
		 .endif
		.elif @0 == 3
		 .ifdef LED3_PORT
		  set_pin	LED3_PORT, LED3_PIN, @1
		 .endif
		.elif @0 == 4
		 .ifdef LED4_PORT
		  set_pin	LED4_PORT, LED4_PIN, @1
		 .endif
		.elif @0 == 5
		 .ifdef LED5_PORT
		  set_pin	LED5_PORT, LED5_PIN, @1
		 .endif
		.elif @0 == 6
		 .ifdef LED6_PORT
		  set_pin	LED6_PORT, LED6_PIN, @1
		 .endif
		.elif @0 == 7
		 .ifdef LED7_PORT
		  set_pin	LED7_PORT, LED7_PIN, @1
		 .endif
		.endif
.endmacro

; same as above, but checks if led debug is enabled
.macro led_dbg	.if LED_DEBUG
		 led @0, @1
		.endif
.endmacro

; *-------------------------------------------------------------------------------------------------------------------*
; |                                                     Timer                                                         |
; *-------------------------------------------------------------------------------------------------------------------*
; Here we implement the 24 bit timer for our ESC, with one timer tick being a one clock cycle. 
; Yes, it is an overkill, but there are no better options. If we used 16 bit timer1 without extended byte,
; with prescaler 8, the whole clock cycle would be only ~30ms, which is pretty much not enough for our purposes.
; And the 64 prescaler gives insufficient resolution on the other hand. So if the extended byte has to be used anyway,
; why not make it prescaler free. Entire clock cycle will take above one second to complete, which is more than enough.
; Speaking of 16MHz crystal here.
; The 3rd tmer byte is called tcnt1e and t's stored in RAM. The extended timer compare value byte called ocr1ae here.
; It's not like the first two bytes though, it's more like ocr1a multiplier. Makes some calculations a bit easier.

; Read the 24 bit timer value.
; We read timer1 value, extended byte stored in RAM [1] and check if timer1 overflow occured during the operation [2].
; If the overflow occured, we have to make sure if the extended byte is synchronized with the first two bytes.
; In order to do that, we check if the timer1 value is close to zero or close to max value (0xFFFF) [3].
; If it's close to zero and timer1 overflew, the extended byte is still old and it must be incremented [4].
; First 3 arguments are registers where read timer value will be written
; The 4th one is a temporary register, needed for reading operation.
; The 5th determines if "cli" and "sei" instructions will be used.
.macro	lti
	.if	@4 > 0
		cli
	.endif
		in	@0, TCNT1L
		in	@1, TCNT1H
		lds	@2, tcnt1e			; [1]	
		in	@3, TIFR
	.if	@4 > 0
		sei
	.endif
		sbrs	@3, TOV1			; [2]
		rjmp	lti_end				; no overflow during the operation, "return"
		sbrs	@1, 7				; check if the MSB is set. yes - timer1 close to max, no - close to zero. [3]
		inc		@2			; no. [4]
		lti_end:				; yes. the read value is fine
.endmacro

; Wait until the timer reaches set value
.macro	wait
wait_loop:	sbrs	flags, TIMER_READY
		rjmp	wait_loop
.endmacro

; the same but with watchdog reset
.macro	wait_wdr
wait_loop_wdr:	wdr
		sbrs	flags, TIMER_READY
		rjmp	wait_loop_wdr
.endmacro

; D, X, Y clobbered
.macro	run_timer
		ldi	XL, byte1(@0)
		ldi	XH, byte2(@0)
		ldi	YL, byte3(@0)
		rcall	timer_set_relative
.endmacro

; Waits specified time. D, X, Y clobbered, using timer.
.macro	delay
		run_timer @0
		wait
.endmacro

; Waits specified time. D, X, Y clobbered, using timer. Keeps watchdog timer reset
.macro	delay_wdr
		run_timer @0
		wait_wdr
.endmacro



; Set 24 bit timer absolute value. The timer will send the signal when it reaches the specified time.
; How it works: 24 bit OCR1A is set, then 24 bit TCNT1 is read to check if it isn't ahead OCR1A already.
; To do that, time delta between OCR1A and TCNT1 is calculated, and checked if it's below mid 24 bit timer value.
; If the MSB of the time delta extended byte is set, it's above mid value, we consider it passed, the timer flag is set to ready.
; The next commutation is also called in such case, if the TIMER_COMMUTATE flag is set
; Args: XL, XH, YL - 24 bit time to set
; Returns: YH, DL, DH - 24 bit current time
timer_set:	ldi	DL, 1<<OCF1A
		cli					; disable interrupts
		out	OCR1AH, XH
		out	OCR1AL, XL
		out	TIFR, DL			; clear interrupt flag if pending
		lti	YH, DL, DH, IL, 0		; read timer value. with 0 in the last arg, it won't reenable interrupts with "sei"
		cbr	flags, 1<<TIMER_READY
		sub	XL, YH				; calculate time delta between set ocr1a time and timer value
		sbc	XH, DL
		sbc	YL, DH
		sts	ocr1ae, YL			; set ocr1a value	
		sbrs	YL, 7				; is the most significant bit of time delta set?
		rjmp	ts_ret				; nah. the time hasn't come yet
		sbr	flags, 1<<TIMER_READY		; yes. time delta is above mid timer value, meaning time of compare match has passed already. set ready flag
		sbrc	flags, TIMER_COMMUTATE		; the timer has to call the commutation?
		rjmp	timer_switch_commutation	; yes
ts_ret:		reti					; no, enable interrupts and return (reti = sei + ret)


; Set 24 bit timer relative value. The timer will send the signal after time specified, counting from now.
; Args: YL:XH:XL timer relative value to set
; Clobbering: YH, DL, DH
; Pre: Make sure the time is at least 7 cycles, otherwise it won't work.
timer_set_relative:
		ldi	YH, 1<<OCF1A
		cli
		in	DL, TCNT1L			; read current time (16 bit timer1)	
		in	DH, TCNT1H
		add	DL, XL				; increment by time to wait
		adc	DH, XH
		out	OCR1AH, DH			; out it
		out	OCR1AL, DL
		out	TIFR, YH			; clear interrupt flag just in case interrupt is waiting
		cbr	flags, 1<<TIMER_READY
		sts	ocr1ae, YL			; store the extended byte
		reti					; reti = sei + ret


; Timer1 compare interrupt A
; Activates when timer reaches set value
; If specified, may switch to the next commutation
timer_ocr1a_int:
		in	IL, SREG
		lds	IH, ocr1ae
		dec	IH
		sts	ocr1ae, IH
		brpl	reti_sreg_restore
		sbr	flags, 1<<TIMER_READY		; --------------------opt


		; The timer may have been set to switch to the next commutation
		sbrs	flags, TIMER_COMMUTATE		; the timer has to call the commutation?
		rjmp	timer_zc_chk			; no, it doesn't

timer_switch_commutation:		
		lds	ZL, next_comm_call_addr_l	; load the function pointer to be called
		lds	ZH, next_comm_call_addr_h
		icall					; call the function
		wdr
		in	JL, TCNT1L
		in	JH, TCNT1H
		ldi	JE, 16				; ---------------------opt
		add	JL, JE
		ldi	JE, 0
		adc	JH, JE
		out	OCR1AH, JH
		out	OCR1AL, JL
		ldi	JE, 1<<OCF1A
		out	TIFR, JE
		cbr	flags, (1<<TIMER_READY) | (1<<TIMER_COMMUTATE)
		ldi	JE, 0
		sts	ocr1ae, JE
		sbr	flags, 1<<TIMER_START_ZC_SCAN
		out	SREG, IL
		reti

timer_zc_chk:	sbrs	flags, TIMER_START_ZC_SCAN	; the timer has to start ZC scan?
		rjmp	reti_sreg_restore		; no

timer_set_zc_scan:
		; Activate the zero crossing scan.
		; First, set the timeout for the operation.
		in	JL, TCNT1L
		in	JH, TCNT1H
		lds	JE, zc_timeout_l		; load calculated timeout for ZC
		add	JL, JE
		lds	JE, zc_timeout_h
		adc	JH, JE
		out	OCR1AH, JH
		out	OCR1AL, JL
		ldi	JE, 1<<OCF1A
		out	TIFR, JE
		cbr	flags, (1<<TIMER_READY) | (1<<TIMER_START_ZC_SCAN)
		lds	JE, zc_timeout_e
		sts	ocr1ae, JE

		; Activate comparator interrupts
		sbi	ACSR, ACIE

		; Return
reti_sreg_restore:
		out	SREG, IL
		reti


; Timer1 overflow interrupt
timer_t1ovf_int:
		in	IL, SREG
		lds	JL, tcnt1e
		inc	JL
		sts	tcnt1e, JL
		sbr	flags3, 1<<UPDATE_RPM
		dec	rcp_timeout
		breq	t_sig_fail
		out	SREG, IL
		reti
t_sig_fail:	rjmp	signal_err


; Timer2 overflow interrupt
t2_ovf_int:		in	IL, SREG
		;led_dbg	2, 0
		out	SREG, IL
		reti


; wait X * 8 cycles, args: XH:XL
delay_8cycles:	sbiw	XL, 1				; 2
		push	XL				; 2
		pop	XL				; 2
		brpl	delay_8cycles			; 2
		ret

; wait X * 1000 cycles, args: XH:XL
delay_1000cycles:	
		push	XL				; 2
		ldi	XL, 248				; 248 * 4 cycles = 992 cycles
d1000c_loop:	dec	XL
		nop
		brne	d1000c_loop						
		pop	XL				; 2
		sbiw	XL, 1				; 2
		brpl	delay_1000cycles		; 2
		ret


; *-------------------------------------------------------------------------------------------------------------------*
; |                                                 Signal input                                                      |
; *-------------------------------------------------------------------------------------------------------------------*

.if CONTROL_METHOD == 1
; --- RC pulse ----
.macro		rcp_init				; Initialize RC pulse input
		cbi	RCP_DIR_REG, RCP_PIN		; set rc pulse pin as input
		ldi	XL, MCUCR_VAL			; set external int to trigger on any logic change
		out	MCUCR, XL
		ldi	XL, GICR_VAL			; enable external interrupt
		out	GICR, XL
.endmacro

; External interrupt, triggers on any logic change
rcp_int:	in	IL, SREG
		sbic	RCP_PIN_REG, RCP_PIN		; is it low or high state?
		rjmp	rcp_rcp_high_state
		led_dbg	4, 0
		sbrs	flags2, RCP_AWAITING_L		; check if we are waiting for a falling edge
		rjmp	reti_sreg_restore				; no, return
		in	JL, TCNT1L			; yes, calculate length of the impulse
		in	JH, TCNT1H
		sub	JL, rcp_time_l
		sbc	JH, rcp_time_h
		ldi	JE, high(RCP_MIN*TICKS_PER_US)	; longer than min accepted pulse lenght?
		cpi	JL, low(RCP_MIN*TICKS_PER_US)
		cpc	JH, JE
		brlo	rcp_rcp_fail			; no, fail
		ldi	JE, high(RCP_MAX*TICKS_PER_US+1); yes. longer than 2200us?
		cpi	JL, low(RCP_MAX*TICKS_PER_US+1)
		cpc	JH, JE
		brsh	rcp_rcp_fail			; yes, fail
		sbr	flags2, 1<<SIGNAL_READY
		bst	flags2, SIGNAL_ERROR
		brtc	rcp_ok				; is it the first correct pulse after error?
		bst	flags, RUN			;  motor still running?
		brtc	rcp_cl
		mov	JL, XL				;  yes, activate sdm which could have been stopped due to error
		rcall	sdm_on
		mov	XL, JL				;  restore the XL register clobbered by the function
rcp_cl:		cbr	flags2, 1<<SIGNAL_ERROR		; no
rcp_ok:		clr	rcp_fail_count			; no, been ok before
		ldi	JE, 25				; ~ 100 ms timeout
		mov	rcp_timeout, JE
		sts	power_signal_l, JL
		sts	power_signal_h, JH
		out	SREG, IL
		reti
rcp_rcp_fail:	inc	rcp_fail_count			; if imuplses are too short/long
rcp_rcp_fail2:	ldi	JL, 3
		cp	rcp_fail_count, JL
		brsh	signal_err
		out	SREG, IL
		reti

rcp_rcp_high_state:					; rising edge detected. start measuring how long it is
		sbr	flags2, 1<<RCP_CHECK_TIMEOUT
		led_dbg	4, 1
		sbr	flags2, 1<<RCP_AWAITING_L	; mark that we are waiting for low state now
		in	rcp_time_l, TCNT1L
		in	rcp_time_h, TCNT1H
		ldi	JE, 3				; ~8000 - ~12000 us timeout
		mov	rcp_timeout, JE
		out	SREG, IL
		reti

.elif CONTROL_METHOD == 2
; --- I2C ---
.macro		i2c_init
		ldi	XL, I2C_SLAVE_ADDRESS<<1
		out	TWAR, XL
		ldi	XL, (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN)
		out	TWCR, XL
		;led	1, 1
.endmacro
	
; I2C interrupt	
i2c_int:	in	IL, SREG
		in	JE, TWSR
		;andi	JE, 0b11111000			; unnecessary for now, these 3 bits are 0 anyway

		; Check I2C status register, see what happened. See Atmega datasheet for more info.
		cpi	JE, 0x60			; SR: SLA+W received, ACK returned
		breq	i2c_sla_w_rec
		cpi	JE, 0x80			; SR: Data received, ACK returned
		breq	i2c_data_rec
		cpi	JE, 0xA8			; ST: SLA+R received, ACK returned
		breq	i2c_sla_r_rec
		cpi	JE, 0xB8			; ST: Data from TWDR sent, ACK received
		breq	i2c_data_sent
		cpi	JE, 0x70			; SR: General call received, ACK returned
		breq	i2c_sla_w_rec
		cpi	JE, 0x90			; SR: Data after general call received, ACK returned
		breq	i2c_data_rec
		tst	JE
		breq	i2c_bus_error
		
		; Unhandled operations (handled by i2c_reset); Slave receiver: 0x68, 0x78, 0x88, 0x98, 0xA0
		; Slave transmitter: 0xB0, 0xC0, 0xC8
i2c_reset:	ldi	JE, (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN)
		out	TWCR, JE			; Switch to unadressed mode
		clr	IH
		;sts	power_signal_l, IH
		;sts	power_signal_h, IH
		out	SREG, IL
		reti
		
		; SR: We have received our slave address, with direction bit set to Slave Receiver
		; We'll be receiving data now
i2c_sla_w_rec:	ldi	JE, (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN)
		out	TWCR, JE			; ACK it
		out	SREG, IL
		reti

		; SR: If data has been received
i2c_data_rec:	in	IH, TWDR
		sts	power_signal_l, IH
		clr	IH
		sts	power_signal_h, IH
		ldi	JE, (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN)
		out	TWCR, JE
		sbr	flags2, 1<<SIGNAL_READY
		cbr	flags2, 1<<SIGNAL_ERROR
		ldi	JE, 25				; ~ 100 ms timeout
		mov	rcp_timeout, JE
		out	SREG, IL
		reti
			
		; ST: We have received our slave address, with direction bit set to Slave Transmitter
		; We'll be transmitting data now
i2c_sla_r_rec:	lds	JE, rpm_h
		sts	i2c_rpm_h, JE
		lds	JE, rpm_l
		out	TWDR, JE
		ldi	JE, (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN)
		out	TWCR, JE			; ACK it
		out	SREG, IL
		reti

i2c_data_sent:	lds	JE, i2c_rpm_h
		out	TWDR, JE
		ldi	JE, (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN)
		out	TWCR, JE			; ACK it
		out	SREG, IL
		reti

		; Bus error has occured
i2c_bus_error:	ldi	JE, (1<<TWSTO) | (1<<TWINT)
		clr	IH				; Release the I2C lines allowing onther devices to work
		sts	power_signal_l, IH
		sts	power_signal_h, IH
		sbr	flags2, 1<<SIGNAL_ERROR
		out	SREG, IL
		reti
.endif
		
signal_err:
		clr	rcp_fail_count
		sbr	flags2, (1<<SIGNAL_READY) | (1<<SIGNAL_ERROR)
		cbr	flags2, 1<<RCP_AWAITING_L
signal_power_off:	clr	JL
		sts	power_signal_l, JL
		sts	power_signal_h, JL
		out	SREG, IL
		reti

; Parse the power value incoming from RCP/I2C
; In case of RCP, limit the pulse length to the power range
; Returns power value in XH:XL
update_power:	;cbr	flags2, 1<<SIGNAL_READY
		cli					; cli, we don't want the rcp_int interrupt between these two lines
		lds	XL, power_signal_l		; read length of the pulse
		lds	XH, power_signal_h
		sei
		.if	CONTROL_METHOD == 1
		ldi	YH, high(RCP_MAX_THR*TICKS_PER_US+1)
		cpi	XL, low(RCP_MAX_THR*TICKS_PER_US+1)	; longer than full power threshold?
		cpc	XH, YH
		brsh	up_fp				; yes, full power, jump
		subi	XL, low(RCP_MIN_THR*TICKS_PER_US)
		sbci	XH, high(RCP_MIN_THR*TICKS_PER_US)
		brcs	up_np				; carry set? was shorter than no power threshold? no power, jump.
		ret					; nah, power non-zero, return
up_fp:		ldi	XL, POWER_RANGE_L		; if full power
		ldi	XH, POWER_RANGE_H
		ret
up_np:		clr	XL				; if no signal or power zero
		clr	XH
		.endif
		ret

handle_signal_error:
		sbrc	flags, SDM_ACTIVE
		rcall	sdm_off
		sbrs	flags, BRAKED
		rjmp	motor_brake


; *-------------------------------------------------------------------------------------------------------------------*
; |                                                    Power stage                                                    |
; *-------------------------------------------------------------------------------------------------------------------*

; mosfet operations
.if PWM_SIDE == 1					; low side does the PWM
	; phase R
	.macro	RX_on
		;nop
		sbi	RL_PORT, RL_PIN
	.endmacro
	.macro	RX_off
		cbi	RL_PORT, RL_PIN
	.endmacro
	.macro	RY_on
		;nop
		sbi	RH_PORT, RH_PIN
	.endmacro
	.macro	RY_off
		cbi	RH_PORT, RH_PIN
	.endmacro

	; phase S
	.macro	SX_on
		;nop
		sbi	SL_PORT, SL_PIN
	.endmacro
	.macro	SX_off
		cbi	SL_PORT, SL_PIN
	.endmacro
	.macro	SY_on
		;nop
		sbi	SH_PORT, SH_PIN
	.endmacro
	.macro	SY_off
		cbi	SH_PORT, SH_PIN
	.endmacro

	; phase T
	.macro	TX_on
		;nop
		sbi	TL_PORT, TL_PIN
	.endmacro
	.macro	TX_off
		cbi	TL_PORT, TL_PIN
	.endmacro
	.macro	TY_on
		;nop
		sbi	TH_PORT, TH_PIN
	.endmacro
	.macro	TY_off
		cbi	TH_PORT, TH_PIN
	.endmacro
.else							; high side does the PWM
	.macro	RX_on
		sbi	RH_PORT, RH_PIN
	.endmacro
	.macro	RX_off
		cbi	RH_PORT, RH_PIN
	.endmacro
	.macro	RY_on
		sbi	RL_PORT, RL_PIN
	.endmacro
	.macro	RY_off
		cbi	RL_PORT, RL_PIN
	.endmacro

	; phase S
	.macro	SX_on
		sbi	SH_PORT, SH_PIN
	.endmacro
	.macro	SX_off
		cbi	SH_PORT, SH_PIN
	.endmacro
	.macro	SY_on
		sbi	SL_PORT, SL_PIN
	.endmacro
	.macro	SY_off
		cbi	SL_PORT, SL_PIN
	.endmacro

	; phase T
	.macro	TX_on
		sbi	TH_PORT, TH_PIN
	.endmacro
	.macro	TX_off
		cbi	TH_PORT, TH_PIN
	.endmacro
	.macro	TY_on
		sbi	TL_PORT, TL_PIN
	.endmacro
	.macro	TY_off
		cbi	TL_PORT, TL_PIN
	.endmacro
.endif

fet_delay:	nop
		nop
		nop
		nop
		nop
		ret

.macro	fets_off
		RY_off
		SY_off		
		TY_off
		RX_off
		SX_off
		TX_off
.endmacro

; cuts the power off, all power FETs closed
motor_free_run:	rcall	sdm_off				; sdm generator off, so it doesn't open the FETs
		fets_off
		cbr	flags, 1<<BRAKED
		ret

; cuts the power off but leaves one side of the bridge open which creates a short circuit between all the phases,
; making the motor a generator with infinite load and thus making high braking torque.
motor_brake:	rcall	sdm_off
		RY_off
		SY_off
		TY_off
		RX_on
		SX_on
		TX_on
		sbr	flags, 1<<BRAKED
		ret

; Commutation subroutines, they must be called from interrupts, or with disabled interrupts, between cli and sei.
; R->S, T undriven. RH open, SL pwm
commutation_50:	ldi	JL, T_COMP_CHANNEL		; switch comparator channel
		out	ADMUX, JL
		TY_off					; close the T phase lower mosfet
		RY_on					; open the R phase higher mosfet
;		bst	flags, SDM_STATE
;		brtc	c0_low				; should it be low sdm state now?
;		.if ACTIVE_FREEWHEELING			; the high S fet may be on now, because of complementary pwm.
;			SY_off				; turn it off
;			fet_delay
;		.endif
;		SX_on					; no, open the S phase lower mosfet (pwm)
c0_low:		ldi	ZL, low(sdm_j0)			; set jump address for sdm generator
		ldi	ZH, high(sdm_j0)
		ldi	JL, low(commutation_01)
		sts	next_comm_call_addr_l, JL
		ldi	JL, high(commutation_01)
		sts	next_comm_call_addr_h, JL
		cbi	ACSR, ACIE
		sbi	ACSR, ACIS0			; set comparator to trigger on rising edge
		;sbi	ACSR, ACI			; clear possibly pending comparator interrupt
		;sbrc	flags, RUN
		;sbi	ACSR, ACIE
		;sbi	ACSR, ACI
		ret

; R->T, S undriven. RH open, TL pwm
commutation_01:	ldi	JL, S_COMP_CHANNEL
		out	ADMUX, JL
		SX_off
		;RY_on
		.if ACTIVE_FREEWHEELING
		  SY_off
		.endif
		bst	flags, SDM_STATE
		brtc	c1_low
		TX_on
c1_low:		ldi	ZL, low(sdm_j2)
		ldi	ZH, high(sdm_j2)
		ldi	JL, low(commutation_12)
		sts	next_comm_call_addr_l, JL
		ldi	JL, high(commutation_12)
		sts	next_comm_call_addr_h, JL
		;led_dbg	1, 0
		cbi	ACSR, ACIE		
		cbi	ACSR, ACIS0			; set comparator to trigger on falling edge
		;sbi	ACSR, ACI
		;sbrc	flags, RUN
		;sbi	ACSR, ACIE
		;sbi	ACSR, ACI
		ret

; S->T, R undriven. SH open, TL pwm
commutation_12:	ldi	JL, R_COMP_CHANNEL
		out	ADMUX, JL
		RY_off
		SY_on
;		bst	flags, SDM_STATE
;		brtc	c2_low
;		.if ACTIVE_FREEWHEELING
;			TY_off
;			fet_delay
;		.endif
;		TX_on
c2_low:		ldi	ZL, low(sdm_j2)
		ldi	ZH, high(sdm_j2)
		ldi	JL, low(commutation_23)
		sts	next_comm_call_addr_l, JL
		ldi	JL, high(commutation_23)
		sts	next_comm_call_addr_h, JL
		cbi	ACSR, ACIE		
		sbi	ACSR, ACIS0
		;sbrc	flags, RUN
		;sbi	ACSR, ACIE
		;sbi	ACSR, ACI
		ret

; S->R, T undriven. SH open, RL pwm
commutation_23:	ldi	JL, T_COMP_CHANNEL
		out	ADMUX, JL
		TX_off
;		SY_on
		.if ACTIVE_FREEWHEELING
		  TY_off
		.endif
		bst	flags, SDM_STATE
		brtc	c3_low
		RX_on
c3_low:		ldi	ZL, low(sdm_j4)
		ldi	ZH, high(sdm_j4)
		ldi	JL, low(commutation_34)
		sts	next_comm_call_addr_l, JL
		ldi	JL, high(commutation_34)
		sts	next_comm_call_addr_h, JL
		cbi	ACSR, ACIE
		cbi	ACSR, ACIS0
		sbi	ACSR, ACI
		;sbrc	flags, RUN
		;sbi	ACSR, ACIE
		;sbi	ACSR, ACI
		ret

; T->R, S undriven. TH open, RL pwm
commutation_34:	ldi	JL, S_COMP_CHANNEL
		out	ADMUX, JL
		SY_off
		TY_on
		;led_dbg	1, 0
;		bst	flags, SDM_STATE
;		brtc	c4_low
;		.if ACTIVE_FREEWHEELING
;			RY_off
;			fet_delay
;		.endif
;		RX_on
c4_low:		ldi	ZL, low(sdm_j4)
		ldi	ZH, high(sdm_j4)
		ldi	JL, low(commutation_45)
		sts	next_comm_call_addr_l, JL
		ldi	JL, high(commutation_45)
		sts	next_comm_call_addr_h, JL
		cbi	ACSR, ACIE
		sbi	ACSR, ACIS0
		;sbrc	flags, RUN
		;sbi	ACSR, ACIE
		;sbi	ACSR, ACI
		ret

; T->S, R undriven. TH open, SL pwm
commutation_45:	ldi	JL, R_COMP_CHANNEL
		out	ADMUX, JL
		RX_off
		;TY_on
		;led_dbg	1, 0
		;led_dbg	6, 0
		.if ACTIVE_FREEWHEELING
		  RY_off
		.endif
		bst	flags, SDM_STATE
		brtc	c5_low
		SX_on
c5_low:		ldi	ZL, low(sdm_j0)
		ldi	ZH, high(sdm_j0)
		ldi	JL, low(commutation_50)
		sts	next_comm_call_addr_l, JL
		ldi	JL, high(commutation_50)
		sts	next_comm_call_addr_h, JL
		cbi	ACSR, ACIE		
		cbi	ACSR, ACIS0
		;sbrc	flags, RUN
		;sbi	ACSR, ACIE
		;sbi	ACSR, ACI
		ret

; call the next commutation. used in startup subroutine. during the run, the timer is responsible for commutation.
commutate:	cli
		lds	ZL, next_comm_call_addr_l
		lds	ZH, next_comm_call_addr_h
		icall
		wdr
		sei
		ret

; *-------------------------------------------------------------------------------------------------------------------*
; |                                                     Beeper	                                                      |
; *-------------------------------------------------------------------------------------------------------------------*

; Enable beep timer
; args: YH:YL cycle time (1s/f). unmodified.
; clobbered: XL, XH
beep_on:	sbrs	flags, SDM_ACTIVE
		rcall	motor_free_run			; all FETs off, SDM off
		in	XL, TIMSK
		sbr	XL, 1<<OCIE1B
		out	TIMSK, XL
		sbr	flags2, 1<<BEEPER
		sts	beep_cycle_time_l, YL
		sts	beep_cycle_time_h, YH
		RX_on
bp_lp:		sbrs	flags2, BEEP_CYCLE		; wait for timer to give a signal for single cycle
		rjmp	bp_ct				; no signal, do down then to check timer
		cbr	flags2, 1<<BEEP_CYCLE
		ldi	XL, low(BEEP_ON_TIME*TICKS_PER_US*TIMER_PRESCALER/8)
		ldi	XH, high(BEEP_ON_TIME*TICKS_PER_US*TIMER_PRESCALER/8)
		TY_on
		rcall	delay_8cycles
		TY_off
		wdr
bp_ct:		sbrs	flags, TIMER_READY		; check timer. if it's ready, break the loop
		rjmp	bp_lp
		in	XL, TIMSK
		cbr	XL, 1<<OCIE1B
		out	TIMSK, XL
		rcall	motor_free_run
		sbrc	flags, BRAKED
		rcall	motor_brake
		cbr	flags2, 1<<BEEPER		; give it some time for demagnetization not to destroy the fet with inductive kickback
		ldi	XL, low(CLOCK_MHZ*1000000/1000/333)	; 3 ms
		ldi	XH, high(CLOCK_MHZ*1000000/1000/333)
		rcall	delay_1000cycles
		wdr
		RX_off
		ret

; args: time[ms], freq[hz]. If frequency given 0, DH:DL will be used
.macro	beep_sound
		run_timer	@0*TICKS_PER_MS		; set time length of the beep
		.if	@1
		  ldi	YL, low(TICKS_PER_SEC/@1)	; set frequency
		  ldi	YH, high(TICKS_PER_SEC/@1)
		.else
		  movw	YL, DL
		.endif
		rcall	beep_on
.endmacro

; Timer1 compare interrupt B
beep_ocr1b_int:	in	JE, SREG
		lds	JL, beep_time_l
		lds	JH, beep_time_h
		lds	IL, beep_cycle_time_l
		lds	IH, beep_cycle_time_h
		add	JL, IL
		adc	JH, IH
		out	OCR1BH, JH
		out	OCR1BL, JL
		sts	beep_time_l, JL
		sts	beep_time_h, JH
		sbr	flags2, 1<<BEEP_CYCLE
		out	SREG, JE
		reti

; Separately, don't want to inline it a few times
beep_delay_16:	delay_wdr 16*TICKS_PER_MS
		ret

; Three sounds, ESC power on.
beep_po:	led	0, 1
		led	1, 1
		beep_sound 84,	BPF_C6
		rcall	beep_delay_16
		beep_sound 84,	BPF_E6
		rcall	beep_delay_16
		beep_sound 100,	BPF_G6
		led	0, 0
		led	1, 0
		ret

; Watchdog reset occured. Fail sound
beep_wdr:	led	0, 1
		beep_sound 133,	(BPF_E6/4)
		rcall	beep_delay_16
		beep_sound 133,	(BPF_Eb6/4)
		rcall	beep_delay_16
		beep_sound 170,	(BPF_D6/4)
		rcall	beep_delay_16
		beep_sound 220,	(BPF_Db6/4)
		rcall	beep_delay_16
		beep_sound 400,	(BPF_C6/4)
		led	0, 0
		ret

; Brown out reset
beep_bo:	ldi	DL, low(BPF_Ab6)
		ldi	DH, high(BPF_Ab6)
		ldi	CL, high(BPF_Ab6/8)
		led	0, 1
bp_bo_loop:	beep_sound 20, 0
		subi	DL, 40
		sbci	DH, 0
		cpi	DL, low(BPF_Ab6/8)
		cpc	DH, CL
		brsh	bp_bo_loop
		led	0, 0
		ret

; External reset
beep_extr:	led	1, 1
		beep_sound 150,	BPF_A6/2
		led	1, 0
		delay_wdr 300*TICKS_PER_MS
		led	1, 1
		beep_sound 150,	BPF_A6/2
		led	1, 0
		ret

; Unrecognized reset reason, no flag set.
beep_single:	led	0, 1
		beep_sound 166,	BPF_G6/4
		led	0, 0
		ret
	
; *-------------------------------------------------------------------------------------------------------------------*
; |                                              Sigma-delta modulator                                                |
; *-------------------------------------------------------------------------------------------------------------------*
; Implementation of Sigma-delta modulator
; Instead of typical PWM for output power control, we use Sigma-Delta modulation (or Pulse-density modulation)
; It's easier to implement from the side of software and results in better, linear power control.
; No power jump between 99 and 100% throttle.
; http://en.wikipedia.org/wiki/Pulse-density_modulation


; Activate the SDM generator
sdm_on:		in	XH, TIMSK
		sbr	XH, 1<<TOIE0
		sbr	flags, 1<<SDM_ACTIVE
		cbr	flags, (1<<BRAKED) | (1<<SDM_STATE)
		clr	sdm_err_l
		clr	sdm_err_h
		out	TIMSK, XH			; enable the interrupt
		ret

; Disable SDM
sdm_off:	in	XH, TIMSK
		cbr	XH, 1<<TOIE0
		cbr	flags, 1<<SDM_ACTIVE
		out	TIMSK, XH
		;led_dbg	5, 0
		ret

; Timer0 overflow interrupt
; Delta-sigma modulator
; This interrupt is called every 256 clock cycles, which gives 62.5KHz sampled delta sigma modulation
; on 16 MHz crystal. Thanks to high optimization of the code, each interrupt takes about 22 cycles,
; which is only ~9% of total cpu time. The jump address for "ijmp" is kept in Z register which is set
; in commutation functions.
sdm_sample_int:
		in	IL, SREG
		movw	JL, sdm_factor_l
		bst	flags, SDM_STATE
		brtc	sdm_tc				; low state now?
		subi	JL, POWER_RANGE_L		; no, skip this
		sbci	JH, POWER_RANGE_H
sdm_tc:		sub	sdm_err_l, JL			; yes
		sbc	sdm_err_h, JH
		ijmp					; jump to the address set in Z register

sdm_j0:		brmi	sdm_h0				; commutations 5 and 0
sdm_l0:	.if ACTIVE_FREEWHEELING				; set the low state
		brtc	sdm_ret				; if it's low already, just return
		SX_off
		rcall	fet_delay
		SY_on
	.else
		SX_off
	.endif
		led_dbg	5, 0
		wdr
		cbr	flags, 1<<SDM_STATE
		out	SREG, IL
		reti
sdm_h0:	.if ACTIVE_FREEWHEELING				; set the high pwm state
		brts	sdm_ret				; if it's high already, just return
		SY_off
		rcall	fet_delay
	.endif
		SX_on
		led_dbg	5, 1
		sbr	flags, 1<<SDM_STATE
		out	SREG, IL
		reti
sdm_j2:		brmi	sdm_h2				; commutations 1 and 2
sdm_l2:	.if ACTIVE_FREEWHEELING
		brtc	sdm_ret
		TX_off
		rcall	fet_delay
		TY_on
	.else
		TX_off
	.endif
		led_dbg	5, 0
		wdr
		cbr	flags, 1<<SDM_STATE
		out	SREG, IL
		reti
sdm_h2:	.if ACTIVE_FREEWHEELING
		brts	sdm_ret
		TY_off
		rcall	fet_delay
	.endif
		TX_on
		led_dbg	5, 1
		sbr	flags, 1<<SDM_STATE
sdm_ret:	out	SREG, IL
		reti
sdm_j4:		brmi	sdm_h4				; commutations 3 and 4
sdm_l4:	.if ACTIVE_FREEWHEELING
		brtc	sdm_ret
		RX_off
		rcall	fet_delay
		RY_on
	.else
		RX_off
	.endif
		led_dbg	5, 0
		wdr
		cbr	flags, 1<<SDM_STATE
		out	SREG, IL
		reti
sdm_h4:	.if ACTIVE_FREEWHEELING
		brts	sdm_ret
		RY_off
		rcall	fet_delay
	.endif
		RX_on
		led_dbg	5, 1
		sbr	flags, 1<<SDM_STATE
		out	SREG, IL
		reti

; *-------------------------------------------------------------------------------------------------------------------*
; |                                                     Start-up                                                      |
; *-------------------------------------------------------------------------------------------------------------------*

; calculate commutation timeout
start_set_commutation_timeout:
		ldi	XL, byte1(STARTUP_COM_TIMEOUT)
		ldi	XH, byte2(STARTUP_COM_TIMEOUT)
		ldi	YL, byte3(STARTUP_COM_TIMEOUT)
		lti	AL, AH, BL, DL, 1
		add	XL, AL
		adc	XH, AH
		adc	YL, BL
		rjmp	timer_set

; Return comparator state.
; This function used during startup is different than the one used during run.
; It returns the comparator state right from before the output pwm goes high.
; The point is to reduce the influence of coil demagnetization and other noise sources.
start_read_comparator_state:
		sbrc	flags, TIMER_READY		; check if timeout
		ret					; yes
		bst	flags, SDM_STATE
		brts	start_read_comparator_state	; high pwm state? wait for low.
		clr	XH				; should be low pwm state now
srcs_loop:						; now wait for high state
		sbrc	flags, TIMER_READY		; timeout?
		ret					; yes
		mov	XL, XH				; save previous read comparator state
		in	XH, ACSR			; read comparator register
		andi	XH, (1<<ACO)			; extract comparator state bit
		bst	flags, SDM_STATE
		brtc	srcs_loop			; low pwm state?
		ret					; no

start_wait_bemf_low:
		sbrc	flags, TIMER_READY		; check if timeout
		ret					; yes, return
		rcall	start_read_comparator_state	; no
		ldi	XH, (1<<ACO)
		cpse	XL, XH
		rjmp	start_wait_bemf_low
		ret

start_wait_bemf_high:
		sbrc	flags, TIMER_READY		; check if timeout
		ret					; yes
		rcall	start_read_comparator_state
		clr	XH
		cpse	XL, XH
		rjmp	start_wait_bemf_high
		ret

; clobbers XL, XH
; wait for zero cross high->low (or timeout)
start_wait_zc_low:
		rcall	start_wait_bemf_high
		sbrc	flags, TIMER_READY
		rjmp	swz_to
		rcall	start_wait_bemf_low
		sbrs	flags, TIMER_READY
		ret
		rjmp	swz_to

; wait for zero cross low->high (or timeout)
start_wait_zc_high:
		rcall	start_wait_bemf_low
		sbrc	flags, TIMER_READY
		rjmp	swz_to
		rcall	start_wait_bemf_high
		sbrs	flags, TIMER_READY		; timeout?
swz_ret:	ret					; no, just ret
swz_to:		lds	XL, startup_forced_com_num
		dec	XL				; there is some number of forced commutations allowed
		sts	startup_forced_com_num, XL
		brpl	swz_ret				; if not reached yet, keep returning
		pop	XL				; zc detections failed, go back a function call
		pop	XL				; remove 16 bit return address from stack
		rjmp	start_up_again			; go to the start

start_reset_counters:
		ldi	XL, START_UP_COUNTER+1		; set number of commutations that must be done sucessfully in start
		sts	startup_counter, XL
		ldi	XL, STARTUP_FORCED_COM_NUMBER	; set number of commutations to be forced if no zc is detected in time
		sts	startup_forced_com_num, XL
		ret

start_handle_signal:
		sbrc	flags2, SIGNAL_ERROR		; signal error?
		rcall	handle_signal_error		; yes, handle it
		rcall	update_power			; read power
		adiw	XL, 0				; if it is 0 or there's an error, go to start
		brne	shs_ret				; non-zero? just return
		pop	XL				; was zero, return to main loop
		pop	XL
		rcall	start_reset_counters
		cbr	flags, 1<<STARTUP
shs_ret:	ret

; start up procedure
start_up_again:	lds	XL, startup_attempts
		inc	XL
		sts	startup_attempts, XL
start_up:	clr	XL
		sts	rpm_l, XL
		sts	rpm_h, XL
		rcall	start_handle_signal
		sbr	flags, (1<<STARTUP)
		cbr	flags, (1<<RUN)
		rcall	commutate
		rcall	commutate
		rcall	sdm_on				; activate sigma-delta generator

start_loop:	rcall	start_handle_signal
		sts	previous_zc_time_l, BL		; store the time of the previous ZC. these variables need to be
		sts	previous_zc_time_h, BH		; set properly in case it switched to running mode right after
		sts	previous_zc_time_e, CL

		ldi	YH, STARTUP_MAX_POWER_H
		cpi	XL, STARTUP_MAX_POWER_L		; compare the power value with max allowed start-up power
		cpc	XH, YH
		brsh	su_lpu				; higher? limit power (up)
		ldi	YH, STARTUP_MIN_POWER_H
		cpi	XL, STARTUP_MIN_POWER_L		; compare the power value with min allowed start-up power
		cpc	XH, YH
		brlo	su_lpd				; lower? limit power (down)
su_ssf:		movw	sdm_factor_l, XL		; set the sdm factor
		rjmp	su_pow_ok
su_lpu:		ldi	XL, STARTUP_MAX_POWER_L		; limit power up
		ldi	XH, STARTUP_MAX_POWER_H
		movw	sdm_factor_l, XL		; like this, because two sdm_factor bytes must be saved at once
		rjmp	su_pow_ok
su_lpd:		ldi	XL, STARTUP_MIN_POWER_L		; limit power down
		ldi	XH, STARTUP_MIN_POWER_H
		movw	sdm_factor_l, XL	
su_pow_ok:	wait					; wait until ZC scan is allowed
		rcall	start_set_commutation_timeout
		
		; Even though we are not using comparator interrupts in start-up mode, the awaited comparator edge (zero cross type)
		; is stored in the ACSR comparator register (has been written there by the previous commutation routine)
		; In this way we gain one free bit in our "flags", "flags2" registers.
		sbic	ACSR, ACIS0			; which type of ZC are we waiting for?
		rcall	start_wait_zc_low
		sbis	ACSR, ACIS0
		rcall	start_wait_zc_high

		lti	BL, BH, CL, CH, 1		; read ZC time
		rcall	commutate
		ldi	XL, byte1(40*TICKS_PER_US)	; so the next zc scan won't happen in 40 us
		ldi	XH, byte2(40*TICKS_PER_US)
		ldi	YL, byte3(40*TICKS_PER_US)
		add	XL, BL
		adc	XH, BH
		adc	YL, CL
		rcall	timer_set

		lds	AL, startup_counter
		dec	AL
		sts	startup_counter, AL
		brne	start_loop
		clr	XL
		sts	startup_attempts, XL
		rjmp	running_mode_switch


; *-------------------------------------------------------------------------------------------------------------------*
; |                                                        Run                                                        |
; *-------------------------------------------------------------------------------------------------------------------*

; The procedure of switch from start-up to run mode.
; There are many variables that have to be set before such switch
running_mode_switch:
		lti	XL, XH, YL, BL, 1		; read the time of ZC that just occured
		lds	YH, previous_zc_time_l
		lds	DL, previous_zc_time_h
		lds	DH, previous_zc_time_e
		sts	previous_zc_time_l, XL		; update the old one
		sts	previous_zc_time_h, XH
		sts	previous_zc_time_e, YL
		movw	AL, XL
		mov	BL, YL
		sub	XL, YH				; calculate the time delta between them
		sbc	XH, DL
		sbc	YL, DH
		sts	com_length_l, XL		; got commutation length
		sts	com_length_h, XH
		sts	com_length_e, YL
		lsl	XL				; multiply by 4 (let's give it a bit more time than usual, 
		rol	XH				; first cycle may be problematic)
		rol	YL
		lsl	XL
		rol	XH
		rol	YL
		rcall	timer_set_relative		; Set the timer	

		clr	XL				; Clear some other stuff
		sts	timing_angle, XL
		cbr	flags, 1<<STARTUP
		sbr	flags, 1<<RUN
		rcall	update_power
	
		
		; Config the comparator
		sbi	ACSR, ACIS1			; This one always must be set (triggers on logic change)
		sbi	ACSR, ACI			; Clear pending interrupt (if there is one)
		nop
		ldi	XL, 32
		ldi	XL, 0
		rcall	delay_8cycles
		sbi	ACSR, ACIE			; Enable interrupts
		
		rjmp	run_wait_zc_timeout		; Everything is set now. Await ZC timeout.


; ---- Step 1. Zero Crossing detection ---
; The single motor cycle begins in this place. The analog comparator which we have set after some commutation routine,
; is telling us that a Zero Crossing occured, by calling an interrupt here.
run_acomp_int:	in	JE, SREG
		lti	IL, IH, JL, JH, 0		; Read the time of the ZC
		sbis	ACSR, ACIS0			; Check what kind of edge we were waiting for
		rjmp	run_aco_1_0

		; We have spent a few cycles reading the timer.
		; Check the ACO state if it's correct, it may have been some spike.
run_aco_0_1:	sbis	ACSR, ACO
		rjmp	run_aco_reti			; --------------- optimize
		rjmp	run_aco_zc

run_aco_1_0:	sbis	ACSR, ACO
		rjmp	run_aco_zc
run_aco_reti:	out	SREG, JE
		reti

		; We've decided that proper ZC occured.
		; However, it may have interrupted some calculations, like speed, governor, power control on so on.
		; In that case we will have to save the context, to restore it later. Somewhat like in multithreading.
run_aco_zc:	bst	flags2, CALC_IN_PROGRESS	; Interrupted some calculations?
		brts	run_save_context		; It did
		pop	JE				; Nah, it didn't
		pop	JE				; Just get rid of the PC that the interrupt threw onto stack
		rjmp	run_process_zc

run_save_context:					; Save all registers used in these calculations
		led	1, 1
		sts	st_sreg, JE			; SREG	
		sts	st_xl, XL			; X
		sts	st_xh, XH
		sts	st_yl, YL			; Y
		sts	st_yh, YH
		sts	st_cl, CL			; C
		sts	st_ch, CH
		sts	st_al, AL			; D
		sts	st_ah, AH
		sts	st_bl, BL			; B
		;sts	st_bh, BH
		; The interrupt has put two bytes of interrupted Program Counter on the stack when it was called.
		; But we won't remove them on purpose, so later we can dig it out of the stack, when it's time
		; to restore the context.
		
run_process_zc:	cbi	ACSR, ACIE			; Disable comparator interrupts
		sbi	ACSR, ACI
		movw	BL, IL				; Move the ZC time
		mov	CL, JL
		sei					; Interrupts on


		; ---- Step 2. Zero Crossing time filtering ----
		; It's beneficial to filter the ZC times to make the motor work more quiet and efficient.
		; To do that we take arithmetic mean of detected ZC time and predicted ZC time.
		; The predicted ZC time is filtered ZC time from the previous commutation + last commutation length.
		lds	CH, previous_zc_time_l		; Load previous ZC time (DH:DL:CH)
		lds	DL, previous_zc_time_h
		lds	DH, previous_zc_time_e
		lds	XL, com_length_l		; load previous commutation length (YL:XH:XL)
		lds	XH, com_length_h
		lds	YL, com_length_e
			
		sub	BL, CH				; get time delta between current and previous ZC (CL:BH:BL)
		sbc	BH, DL
		sbc	CL, DH
	lsl	BL
	rol	BH
	rol	CL
	lsl	XL
	rol	XH
	rol	YL
		add	XL, BL				; so we've got previously saved commutation length, and time delta
		adc	XH, BH				;  between this and previous ZC. make arithmetic mean of it
		adc	YL, CL				;  (YL:XH:XL)

		sts	zc_timeout_l, XL
		sts	zc_timeout_h, XH
		sts	zc_timeout_e, YL
		ror	YL				; with carry, it can still be there from addition
		ror	XH
		ror	XL				; got it
	ror	YL				; with carry, it can still be there from addition
	ror	XH
	ror	XL	
		sts	com_length_l, XL		; store it as commutation length
		sts	com_length_h, XH
		sts	com_length_e, YL
		movw	AL, XL				;   copy (BL:AH:AL) <- (YL:XH:XL)
		mov	BL, YL
		add	XL, CH				; now add previous (filtered) ZC time to it
		adc	XH, DL
		adc	YL, DH				; got filtered time of this ZC (YL:XH:XL)
		sts	previous_zc_time_l, XL		; store it for the next time
		sts	previous_zc_time_h, XH
		sts	previous_zc_time_e, YL

		; --- Step 3. Timing ---
		; There is a thing called timing delay. Usually we don't switch commutation just when ZC is detected, but wait
		; some angle. In the natural case it is 30°, the middle point between two ZCs, but we accept other values of
		; timing here. Just to make it clear: Timing advance means how much the commutation switch is preceding the
		; natural 30° point. And timing delay is the angle between ZC and commutation change.
		; TIMING_ADVANCE = 30° - timing_delay.
		; Here we calculate the timing delay to the next commutation. The timing angle (0°-30°), is given here
		; as a value 0-128. We do a multiplication of commutation time with that value, and then divide by 256
		; which gives us 0 - 0.5 of commutation time as a result, which is 0°-30° of motor cycle.
		mov	CH, AL				; ------- optimize
		mov	DL, AH
		mov	DH, BL
		lds	CL, timing_angle		; multiply it by timing delay represented by value 0 - 128
		clr	BH				; --- 8 x 24 bit multiplication ---
		mul	CH, CL
		;mov	CH, AL				; byte1 (CH) done, won't need it though
		mov	BL, AH
		mul	DL, CL
		add	BL, AL				; byte2 (BL) done
		adc	BH, AH
		mul	DH, CL
		clr	CH
		add	BH, AL				; byte3 (BH) done
		adc	AH, CH				; byte4 (AH) done
							; skipping the youngest byte, we divide the result by 256
		add	XL, BL				; add timing delay (AH:BH:BL) to ZC time (YL:XH:XL)
		adc	XH, BH
		adc	YL, AH
		cli
		sbr	flags, 1<<TIMER_COMMUTATE	; make the timer call the next commutation function
		rcall	timer_set			; launch the timer

		; --- Step 4. Other calculations ---
		; The timer is on now, it will commutate when the time comes. We don't have to worry about it anymore.
		; In the free time, we will calculate some stuff here, like T->RPM coversion, governor mode etc.
		; Note that some calculations will be done no matter how quick the timer commutates and starts the comparator scan.
		; There is an additional delay between commutation and comparator activation.
		
		bst	flags2, CALC_IN_PROGRESS
		brtc	run_no_con_res
		lds	XL, st_sreg
		out	sreg, XL
		lds	XL, st_xl
		lds	XH, st_xh
		lds	YL, st_yl
		lds	YH, st_yh
		lds	CL, st_cl
		lds	CH, st_ch
		lds	AL, st_al
		lds	AH, st_ah
		lds	BL, st_bl
		led	1, 0
		ret					; jump to the PC remaining on the stack (see "run_save_context")

run_no_con_res:	sbr	flags2, 1<<CALC_IN_PROGRESS	; mark that we started calculations

		cpi	CL, COM_DELAY			; increase timing angle smoothly
		brsh	run_tim_ang_ok			; after switchng from start-up (no delay) to running mode.
		inc	CL				; we do that to prevent losing motor sync on that switch.
		sts	timing_angle, CL
run_tim_ang_ok:	
		
		; Read the power from I2C/RCP
		sbrc	flags2, SIGNAL_ERROR		; signal error?
		rcall	handle_signal_error		; yes, handle it
		rcall	update_power			; read power

		.if	GOVERNOR_MODE == 1	

		.else

		; Normal mode
		lds	DL, com_length_l		;  yes, load commutation length
		lds	DH, com_length_h
		lds	YL, com_length_e
		subi	DL, byte1(COM_T_RPM_MAX)
		sbci	DH, byte2(COM_T_RPM_MAX)
		sbci	YL, byte3(COM_T_RPM_MAX)
		brcc	run_power_ok			; no carry? rpm not too fast, just jump and set sdm factor
		lsr	XH				; too fast? divide power by 2
		ror	XL
		.endif

run_power_ok:	movw	sdm_factor_l, XL		; Set the outgoing power
				
		; Cycle time -> RPM conversion. Since f = 1/T, we have to make a division here.
		; Unfortunately the AVR processor doesn't have it. This is a 32 by 24 bit division, speed-optimized.
		; Shouldn't take more than 300 cycles. We don't have to hurry that much anyway, we can always stop the
		; calculation, handle the necessary stuff, and get back to it.
		; Inspired by this code: http://www.mikrocontroller.net/articles/AVR_Arithmetik#32_Bit_.2F_32_Bit
		bst	flags3, UPDATE_RPM
		;brtc	run_rpm_done
		lds	AL, com_length_l
		lds	AH, com_length_h
		lds	BL, com_length_e
		ldi	XL, byte1(TICKS_PER_SEC/6)
		ldi	XH, byte2(TICKS_PER_SEC/6)
		ldi	YL, byte3(TICKS_PER_SEC/6)
		ldi	YH, 0
		cbr	flags3, 1<<UPDATE_RPM

		clr	CH
run_udi10:	tst	BL
		breq	run_udi20
		ldi	CL, 16
run_udi11:	lsl	XL
		rol	XH
		rol	YL
		rol	YH
		rol	CH
		brcs	run_udi12
		cp	YL, AL
		cpc	YH, AH
		cpc	CH, BL
		brcs	run_udi13
run_udi12:	sub	YL, AL
		sbc	YH, AH
		sbc	CH, BL
		inc	XL
run_udi13:	dec	CL
		brne	run_udi11
		mov	AL, YL
		clr	YL
		mov	AH, YH
		clr	YH
		mov	BL, CH
		rjmp	run_div_done
 
run_udi20:	tst	AH
		breq	run_udi30
		ldi	CL, 24
run_udi21:	lsl	XL
		rol	XH
		rol	YL
		rol	YH
		rol	CH
		brcs	run_udi22
		cp	YH, AL
		cpc	CH, AH
		brcs	run_udi23
run_udi22:	sub	YH, AL
		sbc	CH, AH
		inc	XL
run_udi23:	dec	CL
		brne	run_udi21
		mov	AL, YH
		clr	YH
		mov	AH, CH
		rjmp	run_div_done
 
run_udi30:	ldi	CL, 32
run_udi31:	lsl	XL
		rol	XH
		rol	YL
		rol	YH
		rol	CH
		brcs	run_udi32
		cp	CH, AL
		brcs	run_udi33
run_udi32:	sub	CH, AL
		inc	XL
run_udi33:	dec	CL
		brne	run_udi31
		mov	AL, CH				;store remainder

run_div_done:	
		cli
		sts	rpm_l, XL
		sts	rpm_h, XH
		sei
		
run_wait_zc_timeout:
		cbr	flags2, 1<<CALC_IN_PROGRESS	; calculations done

		wait					; wait until the timer is "ready".
		
		; If it comes to this place, it means that no proper ZC occured in time.
		cbi	ACSR, ACIE			; Disable comparator interrupts
		rjmp	start_up			; Go to start-up procedure

fet_test:	;sbi	RL_PORT, RL_PIN
		;sbi	SL_PORT, SL_PIN
		;sbi	TL_PORT, TL_PIN
		;sbi	RH_PORT, RH_PIN
		;sbi	SH_PORT, SH_PIN
		;sbi	TH_PORT, TH_PIN
		cbi	RH_PORT, RH_PIN
		;cbi	SH_PORT, SH_PIN
		;cbi	TH_PORT, TH_PIN
		sbi	RL_PORT, RL_PIN
		;sbi	SL_PORT, SL_PIN
		;sbi	TL_PORT, TL_PIN
		ldi	XL, low(CLOCK_MHZ*1000000/7000/1000)	; wait 50 ms
		ldi	XH, high(CLOCK_MHZ*1000000/7000/1000)
		rcall	delay_1000cycles
fet_test2:	
		cbi	RL_PORT, RL_PIN
		;cbi	SL_PORT, SL_PIN
		;cbi	TL_PORT, TL_PIN
		sbi	RH_PORT, RH_PIN
		;sbi	SH_PORT, SH_PIN
		;sbi	TH_PORT, TH_PIN
		ldi	XL, low(CLOCK_MHZ*1000000/1000/1000)	; wait 50 ms
		ldi	XH, high(CLOCK_MHZ*1000000/1000/1000)
		rcall	delay_1000cycles
		rjmp	fet_test

; *-------------------------------------------------------------------------------------------------------------------*
; |                                                       Main                                                        |
; *-------------------------------------------------------------------------------------------------------------------*
; The program execution stars here
reset:
		in	IL, MCUCSR			; read reset reason
		sbi	RL_DDR, RL_PIN			; set fet controlling pins as out
		sbi	RH_DDR, RH_PIN
		sbi	SL_DDR, SL_PIN
		sbi	SH_DDR, SH_PIN
		sbi	TL_DDR, TL_PIN
		sbi	TH_DDR, TH_PIN

		clr	XH
		out	SREG, XH			; just in case, it should be 0 after reset
		out	MCUCSR, XH			; clear the reset reason register
		ldi	YL, low(RAMEND)			; set stack
		ldi	YH, high(RAMEND)
		out	SPH, YH
		out	SPL, YL

		rcall	motor_free_run			; close all fets

		.if	LED_DEBUG			; and LEDs...
		 .ifdef LED0_PORT
		  sbi	LED0_DDR, LED0_PIN
		  cbi	LED0_PORT, LED0_PIN
		 .endif
		 .ifdef LED1_PORT
		  sbi	LED1_DDR, LED1_PIN
		  cbi	LED1_PORT, LED1_PIN
		 .endif
		 .ifdef LED2_PORT
		  sbi	LED2_DDR, LED2_PIN
		  cbi	LED2_PORT, LED2_PIN
		 .endif
		 .ifdef LED3_PORT
		  sbi	LED3_DDR, LED3_PIN
		  cbi	LED3_PORT, LED3_PIN
		 .endif
		 .ifdef LED4_PORT
		  sbi	LED4_DDR, LED4_PIN
		  cbi	LED4_PORT, LED4_PIN
		 .endif
		.endif

reset_chk_wdr:	sbrs	IL, WDRF
		rjmp	reset_chk_bo
		;rcall	beep_wdr			; watchdog reset occured
		led	1, 1
		rjmp	reset_chk_done

reset_chk_bo:	sbrs	IL, BORF		
		rjmp	reset_chk_ext
		;rcall	beep_bo				; brown-out occured
		led	1, 1
		rjmp	reset_chk_done		
		
reset_chk_ext:	sbrs	IL, EXTRF
		rjmp	reset_chk_po
		;rcall	beep_extr
		led	1, 1
		led	0, 1
		rjmp	reset_chk_done

reset_chk_po:	sbrc	IL, PORF
		rjmp	reset_ok
		led	1, 1
		led	0, 1
		;rcall	beep_single			; uncrecognized reset reason, no flag set. wtf?
		rjmp	reset_chk_done

reset_ok:	;rcall	beep_po				; normal power on start occured

reset_chk_done:
		led	0, 1
		led	1, 1
		ldi	XL, low(CLOCK_MHZ*1000000/20/1000)	; wait 50 ms
		ldi	XH, high(CLOCK_MHZ*1000000/20/1000)
		;rcall	delay_1000cycles
		led	1, 0
		led	0, 0

		; clear all variables in RAM
		clr	YL
		ldi	XL, low(SRAM_START)
		ldi	XH, high(SRAM_START)
		ldi	YH, high(RAM_END)
reset_clr_lp:	st	X+, YL				; clear ram variables
		cpi	XL, low(RAM_END)
		cpc	XH, YH
		brlo	reset_clr_lp


		; enable timer interrupts
		ldi	XL, (1<<TOIE1) | (1<<OCIE1A) | (1<<TOIE2)
		out	TIMSK, XL

		; timer2 prescaler 1024
		ldi	XL, (1<<CS20) | (1<<CS21) |(1<<CS22)
		out	TCCR2, XL
		
		; timer1, main timer, no prescaler
		ldi	XL, 1<<CS10
		out	TCCR1B, XL
		
		; timer0, sigma-delta madulator, no prescaler
		ldi	XL, 1<<CS00			
		out	TCCR0, XL
		
		.if	CONTROL_METHOD == 1
			rcp_init
		.elif	CONTROL_METHOD == 2
			i2c_init
		.endif

		; comparator
		ldi	XL, 1<<ACME			; analog comparator multiplexer enable
		out	SFIOR, XL
		
		; watchdog
		ldi	XL, 1<<WDE			; enable watchdog, with no proescaler, ~16ms
		out	WDTCR, XL
		cbr	flags, 1<<TIMER_COMMUTATE
			
		ldi	XL, low(commutation_01)		; set the first commutation function address
		ldi	XH, high(commutation_01)
		sts	next_comm_call_addr_l, XL
		sts	next_comm_call_addr_h, XH
		
		sei					; enable interrupts

		rjmp	ml_res_sig_cnt
; Main loop
main_loop:	led	1, 0
		wdr
		bst	flags, TIMER_READY		; is the timer ready (not counting)?
		brtc	ml_timer_done			; no, it's working already
		run_timer 100*TICKS_PER_MS		; set it
		lds	XL, loop_led_cnt
		dec	XL
		brpl	ml_led_off
		led	0, 1
		ldi	XL, 20
ml_led_off:	sts	loop_led_cnt, XL
		cpi	XL, 20
		breq	ml_timer_done

		led	0, 0

ml_timer_done:	bst	flags2, SIGNAL_READY		; received some sort of signal?
		brtc	main_loop			; no

		bst	flags2, SIGNAL_ERROR		; signal error?
		brtc	ml_sig_fine			; nah, no error
		rcall	handle_signal_error		; yes, handle it
		rjmp	ml_res_sig_cnt
		
ml_sig_fine:	rcall	motor_free_run			; no error, unbrake the motor
		rcall	update_power			; read the power
		adiw	XL, 0				; test if non-zero
		breq	ml_sig_done			; zero, jump
		
		led	0, 1
		lds	XL, signal_cnt			; non-zero, load the signal counter
		dec	XL
		sts	signal_cnt, XL			
		brne	ml_sig_done
		led	0, 0
		rcall	start_up			; start up

ml_res_sig_cnt:	ldi	XL, 3				; reset down-counter of good signals the controller
		sts	signal_cnt, XL			; must receive to start. accidental start prevention.
		
ml_sig_done:	rjmp	main_loop