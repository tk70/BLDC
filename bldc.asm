;
;	Electronic Speed Controller for a BLDC motor.
;	
;	Software written for ATMega8 microcontroller
;	
;	by tk
;	v 0.2
;	3.09.2013

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
.equ POWER_RANGE = (RCP_MAX_THR - RCP_MIN_THR)*TICKS_PER_US
.equ POWER_RANGE_L = low(POWER_RANGE)
.equ POWER_RANGE_H = high(POWER_RANGE)
.equ GOV_SPEED_CONST = POWER_RANGE * TICKS_PER_SEC / GOV_MAX_RPS / 6 ; 6 - 6 commutations per cycle
;.equ STARTUP_PULL_POWER_L = low(STARTUP_PULL_POWER*TICKS_PER_US)
;.equ STARTUP_PULL_POWER_H = high(STARTUP_PULL_POWER*TICKS_PER_US)
.equ STARTUP_MAX_POWER_L = low(STARTUP_MAX_POWER*TICKS_PER_US)
.equ STARTUP_MAX_POWER_H = high(STARTUP_MAX_POWER*TICKS_PER_US)
.equ STARTUP_MIN_POWER_L = low(STARTUP_MIN_POWER*TICKS_PER_US)
.equ STARTUP_MIN_POWER_H = high(STARTUP_MIN_POWER*TICKS_PER_US)
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
.def JL = r22
.def JH = r23
.def JE = r24

; General flag registers
.def flags = r20					; pwm generator state flag
	.equ PWM_ACTIVE = 0				; is pwm wave being generated? no (0), yes (1)
	.equ TIMER_READY = 1				; ready flag A
	.equ TIMER_OVF = 2				; set once 16bit timer1 overflows
	.equ BRAKED = 3					; set if motor is braked
	.equ STARTUP = 4				; set if motor is in the starting mode
	.equ RUN = 5					; set if motor is in the running mode
	.equ TIMER_COMMUTATE = 6			; should timer call the next commutation?
	.equ AWAITED_ZC_TYPE = 7			; what type of ZC are we aiting for? 1: low to high or 0: high to low

.def flags2 = r21
	.equ BEEPER = 2
	.equ BEEP_CYCLE = 3
	.equ RCP_AWAITING_L = 4				; set if waiting for falling RC signal edge
	.equ RCP_CHECK_TIMEOUT = 5
	.equ SIGNAL_READY = 6
	.equ SIGNAL_ERROR = 7
	

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
	avg_com_length_l:	.byte	1		; averaged commutation length
	avg_com_length_h:	.byte	1
	avg_com_length_e:	.byte	1
	zc_scan_time_l:		.byte	1		; time of comparator scan activation
	zc_scan_time_h:		.byte	1
	zc_scan_time_e:		.byte	1
	rcp_power_factor_l:	.byte	1		; length of rc pulse
	rcp_power_factor_h:	.byte	1
	next_comm_call_addr_l:	.byte	1		; function call pointer to the next commutation
	next_comm_call_addr_h:	.byte	1
	beep_time_l:		.byte	1
	beep_time_h:		.byte	1
	beep_cycle_time_l:	.byte	1
	beep_cycle_time_h:	.byte	1
	gov_error:		.byte	1
	gov_throttle_l:		.byte	1		; governor mode current throttle
	gov_throttle_h:		.byte	1
	tcnt1e:			.byte	1		; extended timer byte
	ocr1ae:			.byte	1		; timer compare output extended byte
	ram_end:		.byte	1		; marker of ram end, for clearing

.cseg
.org 0

; *------------------*
; |  Interrupt table |
; *------------------*

		rcall	reset				; RESET
		.if		RCP_CHANNEL == 0
		rjmp	rcp_int				; INT0
		rjmp	inv_int				; INT1
		.else
		rjmp	inv_int				; INT0
		rjmp	rcp_int				; INT1
		.endif
		rjmp	inv_int				; TIMER2 COMP
		rjmp	t2_ovf				; TIMER2 OVF
		rjmp	inv_int				; TIMER1 CAPT
		rjmp	timer_int_ocr1a			; TIMER1 COMPA
		rjmp	beep_int_ocr1b			; TIMER1 COMPB
		rjmp	timer_int_t1ovf			; TIMER1 OVF
		rjmp	sdm_sample			; TIMER0 OVF
		nop					; SPI, STC Serial Transfer Complete
		nop					; USART, RXC
		nop					; USART, UDRE
		nop					; USART, TXC
		nop					; ADC
		nop					; EE_RDY
		nop					; ANA_COMP
		nop					; TWI
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
		led_dbg	2, 1
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

reset_vars:	clr	rcp_fail_count
		clr	rcp_time_l
		clr	rcp_time_h
		clr	sdm_factor_l
		clr	sdm_factor_h
		clr	sdm_err_l
		clr	sdm_err_h
		clr	DL
		clr	DH
		ldi	XL, low(SRAM_START)
		ldi	XH, high(SRAM_START)
		ldi	YL, low(ram_end)
		ldi	YH, high(ram_end)
rv_clear_loop:	st	X+, DL			; clear ram variables
		cp	XL, YL
		cpc	XH, YH
		brlo	rv_clear_loop
		clr	XL
		clr	XH
		clr	YL
		clr	YH
		ret

; *------------------*
; |       Timer      |
; *------------------*
; Here we implement the 24 bit timer for our ESC, with one timer tick being a one clock cycle. 
; Yes, it is an overkill, but there are no better options. If we used 16 bit timer1 without extended byte,
; with prescaler 8, the whole clock cycle would be only ~30ms, which is pretty much not enough for our purposes.
; And the 64 prescaler gives insufficient resolution on the other hand. So if the extended byte has to be used anyway,
; why not make it prescaler free. Entire clock cycle will take above one second to complete, which is more than enough.
; Speaking of 16MHz crystal here.
; The extended timer byte called ocr1ae here, is stored in RAM. It's not like the first two bytes though, it's more like
; ocr1a multiplier

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
		sbrs	flags, TIMER_COMMUTATE		; the timer has to call the commutation?
		rjmp	ts_ret				; no, it doesn't, return
		lds	ZL, next_comm_call_addr_l	; load the function pointer to be called
		lds	ZH, next_comm_call_addr_h
		icall					; call the function
		wdr
		cbr	flags, 1<<TIMER_COMMUTATE	; clear the function call flag
ts_ret:		sei					; no. enable interrupts.
		ret


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
		sei
		ret


; Timer1 compare interrupt A
; Activates when timer reaches set value
; If specified, may call a commutation function
timer_int_ocr1a:
		in	IL, SREG
		lds	IH, ocr1ae
		dec	IH
		sts	ocr1ae, IH
		brpl	ret_i
		sbr	flags, 1<<TIMER_READY

		; The timer may have been commanded to call a function
		sbrs	flags, TIMER_COMMUTATE		; the timer has to call the commutation?
		rjmp	ret_i				; no, it doesn't, return
		lds	ZL, next_comm_call_addr_l	; load the function pointer to be called
		lds	ZH, next_comm_call_addr_h
		icall					; call the function
		wdr
		cbr	flags, 1<<TIMER_COMMUTATE	; clear the function call flag
ret_i:		out	SREG, IL
		reti


; Timer1 overflow interrupt
timer_int_t1ovf:
		in	IL, SREG
		lds	JL, tcnt1e
		inc	JL
		sbr	flags, 1<<TIMER_OVF
		sts	tcnt1e, JL
		dec	rcp_timeout
		breq	rcp_invalid_signal
		out	SREG, IL
		reti


; Timer2 overflow interrupt
t2_ovf:		in	IL, SREG
		led_dbg	2, 0
		out	SREG, IL
		reti


; wait X * 8 cycles, args: XH:XL
delay_8cycles:	sbiw	XL, 1				; 2
		push	XL
		pop	XL				; 6
		brpl	delay_8cycles			; 8
		ret

; *------------------*
; |    Power input   |
; *------------------*

; External interrupt, triggers on any logic change
rcp_int:	in	IL, SREG
		sbic	RCP_PIN_REG, RCP_PIN		; is it low or high state?
		rjmp	rcp_rcp_high_state
		led_dbg	4, 0
		sbrs	flags2, RCP_AWAITING_L		; check if we are waiting for a falling edge
		rjmp	ret_i				; no, return
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
		sts	rcp_power_factor_l, JL
		sts	rcp_power_factor_h, JH
		out	SREG, IL
		reti
rcp_rcp_fail:	inc	rcp_fail_count			; if imuplses are too short/long
rcp_rcp_fail2:	ldi	JL, 3
		cp	rcp_fail_count, JL
		brsh	rcp_invalid_signal
		out	SREG, IL
		reti
rcp_invalid_signal:
		clr	rcp_fail_count
		sbr	flags2, (1<<SIGNAL_READY) | (1<<SIGNAL_ERROR)
		cbr	flags2, 1<<RCP_AWAITING_L
rcp_no_power:	clr	JL
		sts	rcp_power_factor_l, JL
		sts	rcp_power_factor_h, JL
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

; Parse the power value incoming from RCP/I2C
; In case of RCP, limit the pulse length to the power range
; Returns power value in XH:XL
update_power:	cbr	flags2, 1<<SIGNAL_READY
		cli					; cli, we don't want the rcp_int interrupt between these two lines
		lds	XL, rcp_power_factor_l		; read length of the pulse
		lds	XH, rcp_power_factor_h
		sei
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
		ret

handle_signal_error:
		sbrc	flags, PWM_ACTIVE
		rcall	sdm_off
		sbrs	flags, BRAKED
		rjmp	motor_brake

; *------------------*
; |   Power stage    |
; *------------------*

; mosfet operations
.if PWM_SIDE == 1					; low side does the PWM
	; phase R
	.macro	RX_on
		sbi	RL_PORT, RL_PIN
	.endmacro
	.macro	RX_off
		cbi	RL_PORT, RL_PIN
	.endmacro
	.macro	RY_on
		sbi	RH_PORT, RH_PIN
	.endmacro
	.macro	RY_off
		cbi	RH_PORT, RH_PIN
	.endmacro

	; phase S
	.macro	SX_on
		sbi	SL_PORT, SL_PIN
	.endmacro
	.macro	SX_off
		cbi	SL_PORT, SL_PIN
	.endmacro
	.macro	SY_on
		sbi	SH_PORT, SH_PIN
	.endmacro
	.macro	SY_off
		cbi	SH_PORT, SH_PIN
	.endmacro

	; phase T
	.macro	TX_on
		sbi	TL_PORT, TL_PIN
	.endmacro
	.macro	TX_off
		cbi	TL_PORT, TL_PIN
	.endmacro
	.macro	TY_on
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

; cuts the power off, all power FETs closed
motor_free_run:	rcall	sdm_off				; sdm generator off, so it doesn't open the FETs
		RY_off
		SY_off		
		TY_off
		RX_off
		SX_off
		TX_off
		cbr	flags, 1<<BRAKED
		ret

; cuts the power off but leaves one side of the bridge open which creates a short circuit between all the phases,
; making the motor a generator with infinite load.
motor_brake:	rcall	sdm_off
		RY_off
		SY_off
		TY_off
		RX_on
		SX_on
		TX_on
		sbr	flags, 1<<BRAKED
		ret

; Commutation subroutines, they must be called from interrupts, or with disabled interrupts, between cli and sei
; R->S, T undriven. RH open, SL pwm
commutation_50:	ldi	JL, T_COMP_CHANNEL		; switch comparator channel
		out	ADMUX, JL
		TY_off					; close the T phase lower mosfet
		RY_on					; open the R phase higher mosfet
;		brtc	c0_low				; should it be low sdm state now?
;		.if ACTIVE_FREEWHEELING			; the high S fet may be on now, because of complementary pwm.
;			SY_off				; turn it off
;			fet_delay
;		.endif
;		SX_on					; no, open the S phase lower mosfet (pwm)
c0_low:		ldi		ZL, low(sdm_j0)		; set jump address for sdm generator
		ldi	ZH, high(sdm_j0)
		ldi	JL, low(commutation_01)
		sts	next_comm_call_addr_l, JL
		ldi	JL, high(commutation_01)
		sts	next_comm_call_addr_h, JL
		led_dbg	1, 0
		cbr	flags, 1<<AWAITED_ZC_TYPE	; next awaited ZC will be high to low
		ret

; R->T, S undriven. RH open, TL pwm
commutation_01:	ldi	JL, S_COMP_CHANNEL
		out	ADMUX, JL
		SX_off
		;RY_on
		.if ACTIVE_FREEWHEELING
		  SY_off
		.endif
		brtc	c1_low
		TX_on
c1_low:		ldi	ZL, low(sdm_j2)
		ldi	ZH, high(sdm_j2)
		ldi	JL, low(commutation_12)
		sts	next_comm_call_addr_l, JL
		ldi	JL, high(commutation_12)
		sts	next_comm_call_addr_h, JL
		led_dbg	1, 0
		sbr	flags, 1<<AWAITED_ZC_TYPE	; next awaited ZC will be low to high
		ret

; S->T, R undriven. SH open, TL pwm
commutation_12:	ldi	JL, R_COMP_CHANNEL
		out	ADMUX, JL
		RY_off
		SY_on
;		brtc	c2_low
;		.if ACTIVE_FREEWHEELING
;			TY_off
;			fet_delay
;		.endif
;		TX_on
c2_low:		ldi		ZL, low(sdm_j2)
		ldi	ZH, high(sdm_j2)
		ldi	JL, low(commutation_23)
		sts	next_comm_call_addr_l, JL
		ldi	JL, high(commutation_23)
		sts	next_comm_call_addr_h, JL
		led_dbg	1, 0
		cbr	flags, 1<<AWAITED_ZC_TYPE
		ret

; S->R, T undriven. SH open, RL pwm
commutation_23:	ldi	JL, T_COMP_CHANNEL
		out	ADMUX, JL
		TX_off
;		SY_on
		.if ACTIVE_FREEWHEELING
		  TY_off
		.endif
		brtc	c3_low
		RX_on
c3_low:		ldi	ZL, low(sdm_j4)
		ldi	ZH, high(sdm_j4)
		ldi	JL, low(commutation_34)
		sts	next_comm_call_addr_l, JL
		ldi	JL, high(commutation_34)
		sts	next_comm_call_addr_h, JL
		led_dbg	1, 0
		led_dbg	6, 1
		sbr	flags, 1<<AWAITED_ZC_TYPE
		ret

; T->R, S undriven. TH open, RL pwm
commutation_34:	ldi	JL, S_COMP_CHANNEL
		out	ADMUX, JL
		SY_off
		TY_on
		led_dbg	1, 0
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
		cbr	flags, 1<<AWAITED_ZC_TYPE
		ret

; T->S, R undriven. TH open, SL pwm
commutation_45:	ldi	JL, R_COMP_CHANNEL
		out	ADMUX, JL
		RX_off
		;TY_on
		led_dbg	1, 0
		led_dbg	6, 0
		.if ACTIVE_FREEWHEELING
		  RY_off
		.endif
		brtc	c5_low
		SX_on
c5_low:		ldi	ZL, low(sdm_j0)
		ldi	ZH, high(sdm_j0)
		ldi	JL, low(commutation_50)
		sts	next_comm_call_addr_l, JL
		ldi	JL, high(commutation_50)
		sts	next_comm_call_addr_h, JL
		sbr	flags, 1<<AWAITED_ZC_TYPE
		ret

; call the next commutation
commutate:	cli
		lds	ZL, next_comm_call_addr_l
		lds	ZH, next_comm_call_addr_h
		icall
		wdr
		sei
		ret


; *--------------------------*
; |          Beeper          |
; *--------------------------*

; Enable beep timer
; args: YH:YL cycle time (1s/f). unmodified.
; clobbered: XL, XH
beep_on:	sbrs	flags, PWM_ACTIVE
		rcall	motor_free_run			; all FETs off, SDM off
		in	XL, TIMSK
		sbr	XL, 1<<OCIE1B
		out	TIMSK, XL
		sbr	flags2, 1<<BEEPER
		sts	beep_cycle_time_l, YL
		sts	beep_cycle_time_h, YH
bp_lp:		sbrs	flags2, BEEP_CYCLE		; wait for timer to give a signal for single cycle
		rjmp	bp_ct				; no signal, do down then to check timer
		cbr	flags2, 1<<BEEP_CYCLE
		ldi	XL, low(BEEP_ON_TIME*TICKS_PER_US*TIMER_PRESCALER/8)
		ldi	XH, high(BEEP_ON_TIME*TICKS_PER_US*TIMER_PRESCALER/8)
		RX_on
		TY_on
		rcall	delay_8cycles
		RX_off
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
		cbr	flags2, 1<<BEEPER
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
beep_int_ocr1b:	in	JE, SREG
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

; Watchdog reset occured. Womp. womp. womp.
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
	
; *--------------------------*
; |   Sigma-delta modulator  |
; *--------------------------*
; Implementation of Sigma-delta modulator

; Activate the SDM generator
sdm_on:		in	XH, TIMSK
		sbr	XH, 1<<TOIE0
		sbr	flags, 1<<PWM_ACTIVE
		cbr	flags, 1<<BRAKED
		clr	sdm_err_l
		clr	sdm_err_h
		out	TIMSK, XH			; enable the interrupt
		ret

; Disable SDM
sdm_off:	in	XH, TIMSK
		cbr	XH, 1<<TOIE0
		cbr	flags, 1<<PWM_ACTIVE
		out	TIMSK, XH
		led_dbg	5, 0
		ret

; Timer0 overflow interrupt
; Delta-sigma modulator
; This interrupt is called every 256 clock cycles, which gives 62.5KHz sampled delta sigma modulation
; on 16 MHz crystal. Thanks to high optimization of the code, each interrupt takes about 22 cycles,
; which is only ~9% of total cpu time. The jump address for "ijmp" is kept in Z register which is set
; in commutation functions. The SDM output state (1/0) is stored as T flag in SREG
sdm_sample:
		in	IL, SREG
		movw	JL, sdm_factor_l
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
		out	SREG, IL
		wdr
		clt
		reti
sdm_h0:	.if ACTIVE_FREEWHEELING				; set the high pwm state
		brts	sdm_ret				; if it's high already, just return
		SY_off
		rcall	fet_delay
	.endif
		SX_on
		led_dbg	5, 1
		out	SREG, IL
		set
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
		out	SREG, IL
		wdr
		clt
		reti
sdm_h2:	.if ACTIVE_FREEWHEELING
		brts	sdm_ret
		TY_off
		rcall	fet_delay
	.endif
		TX_on
		led_dbg	5, 1
		out	SREG, IL
		set
		reti
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
		out	SREG, IL
		wdr
		clt
		reti
sdm_h4:	.if ACTIVE_FREEWHEELING
		brts	sdm_ret
		RY_off
		rcall	fet_delay
	.endif
		RX_on
		led_dbg	5, 1
		out	SREG, IL
		set
		reti

; * --------------*
; |    Startup    |
; *---------------*

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
		brts	start_read_comparator_state	; high pwm state? wait for low.
		clr	XH				; should be low pwm state now
srcs_loop:						; now wait for high state
		sbrc	flags, TIMER_READY		; timeout?
		ret					; yes
		mov	XL, XH				; save previous read comparator state
		in	XH, ACSR			; read comparator register
		andi	XH, (1<<ACO)			; extract comparator state bit
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

idle_loop_first:
		
idle_loop:
		rjmp	idle_loop

; start up procedure
start_up_again:
		lds	XL, startup_attempts
		inc	XL
		sts	startup_attempts, XL
start_up:	cbr	flags, (1<<STARTUP) | (1<<RUN)
		; check reset reason

		sbrs	flags, BRAKED
		lds	XL, startup_attempts
		push	XL
		rcall	motor_free_run
		rcall	reset_vars
		pop	XL
		sts	startup_attempts, XL	
		led_dbg	1, 0
		led_dbg	7, 0
su_wait_for_rc_signal:					; wait for i2c/rc signal
		wdr
		sbrc	flags2, SIGNAL_ERROR
		rcall	handle_signal_error
		sbrc	flags2, SIGNAL_ERROR
		rjmp	su_wait_for_rc_signal
		rcall	motor_free_run
		rcall	update_power
		adiw	XL, 0
		breq	su_wait_for_rc_signal		; received signal to start?
		led_dbg	1, 1
		rcall	start_reset_counters
		sbr	flags, (1<<STARTUP)
		lds	XL, startup_attempts
		cpi	XL, 2
		brsh	su_com				; if it's the first startup
		cli					; we must call the first commutation subroutine manually
		wdr
		rcall	commutation_01			; so the call address for the next one will set
		sei
		rcall	sdm_on				; activate sigma-delta generator
su_com:		rcall	commutate
		clr	BL
		clr	BH
		clr	CL
		;delay_wdr	20*TICKS_PER_MS

start_loop:	sbrc	flags2, SIGNAL_ERROR		; signal error?
		rcall	handle_signal_error		; yes, handle it
		rcall	update_power			; read power
		adiw	XL, 0				; if it is 0 or there's an error, go to start
		breq	start_up

		sts	previous_zc_time_l, BL		; store the time of the previous ZC. these variables need to be
		sts	previous_zc_time_h, BH		; set properly in case it switched to running mode right after
		sts	previous_zc_time_e, CL

		ldi	YH, STARTUP_MAX_POWER_H
		cpi	XL, STARTUP_MAX_POWER_L		; compare the power value with max allowed star-tup power
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
su_pow_ok:
		wait					; wait until ZC scan is allowed
		rcall	start_set_commutation_timeout

		sbrs	flags, AWAITED_ZC_TYPE		; which type of ZC are we waiting for?
		rcall	start_wait_zc_low
		sbrc	flags, AWAITED_ZC_TYPE
		rcall	start_wait_zc_high

		lti	BL, BH, CL, CH, 1		; read timer

		rcall	commutate

		ldi	XL, byte1(40*TICKS_PER_US)	; so the next zc scan won't happen in 40 us
		ldi	XH, byte2(40*TICKS_PER_US)
		ldi	YL, byte3(40*TICKS_PER_US)
		add	XL, BL
		adc	XH, BH
		adc	YL, CL
		rcall	timer_set ; ------------------------------------------------------------------------------------------------------------------- rel

		lds	AL, startup_counter
		dec	AL
		sts	startup_counter, AL
		brne	start_loop
		clr	XL
		sts	startup_attempts, XL
		rjmp	running_mode_switch

; * --------------*
; |      Run      |
; *---------------*

; wait for high state on the comparator (back emf goes below 0)
wait_for_bemf_low:
		ldi	CL, 3
cwl_lp:		sbrc	flags, TIMER_READY		; timeout?
		ret
		sbis	ACSR, ACO			; high?
		rjmp	wait_for_bemf_low		; no
		dec	CL				; yes
		breq	cw_ret
		rjmp	cwl_lp				; loop, make sure, could have been a false detection

; wait for low state on the comparator (back emf goes above 0)
wait_for_bemf_high:
		ldi	CL, 3
cwh_lp:		sbrc	flags, TIMER_READY
		ret
		sbic	ACSR, ACO			; low?
		rjmp	wait_for_bemf_high
		dec	CL
		breq	cw_ret
		rjmp	cwh_lp				; loop
cw_ret:		ret

running_mode_switch:
		lti	XL, XH, YL, BL, 1
		lds	YH, previous_zc_time_l
		lds	DL, previous_zc_time_h
		lds	DH, previous_zc_time_e
		sts	previous_zc_time_l, XL
		sts	previous_zc_time_h, XH
		sts	previous_zc_time_e, YL		; all these vars must be set properly in case
		movw	AL, XL				; we want to switch to running mode
		mov	BL, YL
		sub	XL, YH
		sbc	XH, DL
		sbc	YL, DH
		sts	com_length_l, XL
		sts	com_length_h, XH
		sts	com_length_e, YL
		led_dbg	1, 0
		clr	XL
		sts	timing_angle, XL
		sts	gov_error, XL
		sts	avg_com_length_l, XL
		sts	avg_com_length_h, XL
		sts	avg_com_length_e, XL
		cbr	flags, 1<<STARTUP
		sbr	flags, (1<<TIMER_READY) | (1<<RUN)
		lti	XL, XH, YL, YH, 1
		sts	zc_scan_time_l, XL
		sts	zc_scan_time_h, XH
		sts	zc_scan_time_e, YL
		led_dbg	7, 1
		rcall	update_power
		sts	gov_throttle_l, XL
		sts	gov_throttle_h, XH
; Motor running loop
running_loop:
		; --- ZERO CROSS DETECTION ---
		; Before we start scanning for a ZC (zero cross) detection, we must set some timeout for that operation
		lti	YH, DL, DH, YL, 1
		lds	XL, com_length_l		; read commutation length
		lds	XH, com_length_h
		lds	YL, com_length_e
		lsl	XL				; multiply by 2
		rol	XH
		rol	YL
		lsl	XL				; multiply by 2
		rol	XH
		rol	YL
		add	XL, YH				; add two commutation lengths to the current time
		adc	XH, DL
		adc	YL, DH
		rcall	timer_set			; and set it as ZC detection timeout

		; We will need these variables after ZC detection, let's better read them now,
		; so we have more time for calculations later
		lds	CH, previous_zc_time_l		; load previous ZC time (DH:DL:CH)
		lds	DL, previous_zc_time_h
		lds	DH, previous_zc_time_e
		.if	ZC_FILTERING
		lds	XL, com_length_l		; load previous commutation length (YL:XH:XL)
		lds	XH, com_length_h
		lds	YL, com_length_e
		.endif
		
		; Here we wait for the ZC detection, what kind of detection it will be, is already set in
		; the flag register by the previous commutation
		; CL register clobbered
		sbrc	flags, AWAITED_ZC_TYPE		; which type of ZC are we waiting for? low-high? high-low?
		rcall	wait_for_bemf_low
		rcall	wait_for_bemf_high
		sbrs	flags, AWAITED_ZC_TYPE
		rcall	wait_for_bemf_low
rl_zcd:		sbrc	flags, TIMER_READY		; Ok, ZC or timout occured
		rjmp	start_up			; timeout, go to start procedure
							; ZC otherwise
		led_dbg	1, 1
		
		; --- ZC FILTERING ---
		; ZC detection times are not even, it's beneficial to filter them and reduce the error.
		; Arithmetic mean of detected ZC time and predicted ZC time is used.
	.if	ZC_FILTERING
		lti	BL, BH, CL, AL, 1		; get time of detected ZC		
		sub	BL, CH				; get time delta between current and previous ZC (CL:BH:BL)
		sbc	BH, DL
		sbc	CL, DH
		add	XL, BL				; so we've got previously saved commutation length, and time delta
		adc	XH, BH				;  between this and previous ZC. make arithmetic mean of it
		adc	YL, CL				;  (YL:XH:XL)
		ror	YL				; with carry, it can still be there from addition
		ror	XH
		ror	XL				; got it
/*	.if GOVERNOR_MODE
		push	YL
		push	XH
		push	XL
	.endif*/
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
	.else						; if ZC filtering disabled
		lti	AL, AH, BL, BH, 1		; get time of detected ZC
		movw	XL, AL				; copy
		mov	YL, BL
		sts	previous_zc_time_l, XL		; store time of this ZC
		sts	previous_zc_time_h, XH
		sts	previous_zc_time_e, YL
		sub	AL, CH				; calculate time delta between this and previous ZC, to get length
		sbc	AH, DL				;  of the commutation. we need it for timing below
		sbc	BL, DH	
	.endif

		; --- TIMING ---
		; There is a thing called timing delay. Usually we don't switch commutation just when ZC is detected, but wait
		; some angle. In the natural case it is 30°, the middle point between two ZCs, but we accept other values of
		; timing here. Just to make it clear: Timing advance means how much the commutation switch is preceding the
		; natural 30° point. And timing delay is the angle between ZC and commutation change.
		; TIMING_ADVANCE = 30° - timing_delay.
		; Here we calculate the timing delay to the next commutation. The timing angle (0°-30°), is given here
		; as a value 0-128. We do a multiplication of commutation time with that value, and then divide by 256
		; which gives us 0 - 0.5 of commutation time as a result, which is 0°-30° of motor cycle.
		lds	CH, avg_com_length_l		; load average commutation length
		lds	DL, avg_com_length_h
		lds	DH, avg_com_length_e
	;	push	DH
	;	push	DL
	;	push	CH
		add	CH, AL				; arithmetic mean with last commutation length
		adc	DL, AH
		adc	DH, BL
		ror	DH
		ror	DL
		ror	CH				; got filtered commutation length
		sts	avg_com_length_l, CH		; store it
		sts	avg_com_length_h, DL
		sts	avg_com_length_e, DH
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
		; The timer is on now, it will commutate when the time comes. We don't have to worry about it anymore.

		cpi	CL, COM_DELAY			; increase timing angle smoothly, to prevent losing motor sync
		brsh	rl_tim_ang_ok			; after switchng from start-up (no delay) to running mode.
		inc	CL				; we do that to prevent losing motor sync on that switch.
		sts	timing_angle, CL
rl_tim_ang_ok:		
	
ct_skt:	;sei
		
		; Read the power from I2C/RCP
		sbrc	flags2, SIGNAL_ERROR		; signal error?
		rcall	handle_signal_error		; yes, handle it
		rcall	update_power			; read power
		
/*	.if	GOVERNOR_MODE == 1	
WIP				

		; Automatic throttle control in order to achieve specified RPM
		; ClockTicks - number of clock ticks per second
		; T - one cycle (6 commutations) time in ClockTicks, RPS - rounds per second requested
		; Power - power value from I2C/RC Pulse
		;
		; T == ClockTicks / RPS, RPS == Power/PowerRange * RPSmax ==>
		; ==> T == PowerRange/Power/RPSmax ==>
		; ==> T * Power == PowerRange / RPSMax == 6*GOV_SPEED_CONST (6 commutations)
		; The controller will be trying to achieve equality of this equation by modifying T on the left side
		; (commutation time) by adjustment of throttle and thus motor speed. The control method is somewhat
		; similar to PID regulation.
		lds	DL, avg_com_length_l		; load commutation length
		lds	DH, avg_com_length_h
		lds	YL, avg_com_length_e			
		pop	AL
		pop	AH
		pop	BL
		movw	CL, DL
		mov	BH, YL
		sub	CL, AL
		sbc	CH, AH
		sbc	BH, BL
		push	BH
		push	CH
		push	CL		
							; --- 24 x 16 bit multiplication --- (commutation time * Power)
		clr	CL				; clear E and X bytes of the 32 bit result (X:E:H:L -> CH:CL:BH:BL)
		clr	CH				; L and H will be overwritten by movw anyway
		clr	YH				; help register
		mul	XL, DL				; PowerL x TL
		movw	BL, AL				; ResultL done
		mul	XL, DH				; PowerL x TH
		add	BH, AL						
		adc	CL, AH
		mul	XL, YL				; PowerL x TE
		clr	XL				; we don't need PowerL anymore, will use it as 5th result byte
		add	CL, AL
		adc	CH, AH
		adc	XL, YH
		mul	XH, DL				; PowerH x TL
		add	BH, AL				; ResultH done
		adc	CL, AH
		adc	CH, YH
		adc	XL, YH
		mul	XH, DH				; PowerH x TH
		add	CL, AL
		adc	CH, AH
		adc	XL, YH
		mul	XH, YL				; PowerH x TE
		add	CH, AL
		adc	XL, AH				; got the left side of the equation
		movw	YL, BL				; subi cannot be done on that reg, use Y
		subi	YL, byte1(GOV_SPEED_CONST)
		sbci	YH, byte2(GOV_SPEED_CONST)	; now compare both sides of equation
		sbci	CL, byte3(GOV_SPEED_CONST)
		sbci	CH, byte4(GOV_SPEED_CONST)
		
		pop	AL
		pop	AH
		pop	BL

		ldi	DL, GOV_I_FACTOR
		lds	XL, gov_throttle_l
		lds	XH, gov_throttle_h
		brcc	rl_gov_throt_inc		; carry clear? yes: left side greater, jump
rl_gov_throt_dec:					; left side lower => T must increase => throttle must drop
		;sub	YH, AL
		;sbc	CL, AH
		;sbc	CH, BL

		com	YH
		com	CL
		com	CH
		lsr	YH
		ror	CH
		ror	CL
		lsr	YH
		ror	CH
		ror	CL

		mul	YH, DL				; --- 8 x 24 bit multiplication ---
		clr	BH
		mov	BL, AH				; we only use the high byte of multiplication result
		mul	CL, DL
		add	BL, AL				; byte2 (BL) done
		adc	BH, AH
		mul	CH, DL
		add	BH, AL				; byte3 (BH) done
		adc	AH, YH				; byte4 (AH) done
		sub	XL, BH
		sbc	XH, AH
		brcc	rl_gov_ti_01
		clr	XL
		clr	XH
		rjmp	rl_gov_ti_01
rl_gov_throt_inc:					; left side higher => T must drop => throttle must increase
		;sub	YH, AL
		;sbc	CL, AH
		;sbc	CH, BL
		lsr	YH
		ror	CH
		ror	CL
		lsr	YH
		ror	CH
		ror	CL
		mul	YH, DL				; --- 8 x 24 bit multiplication ---
		clr	BH
		mov	BL, AH				; we only use the high byte of multiplication result
		mul	CL, DL
		add	BL, AL				; byte2 (BL) done
		adc	BH, AH
		mul	CH, DL
		add	BH, AL				; byte3 (BH) done
		adc	AH, YH				; byte4 (AH) done
		add	XL, BH
		adc	XH, AH
		ldi	CL, POWER_RANGE_H
		cpi	XL, POWER_RANGE_L
		cpc	XH, CL
		brlo	rl_gov_ti_01
		ldi	XL, POWER_RANGE_L
		ldi	XH, POWER_RANGE_H
rl_gov_ti_01:	sts	gov_throttle_l, XL
		sts	gov_throttle_h, XH*/
	;.else

		; Normal mode
		lds	DL, com_length_l		;  yes, load commutation length
		lds	DH, com_length_h
		lds	YL, com_length_e
		subi	DL, byte1(COM_T_RPM_MAX)
		sbci	DH, byte2(COM_T_RPM_MAX)
		sbci	YL, byte3(COM_T_RPM_MAX)
		brcc	rl_p_ok				; no carry? rpm not too fast, just jump and set sdm factor
		lsr	XH				; too fast? divide power by 2
		ror	XL
	;.endif

rl_p_ok:	movw	sdm_factor_l, XL

		; Synchronize with the timer. If the timer commutated already, it will just loop to the next ZC scan,
		; otherwise it will wait, until the timer commutates.
		wait					; wait until the timer is "ready".
		rjmp	running_loop			; loop

; *------------------*
; |      Reset       |
; *------------------*
reset:		in	XL, MCUCSR			; read reset reason
		push	XL
		clr	XL
		out	MCUCSR, XL
		sbi	RL_DDR, RL_PIN			; set fet controlling pins as out
		sbi	RH_DDR, RH_PIN
		sbi	SL_DDR, SL_PIN
		sbi	SH_DDR, SH_PIN
		sbi	TL_DDR, TL_PIN
		sbi	TH_DDR, TH_PIN
		rcall	motor_free_run			; close all fets , just in case

		.if	LED_DEBUG				; and LEDs...
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
		 .ifdef LED5_PORT
		  sbi	LED5_DDR, LED5_PIN		; my test board actually has so many leds.
		  cbi	LED5_PORT, LED5_PIN
		 .endif
		 .ifdef LED6_PORT
		  sbi	LED6_DDR, LED6_PIN
		  cbi	LED6_PORT, LED6_PIN
		 .endif
		 .ifdef LED7_PORT
		  sbi	LED7_DDR, LED7_PIN
		  cbi	LED7_PORT, LED7_PIN
		 .endif
		.endif

		; init stuff
		rcall	reset_vars

		; enable timer interrupts of timers
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

		; RC pulse input
		cbi	RCP_DIR_REG, RCP_PIN		; set rc pulse pin as input
		ldi	XL, MCUCR_VAL			; set external int to trigger on any logic change
		out	MCUCR, XL
		ldi	XL, GICR_VAL			; enable external interrupt
		out	GICR, XL

		; comparator
		ldi	XL, 1<<ACME			; analog comparator multiplexer enable
		out	SFIOR, XL

		sei					; enable interrupts
	;	delay	50*TICKS_PER_MS
		
		; watchdog
		ldi	XL, 1<<WDE			; enable watchdog, with no proescaler, ~16ms
		out	WDTCR, XL
		wdr					; reset just in case
	
		;rjmp	test

		rjmp	start_up

test:		pop	XL
reset_chk_wdr:	sbrs	XL, WDRF
		rjmp	reset_chk_bo
		rcall	beep_wdr			; watchdog reset occured
		rjmp	reset_chk_done

reset_chk_bo:	sbrs	XL, BORF		
		rjmp	reset_chk_ext
		rcall	beep_bo				; brown-out occured
		rjmp	reset_chk_done		
		
reset_chk_ext:	sbrs	XL, EXTRF
		rjmp	reset_chk_po
		rcall	beep_extr
		rjmp	reset_chk_done

reset_chk_po:	sbrc	XL, PORF
		rjmp	reset_ok
		rcall	beep_single			; uncrecognized reset reason, no flag set. wtf?
		rjmp	reset_chk_done

reset_ok:	rcall	beep_po				; normal power on start occured

reset_chk_done:	rcall	motor_free_run

		rjmp	start_up