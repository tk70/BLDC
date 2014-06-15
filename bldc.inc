
; *------------------------------*
; |       General settings       |
; *------------------------------*
	; The timing advance. [0-128] -> [0°-30°]
	; 0->no advance, ..., 64->15°, ..., 128->30° advance etc
	.equ TIMING_ADVANCE = 64
								
	; Active freewheeling, aka complementary PWM
	; Unfinished, don't use.
	.equ ACTIVE_FREEWHEELING = 0				; Enabled?

	; Max allowed speed on the mottor, if exceeded, the controller will be limiting throttle.
	.equ MAX_RPM = 100000	
	
	; Limit the maximum throttle based on motor speed. An attempt to prevent overcurrent.
	.equ SPEED_POWER_LIMIT = 0				; Enabled?
	.equ THROTTLE_PERCENT_PER_1K_RPM = 5			; Throttle percent allowed per 1000 RPM

	; Frequency of the outgoing power modulation
	.equ SDM_FREQUENCY = 96000
; *------------------------------*
; |    Governor mode settings    |
; *------------------------------*
	.equ GOVERNOR_MODE = 1					; Enabled?

	; Upper speed limit, speed corresponing to 100% power signal.
	.equ GOVERNOR_MAX_RPM = 96000
	
	; Lower speed limit. The controler will be trying to sustain this speed, unless nonzero power signal is given.				
	.equ GOVERNOR_MIN_RPM = 8000

	; The Governor mode uses the PID INDependent algorithm.
	; PID controler settings
	;          .---[     P     ]---.
	;          |                   |
	; E(s)-->--.---[  I/100/s  ]--(x)---> U'(S)
	;          |                   |
	;          .---[           ]---.
	;
	.equ GOV_P = 5						; Proportional gain
	.equ GOV_I = 30						; Integrator factor (percent of error value to be integrated in each cycle)

	; Anti-windup. Percent of power range. After Exceeding this limit, the integrating block stops to integrate.
	.equ AW_RNG = 100					


; *---------------------------*
; |      Startup settings     |
; *---------------------------*
	.equ START_UP_COUNTER = 40				; how many full commutations the motor must do before the switch to the running mode, 254 max
	.equ STARTUP_MAX_POWER = 20				; % max power used during startup
	.equ STARTUP_MIN_POWER = 13				; % min
	.equ STARTUP_MIN_RPM = 100
	.equ STARTUP_RPM_STEP = 7				; %
	.equ STARTUP_MAX_RPM = 200
	.equ STARTUP_FORCED_COM_NUMBER = 10


; *---------------------------*
; |     RC Pulse settings     |
; *---------------------------*
	.equ RCP_MIN_THR = 1050					; rc pulse length threshold corresponding to no power
	.equ RCP_MAX_THR = 1950					; ...and to full power. microseconds.
	.equ RCP_MIN = 800					; minimal accepted pulse length
	.equ RCP_MAX = 2200					; maximal accepted pulse length