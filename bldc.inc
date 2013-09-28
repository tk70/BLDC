; *--------------------------*
; |     General settings     |
; *--------------------------*

.equ RCP_MIN_THR = 1150					; rc pulse length threshold corresponding to no power
.equ RCP_MAX_THR = 1850					; ...and to full power. microseconds.
.equ RCP_MIN = 800					; minimal accepted pulse length
.equ RCP_MAX = 2200					; maximal accepted pulse length

.equ ZC_FILTERING = 1					; filter ZC? 0 - no filtering, 1- filtering

.equ TIMING_ADVANCE = 13				; [0-128] -> [0�-30�]. motor timing advance
							; 0->no advance, ..., 64->15�, ..., 128->30� advance etc
.equ ACTIVE_FREEWHEELING = 0				; aka complementary PWM

.equ GOVERNOR_MODE = 0
.equ GOVERNOR_MAX_RPM = 50000

.equ GOV_I_FACTOR = 2

.equ RCP_RANGE = RCP_MAX_THR - RCP_MIN_THR		; don't touch

.equ MIN_RPM = 4000
.equ MAX_RPM = 120000

.equ RPM_THR_1 = 8000
.equ PWR_THR_1 = RCP_RANGE / 2

.equ START_UP_COUNTER = 250				; how many full commutations the motor must do before the switch to the running mode, 254 max

;.equ STARTUP_PULL_POWER = RCP_RANGE / 6		; pull power, first startup step
.equ STARTUP_MAX_POWER = RCP_RANGE / 6			; max power used during startup
.equ STARTUP_MIN_POWER = RCP_RANGE / 10			; min
.equ STARTUP_MIN_RPM = 150
.equ STARTUP_FORCED_COM_NUMBER = 2

;.equ STARTUP_STABILIZATION = 0				; stabilize the rotor before starting up? may be helpful if the motor has higher load (0/1)
;;.equ STARTUP_STABIL_TIME = 333			; time of rotor stabilization, milliseconds
.equ STARTUP_STABIL_POWER = RCP_RANGE * 3 / 8