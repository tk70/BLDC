; *--------------------------*
; |     General settings     |
; *--------------------------*

.equ RCP_MIN_THR = 1050				; rc pulse length threshold corresponding to no power
.equ RCP_MAX_THR = 1950					; ...and to full power. microseconds.
.equ RCP_MIN = 800					; minimal accepted pulse length
.equ RCP_MAX = 2200					; maximal accepted pulse length


.equ TIMING_ADVANCE = 0				; [0-128] -> [0°-30°]. motor timing advance
							; 0->no advance, ..., 64->15°, ..., 128->30° advance etc
.equ ACTIVE_FREEWHEELING = 0				; aka complementary PWM


.equ RCP_RANGE = RCP_MAX_THR - RCP_MIN_THR		; don't touch


.equ GOVERNOR_MODE = 0
.equ GOVERNOR_MAX_RPM = 40000
.equ GOV_I_FACTOR = 2
.equ MIN_RPM = 4000
.equ MAX_RPM = 120000

.equ RPM_THR_1 = 8000
.equ PWR_THR_1 = RCP_RANGE / 2

.equ START_UP_COUNTER = 254				; how many full commutations the motor must do before the switch to the running mode, 254 max

.equ STARTUP_MAX_POWER = 16				; % max power used during startup
.equ STARTUP_MIN_POWER = 10				; % min
.equ STARTUP_MIN_RPM = 150
.equ STARTUP_FORCED_COM_NUMBER = 2

.equ STARTUP_STABIL_POWER = RCP_RANGE * 3 / 8





.equ ZC_FILTERING = 1