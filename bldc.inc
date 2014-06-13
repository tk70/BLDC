; *--------------------------*
; |     General settings     |
; *--------------------------*

.equ RCP_MIN_THR = 1050					; rc pulse length threshold corresponding to no power
.equ RCP_MAX_THR = 1950					; ...and to full power. microseconds.
.equ RCP_MIN = 800					; minimal accepted pulse length
.equ RCP_MAX = 2200					; maximal accepted pulse length

.equ TIMING_ADVANCE = 32				; [0-128] -> [0°-30°]. motor timing advance
							; 0->no advance, ..., 64->15°, ..., 128->30° advance etc
.equ ACTIVE_FREEWHEELING = 0				; aka complementary PWM

.equ SPEED_POWER_LIMIT = 1
.equ THROTTLE_PERCENT_PER_1K_RPM = 4

.equ GOVERNOR_MODE = 0
.equ GOVERNOR_MAX_RPM = 48000
.equ GOVERNOR_MIN_RPM = 8000
.equ GOV_P = 20
.equ MIN_RPM = 4000
.equ MAX_RPM = 120000

.equ START_UP_COUNTER = 40				; how many full commutations the motor must do before the switch to the running mode, 254 max

.equ STARTUP_MAX_POWER = 20				; % max power used during startup
.equ STARTUP_MIN_POWER = 13				; % min
.equ STARTUP_MIN_RPM = 100
.equ STARTUP_RPM_STEP = 7				; %
.equ STARTUP_MAX_RPM = 200
.equ STARTUP_FORCED_COM_NUMBER = 10