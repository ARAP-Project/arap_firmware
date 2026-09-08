#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#include <Arduino.h>

// ---------- MOTOR PINS (BTS7960) ----------
constexpr uint8_t RPWM_L = 7;
constexpr uint8_t LPWM_L = 6;
constexpr uint8_t RPWM_R = 11;
constexpr uint8_t LPWM_R = 10;
constexpr uint8_t REN_L  = 22;
constexpr uint8_t LEN_L  = 23;
constexpr uint8_t REN_R  = 24;
constexpr uint8_t LEN_R  = 25;

// ---------- ENCODER PINS ----------
constexpr uint8_t ENC_L_A = 2;
constexpr uint8_t ENC_L_B = 3;
constexpr uint8_t ENC_R_A = 18;
constexpr uint8_t ENC_R_B = 19;

// ---------- DIRECTION INVERSION ----------
constexpr bool INVERT_LEFT_MOTOR    = true;
constexpr bool INVERT_RIGHT_MOTOR   = true;
constexpr bool INVERT_LEFT_ENCODER  = true;
constexpr bool INVERT_RIGHT_ENCODER = true;

// ---------- PWM LIMITS ----------
constexpr int16_t PWM_MAX         = 255;
constexpr int16_t PWM_MIN         = -255;
constexpr uint8_t MOTOR_DEADBAND  = 5;
constexpr uint8_t MOTOR_UPDATE_MS = 25;

// ---------- SPEED SCALE ----------
// Calibrated from 'x' table: 255 PWM -> ~2936 ticks/sec -> /40 loops = ~73 ticks/loop.
constexpr int16_t MAX_TICKS_PER_LOOP  = 65;
constexpr int16_t MAX_COUNTS_PER_LOOP = MAX_TICKS_PER_LOOP;

// ---------- STICTION KICK ----------
// CHANGED: was 150. Calibration showed break-free at ~30 PWM, so a 150 PWM
// kick was wildly oversized and caused the lurch/jerk on start. 45 gives a
// small margin above break-free without slamming the motor.
constexpr int16_t STICTION_KICK_PWM_MIN = 45;
constexpr uint8_t STICTION_KICK_LOOPS   = 4;

// =============================================================================
//   LOW-SPEED LIMITS  (derived from 'x' calibration)
// =============================================================================
// Calibration showed the motors do not turn below ~30 PWM FROM A STANDSTILL,
// and at 30 PWM they already produce ~8 ticks/loop. So the SLOWEST achievable
// COLD-START steady speed is ~8 ticks/loop. Targets below that cannot break
// free from rest. ROS should not command below MIN_MOVE_TICKS.
//
//   MOTOR_MIN_MOVE_PWM  : just above the 30 PWM break-free point, for margin.
//                         NOW APPLIED ONLY WHEN THE WHEEL IS STALLED (see
//                         WHEEL_STALL_TICKS), not while it is already turning.
//   MIN_MOVE_TICKS      : floor applied to nonzero targets so tiny commands
//                         map to the slowest real speed instead of stalling.
//   WHEEL_STALL_TICKS   : measured ticks/loop below which the wheel is treated
//                         as stalled; only then is the break-free PWM floor
//                         forced. Once moving, the motor holds far below
//                         MOTOR_MIN_MOVE_PWM, so forcing 35 PWM on a moving
//                         wheel caused the surge/coast jerking at low speed.
// -----------------------------------------------------------------------------
constexpr int16_t MOTOR_MIN_MOVE_PWM = 35;   // break-free ~30 PWM + margin
// CHANGED: was 1. The real measured steady-state floor is ~8 ticks/loop.
// With this set to 8, a command like 'm 5 5' is clamped up to the slowest
// speed the motor can actually hold, instead of fighting the PWM floor and
// glitching with a limit-cycle.
constexpr int16_t MIN_MOVE_TICKS     = 8;    // slowest achievable ticks/loop
// NEW: stall-detection window. If the measured delta this cycle is within
// +/- this many ticks of zero, the wheel is considered stalled and the
// break-free PWM floor is allowed to fire. Above it, the PID is free to use
// the full low PWM range for smooth steady low-speed running. Tighten to 1
// if you still see an occasional hiccup near MIN_MOVE_TICKS.
constexpr long    WHEEL_STALL_TICKS  = 2;

// ---------- TARGET SLEW LIMIT (optional smoothing) ----------
// Max change in target ticks/loop applied per control cycle. Makes large
// speed jumps (e.g. m 5 5 -> m 35 35) ramp over a few loops instead of
// stepping instantly. Set high (e.g. >= MAX_TICKS_PER_LOOP) to effectively
// disable. 8 ticks/loop per 25ms cycle is a gentle but responsive ramp.
constexpr int16_t TARGET_SLEW_STEP = 8;
// =============================================================================

// ---------- PER-MOTOR PWM BIAS (defaults; live-tunable via 'z') -------------
constexpr int16_t DEFAULT_LEFT_PWM_BIAS  = 0;
constexpr int16_t DEFAULT_RIGHT_PWM_BIAS = 0;   // was 15 - motors well-matched

// ---------- PID DEFAULTS ----------
// CHANGED: Kp 2.5 -> 1.2 and PID_CORRECTION_MAX 150 -> 80.
// At low speed (~8 ticks/loop) the +/-1 tick quantization is ~12% of the
// signal. With Kp=2.5 the controller chased that quantization noise and,
// together with the unconditional PWM floor, produced the limit-cycle
// jerking seen on 'm 8 8'. A softer Kp lets Ki do the steady-state work and
// the lower correction clamp keeps single-cycle overshoots small. If higher
// speeds now feel sluggish to settle, nudge Kp back up toward 1.6-1.8.
constexpr float DEFAULT_PID_KP    = 1.2;     // was 2.5 - too hot at low speed
constexpr float DEFAULT_PID_KI    = 1.2;
constexpr float DEFAULT_PID_KD    = 0.1;
constexpr float DEFAULT_PID_I_MAX = 120.0;   // was 300 - lower = less low-speed lurch
constexpr int16_t PID_CORRECTION_MAX = 80;   // was 150 - cap big single-cycle swings

// ---------- ULTRASONIC ----------
constexpr uint32_t ULTRA_BAUD   = 9600;
constexpr int16_t  ULTRA_MIN_MM = 30;
constexpr int16_t  ULTRA_MAX_MM = 4500;

// ---------- LED ----------
constexpr uint8_t LED_PIN        = 12;
constexpr uint8_t NUM_LEDS       = 110;
constexpr uint8_t LED_BRIGHTNESS = 100;
constexpr uint8_t LED_UPDATE_MS  = 80;

// ---------- SERIAL ----------
constexpr uint32_t SERIAL_BAUD     = 57600;
constexpr uint8_t  CMD_BUFFER_SIZE = 64;

// ---------- GPS ----------
constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint8_t  GPS_NAV_FREQ = 5;

// ---------- DELIVERY LOCK ----------
constexpr uint8_t  LOCK_PIN            = 26;
constexpr uint16_t LOCK_PULSE_MS       = 1000;  // pulse duration for brief unlock
constexpr bool     LOCK_ACTIVE_LOW     = false; // SSR is active-HIGH
constexpr uint32_t LOCK_MAX_HOLD_MS    = 60000; // auto re-lock after 60s safety

#endif
