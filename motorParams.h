#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#include <Arduino.h>

// =============================================================================
//                          MOTOR PIN CONFIGURATION
// =============================================================================

// Motor pin groups swapped 2026-09-05: the pins previously labelled LEFT are
// physically wired to the RIGHT motor. Measured by commanding a CCW turn - the
// firmware drove its "left" channel while the robot's physical RIGHT wheel
// responded, so the robot turned the wrong way. Forward travel was unaffected
// (2 m accurate to 1%), the signature of a left/right swap rather than
// inverted polarity. The ENCODERS are not crossed.

// LEFT MOTOR - BTS7960 Driver
constexpr uint8_t RPWM_L = 11;
constexpr uint8_t LPWM_L = 10;
constexpr uint8_t REN_L  = 24;
constexpr uint8_t LEN_L  = 25;

// RIGHT MOTOR - BTS7960 Driver
constexpr uint8_t RPWM_R = 7;
constexpr uint8_t LPWM_R = 6;
constexpr uint8_t REN_R  = 22;
constexpr uint8_t LEN_R  = 23;

// =============================================================================
//                       ENCODER PIN CONFIGURATION
//              Interrupt-capable pins on Arduino Mega 2560
// =============================================================================

// Encoders are crossed AND count backwards. Measured 2026-09-07 by turning
// the physical LEFT wheel forward by hand with the motors idle: it drove
// right_wheel_joint to -2.98 rad while left_wheel_joint stayed at zero. So
// pins 18/19 are the LEFT wheel, and forward rotation counts negative.
//
// The two faults cancel during rotation - the wheels turn opposite ways, so
// the swap and the inversion negate each other - and only show up in straight
// line travel. Verify with a single hand-turned wheel, never with a rotation.

// LEFT ENCODER
constexpr uint8_t ENC_L_A = 18;  // INT5
constexpr uint8_t ENC_L_B = 19;  // INT4

// RIGHT ENCODER
constexpr uint8_t ENC_R_A = 2;   // INT0
constexpr uint8_t ENC_R_B = 3;   // INT1

// Encoder tick direction, mirroring the INVERT_*_MOTOR pattern below.
constexpr bool INVERT_LEFT_ENCODER  = true;
constexpr bool INVERT_RIGHT_ENCODER = true;

// =============================================================================
//                          MOTOR BEHAVIOR SETTINGS
// =============================================================================

// Direction inversion (set true if motor spins opposite to expected)
constexpr bool INVERT_LEFT_MOTOR  = true;
constexpr bool INVERT_RIGHT_MOTOR = true;

// =============================================================================
//                            VELOCITY PID
// =============================================================================
// The 'm' command carries a VELOCITY setpoint in encoder counts per control
// loop, not a PWM value. ros2_control sends
//     counts_per_loop = rad_per_sec / rads_per_count / loop_rate
// with loop_rate 40, which matches MOTOR_UPDATE_MS = 25 ms below.
//
// Integer maths throughout, ros_arduino_bridge convention:
//     output += (KP*err + KD*(err - prev_err) + KI*integral) / KO
// KO is the output divisor that lets integer gains express fractions.
//
// ROS can override these with the 'y' command. Its pid_o is currently 0,
// which would divide by zero, so setPIDGains() rejects KO <= 0.
constexpr int16_t PID_KP_DEFAULT = 20;
constexpr int16_t PID_KD_DEFAULT = 12;
constexpr int16_t PID_KI_DEFAULT = 0;
constexpr int16_t PID_KO_DEFAULT = 50;

// Full scale is about 62 counts/loop (0.42 m/s at the current wheel radius).
constexpr int16_t MAX_COUNTS_PER_LOOP = 120;

// PWM limits
constexpr int16_t PWM_MAX = 255;
constexpr int16_t PWM_MIN = -255;

// Minimum PWM to overcome motor stiction (wiper motors need higher value)
constexpr uint8_t MOTOR_DEADBAND = 35;

// Acceleration rate (PWM units per update)
constexpr uint8_t ACCEL_RATE = 8;

// Deceleration rate (faster than accel for safety)
constexpr uint8_t DECEL_RATE = 15;

// Motor update interval (milliseconds)
constexpr uint8_t MOTOR_UPDATE_MS = 25;

// =============================================================================
//                      ULTRASONIC SENSOR CONFIGURATION
//               A0221AU / A02YYUW via UART (9600 baud)
// =============================================================================

// Left sensor on Serial2 (RX2 = pin 17)
// Right sensor on Serial3 (RX3 = pin 15)
constexpr uint32_t ULTRA_BAUD = 9600;

// =============================================================================
//                          LED CONFIGURATION
// =============================================================================

// LED strip data pin
constexpr uint8_t LED_PIN = 12;

// Number of LEDs on the strip
constexpr uint8_t NUM_LEDS = 16;

// LED brightness (0-255)
constexpr uint8_t LED_BRIGHTNESS = 150;

// LED update interval (milliseconds)
constexpr uint8_t LED_UPDATE_MS = 30;

// =============================================================================
//                          SERIAL CONFIGURATION
// =============================================================================

constexpr uint32_t SERIAL_BAUD = 57600;
constexpr uint8_t  CMD_BUFFER_SIZE = 48;

#endif
