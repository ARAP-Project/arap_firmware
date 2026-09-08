#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>

constexpr char CMD_MOTOR       = 'm';
constexpr char CMD_RAW_PWM     = 'w';
constexpr char CMD_STOP        = 'k';
constexpr char CMD_BRAKE       = 'b';
constexpr char CMD_CALIBRATE   = 'x';
constexpr char CMD_TUNE        = 'y';
constexpr char CMD_BIAS        = 'z';
constexpr char CMD_ZN_FIND     = 'f';
constexpr char CMD_ZN_APPLY    = 'a';
constexpr char CMD_ENCODER     = 'e';
constexpr char CMD_ENC_RESET   = 'r';
constexpr char CMD_GPS         = 'g';
constexpr char CMD_ULTRA       = 'u';
constexpr char CMD_STATUS      = 's';
constexpr char CMD_LED_MODE    = 'l';
constexpr char CMD_LED_COLOR   = 'c';
constexpr char CMD_LED_BRIGHT  = 'd';
constexpr char CMD_PING        = 'p';
constexpr char CMD_HELP        = 'h';
constexpr char CMD_LOCK        = 'o';  // o / o 1 / o 0 / o <ms>
constexpr char CMD_LOCK_STATUS = 'q';  // query lock state (for ROS)
constexpr char CMD_TPR_MEASURE = 'v';  // ticks-per-rev hand-turn measure (toggle)

enum class LedMode : uint8_t {
    OFF = 0,
    IDLE,
    RAINBOW,
    CHASE,
    PULSE,
    FIRE,
    CUSTOM,
    BT_CONNECTED,
    BT_ACTIVE,
    FORWARD_IND,
    REVERSE_IND,
    TURN_LEFT,
    TURN_RIGHT,
    MODE_COUNT
};

enum class RobotState : uint8_t {
    IDLE = 0,
    MOVING_FORWARD,
    MOVING_BACKWARD,
    TURNING_LEFT,
    TURNING_RIGHT,
    SPINNING,
    BRAKING,
    ERROR
};

enum class LockState : uint8_t {
    LOCKED = 0,
    UNLOCKED = 1
};

#endif
