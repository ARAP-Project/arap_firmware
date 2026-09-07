#ifndef COMMANDS_H
#define COMMANDS_H

// =============================================================================
//                          SERIAL COMMAND DEFINITIONS
// =============================================================================

// Motor commands
constexpr char CMD_MOTOR     = 'm';  // m L R - set wheel velocity, counts/loop
constexpr char CMD_PID       = 'y';  // y P:D:I:O - set velocity PID gains
constexpr char CMD_STOP      = 'k';  // k - emergency stop
constexpr char CMD_BRAKE     = 'b';  // b - active brake

// Encoder commands
constexpr char CMD_ENCODER   = 'e';  // e - send encoder ticks to ROS
constexpr char CMD_ENC_RESET = 'r';  // r - reset encoder counts

// GPS commands
constexpr char CMD_GPS       = 'g';  // g - send GPS data to ROS

// Ultrasonic commands
constexpr char CMD_ULTRA     = 'u';  // u - send ultrasonic distances to ROS

// LED commands
constexpr char CMD_LED_MODE  = 'l';  // l N - set LED mode (0-11)
constexpr char CMD_LED_COLOR = 'c';  // c R G B - set custom color
constexpr char CMD_LED_BRIGHT = 'd'; // d N - set brightness

// System commands
constexpr char CMD_STATUS    = 's';  // s - show status
constexpr char CMD_PING      = 'p';  // p - ping (returns PONG)
constexpr char CMD_HELP      = 'h';  // h - show help

// =============================================================================
//                          LED MODE DEFINITIONS
// =============================================================================

enum class LedMode : uint8_t {
    OFF = 0,
    IDLE,           // Blue breathing - robot idle
    RAINBOW,        // Rainbow cycle - moving
    CHASE,          // Color chase
    PULSE,          // Breathing pulse
    FIRE,           // Fire effect
    CUSTOM,         // Custom solid color
    BT_CONNECTED,   // Green breathing - connected
    BT_ACTIVE,      // Cyan flowing - active comms
    FORWARD_IND,    // Green chase forward
    REVERSE_IND,    // Red chase backward
    TURN_LEFT,      // Orange left half
    TURN_RIGHT,     // Orange right half
    MODE_COUNT
};

// =============================================================================
//                          ROBOT STATE DEFINITIONS
// =============================================================================

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

#endif
