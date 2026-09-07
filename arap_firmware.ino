#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "motorParams.h"
#include "commands.h"
#include "led.h"

// =============================================================================
//      ARAP - MOTOR + ENCODER + GPS + LED + ULTRASONIC - 24V WIPER MOTORS
//                         Arduino Mega 2560
// =============================================================================

// --- GPS OBJECT ---
SFE_UBLOX_GNSS gps;
bool gpsAvailable = false;

struct GPSData {
    double lat;
    double lon;
    double alt;
    uint8_t sats;
    uint8_t fix;
    float hAcc;
};

GPSData gpsData;

// --- MOTOR STATE ---
int16_t targetLeft = 0, targetRight = 0;
int16_t currentLeft = 0, currentRight = 0;
unsigned long lastMotorUpdate = 0;

// --- ENCODER STATE ---
volatile long encoderLeftCount  = 0;
volatile long encoderRightCount = 0;

// --- ULTRASONIC STATE ---
// A0221AU sends 4-byte packets: [0xFF] [DATA_H] [DATA_L] [CHECKSUM]
int16_t ultraLeftDist  = -1;  // mm, -1 = no reading
int16_t ultraRightDist = -1;
uint8_t ultraLeftBuf[4];
uint8_t ultraLeftIdx = 0;
uint8_t ultraRightBuf[4];
uint8_t ultraRightIdx = 0;

// --- SERIAL ---
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;

// =============================================================================
//                       ENCODER ISR (Interrupt Routines)
// =============================================================================

constexpr int8_t LEFT_ENC_DIR  = INVERT_LEFT_ENCODER  ? -1 : 1;
constexpr int8_t RIGHT_ENC_DIR = INVERT_RIGHT_ENCODER ? -1 : 1;

void encoderLeftA_ISR() {
    if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) {
        encoderLeftCount -= LEFT_ENC_DIR;
    } else {
        encoderLeftCount += LEFT_ENC_DIR;
    }
}

void encoderLeftB_ISR() {
    if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) {
        encoderLeftCount += LEFT_ENC_DIR;
    } else {
        encoderLeftCount -= LEFT_ENC_DIR;
    }
}

void encoderRightA_ISR() {
    if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) {
        encoderRightCount -= RIGHT_ENC_DIR;
    } else {
        encoderRightCount += RIGHT_ENC_DIR;
    }
}

void encoderRightB_ISR() {
    if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) {
        encoderRightCount += RIGHT_ENC_DIR;
    } else {
        encoderRightCount -= RIGHT_ENC_DIR;
    }
}

// =============================================================================
//                          MOTOR FUNCTIONS
// =============================================================================

void enableMotors() {
    digitalWrite(REN_L, HIGH);
    digitalWrite(LEN_L, HIGH);
    digitalWrite(REN_R, HIGH);
    digitalWrite(LEN_R, HIGH);
}

void disableMotors() {
    digitalWrite(REN_L, LOW);
    digitalWrite(LEN_L, LOW);
    digitalWrite(REN_R, LOW);
    digitalWrite(LEN_R, LOW);
}

void applyPWM(uint8_t rpwmPin, uint8_t lpwmPin, int16_t speed, bool invert) {
    if (invert) speed = -speed;

    if (speed > 0) {
        uint8_t pwm = (speed < MOTOR_DEADBAND) ? MOTOR_DEADBAND : speed;
        analogWrite(rpwmPin, pwm);
        analogWrite(lpwmPin, 0);
    } else if (speed < 0) {
        uint8_t pwm = (-speed < MOTOR_DEADBAND) ? MOTOR_DEADBAND : -speed;
        analogWrite(rpwmPin, 0);
        analogWrite(lpwmPin, pwm);
    } else {
        analogWrite(rpwmPin, 0);
        analogWrite(lpwmPin, 0);
    }
}

int16_t rampValue(int16_t current, int16_t target) {
    if (current == target) return current;

    uint8_t rate;
    if (target == 0 || (abs(target) < abs(current))) {
        rate = DECEL_RATE;
    } else {
        rate = ACCEL_RATE;
    }

    if (current < target) {
        current += rate;
        if (current > target) current = target;
    } else {
        current -= rate;
        if (current < target) current = target;
    }
    return current;
}

void stopMotors() {
    targetLeft = 0;
    targetRight = 0;
    currentLeft = 0;
    currentRight = 0;
    analogWrite(RPWM_L, 0);
    analogWrite(LPWM_L, 0);
    analogWrite(RPWM_R, 0);
    analogWrite(LPWM_R, 0);
    Led::showDirection(0, 0);
}

void brakeMotors() {
    targetLeft = 0;
    targetRight = 0;
    currentLeft = 0;
    currentRight = 0;
    Led::onRobotStateChange(RobotState::BRAKING);
    analogWrite(RPWM_L, 255);
    analogWrite(LPWM_L, 255);
    analogWrite(RPWM_R, 255);
    analogWrite(LPWM_R, 255);
    delay(200);
    analogWrite(RPWM_L, 0);
    analogWrite(LPWM_L, 0);
    analogWrite(RPWM_R, 0);
    analogWrite(LPWM_R, 0);
    Led::showDirection(0, 0);
}

void updateMotors() {
    if (millis() - lastMotorUpdate < MOTOR_UPDATE_MS) return;
    lastMotorUpdate = millis();

    currentLeft  = rampValue(currentLeft, targetLeft);
    currentRight = rampValue(currentRight, targetRight);

    applyPWM(RPWM_L, LPWM_L, currentLeft,  INVERT_LEFT_MOTOR);
    applyPWM(RPWM_R, LPWM_R, currentRight, INVERT_RIGHT_MOTOR);
}

// =============================================================================
//                          ENCODER FUNCTIONS
// =============================================================================

void resetEncoders() {
    noInterrupts();
    encoderLeftCount  = 0;
    encoderRightCount = 0;
    interrupts();
}

void sendEncoderData() {
    long leftTicks, rightTicks;

    noInterrupts();
    leftTicks  = encoderLeftCount;
    rightTicks = encoderRightCount;
    interrupts();

    Serial.print(F("e "));
    Serial.print(leftTicks);
    Serial.print(F(" "));
    Serial.println(rightTicks);
}

// =============================================================================
//                          GPS FUNCTIONS
// =============================================================================

void readGPS() {
    if (!gpsAvailable) return;

    if (gps.getPVT()) {
        gpsData.lat  = gps.getLatitude()  / 10000000.0;
        gpsData.lon  = gps.getLongitude() / 10000000.0;
        gpsData.alt  = gps.getAltitude()  / 1000.0;
        gpsData.sats = gps.getSIV();
        gpsData.fix  = gps.getFixType();

        long acc = gps.getHorizontalAccEst();
        if (acc > 0)
            gpsData.hAcc = acc / 1000.0;
        else
            gpsData.hAcc = -1.0;
    }
}

void sendGPSData() {
    Serial.print(F("g "));
    Serial.print(gpsData.lat, 7);
    Serial.print(F(" "));
    Serial.print(gpsData.lon, 7);
    Serial.print(F(" "));
    Serial.print(gpsData.alt, 2);
    Serial.print(F(" "));
    Serial.print(gpsData.fix);
    Serial.print(F(" "));
    Serial.print(gpsData.sats);
    Serial.print(F(" "));
    Serial.println(gpsData.hAcc, 2);
}

// =============================================================================
//                       ULTRASONIC FUNCTIONS
//          A0221AU protocol: 4 bytes [0xFF][DATA_H][DATA_L][SUM]
//          Distance in mm = (DATA_H << 8) | DATA_L
//          Checksum = (0xFF + DATA_H + DATA_L) & 0xFF
// =============================================================================

void parseUltraSensor(HardwareSerial &ser, uint8_t *buf, uint8_t &idx, int16_t &dist) {
    while (ser.available()) {
        uint8_t b = ser.read();

        if (idx == 0) {
            // Wait for header byte
            if (b == 0xFF) {
                buf[0] = b;
                idx = 1;
            }
        } else {
            buf[idx] = b;
            idx++;

            if (idx >= 4) {
                // Validate checksum
                uint8_t checksum = (buf[0] + buf[1] + buf[2]) & 0xFF;
                if (checksum == buf[3]) {
                    int16_t d = ((int16_t)buf[1] << 8) | buf[2];
                    if (d >= 30 && d <= 4500) {
                        dist = d;  // Valid range: 30mm to 4500mm
                    }
                }
                idx = 0;
            }
        }
    }
}

void readUltrasonic() {
    parseUltraSensor(Serial2, ultraLeftBuf,  ultraLeftIdx,  ultraLeftDist);
    parseUltraSensor(Serial3, ultraRightBuf, ultraRightIdx, ultraRightDist);
}

void sendUltrasonicData() {
    // Output format for ROS: u <left_mm> <right_mm>
    Serial.print(F("u "));
    Serial.print(ultraLeftDist);
    Serial.print(F(" "));
    Serial.println(ultraRightDist);
}

// =============================================================================
//                          COMMAND PROCESSING
// =============================================================================

void processCommand(char* cmd) {
    switch (cmd[0]) {
        case CMD_MOTOR: {
            int16_t left = 0, right = 0;
            if (sscanf(cmd + 1, "%d %d", &left, &right) == 2) {
                left  = constrain(left,  PWM_MIN, PWM_MAX);
                right = constrain(right, PWM_MIN, PWM_MAX);
                targetLeft  = left;
                targetRight = right;
                enableMotors();
                Led::showDirection(left, right);
                Serial.print(F("Motor: L="));
                Serial.print(left);
                Serial.print(F(" R="));
                Serial.println(right);
            } else {
                Serial.println(F("Usage: m <left> <right>  (e.g. m 150 150)"));
            }
            break;
        }
        case CMD_STOP:
            stopMotors();
            Serial.println(F("STOP"));
            break;

        case CMD_BRAKE:
            brakeMotors();
            Serial.println(F("BRAKE"));
            break;

        case CMD_ENCODER:
            sendEncoderData();
            break;

        case CMD_ENC_RESET:
            resetEncoders();
            Serial.println(F("Encoders reset"));
            break;

        case CMD_GPS:
            sendGPSData();
            break;

        case CMD_ULTRA:
            sendUltrasonicData();
            break;

        case CMD_LED_MODE: {
            int mode = 0;
            if (sscanf(cmd + 1, "%d", &mode) == 1) {
                Led::setMode(static_cast<LedMode>(mode));
                Serial.print(F("LED mode: "));
                Serial.println(mode);
            } else {
                Serial.println(F("Usage: l <0-12>"));
            }
            break;
        }

        case CMD_LED_COLOR: {
            int r = 0, g = 0, b = 0;
            if (sscanf(cmd + 1, "%d %d %d", &r, &g, &b) == 3) {
                Led::setColor(r, g, b);
                Serial.println(F("LED color set"));
            } else {
                Serial.println(F("Usage: c <R> <G> <B>"));
            }
            break;
        }

        case CMD_LED_BRIGHT: {
            int bright = 0;
            if (sscanf(cmd + 1, "%d", &bright) == 1) {
                Led::setBrightness(constrain(bright, 0, 255));
                Serial.print(F("LED brightness: "));
                Serial.println(bright);
            } else {
                Serial.println(F("Usage: d <0-255>"));
            }
            break;
        }

        case CMD_STATUS: {
            long lt, rt;
            noInterrupts();
            lt = encoderLeftCount;
            rt = encoderRightCount;
            interrupts();

            Serial.println(F("--- Motor ---"));
            Serial.print(F("Target:  L="));
            Serial.print(targetLeft);
            Serial.print(F(" R="));
            Serial.println(targetRight);
            Serial.print(F("Current: L="));
            Serial.print(currentLeft);
            Serial.print(F(" R="));
            Serial.println(currentRight);

            Serial.println(F("--- Encoder ---"));
            Serial.print(F("Ticks: L="));
            Serial.print(lt);
            Serial.print(F(" R="));
            Serial.println(rt);

            Serial.println(F("--- GPS ---"));
            Serial.print(F("Fix: "));
            Serial.print(gpsData.fix);
            Serial.print(F("  Sats: "));
            Serial.println(gpsData.sats);
            Serial.print(F("Lat: "));
            Serial.print(gpsData.lat, 7);
            Serial.print(F("  Lon: "));
            Serial.println(gpsData.lon, 7);
            Serial.print(F("Alt: "));
            Serial.print(gpsData.alt, 2);
            Serial.print(F(" m  hAcc: "));
            Serial.print(gpsData.hAcc, 2);
            Serial.println(F(" m"));

            Serial.println(F("--- Ultrasonic ---"));
            Serial.print(F("Left: "));
            if (ultraLeftDist < 0) Serial.print(F("N/A"));
            else { Serial.print(ultraLeftDist); Serial.print(F(" mm")); }
            Serial.print(F("  Right: "));
            if (ultraRightDist < 0) Serial.println(F("N/A"));
            else { Serial.print(ultraRightDist); Serial.println(F(" mm")); }

            Serial.println(F("--- LED ---"));
            Serial.print(F("Mode: "));
            Serial.println(static_cast<uint8_t>(Led::getMode()));
            break;
        }

        case CMD_PING:
            Serial.println(F("PONG"));
            break;

        case CMD_HELP:
            Serial.println(F("=== ARAP Commands ==="));
            Serial.println(F("m L R   - set motor speeds (-255 to 255)"));
            Serial.println(F("k       - emergency stop"));
            Serial.println(F("b       - active brake"));
            Serial.println(F("e       - send encoder ticks (for ROS)"));
            Serial.println(F("r       - reset encoder counts"));
            Serial.println(F("g       - send GPS data (for ROS)"));
            Serial.println(F("u       - send ultrasonic distances (for ROS)"));
            Serial.println(F("l N     - set LED mode (0-12)"));
            Serial.println(F("c R G B - set custom LED color"));
            Serial.println(F("d N     - set LED brightness (0-255)"));
            Serial.println(F("s       - show full status"));
            Serial.println(F("p       - ping"));
            Serial.println(F(""));
            Serial.println(F("Motor examples:"));
            Serial.println(F("  m 100 100   - slow forward"));
            Serial.println(F("  m 200 200   - medium forward"));
            Serial.println(F("  m -150 -150 - reverse"));
            Serial.println(F("  m -150 150  - spin left"));
            Serial.println(F("  m 150 -150  - spin right"));
            Serial.println(F(""));
            Serial.println(F("ROS output formats:"));
            Serial.println(F("  e <left> <right>"));
            Serial.println(F("  g <lat> <lon> <alt> <fix> <sats> <hAcc>"));
            Serial.println(F("  u <left_mm> <right_mm>"));
            break;

        default:
            Serial.println(F("Unknown cmd. Send 'h' for help."));
            break;
    }
}

// =============================================================================
//                          SETUP & LOOP
// =============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);

    // --- LED Init ---
    Led::init();
    Led::onRobotStateChange(RobotState::IDLE);

    // --- I2C for GPS ---
    Wire.begin();
    Wire.setClock(400000);

    // --- GPS Init ---
    Serial.println(F("Initializing NEO-M9N GPS..."));
    if (!gps.begin()) {
        Serial.println(F("WARNING: GPS not found on I2C! Continuing without GPS."));
        gpsAvailable = false;
    } else {
        gps.setI2COutput(COM_TYPE_UBX);
        gps.setNavigationFrequency(5);
        gps.setAutoPVT(true);
        gpsAvailable = true;
        Serial.println(F("GPS OK!"));
    }

    // --- Ultrasonic sensors on Serial2 and Serial3 ---
    Serial2.begin(ULTRA_BAUD);  // Left sensor (RX2 = pin 17)
    Serial3.begin(ULTRA_BAUD);  // Right sensor (RX3 = pin 15)
    Serial.println(F("Ultrasonic: Left=Serial2(RX:17) Right=Serial3(RX:15)"));

    // --- Motor pins ---
    pinMode(RPWM_L, OUTPUT);
    pinMode(LPWM_L, OUTPUT);
    pinMode(REN_L,  OUTPUT);
    pinMode(LEN_L,  OUTPUT);
    pinMode(RPWM_R, OUTPUT);
    pinMode(LPWM_R, OUTPUT);
    pinMode(REN_R,  OUTPUT);
    pinMode(LEN_R,  OUTPUT);

    // --- Encoder pins ---
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);

    // --- Attach interrupts (4x quadrature) ---
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), encoderLeftA_ISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_L_B), encoderLeftB_ISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), encoderRightA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_B), encoderRightB_ISR, CHANGE);

    stopMotors();
    enableMotors();

    Serial.println(F(""));
    Serial.println(F("=== ARAP Ready ==="));
    Serial.println(F("Encoder: L(A=2,B=3) R(A=18,B=19)"));
    Serial.println(F("GPS: I2C (SDA=20, SCL=21)"));
    Serial.println(F("LED: Pin 12"));
    Serial.println(F("Send 'h' for help"));
    Serial.println(F(""));
}

void loop() {
    // Read serial commands
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0';
                processCommand(cmdBuffer);
                cmdIndex = 0;
            }
        } else if (cmdIndex < sizeof(cmdBuffer) - 1) {
            cmdBuffer[cmdIndex++] = c;
        }
    }

    // Update all systems
    updateMotors();
    readGPS();
    readUltrasonic();
    Led::update();
}
