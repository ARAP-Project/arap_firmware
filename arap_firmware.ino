#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "motorParams.h"
#include "commands.h"
#include "led.h"

// =============================================================================
//   Arduino Mega 2560 + BTS7960 + Bilba 12V motors + quadrature encoders
//
//   SERIAL OUTPUT POLICY:
//   Only machine-parseable telemetry is printed. Lines ROS may receive:
//       e <Ldelta> <Rdelta> <Ltotal> <Rtotal>   per control cycle (when moving)
//       e <Ltotal> <Rtotal>                      reply to 'e' command
//       g <fix> <lat> <lon> <alt>
//       u <Ldist_mm> <Rdist_mm>
//       lock <state:0|1> <ms_in_state>
//
//   NOTE: the per-cycle 'e' line now carries FOUR fields (deltas + exact
//   running totals). ROS odometry should integrate the TOTALS, not sum the
//   deltas. The 'e' COMMAND reply (sendEncoderTotals) still returns two fields.
//   If your parser can't handle the 4-field cycle line, change its leading
//   token to 't' (see updateMotors) so the 2-field 'e' reply stays clean.
// =============================================================================

// ---------- GPS ----------
SFE_UBLOX_GNSS gps;
bool gpsAvailable = false;

struct GPSData {
    double  lat;
    double  lon;
    double  alt;
    uint8_t sats;
    uint8_t fix;
};
GPSData gpsData = {0.0, 0.0, 0.0, 0, 0};

// ---------- PID STATE ----------
struct PIDState {
    float   integral;
    float   prevError;
    int16_t pwmOutput;
    int16_t ffPWM;
    int16_t lastTarget;
    uint8_t kickLoopsRemaining;
};

static void pidStateReset(PIDState &pid) {
    pid.integral           = 0.0f;
    pid.prevError          = 0.0f;
    pid.pwmOutput          = 0;
    pid.ffPWM              = 0;
    pid.lastTarget         = 0;
    pid.kickLoopsRemaining = 0;
}

// ---------- LIVE-TUNABLE PARAMETERS ----------
float   pidKp     = DEFAULT_PID_KP;
float   pidKi     = DEFAULT_PID_KI;
float   pidKd     = DEFAULT_PID_KD;
float   pidIMax   = DEFAULT_PID_I_MAX;
int16_t leftBias  = DEFAULT_LEFT_PWM_BIAS;
int16_t rightBias = DEFAULT_RIGHT_PWM_BIAS;

// ---------- CONTROLLER STATE ----------
int16_t  targetLeftCPL  = 0;
int16_t  targetRightCPL = 0;
// Slew-limited effective targets (ramp toward target* each control cycle).
int16_t  rampedLeftCPL  = 0;
int16_t  rampedRightCPL = 0;
PIDState pidLeft;
PIDState pidRight;

bool rawPWMMode        = false;
bool pidNeedsResync    = true;
bool tickStreamEnabled = false;
unsigned long lastMotorUpdate = 0;

// ---------- ENCODER STATE ----------
volatile long encoderLeftCount  = 0;
volatile long encoderRightCount = 0;
long prevEncoderLeft  = 0;
long prevEncoderRight = 0;

// ---------- SPEED-ESTIMATE SMOOTHING ----------
// Average the per-cycle delta over SPEED_AVG_WINDOW cycles to cut the
// ~3% single-tick quantization jitter the PID would otherwise chase.
// This smooths ONLY the speed estimate fed to the PID; the telemetry and
// odometry path stays exact (raw deltas + exact totals).
//   - Larger window  = smoother but laggier (more phase delay).
//   - 3 cycles @ 25ms = 75ms averaging window. Drop to 2 if response is soft.
constexpr uint8_t SPEED_AVG_WINDOW = 3;
long    deltaBufLeft[SPEED_AVG_WINDOW]  = {0};
long    deltaBufRight[SPEED_AVG_WINDOW] = {0};
uint8_t deltaBufIdx = 0;

static long pushAndAverage(long *buf, long newVal) {
    buf[deltaBufIdx % SPEED_AVG_WINDOW] = newVal;
    long sum = 0;
    for (uint8_t i = 0; i < SPEED_AVG_WINDOW; i++) sum += buf[i];
    return sum / SPEED_AVG_WINDOW;
}

static void resetSpeedAvg() {
    for (uint8_t i = 0; i < SPEED_AVG_WINDOW; i++) {
        deltaBufLeft[i]  = 0;
        deltaBufRight[i] = 0;
    }
    deltaBufIdx = 0;
}

// ---------- ULTRASONIC STATE ----------
int16_t ultraLeftDist  = -1;
int16_t ultraRightDist = -1;
uint8_t ultraLeftBuf[4]  = {0};
uint8_t ultraLeftIdx     = 0;
uint8_t ultraRightBuf[4] = {0};
uint8_t ultraRightIdx    = 0;

// ---------- LOCK STATE ----------
LockState     lockState          = LockState::LOCKED;
uint32_t      lockStateSince     = 0;     // millis() when state last changed
uint32_t      lockAutoRelockAt   = 0;     // 0 = no pending auto-relock
bool          lockAutoRelockArmed = false;

// ---------- SERIAL ----------
char    cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;


// =============================================================================
//   ENCODER ISRs
// =============================================================================

void encoderLeftA_ISR() {
    const bool same = (digitalRead(ENC_L_A) == digitalRead(ENC_L_B));
    if (INVERT_LEFT_ENCODER) {
        if (same) encoderLeftCount++; else encoderLeftCount--;
    } else {
        if (same) encoderLeftCount--; else encoderLeftCount++;
    }
}

void encoderLeftB_ISR() {
    const bool same = (digitalRead(ENC_L_A) == digitalRead(ENC_L_B));
    if (INVERT_LEFT_ENCODER) {
        if (same) encoderLeftCount--; else encoderLeftCount++;
    } else {
        if (same) encoderLeftCount++; else encoderLeftCount--;
    }
}

void encoderRightA_ISR() {
    const bool same = (digitalRead(ENC_R_A) == digitalRead(ENC_R_B));
    if (INVERT_RIGHT_ENCODER) {
        if (same) encoderRightCount++; else encoderRightCount--;
    } else {
        if (same) encoderRightCount--; else encoderRightCount++;
    }
}

void encoderRightB_ISR() {
    const bool same = (digitalRead(ENC_R_A) == digitalRead(ENC_R_B));
    if (INVERT_RIGHT_ENCODER) {
        if (same) encoderRightCount--; else encoderRightCount++;
    } else {
        if (same) encoderRightCount++; else encoderRightCount--;
    }
}


// =============================================================================
//   MOTOR DRIVER
// =============================================================================

void motorsEnable() {
    digitalWrite(REN_L, HIGH);
    digitalWrite(LEN_L, HIGH);
    digitalWrite(REN_R, HIGH);
    digitalWrite(LEN_R, HIGH);
}

void motorsDisable() {
    digitalWrite(REN_L, LOW);
    digitalWrite(LEN_L, LOW);
    digitalWrite(REN_R, LOW);
    digitalWrite(LEN_R, LOW);
}

void applyPWM(uint8_t rpwmPin, uint8_t lpwmPin, int16_t pwm, bool invert) {
    if (invert) pwm = -pwm;
    if (pwm >  PWM_MAX) pwm = PWM_MAX;
    if (pwm <  PWM_MIN) pwm = PWM_MIN;

    if (pwm > 0) {
        uint8_t v = (pwm < MOTOR_DEADBAND) ? 0 : (uint8_t)pwm;
        analogWrite(rpwmPin, v);
        analogWrite(lpwmPin, 0);
    } else if (pwm < 0) {
        uint8_t mag = (uint8_t)(-pwm);
        uint8_t v   = (mag < MOTOR_DEADBAND) ? 0 : mag;
        analogWrite(rpwmPin, 0);
        analogWrite(lpwmPin, v);
    } else {
        analogWrite(rpwmPin, 0);
        analogWrite(lpwmPin, 0);
    }
}

void motorsZeroPWM() {
    analogWrite(RPWM_L, 0);
    analogWrite(LPWM_L, 0);
    analogWrite(RPWM_R, 0);
    analogWrite(LPWM_R, 0);
}


// =============================================================================
//   CONTROL
// =============================================================================

int16_t computeFeedForward(int16_t target) {
    long pwm = ((long)target * PWM_MAX) / MAX_TICKS_PER_LOOP;
    if (pwm >  PWM_MAX) pwm =  PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;
    return (int16_t)pwm;
}

int16_t applyStictionKick(int16_t basePWM, PIDState &pid) {
    if (pid.kickLoopsRemaining == 0) return basePWM;
    pid.kickLoopsRemaining--;
    if (basePWM > 0) return max(basePWM, STICTION_KICK_PWM_MIN);
    if (basePWM < 0) return min(basePWM, (int16_t)-STICTION_KICK_PWM_MIN);
    return 0;
}

// deltaTicks is the MEASURED ticks-this-cycle for this wheel. It is used to
// decide whether the wheel is actually stalled so the break-free PWM floor is
// applied ONLY at standstill, never while the wheel is already turning.
int16_t computePID(int16_t target, long deltaTicks, PIDState &pid, int16_t sideBias) {
    if (target == 0) {
        pidStateReset(pid);
        return 0;
    }

    // Clamp tiny nonzero targets up to the slowest speed the motor can actually
    // hold. Below MIN_MOVE_TICKS the motor cannot turn steadily (see calibration),
    // so asking for less just stalls; we run at the minimum instead.
    if (target > 0 && target < MIN_MOVE_TICKS)  target =  MIN_MOVE_TICKS;
    if (target < 0 && target > -MIN_MOVE_TICKS) target = -MIN_MOVE_TICKS;

    // Direction reversal -> clear history to avoid integral fighting the change.
    if ((target > 0 && pid.lastTarget < 0) ||
        (target < 0 && pid.lastTarget > 0)) {
        pid.integral   = 0.0f;
        pid.prevError  = 0.0f;
    }
    pid.lastTarget = target;

    const int16_t feedforward = computeFeedForward(target);
    pid.ffPWM = feedforward;

    const float error = (float)target - (float)deltaTicks;
    const float pTerm = pidKp * error;

    // Provisional integral; committed only if output is not saturated (anti-windup).
    const float integralCandidate = pid.integral + error;
    float clampedIntegral = integralCandidate;
    if (clampedIntegral >  pidIMax) clampedIntegral =  pidIMax;
    if (clampedIntegral < -pidIMax) clampedIntegral = -pidIMax;
    const float iTerm = pidKi * clampedIntegral;

    const float dTerm = pidKd * (error - pid.prevError);
    pid.prevError = error;

    int16_t correction = (int16_t)(pTerm + iTerm + dTerm);
    if (correction >  PID_CORRECTION_MAX) correction =  PID_CORRECTION_MAX;
    if (correction < -PID_CORRECTION_MAX) correction = -PID_CORRECTION_MAX;

    int16_t pwm = feedforward + correction;
    if      (pwm > 0) pwm += sideBias;
    else if (pwm < 0) pwm -= sideBias;

    pwm = applyStictionKick(pwm, pid);

    // --- MIN-MOVE FLOOR (stall-gated) -------------------------------------
    // The break-free PWM floor (MOTOR_MIN_MOVE_PWM) exists only to overcome
    // static friction from a DEAD STOP. Once the wheel is already turning it
    // holds far below that PWM, so forcing the floor on a moving wheel makes
    // the PID surge to >=35 PWM then coast -- the source of the low-speed
    // jerking. Apply the floor ONLY when the wheel is essentially stalled.
    const bool wheelStalled =
        (deltaTicks > -WHEEL_STALL_TICKS && deltaTicks < WHEEL_STALL_TICKS);
    if (wheelStalled) {
        if (pwm > 0 && pwm <  MOTOR_MIN_MOVE_PWM) pwm =  MOTOR_MIN_MOVE_PWM;
        if (pwm < 0 && pwm > -MOTOR_MIN_MOVE_PWM) pwm = -MOTOR_MIN_MOVE_PWM;
    }

    const bool saturated = (pwm >= PWM_MAX) || (pwm <= PWM_MIN);

    // Commit integral only if not saturated, or if the new error pushes the
    // integral back toward zero (lets it recover from windup).
    if (!saturated || (error * pid.integral < 0.0f)) {
        pid.integral = clampedIntegral;
    }

    if (pwm >  PWM_MAX) pwm =  PWM_MAX;
    if (pwm <  PWM_MIN) pwm =  PWM_MIN;

    pid.pwmOutput = pwm;
    return pwm;
}

void stopMotors() {
    targetLeftCPL  = 0;
    targetRightCPL = 0;
    rampedLeftCPL  = 0;
    rampedRightCPL = 0;
    pidStateReset(pidLeft);
    pidStateReset(pidRight);
    resetSpeedAvg();
    rawPWMMode        = false;
    pidNeedsResync    = true;
    tickStreamEnabled = false;
    motorsZeroPWM();
    Led::showDirection(0, 0);
}

void brakeMotors() {
    targetLeftCPL  = 0;
    targetRightCPL = 0;
    rampedLeftCPL  = 0;
    rampedRightCPL = 0;
    pidStateReset(pidLeft);
    pidStateReset(pidRight);
    resetSpeedAvg();
    rawPWMMode        = false;
    pidNeedsResync    = true;
    tickStreamEnabled = false;

    Led::onRobotStateChange(RobotState::BRAKING);
    analogWrite(RPWM_L, 255);
    analogWrite(LPWM_L, 255);
    analogWrite(RPWM_R, 255);
    analogWrite(LPWM_R, 255);
    delay(200);
    motorsZeroPWM();
    Led::showDirection(0, 0);
}

void resyncEncoderBaseline() {
    noInterrupts();
    prevEncoderLeft  = encoderLeftCount;
    prevEncoderRight = encoderRightCount;
    interrupts();
    lastMotorUpdate = millis();
    pidNeedsResync  = false;
}

void updateMotors() {
    if (rawPWMMode) return;

    if (pidNeedsResync) {
        resyncEncoderBaseline();
        // Coming from a standstill: seed the ramp at the current speed (0) so
        // the kick + feedforward break free smoothly.
        int16_t ffL = computeFeedForward(rampedLeftCPL);
        int16_t ffR = computeFeedForward(rampedRightCPL);
        pidLeft.ffPWM  = ffL;
        pidRight.ffPWM = ffR;

        if      (ffL > 0) ffL += leftBias;
        else if (ffL < 0) ffL -= leftBias;
        if      (ffR > 0) ffR += rightBias;
        else if (ffR < 0) ffR -= rightBias;

        int16_t kickL = applyStictionKick(ffL, pidLeft);
        int16_t kickR = applyStictionKick(ffR, pidRight);
        pidLeft.pwmOutput  = kickL;
        pidRight.pwmOutput = kickR;

        applyPWM(RPWM_L, LPWM_L, kickL, INVERT_LEFT_MOTOR);
        applyPWM(RPWM_R, LPWM_R, kickR, INVERT_RIGHT_MOTOR);
        return;
    }

    if (millis() - lastMotorUpdate < MOTOR_UPDATE_MS) return;
    lastMotorUpdate = millis();

    // --- SLEW-LIMIT the effective target toward the commanded target -------
    // Makes big jumps (e.g. m 5 5 -> m 35 35) ramp over a few control cycles
    // instead of stepping instantly, removing the jerk/noise on a
    // moving->moving speed change.
    rampedLeftCPL  += constrain(targetLeftCPL  - rampedLeftCPL,
                                (int16_t)-TARGET_SLEW_STEP, TARGET_SLEW_STEP);
    rampedRightCPL += constrain(targetRightCPL - rampedRightCPL,
                                (int16_t)-TARGET_SLEW_STEP, TARGET_SLEW_STEP);

    long curL, curR;
    noInterrupts();
    curL = encoderLeftCount;
    curR = encoderRightCount;
    interrupts();

    const long deltaLeft  = curL - prevEncoderLeft;
    const long deltaRight = curR - prevEncoderRight;
    prevEncoderLeft  = curL;
    prevEncoderRight = curR;

    // --- SPEED ESTIMATE: average the raw delta over a short window for the
    // PID only. Telemetry/odometry below still use the RAW deltas + totals.
    const long avgDeltaLeft  = pushAndAverage(deltaBufLeft,  deltaLeft);
    const long avgDeltaRight = pushAndAverage(deltaBufRight, deltaRight);
    deltaBufIdx++;

    const int16_t pwmL = computePID(rampedLeftCPL,  avgDeltaLeft,  pidLeft,  leftBias);
    const int16_t pwmR = computePID(rampedRightCPL, avgDeltaRight, pidRight, rightBias);

    applyPWM(RPWM_L, LPWM_L, pwmL, INVERT_LEFT_MOTOR);
    applyPWM(RPWM_R, LPWM_R, pwmR, INVERT_RIGHT_MOTOR);

    // Machine telemetry: raw per-cycle deltas + EXACT running totals.
    // Format: e <Ldelta> <Rdelta> <Ltotal> <Rtotal>   (only while moving)
    // ROS odometry should integrate the TOTALS (curL/curR), not the deltas.
    if (tickStreamEnabled) {
        Serial.print(F("e "));
        Serial.print(deltaLeft);
        Serial.print(F(" "));
        Serial.print(deltaRight);
        Serial.print(F(" "));
        Serial.print(curL);
        Serial.print(F(" "));
        Serial.println(curR);
    }
}


// =============================================================================
//   CALIBRATION  (diagnostic only; not used in normal ROS operation)
//   This routine is run manually for tuning and is never triggered during
//   autonomous operation, so its output is harmless to the ROS bridge.
//   Lines are emitted in machine form: "cal <pwm> <Lticks> <Rticks>".
// =============================================================================

void runCalibration() {
    rawPWMMode = true;
    motorsEnable();

    long lt = 0;
    long rt = 0;

    const int16_t testPWMs[] = {30, 40, 50, 60, 70, 80, 100, 120, 150, 200, 255};
    for (uint8_t i = 0; i < sizeof(testPWMs) / sizeof(testPWMs[0]); i++) {
        const int16_t p = testPWMs[i];

        noInterrupts();
        encoderLeftCount  = 0;
        encoderRightCount = 0;
        interrupts();

        applyPWM(RPWM_L, LPWM_L, p, INVERT_LEFT_MOTOR);
        applyPWM(RPWM_R, LPWM_R, p, INVERT_RIGHT_MOTOR);
        delay(500);

        noInterrupts();
        encoderLeftCount  = 0;
        encoderRightCount = 0;
        interrupts();

        delay(1000);

        noInterrupts();
        lt = encoderLeftCount;
        rt = encoderRightCount;
        interrupts();
        motorsZeroPWM();

        Serial.print(F("cal "));
        Serial.print(p);
        Serial.print(F(" "));
        Serial.print(lt);
        Serial.print(F(" "));
        Serial.println(rt);
        delay(500);
    }

    stopMotors();
}


// =============================================================================
//   TELEMETRY & SENSORS
// =============================================================================

void resetEncoders() {
    noInterrupts();
    encoderLeftCount  = 0;
    encoderRightCount = 0;
    interrupts();
    prevEncoderLeft  = 0;
    prevEncoderRight = 0;
    pidNeedsResync   = true;
}

// Reply to the 'e' command: two-field exact totals.
void sendEncoderTotals() {
    long lt, rt;
    noInterrupts();
    lt = encoderLeftCount;
    rt = encoderRightCount;
    interrupts();
    Serial.print(F("e "));
    Serial.print(lt);
    Serial.print(F(" "));
    Serial.println(rt);
}

void readGPS() {
    if (!gpsAvailable) return;
    if (!gps.getPVT()) return;
    gpsData.lat  = gps.getLatitude()  / 10000000.0;
    gpsData.lon  = gps.getLongitude() / 10000000.0;
    gpsData.alt  = gps.getAltitude()  / 1000.0;
    gpsData.sats = gps.getSIV();
    gpsData.fix  = gps.getFixType();
}

void sendGPSData() {
    Serial.print(F("g "));
    Serial.print(gpsData.fix);
    Serial.print(F(" "));
    Serial.print(gpsData.lat, 7);
    Serial.print(F(" "));
    Serial.print(gpsData.lon, 7);
    Serial.print(F(" "));
    Serial.println(gpsData.alt, 2);
}

void parseUltraSensor(HardwareSerial &ser, uint8_t *buf, uint8_t &idx, int16_t &dist) {
    while (ser.available()) {
        const uint8_t b = ser.read();
        if (idx == 0) {
            if (b == 0xFF) { buf[0] = b; idx = 1; }
        } else {
            buf[idx++] = b;
            if (idx >= 4) {
                const uint8_t checksum = (buf[0] + buf[1] + buf[2]) & 0xFF;
                if (checksum == buf[3]) {
                    const int16_t d = ((int16_t)buf[1] << 8) | buf[2];
                    if (d >= ULTRA_MIN_MM && d <= ULTRA_MAX_MM) dist = d;
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
    Serial.print(F("u "));
    Serial.print(ultraLeftDist);
    Serial.print(F(" "));
    Serial.println(ultraRightDist);
}


// =============================================================================
//   DELIVERY LOCK
// =============================================================================
//
// Hardware: 12V solenoid lock -> Solid State Relay -> 12V rail
//           SSR input driven by LOCK_PIN (active-HIGH, see LOCK_ACTIVE_LOW)
//           Flyback diode across lock terminals (1N4007)
//
// State model:
//   LOCKED   = SSR off, latch extended (default / safe state)
//   UNLOCKED = SSR on,  latch retracted (solenoid energized)
//
// ROS status format:
//   lock <state:0|1> <ms_in_state>
// =============================================================================

static inline void lockWritePin(bool energize) {
    if (LOCK_ACTIVE_LOW) {
        digitalWrite(LOCK_PIN, energize ? LOW : HIGH);
    } else {
        digitalWrite(LOCK_PIN, energize ? HIGH : LOW);
    }
}

void lockInit() {
    pinMode(LOCK_PIN, OUTPUT);
    lockWritePin(false);                 // start in locked state
    lockState           = LockState::LOCKED;
    lockStateSince      = millis();
    lockAutoRelockArmed = false;
    lockAutoRelockAt    = 0;
}

void lockPrintStatus() {
    const uint32_t now     = millis();
    const uint32_t elapsed = now - lockStateSince;
    Serial.print(F("lock "));
    Serial.print((uint8_t)lockState);
    Serial.print(F(" "));
    Serial.println(elapsed);
}

void lockSetLocked() {
    lockWritePin(false);
    if (lockState != LockState::LOCKED) {
        lockState      = LockState::LOCKED;
        lockStateSince = millis();
    }
    lockAutoRelockArmed = false;
    lockAutoRelockAt    = 0;
    lockPrintStatus();
}

void lockSetUnlocked(uint32_t holdMs) {
    lockWritePin(true);
    if (lockState != LockState::UNLOCKED) {
        lockState      = LockState::UNLOCKED;
        lockStateSince = millis();
    }

    uint32_t effectiveHold = holdMs;
    if (effectiveHold == 0 || effectiveHold > LOCK_MAX_HOLD_MS) {
        effectiveHold = LOCK_MAX_HOLD_MS;
    }
    lockAutoRelockAt    = millis() + effectiveHold;
    lockAutoRelockArmed = true;

    lockPrintStatus();
}

// Non-blocking update — call every loop
void lockUpdate() {
    if (!lockAutoRelockArmed) return;
    if ((int32_t)(millis() - lockAutoRelockAt) >= 0) {
        lockWritePin(false);
        lockState           = LockState::LOCKED;
        lockStateSince      = millis();
        lockAutoRelockArmed = false;
        lockAutoRelockAt    = 0;
        lockPrintStatus();                // notify ROS on auto re-lock
    }
}


// =============================================================================
//   ZIEGLER-NICHOLS TUNING  (diagnostic only)
// =============================================================================

void startZNFindTest(float kp) {
    pidKp = kp;
    pidKi = 0.0f;
    pidKd = 0.0f;

    pidStateReset(pidLeft);
    pidStateReset(pidRight);
    resetSpeedAvg();

    targetLeftCPL  = 30;
    targetRightCPL = 30;
    rampedLeftCPL  = 30;   // ZN test wants the step applied directly, no slew
    rampedRightCPL = 30;
    pidLeft.kickLoopsRemaining  = STICTION_KICK_LOOPS;
    pidRight.kickLoopsRemaining = STICTION_KICK_LOOPS;

    rawPWMMode        = false;
    pidNeedsResync    = true;
    tickStreamEnabled = true;

    motorsEnable();
    Led::showDirection(30, 30);
    // No human text: watch the 'e' stream for oscillation, then send 'a Ku Tu'.
}

void applyZNGains(float ku, float tu, uint8_t mode) {
    float kp, ki, kd;

    switch (mode) {
        case 0:
            kp = 0.6f * ku;
            ki = 2.0f * kp / tu;
            kd = kp * tu / 8.0f;
            break;
        case 1:
            kp = 0.33f * ku;
            ki = 2.0f * kp / tu;
            kd = kp * tu / 3.0f;
            break;
        case 2:
        default:
            kp = 0.2f * ku;
            ki = 2.0f * kp / tu;
            kd = kp * tu / 3.0f;
            break;
    }

    const float dt = MOTOR_UPDATE_MS / 1000.0f;
    pidKp = kp;
    pidKi = ki * dt;
    pidKd = kd / dt;

    pidStateReset(pidLeft);
    pidStateReset(pidRight);
    resetSpeedAvg();
    pidNeedsResync = true;
}


// =============================================================================
//   COMMAND DISPATCH
//   Command handling is unchanged; only the human-readable Serial.print
//   responses have been removed. Machine telemetry replies (e/g/u/lock) are
//   kept because ROS depends on them.
// =============================================================================

void processCommand(char *cmd) {
    const char c    = cmd[0];
    const char *arg = cmd + 1;

    switch (c) {
        case CMD_MOTOR: {
            int L = 0, R = 0;
            if (sscanf(arg, "%d %d", &L, &R) == 2) {
                L = constrain(L, -MAX_COUNTS_PER_LOOP, MAX_COUNTS_PER_LOOP);
                R = constrain(R, -MAX_COUNTS_PER_LOOP, MAX_COUNTS_PER_LOOP);

                // Only do "cold start" work (stiction kick + encoder resync +
                // PID state wipe) when a side is transitioning FROM A STANDSTILL.
                // While already moving, keep PID history and let the slew limiter
                // ramp the change smoothly.
                const bool leftWasStopped  = (targetLeftCPL  == 0);
                const bool rightWasStopped = (targetRightCPL == 0);

                if (L != 0 && leftWasStopped)  pidLeft.kickLoopsRemaining  = STICTION_KICK_LOOPS;
                if (R != 0 && rightWasStopped) pidRight.kickLoopsRemaining = STICTION_KICK_LOOPS;

                if (L == 0) pidStateReset(pidLeft);
                if (R == 0) pidStateReset(pidRight);

                const bool wasFullyStopped =
                    (targetLeftCPL == 0 && targetRightCPL == 0);

                targetLeftCPL  = (int16_t)L;
                targetRightCPL = (int16_t)R;
                rawPWMMode        = false;
                if (wasFullyStopped) {
                    pidNeedsResync = true;
                    rampedLeftCPL  = 0;
                    rampedRightCPL = 0;
                    resetSpeedAvg();   // start the speed estimate clean
                }
                tickStreamEnabled = (L != 0 || R != 0);

                motorsEnable();
                Led::showDirection(targetLeftCPL, targetRightCPL);
            }
            // No usage/echo text on bad parse — keep the line silent for ROS.
            break;
        }

        case CMD_RAW_PWM: {
            int L = 0, R = 0;
            if (sscanf(arg, "%d %d", &L, &R) == 2) {
                L = constrain(L, -PWM_MAX, PWM_MAX);
                R = constrain(R, -PWM_MAX, PWM_MAX);

                rawPWMMode        = true;
                targetLeftCPL     = 0;
                targetRightCPL    = 0;
                rampedLeftCPL     = 0;
                rampedRightCPL    = 0;
                pidNeedsResync    = true;
                tickStreamEnabled = false;
                pidStateReset(pidLeft);
                pidStateReset(pidRight);
                resetSpeedAvg();
                motorsEnable();
                applyPWM(RPWM_L, LPWM_L, (int16_t)L, INVERT_LEFT_MOTOR);
                applyPWM(RPWM_R, LPWM_R, (int16_t)R, INVERT_RIGHT_MOTOR);
            }
            break;
        }

        case CMD_STOP:      stopMotors();    break;
        case CMD_BRAKE:     brakeMotors();   break;
        case CMD_CALIBRATE: runCalibration(); break;

        case CMD_TUNE: {
            float kp = 0, ki = 0, kd = 0;
            if (sscanf(arg, "%f %f %f", &kp, &ki, &kd) == 3) {
                pidKp = kp; pidKi = ki; pidKd = kd;
                pidStateReset(pidLeft);
                pidStateReset(pidRight);
                pidNeedsResync = true;
            }
            break;
        }

        case CMD_BIAS: {
            int lb = 0, rb = 0;
            if (sscanf(arg, "%d %d", &lb, &rb) == 2) {
                leftBias  = (int16_t)constrain(lb, -100, 100);
                rightBias = (int16_t)constrain(rb, -100, 100);
            }
            break;
        }

        case CMD_ZN_FIND: {
            float kp = 0;
            if (sscanf(arg, "%f", &kp) == 1 && kp > 0) startZNFindTest(kp);
            break;
        }

        case CMD_ZN_APPLY: {
            float ku = 0, tu = 0;
            int mode = 2;
            const int parsed = sscanf(arg, "%f %f %d", &ku, &tu, &mode);
            if (parsed >= 2 && ku > 0 && tu > 0) applyZNGains(ku, tu, (uint8_t)mode);
            break;
        }

        case CMD_ENCODER:   sendEncoderTotals();   break;   // -> "e <Lt> <Rt>"
        case CMD_ENC_RESET: resetEncoders();       break;
        case CMD_GPS:       sendGPSData();         break;   // -> "g ..."
        case CMD_ULTRA:     sendUltrasonicData();  break;   // -> "u <L> <R>"
        case CMD_STATUS:    /* status dump removed for ROS cleanliness */ break;

        case CMD_LED_MODE: {
            int mode = 0;
            if (sscanf(arg, "%d", &mode) == 1) {
                Led::setMode(static_cast<LedMode>(mode));
            }
            break;
        }

        case CMD_LED_COLOR: {
            int r = 0, g = 0, b = 0;
            if (sscanf(arg, "%d %d %d", &r, &g, &b) == 3) {
                Led::setColor((uint8_t)r, (uint8_t)g, (uint8_t)b);
            }
            break;
        }

        case CMD_LED_BRIGHT: {
            int n = 0;
            if (sscanf(arg, "%d", &n) == 1) {
                Led::setBrightness((uint8_t)constrain(n, 0, 255));
            }
            break;
        }

        case CMD_LOCK: {
            long n = -1;
            const int parsed = sscanf(arg, "%ld", &n);

            if (parsed != 1) {
                lockSetUnlocked(LOCK_PULSE_MS);   // bare 'o' -> pulse unlock
            } else if (n == 0) {
                lockSetLocked();
            } else if (n == 1) {
                lockSetUnlocked(0);               // hold (capped)
            } else if (n > 1) {
                lockSetUnlocked((uint32_t)n);
            }
            // Each branch already prints the machine "lock ..." status line.
            break;
        }

        case CMD_LOCK_STATUS:
            lockPrintStatus();                    // -> "lock <state> <ms>"
            break;

        case CMD_PING: Serial.println(F("PONG")); break;  // kept: ROS liveness check
        case CMD_HELP: /* help dump removed for ROS cleanliness */ break;

        default:
            // Unknown command: stay silent so ROS parser sees nothing unexpected.
            break;
    }
}


// =============================================================================
//   SETUP & LOOP
// =============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);

    pidStateReset(pidLeft);
    pidStateReset(pidRight);
    resetSpeedAvg();

    Led::init();
    Led::onRobotStateChange(RobotState::IDLE);

    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    if (!gps.begin()) {
        gpsAvailable = false;
    } else {
        gps.setI2COutput(COM_TYPE_UBX);
        gps.setNavigationFrequency(GPS_NAV_FREQ);
        gps.setAutoPVT(true);
        gpsAvailable = true;
    }

    Serial2.begin(ULTRA_BAUD);
    Serial3.begin(ULTRA_BAUD);

    pinMode(RPWM_L, OUTPUT);
    pinMode(LPWM_L, OUTPUT);
    pinMode(REN_L,  OUTPUT);
    pinMode(LEN_L,  OUTPUT);
    pinMode(RPWM_R, OUTPUT);
    pinMode(LPWM_R, OUTPUT);
    pinMode(REN_R,  OUTPUT);
    pinMode(LEN_R,  OUTPUT);

    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_L_A), encoderLeftA_ISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_L_B), encoderLeftB_ISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), encoderRightA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_B), encoderRightB_ISR, CHANGE);

    lockInit();

    stopMotors();
    motorsEnable();

    // Single machine-readable ready marker (optional; harmless to ROS parsers
    // that key on the first token). Remove if your parser dislikes it.
    Serial.println(F("ready"));
}

void loop() {
    while (Serial.available()) {
        const char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0';
                processCommand(cmdBuffer);
                cmdIndex = 0;
            }
        } else if (cmdIndex < sizeof(cmdBuffer) - 1) {
            cmdBuffer[cmdIndex++] = c;
        } else {
            cmdIndex = 0;
        }
    }
    updateMotors();
    readGPS();
    readUltrasonic();
    lockUpdate();
    Led::update();
}
