#include "led.h"
#include "motorParams.h"
#include <FastLED.h>

// =============================================================================
//   ARAP - LED animation implementations
// =============================================================================

static CRGB     leds[NUM_LEDS];
static LedMode  currentMode = LedMode::IDLE;
static CRGB     customColor = CRGB::Blue;
static uint8_t  animStep    = 0;
static uint32_t lastUpdate  = 0;

// ---------- Private helpers (file-scope) -----------------------------------
namespace {

    CRGB colorWheel(uint8_t pos) {
        pos = 255 - pos;
        if (pos < 85) {
            return CRGB(255 - pos * 3, 0, pos * 3);
        }
        if (pos < 170) {
            pos -= 85;
            return CRGB(0, pos * 3, 255 - pos * 3);
        }
        pos -= 170;
        return CRGB(pos * 3, 255 - pos * 3, 0);
    }

    uint8_t smoothPulse(uint8_t step) {
        return (sin8(step) * 200 / 255) + 55;
    }

    void animateIdle() {
        uint8_t brightness = smoothPulse(animStep);
        fill_solid(leds, NUM_LEDS, CRGB(0, 0, brightness));
        animStep += 2;
    }

    void animateRainbow() {
        for (uint8_t i = 0; i < NUM_LEDS; i++) {
            leds[i] = colorWheel((i * 256 / NUM_LEDS + animStep) & 255);
        }
        animStep += 4;
    }

    void animateBtConnected() {
        uint8_t brightness = smoothPulse(animStep);
        fill_solid(leds, NUM_LEDS, CRGB(0, brightness, 0));
        animStep += 3;
    }

    void animateBtActive() {
        for (uint8_t i = 0; i < NUM_LEDS; i++) {
            uint8_t brightness = sin8((i * 32) + animStep);
            leds[i] = CRGB(0, brightness, brightness);
        }
        animStep += 5;
    }

    void animateForward() {
        fadeToBlackBy(leds, NUM_LEDS, 60);
        uint8_t pos = (animStep / 3) % NUM_LEDS;
        leds[pos] = CRGB::Green;
        leds[(pos + 1) % NUM_LEDS] = CRGB(0, 150, 0);
        animStep++;
    }

    void animateReverse() {
        fadeToBlackBy(leds, NUM_LEDS, 60);
        uint8_t pos = NUM_LEDS - 1 - ((animStep / 3) % NUM_LEDS);
        leds[pos] = CRGB::Red;
        leds[(pos + NUM_LEDS - 1) % NUM_LEDS] = CRGB(150, 0, 0);
        animStep++;
    }

    void animateTurnLeft() {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        for (uint8_t i = 0; i < NUM_LEDS / 2; i++) {
            uint8_t brightness = sin8(animStep + i * 20);
            leds[i] = CRGB(brightness, brightness / 2, 0);
        }
        animStep += 8;
    }

    void animateTurnRight() {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        for (uint8_t i = NUM_LEDS / 2; i < NUM_LEDS; i++) {
            uint8_t brightness = sin8(animStep + i * 20);
            leds[i] = CRGB(brightness, brightness / 2, 0);
        }
        animStep += 8;
    }

    void animateChase() {
        fadeToBlackBy(leds, NUM_LEDS, 50);
        uint8_t pos = (animStep / 4) % NUM_LEDS;
        leds[pos] = colorWheel(animStep * 2);
        animStep++;
    }

    void animatePulse() {
        uint8_t brightness = smoothPulse(animStep);
        CRGB color = colorWheel(animStep / 2);
        color.nscale8(brightness);
        fill_solid(leds, NUM_LEDS, color);
        animStep += 3;
    }

    void animateFire() {
        static uint8_t heat[NUM_LEDS];
        for (uint8_t i = 0; i < NUM_LEDS; i++) {
            uint8_t cooldown = random8(0, ((55 * 10) / NUM_LEDS) + 2);
            heat[i] = (cooldown > heat[i]) ? 0 : heat[i] - cooldown;
        }
        // NOTE: counter must be signed. With uint8_t this loop never ends
        // because (i >= 2) stays true after i underflows past 0 to 255.
        for (int16_t i = NUM_LEDS - 1; i >= 2; i--) {
            heat[i] = (heat[i - 1] + heat[i - 2] + heat[i - 2]) / 3;
        }
        if (random8() < 160) {
            uint8_t y = random8(3);
            heat[y] = qadd8(heat[y], random8(160, 255));
        }
        for (uint8_t i = 0; i < NUM_LEDS; i++) {
            uint8_t h = heat[i];
            uint8_t heatramp = (h & 0x3F) << 2;
            if      (h > 0xC0) leds[i] = CRGB(255, 255, heatramp);
            else if (h > 0x60) leds[i] = CRGB(255, heatramp, 0);
            else               leds[i] = CRGB(heatramp, 0, 0);
        }
        animStep++;
    }

} // anonymous namespace

// ---------- Public API -----------------------------------------------------
namespace Led {

    void init() {
        FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
        FastLED.setBrightness(LED_BRIGHTNESS);
        // FastLED.setMaxPowerInVoltsAndMilliamps(5, 6600);  // 110 LEDs full budget
        clear();
        currentMode = LedMode::IDLE;
    }

    void update() {
        uint32_t now = millis();
        if (now - lastUpdate < LED_UPDATE_MS) return;
        lastUpdate = now;

        switch (currentMode) {
            case LedMode::OFF:          FastLED.clear(); FastLED.show(); return;
            case LedMode::IDLE:         animateIdle();         break;
            case LedMode::RAINBOW:      animateRainbow();      break;
            case LedMode::CHASE:        animateChase();        break;
            case LedMode::PULSE:        animatePulse();        break;
            case LedMode::FIRE:         animateFire();         break;
            case LedMode::BT_CONNECTED: animateBtConnected();  break;
            case LedMode::BT_ACTIVE:    animateBtActive();     break;
            case LedMode::FORWARD_IND:  animateForward();      break;
            case LedMode::REVERSE_IND:  animateReverse();      break;
            case LedMode::TURN_LEFT:    animateTurnLeft();     break;
            case LedMode::TURN_RIGHT:   animateTurnRight();    break;
            case LedMode::CUSTOM:       fill_solid(leds, NUM_LEDS, customColor); break;
            default:                    animateIdle();         break;
        }
        FastLED.show();
    }

    void setMode(LedMode mode) {
        if (static_cast<uint8_t>(mode) >= static_cast<uint8_t>(LedMode::MODE_COUNT)) return;
        if (currentMode == mode) return;
        currentMode = mode;
        animStep = 0;
    }

    LedMode getMode() { return currentMode; }

    void setColor(uint8_t r, uint8_t g, uint8_t b) {
        customColor = CRGB(r, g, b);
        currentMode = LedMode::CUSTOM;
    }

    void setBrightness(uint8_t brightness) {
        FastLED.setBrightness(brightness);
    }

    void clear() {
        FastLED.clear();
        FastLED.show();
    }

    void onRobotStateChange(RobotState state) {
        switch (state) {
            case RobotState::IDLE:            setMode(LedMode::IDLE);        break;
            case RobotState::MOVING_FORWARD:  setMode(LedMode::FORWARD_IND); break;
            case RobotState::MOVING_BACKWARD: setMode(LedMode::REVERSE_IND); break;
            case RobotState::TURNING_LEFT:    setMode(LedMode::TURN_LEFT);   break;
            case RobotState::TURNING_RIGHT:   setMode(LedMode::TURN_RIGHT);  break;
            case RobotState::SPINNING:        setMode(LedMode::RAINBOW);     break;
            case RobotState::BRAKING:         setMode(LedMode::PULSE);       break;
            case RobotState::ERROR:           setColor(255, 255, 255);       break;
        }
    }

    void showDirection(int16_t left, int16_t right) {
        if (left == 0 && right == 0) {
            setMode(LedMode::IDLE);
        } else {
            setMode(LedMode::RAINBOW);
        }
    }

} // namespace Led
