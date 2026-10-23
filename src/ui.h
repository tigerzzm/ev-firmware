#pragma once
#include <cstdint>
#include "config.h"
//==============================================================================
// ui — LCD + rotary dial + Set/Start buttons.
// Ported from the existing Arduino EV kit (code_ref/main_code): the dial+Set
// entry flow with increment cycling, translated to the Pico (gpio_get instead of
// digitalRead; lcd.* instead of LiquidCrystal_I2C). The #2-pencil trigger reads
// on START_BTN_PIN.
//==============================================================================

void  uiInit();
void  uiShow(const char *line1, const char *line2);

bool  uiStartPressed();       // pencil trigger (START_BTN_PIN, active-low)
bool  uiSetPressed();

// Blocking value entry (dial to change, DialBtn cycles increment, SET confirms).
// Each starts from `current` so run 2 begins at what run 1 used, and clamps to
// the rules range in config.h §5.
float uiEnterDistanceM(float current);     // Target Distance D, [5.00, 9.00]
float uiEnterBottleLineM(float current);   // Bottle Line Distance B, [1.00, 3.00]
float uiEnterTimeS(float current);         // Target Time T, [10.0, 20.0]
int   uiSelectLevel(int current);          // 0 regional 2.0 m, 1 state, 2 national

// Run-2 correction, metres, [STOP_OFFSET_MIN_M, STOP_OFFSET_MAX_M]. `label` is the
// LCD prefix ("MPofs:" or "BtlOfs:"); see config.h §5 for the sign conventions.
float uiEnterOffsetM(const char *label, float current);

// Two full 16-character lines WITHOUT lcdClear() and WITHOUT printf — for live
// screens. Shorter strings are space-padded so nothing stale is left behind. An
// LCD write is ~30 ms of blocking I2C: never call this while the car is moving.
void  uiShowLive(const char *line1, const char *line2);

// Bench calibration: drive PWM percent, clamped to [CAL_PWM_MIN, CAL_PWM_MAX].
float uiEnterPwmPercent(float current);

// Raw servo pulse width, us. Clamped to [SERVO_MIN_US, SERVO_MAX_US].
int uiEnterServoUs(int current);

// Dial menu: rotate to highlight, SET to choose. Returns the chosen index.
int uiMenuSelect(const char *title, const char *const *items, int n, int startIdx = 0);

// --- dial primitives, for building live/interactive bench loops --------------
// One detent since the last call: -1 / 0 / +1. Keeps its own edge state.
int  uiRotaryStep();
void uiRotaryResync();     // call when a screen starts using uiRotaryStep()
// Wait for SET to be released, then let the contacts settle, so the bounce can't
// read as a new press on whatever screen comes next.
void uiWaitSetRelease();
bool uiDialPressed();      // KY-040 push, active-low

// A button that only counts once held for BUTTON_QUALIFY_MS. The inputs are raw
// GPIOs with weak internal pull-ups next to a 550 motor and a servo line; one
// noise sample must not abort a run on the track (or start one).
struct HeldButton {
  uint32_t since = 0;               // when the current press began, 0 = released
  bool update(bool down, uint32_t ms) {
    if (!down) { since = 0; return false; }
    if (since == 0) since = ms ? ms : 1;
    return ms - since >= BUTTON_QUALIFY_MS;
  }
};
