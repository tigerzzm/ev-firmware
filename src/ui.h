#pragma once
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

// Blocking dial entry (dial to change, DialBtn cycles increment, Set confirms).
float uiEnterDistanceM();     // clamped to [7.00, 10.00]
float uiEnterTimeS();         // clamped to [10.0, 20.0]
float uiEnterBulgeH();        // gate bulge h — preset from config (dial-tunable later)

// Live run telemetry. Writes both lines WITHOUT lcdClear() — no flicker, and it
// skips the 2 ms clear. Throttle the call site to LCD_RUN_UPDATE_MS; this is far
// too slow to call every control tick.
void uiShowRun(float distM, float targetM, float speedMps, float elapsedSec);

// Run-2 stop correction, metres. Starts from `current` so run 2 begins at the
// value run 1 used and you adjust from there. Positive = travel FURTHER.
float uiEnterStopOffsetM(float current);

// Bench calibration: drive PWM percent, clamped to [CAL_PWM_MIN, CAL_PWM_MAX].
float uiEnterPwmPercent(float current);

// Raw servo pulse width, us. Clamped to [SERVO_MIN_US, SERVO_MAX_US].
int uiEnterServoUs(int current);

// Dial menu: rotate to highlight, SET to choose. Returns the chosen index.
int uiMenuSelect(const char *title, const char *const *items, int n);

// --- dial primitives, for building live/interactive bench loops --------------
// One detent since the last call: -1 / 0 / +1. Keeps its own edge state.
int  uiRotaryStep();
bool uiDialPressed();      // KY-040 push, active-low
