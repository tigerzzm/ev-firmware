#include "ui.h"
#include "config.h"
#include "lcd.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <algorithm>
#include <cstdio>

//==============================================================================
// Ported from code_ref/main_code (Unphayzed EV kit). Arduino -> Pico changes:
//   digitalRead()->gpio_get(), internal pull-ups via gpio_pull_up(),
//   LiquidCrystal_I2C -> lcd.*. The kit polled CLK/DT low as +/- steps; here we
//   use a proper KY-040 falling-edge decode (cleaner, fewer double-counts).
// Buttons are active-low (pull-ups): pressed == gpio_get()==0.
//==============================================================================

// Dial direction lives in config.h §3 as DIAL_DIR_SIGN.

void uiInit() {
  const uint ins[] = {START_BTN_PIN, SET_BTN_PIN, DIAL_BTN_PIN, DIAL_CLK_PIN, DIAL_DT_PIN};
  for (uint p : ins) { gpio_init(p); gpio_set_dir(p, GPIO_IN); gpio_pull_up(p); }
  lcdInit();
  lcdClear();
}

bool uiStartPressed() { return !gpio_get(START_BTN_PIN); }
bool uiSetPressed()   { return !gpio_get(SET_BTN_PIN); }

void uiShow(const char *line1, const char *line2) {
  lcdClear();
  lcdPrintAt(0, 0, line1 ? line1 : "");
  lcdPrintAt(0, 1, line2 ? line2 : "");
  printf("[UI] %s | %s\n", line1 ? line1 : "", line2 ? line2 : "");  // serial mirror
}

// Fixed-width so the old text is always fully overwritten — that is what lets us
// skip lcdClear(). Deliberately does NOT printf: at 4 Hz that would be a steady
// stream into a USB CDC pipe that may have nothing draining it. (For real logging
// use ram_logger.* and dump after the run.)
void uiShowRun(float distM, float targetM, float speedMps, float elapsedSec) {
  // Must be padded to the full 16 columns: this deliberately skips lcdClear(),
  // so anything shorter leaves characters from the previous screen behind.
  char l0[17], l1[17];
  snprintf(l0, sizeof l0, "%-16s", "");
  snprintf(l1, sizeof l1, "%-16s", "");
  char a[17], b[17];
  snprintf(a, sizeof a, "%5.2f/%5.2f m", (double)distM, (double)targetM);
  snprintf(b, sizeof b, "v%4.2f  t%5.1fs", (double)speedMps, (double)elapsedSec);
  snprintf(l0, sizeof l0, "%-16s", a);
  snprintf(l1, sizeof l1, "%-16s", b);
  lcdPrintAt(0, 0, l0);
  lcdPrintAt(0, 1, l1);
}

// --- one rotary detent since last call: -1 / 0 / +1 -------------------------
static int rotaryStep(uint8_t &lastClk) {
  uint8_t clk = gpio_get(DIAL_CLK_PIN);
  int step = 0;
  if (lastClk == 1 && clk == 0) {                 // CLK falling edge = one detent
    step = gpio_get(DIAL_DT_PIN) ? +1 : -1;       // DT level gives direction
    step *= DIAL_DIR_SIGN;
  }
  lastClk = clk;
  return step;
}

// --- render the entry screen -------------------------------------------------
static void renderEntry(const char *label, const char *unit,
                        float value, float inc, int decimals) {
  char l0[17], l1[17];
  char valbuf[12];
  snprintf(valbuf, sizeof valbuf, "%.*f", decimals, value);
  snprintf(l0, sizeof l0, "%s%s%s", label, valbuf, unit);
  snprintf(l1, sizeof l1, "Inc: %.3f", inc);
  lcdClear();
  lcdPrintAt(0, 0, l0);
  lcdPrintAt(0, 1, l1);
  // Mirror to serial so the whole UI is usable headless — the LCD may not be
  // wired yet, and every other screen already prints.
  printf("[UI] %s   (%s)\n", l0, l1);
}

#if !USE_DIAL_ENTRY
// --- no-dial value entry -----------------------------------------------------
// SET steps the value (holding it auto-repeats and then accelerates) and wraps at
// the top so you can always get back down; START confirms. Uses the FINEST of the
// caller's increments, since without a dial there is no button free to cycle it —
// the auto-repeat is what makes a long traverse bearable.
static float enterValue(const char *label, const char *unit, float value,
                        float lo, float hi, const float *incs, int nIncs,
                        int decimals) {
  float inc = incs[0];
  for (int i = 1; i < nIncs; i++) if (incs[i] < inc) inc = incs[i];

  bool     lastSet   = gpio_get(SET_BTN_PIN);
  uint32_t heldSince = 0, lastRepeat = 0;
  renderEntry(label, unit, value, inc, decimals);

  while (true) {
    const bool     pressed = !gpio_get(SET_BTN_PIN);
    const uint32_t now     = to_ms_since_boot(get_absolute_time());
    bool bump = false;

    if (lastSet && pressed) {                  // press edge
      bump = true;
      heldSince = now; lastRepeat = now;
    } else if (pressed && (now - heldSince) > 500) {
      // accelerate: 150 ms/step at first, 40 ms/step after 2 s held
      const uint32_t period = (now - heldSince > 2000) ? 40 : 150;
      if (now - lastRepeat >= period) { bump = true; lastRepeat = now; }
    }
    lastSet = !pressed;

    if (bump) {
      value += inc;
      if (value > hi + inc * 0.5f) value = lo;  // wrap
      value = std::clamp(value, lo, hi);
      renderEntry(label, unit, value, inc, decimals);
    }

    if (uiStartPressed()) {                    // confirm
      sleep_ms(50);
      while (uiStartPressed()) sleep_ms(5);
      return value;
    }
    sleep_ms(2);
  }
}
#else
// --- generic blocking dial entry --------------------------------------------
static float enterValue(const char *label, const char *unit, float value,
                        float lo, float hi, const float *incs, int nIncs,
                        int decimals) {
  int incIdx = 0;
  uint8_t lastClk    = gpio_get(DIAL_CLK_PIN);
  bool    lastDialBt = gpio_get(DIAL_BTN_PIN);
  bool    lastSet    = gpio_get(SET_BTN_PIN);
  renderEntry(label, unit, value, incs[incIdx], decimals);

  while (true) {
    // rotary -> change value
    int s = rotaryStep(lastClk);
    if (s != 0) {
      value = std::clamp(value + s * incs[incIdx], lo, hi);
      renderEntry(label, unit, value, incs[incIdx], decimals);
      sleep_ms(2);
    }

    // DialBtn -> cycle increment (press edge, active-low)
    bool db = gpio_get(DIAL_BTN_PIN);
    if (lastDialBt && !db) {
      incIdx = (incIdx + 1) % nIncs;
      renderEntry(label, unit, value, incs[incIdx], decimals);
      sleep_ms(200);                              // debounce
    }
    lastDialBt = db;

    // Set -> confirm (press edge, active-low)
    bool st = gpio_get(SET_BTN_PIN);
    if (lastSet && !st) {
      sleep_ms(50);
      while (!gpio_get(SET_BTN_PIN)) sleep_ms(5); // wait for release
      return value;
    }
    lastSet = st;

    sleep_ms(2);
  }
}
#endif  // USE_DIAL_ENTRY

// --- public entry points -----------------------------------------------------
float uiEnterDistanceM() {
  static const float incs[] = {1.0f, 0.5f, 0.1f, 0.01f};
  return enterValue("Dist:", "m", 8.00f, TARGET_DIST_MIN_M, TARGET_DIST_MAX_M,
                    incs, 4, 2);
}

float uiEnterTimeS() {
  static const float incs[] = {1.0f, 0.5f, 0.1f};
  return enterValue("Time:", "s", 15.0f, TARGET_TIME_MIN_S, TARGET_TIME_MAX_S,
                    incs, 3, 1);
}

float uiEnterStopOffsetM(float current) {
  // mm-level resolution: scoring is 2 pt/cm, so 0.001 m steps are finer than the
  // scoring granularity and 0.01 m gets you there fast.
  static const float incs[] = {0.010f, 0.001f, 0.050f};
  return enterValue("Ofs:", "m", current, STOP_OFFSET_MIN_M, STOP_OFFSET_MAX_M,
                    incs, 3, 3);
}

// Public wrapper over the file-static decoder, carrying its own edge state so a
// caller can drive an interactive loop without owning the CLK history.
int uiRotaryStep() {
  // Seed from the ACTUAL pin on first use. Assuming "high" meant that if CLK
  // happened to be low at that moment, the first call saw a phantom falling edge
  // and emitted a spurious detent — a menu jumping one item, or the servo
  // stepping once, immediately on entering a screen.
  static bool    seeded  = false;
  static uint8_t lastClk = 1;
  if (!seeded) { lastClk = gpio_get(DIAL_CLK_PIN); seeded = true; }
  return rotaryStep(lastClk);
}

bool uiDialPressed() { return !gpio_get(DIAL_BTN_PIN); }

int uiMenuSelect(const char *title, const char *const *items, int n) {
  int  idx     = 0;
  bool lastSet = gpio_get(SET_BTN_PIN);
  auto render  = [&]() {
    lcdClear();
    lcdPrintAt(0, 0, title);
    lcdPrintAt(0, 1, items[idx]);
    printf("[UI] %s | %s\n", title, items[idx]);
  };
  render();
  while (true) {
#if USE_DIAL_ENTRY
    // Dial rotates the highlight, SET chooses.
    int s = uiRotaryStep();
    if (s != 0) { idx = (idx + s + n) % n; render(); sleep_ms(2); }
    bool st = gpio_get(SET_BTN_PIN);
    if (lastSet && !st) {                       // press edge
      sleep_ms(50);
      while (!gpio_get(SET_BTN_PIN)) sleep_ms(5);
      return idx;
    }
    lastSet = st;
#else
    // No dial: SET cycles the highlight, START chooses.
    bool st = gpio_get(SET_BTN_PIN);
    if (lastSet && !st) {                       // press edge
      idx = (idx + 1) % n;
      render();
      sleep_ms(200);
    }
    lastSet = st;
    if (uiStartPressed()) {
      sleep_ms(50);
      while (uiStartPressed()) sleep_ms(5);
      return idx;
    }
#endif
    sleep_ms(2);
  }
}

int uiEnterServoUs(int current) {
  static const float incs[] = {10.0f, 1.0f, 100.0f};
  float v = enterValue("Servo:", "us", (float)current,
                       (float)SERVO_MIN_US, (float)SERVO_MAX_US, incs, 3, 0);
  return (int)v;
}

float uiEnterPwmPercent(float current) {
  static const float incs[] = {5.0f, 1.0f, 10.0f};
  return enterValue("PWM:", "%", current, CAL_PWM_MIN, CAL_PWM_MAX, incs, 3, 0);
}

float uiEnterBulgeH() {
  // Preset from config (matches the kit's fixed arc height). Dial-tunable later
  // by swapping this for enterValue() over a bulge range.
  char l1[17];
  snprintf(l1, sizeof l1, "gate h=%.2fm", GATE_BULGE_H_M);
  uiShow("Arc preset", l1);
  sleep_ms(400);
  return GATE_BULGE_H_M;
}
