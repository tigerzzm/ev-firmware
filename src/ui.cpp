#include "ui.h"
#include "config.h"
#include "lcd.h"
#include "plan.h"
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
// skip lcdClear(). Deliberately does NOT printf: a live screen redraws several
// times a second, and stdio over USB CDC can block when nothing drains it.
void uiShowLive(const char *line1, const char *line2) {
  char l0[17], l1[17];
  snprintf(l0, sizeof l0, "%-16s", line1 ? line1 : "");
  snprintf(l1, sizeof l1, "%-16s", line2 ? line2 : "");
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
      sleep_ms(BUTTON_RELEASE_SETTLE_MS);
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
      uiWaitSetRelease();
      return value;
    }
    lastSet = st;

    sleep_ms(2);
  }
}
#endif  // USE_DIAL_ENTRY

// --- public entry points -----------------------------------------------------
// Increments start at the regional interval (0.25 m), the coarsest legal step for
// D and B (§8.h, §8.j); DialBtn cycles to finer ones for state and nationals.
static constexpr int N_STEPS_M = sizeof ENTRY_STEPS_M / sizeof ENTRY_STEPS_M[0];
static constexpr int N_STEPS_S = sizeof ENTRY_STEPS_S / sizeof ENTRY_STEPS_S[0];

float uiEnterDistanceM(float current) {
  return enterValue("D:", "m", std::clamp(current, TARGET_DIST_MIN_M, TARGET_DIST_MAX_M),
                    TARGET_DIST_MIN_M, TARGET_DIST_MAX_M, ENTRY_STEPS_M, N_STEPS_M, 2);
}

float uiEnterBottleLineM(float current) {
  return enterValue("B:", "m", std::clamp(current, BOTTLE_LINE_MIN_M, BOTTLE_LINE_MAX_M),
                    BOTTLE_LINE_MIN_M, BOTTLE_LINE_MAX_M, ENTRY_STEPS_M, N_STEPS_M, 2);
}

float uiEnterTimeS(float current) {
  return enterValue("T:", "s", std::clamp(current, TARGET_TIME_MIN_S, TARGET_TIME_MAX_S),
                    TARGET_TIME_MIN_S, TARGET_TIME_MAX_S, ENTRY_STEPS_S, N_STEPS_S, 1);
}

int uiSelectLevel(int current) {
  // Labels are built from config.h §5, so they can't drift from what the plan
  // actually uses if the rules move the bottle.
  static char labels[3][17];
  static const char *names[] = {"Regional", "State", "National"};
  static const char *items[3];
  for (int i = 0; i < 3; i++) {
    snprintf(labels[i], sizeof labels[i], "%s %.1fm", names[i], (double)bottleXForLevel(i));
    items[i] = labels[i];
  }
  return uiMenuSelect("Bottle level", items, 3, std::clamp(current, 0, 2));
}

float uiEnterOffsetM(const char *label, float current) {
  // mm-level resolution: the vehicle term is 2 pt/cm, so 0.001 m steps are finer
  // than the scoring granularity and 0.01 m gets you there fast.
  static const float incs[] = {0.010f, 0.001f, 0.050f};
  return enterValue(label, "m", std::clamp(current, STOP_OFFSET_MIN_M, STOP_OFFSET_MAX_M),
                    STOP_OFFSET_MIN_M, STOP_OFFSET_MAX_M, incs, 3, 3);
}

// The CLK history for uiRotaryStep(). Re-seeded from the ACTUAL pin whenever a
// screen starts using the dial (uiRotaryResync): the value entry screens keep their
// own history, so after the dial turns there this one is stale, and a stale "high"
// with CLK now low reads as a phantom detent — a menu jumping one item (the bottle
// level!) or the servo stepping once, the moment the screen opens.
static uint8_t s_rotaryLastClk = 1;

void uiRotaryResync() { s_rotaryLastClk = gpio_get(DIAL_CLK_PIN); }

int uiRotaryStep() {
  return rotaryStep(s_rotaryLastClk);
}

void uiWaitSetRelease() {
  while (uiSetPressed()) sleep_ms(5);
  sleep_ms(BUTTON_RELEASE_SETTLE_MS);       // a release bounce is not a new press
}

bool uiDialPressed() { return !gpio_get(DIAL_BTN_PIN); }

int uiMenuSelect(const char *title, const char *const *items, int n, int startIdx) {
  int  idx     = (startIdx >= 0 && startIdx < n) ? startIdx : 0;
  uiRotaryResync();                         // no phantom detent on opening
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
      uiWaitSetRelease();
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
      sleep_ms(BUTTON_RELEASE_SETTLE_MS);
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
