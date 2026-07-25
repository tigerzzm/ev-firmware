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

// TODO(bench): if the dial counts the wrong way, flip DIR_SIGN.
static constexpr int DIR_SIGN = +1;

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

// --- one rotary detent since last call: -1 / 0 / +1 -------------------------
static int rotaryStep(uint8_t &lastClk) {
  uint8_t clk = gpio_get(DIAL_CLK_PIN);
  int step = 0;
  if (lastClk == 1 && clk == 0) {                 // CLK falling edge = one detent
    step = gpio_get(DIAL_DT_PIN) ? +1 : -1;       // DT level gives direction
    step *= DIR_SIGN;
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
}

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

float uiEnterBulgeH() {
  // Preset from config (matches the kit's fixed arc height). Dial-tunable later
  // by swapping this for enterValue() over a bulge range.
  char l1[17];
  snprintf(l1, sizeof l1, "gate h=%.2fm", GATE_BULGE_H_M);
  uiShow("Arc preset", l1);
  sleep_ms(400);
  return GATE_BULGE_H_M;
}
