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
