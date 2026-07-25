#pragma once
//==============================================================================
// lcd — minimal 16x2 HD44780 LCD over a PCF8574 I2C backpack (addr 0x27).
// Pico replacement for the Arduino LiquidCrystal_I2C the EV kit used.
// Shares I2C0 with the BNO085 (both at 100 kHz — see imu.cpp / config.h).
//
// PCF8574 bit map (standard backpack): P0=RS, P1=RW, P2=EN, P3=backlight,
// P4..P7 = data D4..D7.  RW is tied low (write-only).
//==============================================================================
#include <cstdint>

void lcdInit();                                   // 4-bit init; assumes I2C0 pins set
void lcdClear();
void lcdSetCursor(uint8_t col, uint8_t row);      // row 0 or 1
void lcdPrint(const char *s);
void lcdPrintAt(uint8_t col, uint8_t row, const char *s);
