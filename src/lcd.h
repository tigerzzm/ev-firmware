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

// Scans the LCD's I2C bus and prints every address that responds. Returns true if
// something answered at LCD_I2C_ADDR. Worth having because HD44780 backpacks are
// written blind — without this, a missing LCD, a wrong address and a broken wire
// all look identical (silence).
bool lcdProbe();

void lcdInit();                                   // 4-bit init; assumes I2C0 pins set
void lcdClear();
void lcdSetCursor(uint8_t col, uint8_t row);      // row 0 or 1
void lcdPrint(const char *s);
void lcdPrintAt(uint8_t col, uint8_t row, const char *s);
