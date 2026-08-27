#include "lcd.h"
#include "config.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <cstdio>

// Bus + address. The LCD owns I2C1 outright (config.h §6) — the IMU is on I2C0 —
// so this init is not racing anything and the display can be written at any time,
// including mid-run.
static i2c_inst_t *s_i2c = LCD_I2C_INST;
static const uint8_t ADDR = LCD_I2C_ADDR;

static constexpr uint8_t BL = 0x08;  // backlight bit (kept on)
static constexpr uint8_t EN = 0x04;  // enable strobe
static constexpr uint8_t RS = 0x01;  // register select (1 = data)

static void busWrite(uint8_t b) {
  i2c_write_blocking(s_i2c, ADDR, &b, 1, false);
}

static void pulse(uint8_t data) {
  busWrite(data | EN);
  sleep_us(1);
  busWrite(data & ~EN);
  sleep_us(50);
}

static void writeNibble(uint8_t nibble, uint8_t rs) {
  uint8_t data = (nibble & 0xF0) | BL | rs;
  pulse(data);
}

static void send(uint8_t val, uint8_t rs) {
  writeNibble(val & 0xF0, rs);
  writeNibble((uint8_t)(val << 4) & 0xF0, rs);
}

static void command(uint8_t c) { send(c, 0); }
static void data(uint8_t d)    { send(d, RS); }

bool lcdProbe() {
  printf("[LCD] scanning I2C1 (SDA=GP%d, SCL=GP%d)...\n", LCD_SDA_PIN, LCD_SCL_PIN);
  int  found = 0;
  bool atConfigured = false;
  // 0x00-0x07 and 0x78-0x7F are reserved by the I2C spec — skip them. A 1-byte
  // read is the safe probe: it cannot alter a device the way a write might.
  for (uint8_t a = 0x08; a < 0x78; a++) {
    uint8_t rx = 0;
    if (i2c_read_blocking(s_i2c, a, &rx, 1, false) >= 0) {
      found++;
      const bool isCfg = (a == LCD_I2C_ADDR);
      if (isCfg) atConfigured = true;
      printf("[LCD]   device at 0x%02X%s\n", a, isCfg ? "   <-- configured LCD_I2C_ADDR" : "");
    }
  }
  if (found == 0) {
    printf("[LCD]   NOTHING ON THE BUS — check LCD power (3.3 V) and SDA/SCL wiring\n");
  } else if (!atConfigured) {
    printf("[LCD]   nothing at configured 0x%02X — set LCD_I2C_ADDR (config.h) to an "
           "address above\n", LCD_I2C_ADDR);
  }
  return atConfigured;
}

void lcdInit() {
  i2c_init(s_i2c, 100 * 1000);
  gpio_set_function(LCD_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(LCD_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(LCD_SDA_PIN);
  gpio_pull_up(LCD_SCL_PIN);

  lcdProbe();     // report what is actually out there before writing blind

  sleep_ms(50);
  // HD44780 4-bit wake-up sequence.
  writeNibble(0x30, 0); sleep_ms(5);
  writeNibble(0x30, 0); sleep_us(150);
  writeNibble(0x30, 0); sleep_us(150);
  writeNibble(0x20, 0); sleep_us(150);   // set 4-bit mode

  command(0x28);   // function set: 4-bit, 2 lines, 5x8
  command(0x08);   // display off
  command(0x01);   // clear
  sleep_ms(2);
  command(0x06);   // entry mode: increment, no shift
  command(0x0C);   // display on, cursor off, blink off
}

void lcdClear() {
  command(0x01);
  sleep_ms(2);
}

void lcdSetCursor(uint8_t col, uint8_t row) {
  static const uint8_t rowOffset[2] = {0x00, 0x40};
  if (row > 1) row = 1;
  command(0x80 | (col + rowOffset[row]));
}

void lcdPrint(const char *s) {
  for (; s && *s; ++s) data((uint8_t)*s);
}

void lcdPrintAt(uint8_t col, uint8_t row, const char *s) {
  lcdSetCursor(col, row);
  lcdPrint(s);
}
