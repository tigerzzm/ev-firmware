#include "lcd.h"
#include "config.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

// Bus + address. I2C0 is brought up at 100 kHz by imu.cpp's begin_I2C, but we
// also (idempotently) ensure the pins/rate here so the LCD works regardless of
// init order.
static i2c_inst_t *s_i2c = i2c0;
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

void lcdInit() {
  // Ensure the shared bus is up (100 kHz) and pins are I2C — harmless if the
  // IMU already did this.
  i2c_init(s_i2c, 100 * 1000);
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);

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
