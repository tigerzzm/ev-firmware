#pragma once
#include <cstdint>
#include "hardware/i2c.h"
#include "sh2_SensorValue.h"
#define BNO08x_I2CADDR_DEFAULT 0x4A
class Adafruit_BNO08x {
public:
  Adafruit_BNO08x(int8_t reset = -1){ (void)reset; }
  bool begin_I2C(uint8_t addr = BNO08x_I2CADDR_DEFAULT, i2c_inst_t* wire = i2c0,
                 uint sda = 0, uint scl = 0, int32_t sid = 0){ return true; }
  bool enableReport(uint8_t id, uint32_t interval_us){ return true; }
  bool getSensorEvent(sh2_SensorValue_t* v){ return true; }
  bool wasReset(){ return false; }
};
