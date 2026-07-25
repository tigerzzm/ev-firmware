#pragma once
#include <cstdint>
struct sh2_RotationVector_t { float i, j, k, real; };
struct sh2_SensorValue_t {
  uint8_t  sensorId;
  uint32_t timestamp;
  union { sh2_RotationVector_t gameRotationVector; } un;
};
