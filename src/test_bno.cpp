#include <Arduino.h>
#include "BNO085.hpp"
#include "Config.h"
#include "DebugLog.h"

namespace {
Adafruit_BNO08x bno;
}

void setup() {
  LOG_BEGIN(kSerialBaud);
  LOG_PRINTLN(F("[TEST_BNO] setup(): starting BNO-only test"));
  LOG_PRINTLN(F("[TEST_BNO] setup(): initializing BNO085"));
  setupBNO085(&bno);
  checkBNO085Connection(&bno);
  LOG_PRINTLN(F("[TEST_BNO] setup(): BNO ready, streaming accel every 2s"));
}

void loop() {
  const sh2_SensorValue_t event = getBNO085Event(&bno);
  LOG_PRINT(F("[TEST_BNO] accel x="));
  LOG_PRINT(event.un.accelerometer.x, 4);
  LOG_PRINT(F(" y="));
  LOG_PRINT(event.un.accelerometer.y, 4);
  LOG_PRINT(F(" z="));
  LOG_PRINTLN(event.un.accelerometer.z, 4);
  delay(kTestBnoSamplePeriodMs);
}
