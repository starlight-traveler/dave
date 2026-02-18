#include <Arduino.h>
#include "ICM20649.hpp"
#include "Config.h"
#include "DebugLog.h"

namespace {
 Adafruit_ICM20649 icm;
}

void setup() {
  LOG_BEGIN(kSerialBaud);
  LOG_PRINTLN(F("[TEST_ICM setup(): starting ICM-only test"));
  LOG_PRINTLN(F("[TEST_ICM] setup(): initializing BNO085"));
  setupICM20649(&icm);
  checkICMConnection(&icm);
  LOG_PRINTLN(F("[TEST_ICM] setup(): ICM ready, streaming accel every 2s"));
}

void loop() {
  const sensors_event_t accel = getICM20649Accel(&icm);
  LOG_PRINT(F("[TEST_ICM] accel x="));
  LOG_PRINT(accel.acceleration.x, 4);
  LOG_PRINT(F(" y="));
  LOG_PRINT(accel.acceleration.y, 4);
  LOG_PRINT(F(" z="));
  LOG_PRINTLN(accel.acceleration.z, 4);
  delay(kTestBnoSamplePeriodMs);
}
