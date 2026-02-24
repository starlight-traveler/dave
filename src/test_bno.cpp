#include <Arduino.h>
#include "BNO055.hpp"
#include "Config.h"
#include "DebugLog.h"

namespace {
  Adafruit_BNO055 bno =  Adafruit_BNO055(55, 0x28);
}

void setup() {
  LOG_BEGIN(kSerialBaud);
  LOG_PRINTLN(F("[TEST_BNO] setup(): starting BNO-only test"));
  LOG_PRINTLN(F("[TEST_BNO] setup(): initializing BNO055"));
  setupBNO055(&bno);
  checkBNO055Connection(&bno);
  LOG_PRINTLN(F("[TEST_BNO] setup(): BNO ready, streaming orientation every 2s"));
}

void loop() {
  const sensors_event_t event = getBNO055Event(&bno);
  LOG_PRINT(F("[TEST_BNO] orientation x="));
  LOG_PRINT(event.orientation.x, 4);
  LOG_PRINT(F(" y="));
  LOG_PRINT(event.orientation.y, 4);
  LOG_PRINT(F(" z="));
  LOG_PRINTLN(event.orientation.z, 4);
  delay(kTestBnoSamplePeriodMs);
}

