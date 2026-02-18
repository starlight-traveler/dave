#include <Arduino.h>
#include "Config.h"
#include "DebugLog.h"
#include "motorDriver.hpp"

namespace {
motorDriver augerMotor;
}

void setup() {
  LOG_BEGIN(kSerialBaud);
  LOG_PRINTLN(F("[TEST_AUGER] setup(): creating auger MOSFET driver"));
  LOG_PRINT(F("[TEST_AUGER] gpio="));
  LOG_PRINTLN(gpioAuger);
  augerMotor = motorDriver(gpioAuger);
  LOG_PRINTLN(F("[TEST_AUGER] setup(): ready"));
}

void loop() {
  LOG_PRINTLN(F("[TEST_AUGER] loop(): waiting 5s before ON"));
  delay(kTestMotorStepDelayMs);
  LOG_PRINTLN(F("[TEST_AUGER] loop(): moveMosfet() -> ON"));
  augerMotor.moveMosfet();
  delay(kTestMotorStepDelayMs);
  LOG_PRINTLN(F("[TEST_AUGER] loop(): stopMosfet() -> OFF"));
  augerMotor.stopMosfet();
}
