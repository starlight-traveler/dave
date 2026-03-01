#include <Arduino.h>
#include "Config.h"
#include "DebugLog.h"
#include "motorDriver.hpp"

namespace {
motorDriver waterMotor = motorDriver(gpioWater);
}

void setup() {
  LOG_BEGIN(kSerialBaud);
  LOG_PRINT(gpioWater);
  LOG_PRINT(F("/"));
  LOG_PRINT(gpioWater);
  LOG_PRINT(F("/"));
  waterMotor = motorDriver(gpioWater);
  LOG_PRINTLN(F("[TEST_WATER] setup(): ready"));

  LOG_PRINTLN(F("[TEST_WATER] loop(): waiting 5s before motor ON"));
  delay(kTestMotorStepDelayMs);
  LOG_PRINTLN(F("[TEST_WATER] loop(): moveMotorForward(1.0)"));
  waterMotor.moveMosfet();
  delay(60000);
  LOG_PRINTLN(F("[TEST_WATER] loop(): stopMotorWithCoast()"));
  waterMotor.stopMosfet();
}


void loop() {
  // LOG_PRINTLN(F("[TEST_WATER] loop(): waiting 5s before motor ON"));
  // delay(kTestMotorStepDelayMs);
  // LOG_PRINTLN(F("[TEST_WATER] loop(): moveMotorForward(1.0)"));
  // waterMotor.moveMosfet();
  // delay(60000);
  // LOG_PRINTLN(F("[TEST_WATER] loop(): stopMotorWithCoast()"));
  // waterMotor.stopMosfet();
}
