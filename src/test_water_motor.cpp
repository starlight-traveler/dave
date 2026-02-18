#include <Arduino.h>
#include "Config.h"
#include "DebugLog.h"
#include "motorDriver.hpp"

namespace {
motorDriver waterMotor;
}

void setup() {
  LOG_BEGIN(9600);
  LOG_PRINTLN(F("[TEST_WATER] setup(): creating water motor driver"));
  LOG_PRINT(F("[TEST_WATER] pins in1/in2/slp="));
  LOG_PRINT(kWaterMotorIn1);
  LOG_PRINT(F("/"));
  LOG_PRINT(kWaterMotorIn2);
  LOG_PRINT(F("/"));
  LOG_PRINTLN(kWaterMotorSleep);
  waterMotor = motorDriver(kWaterMotorIn1, kWaterMotorIn2, kWaterMotorSleep);
  LOG_PRINTLN(F("[TEST_WATER] setup(): ready"));
}

void loop() {
  LOG_PRINTLN(F("[TEST_WATER] loop(): waiting 5s before motor ON"));
  delay(5000);
  LOG_PRINTLN(F("[TEST_WATER] loop(): moveMotorForward(1.0)"));
  waterMotor.moveMotorForward(1.0f);
  delay(5000);
  LOG_PRINTLN(F("[TEST_WATER] loop(): stopMotorWithCoast()"));
  waterMotor.stopMotorWithCoast();
}
