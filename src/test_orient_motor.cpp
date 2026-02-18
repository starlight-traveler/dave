#include <Arduino.h>
#include "Config.h"
#include "DebugLog.h"
#include "motorDriver.hpp"

namespace {
motorDriver orientMotor;
}

void setup() {
  LOG_BEGIN(kSerialBaud);
  LOG_PRINTLN(F("[TEST_ORIENT_MOTOR] setup(): creating orientation motor driver"));
  LOG_PRINT(F("[TEST_ORIENT_MOTOR] pins in1/in2/slp="));
  LOG_PRINT(kOrientMotorIn1);
  LOG_PRINT(F("/"));
  LOG_PRINT(kOrientMotorIn2);
  LOG_PRINT(F("/"));
  LOG_PRINTLN(kOrientMotorSleep);
  orientMotor = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep);
  LOG_PRINTLN(F("[TEST_ORIENT_MOTOR] setup(): ready"));
}

void loop() {
  LOG_PRINTLN(F("[TEST_ORIENT_MOTOR] loop(): waiting 5s before forward command"));
  delay(kTestMotorStepDelayMs);
  LOG_PRINTLN(F("[TEST_ORIENT_MOTOR] loop(): moveMotorForward(1.0)"));
  orientMotor.moveMotorForward(kOrientationDutyCycle);
  delay(kTestMotorStepDelayMs);
  LOG_PRINTLN(F("[TEST_ORIENT_MOTOR] loop(): stopMotorWithCoast()"));
  orientMotor.stopMotorWithCoast();
}
