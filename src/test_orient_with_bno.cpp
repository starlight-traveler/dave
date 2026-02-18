#include <Arduino.h>
#include "BNO085.hpp"
#include "DebugLog.h"
#include "Config.h"
#include "motorDriver.hpp"

namespace {
motorDriver orientationMotor;
Adafruit_BNO08x bno;
}

void setup() {
  LOG_BEGIN(kSerialBaud);
  LOG_PRINTLN(F("[TEST_ORIENT_BNO] setup(): initializing BNO085"));
  setupBNO085(&bno);
  checkBNO085Connection(&bno);

  LOG_PRINTLN(F("[TEST_ORIENT_BNO] setup(): creating orientation motor driver"));
  orientationMotor = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep, kOrientMotorNFault);
  LOG_PRINTLN(F("[TEST_ORIENT_BNO] setup(): ready"));
}

void loop() {
  LOG_PRINTLN(F("[TEST_ORIENT_BNO] loop(): driving motor forward for 5s"));
  orientationMotor.moveMotorForward(kOrientationDutyCycle);
  delay(kTestMotorStepDelayMs);

  const sh2_SensorValue_t event = getBNO085Event(&bno);
  LOG_PRINT(F("[TEST_ORIENT_BNO] gravity x/y/z="));
  LOG_PRINT(event.un.gravity.x, 3);
  LOG_PRINT(F("/"));
  LOG_PRINT(event.un.gravity.y, 3);
  LOG_PRINT(F("/"));
  LOG_PRINTLN(event.un.gravity.z, 3);

  if (event.un.gravity.z <= kOrientationAlignedZMax &&
      event.un.gravity.z >= kOrientationAlignedZMin) {
    LOG_PRINTLN(F("[TEST_ORIENT_BNO] condition met: z near -1g, stopping motor"));
    orientationMotor.stopMotorWithCoast();
    delay(kTestMotorStepDelayMs);
  } else {
    LOG_PRINTLN(F("[TEST_ORIENT_BNO] condition not met: keep running next loop"));
  }
}
