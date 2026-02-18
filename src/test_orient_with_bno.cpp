#include <Arduino.h>
#include "BNO055.hpp"
#include "DebugLog.h"
#include "Config.h"
#include "motorDriver.hpp"

namespace {
motorDriver orientationMotor;
Adafruit_BNO055 bno =  Adafruit_BNO055(55, 0x28);
}

void setup() {
  LOG_BEGIN(kSerialBaud);
  LOG_PRINTLN(F("[TEST_ORIENT_BNO] setup(): initializing BNO085"));
  setupBNO055(&bno);
  checkBNO055Connection(&bno);

  LOG_PRINTLN(F("[TEST_ORIENT_BNO] setup(): creating orientation motor driver"));
  orientationMotor = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep, kOrientMotorNFault);
  LOG_PRINTLN(F("[TEST_ORIENT_BNO] setup(): ready"));
}

void loop() {
  LOG_PRINTLN(F("[TEST_ORIENT_BNO] loop(): driving motor forward for 5s"));
  orientationMotor.moveMotorForward(kOrientationDutyCycle);
  delay(kTestMotorStepDelayMs);

  const sensors_event_t event = getBNO055Event(&bno);
  LOG_PRINT(F("[TEST_ORIENT_BNO] gravity x/y/z="));
  LOG_PRINT(event.orientation.x, 3);
  LOG_PRINT(F("/"));
  LOG_PRINT(event.orientation.y, 3);
  LOG_PRINT(F("/"));
  LOG_PRINTLN(event.orientation.z, 3);

  if (event.orientation.z <= kOrientationAlignedZMax &&
      event.orientation.z >= kOrientationAlignedZMin) {
    LOG_PRINTLN(F("[TEST_ORIENT_BNO] condition met: z near -1g, stopping motor"));
    orientationMotor.stopMotorWithCoast();
    delay(kTestMotorStepDelayMs);
  } else {
    LOG_PRINTLN(F("[TEST_ORIENT_BNO] condition not met: keep running next loop"));
  }
}
