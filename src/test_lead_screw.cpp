#include <Arduino.h>
#include "Config.h"
#include "DebugLog.h"
#include "motorDriver.hpp"

namespace {
int32_t topHits = 0;
int32_t bottomHits = 0;
bool upperSwitchPressed = false;
bool lowerSwitchPressed = false;
bool leadScrewFullyExtended = false;
elapsedMillis switchPollTimer;
motorDriver leadScrewMotor;

void pollLimitSwitches() {
  if (switchPollTimer < 5) {
    return;
  }
  switchPollTimer = 0;

  const bool prevUpper = upperSwitchPressed;
  const bool prevLower = lowerSwitchPressed;

  upperSwitchPressed = (digitalReadFast(kUpperLimitSwitchPin) == LOW);
  lowerSwitchPressed = (digitalReadFast(kLowerLimitSwitchPin) == LOW);

  if (prevUpper != upperSwitchPressed || prevLower != lowerSwitchPressed) {
    LOG_PRINT(F("[TEST_LEAD] switch change upper/lower="));
    LOG_PRINT(upperSwitchPressed ? F("PRESSED") : F("RELEASED"));
    LOG_PRINT(F("/"));
    LOG_PRINTLN(lowerSwitchPressed ? F("PRESSED") : F("RELEASED"));
  }
}
}

void setup() {
  LOG_BEGIN(9600);
  LOG_PRINTLN(F("[TEST_LEAD] setup(): creating lead screw motor driver"));
  leadScrewMotor = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);

  LOG_PRINTLN(F("[TEST_LEAD] setup(): configuring limit switches with pullups"));
  pinMode(kUpperLimitSwitchPin, INPUT_PULLUP);
  pinMode(kLowerLimitSwitchPin, INPUT_PULLUP);

  LOG_PRINTLN(F("[TEST_LEAD] setup(): test sequence started"));
}

void loop() {
  pollLimitSwitches();

  if (topHits == 0) {
    LOG_PRINTLN(F("[TEST_LEAD] action: searching top switch (move backward)"));
    leadScrewMotor.moveMotorBackward(0.5f);
  }

  if (upperSwitchPressed) {
    topHits++;
    LOG_PRINT(F("[TEST_LEAD] counter update topHits="));
    LOG_PRINTLN(topHits);
  }

  if (bottomHits == 0 && topHits == 1) {
    LOG_PRINTLN(F("[TEST_LEAD] action: moving down toward lower switch"));
    leadScrewMotor.moveMotorForward(0.5f);
  }

  if (lowerSwitchPressed) {
    bottomHits++;
    LOG_PRINT(F("[TEST_LEAD] counter update bottomHits="));
    LOG_PRINTLN(bottomHits);
  }

  if (bottomHits == 1 && topHits == 1) {
    LOG_PRINTLN(F("[TEST_LEAD] action: lower reached once, stopping and marking extended"));
    leadScrewMotor.stopMotorWithCoast();
    leadScrewFullyExtended = true;
  }

  if (leadScrewFullyExtended && bottomHits == 1 && topHits == 1) {
    LOG_PRINTLN(F("[TEST_LEAD] action: retracting after extension"));
    leadScrewFullyExtended = false;
    leadScrewMotor.moveMotorBackward(0.5f);
  }

  if (upperSwitchPressed) {
    LOG_PRINTLN(F("[TEST_LEAD] action: upper hit while retracting, stopping"));
    leadScrewMotor.stopMotorWithCoast();
    topHits++;
  }

  if (topHits == 2 && bottomHits == 1) {
    LOG_PRINTLN(F("[TEST_LEAD] cycle complete, resetting counters"));
    topHits = 0;
    bottomHits = 0;
  }

  static elapsedMillis heartbeat;
  if (heartbeat >= 1000) {
    heartbeat = 0;
    LOG_PRINT(F("[TEST_LEAD] heartbeat top/bottom="));
    LOG_PRINT(topHits);
    LOG_PRINT(F("/"));
    LOG_PRINTLN(bottomHits);
  }
}
