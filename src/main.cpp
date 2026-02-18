#include <Arduino.h>
#include "Config.h"
#include "DebugLog.h"
#include "FlightController.h"
/** @brief Single global controller instance. */
static FlightController controller(Serial1);

/** @brief Arduino setup hook. */
void setup() {
  LOG_BEGIN(kSerialBaud);
  LOG_PRINTLN(F("[MAIN] setup(): starting main flight firmware"));
  controller.begin();
  LOG_PRINTLN(F("[MAIN] setup(): controller initialized"));
}

/** @brief Arduino loop hook. */
void loop() {
  static elapsedMillis heartbeat;
  if (heartbeat >= kMainHeartbeatMs) {
    heartbeat = 0;
    LOG_PRINTLN(F("[MAIN] loop(): controller.update() heartbeat"));
  }
  controller.update();
}

