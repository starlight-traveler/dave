#include <Arduino.h>
#include "FlightController.h"
/** @brief Single global controller instance. */
static FlightController controller(Serial1);

/** @brief Arduino setup hook. */
void setup() {
  Serial.begin(9600);
  Serial.println(F("[MAIN] setup(): starting main flight firmware"));
  controller.begin();
  Serial.println(F("[MAIN] setup(): controller initialized"));
}

/** @brief Arduino loop hook. */
void loop() {
  static elapsedMillis heartbeat;
  if (heartbeat >= 1000) {
    heartbeat = 0;
    Serial.println(F("[MAIN] loop(): controller.update() heartbeat"));
  }
  controller.update();
}


