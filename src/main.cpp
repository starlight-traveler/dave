#include <Arduino.h>

#include "FlightController.h"

/** @brief Single global controller instance. */
static FlightController controller(Serial1);

/** @brief Arduino setup hook. */
void setup() {
  controller.begin();
}

/** @brief Arduino loop hook. */
void loop() {
  controller.update();
}
