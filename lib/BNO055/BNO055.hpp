#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Config.h"

#ifndef BNO055_HPP
#define BNO055_HPP

void setupBNO055(Adafruit_BNO055 *bno);
bool reconnectBNO055(Adafruit_BNO055 *bno);
void checkBNO055Connection(Adafruit_BNO055 *bno);
sensors_event_t getBNO055Event(Adafruit_BNO055 *bno);

#endif