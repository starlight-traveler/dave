#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>

#ifndef MATHUTILS_H
#ifndef ICM20649_H

#define MATHUTILS_H
#define ICM20649_H

//functions to setup and get data from the icm20649

void checkICMConnection(Adafruit_ICM20649 *icm);

void setupICM20649(Adafruit_ICM20649 *icm);

sensors_event_t getICM20649Accel(Adafruit_Sensor *accel, Adafruit_ICM20649 *icm);

#endif
#endif