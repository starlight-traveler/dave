#include <Wire.h>
#include <SPI.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>

#ifndef H3LIS331_HPP
#define H3LIS331_HPP

//functions to setup and get data from the H3LIS331
void checkH3LIS331Connection(Adafruit_H3LIS331 *lis);
void setupH3LIS331(Adafruit_H3LIS331 *lis);
sensors_event_t getH3LIS331Accel(Adafruit_H3LIS331 *lis);

#endif
