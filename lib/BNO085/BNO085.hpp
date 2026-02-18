#include <Adafruit_BNO08x.h>
#include <sh2_SensorValue.h>

//If MATHUTILS_H is not defined, define it (start of header guard)
#ifndef BNO085_H
#define BNO085_H

//functions to setup and get data from the bno085

void checkBNO085Connection(Adafruit_BNO08x *bno);

void setupBNO085(Adafruit_BNO08x *bno);

sh2_SensorValue_t getBNO085Event(Adafruit_BNO08x *bno);

#endif
