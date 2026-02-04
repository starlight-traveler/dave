#include "Adafruit_BMP3XX.h"

// If MATHUTILS_H is not defined, define it (start of header guard)
#ifndef BMPEXAMPLE_H

// Define MATHUTILS_H to prevent multiple inclusions
#define BMPEXAMPLE_H

void checkBMP390Connection(Adafruit_BMP3XX *bmp);

void setupBMP(Adafruit_BMP3XX *bmp);

float getAltitude(Adafruit_BMP3XX * bmp);

#endif