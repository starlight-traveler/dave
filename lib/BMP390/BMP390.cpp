#include "BMP390.hpp"

#define SEALEVELPRESSURE_HPA (1013.25)
// Adafruit_BMP3XX
//this function will check the connection to the bmp, and try again if it fails

void checkBMP390Connection(Adafruit_BMP3XX *bmp)
{
  if(!bmp->performReading())
  {
    bmp->begin_I2C();
  }
}

void setupBMP(Adafruit_BMP3XX *bmp)
{
    if (!bmp->begin_I2C()) 
    {  
        checkBMP390Connection(bmp);
    }
    bmp->setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp->setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp->setOutputDataRate(BMP3_ODR_50_HZ);
} 

float getAltitude(Adafruit_BMP3XX *bmp)
{
     if (!bmp->performReading()) 
    {
          return -1;
    }
    float altitude = bmp->readAltitude(SEALEVELPRESSURE_HPA);
    return altitude;
}

