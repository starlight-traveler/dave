#include "BMP390.hpp"

#define SEALEVELPRESSURE_HPA (1013.25)

uint32_t lastEventMs = 0; //last event recorded in ms
uint32_t lastReconnectAttemptMs = 0; //last attempt to get an event in ms


bool reconnectBMP390(Adafruit_BMP3XX *bmp) {
  if(!bmp->begin_I2C()){
    return false;
  }
  return true;

}

void checkBMP390Connection(Adafruit_BMP3XX *bmp) {
  //reutrning if no bmp
  if(bmp == nullptr)
    return;

  //returning if an a reading can be performed
  if(bmp->performReading()){
    lastEventMs = millis();
    return;
  }

  const uint32_t now = millis();
  if (lastEventMs == 0) {
    lastEventMs = now;
  }

  const bool dataTimedOut = (now - lastEventMs) >= kBmpNoDataReconnectMs;
  const bool backoffElapsed = (now - lastReconnectAttemptMs) >= kBnoReconnectBackoffMs;
  if (!dataTimedOut || !backoffElapsed) {
    return;
  }

  lastReconnectAttemptMs = now; //timestamp of last reconnect attempt (happening rn)
  if (reconnectBMP390(bmp)) { // if connection is successful, setting the last event to now (that happens in getBNO055Event())
      lastEventMs = now;
  }

}

void setupBMP(Adafruit_BMP3XX *bmp) {
  if (bmp == nullptr) {
    return;
  }

  for (uint8_t attempt = 0; attempt < kBmpStartupInitAttempts; ++attempt) {
    if(reconnectBMP390(bmp)){
      bmp->setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
      bmp->setPressureOversampling(BMP3_OVERSAMPLING_4X);
      bmp->setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp->setOutputDataRate(BMP3_ODR_50_HZ);

      lastEventMs = millis();
      lastReconnectAttemptMs = 0;
      return;
    }

    lastReconnectAttemptMs = millis();
    delay(kBmpReconnectBackoffMs);
  } 

}

float getAltitude(Adafruit_BMP3XX *bmp) {
  float alt;

  if(bmp != nullptr && bmp->performReading()) {
    lastEventMs = millis();
    alt = bmp->readAltitude(SEALEVELPRESSURE_HPA);
     
  }
  
  checkBMP390Connection(bmp);
  return alt;
}

