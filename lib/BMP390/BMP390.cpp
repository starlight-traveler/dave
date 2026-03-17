#include "BMP390.hpp"

#define SEALEVELPRESSURE_HPA (1013.25)

uint32_t lastEventMsBMP = 0; //last event recorded in ms
uint32_t lastReconnectAttemptMsBMP = 0; //last attempt to get an event in ms
uint32_t kBmpReportIntervalUs = 10000;              // all configurations are the same as the bno and icm ones
uint32_t kBmpNoDataReconnectMs = 1500;              
uint32_t kBmpReconnectBackoffMs = 2000;            
uint8_t kBmpStartupInitAttempts = 3;               
uint32_t kBmpStartupRetryDelayMs = 50;  


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
    lastEventMsBMP = millis();
    return;
  }

  const uint32_t now = millis();
  if (lastEventMsBMP == 0) {
    lastEventMsBMP = now;
  }

  const bool dataTimedOut = (now - lastEventMsBMP) >= kBmpNoDataReconnectMs;
  const bool backoffElapsed = (now - lastReconnectAttemptMsBMP) >= kBmpReconnectBackoffMs;
  if (!dataTimedOut || !backoffElapsed) {
    return;
  }

  lastReconnectAttemptMsBMP = now; //timestamp of last reconnect attempt (happening rn)
  if (reconnectBMP390(bmp)) { // if connection is successful, setting the last event to now (that happens in getBNO055Event())
      lastEventMsBMP = now;
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

      lastEventMsBMP = millis();
      lastReconnectAttemptMsBMP = 0;
      return;
    }

    lastReconnectAttemptMsBMP = millis();
    delay(kBmpReconnectBackoffMs);
  } 

}

float getAltitude(Adafruit_BMP3XX *bmp) {
  float alt = 0;

  if(bmp != nullptr && bmp->performReading()) {
    lastEventMsBMP = millis();
    alt = bmp->readAltitude(SEALEVELPRESSURE_HPA);
     
  }
  checkBMP390Connection(bmp);
  return alt;
}

