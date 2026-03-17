#include "ICM20649.hpp"

uint32_t lastEventMsICM = 0; //last event recorded in ms
uint32_t lastReconnectAttemptMsICM = 0; //last attempt to get an event in ms
uint32_t kIcmReportIntervalUs = 10000;              // all configurations are the same as the bno ones
uint32_t kIcmNoDataReconnectMs = 1500;              
uint32_t kIcmReconnectBackoffMs = 2000;            
uint8_t kIcmStartupInitAttempts = 3;               
uint32_t kIcmStartupRetryDelayMs = 50;      


bool reconnectICM(Adafruit_ICM20649 *icm){
  if (!icm->begin_I2C()) {
      return false;
    }

    return true;

}

//this function will check the icm connection, and try to begin again if it fails
void checkICMConnection(Adafruit_ICM20649 *icm){
    if (icm == nullptr) {
        return;
    }

    //seeing if we can get event
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    // Get all data at once efficiently
    if(icm->getEvent(&accel, &gyro, &temp)){
      lastEventMsICM = millis();
      return;
    }

    //if no getting event, get time
    const uint32_t now = millis(); // current time
    if (lastEventMsICM == 0) {
        lastEventMsICM = now;
    }

    const bool dataTimedOut = (now - lastEventMsICM) >= kIcmNoDataReconnectMs;
    const bool backoffElapsed = (now - lastReconnectAttemptMsICM) >= kIcmReconnectBackoffMs;
    if (!dataTimedOut || !backoffElapsed) { //if either are still false, return back to program
        return;
    }

    lastReconnectAttemptMsICM = now;
    if (reconnectICM(icm)) {  // if you can reconnect, the last event is current
       lastEventMsICM = now;
    }

}

//this function will set up the icm
void setupICM20649(Adafruit_ICM20649 *icm){
  if (icm == nullptr) {
    return; // nothing to attempt to connect to
  }

  for (uint8_t attempt = 0; attempt < kIcmStartupInitAttempts; ++attempt) {
    if(reconnectICM(icm)){
      icm->setAccelRange(ICM20649_ACCEL_RANGE_30_G);
      lastEventMsICM = millis();
      lastReconnectAttemptMsICM = 0; // no attemps thus far
      return;
    }
    lastReconnectAttemptMsICM = 0;
    delay(kIcmReconnectBackoffMs);
  }

}

//this function will get the acceleration from the icm, returning the overall event
sensors_event_t getICM20649Accel(Adafruit_ICM20649 *icm){
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  if(icm != nullptr && icm->getEvent(&accel, &gyro, &temp)){
    lastEventMsICM = millis();
    return accel; // only need accel

  }

  checkICMConnection(icm);
  return accel; // only need accel

}
