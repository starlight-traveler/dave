#include "ICM20649.hpp"

uint32_t lastEventMs = 0; //last event recorded in ms
uint32_t lastReconnectAttemptMs = 0; //last attempt to get an event in ms

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
      lastEventMs = millis();
      return;
    }

    //if no getting event, get time
    const uint32_t now = millis(); // current time
    if (lastEventMs == 0) {
        lastEventMs = now;
    }

    const bool dataTimedOut = (now - lastEventMs) >= kIcmNoDataReconnectMs;
    const bool backoffElapsed = (now - lastReconnectAttemptMs) >= kIcmReconnectBackoffMs;
    if (!dataTimedOut || !backoffElapsed) { //if either are still false, return back to program
        return;
    }

    lastReconnectAttemptMs = now;
    if (reconnectICM(icm)) {  // if you can reconnect, the last event is current
       lastEventMs = now;
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
      lastEventMs = millis();
      lastReconnectAttemptMs = 0; // no attemps thus far
      return;
    }
    lastReconnectAttemptMs = 0;
    delay(kIcmReconnectBackoffMs);
  }

}

//this function will get the acceleration from the icm, returning the overall event
sensors_event_t getICM20649Accel(Adafruit_ICM20649 *icm){
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  if(icm != nullptr && icm->getEvent(&accel, &gyro, &temp)){
    lastEventMs = millis();
    return accel; // only need accel

  }

  checkICMConnection(icm);
  return accel; // only need accel

}
