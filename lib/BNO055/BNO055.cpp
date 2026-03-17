#include "BNO055.hpp"

uint32_t lastEventMsBNO = 0; //last event recorded in ms
uint32_t lastReconnectAttemptMsBNO = 0; //last attempt to get an event in ms
uint32_t kBnoReportIntervalUs = 10000;              // Desired BNO sensor report period in microseconds.
uint32_t kBnoNoDataReconnectMs = 1500;              // No-data timeout before attempting BNO reconnect (milliseconds).
uint32_t kBnoReconnectBackoffMs = 2000;             // Minimum spacing between reconnect attempts (milliseconds).
uint8_t kBnoStartupInitAttempts = 3;                // Number of startup initialization/reconnect attempts.
uint32_t kBnoStartupRetryDelayMs = 50;              // Delay between BNO startup retries (milliseconds).

bool reconnectBNO055(Adafruit_BNO055 *bno){
    if(!bno->begin()){
        return false;
    }
    return true;

}

void checkBNO055Connection(Adafruit_BNO055 *bno){
    //if the memory address of the bno doesnt exist, exiting the program because no connection to check
        if (bno == nullptr) {
            return;
        }

    // checking to see the the bno can return an event and exiting the function is so
    // then saving that time as the last time an event was recorded
        sensors_event_t event;
        if (bno->getEvent(&event)) {
            lastEventMsBNO = millis();
            return;
        }

    const uint32_t now = millis(); // current time
    if (lastEventMsBNO == 0) { // if there is to no timestamp of last event to reference back to, setting that vairable to current time
        lastEventMsBNO = now;
    }

    // the amount of time since last successful event retrived from bno
    // if the amount of time is greater than 1.5 seconds, the data has timed out
    const bool dataTimedOut = (now - lastEventMsBNO) >= kBnoNoDataReconnectMs;
    // if the amount of the time between current time and the last attempted connection is more than 2 seconds
    //setting back off elasped to true, protection againt attempting to reconnect to many times 
    const bool backoffElapsed = (now - lastReconnectAttemptMsBNO) >= kBnoReconnectBackoffMs;
    if (!dataTimedOut || !backoffElapsed) { //if either are still false, return back to program, no need to reconnect again
        return;
    }

    lastReconnectAttemptMsBNO = now; //timestamp of last reconnect attempt (happening rn)
    if (reconnectBNO055(bno)) { // if connection is successful, setting the last event to now (that happens in getBNO055Event())
        lastEventMsBNO = now;
    }
}


void setupBNO055(Adafruit_BNO055 *bno){
    if (bno == nullptr) {
        return; // nothing to attempt to connect to
    }

    //connecting to the bno three times in a row as apart of the startup process, im assuming to get rid of faulty early values
    for (uint8_t attempt = 0; attempt < kBnoStartupInitAttempts; ++attempt) {
        if (reconnectBNO055(bno)) { // it it connects
            bno->setExtCrystalUse(true);
            lastEventMsBNO = millis(); //setting the last event found to current
            lastReconnectAttemptMsBNO = 0; // no attemps thus far
            return;
        }
        lastReconnectAttemptMsBNO = millis(); // adding this because it should be documented nontheless
        delay(kBnoStartupRetryDelayMs); //waiting 50 ms before going again
    }

}

sensors_event_t getBNO055Event(Adafruit_BNO055 *bno){
  sensors_event_t event;
  if(bno != nullptr && bno->getEvent(&event)){
    lastEventMsBNO = millis();
    return event;
  }

  // Don't reconnect on every miss; check routine has timeout/backoff protection.
  checkBNO055Connection(bno);
  return event;

}

