#include "BNO055.hpp"

uint32_t lastEventMs = 0; //last event recorded in ms
uint32_t lastReconnectAttemptMs = 0; //last attempt to get an event in ms

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
            lastEventMs = millis();
            return;
        }

    const uint32_t now = millis(); // current time
    if (lastEventMs == 0) { // if there is to no timestamp of last event to reference back to, setting that vairable to current time
        lastEventMs = now;
    }

    // the amount of time since last successful event retrived from bno
    // if the amount of time is greater than 1.5 seconds, the data has timed out
    const bool dataTimedOut = (now - lastEventMs) >= kBnoNoDataReconnectMs;
    // if the amount of the time between current time and the last attempted connection is more than 2 seconds
    //setting back off elasped to true, protection againt attempting to reconnect to many times 
    const bool backoffElapsed = (now - lastReconnectAttemptMs) >= kBnoReconnectBackoffMs;
    if (!dataTimedOut || !backoffElapsed) { //if either are still false, return back to program, no need to reconnect again
        return;
    }

    lastReconnectAttemptMs = now; //timestamp of last reconnect attempt (happening rn)
    if (reconnectBNO055(bno)) { // if connection is successful, setting the last event to now (that happens in getBNO055Event())
        lastEventMs = now;
    }
}


void setupBNO055(Adafruit_BNO055 *bno){
    if (bno == nullptr) {
        return; // nothing to attempt to connect to
    }

    //connecting to the bno three times in a row as apart of the startup process, im assuming to get rid of faulty early values
    for (uint8_t attempt = 0; attempt < kBnoStartupInitAttempts; attempt) {
        if (reconnectBNO055(bno)) { // it it connects
            lastEventMs = millis(); //setting the last event found to current
            lastReconnectAttemptMs = 0; // no attemps thus far
            return;
        }
        lastReconnectAttemptMs = millis(); // adding this because it should be documented nontheless
        delay(kBnoStartupRetryDelayMs); //waiting 50 seconds before gong again
    }

}

sensors_event_t getBNO055Event(Adafruit_BNO055 *bno){
  sensors_event_t event;
  if(bno != nullptr && bno->getEvent(&event)){
    lastEventMs = millis();
    return event;
  }

  // Don't reconnect on every miss; check routine has timeout/backoff protection.
  checkBNO055Connection(bno); // this will return event as well 
  return event;

}



