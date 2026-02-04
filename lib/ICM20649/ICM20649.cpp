#include "ICM20649.hpp"

//this function will check the icm connection, and try to begin again if it fails
void checkICMConnection(Adafruit_ICM20649 *icm){
  if(!icm->getEvent(NULL, NULL, NULL, NULL)){ //should still answer true if it connects
    icm->begin_I2C();
  }
}

//this function will set up the icm
void setupICM20649(Adafruit_ICM20649 *icm){
    if (!icm->begin_I2C()) {
      checkICMConnection(icm);
    }

    //setting icm range
    icm->setAccelRange(ICM20649_ACCEL_RANGE_30_G);
}

//this function will get the acceleration from the icm, returning the overall event
sensors_event_t getICM20649Accel(Adafruit_Sensor *accel, Adafruit_ICM20649 *icm){
    if (!icm->begin_I2C()) {
      checkICMConnection(icm);
    }

    //getting acceleration data
    sensors_event_t a;
    accel->getEvent(&a);
    return a;
}