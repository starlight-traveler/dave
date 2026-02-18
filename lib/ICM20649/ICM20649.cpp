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
sensors_event_t getICM20649Accel(Adafruit_ICM20649 *icm){
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  // Get all data at once efficiently
  icm->getEvent(&accel, &gyro, &temp);
  return accel; // only need excel
}
