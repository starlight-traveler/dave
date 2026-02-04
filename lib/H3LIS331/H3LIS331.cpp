#include "H3LIS331.hpp"


//this function checks the connection to the H3LIS331. If it fails, it will try to begin again
void checkH3LIS331Connection(Adafruit_H3LIS331 *lis){
    if(lis->getDeviceID() != LIS331_CHIP_ID){
        lis->begin_I2C();
    }
}

//this function well set the connection for the H3LIS331
void setupH3LIS331(Adafruit_H3LIS331 *lis){
    if (!lis->begin_I2C()) {
      checkH3LIS331Connection(lis);
    }

    //setting lis range
    lis->setRange(H3LIS331_RANGE_400_G);
}

//this function will get the acceleration from the H3LIS331, returning the overall event
sensors_event_t getH3LIS331Accel(Adafruit_H3LIS331 *lis){
    if (!lis->begin_I2C()) {
      checkH3LIS331Connection(lis);
    }

    //getting acceleration data
    sensors_event_t a;
    lis->getEvent(&a);
    return a;
}
