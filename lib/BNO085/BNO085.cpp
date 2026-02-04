#include "BNO085.hpp"

//this function will check the connection to the bno, and try again if it fails
void checkBNO085Connection(Adafruit_BNO08x *bno){
  sh2_SensorValue_t event;
  if(!bno->getSensorEvent(&event));
    bno->begin_I2C();
  }

//this function will set up the bno
void setupBNO085(Adafruit_BNO08x *bno){
    if (!bno->begin_I2C()) {
      checkBNO085Connection(bno);
    }
}

//this function will get the orientation from the bno, returning the overall event
sh2_SensorValue_t getBNO085Event(Adafruit_BNO08x *bno){
    sh2_SensorValue_t event;
    if(!bno->getSensorEvent(&event)){
      checkBNO085Connection(bno); //cant return int, so just check connection if it fails
    }
    return event;
}

