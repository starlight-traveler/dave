//soil data reading and water pump activation
//bidirectional orientation
#include <Arduino.h>
#include <iostream>
#include <arm_math.h>

#include "BNO055.hpp"
#include "ICM20649.hpp"
#include "motorDriver.hpp"
#include "Constants.hpp"

motorDriver orientMotor  = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep);

//creating sensor objects
Adafruit_BNO055  bno =  Adafruit_BNO055(55, 0x28);
Adafruit_ICM20649 icm;

/*This function will return the absolute value of val*/
float32_t absScalarF32(float32_t val) {
  float32_t src[1] = {val};
  float32_t dst[1] = {0.0f};
  arm_abs_f32(src, dst, 1);
  return dst[0];
}

/*This function will return true if the acceleration value inputted is finite and the absolute value is under 1500 m/s^2*/
bool isFiniteAndReasonable(float32_t value) {
  return __builtin_isfinite(value) && absScalarF32(value) <= kAccelSanityMaxAbsMs2;
}


/*This function will return true if the acceleration componants inputted are all finite and reasonable*/
bool accelVectorIsSane(float32_t x, float32_t y, float32_t z) {
  return isFiniteAndReasonable(x) && isFiniteAndReasonable(y) && isFiniteAndReasonable(z);
}

  /*This function will check the orientation of the nosecone and turn the nosecone accordingly if not aligned. */
void checkOrientationStep() {
  //getting the gravity vector from bno
    const imu::Vector<3> gravity =  bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    const float32_t gravityX = gravity.x();
    const float32_t gravityY = gravity.y();
    const float32_t gravityZ = gravity.z();

 //if the gravity vector inputted is not same, then start a timer to log how long the program has gone wihtout valid gravity data.
  if (!accelVectorIsSane(gravityX, gravityY, gravityZ)) {
    static elapsedMillis invalidGravityLogTimer;
    //only prints out message every 500 ms so not bombarded with error messages
    if (invalidGravityLogTimer >= kOrientationLogPeriodMs) {
      invalidGravityLogTimer = 0;
      Serial.println("[LANDED][ORIENT] gravity invalid, skipping orientation step");
    }
    //if not valid, stopping motor and returning to main program
    orientMotor.stopMotorWithCoast();
    return;
  }

  //if the y axis is within the needed range, stopping the motor and setting the orientMotor to true as it was aligned properly and returning to main program
  if(gravityY>kOrientationAlignedYMax){
    orientMotor.moveMotorForward(1);
  }
  else if (gravityY<kOrientationAlignedYMin){
    orientMotor.moveMotorBackward(1);
  }
  else
    orientMotor.stopMotorWithCoast();
    Serial.println("[LANDED][ORIENT] y-axis aligned -> orientation complete");
    return;
  }


void setup(){
//set up serial
  Serial.begin(9600);
  orientMotor  = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep);

}

void loop(){
    checkOrientationStep();
    delay(2000);
    
}