#include <Arduino.h>
#include "FlightController.h"
#include "motorDriver.hpp"
#include "BNO085.hpp"
#include "Config.h"

// // TODO: X is negative, Z is positive, equal but opposite magnitudes

// /** @brief Single global controller instance. */
// static FlightController controller(Serial1);

// /** @brief Arduino setup hook. */
// void setup() {
//   controller.begin();
// }

// /** @brief Arduino loop hook. */
// void loop() {
//   controller.update();
// }


//----------------------------------------- for testing BNO -----------------------------------------

Adafruit_BNO08x bno;
sh2_SensorValue_t event;

void setup(){
  Serial.begin(9600);

  setupBNO085(&bno);
  checkBNO085Connection(&bno);

 
  // Serial.print("BNO Acceleration x: ");
  // Serial.println(event.un.accelerometer.x);

  // Serial.print("BNO Acceleration y: ");
  // Serial.println(event.un.accelerometer.y);

  // Serial.print("BNO Acceleration z: ");
  // Serial.println(event.un.accelerometer.z);

}


void loop(){
  
   event = getBNO085Event(&bno);
  Serial.print("BNO Acceleration x: ");
  Serial.println(event.un.accelerometer.x);

  Serial.print("BNO Acceleration y: ");
  Serial.println(event.un.accelerometer.y);

  Serial.print("BNO Acceleration z: ");
  Serial.println(event.un.accelerometer.z);

  delay(2000);


}


//----------------------------------------- for testing orient motor -----------------------------------------
// not pwm
  motorDriver orientMotor_;

  void setup(){
    orientMotor_ = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep);

  }

  void loop(){
    delay(5000);
    orientMotor_.moveMotorForward(1);
    delay(5000);
    orientMotor_.stopMotorWithCoast();

  }

//----------------------------------------- for testing auger motor -----------------------------------------
// not pwm
  motorDriver augerMotor_;

  void setup(){
    augerMotor_ = motorDriver(gpioAuger);

  }

  void loop(){
    delay(5000);
    augerMotor_.moveMosfet();
    delay(5000);
    augerMotor_.stopMosfet();

  }

  //----------------------------------------- for testing lead screw motor -----------------------------------------
  // pwm

  constexpr int32_t kUpperLimitSwitchPin = 2;
  constexpr int32_t kLowerLimitSwitchPin = 3;
  int32_t topHits_ = 0;
  int32_t bottomHits_ = 0;
  bool upperSwitchPressed_ = false;
  bool lowerSwitchPressed_ = false;
  bool leadScrewFullyExtended_ = false;
  elapsedMillis switchPollTimer_;
  motorDriver leadScrewMotor_;

  void pollLimitSwitches() {
    if (switchPollTimer_ < 5) {
      return;
    }
    switchPollTimer_ = 0;
    upperSwitchPressed_ = (digitalReadFast(kUpperLimitSwitchPin) == LOW);
    lowerSwitchPressed_ = (digitalReadFast(kLowerLimitSwitchPin) == LOW);
  }

    void setup(){
      leadScrewMotor_ = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);
      pinMode(kUpperLimitSwitchPin, INPUT_PULLUP);
      pinMode(kLowerLimitSwitchPin, INPUT_PULLUP);

  }

  void loop(){
    pollLimitSwitches();

    if (topHits_ == 0) {
      leadScrewMotor_.moveMotorBackward(0.5f);
    }

    if (upperSwitchPressed_) {
      topHits_++;
    }

    if (bottomHits_ == 0 && topHits_ == 1) {
      //augerMotor_.moveMsosfet();
      leadScrewMotor_.moveMotorForward(0.5f);
    }

    if (lowerSwitchPressed_) {
      bottomHits_++;
    }

    if (bottomHits_ == 1 && topHits_ == 1) {
        leadScrewMotor_.stopMotorWithCoast();
        leadScrewFullyExtended_ = true;
      }
    
    if (leadScrewFullyExtended_ && bottomHits_ == 1 && topHits_ == 1) {
      leadScrewFullyExtended_ = false;
      leadScrewMotor_.moveMotorBackward(0.5f);
    }

    if (upperSwitchPressed_) {
      leadScrewMotor_.stopMotorWithCoast();
      topHits_++;
    }

    if (topHits_ == 2 && bottomHits_ == 1) {
      topHits_ = 0;
      bottomHits_ = 0;
    }
  }

  //----------------------------------------- for testing water motor -----------------------------------------
  motorDriver waterMotor_;
    void setup(){
      waterMotor_ = motorDriver(kWaterMotorIn1, kWaterMotorIn2, kWaterMotorSleep);

  }

  void loop(){
    delay(5000);
    waterMotor_.moveMotorForward(1);
    delay(5000);
    waterMotor_.stopMotorWithCoast();
  }


  //----------------------------------------- for testing orientation motor w bno -----------------------------------------
  // checing orientation motor moving with bno acceleration vectors
  motorDriver orientationMotor;
  Adafruit_BNO08x bno;
  sh2_SensorValue_t event;

void setup() {
  Serial.begin(9600);
  setupBNO085(&bno);
  checkBNO085Connection(&bno);

  orientationMotor = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep, kOrientMotorNFault);
  
  // pinMode(kOrientMotorIn1, INPUT);
  // pinMode(kOrientMotorIn2, INPUT);
  // pinMode(kOrientMotorSleep, INPUT);

  // Serial.begin(9600);
  // Serial.print("Orientation object");
  // digitalWrite(kOrientMotorSleep, HIGH);
  
}

void loop() {

    // digitalWrite(kOrientMotorIn1, LOW);
    // digitalWrite(kOrientMotorIn2, LOW);
    //delay(5000);
    // digitalWrite(kOrientMotorIn1, HIGH);
    // digitalWrite(kOrientMotorIn2, LOW);

    orientationMotor.moveMotorForward(1.0f);

    delay(5000);
    
    event = getBNO085Event(&bno);

    if (event.un.gravity.z <= -9.0f && event.un.gravity.z >= -11.0f) {
      orientationMotor.stopMotorWithCoast();
      delay(5000);
    }
      
    // if (event.un.gravity.x > 0) {
    //   orientationMotor.moveMotorForward(1.0f);
    // } else {
    //   orientationMotor.moveMotorBackward(1.0f);
    // }
}




