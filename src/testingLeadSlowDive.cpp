#include <Arduino.h>
#include "Constants.hpp"
#include "motorDriver.hpp"

//auger, leadscrew, limit switches

motorDriver leadScrewMotor  = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);
motorDriver augerMotor = motorDriver(kAugerControlPin);

int32_t topHits = 0; //amount of times the top limit switch is hit
int32_t bottomHits = 0; // amount of times the bottom limit switch is hit

int dutyCycle = 1;

//motor driver logic booleans
  bool leadScrewFullyExtended = false;
  bool stationaryAugerSpinActive = false;
  bool landedFinalized = false;
  
//limit switch logic booleans
  bool upperSwitchPressed = false;
  bool lowerSwitchPressed = false;
  bool lastUpperSwitchPressed = false;
  bool lastLowerSwitchPressed = false;
  bool upperStateChange = false;
  bool lowerStateChange = false;
  bool isMovingUp = false;
  bool waterDispensed = false; //will be true once water dipen
  bool stationarySpinComplete = false;

  elapsedMillis augerSpinTimer;

void enterLandedState() {
  upperSwitchPressed = false;
  lowerSwitchPressed = false;
  lastUpperSwitchPressed = false;
  lastLowerSwitchPressed = false;
  upperStateChange = false;
  lowerStateChange = false;
  stationarySpinComplete = false;

  topHits  = 0;
  bottomHits  = 0;

  leadScrewFullyExtended  = false;
  stationaryAugerSpinActive  = false;
  landedFinalized  = false;

}



void setup(){

    Serial.begin(9600);

    leadScrewMotor  = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);
    augerMotor = motorDriver(kAugerControlPin);

    delay(200);
    Serial.print("motors begun...");

    pinMode(13, OUTPUT);
    pinMode(kUpperLimitSwitchPin, INPUT_PULLUP);
    pinMode(kLowerLimitSwitchPin, INPUT_PULLUP);

    augerMotor.stopMosfet();
    leadScrewMotor.stopMotorWithCoast();
    
}

void loop(){
     //switch logic
    upperSwitchPressed = (digitalReadFast(kUpperLimitSwitchPin) == HIGH); //true if the upper limit switch is currently pressed
    lowerSwitchPressed = (digitalReadFast(kLowerLimitSwitchPin) == HIGH); //true if the lower limit switch is currently pressed

    upperStateChange = (upperSwitchPressed && !lastUpperSwitchPressed); //when the switch is pressed but wasnt before
    lowerStateChange = (lowerSwitchPressed && !lastLowerSwitchPressed);

    lastUpperSwitchPressed = upperSwitchPressed; //making last the current for next loop
    lastLowerSwitchPressed = lowerSwitchPressed;

    //if the upper limit switch is pressed and was not before, and the number of top hits is zero, stopping the lead screw motors and increasing number of top hits
    if (upperStateChange && topHits  == 0) { 
      Serial.println("upper switch pressed");
      leadScrewMotor.stopMotorWithCoast();
      topHits++; 
    }

    //if the number of top hits is zero, moving the lead screw forward (up) in order to reach the upper limit switch
    if (topHits  == 0 ) {
      Serial.println("Retracting lead screw to find top switch");
      leadScrewMotor.moveMotorForward(dutyCycle);
    }

    //if the bottom hits is zero and the top hits is one, means the lead screw is at the top, so starting the auger turning, moving the leadscrew downwards, and setting isMovingUp to false
    if (bottomHits  == 0 && topHits  == 1) {
      Serial.println("top reached once, drilling down and spinning auger");
      augerMotor.moveMosfet();
      leadScrewMotor.moveMotorBackward(dutyCycle);
      isMovingUp = false;
    }

    //if the lead screw is moving downwards, bottom hits is currently zero, and the lower limit switch was hit, increasing the number of bottom hits
    if (isMovingUp==false && bottomHits  == 0 && lowerStateChange) {
      Serial.println("Lower switch pressed");
      bottomHits ++; //hit the bottom, wasnt there before, increase bottom hits
    }

    //if the top hits is one and bottom hits is one, and the auger is not stationarily moving, stopping the lead screw, setting leadScrewFullyExtended to true, and setting stationaryAugerSpinActive, then starting the auger spin timer
    if (bottomHits  == 1 && topHits  == 1 && stationarySpinComplete == false) {
      if (!stationaryAugerSpinActive ) {
        Serial.println("bottom reached, stop lead screw, hold auger spin for 30s");
        leadScrewMotor.stopMotorWithCoast();
        leadScrewFullyExtended  = true;
        stationaryAugerSpinActive  = true; //stop extending, keep spinning
        augerSpinTimer  = 0;
        //Serial.println(augerSpinTimer);
      }
      Serial.println(augerSpinTimer);
    }

    //if the lead screw is fully exteneded, bottom and top hits are one, the auger is spinning stationary, and the alloted 10s as passed, setting augerSpinStationary to false, leadScrewFullyExtended to false,
    // and moving the lead screw up, then setting isMovingUp to ture
    if (leadScrewFullyExtended  && bottomHits  == 1 && topHits  == 1 && stationaryAugerSpinActive  && augerSpinTimer  >= 30000) {
      Serial.println("Auger spin window complete, retracting lead screw");
      stationaryAugerSpinActive  = false;
      leadScrewFullyExtended  = false;
      stationarySpinComplete = true;
      leadScrewMotor.moveMotorForward(dutyCycle);
      isMovingUp = true;
    }

    //if the lead screw is moving up, the upper limit switch is hit, and the bottom and top hits are one, 
    //increasing top hits to 2 and stopping the lead screw, then setting isMovingUp to false
    if (isMovingUp && upperStateChange && bottomHits  == 1 && topHits  == 1) {
      Serial.print("Up Switch pressed while retracting");
      leadScrewMotor.stopMotorWithCoast();
      isMovingUp = false;
      topHits ++;
    }

    if (topHits  == 2 && bottomHits  == 1) {
      Serial.println("cycle complete, resetting counters to start the process again (no water pump this time)");
      enterLandedState();
    }

    delay(50);

}