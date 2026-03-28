#include <Arduino.h>
#include <iostream>
#include <arm_math.h>
#include <SD.h>

#include "driverSD.hpp"
#include "motorDriver.hpp"
#include "Constants.hpp"

motorDriver waterMotor  = motorDriver(kWaterMotorIn1, kWaterMotorIn2, kWaterMotorSleep);

float landedStartTime  = 0;
float waterMotorStartTime = 0;

bool waterDispensed = false; //will be true once water dipenses once
bool waterGone = false; // will be true after

void setup(){
 //set up serial
  Serial.begin(9600);
  
  //starting ledpin
  pinMode(ledPin, OUTPUT);

  //intializing the motors inside the setup
  waterMotor  = motorDriver(kWaterMotorIn1, kWaterMotorIn2, kWaterMotorSleep);

  //setting pins to low so they dont float
  waterMotor.stopMotorWithCoast();

  Serial.println("Configured Water Motor");
  delay(2000);

  waterMotor.moveMotorBackward(1); //starting water motor
  landedStartTime = millis();
  waterMotorStartTime = millis();

}

void loop(){

    //if the water motor has started moving (triggers the water start time to be  saved number) and has been moving for more tahn 90 seconds, stopping the water motor and setting the water dispensed as true
    if(waterMotorStartTime!=0 && millis() - waterMotorStartTime  >= 90000){
      waterDispensed = true;
      waterGone = true;
      waterMotor.stopMotorWithCoast();
      Serial.println("Water motor stopped");
    }
    else{
    Serial.println("Moving Water pump...");
    Serial.println(millis() - waterMotorStartTime);
    }

    delay(200);


    //if the different in start time and the current time is larger than 15 mintutes, stopping all mototes, finishing and saving soil logging, then setting landedFinalized to true to stop all functions
    if (millis() - landedStartTime  >= kLandedTimeoutMs) {
      Serial.println("landed timeout reached, stopping all motors");
      waterMotor.stopMotorWithCoast();
      while(1){
        Serial.println("time elapsed...");
        delay(20000);
      }
    }


    
}