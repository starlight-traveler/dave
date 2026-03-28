#include <Arduino.h>
#include <iostream>
#include <arm_math.h>
#include <SD.h>

#include "Constants.hpp"
#include "motorDriver.hpp"

motorDriver leadScrewMotor  = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);

void movingUp(){
    digitalWrite(kLeadScrewMotorIn1, HIGH);
    digitalWrite(kLeadScrewMotorIn2, LOW);

  if(digitalRead(kUpperLimitSwitchPin) == HIGH){
    Serial.println("hit");
    digitalWrite(kLeadScrewMotorIn2, LOW);
    digitalWrite(kLeadScrewMotorIn1, LOW);

    delay(5000);

  }

}

void movingDown(){
    digitalWrite(kLeadScrewMotorIn1, LOW);
    digitalWrite(kLeadScrewMotorIn2, HIGH);

  if(digitalRead(kLowerLimitSwitchPin) == HIGH){
    Serial.println("hit");
    digitalWrite(kLeadScrewMotorIn2, LOW);
    digitalWrite(kLeadScrewMotorIn1, LOW);

    delay(5000);


  }

}

void setup(){
    Serial.begin(9600);
    Serial.println("Started Serial");

    leadScrewMotor  = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);
    leadScrewMotor.stopMotorWithCoast();
    
    delay(2000);


}

void loop(){
    leadScrewMotor.moveMotorForward(1);
    delay(50);

}

