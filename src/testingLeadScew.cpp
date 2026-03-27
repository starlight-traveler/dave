#include <Arduino.h>
#include <iostream>
#include <arm_math.h>
#include <SD.h>

#include "Constants.hpp"
#include "motorDriver.hpp"

motorDriver leadScrewMotor  = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);

void setup(){
    Serial.begin(9600);
    Serial.println("Started Serial");

    leadScrewMotor  = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);
    
    delay(2000);


}

void loop(){
    leadScrewMotor.moveMotorForward(0.5);

    delay(6000);

    leadScrewMotor.moveMotorBackward(0.5);

    delay(6000);

}

