#include <Arduino.h>
#include <iostream>
#include <arm_math.h>
#include <SD.h>

#include "driverSD.hpp"
#include "motorDriver.hpp"
#include "Constants.hpp"

motorDriver augerMotor = motorDriver(kAugerControlPin);

void setup(){
    Serial.begin(9600);
    Serial.println("Started Serial");

    augerMotor = motorDriver(kAugerControlPin);
    augerMotor.stopMosfet();
    Serial.print("Pulling low...");

    delay(2000);

}

void loop(){
    augerMotor.moveMosfet();
    Serial.print("moving forward");

    delay(6000);

    augerMotor.stopMosfet();

    delay(6000);

}
