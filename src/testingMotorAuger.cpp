#include <Arduino.h>
#include <iostream>
#include <arm_math.h>
#include <SD.h>

#include "driverSD.hpp"
#include "motorDriver.hpp"
#include "Constants.hpp"

motorDriver augerMotor = motorDriver(41, 12, 40, true);

void setup()
{
    Serial.begin(9600);
    augerMotor.stopPololu();
}

void loop()
{
    augerMotor.moveBackwardPololu();
    // backward = pull dirt up into chamber
    int current = augerMotor.getCurrentSensePololu();
    Serial.println(current);
    //augerMotor.moveForwardPololu();
    // forward = run auger in reverse, push dirt out
    //delay(5000);
    // supposedly max current sense is at ~250mV (50mV + 10mV/A * 20A)
    // analog read will supposedly return ~77 if current is at max
    delay(200);
}
