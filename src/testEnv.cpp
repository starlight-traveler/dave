#include <Arduino.h>
#include "Constants.hpp"

void setup(){
    Serial.begin(9600);
}

void loop(){
    Serial.println("Loop running...");
    delay(2000);
}
