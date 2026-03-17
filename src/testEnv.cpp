#include <Arduino.h>
namespace{
void setup(){
    Serial.begin(9600);
}

void loop(){
    Serial.println("Loop running...");
    delay(2000);
}

}