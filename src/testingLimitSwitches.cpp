
#include <Arduino.h>
#include "Constants.hpp"
namespace{

int led = 13;

}

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  pinMode(kUpperLimitSwitchPin, INPUT_PULLUP);
  pinMode(kLowerLimitSwitchPin, INPUT_PULLUP);

}

void loop() {
  // put your main code here, to run repeatedly:

  // if (digitalRead(kUpperLimitSwitchPin) == HIGH){
  //   //Serial.println("Upper Hit");
  //   digitalWrite(led, LOW);
  //   Serial.println("Upper is high, hit");
  // }
  // else{
  //   Serial.println("Upper is low");
  // }

  if (digitalRead(kLowerLimitSwitchPin) == HIGH){
    Serial.println("Lower is high, hit");
  }
  else{
    Serial.println("Lower is low");
  }

  delay(200);

}
