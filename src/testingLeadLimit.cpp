
#include <Arduino.h>
#include "Constants.hpp"

namespace{

int ledOuputPin = 13;
}

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

    movingUp();

  }

}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(kLeadScrewMotorIn1, OUTPUT);
  pinMode(kLeadScrewMotorIn2, OUTPUT);
 // pinMode(sleep, OUTPUT);
  //pinMode(motorPin, OUTPUT);
  pinMode(ledOuputPin, OUTPUT);
  pinMode(kUpperLimitSwitchPin, INPUT_PULLUP);
  pinMode(kLowerLimitSwitchPin, INPUT_PULLUP);


  // digitalWrite(sleep, HIGH);

  //digitalWrite(augerPin, LOW);
  digitalWrite(ledOuputPin, HIGH);
  delay(50);
  digitalWrite(ledOuputPin, LOW);
  delay(500);

}

void loop(){

    digitalWrite(kLeadScrewMotorIn1, LOW);
    digitalWrite(kLeadScrewMotorIn2, HIGH);

delay(2000);

}
