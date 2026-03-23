
#include <Arduino.h>
#include "Constants.hpp"

// int in1 = 28;
// int in2 = 29;
//28, 29
// int sleep = 38;


//10, 
//14
//24 and 25
//38, 37, 36

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

    movingDown();

    //while(1){}
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

   // while(1){}
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

  movingDown();

delay(2000);

}
