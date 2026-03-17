
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

// int motorPin = 10;
int in1 = 24;
int in2 = 25;
// int sleep = 38;
int ledOuputPin = 13;
int upperPin = 2;
int lowerPin = 3;

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
 // pinMode(sleep, OUTPUT);
  //pinMode(motorPin, OUTPUT);
  pinMode(ledOuputPin, OUTPUT);
  pinMode(upperPin, INPUT_PULLUP);
  pinMode(lowerPin, INPUT_PULLUP);


  // digitalWrite(sleep, HIGH);

  //digitalWrite(augerPin, LOW);
  digitalWrite(ledOuputPin, HIGH);
  delay(50);
  digitalWrite(ledOuputPin, LOW);
  delay(500);

}

void loop(){

  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

if(digitalRead(upperPin) == HIGH){
  Serial.println("hit");
  digitalWrite(in2, LOW);
  digitalWrite(in1, LOW);

  while(1){}
}

delay(2000);

}
