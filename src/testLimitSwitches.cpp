
#include <Arduino.h>
namespace{
int upperPin = 2;
int lowerPin = 3;
int led = 13;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  pinMode(upperPin, INPUT_PULLUP);
  pinMode(lowerPin, INPUT_PULLUP);

}

void loop() {
  // put your main code here, to run repeatedly:
 // Serial.print(digitalRead(upperPin));
  Serial.println("loop");
  
  if (digitalRead(upperPin) == HIGH){
    Serial.println("Upper Hit");
    digitalWrite(led, HIGH);
    delay(20);
    digitalWrite(led, LOW);
    delay(20);
    digitalWrite(led, HIGH);
    delay(20);
    digitalWrite(led, LOW);
  }

  if (digitalRead(lowerPin) == HIGH){
    Serial.println("Lower Hit");
    digitalWrite(led, HIGH);
    delay(50);
    digitalWrite(led, LOW);
    delay(50);
  }

  delay(50);

}

}