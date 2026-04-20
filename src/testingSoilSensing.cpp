#include <Arduino.h>
#include <iostream>
#include <arm_math.h>
#include <SD.h>

#include "driverSD.hpp"
#include "SoilSensor.hpp"
#include "Constants.hpp"

namespace{

#define RS485_DIR_PIN 21
#define SLAVE_ID 0x01

HardwareSerial &modbus = Serial1;

const char* testSoilRoot = "TestingSoil";
File dataFile;

float32_t nitrogenMgKg = 0.0f;
float32_t pH = 0.0f;
float32_t electricalConductivity = 0.0f;

float startTime = 0;

driverSD soilData = driverSD(kSoilDataBufferSize);

void startSoilLoggingIfNeeded() {
  if (soilData .getCurrentIndex() == 0) {
    //Serial.print("Opening soil log file: ");
    dataFile  = SD.open(soilData .getCurrentFileName(), FILE_WRITE);
    //Serial.print(dataFile  ? F("File open OK") : F("File open FAILED"));
  }
}

void finishSoilLogging() {
  Serial.print("Finalizing soil log");
  soilData.increaseCurrentIndexBy(-1);
  soilData.printSoilDataToFile(dataFile );
}

void getAndLogSoilData(){
    static elapsedMillis soilTimer; //starts the timer for soilTimer
    if (soilTimer > kSoilDataRequestPeriodMs) { // Read every 30 milliseconds seconds
      float32_t raw;

      //getting pH, eletrical conductivity, and nitrogen content
      if (readRegister(0x0006, raw, RS485_DIR_PIN, SLAVE_ID, modbus)) {
        pH  = raw/100.0f;
      }
      if (readRegister(0x0015, raw, RS485_DIR_PIN, SLAVE_ID, modbus)) {
        electricalConductivity  = raw;

      }
      if (readRegister(0x001E, raw, RS485_DIR_PIN, SLAVE_ID, modbus)) {
        nitrogenMgKg  = raw;
      }

      //adding data to the soil data file
      soilData.addSoilSensorData(nitrogenMgKg , pH , electricalConductivity , dataFile);

      //resetting the soilTimer
      soilTimer = 0;
    }
  }

}

void setup(){
    Serial.begin(9600);
    modbus.begin(9600, SERIAL_8N1);
    pinMode(RS485_DIR_PIN, OUTPUT);
    digitalWrite(RS485_DIR_PIN, LOW);

    Serial.println("Soil sensor Modbus reader started");
    delay(2000);

    //starting ledpin
    pinMode(13, OUTPUT);


    if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD mount failed, halting and outputting light");
    delay(2000);
    while (1) {
        digitalWrite(ledPin, HIGH);
    }
    }

    soilData.begin(testSoilRoot);
    Serial.print("Beginning running...");

    startTime = millis();
}


void loop(){
    startSoilLoggingIfNeeded();
    getAndLogSoilData();

    if(millis() - startTime>60000){
      finishSoilLogging();
      Serial.print("Done");
      while(1){}
    }

  }
