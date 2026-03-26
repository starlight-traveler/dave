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

driverSD soilData = driverSD(kSoilDataBufferSize);

void startSoilLoggingIfNeeded() {
  if (soilData .getCurrentIndex() == 0) {
    Serial.print("[FC][LANDED] opening soil log file: ");
    Serial.print(soilData .getCurrentFileName());
    dataFile  = SD.open(soilData .getCurrentFileName(), FILE_WRITE);
    Serial.print(dataFile  ? F("[FC][LANDED] file open OK") : F("[FC][LANDED] file open FAILED"));
  }
}

void finishSoilLogging() {
  Serial.print("[FC][LANDED] finalizing soil log");
  soilData .increaseCurrentIndexBy(-1);
  soilData .printSoilDataToFile(dataFile );
}


}

void setup(){
    Serial.begin(9600);
    modbus.begin(9600, SERIAL_8N1);
    pinMode(RS485_DIR_PIN, OUTPUT);
    digitalWrite(RS485_DIR_PIN, LOW);

    Serial.println("Soil sensor Modbus reader started");

    //starting ledpin
    pinMode(13, OUTPUT);


      if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD mount failed, halting and outputting light");
    while (1) {
        digitalWrite(ledPin, HIGH);
    }
    }

    soilData.begin(testSoilRoot);
}


void loop(){
    startSoilLoggingIfNeeded();

    static elapsedMillis soilTimer; //starts the timer
    if (soilTimer > 2000) { // Read every 2 seconds
      float32_t raw;

      if (readRegister(0x0006, raw, RS485_DIR_PIN, SLAVE_ID, modbus)) {
        pH  = raw/100.0f;
      }
      if (readRegister(0x0015, raw, RS485_DIR_PIN, SLAVE_ID, modbus)) {
        electricalConductivity  = raw;

      }
      if (readRegister(0x001E, raw, RS485_DIR_PIN, SLAVE_ID, modbus)) {
        nitrogenMgKg  = raw;
      }

      soilData .addSoilSensorData(nitrogenMgKg , pH , electricalConductivity , dataFile);
      soilTimer = 0;
    }
    
}