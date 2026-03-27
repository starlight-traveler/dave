#include <Arduino.h>
#include <iostream>
#include <arm_math.h>
#include <SD.h>

#include "driverSD.hpp"
#include "motorDriver.hpp"
#include "SoilSensor.hpp"
#include "Constants.hpp"

//soil data reading and water pump activation
//for comunication with soil sensor
#define RS485_DIR_PIN 27
#define SLAVE_ID 0x01

HardwareSerial &modbus = Serial2;
motorDriver waterMotor  = motorDriver(kWaterMotorIn1, kWaterMotorIn2, kWaterMotorSleep);
File dataFile;

  float landedStartTime  = 0;
  float waterMotorStartTime = 0;

//varaibles to hold fetched soil sensor data
  float32_t nitrogenMgKg = 0.0f;
  float32_t pH = 0.0f;
  float32_t electricalConductivity = 0.0f;

  driverSD soilData = driverSD(kSoilDataBufferSize); //creating soil data object w correct buffer size. 

  bool waterDispensed = false; //will be true once water dipenses once
  bool waterGone = false; // will be true after

  elapsedMillis soilTimer;


  /*This function will start or restart the soil logging if the current index of the buffer is zero. It will set the SD datafile to soil log name.*/
void startSoilLoggingIfNeeded() {
  if (soilData .getCurrentIndex() == 0) {
    Serial.print("[LANDED] opening soil log file: ");
    dataFile  = SD.open(soilData .getCurrentFileName(), FILE_WRITE);
    Serial.print(dataFile  ? F("[LANDED] file open OK") : F("[LANDED] file open FAILED"));
  }
}

/*This fucntion will print the soil data to SD, no matter if the buffer is full or not because the program is done running.*/
void finishSoilLogging() {
  Serial.print("[FC][LANDED] finalizing soil log");
  soilData.increaseCurrentIndexBy(-1);
  soilData.printSoilDataToFile(dataFile );
}




void setup(){
 //set up serial
  Serial.begin(9600);

  //set up modbus for soil sensor
  modbus.begin(9600, SERIAL_8N1);

  //configure rs485 pin and initialze to low
  pinMode(RS485_DIR_PIN, OUTPUT);
  digitalWrite(RS485_DIR_PIN, LOW);

  Serial.println("Soil sensor Modbus reader started");

  
  //starting ledpin
  pinMode(ledPin, OUTPUT);

  //intializing the motors inside the setup
  waterMotor  = motorDriver(kWaterMotorIn1, kWaterMotorIn2, kWaterMotorSleep);

  //setting pins to low so they dont float
  waterMotor.stopMotorWithCoast();
  

  //begin the sd card, shining led light if cannot find or connect to
  if (!SD.begin(BUILTIN_SDCARD)) {
  Serial.println("SD mount failed, halting and outputting light");
  while (1) {
    digitalWrite(ledPin, HIGH);
  }
  }

  //setting up the file names for the flight data and soil data so can be logged properly
  soilData.begin(kSoilDataRoot);

  waterMotor.moveMotorForward(0.5); //starting water motor
  landedStartTime = millis();

}

void loop(){
    
    startSoilLoggingIfNeeded();
    static elapsedMillis soilTimer; //starts the timer for soilTimer
    if (soilTimer > 2000) { // Read every 2 seconds
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
    
    //if the water motor has started moving (triggers the water start time to be  saved number) and has been moving for more tahn 60 seconds, stopping the water motor and setting the water dispensed as true
    if(waterMotorStartTime!=0 && millis() - waterMotorStartTime  >= kWaterTimeoutMs){
      waterDispensed = true;
      waterGone = true;
      waterMotor.stopMotorWithCoast();

    }


    //if the different in start time and the current time is larger than 15 mintutes, stopping all mototes, finishing and saving soil logging, then setting landedFinalized to true to stop all functions
    if (millis() - landedStartTime  >= kLandedTimeoutMs) {
      Serial.println("landed timeout reached, stopping all motors");
      waterMotor.stopMotorWithCoast();

      finishSoilLogging();
      while(1){
        Serial.println("time elapsed...");
        delay(20000);
      }
    }

    
}