#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
//drivers
#include "BMP390.hpp"
#include "ICM20649.hpp"
#include "BNO085.hpp"
#include "SoilSensor.h"
#include "H3LIS331.hpp"
//classes
#include "motorDriver.hpp"
#include <arm_math.h>
#include "driverSD.hpp"

//SCK and MISO are used on I2C as SCL and SDA respectively
#define BMP_SCK 19 //from kicad
#define BMP_MISO 18

//dont know if we can delete these, even tho we dont need them
#define BMP_MOSI 11 
#define BMP_CS 10

// defining RS485 pins and slave ID for soil sensor
#define RS485_DIR_PIN 2
#define SLAVE_ID 0x01

//setting up hardware serial for modbus communication for the soil sensor
  HardwareSerial &modbus = Serial1;

//defining limit switch pins from kiCad
  const int32_t upperLimitSwitchPin = 2;
  const int32_t lowerLimitSwitchPin = 3;

//initiazing the different motors according to their pins
  motorDriver orientMotor = motorDriver(27, 26, 25); //not PWM
  motorDriver augerMotor = motorDriver(31, 30, 29); // not PWM
  motorDriver leadScrewMotor = motorDriver(11, 12, -1); //no sleep pin  
  motorDriver waterMotor = motorDriver(9, 10, -1); //no sleep pin

//defining sensors
  Adafruit_BMP3XX bmp;
  Adafruit_BNO08x bno;
  //Adafruit_ICM20649 icm;
  Adafruit_H3LIS331 lis;
  //Adafruit_Sensor *accel;

//File obj for data writing and SD card to write to
  File dataFile;

//State enum for state machine
  enum flightState
  {
    PREFLIGHT,
    INFLIGHT,
    LANDED,
  };

// Initialize state
  flightState state = PREFLIGHT;

// this will hold the start time of the inflight state
  uint32_t inflightStartTime;

// time difference variable for inflight state to see if 3 mintues have passed
  uint32_t timeDiffInFLight;

//initizaling the variable to track 15 minutes after landing
  uint32_t landedStartTime;

//inintializing variables for soil data
  float32_t nitrogenPercentage;
  float32_t pH;
  float32_t eletricalConductivity;

//global variabels for bottom and top hits for limit switches
  int32_t topHits = 0;
  int32_t bottomHits = 0;

//number of times the change in altitude has to be below the threshold to count as landed
  int32_t altitudeBelowThresholdCount = 0;
  const int32_t altitudeThreshold = 5; //number of readings with necesary to change to count as landed
  const float32_t minChangeInAltitude = 0.3; //change has to be less than about a third of a meter to count as landed

//current altitude and previous altitude for landed detection
  float32_t currentAltitude = 0;
  float32_t previousAltitude = 0;

//set up objects that hold the file names for landed, flight, and soil data with the amount of columns they will need to store data
  driverSD flightData = driverSD("flightData", 8);
  //driverSD landedData = driverSD("landedData", 4);
  driverSD soilData = driverSD("soilData", 4);

//boolean to track if lead screw motor has stopped after auger deployment
  bool leadScrewFullyExtended = false;

//----- setup -----
void setup() {
  Serial.begin(9600);

  //setup sensors
    setupBMP(&bmp);
    setupBNO085(&bno);
    //setupICM20649(&icm);
    setupH3LIS331(&lis);

  //setting up the limit switch pins
    pinMode(upperLimitSwitchPin, INPUT); //they send signals to the teensy?
    pinMode(lowerLimitSwitchPin, INPUT);

  //set up SD card
    if (!SD.begin(BUILTIN_SDCARD))
    {
      while (1);
    }

  //setting up modbus serial communication
    modbus.begin(9600, SERIAL_8N1);

    pinMode(RS485_DIR_PIN, OUTPUT);
    digitalWrite(RS485_DIR_PIN, LOW);

}

//----- loop -----
void loop() {
  //check connection to all sensors
    checkBNO085Connection(&bno);
    checkBMP390Connection(&bmp); 
    checkH3LIS331Connection(&lis);

  if (state == PREFLIGHT){
    //check acceleration to be above threshold
      sensors_event_t a = getH3LIS331Accel(&lis);
      float32_t accelSquared = a.acceleration.x * a.acceleration.x 
      + a.acceleration.y * a.acceleration.y 
      + a.acceleration.z * a.acceleration.z;

    //must be greater than 4gs to change flight
      if(accelSquared > 1536.64){  //must be greater than 4gs to change flight
        state = INFLIGHT;
        inflightStartTime = millis();
        previousAltitude = getAltitude(&bmp); //getting initial altitude for landed detection
      }

    //check every 0.05 seconds
      delay(50);

  }
  else if(state == INFLIGHT){
    //get the current time
      timeDiffInFLight = millis() - inflightStartTime;

    //opening the file for writing flight data if it was closed
      if(flightData.getCurrentIndex() == 0){
        dataFile = SD.open(flightData.getCurrentFileName(), FILE_WRITE);
      }

    //adding data to buffer
      flightData.addFlightData(getH3LIS331Accel(&lis), getBNO085Event(&bno), getAltitude(&bmp), dataFile); 

    //getting the current altitude
      currentAltitude = getAltitude(&bmp);

    //getting change in altutude, and if it is below threshold increasing count
      if(abs(currentAltitude - previousAltitude) < minChangeInAltitude){
        altitudeBelowThresholdCount = altitudeBelowThresholdCount + 1;
      }
      else{
        altitudeBelowThresholdCount = 0; //resetting count if change in altitude is above threshold bc must be consucutive
      }

    //If 3 minutes have elapsed; change state
      if(altitudeBelowThresholdCount==5 || (timeDiffInFLight > 180000)) { //180000 milliseconds = 3 minutes
        state = LANDED;
        landedStartTime = millis(); //getting start time of landed state
        //saving the rest of in air data to the flightData file, then closing said file
        flightData.increaseCurrentIndexBy(-1);
        flightData.printFlightDataToFile(dataFile); //closes the file to be reopened for landed data
      }

      //making the current altitude past for next loop
        previousAltitude = currentAltitude;

      delay(30); // Make delay more frequent for more data every 0.03 seconds
  }
  else if (state == LANDED){
    //opening data file to store the soil data if it is closed
      if(soilData.getCurrentIndex() == 0){
        dataFile = SD.open(soilData.getCurrentFileName(), FILE_WRITE);
      }

    //check orientation and make sure the auger is facing the ground and move it to face the ground if not
      checkOrientation();
      
    //start spinning the lead screw motor so it hits the top limit switch, continue to do so until it hits the switch
      if(topHits == 0){
        leadScrewMotor.moveMotorBackward(0.5); //how fast do we want?
      }

    //seeing if the top limit switch is hit, and if no increasing the top hits count so the lead screw motor stops
    if(digitalRead(upperLimitSwitchPin) == HIGH){ // I have read that normally open switches read HIGH when not pressed and LOW when pressed, and are common for detecting movment
          topHits = topHits + 1;

        }
    
    //if the bottom hits are zero and the top hits are one, start spinning the auger motor and moving the lead screw down until the bottom limit switch is hit
      if (bottomHits == 0 && topHits == 1){
        augerMotor.moveMotorForward(1); //spin auger at half rate
        leadScrewMotor.moveMotorForward(0.5); //moving lead screw down and out of the rocket
        //seeing if the bottom limit switch is hit
      }

    //seeing if the bottom limit switch is hit, and if so  increasing the bottom hits count so the auger and lead screw motors stop
    if(digitalRead(lowerLimitSwitchPin) == HIGH){
          bottomHits = bottomHits + 1;
        }

    //if the bottom hits are one and the top hits are one, letting the auger motor continue to spin for 10 seconds while stopping the lead screw motor
      if(bottomHits == 1 && topHits == 1){
        leadScrewMotor.stopMotorWithCoast();
        leadScrewFullyExtended = true;
        delay(10000);
      }

    //beinging the auger back into the rocket by moving the lead screw motor back up until the top limit switch is hit again if the lead screw has stopped
      if (leadScrewFullyExtended && bottomHits == 1 && topHits == 1){
        leadScrewFullyExtended = false; //resetting for next time
        leadScrewMotor.moveMotorBackward(0.5); //moving lead screw back up into the rocket
      }
      
    //seeing if the top limit switch is hit again to stop the lead screw motor
      if(digitalRead(upperLimitSwitchPin) == HIGH){
          leadScrewMotor.stopMotorWithCoast();
          topHits = topHits + 1;
        }

    //if the leadScrew has stopped, the bottom hits are one and the top hits are two, turning on the water motor to deploy water
      if(!leadScrewFullyExtended && bottomHits == 1 && topHits == 2){
        waterMotor.moveMotorForward(0.75);
      }

    //reading in the soil data
      nitrogenPercentage = readRegister(0x001E, nitrogenPercentage, RS485_DIR_PIN, SLAVE_ID, modbus);
      pH = readRegister(0x0006, pH, RS485_DIR_PIN, SLAVE_ID, modbus);
      eletricalConductivity = readRegister(0x0015, eletricalConductivity, RS485_DIR_PIN, SLAVE_ID, modbus);

    //adding soil data to buffer
      soilData.addSoilSensorData(nitrogenPercentage, pH, eletricalConductivity, dataFile);

    //checking to see if 15 minutes have passed, and if so shutting everything off
      if(millis() - landedStartTime >= 900000){ //900000 milliseconds = 15 minutes
        //shutting off all of the motors
          augerMotor.stopMotorWithCoast();
          leadScrewMotor.stopMotorWithCoast();
          waterMotor.stopMotorWithCoast();
          orientMotor.stopMotorWithCoast();
          
          soilData.increaseCurrentIndexBy(-1);
          soilData.printSoilDataToFile(dataFile); //closes the file
          }
  
    //if the top hits are two and bottom hits are one resetting them to zero so the loop can continue
      if(topHits == 2 && bottomHits == 1){
        topHits = 0;
        bottomHits = 0;
      }

      state = LANDED; //just making sure the state is still in landed
  }

}


//this method will check the orientation of the module to make sure the auger is facing downwards. If it is not, it will move the orientaiton motor to make it face downwards.
void checkOrientation(){
  //if the z gravity vector is neagtive, and within a few degrees of whats needed, it will stop turning. If not, it will keep turning
    sh2_SensorValue_t event = getBNO085Event(&bno);

    while((event.un.gravity.z > -9.0 || event.un.gravity.z < -11.0)){ //hen gravity is about -10, it should be facing downwards
      //if the x gravity vector is positive, move motor one way, if negative move the other
      if(event.un.gravity.x > 0){
      orientMotor.moveMotorForward(1);
      }
      else{
      orientMotor.moveMotorBackward(1);
      }
      event = getBNO085Event(&bno);

    }

}
