#include <Arduino.h>
#include <iostream>
#include <arm_math.h>
#include <SD.h>

#include "BMP390.hpp"
#include "BNO055.hpp"
#include "ICM20649.hpp"
#include "driverSD.hpp"
#include "motorDriver.hpp"
#include "SoilSensor.hpp"
#include "Constants.hpp"

//for coomunication with soil sensor
#define RS485_DIR_PIN 21
#define SLAVE_ID 0x01

HardwareSerial &modbus = Serial1;

//defining states for flight
  enum FlightState {
    PREFLIGHT,
    INFLIGHT,
    LANDED,
  };

//defining states for landed state
  enum LandedState { 
    PLUNGE,
    IDLE, 
    RETRACT
  };

//creating motor objects
  motorDriver orientMotor  = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep);
  motorDriver augerMotor = motorDriver(kAugerControlPin);
  motorDriver leadScrewMotor  = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);
  motorDriver waterMotor  = motorDriver(kWaterMotorIn1, kWaterMotorIn2, kWaterMotorSleep);

//creating sensor objects
  Adafruit_BMP3XX bmp;
  Adafruit_BNO055  bno =  Adafruit_BNO055(55, 0x28);
  Adafruit_ICM20649 icm;

//data file objeect for storing data in sd card
  File dataFile;

  FlightState state = PREFLIGHT; //initializng state
  LandedState whereInLanded = PLUNGE;

//initalizing time markers
  float inFlightStartTime  = 0;
  float landedStartTime  = 0;
  float waterMotorStartTime = 0;
  float idleStartTime = 0;
  float landedStartTimeAltAccel = 0;

//varaibles to hold fetched soil sensor data
  float32_t nitrogenMgKg = 0.0f;
  float32_t pH = 0.0f;
  float32_t electricalConductivity = 0.0f;

  int32_t topHits = 0; //amount of times the top limit switch is hit
  int32_t bottomHits = 0; // amount of times the bottom limit switch is hit

  uint16_t launchDetectCount = 0; // amount of times the liftoff threshold has been met, must has valid and high enoough accel from both sensors to count. Must have 5 consecutive samples
  uint16_t landingDetectCount = 0; // amount of times the landing threshold has been met, must have low accel and low change in altitude to count. Must have 5 consecutive samples

  //initalizng the current and previous altitude, as well as valid bool
  float32_t currentAltitude = 0.0f;
  float32_t previousAltitude = 0.0f;
  bool hasValidInflightAltitude = false;

  driverSD flightData = driverSD(kFlightDataBufferSize); //creating flight data object with correct buffer size (8), to store needed data
  driverSD soilData = driverSD(kSoilDataBufferSize); //creating soil data object w correct buffer size. 
  
//limit switch logic booleans
  bool upperSwitchPressed = false;
  bool lowerSwitchPressed = false;
  bool lastUpperSwitchPressed = false;
  bool lastLowerSwitchPressed = false;
  bool upperStateChange = false;
  bool lowerStateChange = false;

  bool waterGone = false; 
  bool secondPlunge = false;
  bool isOriented = false;
  bool landedFinalized = false;


//timers for state change tracking
  elapsedMillis preflightTimer;
  elapsedMillis preflightStateTimer;
  elapsedMillis inflightTimer;
  elapsedMillis augerSpinTimer;
  elapsedMillis soilTimer;

/*---------------------------------------------------------------------------FUNCTIONS-------------------------------------------------------------------------*/


//------------------------------------------------------ SANITY CHECKS AND MATHMATICAL SENSOR DATA FUNCTIONS ------------------------------------------------------

/*This function will return the absolute value of val*/
float32_t absScalarF32(float32_t val) {
  float32_t src[1] = {val};
  float32_t dst[1] = {0.0f};
  arm_abs_f32(src, dst, 1);
  return dst[0];
}

/*This function will return true if the acceleration value inputted is finite and the absolute value is under 1500 m/s^2*/
bool isFiniteAndReasonable(float32_t value) {
  return __builtin_isfinite(value) && absScalarF32(value) <= kAccelSanityMaxAbsMs2;
}

/*This function will return true if the acceleration componants inputted are all finite and reasonable*/
bool accelVectorIsSane(float32_t x, float32_t y, float32_t z) {
  return isFiniteAndReasonable(x) && isFiniteAndReasonable(y) && isFiniteAndReasonable(z);
}

/*This function will return true if the altitude passed is finite, greater tahn -500 and less than 100,000 m*/
bool altitudeIsSane(float32_t altitudeM) {
  return __builtin_isfinite(altitudeM) &&
         altitudeM >= kAltitudeValidMinM &&
         altitudeM <= kAltitudeValidMaxM;
}

/*This funcntion will return the magnitude squared of a 3d vector*/
float32_t squaredMagnitude(float32_t x, float32_t y, float32_t z) {
  float32_t vec[3] = {x, y, z};
  float32_t magnitudeSquared = 0.0f;
  arm_dot_prod_f32(vec, vec, 3, &magnitudeSquared);
  return magnitudeSquared;
}

//------------------------------------------------------ LANDED STATE CHANGE/INITIALIZATION FUNCTION ------------------------------------------------------

/*This function will reset all vairables needed when entering the landed state, as well as log the landed state start time*/
void enterLandedState() {
  state = LANDED;
  landedStartTime  = millis();

  //making sure the motors are stopped
  orientMotor.stopMotorWithCoast();
  leadScrewMotor.stopMotorWithCoast();
  waterMotor.stopMotorWithCoast();
  augerMotor.stopMosfet();

  landedFinalized  = false;
  upperSwitchPressed = false;
  lowerSwitchPressed = false;
  lastUpperSwitchPressed = false;
  lastLowerSwitchPressed = false;
  upperStateChange = false;
  lowerStateChange = false;
  secondPlunge = false;

}

//------------------------------------------------------ ORIENTATION FUNCTION ------------------------------------------------------


void checkOrientationStep() {
    //getting the gravity vector from bno
    const imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    const float32_t gravityZ = gravity.z();

    sensors_event_t event = getBNO055Event(&bno);
    float orientY = event.orientation.y;

    //if the y axis is within the needed range, stopping the motor and setting the orientMotor to true as it was aligned properly and returning to main program
    if(orientY<kOrientationAlignedYMin){
    orientMotor.moveMotorForward(kOrientationDutyCycle);
    isOriented = false;
    } // || gravityZ<-3.0
    else if (orientY>kOrientationAlignedYMin){
    orientMotor.moveMotorBackward(kOrientationDutyCycle);
    isOriented = false;
    }
    if(orientY>kOrientationAlignedYMin && orientY<kOrientationAlignedYMax && gravityZ > 0.0){
    orientMotor.stopMotorWithCoast();
    isOriented = true;
    Serial.println("[LANDED][ORIENT] y-axis aligned -> orientation complete");
    return;
    }

    }



//------------------------------------------------------ SOIL DATA FUNCTION ------------------------------------------------------


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

//------------------------------------------------------ DATA FILE FUNCTIONS ------------------------------------------------------

/*This function will start or restart the flight logging if the current index of the buffer is zero. It will set the datafile in SD card to the current file
name of the flight log file.*/
void startFlightLoggingIfNeeded() {
  if (flightData.getCurrentIndex() == 0) {
    Serial.print("[INFLIGHT] opening flight log file: ");
    dataFile = SD.open(flightData .getCurrentFileName(), FILE_WRITE);
    Serial.print(dataFile  ? F("[INFLIGHT] file open OK") : F("[INFLIGHT] file open FAILED"));
  }
}

/*This function will start or restart the soil logging if the current index of the buffer is zero. It will set the SD datafile to soil log name.*/
void startSoilLoggingIfNeeded() {
  if (soilData .getCurrentIndex() == 0) {
    Serial.print("[LANDED] opening soil log file: ");
    dataFile  = SD.open(soilData .getCurrentFileName(), FILE_WRITE);
    Serial.print(dataFile  ? F("[LANDED] file open OK") : F("[LANDED] file open FAILED"));
  }
}

/*This function will print flight data to the SD card, no matter if the buffer is full or not because there is a state change to landed */
void finishFlightLogging() {
  Serial.print("[INFLIGHT] finalizing flight log");
  flightData.increaseCurrentIndexBy(-1);
  flightData.printFlightDataToFile(dataFile);
}

/*This fucntion will print the soil data to SD, no matter if the buffer is full or not because the program is done running.*/
void finishSoilLogging() {
  Serial.print("[FC][LANDED] finalizing soil log");
  soilData.increaseCurrentIndexBy(-1);
  soilData.printSoilDataToFile(dataFile );
}



//------------------------------------------------------ SENSOR FUNCTION ------------------------------------------------------

/*This function checks if all the sensors are properly communicating iwht the teensy, does not return anything, just performs connection checks*/
void checkSensorConnections() {
  //if this function was run less than 2 seconds ago, just returning because there is no need to check more than every 2 seconds.
  static elapsedMillis sensorLogTimer;
  if (sensorLogTimer < kSensorHealthPollMs) {
    return;
  }
  sensorLogTimer = 0;

  //checking connections
  checkBNO055Connection(&bno);
  checkBMP390Connection(&bmp );
  checkICMConnection(&icm);
}

/*-------------------------------SETUP---------------------------------------------*/

void setup(){
  //set up serial
  Serial.begin(9600);

  //starting ledpin
  pinMode(ledPin, OUTPUT);

  //intializing the motors inside the setup
  orientMotor  = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep);
  augerMotor  = motorDriver(kAugerControlPin);
  leadScrewMotor = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);
  waterMotor  = motorDriver(kWaterMotorIn1, kWaterMotorIn2, kWaterMotorSleep);

  //setting pins to low so they dont float
  orientMotor.stopMotorWithCoast();
  leadScrewMotor.stopMotorWithCoast();
  waterMotor.stopMotorWithCoast();
  augerMotor.stopMosfet();

}

/*------------------------------------------------- LOOP -----------------------------------------------------------*/
void loop(){
    upperSwitchPressed = (digitalReadFast(kUpperLimitSwitchPin) == HIGH); //true if the upper limit switch is currently pressed
    lowerSwitchPressed = (digitalReadFast(kLowerLimitSwitchPin) == HIGH); //true if the lower limit switch is currently pressed

    upperStateChange = (upperSwitchPressed && !lastUpperSwitchPressed); //when the switch is pressed but wasnt before
    lowerStateChange = (lowerSwitchPressed && !lastLowerSwitchPressed);

    lastUpperSwitchPressed = upperSwitchPressed; //making last the current for next loop
    lastLowerSwitchPressed = lowerSwitchPressed;


   leadScrewMotor.moveMotorForward(kLeadScrewDutyCycle);

  if(upperStateChange){
    leadScrewMotor.stopMotorWithCoast();
    while(1);

  }

  delay(50);

}

