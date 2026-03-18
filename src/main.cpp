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
#define RS485_DIR_PIN 27
#define SLAVE_ID 0x01

HardwareSerial &modbus = Serial2;

//defining states
  enum FlightState {
    PREFLIGHT,
    INFLIGHT,
    LANDED,
  };

//creating motor objects
  motorDriver orientMotor  = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep);
  motorDriver augerMotor = motorDriver(gpioAuger);
  motorDriver leadScrewMotor  = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);
  motorDriver waterMotor  = motorDriver(gpioWater);

//creating sensor objects
  Adafruit_BMP3XX bmp;
  Adafruit_BNO055  bno =  Adafruit_BNO055(55, 0x28);
  Adafruit_ICM20649 icm;

//data file objeect for storing data in sd card
  File dataFile;

  FlightState state = PREFLIGHT; //initializng state
  FlightState lastLoggedstate = PREFLIGHT;
  bool hasLoggedInitialstate = false;

//initalizing time markers
  float inflightStartTime  = 0;
  float landedStartTime  = 0;
  float waterMotorStartTime = 0;

//varaibles to hold fetched soil sensor data
  float32_t nitrogenMgKg = 0.0f;
  float32_t pH = 0.0f;
  float32_t electricalConductivity = 0.0f;


  int32_t topHits = 0; //amount of times the top limit switch is hit
  int32_t bottomHits = 0; // amount of times the bottom limit switch is hit

  uint16_t launchDetectCount = 0; // amount of times the liftoff threshold has been met, must has valid and high enoough accel from both sensors to count. Must have 5 consecutive samples
  uint16_t landingDetectCount = 0; // amount of times the landing threshold has been met, must have low accel and low change in altitude to count. Must have 5 consecutive samples

  float32_t currentAltitude = 0.0f;
  float32_t previousAltitude = 0.0f;
  bool hasValidInflightAltitude = false;

  driverSD flightData = driverSD(kFlightDataBufferSize); //creating flight data object with correct buffer size (8), to store needed data
  driverSD soilData = driverSD(kSoilDataBufferSize); //creating soil data object w correct buffer size. 

//motor driver logic booleans
  bool leadScrewFullyExtended = false;
  bool stationaryAugerSpinActive = false;
  bool landedFinalized = false;
  
//limit switch logic booleans
  bool upperSwitchPressed = false;
  bool lowerSwitchPressed = false;
  bool lastUpperSwitchPressed = false;
  bool lastLowerSwitchPressed = false;
  bool upperStateChange = false;
  bool lowerStateChange = false;
  bool isMovingUp = false;
  bool waterDispensed = false; //will be true once water dipenses once
  bool waterGone = false; // will be true after


//timers for state change tracking
  elapsedMillis preflightTimer;
  elapsedMillis preflightStateTimer;
  elapsedMillis inflightTimer;
  elapsedMillis augerSpinTimer;
  elapsedMillis soilTimer;

/*-------------------------FUNCTIONS-------------------------------------*/

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

/*This function will reset all vairables needed when entering the landed state, as well as log the landed state start time*/
void enterLandedState() {
  state = LANDED;
  landedStartTime  = millis();

  upperSwitchPressed = false;
  lowerSwitchPressed = false;
  lastUpperSwitchPressed = false;
  lastLowerSwitchPressed = false;
  upperStateChange = false;
  lowerStateChange = false;

  topHits  = 0;
  bottomHits  = 0;
  launchDetectCount  = 0;
  landingDetectCount  = 0;
  hasValidInflightAltitude  = false;
  leadScrewFullyExtended  = false;
  stationaryAugerSpinActive  = false;
  landedFinalized  = false;

}

/*This function will check the orientation of the nosecone and turn the nosecone accordingly if not aligned. */
void checkOrientationStep() {
  //getting the gravity vector from bno
    const imu::Vector<3> gravity =  bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    const float32_t gravityX = gravity.x();
    const float32_t gravityY = gravity.y();
    const float32_t gravityZ = gravity.z();

 //if the gravity vector inputted is not same, then start a timer to log how long the program has gone wihtout valid gravity data.
  if (!accelVectorIsSane(gravityX, gravityY, gravityZ)) {
    static elapsedMillis invalidGravityLogTimer;
    //only prints out message every 500 ms so not bombarded with error messages
    if (invalidGravityLogTimer >= kOrientationLogPeriodMs) {
      invalidGravityLogTimer = 0;
      Serial.println("[LANDED][ORIENT] gravity invalid, skipping orientation step");
    }
    //if not valid, stopping motor and returning to main program
    orientMotor.stopMotorWithCoast();
    return;
  }

  //if the y axis is within the needed range, stopping the motor and setting the orientMotor to true as it was aligned properly and returning to main program
  if (gravityY <= kOrientationAlignedYMax && gravityY >= kOrientationAlignedYMin) {
    orientMotor.stopMotorWithCoast();
    Serial.println("[LANDED][ORIENT] y-axis aligned -> orientation complete");
    return;
  }
}

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

  //set up modbus for soil sensor
  modbus.begin(9600, SERIAL_8N1);

  //configure rs485 pin and initialze to low
  pinMode(RS485_DIR_PIN, OUTPUT);
  digitalWrite(RS485_DIR_PIN, LOW);

  Serial.println("Soil sensor Modbus reader started");

  
  //starting ledpin
  pinMode(ledPin, OUTPUT);

  //intializing the motors inside the setup
  orientMotor  = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep);
  augerMotor  = motorDriver(gpioAuger);
  leadScrewMotor = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);
  waterMotor  = motorDriver(gpioWater);

  //setting pins to low so they dont float
  augerMotor.stopMosfet();
  waterMotor.stopMosfet();

  //setup the sensors
  setupBMP(&bmp);
  setupBNO055(&bno);
  setupICM20649(&icm);
  
  //set up the limnit switches as input pullup
  pinMode(kUpperLimitSwitchPin, INPUT_PULLUP);
  pinMode(kLowerLimitSwitchPin, INPUT_PULLUP);

  //begin the sd card, shining led light if cannot find or connect to
  if (!SD.begin(BUILTIN_SDCARD)) {
  Serial.println("SD mount failed, halting and outputting light");
  while (1) {
    digitalWrite(ledPin, HIGH);
  }
  }

  //setting up the file names for the flight data and soil data so can be logged properly
  flightData.begin(kFlightDataRoot);
  soilData.begin(kSoilDataRoot);

  //starting the preflight state timer so it can accumulate in loop
  preflightStateTimer  = 0;

}


/*------------------------------------------------- LOOP -----------------------------------------------------------*/
void loop(){
  //checking sensor connections
  checkSensorConnections();






  switch (state){
    //only update and go thorugh the preflight loop every 50 ms
    case(PREFLIGHT): {
      if (preflightTimer  < kPreflightUpdatePeriodMs) {
      return;
    }

    //getting ICM acceleration and checking if it sane, if so, getting it squared for comparison to threshold
    sensors_event_t accelICM = getICM20649Accel(&icm);
    const bool icmAccelSane = accelVectorIsSane(accelICM.acceleration.x, accelICM.acceleration.y, accelICM.acceleration.z);
    const float32_t accelICMSquared = icmAccelSane ? squaredMagnitude(accelICM.acceleration.x, accelICM.acceleration.y, accelICM.acceleration.z) : 0.0f; //if not sane, it will equal zero as to not change states

    //getting BNO acceleration and checking if it sane, if so, getting it squared for comparison to threshold
    sensors_event_t bnoAccel = getBNO055Event(&bno);
    const bool bnoAccelSane = accelVectorIsSane(bnoAccel.acceleration.x,bnoAccel.acceleration.x, bnoAccel.acceleration.x);
    const float32_t accelBNOSquared = bnoAccelSane ? squaredMagnitude(bnoAccel.acceleration.x,bnoAccel.acceleration.x, bnoAccel.acceleration.x) : 0.0f;

    //getting the current altitude from BMP, checking if sane.
    const float32_t currentAlt = getAltitude(&bmp);
    const bool altitudeValid = altitudeIsSane(currentAlt);

    //launch sample will return true if all values gotten are same and within thresholds to count towards launch, and false if not
    //both accels must be above 4gs
    const bool launchSample = icmAccelSane && bnoAccelSane && accelICMSquared > kAccelThresholdSquared && accelBNOSquared > kAccelThresholdSquared;

    //if the launch sample is true, increaeing the detection, which counts how many consectutive times it is true. If not true, consectuve counts goes back to zero.
    if (launchSample) {
      if (launchDetectCount  < kLaunchDetectConsecutiveSamples) {
        launchDetectCount ++;
      }
    } else {
      launchDetectCount  = 0;
    }

    //if the consective samples reach the needed amount, changing state to INFLIGHT
    if (launchDetectCount  >= kLaunchDetectConsecutiveSamples) {
      state = INFLIGHT;
      inflightStartTime  = millis(); //saving inflight start time
      landingDetectCount  = 0; 
      //if the altitude is valid, saving it as both current and past altitude so change can start to be tracked
      if (altitudeValid) {
          previousAltitude  = currentAlt;
          currentAltitude  = currentAlt;
          hasValidInflightAltitude  = true;
      //if not initializng both to 0
        } else {
          previousAltitude  = 0.0f;
          currentAltitude  = 0.0f;
          hasValidInflightAltitude  = false;
        }
      return;
    }
  break;
  }





  case (INFLIGHT): {
    if (inflightTimer  < kInflightUpdatePeriodMs) { //only updates every 30 ms
    return;
    }

  const uint32_t timeDiffInFlight = millis() - inflightStartTime ; //getting the current time in flight

  //getting all sensor events needed for logging
  sensors_event_t accel = getICM20649Accel(&icm);
  sensors_event_t orient = getBNO055Event(& bno);
  const float32_t altitude = getAltitude(&bmp );

  //checking if sensor values are sane
  const bool altitudeValid = altitudeIsSane(altitude);
  const bool accelValid = accelVectorIsSane(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

  //getting accel squared if valid
  const float32_t accelSquared = accelValid ? squaredMagnitude(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z) : 0.0f;

  //starting the logging again if needed (index == 0)
  startFlightLoggingIfNeeded();


  //adding data to the flighData buffer to be stored in sd card
  flightData.addFlightData(accel, orient, altitude, dataFile );

  //if the current alt is valid and wasnt before, setting both current and past to the current alt, then setting has valid alt to true
  if (altitudeValid) {
    if (!hasValidInflightAltitude ) {
      previousAltitude  = altitude;
      currentAltitude  = altitude;
      hasValidInflightAltitude  = true;
  //if it valid and has had previuos valid alt, just saving current alt to current
    } else {
      currentAltitude  = altitude;
    }
  //threshold of consectuve counts cant be met if alt isnt valid
  } else {
    landingDetectCount  = 0;
  }

  //must be in flight for at least 5 seconds, this will be true after 5 seconds of inflight state
  const bool landingEvalArmed = timeDiffInFlight >= kMinInflightBeforeLandingEvalMs;

  //resetting landing sample to false
  bool landingSample = false;

  //if after 5 seconds, there is valid alt and accel, seeing if 
  if (landingEvalArmed && altitudeValid && hasValidInflightAltitude  && accelValid) {
    //will be true if the difference in altitude is less than 0.5 meters (falling slowly)
    const bool altLessThanHalfM = absScalarF32(currentAltitude  - previousAltitude ) <= kLandingAltitudeDeltaThresholdM;

    //will be true is accel is less than 4
    const bool lowAccel = accelSquared <= kLandingAccelThresholdSquared;

    //will be true is accel is less than 4 and delta alt is less than 0.5
    landingSample = altLessThanHalfM && lowAccel;
  }

  //if valid sample, and not yet at threshold, increasing consective samples. If not valid, resetting detection count
  if (landingSample) {
    if (landingDetectCount  < kLandingDetectConsecutiveSamples) {
      landingDetectCount ++;
    }
  } else {
    landingDetectCount  = 0;
  }

  //if after five seconds and the detection count gets to five, finish and close the flight logging, enter landed state, and set topHits to one bc it sits at top intially
  if (landingEvalArmed && landingDetectCount  >= kLandingDetectConsecutiveSamples) {
    finishFlightLogging();
    enterLandedState();
    topHits  = 1;
    return;

  //if been in inflight for 3 mintutes finish and close the flight logging, enter landed state, and set topHits to one bc it sits at top intially
  if (timeDiffInFlight > kInflightTimeoutMs) {
    finishFlightLogging();
    enterLandedState();
    topHits  = 1;
    return;
  }

  //saving current alt for next loop's last alt
  if (altitudeValid && hasValidInflightAltitude ) {
      previousAltitude  = currentAltitude ;
    }
  }
  break;
  }







  case (LANDED): {
    //if the 15 minutes have passed, just return to the program continues forever
    if (landedFinalized ) {
      return;
    }

    //delay 2 seconds
    delay(2000);

    startSoilLoggingIfNeeded();
    checkOrientationStep();

    //switch logic
    upperSwitchPressed = (digitalReadFast(kUpperLimitSwitchPin) == HIGH); //true if the upper limit switch is currently pressed
    lowerSwitchPressed = (digitalReadFast(kLowerLimitSwitchPin) == HIGH); //true if the lower limit switch is currently pressed

    upperStateChange = (upperSwitchPressed && !lastUpperSwitchPressed); //when the switch is pressed but wasnt before
    lowerStateChange = (lowerSwitchPressed && !lastLowerSwitchPressed);

    lastUpperSwitchPressed = upperSwitchPressed; //making last the current for next loop
    lastLowerSwitchPressed = lowerSwitchPressed;

    //if the upper limit switch is pressed and was not before, and the number of top hits is zero, stopping the lead screw motors and increasing number of top hits
    if (upperStateChange && topHits  == 0) { 
      Serial.println("upper switch pressed");
      leadScrewMotor.stopMotorWithCoast();
      topHits ++; 
    }

    //if the number of top hits is zero, moving the lead screw forward (up) in order to reach the upper limit switch
    if (topHits  == 0) {
      Serial.println("Retracting lead screw to find top switch");
      leadScrewMotor.moveMotorForward(kLeadScrewDutyCycle);
    }

    //if the bottom hits is zero and the top hits is one, means the lead screw is at the top, so starting the auger turning, moving the leadscrew downwards, and setting isMovingUp to false
    if (bottomHits  == 0 && topHits  == 1) {
      Serial.println("top reached once, drilling down and spinning auger");
      augerMotor.moveMosfet();
      leadScrewMotor.moveMotorBackward(kLeadScrewDutyCycle);
      isMovingUp = false;
    }

    //if the lead screw is moving downwards, bottom hits is currently zero, and the lower limit switch was hit, increasing the number of bottom hits
    if (isMovingUp==false && bottomHits  == 0 && lowerStateChange) {
      Serial.println("Lower switch pressed");
      bottomHits ++; //hit the bottom, wasnt there before, increase bottom hits
    }

    //if the top hits is one and bottom hits is one, and the auger is not stationarily moving, stopping the lead screw, setting leadScrewFullyExtended to true, and setting stationaryAugerSpinActive, then starting the auger spin timer
    if (bottomHits  == 1 && topHits  == 1) {
      if (!stationaryAugerSpinActive ) {
        Serial.println("bottom reached, stop lead screw, hold auger spin for 10s");
        leadScrewMotor.stopMotorWithCoast();
        leadScrewFullyExtended  = true;
        stationaryAugerSpinActive  = true; //stop extending, keep spinning
        augerSpinTimer  = 0;
      }
    }

    //if the lead screw is fully exteneded, bottom and top hits are one, the auger is spinning stationary, and the alloted 10s as passed, setting augerSpinStationary to false, leadScrewFullyExtended to false,
    // and moving the lead screw up, then setting isMovingUp to ture
    if (leadScrewFullyExtended  && bottomHits  == 1 && topHits  == 1 && stationaryAugerSpinActive  && augerSpinTimer  >= kAugerSpinDurationMs) {
      Serial.println("Auger spin window complete, retracting lead screw");
      stationaryAugerSpinActive  = false;
      leadScrewFullyExtended  = false;
      leadScrewMotor.moveMotorForward(kLeadScrewDutyCycle);
      isMovingUp = true;
    }

    //if the lead screw is moving up, the upper limit switch is hit, and the bottom and top hits are one, 
    //increasing top hits to 2 and stopping the lead screw, then setting isMovingUp to false
    if (isMovingUp && upperStateChange && bottomHits  == 1 && topHits  == 1) {
      Serial.print("Upp Switch pressed while retracting");
      leadScrewMotor.stopMotorWithCoast();
      isMovingUp = false;
      topHits ++;
    }

    //is not moving up, the bottom hits is 1, top is 2, water is not dispensed and the water is not gone, then setting water start time to current millis(), then starting the warter motor
    if (isMovingUp == false && bottomHits  == 1 && topHits  == 2 && !waterDispensed && !waterGone) {
      Serial.println("drilling sequence complete, starting water motor");
      waterMotorStartTime = millis();
      waterMotor.moveMosfet();

    }

    //starting the 
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
      waterMotor.stopMosfet();

    }


    //if the different in start time and the current time is larger than 15 mintutes, stopping all mototes, finishing and saving soil logging, then setting landedFinalized to true to stop all functions
    if (millis() - landedStartTime  >= kLandedTimeoutMs) {
      Serial.println("landed timeout reached, stopping all motors");
      augerMotor.stopMosfet();
      leadScrewMotor.stopMotorWithCoast();
      waterMotor.stopMosfet();
      orientMotor.stopMotorWithCoast();

      finishSoilLogging();
      landedFinalized  = true;
      return;
    }

    //else, if the tophits is 2, bottom is one, and the water is dispensedand gone, resetting the landed state to start the process again (this time w no water being dispensed for future iterations)
    if (topHits  == 2 && bottomHits  == 1 && waterDispensed && waterGone) {
      Serial.println("cycle complete, resetting counters to start the process again (no water pump this time)");
      enterLandedState();
    }

    //just making sure the state staying in landed
    state = LANDED;

  break;
  }

  default:
  return;

  }

}

