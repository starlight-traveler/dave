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
    REVERSE,
    IDLE, 
    RETRACT
  };

//creating motor objects
  motorDriver orientMotor  = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep);
  motorDriver augerMotor = motorDriver(kAugerPWMPin, kAugerDIRPin, kAugerCurrentSensePin, true);
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
  float sliceStartTime = 0;
  float landedStartTimeAlt = 0;
  float draggedDelayStartTime = 0;
  float plungeStartTime = 0;
  float retractStartTime = 0;

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
  
  //landed booleans
  //limit switch logic booleans
  bool upperSwitchPressed = false;
  bool lowerSwitchPressed = false;
  bool lastUpperSwitchPressed = false;
  bool lastLowerSwitchPressed = false;
  bool upperStateChange = false;
  bool lowerStateChange = false;
  //landed checkpoints to keep track of
  bool waterGone = false; 
  bool secondPlunge = false;
  bool isOriented = false;
  bool landedFinalized = false;
  bool isFirstPlunge = true;
  bool firstPlungeComplete = false;
  bool isBeingDragged = true;

  //timers for state change tracking
  elapsedMillis preflightTimer;
  elapsedMillis preflightStateTimer;
  elapsedMillis inflightTimer;
  elapsedMillis augerSpinTimer;
  elapsedMillis soilTimer;

  //auger movement timers
  elapsedMillis augerMovingDown;
  elapsedMillis augerMovingUp;
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

/*This function will return true if the altitude passed is finite, greater than -500 and less than 100,000 m*/
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
  augerMotor.stopPololu();

  landedFinalized  = false;
  upperSwitchPressed = false;
  lowerSwitchPressed = false;
  lastUpperSwitchPressed = false;
  lastLowerSwitchPressed = false;
  upperStateChange = false;
  lowerStateChange = false;
  secondPlunge = false;
  isBeingDragged = true;

}

// - delay up t 10 minutes while being dragged when entering landed state

void delayIfDragged(){
  //in italizng the start time of checking if being dragged
    draggedDelayStartTime = millis();
    sensors_event_t event;

  //going into a while loop that will check if the vehicle is bing dragged. If it is, it will delay for a half second and check again. 
  //it will do this up for 10 mnutes, and if the vehicle has not settled in that time, it will exit anyways
  while(isBeingDragged && ((millis() - draggedDelayStartTime) < 600000)){
    //getting acceleration
    event = getBNO055Event(&bno);
    float accelX = event.acceleration.x;
    float accelY = event.acceleration.y;
    float accelZ = event.acceleration.z;

    //if the accelration mag is above 2, the rocket is being dragged, delay and continue to recheck for up to 10 minutes
    if (squaredMagnitude(accelX, accelY, accelZ) < kDraggedAccelThresholdSquared){
      isBeingDragged = false;
    }

    delay(500);

  }

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
    // maybe if its pretty close in degrees, we halve the power?
    if(orientY>kOrientationAlignedYMin && orientY<kOrientationAlignedYMax && gravityZ > 0.0){
    orientMotor.stopMotorWithCoast();
    isOriented = true;
    //Serial.println("[LANDED][ORIENT] y-axis aligned -> orientation complete");
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
void checkSensorConnections() 
{
  //if this function was run less than 2 seconds ago, just returning because there is no need to check more than every 2 seconds.
  static elapsedMillis sensorLogTimer;
  if (sensorLogTimer < kSensorHealthPollMs) 
  {
    return;
  }
  sensorLogTimer = 0;

  //checking connections
  checkBNO055Connection(&bno);
  checkBMP390Connection(&bmp );
  checkICMConnection(&icm);
}

//--------------------------------------------------- LIMIT SWITCH CHECKER ------------------------------------------------

void updateLimitSwitches()
{
  upperSwitchPressed = (digitalReadFast(kUpperLimitSwitchPin) == HIGH); //true if the upper limit switch is currently pressed
  lowerSwitchPressed = (digitalReadFast(kLowerLimitSwitchPin) == HIGH); //true if the lower limit switch is currently pressed

  upperStateChange = (upperSwitchPressed && !lastUpperSwitchPressed); //when the switch is pressed but wasnt before
  lowerStateChange = (lowerSwitchPressed && !lastLowerSwitchPressed);

  lastUpperSwitchPressed = upperSwitchPressed; //making last the current for next loop
  lastLowerSwitchPressed = lowerSwitchPressed;
}

// ------------------------------------------------- WATER PUMP CONTROL CODE -------------------------------

void controlWaterPump()
{
  if(firstPlungeComplete)
  {
    if(waterMotorStartTime == 0)
    {
      waterMotorStartTime = millis();
      waterMotor.moveMotorBackward(kWaterDutyCycle);
      Serial.println("Water motor time reset");
    }
    Serial.println(millis()-waterMotorStartTime);
    if(millis()-waterMotorStartTime>kWaterTimeoutMs)
    {
      waterMotor.stopMotorWithCoast();
    }
  }
  else
  {
    waterMotor.stopMotorWithCoast();
  }
}

// dave shutdown function
void shutdownDave(){
    Serial.println("Shutting DAVE down.");
    augerMotor.stopPololu();
    leadScrewMotor.stopMotorWithCoast();
    waterMotor.stopMotorWithCoast();
    orientMotor.stopMotorWithCoast();

    finishSoilLogging();
    landedFinalized  = true;
}

/*-------------------------------SETUP---------------------------------------------*/

void setup()
{
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
  orientMotor = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep);
  augerMotor = motorDriver(kAugerPWMPin, kAugerDIRPin, kAugerCurrentSensePin, true);
  leadScrewMotor = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);
  waterMotor = motorDriver(kWaterMotorIn1, kWaterMotorIn2, kWaterMotorSleep);

  //setting pins to low so they dont float
  orientMotor.stopMotorWithCoast();
  leadScrewMotor.stopMotorWithCoast();
  waterMotor.stopMotorWithCoast();
  augerMotor.stopPololu();

  //setup the sensors
  setupBMP(&bmp);
  setupBNO055(&bno);
  setupICM20649(&icm);
  
  //set up the limnit switches as input pullup
  pinMode(kUpperLimitSwitchPin, INPUT_PULLUP);
  pinMode(kLowerLimitSwitchPin, INPUT_PULLUP);

  //begin the sd card, shining led light if cannot find or connect to
  if (!SD.begin(BUILTIN_SDCARD)) 
  {
    Serial.println("SD mount failed, halting and outputting light");
    while (1) 
    {
      digitalWrite(ledPin, HIGH);
    }
  }

  //setting up the file names for the flight data and soil data so can be logged properly
  flightData.begin(kFlightDataRoot);
  soilData.begin(kSoilDataRoot);

  //starting the preflight state timer so it can accumulate in loop
  preflightStateTimer  = 0;
  Serial.println("Made it to end of setup");
}

/*------------------------------------------------- LOOP -----------------------------------------------------------*/
void loop()
{

  if (landedFinalized) 
  {
    return;
  }

  //checking sensor connections
  checkSensorConnections();

  switch (state){
    //only update and go thorugh the preflight loop every 50 ms
    case(PREFLIGHT): 
    {
      if (preflightTimer < kPreflightUpdatePeriodMs) 
      {
        return;
      }
    // probably shouldn't ever trigger due to the 50ms delay at the end of the function

    Serial.println("PREFLIGHT");

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
    const bool launchSample = icmAccelSane && bnoAccelSane && accelICMSquared > kAccelThresholdSquared && accelBNOSquared > kAccelTestThresholdSquared;
    //Serial.println(accelICMSquared);
    //Serial.println(accelBNOSquared);
    //if the launch sample is true, increaeing the detection, which counts how many consectutive times it is true. If not true, consectuve counts goes back to zero.
    
    if (launchSample) 
    {
      if (launchDetectCount  < kLaunchDetectConsecutiveSamples) 
      {
        launchDetectCount ++;
      }
    } 
    else 
    {
      launchDetectCount  = 0;
    }

    //if the consective samples reach the needed amount, changing state to INFLIGHT
    if (launchDetectCount >= kLaunchDetectConsecutiveSamples) 
    {
      state = INFLIGHT;
      inFlightStartTime = millis(); //saving inflight start time
      Serial.print("Inflight start time: ");
      Serial.println(inFlightStartTime);
      landingDetectCount  = 0; //initalizng the landed detecr count to zero
      //if the altitude is valid, saving it as both current and past altitude so change can start to be tracked
      if (altitudeValid) 
      {
        previousAltitude  = currentAlt;
        currentAltitude  = currentAlt;
        hasValidInflightAltitude  = true;
      //if not initializng both to 0
      } 
      else 
      {
        previousAltitude  = 0.0f;
        currentAltitude  = 0.0f;
        hasValidInflightAltitude  = false;
      }
      return;
    }
    break;
  }

  case (INFLIGHT): 
  {
    if (inflightTimer < kInflightUpdatePeriodMs){ //only updates every 30 ms
    
      return;
    }
    Serial.println("INFLIGHT");
    //starting the logging again if needed (index == 0)
    startFlightLoggingIfNeeded();

    //time in the inflight state
    const uint32_t timeDiffInFlight = millis() - inFlightStartTime ;

    //getting all sensor events needed for logging
    sensors_event_t accel = getICM20649Accel(&icm);
    sensors_event_t orient = getBNO055Event(&bno);
    const float32_t altitude = getAltitude(&bmp);

    //adding data to the flighData buffer to be stored in sd card
    flightData.addFlightData(accel, orient, altitude, dataFile);

    //checking if sensor values are sane
    const bool altitudeValid = altitudeIsSane(altitude);
    //const bool accelValid = accelVectorIsSane(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

    //getting accel squared if valid, if not set to zero
    //const float32_t accelSquared = accelValid ? squaredMagnitude(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z) : 0.0f;

    //if the current alt is valid and wasnt before, setting both current and past to the current alt, then setting has valid alt to true
    if (altitudeValid) 
    {
      if (!hasValidInflightAltitude) 
      {
        //if it valid and has had previuos valid alt, just saving current alt to current
        previousAltitude  = altitude;
        currentAltitude  = altitude;
        hasValidInflightAltitude  = true;
      }
      else 
      {
        currentAltitude  = altitude;
      }
    //threshold of consectuve counts cant be met if alt isnt valid
    } 
    else 
    {
      landingDetectCount  = 0;
    }

    //must be in flight for at least 5 seconds, this will be true after 5 seconds of inflight state
    const bool landingEvalArmed = timeDiffInFlight >= kMinInflightBeforeLandingEvalMs;

    //resetting landing sample to false
    //bool landingSample = false;
    bool altLessThanHalfM;

    //if after 5 seconds, there is valid alt and accel, seeing if alt is less than half a meter and accel is less than 4
    if (landingEvalArmed && altitudeValid && hasValidInflightAltitude) // used to have && accelValid
    {
      //will be true if the difference in altitude is less than 0.5 meters (falling slowly)
        altLessThanHalfM = absScalarF32(currentAltitude  - previousAltitude ) <= kLandingAltitudeDeltaThresholdM;

      //will be true is accel is less than 4
      //const bool lowAccel = accelSquared <= kLandingAccelThresholdSquared;

      //will be true is accel is less than 4 and delta alt is less than 0.5
      //landingSample = altLessThanHalfM && lowAccel;
    }

    //if valid sample, and not yet at threshold, increasing consective samples. If not valid, resetting detection count
    if (altLessThanHalfM) 
    {
      if (landingDetectCount  < kLandingDetectConsecutiveSamples) 
      {
        landingDetectCount++;
      }
    } 
    else 
    {
      landingDetectCount = 0;
    }

    const bool landedDetectedFromAlt = landingEvalArmed && landingDetectCount >= kLandingDetectConsecutiveSamples;

    //if after five seconds and the detection count gets to five, save the time landed would have been detected from landed
    if (landedDetectedFromAlt) 
    {
      landedStartTimeAlt = millis();
      Serial.print("Landed start time from alitmeter (from beginning of program): ");
      Serial.println(landedDetectedFromAlt);
    }

    Serial.println(timeDiffInFlight);

    //if been in inflight for 3 mintutes finish and close the flight logging, enter landed state, and set topHits to one bc it sits at top intially
    if (timeDiffInFlight > kInflightTimeoutMs) 
    {
      finishFlightLogging();
      enterLandedState();
      return;
    }

    //saving current alt for next loop's last alt
    if (altitudeValid && hasValidInflightAltitude) 
    {
      previousAltitude  = currentAltitude;
    }

    break;
  }


  case (LANDED): 
  {
    //Serial.println("LANDED");

    //if the different in start time and the current time is larger than 15 mintutes, stopping all mototes, finishing and saving soil logging, then setting landedFinalized to true to stop all functions
    if (millis() - landedStartTime  >= kLandedTimeoutMs) 
    {
      Serial.println("landed timeout reached, stopping all motors");
      shutdownDave();
    }


    
    delayIfDragged();
    startSoilLoggingIfNeeded();
    checkOrientationStep();
    getAndLogSoilData();
    updateLimitSwitches();
    controlWaterPump();

    if(isOriented)
    {
      switch(whereInLanded)
      {
        case(PLUNGE):
          augerMotor.stopPololu();
          leadScrewMotor.moveMotorBackward(kLeadScrewDutyCycle);

          if(isFirstPlunge)
          {
            isFirstPlunge = !isFirstPlunge;
            sliceStartTime = millis();
          }

          if(millis()-sliceStartTime>kPlungePeriod)
          {
            whereInLanded = REVERSE;
            sliceStartTime = millis();
          }

          if(lowerStateChange)
          {
            leadScrewMotor.stopMotorWithCoast();
            whereInLanded = IDLE;
            idleStartTime = millis();
          }

          if((millis() - plungeStartTime) > kPlungeTimeToBottomLimit){
            whereInLanded = RETRACT;
            retractStartTime = millis();
          }

          break;


        case(REVERSE):
          leadScrewMotor.moveMotorForward(kLeadScrewDutyCycle);
          if(millis()-sliceStartTime>kReversePeriod)
          {
            whereInLanded = PLUNGE;
            sliceStartTime = millis();
          }
          break;

        case(IDLE):
          augerMotor.stopPololu();
          if(millis() - idleStartTime >= kAugerSpinDurationMs) // this should be changed to a variable
          {
            whereInLanded = RETRACT;
            leadScrewMotor.moveMotorForward(kLeadScrewDutyCycle);
            retractStartTime = millis();
          }
          break;


        case(RETRACT):
          leadScrewMotor.moveMotorForward(kLeadScrewDutyCycle);
          augerMotor.stopPololu();
          if(upperStateChange)
          {
            //resetting all landed state variables after complete cycle
            firstPlungeComplete = true;
            upperSwitchPressed = false;
            lowerSwitchPressed = false;
            lastUpperSwitchPressed = false;
            lastLowerSwitchPressed = false;
            upperStateChange = false;
            lowerStateChange = false;
            secondPlunge = false;
            plungeStartTime = millis();
            whereInLanded = PLUNGE;            
          }

          if(millis() - retractStartTime >= kRetractTimeToTopLimit){
            Serial.println("Top limit switch not hit after 35 seconds.. shutting down");
            shutdownDave();
          }

          break;
        default:
          break;
      }
    }
    else
    {
      leadScrewMotor.stopMotorWithCoast();
    }
    state = LANDED;
    break;
  }

  default:
  return;

  }

  delay(50);

}



/*
//if the 15 minutes have passed, just return to the program continues forever
    if (landedFinalized) 
    {
      return;
    }

    Serial.println("LANDED");

    //if the different in start time and the current time is larger than 15 mintutes, stopping all mototes, finishing and saving soil logging, then setting landedFinalized to true to stop all functions
    if (millis() - landedStartTime  >= kLandedTimeoutMs) 
    {
      Serial.println("landed timeout reached, stopping all motors");
      augerMotor.stopMosfet();
      leadScrewMotor.stopMotorWithCoast();
      waterMotor.stopMotorWithCoast();
      orientMotor.stopMotorWithCoast();

      finishSoilLogging();
      landedFinalized  = true;
      return;
    }

    startSoilLoggingIfNeeded();
    checkOrientationStep();
    getAndLogSoilData();

    //switch logic
    upperSwitchPressed = (digitalReadFast(kUpperLimitSwitchPin) == HIGH); //true if the upper limit switch is currently pressed
    lowerSwitchPressed = (digitalReadFast(kLowerLimitSwitchPin) == HIGH); //true if the lower limit switch is currently pressed

    upperStateChange = (upperSwitchPressed && !lastUpperSwitchPressed); //when the switch is pressed but wasnt before
    lowerStateChange = (lowerSwitchPressed && !lastLowerSwitchPressed);

    lastUpperSwitchPressed = upperSwitchPressed; //making last the current for next loop
    lastLowerSwitchPressed = lowerSwitchPressed;

    if (isOriented)
    {
      switch(whereInLanded)
      {
        case(PLUNGE): 
        {
          augerMotor.moveMosfet();
          leadScrewMotor.moveMotorBackward(kLeadScrewDutyCycle);

          if(secondPlunge && ((millis() - waterMotorStartTime)<kWaterTimeoutMs))
          {
            waterMotor.moveMotorBackward(kWaterDutyCycle);
          }
          else 
          {
            waterMotor.stopMotorWithCoast();
            waterGone = true;
          }

          if(lowerStateChange)
          {
            leadScrewMotor.stopMotorWithCoast();
            augerMotor.moveMosfet();
            whereInLanded = IDLE;
            idleStartTime = millis();
          }

          break;
        }
        case(PAUSE):
        {

          break;
        }
        case(IDLE):
        {
          if(millis() - idleStartTime >= 12000)
          {
            whereInLanded = RETRACT;
            leadScrewMotor.moveMotorForward(kLeadScrewDutyCycle);
          }
          break;
        }
        case (RETRACT):
        {
          if(upperStateChange)
          {
            if(!secondPlunge && !waterGone)
            {
              secondPlunge = true;
              waterMotorStartTime = millis();
            }
            //resetting all landed state variables after complete cycle
            enterLandedState();
            whereInLanded = PLUNGE;            
          }
          break;
        }

        default: 
        {
          break;
        }
      }
    }

  //just making sure the state staying in landed
  state = LANDED;
*/

