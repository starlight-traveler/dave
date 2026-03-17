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

namespace{

#define RS485_DIR_PIN 27
#define SLAVE_ID 0x01

HardwareSerial &modbus = Serial2;

  enum FlightState {
    PREFLIGHT,
    INFLIGHT,
    LANDED,
  };

  motorDriver orientMotor  = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep);
  motorDriver augerMotor = motorDriver(gpioAuger);
  motorDriver leadScrewMotor  = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);
  motorDriver waterMotor  = motorDriver(gpioWater);

  Adafruit_BMP3XX bmp;
  Adafruit_BNO055  bno =  Adafruit_BNO055(55, 0x28);
  Adafruit_ICM20649 icm;

  File dataFile;

  float landedStartTime  = 0;
  float waterMotorStartTime = 0;

  float32_t nitrogenMgKg = 0.0f;
  float32_t pH = 0.0f;
  float32_t electricalConductivity = 0.0f;

  int32_t topHits = 0;
  int32_t bottomHits = 0;
  uint16_t launchDetectCount = 0;
  uint16_t landingDetectCount = 0;

  driverSD soilData = driverSD(kSoilDataBufferSize);

  bool leadScrewFullyExtended = false;
  bool augerSpinActive = false;
  bool orientationAligned = false;
  bool landedFinalized = false;
  
  bool upperSwitchPressed = false;
  bool lowerSwitchPressed = false;
  bool lastUpperSwitchPressed = false;
  bool lastLowerSwitchPressed = false;
  bool upperStateChange = false;
  bool lowerStateChange = false;

  bool isMovingUp = false;

  bool waterDispensed = false;
  bool waterGone = false;

  elapsedMillis preflightTimer;
  elapsedMillis preflightStateTimer;
  elapsedMillis inflightTimer;
  elapsedMillis augerSpinTimer;
  elapsedMillis orientationTimer;
  elapsedMillis soilTimer;

/*-------------------------FUNCTIONS-------------------------------------*/
float32_t absScalarF32(float32_t value) {
  float32_t src[1] = {value};
  float32_t dst[1] = {0.0f};
  arm_abs_f32(src, dst, 1);
  return dst[0];
}

bool isFiniteAndReasonable(float32_t value) {
  return __builtin_isfinite(value) && absScalarF32(value) <= kAccelSanityMaxAbsMs2;
}

bool accelVectorIsSane(float32_t x, float32_t y, float32_t z) {
  return isFiniteAndReasonable(x) && isFiniteAndReasonable(y) && isFiniteAndReasonable(z);
}

void enterLandedState() {
  landedStartTime  = millis();
  orientationAligned  = false;
  orientationTimer  = 0;


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
  leadScrewFullyExtended  = false;
  augerSpinActive  = false;
  landedFinalized  = false;
}


void checkOrientationStep() {
  if (orientationAligned ) {
    return;
  }

  const imu::Vector<3> gravity =  bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  const float32_t gravityX = gravity.x();
  const float32_t gravityY = gravity.y();
  const float32_t gravityZ = gravity.z();
  if (!accelVectorIsSane(gravityX, gravityY, gravityZ)) {
    static elapsedMillis invalidGravityLogTimer;
    if (invalidGravityLogTimer >= kOrientationLogPeriodMs) {
      invalidGravityLogTimer = 0;
      Serial.println("[FC][LANDED][ORIENT] gravity invalid, skipping orientation step");
    }
    orientMotor .stopMotorWithCoast();
    return;
  }

  static elapsedMillis orientLogTimer;
  if (orientLogTimer >= kOrientationLogPeriodMs) {
    orientLogTimer = 0;

  }

  if (gravityY <= kOrientationAlignedYMax &&
     gravityY >= kOrientationAlignedYMin) {
    orientMotor .stopMotorWithCoast();
    orientationAligned  = true;
    Serial.println("[FC][LANDED][ORIENT] z-axis aligned -> orientation complete");
    return;
  }

  if (orientationTimer  >= kOrientationTimeoutMs) { // do we think 5 seconds is enough time?
    orientMotor .stopMotorWithCoast();
    orientationAligned  = true;
    Serial.println("[FC][LANDED][ORIENT] orientation timeout -> stopping motor");
  }
}

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

void checkSensorConnections() {
  static elapsedMillis sensorLogTimer;
  if (sensorLogTimer < kSensorHealthPollMs) {
    return;
  }
  sensorLogTimer = 0;
  Serial.print("[FC] checkSensorConnections(): polling BNO health");
  checkBNO055Connection(& bno);
  checkBMP390Connection(&bmp );
  checkICMConnection(&icm);
}

}

/*-------------------------------SETUP---------------------------------------------*/

void setup(){
  Serial.begin(9600);

  modbus.begin(9600, SERIAL_8N1);
  pinMode(RS485_DIR_PIN, OUTPUT);
  digitalWrite(RS485_DIR_PIN, LOW);

  Serial.println("Soil sensor Modbus reader started");

  
  //starting ledpin
  pinMode(ledPin, OUTPUT);

  orientMotor  = motorDriver(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep);
  augerMotor  = motorDriver(gpioAuger);
  leadScrewMotor = motorDriver(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep);
  waterMotor  = motorDriver(gpioWater);

  //setting pins to low so they dont float
  augerMotor.stopMosfet();
  waterMotor.stopMosfet();

  setupBMP(&bmp );
  setupBNO055(&bno );
  setupICM20649(&icm);
  
  pinMode(kUpperLimitSwitchPin, INPUT_PULLUP);
  pinMode(kLowerLimitSwitchPin, INPUT_PULLUP);

  if (!SD.begin(BUILTIN_SDCARD)) {
  Serial.println("SD mount failed, halting and outputting light");
  while (1) {
    digitalWrite(ledPin, HIGH);
  }
  }

  soilData.begin(kSoilDataRoot);

}


/*------------------------------------------------- LOOP -----------------------------------------------------------*/

void loop(){
    enterLandedState();
    checkSensorConnections();

    if (landedFinalized ) {
      return;
    }

    delay(2000);

    startSoilLoggingIfNeeded();
    checkOrientationStep();

    //switch logic
    upperSwitchPressed = (digitalReadFast(kUpperLimitSwitchPin) == HIGH);
    lowerSwitchPressed = (digitalReadFast(kLowerLimitSwitchPin) == HIGH);

    upperStateChange = (upperSwitchPressed && !lastUpperSwitchPressed); //when the switch is pressed but wasnt before
    lowerStateChange = (lowerSwitchPressed && !lastLowerSwitchPressed);

    lastUpperSwitchPressed = upperSwitchPressed; //resetting for next time
    lastLowerSwitchPressed = lowerSwitchPressed;

    static elapsedMillis landedLogTimer;

    if (upperStateChange && topHits  == 0) { 
      Serial.println("upper switch pressed");
      leadScrewMotor.stopMotorWithCoast();
      topHits ++; 
    }

    if (topHits  == 0) {
      Serial.println("Retracting lead screw to find top switch");
      leadScrewMotor.moveMotorForward(kLeadScrewDutyCycle);
    }

    if(millis() - landedStartTime  >= kWaterTimeoutMs){
      waterMotor.stopMosfet();
      waterDispensed = true;

    }

    if (bottomHits  == 0 && topHits  == 1) {
      Serial.println("top reached once, drilling down and spinning auger");
      augerMotor.moveMosfet();
      leadScrewMotor.moveMotorBackward(kLeadScrewDutyCycle);
      isMovingUp = false;
    }

    if (isMovingUp==false && bottomHits  == 0 && lowerStateChange) {
      Serial.println("Lower switch pressed");
      bottomHits ++; //hit the bottom, wasnt there before, increase bottom hits
    }

    if (bottomHits  == 1 && topHits  == 1) {
      if (!augerSpinActive ) {
        Serial.println("bottom reached, stop lead screw, hold auger spin for 10s");
        leadScrewMotor.stopMotorWithCoast();
        leadScrewFullyExtended  = true;
        augerSpinActive  = true; //stop extending, keep spinning
        augerSpinTimer  = 0;
      }
    }

    if (leadScrewFullyExtended  && bottomHits  == 1 && topHits  == 1 &&
        augerSpinActive  && augerSpinTimer  >= kAugerSpinDurationMs) {
      Serial.println("Auger spin window complete, retracting lead screw");
      augerSpinActive  = false;
      leadScrewFullyExtended  = false;
      leadScrewMotor.moveMotorForward(kLeadScrewDutyCycle);
      isMovingUp = true;
    }

    if (isMovingUp && upperStateChange && bottomHits  == 1 && topHits  == 1) {
      Serial.print("Upp Switch pressed while retracting");
      leadScrewMotor.stopMotorWithCoast();
      isMovingUp = false;
      topHits ++;
    }

    if (isMovingUp == false && bottomHits  == 1 && topHits  == 2 && !waterDispensed && !waterGone) {
      Serial.println("drilling sequence complete, starting water motor");
      waterMotorStartTime = millis();
      waterMotor.moveMosfet();

    }

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
    
    if(millis() - waterMotorStartTime >= kWaterTimeoutMs){
      waterDispensed = true;
      waterGone = true;
      waterMotor.stopMosfet();

    }

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

    if (topHits  == 2 && bottomHits  == 1 && waterDispensed) {
      Serial.println("cycle complete, resetting counters to start the process again (no water pump this time)");
      enterLandedState();
    }
    delay(2000);
  
  }



