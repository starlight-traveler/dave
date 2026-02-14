#include "FlightController.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>

#include "Config.h"

/** @brief Construct controller with configured hardware. */
FlightController::FlightController(HardwareSerial &modbus)
    : modbus_(modbus),
      orientMotor_(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep, kOrientMotorNFault),
      augerMotor_(gpioAuger),
      leadScrewMotor_(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep),
      waterMotor_(kWaterMotorIn1, kWaterMotorIn2, kWaterMotorSleep),
      soil_(modbus_, kRs485DirPin),
      flightData_(kFlightDataRoot, 8),
      soilData_(kSoilDataRoot, 4) {}

void FlightController::begin() {

  //setting pin 30 to low so it doesnt float
    augerMotor_.stopMosfet();

    Serial.begin(9600);

  //setupBMP(&bmp_);
  setupBNO085(&bno_);
  //setupH3LIS331(&lis_);

  pinMode(kUpperLimitSwitchPin, INPUT_PULLUP);
  pinMode(kLowerLimitSwitchPin, INPUT_PULLUP);

  if (!SD.begin(BUILTIN_SDCARD)) {
    while (1) {
    }
  }

  soil_.begin(kSoilSensorBaud, kSoilSensorSlaveId);
  preflightStateTimer_ = 0;
}

void FlightController::update() {
  checkSensorConnections();

  switch (state_) {
    case PREFLIGHT:
      updatePreflight();
      break;
    case INFLIGHT:
      updateInflight();
      break;
    case LANDED:
      updateLanded();
      break;
    default:
      state_ = PREFLIGHT;
      break;
  }
}

void FlightController::updatePreflight() {
  if (preflightTimer_ < 50) {
    return;
  }
  preflightTimer_ = 0;

  //sensors_event_t accel = getH3LIS331Accel(&lis_);
  // float32_t accelSquared = accel.acceleration.x * accel.acceleration.x
  //                        + accel.acceleration.y * accel.acceleration.y
  //                        + accel.acceleration.z * accel.acceleration.z;

  sh2_SensorValue_t event = getBNO085Event(&bno_);

  float32_t accelSquared = event.un.accelerometer.x*event.un.accelerometer.x + event.un.accelerometer.y*event.un.accelerometer.y + 
  event.un.accelerometer.z*event.un.accelerometer.z;

  if (accelSquared > kAccelThresholdSquared) {
    state_ = INFLIGHT;
    inflightStartTime_ = millis();
    //previousAltitude_ = getAltitude(&bmp_);
    preflightStateTimer_ = 0;
  }

  if (preflightStateTimer_ >= kPreflightTimeoutMs) {
    state_ = LANDED;
    landedStartTime_ = millis();
    orientationAligned_ = false;
    orientationTimer_ = 0;
    switchPollTimer_ = 0;
  }

}

void FlightController::updateInflight() {
  if (inflightTimer_ < 30) {
    return;
  }
  inflightTimer_ = 0;

  const uint32_t timeDiffInFlight = millis() - inflightStartTime_;
  //sensors_event_t accel = getH3LIS331Accel(&lis_);
  sh2_SensorValue_t bnoEvent = getBNO085Event(&bno_);
  //float32_t altitude = getAltitude(&bmp_);

  startFlightLoggingIfNeeded();
  //flightData_.addFlightData(accel, bnoEvent, altitude, dataFile_);
  flightData_.addFlightData(bnoEvent, dataFile_);

  // currentAltitude_ = altitude;
  // if (abs(currentAltitude_ - previousAltitude_) < kMinChangeInAltitude) {
  //   altitudeBelowThresholdCount_++;
  // } else {
  //   altitudeBelowThresholdCount_ = 0;
  // }

  // if (altitudeBelowThresholdCount_ == kAltitudeBelowThresholdCount || timeDiffInFlight > kInflightTimeoutMs) {
  if (timeDiffInFlight > kInflightTimeoutMs) {
    state_ = LANDED;
    landedStartTime_ = millis();
    orientationAligned_ = false;
    orientationTimer_ = 0;
    switchPollTimer_ = 0;
    finishFlightLogging();
  }

  previousAltitude_ = currentAltitude_;
}

void FlightController::updateLanded() {
  startSoilLoggingIfNeeded();
  checkOrientationStep();
  pollLimitSwitches();

  if (topHits_ == 0) {
    leadScrewMotor_.moveMotorBackward(0.5f);
  }

  if (upperSwitchPressed_) {
    topHits_++;
  }

  if (bottomHits_ == 0 && topHits_ == 1) {
    augerMotor_.moveMosfet();
    leadScrewMotor_.moveMotorForward(0.5f);
  }

  if (lowerSwitchPressed_) {
    bottomHits_++;
  }

  if (bottomHits_ == 1 && topHits_ == 1) {
    if (!augerSpinActive_) {
      leadScrewMotor_.stopMotorWithCoast();
      leadScrewFullyExtended_ = true;
      augerSpinActive_ = true;
      augerSpinTimer_ = 0;
    }
  }

  if (leadScrewFullyExtended_ && bottomHits_ == 1 && topHits_ == 1 && augerSpinActive_ && augerSpinTimer_ >= 10000) {
    augerSpinActive_ = false;
    leadScrewFullyExtended_ = false;
    leadScrewMotor_.moveMotorBackward(0.5f);
  }

  if (upperSwitchPressed_) {
    leadScrewMotor_.stopMotorWithCoast();
    topHits_++;
  }

  if (!leadScrewFullyExtended_ && bottomHits_ == 1 && topHits_ == 2) {
    waterMotor_.moveMotorForward(0.75f);
  }

  float phReading = pH_;
  uint16_t conductivity = static_cast<uint16_t>(electricalConductivity_);
  uint16_t nitrogen = static_cast<uint16_t>(nitrogenMgKg_);
  uint16_t phosphorus = 0;
  uint16_t potassium = 0;

  if (soil_.readPH(phReading)) {
    pH_ = phReading;
  }
  if (soil_.readConductivity(conductivity)) {
    electricalConductivity_ = static_cast<float32_t>(conductivity);
  }
  if (soil_.readNPK(nitrogen, phosphorus, potassium)) {
    nitrogenMgKg_ = static_cast<float32_t>(nitrogen);
  }

  soilData_.addSoilSensorData(nitrogenMgKg_, pH_, electricalConductivity_, dataFile_);

  if (millis() - landedStartTime_ >= kLandedTimeoutMs) {
    augerMotor_.stopMosfet();
    leadScrewMotor_.stopMotorWithCoast();
    waterMotor_.stopMotorWithCoast();
    orientMotor_.stopMotorWithCoast();

    finishSoilLogging();
  }

  if (topHits_ == 2 && bottomHits_ == 1) {
    topHits_ = 0;
    bottomHits_ = 0;
  }

  state_ = LANDED;
}

void FlightController::checkOrientationStep() {
  if (orientationAligned_) {
    return;
  }

  sh2_SensorValue_t event = getBNO085Event(&bno_);

  if (event.un.gravity.z <= -9.0f && event.un.gravity.z >= -11.0f) {
    orientMotor_.stopMotorWithCoast();
    orientationAligned_ = true;
    return;
  }

  if (event.un.gravity.x > 0) {
    orientMotor_.moveMotorForward(1.0f);
  } else {
    orientMotor_.moveMotorBackward(1.0f);
  }

  if (orientationTimer_ >= 5000) {
    orientMotor_.stopMotorWithCoast();
    orientationAligned_ = true;
  }
}

void FlightController::startFlightLoggingIfNeeded() {
  if (flightData_.getCurrentIndex() == 0) {
    dataFile_ = SD.open(flightData_.getCurrentFileName(), FILE_WRITE);
  }
}

void FlightController::startSoilLoggingIfNeeded() {
  if (soilData_.getCurrentIndex() == 0) {
    dataFile_ = SD.open(soilData_.getCurrentFileName(), FILE_WRITE);
  }
}

void FlightController::finishFlightLogging() {
  flightData_.increaseCurrentIndexBy(-1);
  flightData_.printFlightDataToFile(dataFile_);
}

void FlightController::finishSoilLogging() {
  soilData_.increaseCurrentIndexBy(-1);
  soilData_.printSoilDataToFile(dataFile_);
}

void FlightController::checkSensorConnections() {
  checkBNO085Connection(&bno_);
  //checkBMP390Connection(&bmp_);
  //checkH3LIS331Connection(&lis_);
}

void FlightController::pollLimitSwitches() {
  if (switchPollTimer_ < 5) {
    return;
  }
  switchPollTimer_ = 0;
  upperSwitchPressed_ = (digitalReadFast(kUpperLimitSwitchPin) == LOW);
  lowerSwitchPressed_ = (digitalReadFast(kLowerLimitSwitchPin) == LOW);
}
