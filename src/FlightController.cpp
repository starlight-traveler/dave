#include "FlightController.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>

#include "Config.h"

/** @brief Construct controller with configured hardware. */
FlightController::FlightController(HardwareSerial &modbus)
    : modbus_(modbus),
      orientMotor_(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep),
      augerMotor_(kAugerMotorIn1, kAugerMotorIn2, kAugerMotorSleep),
      leadScrewMotor_(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep),
      waterMotor_(kWaterMotorIn1, kWaterMotorIn2, kWaterMotorSleep),
      soil_(modbus_, kRs485DirPin),
      flightData_(kFlightDataRoot, 8),
      soilData_(kSoilDataRoot, 4) {}

void FlightController::begin() {
  Serial.begin(9600);

  setupBMP(&bmp_);
  setupBNO085(&bno_);
  setupH3LIS331(&lis_);

  pinMode(kUpperLimitSwitchPin, INPUT);
  pinMode(kLowerLimitSwitchPin, INPUT);

  if (!SD.begin(BUILTIN_SDCARD)) {
    while (1) {
    }
  }

  soil_.begin(kSoilSensorBaud, kSoilSensorSlaveId);
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
  sensors_event_t accel = getH3LIS331Accel(&lis_);
  float32_t accelSquared = accel.acceleration.x * accel.acceleration.x
                         + accel.acceleration.y * accel.acceleration.y
                         + accel.acceleration.z * accel.acceleration.z;

  if (accelSquared > kAccelThresholdSquared) {
    state_ = INFLIGHT;
    inflightStartTime_ = millis();
    previousAltitude_ = getAltitude(&bmp_);
  }

  delay(50);
}

void FlightController::updateInflight() {
  const uint32_t timeDiffInFlight = millis() - inflightStartTime_;

  startFlightLoggingIfNeeded();
  flightData_.addFlightData(getH3LIS331Accel(&lis_), getBNO085Event(&bno_), getAltitude(&bmp_), dataFile_);

  currentAltitude_ = getAltitude(&bmp_);
  if (abs(currentAltitude_ - previousAltitude_) < kMinChangeInAltitude) {
    altitudeBelowThresholdCount_++;
  } else {
    altitudeBelowThresholdCount_ = 0;
  }

  if (altitudeBelowThresholdCount_ == kAltitudeBelowThresholdCount || timeDiffInFlight > kInflightTimeoutMs) {
    state_ = LANDED;
    landedStartTime_ = millis();
    finishFlightLogging();
  }

  previousAltitude_ = currentAltitude_;
  delay(30);
}

void FlightController::updateLanded() {
  startSoilLoggingIfNeeded();
  checkOrientation();

  if (topHits_ == 0) {
    leadScrewMotor_.moveMotorBackward(0.5f);
  }

  if (digitalRead(kUpperLimitSwitchPin) == HIGH) {
    topHits_++;
  }

  if (bottomHits_ == 0 && topHits_ == 1) {
    augerMotor_.moveMotorForward(1.0f);
    leadScrewMotor_.moveMotorForward(0.5f);
  }

  if (digitalRead(kLowerLimitSwitchPin) == HIGH) {
    bottomHits_++;
  }

  if (bottomHits_ == 1 && topHits_ == 1) {
    leadScrewMotor_.stopMotorWithCoast();
    leadScrewFullyExtended_ = true;
    delay(10000);
  }

  if (leadScrewFullyExtended_ && bottomHits_ == 1 && topHits_ == 1) {
    leadScrewFullyExtended_ = false;
    leadScrewMotor_.moveMotorBackward(0.5f);
  }

  if (digitalRead(kUpperLimitSwitchPin) == HIGH) {
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
    augerMotor_.stopMotorWithCoast();
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

void FlightController::checkOrientation() {
  sh2_SensorValue_t event = getBNO085Event(&bno_);

  while (event.un.gravity.z > -9.0f || event.un.gravity.z < -11.0f) {
    if (event.un.gravity.x > 0) {
      orientMotor_.moveMotorForward(1.0f);
    } else {
      orientMotor_.moveMotorBackward(1.0f);
    }
    event = getBNO085Event(&bno_);
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
  checkBMP390Connection(&bmp_);
  checkH3LIS331Connection(&lis_);
}
