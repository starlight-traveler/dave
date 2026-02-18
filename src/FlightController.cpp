#include "FlightController.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Config.h"
#include "DebugLog.h"

namespace {
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

float32_t squaredMagnitude(float32_t x, float32_t y, float32_t z) {
  float32_t vec[3] = {x, y, z};
  float32_t magnitudeSquared = 0.0f;
  arm_dot_prod_f32(vec, vec, 3, &magnitudeSquared);
  return magnitudeSquared;
}
}

/** @brief Construct controller with configured hardware. */
FlightController::FlightController(HardwareSerial &modbus)
    : modbus_(modbus),
      orientMotor_(kOrientMotorIn1, kOrientMotorIn2, kOrientMotorSleep, kOrientMotorNFault),
      augerMotor_(gpioAuger),
      leadScrewMotor_(kLeadScrewMotorIn1, kLeadScrewMotorIn2, kLeadScrewMotorSleep),
      waterMotor_(kWaterMotorIn1, kWaterMotorIn2, kWaterMotorSleep),
      soil_(modbus_, kRs485DirPin),
      flightData_(kFlightDataRoot, kFlightDataBufferSize),
      soilData_(kSoilDataRoot, kSoilDataBufferSize) {}

void FlightController::begin() {
  LOG_BEGIN(kSerialBaud);
  LOG_PRINTLN(F("[FC] begin(): booting flight controller"));
  LOG_PRINTLN(F("[FC] begin(): forcing auger MOSFET OFF to avoid floating gate"));

  //setting pin 30 to low so it doesnt float
  augerMotor_.stopMosfet();

  setupBMP(&bmp_);
  LOG_PRINTLN(F("[FC] begin(): initializing BNO085"));
  setupBNO055(&bno_);
  setupICM20649(&icm);

  LOG_PRINTLN(F("[FC] begin(): configuring limit switch GPIO"));
  pinMode(kUpperLimitSwitchPin, INPUT_PULLUP);
  pinMode(kLowerLimitSwitchPin, INPUT_PULLUP);

  LOG_PRINTLN(F("[FC] begin(): mounting SD card"));
  if (!SD.begin(BUILTIN_SDCARD)) {
    LOG_PRINTLN(F("[FC] begin(): SD mount FAILED, halting"));
    while (1) {
    }
  }
  LOG_PRINTLN(F("[FC] begin(): SD mount OK"));

  LOG_PRINTLN(F("[FC] begin(): initializing RS-485 soil sensor bus"));
  soil_.begin(kSoilSensorBaud, kSoilSensorSlaveId);
  preflightStateTimer_ = 0;
  LOG_PRINT(F("[FC] begin(): state initialized to "));
  LOG_PRINTLN(stateName(state_));
}

void FlightController::update() {
  if (!hasLoggedInitialState_ || lastLoggedState_ != state_) {
    LOG_PRINT(F("[FC] update(): state transition -> "));
    LOG_PRINTLN(stateName(state_));
    lastLoggedState_ = state_;
    hasLoggedInitialState_ = true;
  }

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
  if (preflightTimer_ < kPreflightUpdatePeriodMs) {
    return;
  }
  preflightTimer_ = 0;

  sensors_event_t accelICM = getICM20649Accel(&icm);
  const bool icmAccelSane = accelVectorIsSane(
      accelICM.acceleration.x, accelICM.acceleration.y, accelICM.acceleration.z);
  const float32_t accelICMSquared = icmAccelSane
      ? squaredMagnitude(accelICM.acceleration.x, accelICM.acceleration.y, accelICM.acceleration.z)
      : 0.0f;

  const imu::Vector<3> bnoLinearAccel = bno_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  const bool bnoAccelSane = accelVectorIsSane(
      bnoLinearAccel.x(), bnoLinearAccel.y(), bnoLinearAccel.z());
  const float32_t accelBNOSquared = bnoAccelSane
      ? squaredMagnitude(bnoLinearAccel.x(), bnoLinearAccel.y(), bnoLinearAccel.z())
      : 0.0f;

  float currentAlt = getAltitude(&bmp_); //getting current altitude

  static elapsedMillis preflightLogTimer;
  if (preflightLogTimer >= kPreflightLogPeriodMs) {
    preflightLogTimer = 0;
    LOG_PRINT(F("[FC][PREFLIGHT] accel^2="));
    LOG_PRINT(accelICMSquared, 3);
    LOG_PRINT(F(" threshold="));
    LOG_PRINT(kAccelThresholdSquared, 3);
    LOG_PRINT(F(" elapsed_ms="));
    LOG_PRINTLN(static_cast<uint32_t>(preflightStateTimer_));
    LOG_PRINT(F("[FC][PREFLIGHT] accel sanity ICM/BNO="));
    LOG_PRINT(icmAccelSane ? F("OK") : F("BAD"));
    LOG_PRINT(F("/"));
    LOG_PRINTLN(bnoAccelSane ? F("OK") : F("BAD"));
  }

  if (icmAccelSane && bnoAccelSane &&
      accelICMSquared > kAccelThresholdSquared &&
      accelBNOSquared > kAccelThresholdSquared) { // make sure both sensors read above the threshold
    LOG_PRINTLN(F("[FC][PREFLIGHT] launch threshold crossed -> INFLIGHT"));
    state_ = INFLIGHT;
    inflightStartTime_ = millis();
    previousAltitude_ = currentAlt;
    preflightStateTimer_ = 0;
  }

  if (preflightStateTimer_ >= kPreflightTimeoutMs) {
    LOG_PRINTLN(F("[FC][PREFLIGHT] timeout reached -> LANDED"));
    enterLandedState();
  }

}

void FlightController::updateInflight() {
  if (inflightTimer_ < kInflightUpdatePeriodMs) {
    return;
  }
  inflightTimer_ = 0;

  const uint32_t timeDiffInFlight = millis() - inflightStartTime_;
  sensors_event_t accel = getICM20649Accel(&icm);
  sensors_event_t orient = getBNO055Event(&bno_);
  float32_t altitude = getAltitude(&bmp_);

  startFlightLoggingIfNeeded();
  flightData_.addFlightData(accel, orient, altitude, dataFile_);
  static elapsedMillis inflightLogTimer;
  if (inflightLogTimer >= kInflightLogPeriodMs) {
    inflightLogTimer = 0;
    LOG_PRINT(F("[FC][INFLIGHT] t_ms="));
    LOG_PRINT(timeDiffInFlight);
    LOG_PRINT(F(" accel_xyz=("));
    LOG_PRINT(accel.acceleration.x, 3);
    LOG_PRINT(F(","));
    LOG_PRINT(accel.acceleration.y, 3);
    LOG_PRINT(F(","));
    LOG_PRINT(accel.acceleration.z, 3);
    LOG_PRINTLN(F(")"));
  }

  currentAltitude_ = altitude;
  if (absScalarF32(currentAltitude_ - previousAltitude_) < kMinChangeInAltitude) {
    altitudeBelowThresholdCount_++;
  } else {
    altitudeBelowThresholdCount_ = 0;
  }

  if (altitudeBelowThresholdCount_ == kAltitudeBelowThresholdCount || timeDiffInFlight > kInflightTimeoutMs) {
    LOG_PRINTLN(F("[FC][INFLIGHT] timeout reached -> LANDED"));
    finishFlightLogging();
    enterLandedState();
  }

  previousAltitude_ = currentAltitude_;
}

void FlightController::updateLanded() {
  if (landedFinalized_) {
    return;
  }

  startSoilLoggingIfNeeded();
  checkOrientationStep();
  pollLimitSwitches();
  static elapsedMillis landedLogTimer;
  const bool upperRisingEdge = upperSwitchPressed_ && !upperSwitchLatched_;
  const bool lowerRisingEdge = lowerSwitchPressed_ && !lowerSwitchLatched_;
  upperSwitchLatched_ = upperSwitchPressed_;
  lowerSwitchLatched_ = lowerSwitchPressed_;

  if (topHits_ == 0) {
    LOG_PRINTLN(F("[FC][LANDED] retracting lead screw to find top switch"));
    leadScrewMotor_.moveMotorBackward(kLeadScrewDutyCycle);
  }

  if (upperRisingEdge && topHits_ == 0) {
    LOG_PRINTLN(F("[FC][LANDED] upper switch active"));
    topHits_++;
  }

  if (bottomHits_ == 0 && topHits_ == 1) {
    LOG_PRINTLN(F("[FC][LANDED] top reached once -> drilling down and spinning auger"));
    augerMotor_.moveMosfet();
    leadScrewMotor_.moveMotorForward(kLeadScrewDutyCycle);
  }

  if (lowerRisingEdge && bottomHits_ == 0) {
    LOG_PRINTLN(F("[FC][LANDED] lower switch active"));
    bottomHits_++;
  }

  if (bottomHits_ == 1 && topHits_ == 1) {
    if (!augerSpinActive_) {
      LOG_PRINTLN(F("[FC][LANDED] bottom reached -> stop lead screw, hold auger spin for 10s"));
      leadScrewMotor_.stopMotorWithCoast();
      leadScrewFullyExtended_ = true;
      augerSpinActive_ = true;
      augerSpinTimer_ = 0;
    }
  }

  if (leadScrewFullyExtended_ && bottomHits_ == 1 && topHits_ == 1 &&
      augerSpinActive_ && augerSpinTimer_ >= kAugerSpinDurationMs) {
    LOG_PRINTLN(F("[FC][LANDED] auger spin window complete -> retracting lead screw"));
    augerSpinActive_ = false;
    leadScrewFullyExtended_ = false;
    leadScrewMotor_.moveMotorBackward(kLeadScrewDutyCycle);
  }

  if (upperRisingEdge && !leadScrewFullyExtended_ && bottomHits_ == 1 && topHits_ == 1) {
    LOG_PRINTLN(F("[FC][LANDED] upper switch active while retracting -> stop lead screw"));
    leadScrewMotor_.stopMotorWithCoast();
    topHits_++;
  }

  if (!leadScrewFullyExtended_ && bottomHits_ == 1 && topHits_ == 2) {
    LOG_PRINTLN(F("[FC][LANDED] drilling sequence complete -> starting water motor"));
    waterMotor_.moveMotorForward(kWaterDutyCycle);
  }

  float phReading = pH_;
  uint16_t conductivity = static_cast<uint16_t>(electricalConductivity_);
  uint16_t nitrogen = static_cast<uint16_t>(nitrogenMgKg_);
  uint16_t phosphorus = 0;
  uint16_t potassium = 0;

  if (soil_.readPH(phReading)) {
    pH_ = phReading;
    LOG_PRINT(F("[FC][LANDED] pH update="));
    LOG_PRINTLN(pH_, 2);
  }
  if (soil_.readConductivity(conductivity)) {
    electricalConductivity_ = static_cast<float32_t>(conductivity);
    LOG_PRINT(F("[FC][LANDED] conductivity update(us/cm)="));
    LOG_PRINTLN(electricalConductivity_, 1);
  }
  if (soil_.readNPK(nitrogen, phosphorus, potassium)) {
    nitrogenMgKg_ = static_cast<float32_t>(nitrogen);
    LOG_PRINT(F("[FC][LANDED] NPK update N/P/K="));
    LOG_PRINT(nitrogen);
    LOG_PRINT(F("/"));
    LOG_PRINT(phosphorus);
    LOG_PRINT(F("/"));
    LOG_PRINTLN(potassium);
  }

  soilData_.addSoilSensorData(nitrogenMgKg_, pH_, electricalConductivity_, dataFile_);
  if (landedLogTimer >= kLandedLogPeriodMs) {
    landedLogTimer = 0;
    LOG_PRINT(F("[FC][LANDED] counters top/bottom="));
    LOG_PRINT(topHits_);
    LOG_PRINT(F("/"));
    LOG_PRINT(bottomHits_);
    LOG_PRINT(F(" orientAligned="));
    LOG_PRINT(orientationAligned_ ? F("yes") : F("no"));
    LOG_PRINT(F(" leadExtended="));
    LOG_PRINTLN(leadScrewFullyExtended_ ? F("yes") : F("no"));
  }

  if (millis() - landedStartTime_ >= kLandedTimeoutMs) {
    LOG_PRINTLN(F("[FC][LANDED] landed timeout reached -> stopping all motors"));
    augerMotor_.stopMosfet();
    leadScrewMotor_.stopMotorWithCoast();
    waterMotor_.stopMotorWithCoast();
    orientMotor_.stopMotorWithCoast();

    finishSoilLogging();
    landedFinalized_ = true;
    return;
  }

  if (topHits_ == 2 && bottomHits_ == 1) {
    LOG_PRINTLN(F("[FC][LANDED] cycle complete -> resetting counters"));
    topHits_ = 0;
    bottomHits_ = 0;
  }

  state_ = LANDED;
}

void FlightController::enterLandedState() {
  state_ = LANDED;
  landedStartTime_ = millis();
  orientationAligned_ = false;
  orientationTimer_ = 0;
  switchPollTimer_ = 0;
  upperSwitchPressed_ = false;
  lowerSwitchPressed_ = false;
  upperSwitchLatched_ = false;
  lowerSwitchLatched_ = false;
  topHits_ = 0;
  bottomHits_ = 0;
  leadScrewFullyExtended_ = false;
  augerSpinActive_ = false;
  landedFinalized_ = false;
}

void FlightController::checkOrientationStep() {
  if (orientationAligned_) {
    return;
  }

  const imu::Vector<3> gravity = bno_.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  const float32_t gravityX = gravity.x();
  const float32_t gravityY = gravity.y();
  const float32_t gravityZ = gravity.z();
  if (!accelVectorIsSane(gravityX, gravityY, gravityZ)) {
    static elapsedMillis invalidGravityLogTimer;
    if (invalidGravityLogTimer >= kOrientationLogPeriodMs) {
      invalidGravityLogTimer = 0;
      LOG_PRINTLN(F("[FC][LANDED][ORIENT] gravity invalid, skipping orientation step"));
    }
    orientMotor_.stopMotorWithCoast();
    return;
  }

  static elapsedMillis orientLogTimer;
  if (orientLogTimer >= kOrientationLogPeriodMs) {
    orientLogTimer = 0;
    LOG_PRINT(F("[FC][LANDED][ORIENT] gravity xyz=("));
    LOG_PRINT(gravityX, 3);
    LOG_PRINT(F(","));
    LOG_PRINT(gravityY, 3);
    LOG_PRINT(F(","));
    LOG_PRINT(gravityZ, 3);
    LOG_PRINTLN(F(")"));
  }

  if (gravityZ <= kOrientationAlignedZMax &&
     gravityZ >= kOrientationAlignedZMin) {
    orientMotor_.stopMotorWithCoast();
    orientationAligned_ = true;
    LOG_PRINTLN(F("[FC][LANDED][ORIENT] z-axis aligned -> orientation complete"));
    return;
  }

  if (gravityX > 0) {
    LOG_PRINTLN(F("[FC][LANDED][ORIENT] tilting +x -> driving motor forward"));
    orientMotor_.moveMotorForward(kOrientationDutyCycle);
  } else {
    LOG_PRINTLN(F("[FC][LANDED][ORIENT] tilting -x -> driving motor backward"));
    orientMotor_.moveMotorBackward(kOrientationDutyCycle);
  }

  if (orientationTimer_ >= kOrientationTimeoutMs) {
    orientMotor_.stopMotorWithCoast();
    orientationAligned_ = true;
    LOG_PRINTLN(F("[FC][LANDED][ORIENT] orientation timeout -> stopping motor"));
  }
}

void FlightController::startFlightLoggingIfNeeded() {
  if (flightData_.getCurrentIndex() == 0) {
    LOG_PRINT(F("[FC][INFLIGHT] opening flight log file: "));
    LOG_PRINTLN(flightData_.getCurrentFileName());
    dataFile_ = SD.open(flightData_.getCurrentFileName(), FILE_WRITE);
    LOG_PRINTLN(dataFile_ ? F("[FC][INFLIGHT] file open OK") : F("[FC][INFLIGHT] file open FAILED"));
  }
}

void FlightController::startSoilLoggingIfNeeded() {
  if (soilData_.getCurrentIndex() == 0) {
    LOG_PRINT(F("[FC][LANDED] opening soil log file: "));
    LOG_PRINTLN(soilData_.getCurrentFileName());
    dataFile_ = SD.open(soilData_.getCurrentFileName(), FILE_WRITE);
    LOG_PRINTLN(dataFile_ ? F("[FC][LANDED] file open OK") : F("[FC][LANDED] file open FAILED"));
  }
}

void FlightController::finishFlightLogging() {
  LOG_PRINTLN(F("[FC][INFLIGHT] finalizing flight log"));
  flightData_.increaseCurrentIndexBy(-1);
  flightData_.printFlightDataToFile(dataFile_);
}

void FlightController::finishSoilLogging() {
  LOG_PRINTLN(F("[FC][LANDED] finalizing soil log"));
  soilData_.increaseCurrentIndexBy(-1);
  soilData_.printSoilDataToFile(dataFile_);
}

void FlightController::checkSensorConnections() {
  static elapsedMillis sensorLogTimer;
  if (sensorLogTimer < kSensorHealthPollMs) {
    return;
  }
  sensorLogTimer = 0;
  LOG_PRINTLN(F("[FC] checkSensorConnections(): polling BNO health"));
  checkBNO055Connection(&bno_);
  checkBMP390Connection(&bmp_);
  checkICMConnection(&icm);
}

void FlightController::pollLimitSwitches() {
  if (switchPollTimer_ < kSwitchPollPeriodMs) {
    return;
  }
  switchPollTimer_ = 0;
  const bool prevUpper = upperSwitchPressed_;
  const bool prevLower = lowerSwitchPressed_;
  upperSwitchPressed_ = (digitalReadFast(kUpperLimitSwitchPin) == LOW);
  lowerSwitchPressed_ = (digitalReadFast(kLowerLimitSwitchPin) == LOW);
  if (prevUpper != upperSwitchPressed_ || prevLower != lowerSwitchPressed_) {
    LOG_PRINT(F("[FC][LANDED] switch change upper/lower="));
    LOG_PRINT(upperSwitchPressed_ ? F("PRESSED") : F("RELEASED"));
    LOG_PRINT(F("/"));
    LOG_PRINTLN(lowerSwitchPressed_ ? F("PRESSED") : F("RELEASED"));
  }
}

const char *FlightController::stateName(FlightState state) const {
  switch (state) {
    case PREFLIGHT:
      return "PREFLIGHT";
    case INFLIGHT:
      return "INFLIGHT";
    case LANDED:
      return "LANDED";
    default:
      return "UNKNOWN";
  }
}
