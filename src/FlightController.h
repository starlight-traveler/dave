#pragma once

#include <Arduino.h>
#include <SD.h>
#include <arm_math.h>
#include "BMP390.hpp"
#include "BNO055.hpp"
#include "H3LIS331.hpp"
#include "Soil485.h"
#include "driverSD.hpp"
#include "motorDriver.hpp"

/**
 * @brief Controls the rocket payload state machine and hardware orchestration.
 */
class FlightController {
public:
  /**
   * @brief Construct a controller bound to a Modbus serial interface.
   * @param modbus Serial port used for RS-485 soil sensor communication.
   */
  explicit FlightController(HardwareSerial &modbus);

  /** @brief Initialize sensors, storage, IO, and Modbus. */
  void begin();

  /** @brief Run one update tick of the state machine. */
  void update();

private:
  /** @brief Enumerates the high-level flight states. */
  enum FlightState {
    PREFLIGHT,
    INFLIGHT,
    LANDED,
  };

  /** @brief Run preflight state logic. */
  void updatePreflight();
  /** @brief Run inflight state logic and logging. */
  void updateInflight();
  /** @brief Run landed state logic and soil sampling. */
  void updateLanded();
  /** @brief Orient the payload so the auger faces downward. */
  void checkOrientation();
  /** @brief Open the flight log file if it is not already open. */
  void startFlightLoggingIfNeeded();
  /** @brief Open the soil log file if it is not already open. */
  void startSoilLoggingIfNeeded();
  /** @brief Flush and close the flight log file. */
  void finishFlightLogging();
  /** @brief Flush and close the soil log file. */
  void finishSoilLogging();
  /** @brief Poll sensor connectivity for fault detection. */
  void checkSensorConnections();
  /** @brief Non-blocking orientation alignment step. */
  void checkOrientationStep();
  /** @brief Poll limit switches at a fixed interval. */
  void pollLimitSwitches();
  /** @brief Human-readable state name for serial logs. */
  const char *stateName(FlightState state) const;

  HardwareSerial &modbus_;

  motorDriver orientMotor_;
  motorDriver augerMotor_;
  motorDriver leadScrewMotor_;
  motorDriver waterMotor_;

  Adafruit_BMP3XX bmp_;
  Adafruit_BNO055 bno_ =  Adafruit_BNO055(55, 0x28);
  Adafruit_H3LIS331 lis_;
  Soil485 soil_;

  File dataFile_;

  FlightState state_ = PREFLIGHT;
  FlightState lastLoggedState_ = PREFLIGHT;
  bool hasLoggedInitialState_ = false;
  uint32_t inflightStartTime_ = 0;
  uint32_t landedStartTime_ = 0;

  float32_t nitrogenMgKg_ = 0.0f;
  float32_t pH_ = 0.0f;
  float32_t electricalConductivity_ = 0.0f;

  int32_t topHits_ = 0;
  int32_t bottomHits_ = 0;
  int32_t altitudeBelowThresholdCount_ = 0;

  float32_t currentAltitude_ = 0.0f;
  float32_t previousAltitude_ = 0.0f;

  driverSD flightData_;
  driverSD soilData_;

  bool leadScrewFullyExtended_ = false;
  bool augerSpinActive_ = false;
  bool orientationAligned_ = false;
  bool upperSwitchPressed_ = false;
  bool lowerSwitchPressed_ = false;
  elapsedMillis preflightTimer_;
  elapsedMillis preflightStateTimer_;
  elapsedMillis inflightTimer_;
  elapsedMillis augerSpinTimer_;
  elapsedMillis orientationTimer_;
  elapsedMillis switchPollTimer_;
};
