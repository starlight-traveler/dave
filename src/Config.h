#pragma once

#include <Arduino.h>
#include <arm_math.h>

/**
 * @brief RS-485 direction pin for the soil sensor transceiver.
 * @warning This pin assignment matches the existing code; verify against hardware.
 */
constexpr uint8_t kRs485DirPin = 2;

/** @brief Modbus slave ID for the soil sensor. */
constexpr uint8_t kSoilSensorSlaveId = 0x01;

/** @brief Modbus baud rate for the soil sensor. */
constexpr uint32_t kSoilSensorBaud = 9600;

/**
 * @brief Upper limit switch pin for the lead screw.
 * @warning This pin assignment matches the existing code; verify against hardware.
 */
constexpr int32_t kUpperLimitSwitchPin = 2;

/** @brief Lower limit switch pin for the lead screw. */
constexpr int32_t kLowerLimitSwitchPin = 3;

/** @brief Motor driver pins for the orientation motor. */
constexpr int32_t kOrientMotorIn1 = 27;
constexpr int32_t kOrientMotorIn2 = 26;
constexpr int32_t kOrientMotorSleep = 25;

/** @brief Motor driver pins for the auger motor. */
constexpr int32_t kAugerMotorIn1 = 31;
constexpr int32_t kAugerMotorIn2 = 30;
constexpr int32_t kAugerMotorSleep = 29;

/** @brief Motor driver pins for the lead screw motor. */
constexpr int32_t kLeadScrewMotorIn1 = 11;
constexpr int32_t kLeadScrewMotorIn2 = 12;
constexpr int32_t kLeadScrewMotorSleep = -1;

/** @brief Motor driver pins for the water motor. */
constexpr int32_t kWaterMotorIn1 = 9;
constexpr int32_t kWaterMotorIn2 = 10;
constexpr int32_t kWaterMotorSleep = -1;

/** @brief Squared acceleration threshold (m/s^2)^2 to enter flight state (~4g). */
constexpr float32_t kAccelThresholdSquared = 1536.64f;

/** @brief Minimum change in altitude (m) to consider still changing during flight. */
constexpr float32_t kMinChangeInAltitude = 0.3f;

/** @brief Number of consecutive samples below altitude change threshold to mark landed. */
constexpr int32_t kAltitudeBelowThresholdCount = 5;

/** @brief Maximum inflight time (ms) before forcing landed state. */
constexpr uint32_t kInflightTimeoutMs = 180000;

/** @brief Maximum preflight wait (ms) before forcing landed/drill sequence. */
constexpr uint32_t kPreflightTimeoutMs = 21600000;

/** @brief Maximum landed time (ms) before shutting down motors/logging. */
constexpr uint32_t kLandedTimeoutMs = 900000;

/** @brief Flight data file root name. */
constexpr const char *kFlightDataRoot = "flightData";

/** @brief Soil data file root name. */
constexpr const char *kSoilDataRoot = "soilData";
