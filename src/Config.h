#pragma once

#include <Arduino.h>
#include <arm_math.h>

// -------------------- Common --------------------
constexpr uint32_t kSerialBaud = 9600;
constexpr uint32_t kMainHeartbeatMs = 1000;

// -------------------- Soil/RS-485 --------------------
constexpr uint8_t kRs485DirPin = 4;
constexpr uint8_t kSoilSensorSlaveId = 0x01;
constexpr uint32_t kSoilSensorBaud = 9600;

// -------------------- GPIO --------------------
constexpr int32_t kUpperLimitSwitchPin = 2;
constexpr int32_t kLowerLimitSwitchPin = 3;

constexpr int32_t kOrientMotorIn1 = 27;
constexpr int32_t kOrientMotorIn2 = 26;
constexpr int32_t kOrientMotorSleep = 25;
constexpr int32_t kOrientMotorNFault = 24;

constexpr int32_t gpioAuger = 30;

constexpr int32_t kLeadScrewMotorIn1 = 11;
constexpr int32_t kLeadScrewMotorIn2 = 12;
constexpr int32_t kLeadScrewMotorSleep = -1;

constexpr int32_t kWaterMotorIn1 = 9;
constexpr int32_t kWaterMotorIn2 = 10;
constexpr int32_t kWaterMotorSleep = -1;

// -------------------- Flight Controller Thresholds --------------------
constexpr float32_t kAccelThresholdSquared = 864.36f;
constexpr float32_t kAccelSanityMaxAbsMs2 = 1500.0f;
constexpr float32_t kMinChangeInAltitude = 0.3f;
constexpr int32_t kAltitudeBelowThresholdCount = 5;

// -------------------- Flight Controller Timing --------------------
constexpr uint32_t kPreflightUpdatePeriodMs = 50;
constexpr uint32_t kInflightUpdatePeriodMs = 30;
constexpr uint32_t kPreflightTimeoutMs = 21600000;
constexpr uint32_t kInflightTimeoutMs = 180000;
constexpr uint32_t kLandedTimeoutMs = 900000;
constexpr uint32_t kAugerSpinDurationMs = 10000;
constexpr uint32_t kOrientationTimeoutMs = 5000;
constexpr uint32_t kSwitchPollPeriodMs = 5;
constexpr uint32_t kSensorHealthPollMs = 2000;

// -------------------- Motion Tuning --------------------
constexpr float32_t kLeadScrewDutyCycle = 0.5f;
constexpr float32_t kWaterDutyCycle = 0.75f;
constexpr float32_t kOrientationDutyCycle = 1.0f;
constexpr float32_t kOrientationAlignedZMin = -11.0f;
constexpr float32_t kOrientationAlignedZMax = -9.0f;

// -------------------- Logging/Buffering --------------------
constexpr uint32_t kPreflightLogPeriodMs = 500;
constexpr uint32_t kInflightLogPeriodMs = 500;
constexpr uint32_t kLandedLogPeriodMs = 1000;
constexpr uint32_t kOrientationLogPeriodMs = 500;
constexpr uint8_t kFlightDataBufferSize = 8;
constexpr uint8_t kSoilDataBufferSize = 4;
constexpr const char *kFlightDataRoot = "flightData";
constexpr const char *kSoilDataRoot = "soilData";

// -------------------- BNO085 --------------------
constexpr uint32_t kBnoReportIntervalUs = 10000;
constexpr uint32_t kBnoNoDataReconnectMs = 1500;
constexpr uint32_t kBnoReconnectBackoffMs = 2000;
constexpr uint8_t kBnoStartupInitAttempts = 3;
constexpr uint32_t kBnoStartupRetryDelayMs = 50;

// -------------------- Test Harness --------------------
constexpr uint32_t kTestBnoSamplePeriodMs = 2000;
constexpr uint32_t kTestMotorStepDelayMs = 5000;
