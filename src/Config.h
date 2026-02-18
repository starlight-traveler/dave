#pragma once

#include <Arduino.h>
#include <arm_math.h>

// -------------------- Common --------------------
constexpr uint32_t kSerialBaud = 9600;          // UART baud rate for debug logs over USB serial.
constexpr uint32_t kMainHeartbeatMs = 1000;     // Heartbeat print period from main loop (milliseconds).

// -------------------- Soil/RS-485 --------------------
constexpr uint8_t kRs485DirPin = 4;             // RS-485 DE/RE direction control GPIO.
constexpr uint8_t kSoilSensorSlaveId = 0x01;    // Modbus slave address of the soil sensor.
constexpr uint32_t kSoilSensorBaud = 9600;      // UART baud rate for the soil sensor bus.

// -------------------- GPIO --------------------
constexpr int32_t kUpperLimitSwitchPin = 2;     // Upper travel limit switch input pin (active low with pull-up).
constexpr int32_t kLowerLimitSwitchPin = 3;     // Lower travel limit switch input pin (active low with pull-up).

constexpr int32_t kOrientMotorIn1 = 27;         // Orientation motor driver IN1/PWM pin.
constexpr int32_t kOrientMotorIn2 = 26;         // Orientation motor driver IN2/PWM pin.
constexpr int32_t kOrientMotorSleep = 25;       // Orientation motor driver nSLEEP pin.
constexpr int32_t kOrientMotorNFault = 24;      // Orientation motor driver nFAULT pin.

constexpr int32_t gpioAuger = 30;               // Auger MOSFET gate control pin.

constexpr int32_t kLeadScrewMotorIn1 = 11;      // Lead screw motor driver IN1/PWM pin.
constexpr int32_t kLeadScrewMotorIn2 = 12;      // Lead screw motor driver IN2/PWM pin.
constexpr int32_t kLeadScrewMotorSleep = -1;    // Lead screw nSLEEP pin; -1 means not connected/unused.

constexpr int32_t kWaterMotorIn1 = 9;           // Water pump motor driver IN1/PWM pin.
constexpr int32_t kWaterMotorIn2 = 10;          // Water pump motor driver IN2/PWM pin.
constexpr int32_t kWaterMotorSleep = -1;        // Water motor nSLEEP pin; -1 means not connected/unused.

// -------------------- Flight Controller Thresholds --------------------
constexpr float32_t kAccelThresholdSquared = 864.36f;         // Launch accel threshold (m/s^2)^2; 864.36 ~= (29.4 m/s^2)^2.
constexpr float32_t kAccelSanityMaxAbsMs2 = 1500.0f;          // Absolute accel sanity bound per axis in m/s^2 (reject NaN/Inf/outliers).
constexpr uint8_t kLaunchDetectConsecutiveSamples = 5;        // Required consecutive preflight samples above launch threshold.
constexpr float32_t kLandingAccelThresholdSquared = 4.0f;     // Landing low-accel threshold (m/s^2)^2; 4.0 = (2 m/s^2)^2.
constexpr float32_t kLandingAltitudeDeltaThresholdM = 0.3f;   // Max per-sample altitude change (meters) considered "stable".
constexpr uint8_t kLandingDetectConsecutiveSamples = 20;      // Required consecutive inflight samples meeting landing criteria.
constexpr float32_t kAltitudeValidMinM = -500.0f;             // Minimum valid barometric altitude (meters).
constexpr float32_t kAltitudeValidMaxM = 100000.0f;           // Maximum valid barometric altitude (meters).

// -------------------- Flight Controller Timing --------------------
constexpr uint32_t kPreflightUpdatePeriodMs = 50;             // Preflight state update period (milliseconds).
constexpr uint32_t kInflightUpdatePeriodMs = 30;              // Inflight state update period (milliseconds).
constexpr uint32_t kPreflightTimeoutMs = 21600000;            // Max time to stay in preflight before failover to landed (ms, 6 hours).
constexpr uint32_t kInflightTimeoutMs = 180000;               // Hard inflight timeout to force landed transition (ms, 3 minutes).
constexpr uint32_t kMinInflightBeforeLandingEvalMs = 5000;    // Lockout time before landing checks are allowed (ms, 5 seconds).
constexpr uint32_t kLandedTimeoutMs = 900000;                 // Max landed operation duration before stopping motors (ms, 15 minutes).
constexpr uint32_t kAugerSpinDurationMs = 10000;              // Extra auger spin hold time after full extension (milliseconds).
constexpr uint32_t kOrientationTimeoutMs = 5000;              // Max orientation motor run time before forcing complete (milliseconds).
constexpr uint32_t kSwitchPollPeriodMs = 5;                   // Limit switch polling interval (milliseconds).
constexpr uint32_t kSensorHealthPollMs = 2000;                // Sensor connectivity/health check interval (milliseconds).

// -------------------- Motion Tuning --------------------
constexpr float32_t kLeadScrewDutyCycle = 0.5f;               // Lead screw motor PWM duty cycle (0.0 to 1.0).
constexpr float32_t kWaterDutyCycle = 0.75f;                  // Water motor PWM duty cycle (0.0 to 1.0).
constexpr float32_t kOrientationDutyCycle = 1.0f;             // Orientation motor PWM duty cycle (0.0 to 1.0).
constexpr float32_t kOrientationAlignedZMin = -11.0f;         // Lower gravity-Z bound for "aligned" orientation (m/s^2).
constexpr float32_t kOrientationAlignedZMax = -9.0f;          // Upper gravity-Z bound for "aligned" orientation (m/s^2).

// -------------------- Logging/Buffering --------------------
constexpr uint32_t kPreflightLogPeriodMs = 500;               // Serial log period during preflight (milliseconds).
constexpr uint32_t kInflightLogPeriodMs = 500;                // Serial log period during inflight (milliseconds).
constexpr uint32_t kLandedLogPeriodMs = 1000;                 // Serial log period during landed operations (milliseconds).
constexpr uint32_t kOrientationLogPeriodMs = 500;             // Orientation-specific serial log period (milliseconds).
constexpr uint8_t kFlightDataBufferSize = 8;                  // Number of numeric columns written per flight log row.
constexpr uint8_t kSoilDataBufferSize = 4;                    // Number of numeric columns written per soil log row.
constexpr const char *kFlightDataRoot = "flightData";         // Base filename/root used for flight data files.
constexpr const char *kSoilDataRoot = "soilData";             // Base filename/root used for soil data files.

// -------------------- BNO085 --------------------
constexpr uint32_t kBnoReportIntervalUs = 10000;              // Desired BNO sensor report period in microseconds.
constexpr uint32_t kBnoNoDataReconnectMs = 1500;              // No-data timeout before attempting BNO reconnect (milliseconds).
constexpr uint32_t kBnoReconnectBackoffMs = 2000;             // Minimum spacing between reconnect attempts (milliseconds).
constexpr uint8_t kBnoStartupInitAttempts = 3;                // Number of startup initialization/reconnect attempts.
constexpr uint32_t kBnoStartupRetryDelayMs = 50;              // Delay between BNO startup retries (milliseconds).

// -------------------- Test Harness --------------------
constexpr uint32_t kTestBnoSamplePeriodMs = 2000;             // Sample period used by BNO test sketch (milliseconds).
constexpr uint32_t kTestMotorStepDelayMs = 5000;              // Delay between motor test steps (milliseconds).
