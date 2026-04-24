#pragma once

#include <Arduino.h>
#include <arm_math.h>

// full retract time: ~12s
// plunge time:
// 

/*---------------------CONSTANTS------------------------*/

// -------------------- Common --------------------
 const uint32_t kSerialBaud = 9600;          // UART baud rate for debug logs over USB serial.
 const uint32_t kMainHeartbeatMs = 1000;     // Heartbeat print period from main loop (milliseconds).

// -------------------- Soil/RS-485 --------------------
 const uint8_t kRs485DirPin = 21;             // RS-485 DE/RE direction control GPIO.
 const uint8_t kSoilSensorSlaveId = 0x01;    // Modbus slave address of the soil sensor.
 const uint32_t kSoilSensorBaud = 9600;      // UART baud rate for the soil sensor bus.

// -------------------- GPIO --------------------
 const int32_t kUpperLimitSwitchPin = 2;     // Upper travel limit switch input pin (active low with pull-up).
 const int32_t kLowerLimitSwitchPin = 3;     // Lower travel limit switch input pin (active low with pull-up).

 const int32_t kOrientMotorIn1 = 36;         // Orientation motor driver IN1/PWM pin.
 const int32_t kOrientMotorIn2 = 37;         // Orientation motor driver IN2/PWM pin.
 const int32_t kOrientMotorSleep = 38;       // Orientation motor driver nSLEEP pin.

 const  int32_t kAugerPWMPin = 12;
 const  int32_t kAugerDIRPin = 41;
 const  int32_t kAugerCurrentSensePin = 40;
 
const  int32_t kWaterMotorIn1 = 24;          // Water motor driver IN1/PWM pin.
const  int32_t kWaterMotorIn2 = 25;         // Water motor driver IN2/PWM pin.
const  int32_t kWaterMotorSleep = -1;       // Water motor driver nSLEEP pin (not used)

 const int32_t kLeadScrewMotorIn1 = 28;      // Lead screw motor driver IN1/PWM pin.
 const int32_t kLeadScrewMotorIn2 = 29;      // Lead screw motor driver IN2/PWM pin.
 const  int32_t kLeadScrewMotorSleep = -1;    // Lead screw nSLEEP pin; -1 means not connected/unused.

 const  int32_t ledPin = 13;

// -------------------- Flight Controller Thresholds --------------------
const float32_t kAccelThresholdSquared = 180.0f;         // Launch accel threshold (m/s^2)^2; 864.36 ~= (29.4 m/s^2)^2.
const float32_t kAccelSanityMaxAbsMs2 = 1500.0f;          // Absolute accel sanity bound per axis in m/s^2 (reject NaN/Inf/outliers).
const uint8_t kLaunchDetectConsecutiveSamples = 2;        // Required consecutive preflight samples above launch threshold.
const float32_t kLandingAccelThresholdSquared = 4.0f;     // Landing low-accel threshold (m/s^2)^2; 4.0 = (2 m/s^2)^2.
const float32_t kAccelTestThresholdSquared = 4.0f;
const float32_t kLandingAltitudeDeltaThresholdM = 0.5f;   // Max per-sample altitude change (meters) considered "stable".
const uint8_t kLandingDetectConsecutiveSamples = 5;      // Required consecutive inflight samples meeting landing criteria.
const float32_t kAltitudeValidMinM = -500.0f;             // Minimum valid barometric altitude (meters).
const float32_t kAltitudeValidMaxM = 100000.0f;           // Maximum valid barometric altitude (meters).
const uint32_t kCyclesUpperLowerNotHit = 4;
const float32_t kDraggedAccelThresholdSquared = 4.0f;

// -------------------- Flight Controller Timing --------------------
 const uint32_t kPreflightUpdatePeriodMs = 50;             // Preflight state update period (milliseconds).
 const  uint32_t kInflightUpdatePeriodMs = 30;              // Inflight state update period (milliseconds).
 const uint32_t kPreflightTimeoutMs = 21600000;            // Max time to stay in preflight before failover to landed (ms, 6 hours).
 const  uint32_t kInflightTimeoutMs = 15000;               // Hard inflight timeout to force landed transition (ms, 3 minutes).
 const uint32_t kMinInflightBeforeLandingEvalMs = 5000;    // Lockout time before landing checks are allowed (ms, 5 seconds).
 const  uint32_t kLandedTimeoutMs = 60000;                 // Max landed operation duration before stopping motors (ms, 15 minutes).
 const  uint32_t kAugerSpinDurationMs = 12000;              // Extra auger spin hold time after full extension (milliseconds).
 const  uint32_t kOrientationTimeoutMs = 10000;              // Max orientation motor run time before forcing complete (milliseconds).
 const  uint32_t kSwitchPollPeriodMs = 5;                   // Limit switch polling interval (milliseconds).
 const  uint32_t kSensorHealthPollMs = 2000;                // Sensor connectivity/health check interval (milliseconds).
 const  uint32_t kWaterTimeoutMs = 90000;                   // the time water will run before stopping
 const uint32_t kRetractPeriod = 2000;                      // how long to retract for (usually 1/2 plunge time)
 const uint32_t kPlungePeriod = 3000;                      // how long to plunge for
 const uint32_t kPlungeTimeToBottomLimit = 10000;           // 10 seconds
 const uint32_t kRetractTimeToTopLimit = 25000;              //25 seconds
 const uint32_t kReversePeriod = 3000;                       // period of reversing auger when jammed
 const uint32_t kGroundPeriod = 480000;                      // time to wait on ground until deploy anyways (8min)

// -------------------- Motion Tuning --------------------
 const float32_t kLeadScrewDutyCycle = 1.0f;               // Lead screw motor PWM duty cycle (0.0 to 1.0).
 const  float32_t kWaterDutyCycle = 1.0f;                  // Water motor PWM duty cycle (0.0 to 1.0).
 const  float32_t kOrientationDutyCycle = 1.0f;             // Orientation motor PWM duty cycle (0.0 to 1.0).
 const float32_t kOrientationAlignedYMin = 35.0f;         // Lower gravity-Z bound for "aligned" orientation (m/s^2).
 const  float32_t kOrientationAlignedYMax = 48.0f;          // Upper gravity-Z bound for "aligned" orientation (m/s^2).
 const uint32_t kCurrentThresh = 70;                         // current threshhold for auger, theoretically a bit under ~20A

// -------------------- Logging/Buffering --------------------
 const uint32_t kPreflightLogPeriodMs = 500;               // Serial log period during preflight (milliseconds).
 const uint32_t kInflightLogPeriodMs = 500;                // Serial log period during inflight (milliseconds).
 const uint32_t kLandedLogPeriodMs = 1000;                 // Serial log period during landed operations (milliseconds).
 const uint32_t kSoilDataRequestPeriodMs = 30;
 const uint32_t kOrientationLogPeriodMs = 500;             // Orientation-specific serial log period (milliseconds).
 const uint8_t kFlightDataBufferSize = 8;                  // Number of numeric columns written per flight log row.
 const uint8_t kSoilDataBufferSize = 4;                    // Number of numeric columns written per soil log row.
 inline const char *kFlightDataRoot = "flightData";         // Base filename/root used for flight data files.
 inline const char *kSoilDataRoot = "soilData";             // Base filename/root used for soil data files.

// -------------------- Test Harness --------------------
 const uint32_t kTestBnoSamplePeriodMs = 2000;             // Sample period used by BNO test sketch (milliseconds).
 const uint32_t kTestMotorStepDelayMs = 5000;              // Delay between motor test steps (milliseconds).