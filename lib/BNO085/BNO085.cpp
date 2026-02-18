#include "BNO085.hpp"

#include <Arduino.h>

namespace {
constexpr uint32_t kReportIntervalUs = 10000;       // 100 Hz
constexpr uint32_t kNoDataReconnectMs = 1500;       // No samples before considering reconnect
constexpr uint32_t kReconnectBackoffMs = 2000;      // Avoid rapid begin_I2C loops
constexpr uint8_t kStartupInitAttempts = 3;

uint32_t gLastEventMs = 0;
uint32_t gLastReconnectAttemptMs = 0;

bool configureReports(Adafruit_BNO08x *bno) {
  const bool accelEnabled = bno->enableReport(SH2_ACCELEROMETER, kReportIntervalUs);
  const bool gravityEnabled = bno->enableReport(SH2_GRAVITY, kReportIntervalUs);
  return accelEnabled && gravityEnabled;
}

bool reconnectAndConfigure(Adafruit_BNO08x *bno) {
  if (!bno->begin_I2C()) {
    return false;
  }
  return configureReports(bno);
}
}

// this function will check the connection to the bno, and try again if it fails
void checkBNO085Connection(Adafruit_BNO08x *bno) {
  if (bno == nullptr) {
    return;
  }

  sh2_SensorValue_t event = {};
  if (bno->getSensorEvent(&event)) {
    gLastEventMs = millis();
    return;
  }

  const uint32_t now = millis();
  if (gLastEventMs == 0) {
    gLastEventMs = now;
  }

  const bool dataTimedOut = (now - gLastEventMs) >= kNoDataReconnectMs;
  const bool backoffElapsed = (now - gLastReconnectAttemptMs) >= kReconnectBackoffMs;
  if (!dataTimedOut || !backoffElapsed) {
    return;
  }

  gLastReconnectAttemptMs = now;
  if (reconnectAndConfigure(bno)) {
    gLastEventMs = now;
  }
}

// this function will set up the bno
void setupBNO085(Adafruit_BNO08x *bno) {
  if (bno == nullptr) {
    return;
  }

  for (uint8_t attempt = 0; attempt < kStartupInitAttempts; ++attempt) {
    if (reconnectAndConfigure(bno)) {
      gLastEventMs = millis();
      gLastReconnectAttemptMs = 0;
      return;
    }
    delay(50);
  }
}

// this function will get the orientation from the bno, returning the overall event
sh2_SensorValue_t getBNO085Event(Adafruit_BNO08x *bno) {
  sh2_SensorValue_t event = {};
  if (bno != nullptr && bno->getSensorEvent(&event)) {
    gLastEventMs = millis();
    return event;
  }

  // Don't reconnect on every miss; check routine has timeout/backoff protection.
  checkBNO085Connection(bno);
  return event;
}
