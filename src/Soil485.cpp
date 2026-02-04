#include "Soil485.h"

namespace {
constexpr uint8_t kFuncReadHolding = 0x03;

uint16_t wordFromBytes(uint8_t hi, uint8_t lo) {
  return static_cast<uint16_t>(hi) << 8 | lo;
}

uint32_t charTimeMicros(uint32_t baud) {
  if (baud == 0) {
    return 0;
  }
  // 11 bits: 1 start + 8 data + 1 stop + parity off
  return (11UL * 1000000UL) / baud;
}
}

Soil485::Soil485(HardwareSerial &serial, uint8_t dePin, int8_t rePin)
    : serial_(serial), dePin_(dePin), rePin_(rePin) {}

void Soil485::begin(uint32_t baud, uint8_t address) {
  address_ = address;
  pinMode(dePin_, OUTPUT);
  digitalWriteFast(dePin_, LOW);
  if (rePin_ >= 0) {
    pinMode(static_cast<uint8_t>(rePin_), OUTPUT);
    digitalWriteFast(static_cast<uint8_t>(rePin_), LOW);
  }
  serial_.begin(baud, SERIAL_8N1);
  serial_.setTimeout(timeoutMs_);

  const uint32_t settle = charTimeMicros(baud);
  if (settle > 0) {
    delayMicroseconds(static_cast<uint16_t>(settle));
  }
}

void Soil485::setAddress(uint8_t address) {
  address_ = address;
}

void Soil485::setTimeout(uint16_t timeoutMs) {
  timeoutMs_ = timeoutMs;
  serial_.setTimeout(timeoutMs_);
}

Soil485::Error Soil485::lastError() const {
  return lastError_;
}

bool Soil485::readTemperatureHumidity(float &temperatureC, float &humidityPercent) {
  uint16_t values[2] = {};
  if (!readRegisters(0x0012, 2, values)) {
    return false;
  }
  humidityPercent = static_cast<float>(values[0]) / 10.0f;
  temperatureC = static_cast<int16_t>(values[1]) / 10.0f;
  return true;
}

bool Soil485::readPH(float &ph) {
  uint16_t value = 0;
  if (!readRegister(0x0006, value)) {
    return false;
  }
  ph = static_cast<float>(value) / 100.0f;
  return true;
}

bool Soil485::readConductivity(uint16_t &conductivityUsCm) {
  return readRegister(0x0015, conductivityUsCm);
}

bool Soil485::readNPK(uint16_t &nitrogenMgKg, uint16_t &phosphorusMgKg, uint16_t &potassiumMgKg) {
  uint16_t values[3] = {};
  if (!readRegisters(0x001E, 3, values)) {
    return false;
  }
  nitrogenMgKg = values[0];
  phosphorusMgKg = values[1];
  potassiumMgKg = values[2];
  return true;
}

bool Soil485::readAll(Readings &out) {
  if (!readPH(out.ph)) {
    return false;
  }
  if (!readTemperatureHumidity(out.temperature_c, out.humidity_percent)) {
    return false;
  }
  if (!readConductivity(out.conductivity_us_cm)) {
    return false;
  }
  return readNPK(out.nitrogen_mg_kg, out.phosphorus_mg_kg, out.potassium_mg_kg);
}

bool Soil485::readRegister(uint16_t reg, uint16_t &value) {
  return readRegisters(reg, 1, &value);
}

bool Soil485::readRegisters(uint16_t startReg, uint16_t count, uint16_t *values) {
  if (count == 0 || values == nullptr) {
    setError(ErrFrame);
    return false;
  }

  if (!sendReadRequest(startReg, count)) {
    return false;
  }
  return readResponse(count, values);
}

void Soil485::setTxMode(bool enable) {
  digitalWriteFast(dePin_, enable ? HIGH : LOW);
  if (rePin_ >= 0) {
    digitalWriteFast(static_cast<uint8_t>(rePin_), enable ? HIGH : LOW);
  }
}

uint16_t Soil485::crc16(const uint8_t *data, size_t len) const {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

bool Soil485::sendReadRequest(uint16_t startReg, uint16_t count) {
  uint8_t frame[8];
  frame[0] = address_;
  frame[1] = kFuncReadHolding;
  frame[2] = static_cast<uint8_t>(startReg >> 8);
  frame[3] = static_cast<uint8_t>(startReg & 0xFF);
  frame[4] = static_cast<uint8_t>(count >> 8);
  frame[5] = static_cast<uint8_t>(count & 0xFF);
  const uint16_t crc = crc16(frame, 6);
  frame[6] = static_cast<uint8_t>(crc & 0xFF);
  frame[7] = static_cast<uint8_t>(crc >> 8);

  setTxMode(true);
  serial_.write(frame, sizeof(frame));
  serial_.flush();
  setTxMode(false);
  return true;
}

bool Soil485::readResponse(uint16_t count, uint16_t *values) {
  const size_t expected = 5 + static_cast<size_t>(count) * 2;
  if (expected > 64) {
    setError(ErrFrame);
    return false;
  }

  uint8_t buffer[64];
  size_t received = 0;
  const uint32_t start = millis();

  while (received < expected && (millis() - start) < timeoutMs_) {
    if (serial_.available()) {
      int byteIn = serial_.read();
      if (byteIn < 0) {
        continue;
      }
      buffer[received++] = static_cast<uint8_t>(byteIn);
    }
  }

  if (received != expected) {
    setError(ErrTimeout);
    return false;
  }

  const uint16_t crc = crc16(buffer, expected - 2);
  const uint16_t rxCrc = wordFromBytes(buffer[expected - 1], buffer[expected - 2]);
  if (crc != rxCrc) {
    setError(ErrCrc);
    return false;
  }

  if (buffer[0] != address_ || buffer[1] != kFuncReadHolding) {
    if (buffer[1] & 0x80) {
      setError(ErrException);
    } else {
      setError(ErrFrame);
    }
    return false;
  }

  const uint8_t byteCount = buffer[2];
  if (byteCount != count * 2) {
    setError(ErrFrame);
    return false;
  }

  for (uint16_t i = 0; i < count; ++i) {
    const uint8_t hi = buffer[3 + i * 2];
    const uint8_t lo = buffer[4 + i * 2];
    values[i] = wordFromBytes(hi, lo);
  }

  setError(Ok);
  return true;
}

void Soil485::setError(Error err) {
  lastError_ = err;
}
