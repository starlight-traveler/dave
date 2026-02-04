#pragma once

#include <Arduino.h>

class Soil485 {
public:
  enum Error : uint8_t {
    Ok = 0,
    ErrTimeout,
    ErrCrc,
    ErrException,
    ErrFrame,
  };

  struct Readings {
    float ph;
    float humidity_percent;
    float temperature_c;
    uint16_t conductivity_us_cm;
    uint16_t nitrogen_mg_kg;
    uint16_t phosphorus_mg_kg;
    uint16_t potassium_mg_kg;
  };

  Soil485(HardwareSerial &serial, uint8_t dePin, int8_t rePin = -1);

  void begin(uint32_t baud, uint8_t address = 0x01);
  void setAddress(uint8_t address);
  void setTimeout(uint16_t timeoutMs);
  Error lastError() const;

  bool readTemperatureHumidity(float &temperatureC, float &humidityPercent);
  bool readPH(float &ph);
  bool readConductivity(uint16_t &conductivityUsCm);
  bool readNPK(uint16_t &nitrogenMgKg, uint16_t &phosphorusMgKg, uint16_t &potassiumMgKg);
  bool readAll(Readings &out);

  bool readRegister(uint16_t reg, uint16_t &value);
  bool readRegisters(uint16_t startReg, uint16_t count, uint16_t *values);

private:
  HardwareSerial &serial_;
  uint8_t address_ = 0x01;
  uint8_t dePin_;
  int8_t rePin_;
  uint16_t timeoutMs_ = 200;
  Error lastError_ = Ok;

  void setTxMode(bool enable);
  uint16_t crc16(const uint8_t *data, size_t len) const;
  bool sendReadRequest(uint16_t startReg, uint16_t count);
  bool readResponse(uint16_t count, uint16_t *values);
  void setError(Error err);
};
