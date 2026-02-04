#pragma once

#include <Arduino.h>

/**
 * @brief Modbus RTU helper for common RS-485 soil sensor modules.
 */
class Soil485 {
public:
  /** @brief Error codes for the last Modbus transaction. */
  enum Error : uint8_t {
    Ok = 0,
    ErrTimeout,
    ErrCrc,
    ErrException,
    ErrFrame,
  };

  /** @brief Aggregated readings returned by readAll(). */
  struct Readings {
    float ph;
    float humidity_percent;
    float temperature_c;
    uint16_t conductivity_us_cm;
    uint16_t nitrogen_mg_kg;
    uint16_t phosphorus_mg_kg;
    uint16_t potassium_mg_kg;
  };

  /**
   * @brief Create a Soil485 instance.
   * @param serial Serial port used for Modbus RTU.
   * @param dePin Driver enable (DE) pin for the RS-485 transceiver.
   * @param rePin Receiver enable (RE) pin for the RS-485 transceiver, or -1 if tied.
   */
  Soil485(HardwareSerial &serial, uint8_t dePin, int8_t rePin = -1);

  /**
   * @brief Initialize serial port and transceiver control pins.
   * @param baud Baud rate for Modbus RTU.
   * @param address Modbus slave address.
   */
  void begin(uint32_t baud, uint8_t address = 0x01);
  /** @brief Set the Modbus slave address. */
  void setAddress(uint8_t address);
  /** @brief Set the Modbus response timeout (ms). */
  void setTimeout(uint16_t timeoutMs);
  /** @brief Return the last error state. */
  Error lastError() const;

  /** @brief Read temperature and humidity registers. */
  bool readTemperatureHumidity(float &temperatureC, float &humidityPercent);
  /** @brief Read pH register. */
  bool readPH(float &ph);
  /** @brief Read conductivity register. */
  bool readConductivity(uint16_t &conductivityUsCm);
  /** @brief Read nitrogen, phosphorus, potassium registers. */
  bool readNPK(uint16_t &nitrogenMgKg, uint16_t &phosphorusMgKg, uint16_t &potassiumMgKg);
  /** @brief Read all available sensor registers. */
  bool readAll(Readings &out);

  /** @brief Read a single Modbus holding register. */
  bool readRegister(uint16_t reg, uint16_t &value);
  /** @brief Read multiple Modbus holding registers. */
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
