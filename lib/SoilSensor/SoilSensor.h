#include <HardwareSerial.h>
#include <Arduino.h>
#include <usb_seremu.h>
#include <usb_serial.h>
#include <arm_math.h>

#ifndef SoilSensor_H
#define SoilSensor_H

uint16_t modbusCRC(uint8_t *buf, int len, uint8_t RS485_DIR_PIN, uint8_t SLAVE_ID, HardwareSerial &modbus);
void sendReadHoldingRegisters(uint16_t reg, uint8_t RS485_DIR_PIN, uint8_t SLAVE_ID, HardwareSerial &modbus);
bool readRegister(uint16_t reg, float32_t &value, uint8_t RS485_DIR_PIN, uint8_t SLAVE_ID, HardwareSerial &modbus);

#endif

