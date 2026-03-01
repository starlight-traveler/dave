#include <Arduino.h>
#include "Soil485.h"
#include "Config.h"
#include "FlightController.h"
#include "DebugLog.h"

    float32_t nitrogenMgKg_ = 0.0f;
    float32_t pH_ = 0.0f;
    float32_t electricalConductivity_ = 0.0f;

    
    float phReading = pH_;
    uint16_t conductivity = static_cast<uint16_t>(electricalConductivity_);
    uint16_t nitrogen = static_cast<uint16_t>(nitrogenMgKg_);
    uint16_t phosphorus = 0;
    uint16_t potassium = 0;

    Soil485 soilSensor = Soil485(Serial1, kRs485DirPin);


void setup(){

    LOG_BEGIN(kSerialBaud);
    Soil485 soilSensor = Soil485(Serial1, kRs485DirPin);

    LOG_PRINTLN(F("[FC] begin(): initializing RS-485 soil sensor bus"));
    soilSensor.begin(kSoilSensorBaud, kSoilSensorSlaveId);

}

void loop(){
  if (soilSensor.readPH(phReading)) {
    pH_ = phReading;
    LOG_PRINT(F("[FC][LANDED] pH update="));
    LOG_PRINTLN(pH_, 2);
  }
  if (soilSensor.readConductivity(conductivity)) {
    electricalConductivity_ = static_cast<float32_t>(conductivity);
    LOG_PRINT(F("[FC][LANDED] conductivity update(us/cm)="));
    LOG_PRINTLN(electricalConductivity_, 1);
  }
  if (soilSensor.readNPK(nitrogen, phosphorus, potassium)) {
    nitrogenMgKg_ = static_cast<float32_t>(nitrogen);
    LOG_PRINT(F("[FC][LANDED] NPK update N/P/K="));
    LOG_PRINT(nitrogen);
    LOG_PRINT(F("/"));
    LOG_PRINT(phosphorus);
    LOG_PRINT(F("/"));
    LOG_PRINTLN(potassium);
  }


}