#include <Arduino.h>
#include <SD.h>
#include <arm_math.h>
#include <Adafruit_Sensor.h>

#ifndef driverSD_H
#define driverSD_H

class driverSD {
    private:
        String fileName;
        float32_t dataBuffer[40][10];
        int numBufferCol;
        int dataBufferIndex = 0;

    public:
        //constructor
        driverSD(const char* fileRoot, int numOfColumns);
        
        //other functions
        void findCurrentFileName(String fileRoot);
        const char* getCurrentFileName();
        int getCurrentIndex();
        void increaseCurrentIndexBy(int increment);
        void addFlightData(sensors_event_t accel, sensors_event_t event, float32_t altitude, File dataFile);
        void addSoilSensorData(float32_t nitrogenPercentage, float32_t pH, float32_t electricalConductivity, File dataFile);
        void printFlightDataToFile(File dataFile);
        void printSoilDataToFile(File dataFile);

};

#endif
