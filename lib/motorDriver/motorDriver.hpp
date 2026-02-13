#include <core_pins.h>

#ifndef PWMCONTROLMODE_H
#define PWMCONTROLMODE_H

class motorDriver {
    private:
        int IN1;
        int IN2;
        int nSLEEP;
        int nFault;
        int DRVOFF;
        bool drvControlOn = false;

    public:
        //constructor
        motorDriver(int in1, int in2, int nSleep);
        motorDriver(int in1, int in2, int nSleep, int nFault);
        motorDriver(int in1, int in2, int nSleep, int nFault, int DRVOFF);
        
        //other functions
        void moveMotorForward(float dutyCycle);
        void moveMotorBackward(float dutyCycle);
        void stopMotorWithCoast();
        bool nFaultPulledLow();
        void turnOnDRVOFF();
};

#endif