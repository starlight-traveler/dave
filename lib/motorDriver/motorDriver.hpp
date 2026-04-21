#include <core_pins.h>

#ifndef PWMCONTROLMODE_H
#define PWMCONTROLMODE_H

class motorDriver {
    private:
        int dirPin = -1;
        int pwmPin = -1;
        int csPin = -1;
        int IN1 = -1;
        int IN2 = -1;
        int nSLEEP = -1;
        int nFault = -1;
        int DRVOFF = -1;

    public:
        //constructor
        motorDriver(int dir, int pwm, int currentSense, bool pololu);
        motorDriver(int in1, int in2, int nSleep);
        motorDriver(int in1, int in2, int nSleep, int nFault);
        motorDriver(int in1, int in2, int nSleep, int nFault, int DRVOFF);
        
        //other functions
        void moveForwardPololu();
        void moveBackwardPololu();
        int getCurrentSensePololu();
        void stopPololu();
        void moveMotorForward(float dutyCycle);
        void moveMotorBackward(float dutyCycle);
        void stopMotorWithCoast();
        bool nFaultPulledLow();
        void turnOnDRVOFF();
};

#endif
