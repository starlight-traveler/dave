#include <core_pins.h>

#ifndef PWMCONTROLMODE_H
#define PWMCONTROLMODE_H

class motorDriver {
    private:
        int IN1;
        int IN2;
        int nSLEEP;

    public:
        //constructor
        motorDriver(int in1, int in2, int nSleep);
        
        //other functions
        void moveMotorForward(float dutyCycle);
        void moveMotorBackward(float dutyCycle);
        void stopMotorWithCoast();
};

#endif