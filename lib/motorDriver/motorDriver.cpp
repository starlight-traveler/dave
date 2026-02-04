#include "motorDriver.hpp"

//Here are the datasheets for all of the motor drivers being used:
//https://www.ti.com/lit/ds/symlink/drv8874.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1768564255162&ref_url=https%253A%252F%252Fwww.ti.com%252Fgeneral%252Fdocs%252Fsuppproductinfo.tsp%253FdistId%253D10%2526gotoUrl%253Dhttps%253A%252F%252Fwww.ti.com%252Flit%252Fgpn%252Fdrv8874
//https://www.ti.com/lit/ds/symlink/drv8244-q1.pdf?ts=1701345632184
//https://www.ti.com/lit/ds/symlink/drv8212p.pdf

motorDriver::motorDriver(int in1, int in2, int nSleep){
    IN1 = in1;
    IN2 = in2;
    nSLEEP = nSleep;

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(nSLEEP, OUTPUT);

    if (nSleep != -1) {
        digitalWrite(nSLEEP, HIGH);
    }
}

//this function will cause the motor to move forward
void motorDriver::moveMotorForward(float dutyCycle) {
    analogWrite(IN1, int(255*dutyCycle)); //it seems like high is the same as 1 or true
    analogWrite(IN2, 0);   //low is the same as 0 or false
}

//this function will cause the motor to move backward
void motorDriver::moveMotorBackward(float dutyCycle) {
    analogWrite(IN1, 0);
    analogWrite(IN2, int(255*dutyCycle));

}

//this function will stop the motor
void motorDriver::stopMotorWithCoast() {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
}