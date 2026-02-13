#include "motorDriver.hpp"

//Here are the datasheets for all of the motor drivers being used:
//https://www.ti.com/lit/ds/symlink/drv8874.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1768564255162&ref_url=https%253A%252F%252Fwww.ti.com%252Fgeneral%252Fdocs%252Fsuppproductinfo.tsp%253FdistId%253D10%2526gotoUrl%253Dhttps%253A%252F%252Fwww.ti.com%252Flit%252Fgpn%252Fdrv8874
//https://www.ti.com/lit/ds/symlink/drv8244-q1.pdf?ts=1701345632184
//https://www.ti.com/lit/ds/symlink/drv8212p.pdfint

motorDriver::motorDriver(int gpio){
    IN1 = gpio;
    pinMode(IN1, INPUT);
}

motorDriver::motorDriver(int in1, int in2, int nSleep){
    IN1 = in1;
    IN2 = in2;
    nSLEEP = nSleep;

    pinMode(IN1, INPUT);
    pinMode(IN2, INPUT);
    pinMode(nSLEEP, INPUT);

    if (nSleep != -1) {
        digitalWrite(nSLEEP, HIGH);
    }
}

motorDriver::motorDriver(int in1, int in2, int nSleep, int nfault){
    IN1 = in1;
    IN2 = in2;
    nSLEEP = nSleep;
    nFault = nfault;

    pinMode(IN1, INPUT);
    pinMode(IN2, INPUT);
    pinMode(nSLEEP, INPUT);
    pinMode(nFault, INPUT);

    if (nSleep != -1) {
        digitalWrite(nSLEEP, HIGH);
    }
}

motorDriver::motorDriver(int in1, int in2, int nSleep, int nfault, int drvoff){
    IN1 = in1;
    IN2 = in2;
    nSLEEP = nSleep;
    nFault = nfault;
    DRVOFF = drvoff;

    pinMode(IN1, INPUT);
    pinMode(IN2, INPUT);
    pinMode(nSLEEP, INPUT);
    pinMode(nFault, INPUT);
    pinMode(DRVOFF, INPUT);

    //in order to turn the motor with DRVOFF, the pin must be set to 0 in order for anything to run. If it is
    // set to one, it nothing will work
        digitalWrite(DRVOFF, LOW);


    if (nSleep != -1) {
        digitalWrite(nSLEEP, HIGH);
    }
}

void motorDriver::moveMosfet(){
    digitalWrite(IN1, HIGH);
}

void motorDriver::stopMosfet(){
    digitalWrite(IN1, LOW);
}

//this function will cause the motor to move forward
void motorDriver::moveMotorForward(float dutyCycle) {
    if(drvControlOn==0){
    analogWrite(IN1, int(255*dutyCycle)); //it seems like high is the same as 1 or true
    analogWrite(IN2, 0);   //low is the same as 0 or false
    }
}

//this function will cause the motor to move backward
void motorDriver::moveMotorBackward(float dutyCycle) {
    if(drvControlOn==0){
    analogWrite(IN1, 0);
    analogWrite(IN2, int(255*dutyCycle));
    }

}

//this function will stop the motor
void motorDriver::stopMotorWithCoast() {
    if(drvControlOn==false){
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
    }
}

//this function will get the status of the nFault pin (if the motor has it), and will return true if the nFault pin has returned low 
// voltage (indecating) something is wrong, and false if the pin returns high voltage, which indicates everything is alright
bool motorDriver::nFaultPulledLow(){
    if(drvControlOn==false){
    if (digitalRead(nFault) == LOW)
        return true;
    else
        return false;
    }
}

//this method will check to see if the nFault pin is true (which means something is wrong with the motor), and if so
//turning the DRVOFF pin on high so it stops working overall
void motorDriver::turnOnDRVOFF(){
    if(nFaultPulledLow()==true){
        digitalWrite(DRVOFF, HIGH);
        drvControlOn = true;
    }
}