#include "motors.h"
#include <Arduino.h>

Motor::Motor(int pwma, int pwmb){
    pwmPinA = pwma;
    pwmPinB = pwmb;
}

void Motor::initPins(){
    pinMode(pwmPinA, OUTPUT);
    pinMode(pwmPinB, OUTPUT);
}

void Motor::driveMotors(int speed){
    if (speed > 0) {
        analogWrite(pwmPinA, speed);
        analogWrite(pwmPinB, 0);
    } else if (speed < 0) {
        analogWrite(pwmPinA, 0);
        analogWrite(pwmPinB, -speed);
    } else {
        analogWrite(pwmPinA, 0);
        analogWrite(pwmPinB, 0);
    }
}

void Motor::shout(){
    Serial.println("Motor class working!");
}
