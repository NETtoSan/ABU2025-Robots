#include "motors.h"
#include <stdio.h>
#include <Arduino.h>


Motor::Motor(int pina, int pinb, int pwm){
    motorPinA = pina;
    motorPinB = pinb;
    motorPwm = pwm;
}

void Motor::initPins()
{
    if(motorPinA == 0 || motorPinB == 0 || motorPwm == 0) {
        Serial.println("Motor pins not set correctly!");
        return;
    }
    
    pinMode(motorPinA, OUTPUT);
    pinMode(motorPinB, OUTPUT);
    pinMode(motorPwm, OUTPUT);

    Serial.printf("Motor initialized!\n");
}

void Motor::setPwmFreq(int frequency, int channel) {
    pwmFrequency = frequency;
    pwmChannel = channel;
    ledcSetup(pwmChannel, pwmFrequency, 8); // 8-bit resolution
    ledcAttachPin(motorPwm, pwmChannel);
}

void Motor::driveMotors(int speed) {
    if (speed > 0) {
        digitalWrite(motorPinA, HIGH);
        digitalWrite(motorPinB, LOW);
    } else if (speed < 0) {
        digitalWrite(motorPinA, LOW);
        digitalWrite(motorPinB, HIGH);
    } else {
        digitalWrite(motorPinA, LOW);
        digitalWrite(motorPinB, LOW);
    }
    ledcWrite(pwmChannel, abs(speed)); // Use channel 0 for PWM
}

void Motor::allON(){
    if(motorPinA == 0 || motorPinB == 0){
        Serial.println("Some of the pins aren't set correctly!");
    }

    digitalWrite(motorPinA, HIGH);
    digitalWrite(motorPinB, HIGH);
}

void Motor::allOFF(){
    if(motorPinA == 0 || motorPinB == 0){
        Serial.println("Some of the pins aren't set correctly!");
    }

    digitalWrite(motorPinA, LOW);
    digitalWrite(motorPinB, LOW);
}