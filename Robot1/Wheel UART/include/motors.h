#ifndef MOTORS_H
#define MOTORS_H

class Motor{
public:
    // Set the pin for the motor
    Motor(int pina, int pinb, int pwm);

    void initPins();

    // Set the PWM frequency for the motor
    void setPwmFreq(int frequency, int channel);

    // Drive the motor with a specific speed
    void driveMotors(int speed);

    //Test LED lights if the circuit is connected to
    void allOFF();
    void allON();

private:
    int motorPinA;
    int motorPinB;
    int motorPwm; 
    int pwmFrequency;
    int pwmChannel;
};

#endif // MOTORS_H