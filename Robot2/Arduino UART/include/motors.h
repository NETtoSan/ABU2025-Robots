#ifndef MOTORS_H
#define MOTORS_H

class Motor{
public:
    // Set the pin for the motor
    Motor(int pwma, int pwmb);

    void initPins();

    // Drive the motor with a specific speed
    void driveMotors(int speed);

    // Print a message whether the class is working
    void shout();
private:
    int pwmPinA;
    int pwmPinB;
    int motorPwm;
};

#endif // MOTORS_H