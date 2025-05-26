#include <Arduino.h>

// Define the PWM pins for the IBT-2 motor controller
const int L_PWM = 9; // Left PWM pin (connect to IBT-2 LPWM)
const int R_PWM = 10; // Right PWM pin (connect to IBT-2 RPWM)

// Function to control the motor
void setMotorSpeed(int speed) {
  // Constrain the speed to the range -100 to 100
  speed = constrain(speed, -100, 100);

  if (speed > 0) {
    // Forward direction
    analogWrite(L_PWM, speed * 2.55); // Scale 0-100 to 0-255
    analogWrite(R_PWM, 0);
  } else if (speed < 0) {
    // Reverse direction
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, abs(speed) * 2.55); // Scale 0-100 to 0-255
  } else {
    // Stop the motor
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, 0);
  }
}

void setup() {
  // Initialize the PWM pins as outputs
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  // Example: Set motor speed to 50 (forward)
  setMotorSpeed(50);
}

void loop() {
  // Example: Change motor speed dynamically
  for (int speed = -100; speed <= 100; speed += 10) {
    setMotorSpeed(speed);
    delay(500); // Wait 500ms
  }
  for (int speed = 100; speed >= -100; speed -= 10) {
    setMotorSpeed(speed);
    delay(500); // Wait 500ms
  }
}