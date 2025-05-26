#include <Arduino.h>
#include "motors.h"

#define BAUD_RATE 15200
#define HEADER 0xAA
#define MSG_LENGTH 22  // 1 (header) + 20 (5 ints) + 1 (checksum)

int X = 0, Y = 0, rot = 0, V1 = 0, V2 = 0;

Motor motorFL(2, 3);
Motor motorFR(4, 5);
Motor motorBL(6, 7);
Motor motorBR(8, 9);

void receiveMotorData() {
  static uint8_t buffer[MSG_LENGTH];
  static uint8_t index = 0;
  static bool receiving = false;

  while (Serial1.available()) {
    uint8_t byte = Serial1.read();
    //Serial.println("Available!.");
    if (!receiving) {
      if (byte == HEADER) {
        buffer[0] = byte;
        index = 1;
        receiving = true;
      }
    } else {
      buffer[index++] = byte;
      if (index == MSG_LENGTH) {
        receiving = false;

        // Calculate checksum
        uint8_t calcChecksum = 0;
        for (uint8_t i = 0; i < MSG_LENGTH - 1; i++) {
          calcChecksum += buffer[i];
        }

        if (calcChecksum == buffer[MSG_LENGTH - 1]) {
          int* intValues = (int*)(buffer + 1);

          X = intValues[0];
          Y = intValues[2];
          rot = intValues[4];
          V1 = intValues[6];
          V2 = intValues[8];
        }

        index = 0; // Reset for next message
      }
    }
  }
}

void setup() {
  Serial.begin(9600);       // Debug output
  Serial1.begin(BAUD_RATE); // Use Serial1 RX1 (pin 19)

  motorFL.initPins();
  motorFR.initPins();
  motorBL.initPins();
  motorBR.initPins();
  motorFL.shout();
  motorFR.shout();
  motorBL.shout();
  motorBR.shout();

  delay(500);
}

void loop() {
  receiveMotorData();

  Serial.print("X: ");
  Serial.print(X);
  Serial.print(", Y: ");
  Serial.print(Y);
  Serial.print(", rot: ");
  Serial.print(rot);
  Serial.print(", ");

  if (X > 0) {
    Serial.print("Right ");
  } else if (X < 0) {
    Serial.print("Left ");
  }

  if (Y > 0) {
    Serial.print("Forward ");
  } else if (Y < 0) {
    Serial.print("Backward ");
  }

  if (rot > 0) {
    Serial.print("Clockwise ");
  } else if (rot < 0) {
    Serial.print("Counterclockwise ");
  }

  if (X == 0 && Y == 0 && rot == 0) {
    Serial.print("Stationary");
  }

  Serial.println();

  // Calculate motor speeds based on X, Y, and rot
  
  int speedFL = Y + X + rot;
  int speedFR = Y - X - rot;
  int speedBL = Y - X + rot;
  int speedBR = Y + X - rot;

  // Constrain motor speeds to valid range (-255 to 255)
  speedFL = -1 * (map(constrain(speedFL, -255, 255), -255, 255, -128, 128));
  speedFR = -1 * (map(constrain(speedFR, -255, 255), -255, 255, -128, 128));
  speedBL = -1 * (map(constrain(speedBL, -255, 255), -255, 255, -128, 128));
  speedBR = -1 * (map(constrain(speedBR, -255, 255), -255, 255, -128, 128));

  // Drive motors with calculated speeds
  motorFL.driveMotors(speedFL);
  motorFR.driveMotors(speedFR);
  motorBL.driveMotors(speedBL);
  motorBR.driveMotors(speedBR);
  
  delay(50);
}