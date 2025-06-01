#include <Arduino.h>
#include "motors.h"

#define BAUD_RATE 15200
#define HEADER 0xAA
#define MSG_LENGTH 30  // 1 (header) + 28 (7 ints) + 1 (checksum)

int X = 0, Y = 0, rot = 0, V1 = 0, V2 = 0, L1 = 0, R1 = 0, actuatorState = 0;
bool shootRelay = false;

Motor motorFL(2, 3);
Motor motorFR(4, 5);
Motor motorBL(6, 7);
Motor motorBR(8, 9);

int shootUpA = 22;
int shootUpB = 33;
int shootUpPWM = 11;

int shootDownA = 24;
int shootDownB = 35;
int shootDownPWM = 10;

int actuatorPinA = 0;
int actuatorPinB = 0;
int actuatorPWM = 0;

// Change this to the correct pin numbers for your relays
int shootRelayPin = 0;
int blockRelayPin = 53;
// ------------------------------------------------------

void shout(){
  Serial.print("X: ");
  Serial.print(X);
  Serial.print(", Y: ");
  Serial.print(Y);
  Serial.print(", rot: ");
  Serial.print(rot);
  Serial.print(", L1: ");
  Serial.print(L1);
  Serial.print(", R1: ");
  Serial.print(R1);
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
}
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
          rot = -1 * intValues[4];
          V1 = intValues[6];
          V2 = intValues[8];
          L1 = intValues[10];
          R1 = intValues[12];
          //shootRelay = intValues[14];
          //actuatorState = intValues[16];
        }

        index = 0; // Reset for next message
      }
    }
  }
}

void setup() {
  Serial.begin(9600);       // Debug output
  Serial1.begin(BAUD_RATE); // Use Serial1 RX1 (pin 19)


  pinMode(blockRelayPin, OUTPUT);
  pinMode(shootUpA, OUTPUT);
  pinMode(shootUpB, OUTPUT);
  pinMode(shootUpPWM, OUTPUT);
  pinMode(shootDownA, OUTPUT);
  pinMode(shootDownB, OUTPUT);
  pinMode(shootDownPWM, OUTPUT);

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
  shout(); // Print debug information

  // Calculate motor speeds based on X, Y, and rot
    
  int speedFL = Y + X + rot;
  int speedFR = Y - X - rot;
  int speedBL = Y - X + rot;
  int speedBR = Y + X - rot;

  // Constrain motor speeds to valid range (-255 to 255)
  speedFL = -1 * (map(constrain(speedFL, -255, 255), -255, 255, -150, 150));
  speedFR = -1 * (map(constrain(speedFR, -255, 255), -255, 255, -150, 150));
  speedBL = -1 * (map(constrain(speedBL, -255, 255), -255, 255, -150, 150));
  speedBR = -1 * (map(constrain(speedBR, -255, 255), -255, 255, -150, 150));

  // Drive motors with calculated speeds
  motorFL.driveMotors(speedFL);
  motorFR.driveMotors(speedFR);
  motorBL.driveMotors(speedBL);
  motorBR.driveMotors(speedBR);
  
  if(L1 == 1){
    digitalWrite(shootUpA, HIGH);
    digitalWrite(shootUpB, LOW);
    analogWrite(shootUpPWM, 255);

    digitalWrite(shootDownA, HIGH);
    digitalWrite(shootDownB, LOW);
    analogWrite(shootDownPWM, 255);
  }
  else if(R1 == 1){
    digitalWrite(shootUpA, LOW);
    digitalWrite(shootUpB, HIGH);
    analogWrite(shootUpPWM, 255);

    digitalWrite(shootDownA, LOW);
    digitalWrite(shootDownB, HIGH);
    analogWrite(shootDownPWM, 255);
  }
  else{
    digitalWrite(shootUpA, LOW);
    digitalWrite(shootUpB, LOW);
    analogWrite(shootUpPWM, 0);
    
    digitalWrite(shootDownA, LOW);
    digitalWrite(shootDownB, LOW);
    analogWrite(shootDownPWM, 0);
  }

  delay(50);
}