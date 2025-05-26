#include <Arduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>

// UART communication pins
#define RX_PIN 16
#define TX_PIN 17

// IBT-2 Motor Driver Pins
#define MOTOR_L_PWM 25 // Left PWM pin for speed control
#define MOTOR_R_PWM 26 // Right PWM pin for speed control
#define MOTOR_L_EN 27  // Left enable pin for direction
#define MOTOR_R_EN 14  // Right enable pin for direction

// Initialize SoftwareSerial
SoftwareSerial debUART(5, 18);
HardwareSerial ardUART(2); // RX, TX pins for SoftwareSerial
int var1 = 0, var2 = 0, var3 = 0, var4 = 0, var5 = 0;
int dead = 0; int i = 0; int stat = 0;

static unsigned long lastToggleTime = 0;
static bool displayState = false;

void controlMotor(int speed, bool direction) {
  if (direction) {
    digitalWrite(MOTOR_L_EN, HIGH);
    digitalWrite(MOTOR_R_EN, LOW);
    ledcWrite(0, abs(speed)); // Set PWM duty cycle for L_PWM
    ledcWrite(1, 0);          // Disable R_PWM
  } else {
    digitalWrite(MOTOR_L_EN, LOW);
    digitalWrite(MOTOR_R_EN, HIGH);
    ledcWrite(1, abs(speed)); // Set PWM duty cycle for R_PWM
    ledcWrite(0, 0);          // Disable L_PWM
  }
}

void checkUART(void *parameter) {
  for (;;) {
    if (debUART.available()) {
      dead = 0;
      stat = 1;
      digitalWrite(2, HIGH);
      String receivedData = debUART.readStringUntil('\n'); // Read data until newline
      receivedData.trim();                                 // Remove any leading/trailing whitespace

      Serial.println(receivedData); // Print received data for debugging
      // Validate the format (e.g., "10 10 0 0")
      if (receivedData.length() > 0) {
        int checksum = 0, calculatedChecksum = 0;
        if (sscanf(receivedData.c_str(), "%d %d %d %d %d %d", &var1, &var2, &var3, &var4, &var5, &checksum) == 6) {
          calculatedChecksum = var1 + var2 + var3 + var4 + var5; // Calculate checksum
          if (calculatedChecksum == checksum) {
            // Data is valid
            /*
            Serial.print("Received: ");
            Serial.print(var1);
            Serial.print(" ");
            Serial.print(var2);
            Serial.print(" ");
            Serial.print(var3);
            Serial.print(" ");
            Serial.print(var4);
            Serial.print(" ");
            Serial.print(var5);
            Serial.print(" ");
            Serial.println(checksum);
            */
          } else {
            Serial.println("Checksum mismatch, data corrupt");
          }
        } else {
          Serial.println("Invalid data format");
        }
      }
      digitalWrite(2, LOW);
    } else {
      dead++;
      if (dead >= 100) {
        unsigned long currentTime = millis();
        if (currentTime - lastToggleTime >= 500) {
          lastToggleTime = currentTime;
          displayState = !displayState;
          if (displayState) {
            digitalWrite(2, HIGH);
            stat = 0;
            Serial.println("No UART Data");
          } else {
            digitalWrite(2, LOW);
            dead = 0;
          }
        }
      }
    }

    vTaskDelay(30 / portTICK_PERIOD_MS); // Reduce delay to ensure watchdog resets
    //yield(); // Explicitly yield to reset the watchdog
  }
}

void setup() {
  Serial.begin(9600);
  debUART.begin(15200);
  ardUART.begin(15200, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(2, OUTPUT);

  // Initialize IBT-2 Motor Driver Pins
  pinMode(MOTOR_L_EN, OUTPUT);
  pinMode(MOTOR_R_EN, OUTPUT);

  // Configure PWM channels
  ledcSetup(0, 5000, 8); // Channel 0, 5 kHz frequency, 8-bit resolution for L_PWM
  ledcSetup(1, 5000, 8); // Channel 1, 5 kHz frequency, 8-bit resolution for R_PWM
  ledcAttachPin(MOTOR_L_PWM, 0);
  ledcAttachPin(MOTOR_R_PWM, 1);

  xTaskCreate(
      checkUART,    // Function to implement the task
      "CheckUART",  // Name of the task
      10000,        // Stack size in words
      NULL,         // Task input parameter
      0,            // Priority of the task
      NULL);        // Task handle
}

void loop() {
  int motorSpeed = constrain(var4, -255, 255); // Map var1 to motor speed (-255 to 255)
  bool motorDirection = motorSpeed >= 0;      // Determine direction
  controlMotor(abs(motorSpeed), motorDirection);

  char buffer[50];
  snprintf(buffer, sizeof(buffer), "%d %d %d %d %d %d %d\n", var1, var2, var3, var4, var5, i, stat);
  ardUART.print(buffer); // Send data to SoftwareSerial
  //Serial.printf("Sending UART %d\n", i);

  if(i >= 50){
    i = 0;
  }
  i++;
  //ardUART.printf("%d %d %d %d %d\n", var1, var2, var3, var4, var5);
  delay(50);
}