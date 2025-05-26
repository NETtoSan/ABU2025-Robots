#include <stdio.h>
#include <Arduino.h>
#include "motors.h"

HardwareSerial wheelRX(1); // Use UART1 for receiving (adjust pins as needed)
Motor motor1(15, 2, 4);
Motor motor2(12, 14, 27);
Motor motor3(26, 25, 33);
Motor motor4(5, 18, 19);


// Variables
int X, Y, rot, val1, val2;

// FreeRTOS task function
void receiveUARTTask(void *parameter) {
    unsigned long previousMillis = 0;
    unsigned long uartTimeoutMillis = 0;
    const unsigned long interval = 250; // Normal blinking interval
    const unsigned long rapidInterval = 100; // Rapid blinking interval
    static int previousPacketCount = 0;

    while (true) {
        unsigned long currentMillis = millis();

        if (wheelRX.available() >= 22) { // Wait for 1 header + 20 data bytes + 1 checksum
            digitalWrite(18, LOW);
            digitalWrite(25, HIGH);
            uint8_t header = wheelRX.read(); // Read the header
            if (header == 0xAA) { // Check if the header is correct
                uint8_t buffer[20];
                wheelRX.readBytes(buffer, sizeof(buffer)); // Read the 20 bytes of data
                uint8_t receivedChecksum = wheelRX.read(); // Read the checksum

                // Calculate checksum
                uint8_t calculatedChecksum = header;
                for (int i = 0; i < sizeof(buffer); i++) {
                    calculatedChecksum += buffer[i];
                }

                if (calculatedChecksum == receivedChecksum) { // Validate checksum
                    int *data = (int *)buffer;
                    X = data[0];
                    Y = data[1];
                    rot = data[2];
                    val1 = data[3];
                    val2 = data[4];

                    if (previousPacketCount != 0 && val1 != previousPacketCount + 1) {
                        Serial.printf("Packet skipped! Expected: %d, Received: %d\n", previousPacketCount + 1, val1);
                    }
                    previousPacketCount = val1;
                    uartTimeoutMillis = currentMillis; // Reset UART timeout
                    Serial.printf("Received [%5d]: %5d %5d %5d, %5d, %5d\n", val1, X, Y, rot, val1, val2);
                    digitalWrite(26, HIGH); // Indicate successful reception


                } else {
                    Serial.println("Checksum mismatch! Data corrupted.");
                    digitalWrite(26, LOW); // Indicate checksum error
                }
            }
        }

        if (wheelRX.available()) {
            uartTimeoutMillis = currentMillis; // Reset UART timeout
            if (currentMillis - previousMillis >= interval) {
                previousMillis = currentMillis;
                digitalWrite(12, !digitalRead(12)); // Toggle LED state
                if(val2 == 0){ digitalWrite(15, !digitalRead(12)); } else { digitalWrite(15, LOW);}
            }
        } else if (currentMillis - uartTimeoutMillis >= 500) {
            if (currentMillis - previousMillis >= rapidInterval) {
                previousMillis = currentMillis;
                digitalWrite(12, !digitalRead(12)); // Rapidly toggle LED state
                Serial.println("UART not available!");
                X = 0; Y = 0; rot = 0;
            } else {
                Serial.println("Waiting UART signal......");
            }
        }

        vTaskDelay(30 / portTICK_PERIOD_MS); // Add a small delay to avoid busy looping
        digitalWrite(18, HIGH);
        digitalWrite(25, LOW);
    }
}

void setup() {
    Serial.begin(9600); // For debugging

    motor1.initPins();
    motor1.setPwmFreq(15000, 1); // Set PWM frequency and channel

    motor2.initPins();
    motor2.setPwmFreq(15000, 2);

    motor3.initPins();
    motor3.setPwmFreq(15000, 3);

    motor4.initPins();
    motor4.setPwmFreq(15000, 4);

    for (int j = 0; j < 2; j++) {
        int delayTime = (j == 0) ? 50 : 200; // Fast in the first loop, slow in the second loop
        for (int i = 0; i < 2; i++) {
            motor1.allON();
            delay(delayTime);
            motor1.allOFF();

            motor2.allON();
            delay(delayTime);
            motor2.allOFF();

            motor3.allON();
            delay(delayTime);
            motor3.allOFF();

            motor4.allON();
            delay(delayTime);
            motor4.allOFF();
        }
    }

    delay(500);

    wheelRX.begin(15200, SERIAL_8N1, 16, 17); // Match baud rate and pins

    // Create the FreeRTOS task
    xTaskCreate(receiveUARTTask, "Receive UART", 2048, NULL, 1, NULL);
}

void loop() {
    // Empty loop since the task handles the logic

    int motfl = (-X) - (-Y) - (-rot); // Front Left
    int motfr = (-X) + (-Y) + (-rot); // Front Right
    int motrl = (-X) - (-Y) + (-rot); // Rear Left
    int motrr = (-X) + (-Y) - (-rot); // Rear Right

    //motor1.driveMotors(motfl);
    //motor2.driveMotors(motfr);
    //motor3.driveMotors(motrl);
    //motor4.driveMotors(motrr);

}