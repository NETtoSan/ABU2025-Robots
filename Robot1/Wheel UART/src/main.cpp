#include <stdio.h>
#include <Arduino.h>
#include "motors.h"

HardwareSerial wheelRX(1); // Use UART1 for receiving (adjust pins as needed)
Motor motor1(15, 22, 4);
Motor motor2(12, 14, 27);
Motor motor3(26, 25, 33);
Motor motor4(5, 18, 19);


// Variables
int X, Y, rot, val1, val2;

// FreeRTOS task function
void receiveUARTTask(void *parameter) {
    uint32_t lastDataTime = millis(); // Track the last time data was received

    while (true) {
        try{
            if (wheelRX.available() >= 22) { // Wait for 1 header + 20 data bytes + 1 checksum
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

                // Validate checksum
                if (calculatedChecksum == receivedChecksum) {
                    // Interpret the data as integers
                    int *data = (int *)buffer;
                    X = data[0];
                    Y = data[1];
                    rot = data[2];
                    val1 = data[3];
                    val2 = data[4];

                    //Serial.printf("Received [%3d]: X:%3d, Y:%3d, rot:%3d, val1:%3d, val2:%3d\n", val1, X, Y, rot, val1, val2);
                    if(val2 == false){
                        Serial.printf("Joystick is not connected!\n");
                    }
                        lastDataTime = millis();
                    } else {
                        Serial.println("Checksum mismatch! Data corrupted.");
                    }
                }

                // Check if no data has been received for 500 ms
                else if (millis() - lastDataTime > 500) {
                    X = 0; Y = 0; rot = 0;
                    Serial.println("No UART data received for 500 ms.");
                    lastDataTime = millis(); // Reset the timer to avoid repeated messages
                }
            }
        }
        catch (const std::exception &e) {
            Serial.printf("Error reading UART data: %s\n", e.what());
        }

        vTaskDelay(50/ portTICK_PERIOD_MS); // Add a small delay to avoid busy looping
    }
}

void shout(){
    // Determine direction based on X and Y values
    Serial.printf("[%3d %3d %3d] ", X, Y, rot);
    if (Y > 0 && X == 0) {
        Serial.println("Moving Forward");
    } else if (Y < 0 && X == 0) {
        Serial.println("Moving Backward");
    } else if (X > 0 && Y == 0) {
        Serial.println("Moving Right");
    } else if (X < 0 && Y == 0) {
        Serial.println("Moving Left");
    } else if (X > 0 && Y > 0) {
        Serial.println("Moving Top Right");
    } else if (X < 0 && Y > 0) {
        Serial.println("Moving Top Left");
    } else if (X > 0 && Y < 0) {
        Serial.println("Moving Bottom Right");
    } else if (X < 0 && Y < 0) {
        Serial.println("Moving Bottom Left");
    } else if (X == 0 && Y == 0) {
        Serial.println("Stationary");
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

    delay(500);

    wheelRX.begin(15200, SERIAL_8N1, 16, 17); // Match baud rate and pins

    // Create the FreeRTOS task
    xTaskCreate(receiveUARTTask, "Receive UART", 2048, NULL, 1, NULL);
}

void loop() {
    // Empty loop since the task handles the logic

    //rot = map(rot, -255, 255, -128, 128);

    int motfl = -1 * ((X) - (-Y) - (-rot)); // Front Left
    int motfr = (X) + (-Y) + (rot); // Front Right
    int motrl = -1 * ((-X) - (-Y) + (rot)); // Rear Left
    int motrr = (-X) + (-Y) - (-rot); // Rear Right

    shout();

    try{
        // Suspected errors in motor control it may be the cause here.
        motor1.driveMotors(motfl);
        motor2.driveMotors(motfr);
        motor3.driveMotors(motrl);
        motor4.driveMotors(motrr);
    }
    catch (const std::exception &e){
        Serial.printf("Error in motor control: %s\n", e.what());
    }

    delay(25);
}