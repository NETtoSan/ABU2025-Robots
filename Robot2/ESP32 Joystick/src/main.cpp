#include <Bluepad32.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
ControllerPtr myControllers;

SoftwareSerial wheelTX(5, 18);
SoftwareSerial topbodyTX(99, 99);

int X = 0, Y = 0, rot = 0, dpadState = 0, buttons = 0, pwm = 0;
bool L1 = 0, R1 = 0;
bool joystickConnected = false;
static bool oButtonState = false; // State of the O button, or B button
int count = 0;

void sendMotorData(int X, int Y, int rot, int val1, int val2, int var3, int var4) {
    
    int values[7] = {X, Y, rot, val1, val2, var3, var4}; // Array of integers

    uint8_t header = 0xAA; // Start-of-frame marker
    // Calculate checksum (simple sum of all bytes)
    uint8_t checksum = header;
    uint8_t* bytePtr = (uint8_t*)values; // Treat the array as a byte array
    for (size_t i = 0; i < sizeof(values); i++) {
        checksum += bytePtr[i];
    }
    
    // Transmit the message
    wheelTX.write(header);                  // Send header
    wheelTX.write((uint8_t*)values, sizeof(values)); // Send the 7 integers (14 bytes)
    wheelTX.write(checksum);                // Send checksum

    wheelTX.flush();
} 

void testVariables(){
    static unsigned long lastUpdateTime = 0;
    static int state = 0;

    unsigned long currentTime = millis();

    if (currentTime - lastUpdateTime >= 2000) { // 2 seconds interval
        lastUpdateTime = currentTime;

        switch (state) {
            case 0:
                X = 255;
                state = 1;
                break;
            case 1:
                X = -255;
                state = 2;
                break;
            case 2:
                Y = 255;
                state = 3;
                break;
            case 3:
                Y = -255;
                state = 0;
                break;
        }
    }
}

void controlButtons() {
  count = (count + 1) % 100;

  const int deadzone = 30;
  X = (abs(X = map(X, -512, 512, -255, 255)) < deadzone) ? 0 : X;
  Y = (abs(Y = map(Y, -512, 512, -255, 255)) < deadzone) ? 0 : Y;
  rot = (abs(rot = map(rot, -512, 512, -255, 255)) < deadzone) ? 0 : rot;

  if (dpadState & 0x01) { // Button UP is pressed
    pwm = min(pwm + 5, 255);
  } else if (dpadState & 0x02) { // Button DOWN is pressed
    pwm = max(pwm - 5, 0);
  }

  R1 = buttons & 0x20;
  L1 = buttons & 0x10;

  if (buttons & 0x02) { // Check if the O button is pressed
    static unsigned long lastPressTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastPressTime > 200) { // Debounce delay (200ms)
      oButtonState = !oButtonState; // Toggle the state
      lastPressTime = currentTime;
    }
  }
}

void onConnectedController(ControllerPtr controller) {
    Serial.printf("Connected\n");
    joystickConnected = true;
    myControllers = controller;
}
void onDisconnectedController(ControllerPtr controller) {
    Serial.printf("Disconnected\n");
    joystickConnected = false;
}
void ProcessGamepad(GamepadPtr myGamepad){
    X = myGamepad->axisX();
    Y = -1 * (myGamepad->axisY());
    rot = myGamepad->axisRX();
    buttons = myGamepad -> buttons();
    // Apply dead zone calibration
    if (X > -30 && X < 30) {
        X = 0;
    }
    if (Y > -30 && Y < 30) {
        Y = 0;
    }
    if (rot > -30 && rot < 30) {
        rot = 0;
    }
    //Serial.printf("X: %d Y: %d\n", X, Y);
}
void setup() {
    Serial.begin(9600);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

    pinMode(2, OUTPUT);
    topbodyTX.begin(15200);
    wheelTX.begin(15200);

    //xTaskCreate(updateController, "Update Controller", 8192, NULL, 1, NULL);
}

void loop() {
    BP32.update();

    if (myControllers && myControllers->isConnected()) {
        digitalWrite(2, HIGH);
        ProcessGamepad(myControllers);
        controlButtons();

        sendMotorData(X, Y, rot, count, joystickConnected, L1, R1); // Send data to the wheel

        //topbodyTX.printf("%d %d %d %d %d %d %d\n", oButtonState, L1, R1, pwm, 0, count, 1);
        delay(10);
    
        //Serial.printf("Packet [%3d] X:%5d Y:%5d Rot:%5d DPad:0x%02x Btns:0x%02x L1:%d R1:%d PWM:%5d\n",
        //                            count, X, Y, rot, dpadState, buttons, L1, R1, pwm);

        digitalWrite(2, LOW);
        delay(40);
        return;
    }
    static unsigned long lastBeepTime = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastBeepTime >= 1000) { // Beep every 1 second
        digitalWrite(2, HIGH);
        delay(500); // Beep for 500ms
        digitalWrite(2, LOW);
        lastBeepTime = currentTime;
    }

}