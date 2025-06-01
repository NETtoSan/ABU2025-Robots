#include <Bluepad32.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
ControllerPtr myControllers;

SoftwareSerial wheelTX(16, 17);
SoftwareSerial topbodyTX(5, 18);

int X = 0, Y = 0, rot = 0, dpadState = 0, buttons = 0, pwm = 0, actuatorState = 0, espState = 1;
bool L1 = 0, R1 = 0, shoot = false;
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

void controlButtons() {
    count = (count + 1) % 100;

    const int deadzone = 30;
    X = (abs(X = map(X, -512, 512, -255, 255)) < deadzone) ? 0 : X;
    Y = (abs(Y = map(Y, -512, 512, -255, 255)) < deadzone) ? 0 : Y;
    rot = (abs(rot = map(rot, -512, 512, -255, 255)) < deadzone) ? 0 : rot;

}

void onConnectedController(ControllerPtr controller) {
    for( int i = 0; i < BP32_MAX_GAMEPADS; i++){
        if(myControllers == nullptr){
            Serial.printf("Connected!\n");
            joystickConnected = true;
            myControllers = controller;
            espState = 999;
            return;
        }
    }
    Serial.println("Controller connected, but no empty slot available.");
}

void onDisconnectedController(ControllerPtr controller) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers == controller) {
            Serial.printf("CALLBACK: Controller disconnected, index=%d\n", i);
            myControllers = nullptr;
            joystickConnected = false;
            espState = 1;
            return;
        }
    }
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");

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

     // Control PWM based on D-pad
    if (dpadState & 0x01) { // Button UP is pressed
        if (pwm < 255) {
            pwm += 5;
        }
        //Serial.printf("Button UP is pressed, PWM: %d\n", pwm);
    } else if (dpadState & 0x02) { // Button DOWN is pressed
        if (pwm > 0) {
            pwm -= 5;
        }
        //Serial.printf("Button DOWN is pressed, PWM: %d\n", pwm);
    }

    // Update R1 and L1 based on the buttons value
    if (buttons == 0x30) {
        R1 = true;
        L1 = true;
    } else if (buttons  == 0x20) {
        R1 = true;
        L1 = false;
    } else if (buttons == 0x10) {
        R1 = false;
        L1 = true;
    } else {
        R1 = false;
        L1 = false;
    }

    // Control solenoid state based on O button
    oButtonState = buttons & 0x02; // O button is pressed
    /*if (buttons & 0x02) { // Check if the O button is pressed
        static unsigned long CirclelastPressTime = 0;
        unsigned long currentCircleTime = millis();
        if (currentCircleTime - CirclelastPressTime > 200) { // Debounce delay (200ms)
          oButtonState = !oButtonState; // Toggle the state
          CirclelastPressTime = currentCircleTime;
        }
    }
    */


    // Control linear actuator from A and Y button
    if (buttons & 0x01){
        actuatorState = -1;
    }
    else if(buttons & 0x08){
        actuatorState = 1;
    }
    else{
        actuatorState = 0;
    }
}

void setup() {
    Serial.begin(9600);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);

    //BP32.forgetBluetoothKeys();
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

        //topbodyTX.printf("%d %d %d %d %d %d %d\n", oButtonState, L1, R1, pwm, 0, count, 1);
    
        //Serial.printf("Packet [%3d] X:%5d Y:%5d Rot:%5d DPad:0x%02x Btns:0x%02x L1:%d R1:%d PWM:%5d\n",
        //                            count, X, Y, rot, dpadState, buttons, L1, R1, pwm);

        sendMotorData(X, Y, rot, count, joystickConnected, L1, L1); // Send data to the wheel
       // Write UART to top body
                                                   //var1      , 2 , 3 , 4  , 5          , 6    , 7
        topbodyTX.printf("%d %d %d %d %d %d %d\n", oButtonState, L1, R1, pwm, actuatorState, count, espState);
        topbodyTX.flush();
        
        digitalWrite(2, LOW);
        delay(50);
        return;
    }
    else{
        static unsigned long lastBeepTime = 0;
        unsigned long currentTime = millis();

        if (currentTime - lastBeepTime >= 1000) { // Beep every 1 second
            BP32.update(); // Ensure the controller state is updated
            digitalWrite(2, HIGH);

            sendMotorData(0, 0, 0, 0, 0, 0, 0); // Send zero data to the wheel
            delay(500); // Beep for 500ms
            digitalWrite(2, LOW);
            lastBeepTime = currentTime;
        }
    }

}