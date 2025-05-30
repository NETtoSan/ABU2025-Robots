#include <Bluepad32.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
ControllerPtr myControllers;

SoftwareSerial wheelTX(16, 17);
SoftwareSerial topbodyTX(5, 18);
SoftwareSerial openCVRX(15, 4);

Adafruit_MPU6050 mpu;

int X = 0, Y = 0, rot = 0, dpadState = 0, buttons = 0, actuatorState = 0, espState = 1, pwm = 0;
int shooterAngle = 0;
int maxAngle = 50;

float yaw = 0;

int basketballX = 0, basketballY = 0, padX = 0, padY = 0, angleTarget = 90;
// espState 0 = Offline; 1 = Controller offline; 2 = opencv_offline; 999 = OK

bool L1 = false, R1 = false, joystickConnected = false;
static bool oButtonState = false; // State of the O button, or B button
int count = 0;


// This callback gets called any time a new gamepad is connected.
void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers == nullptr) {
            Serial.printf("CALLBACK: Controller connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
            myControllers = ctl;
            joystickConnected = true;
            espState = 999;
            return;
        }
    }
    Serial.println("CALLBACK: Controller connected, but no empty slot available");
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers == ctl) {
            Serial.printf("CALLBACK: Controller disconnected, index=%d\n", i);
            myControllers = nullptr;
            joystickConnected = false;
            espState = 1;
            return;
        }
    }
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
}

void dumpGamepad(ControllerPtr ctl) {

    X = ctl->axisX();
    //Y = -1 * ctl->axisY();
    rot = ctl->axisRX(); // Save right X Axis as rot
    Y = map(ctl->throttle(), 0, 1023, 0, 512) - map(ctl->brake(), 0, 1023, 0, 512); // Use L2 and R2 for rotation control
    dpadState = ctl -> dpad();
    buttons = ctl->buttons();
}

void processGamepad(ControllerPtr ctl) {
    dumpGamepad(ctl);
    if (ctl->a()) {
        static int colorIdx = 0;
        switch (colorIdx % 3) {
            case 0: ctl->setColorLED(255, 0, 0); break; // Red
            case 1: ctl->setColorLED(0, 255, 0); break; // Green
            case 2: ctl->setColorLED(0, 0, 255); break; // Blue
        }
        colorIdx++;
    }

    if (ctl->b()) {
        static int led = 0;
        led++;
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) {
        ctl->playDualRumble(0, 250, 0x80, 0x40);
    }

    const int deadzone = 30;
    X = map(X, -512, 512, -127, 127);
    Y = map(Y, -512, 512, -127, 127);
    rot = map(rot, -512, 512, -127, 127);

    //X = (abs(X) < deadzone) ? 0 : X;
    Y = (abs(Y) < deadzone) ? 0 : Y;
    rot = (abs(rot) < deadzone) ? 0 : rot;
    

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
    } else if (buttons == 0x20) {
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

void sendMotorData(int X, int Y, int rot, int val1, int val2) {
    uint8_t header = 0xAA; // Start-of-frame marker
    int values[5] = {X, Y, rot, val1, val2}; // Array of integers to send

    // Calculate checksum (simple sum of all bytes)
    uint8_t checksum = header;
    uint8_t* bytePtr =  (uint8_t*)values; // Treat the array as a byte array
    for (size_t i = 0; i < sizeof(values); i++) {
        checksum += bytePtr[i];
    }

    // Transmit the message
    wheelTX.write(header);                  // Send header
    wheelTX.write((uint8_t*)values, sizeof(values)); // Send the 5 integers (20 bytes)
    wheelTX.write(checksum);                // Send checksum

    wheelTX.flush();
} 

void receiveOpenCVData(void *parameter){
    unsigned long lastReceivedTime = millis();
    unsigned long notAvailableStart = 0;
    bool printedNotAvailable = false;

    while(true){
        if (openCVRX.available()) {
            char buffer[50];
            int bytesRead = openCVRX.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
            buffer[bytesRead] = '\0'; // Null-terminate the string


            if (sscanf(buffer, "%d %d %d %d %d", &basketballX, &basketballY, &padX, &padY, &angleTarget) == 4) {
                //Serial.printf("%d %d %d %d\n", basketballX, basketballY, padX, padY);
            }

            lastReceivedTime = millis();
            printedNotAvailable = false;
            if (espState == 2) espState = 999; // Reset state if previously set to 2
        } else {
            unsigned long now = millis();
            if (now - lastReceivedTime > 5000) {
                espState = 2;
                if (!printedNotAvailable) {
                    //Serial.println("openCV not available");
                    notAvailableStart = now;
                    printedNotAvailable = true;
                }
                // Print for 1 second in a non-blocking way
                if (printedNotAvailable && (now - notAvailableStart > 1000)) {
                    printedNotAvailable = false;
                }
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void readAngle(void *parameter){
    while(true){
        // Calculate pitch and yaw from accelerometer data
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        float pitch = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;
        yaw = atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;

        pitch = 90.0f + pitch ; // Adjust so pitch=0 becomes 90
        shooterAngle = static_cast<int>(pitch);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to avoid blocking
    }
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
    
    openCVRX.begin(15200);
    topbodyTX.begin(15200);
    wheelTX.begin(15200);

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
    }
    else{
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    }

    xTaskCreate(readAngle, "Read Angle", 8192, NULL, 1, NULL);
    xTaskCreate(receiveOpenCVData, "Receive OpenCV Data", 8192, NULL, 1, NULL);
}

void loop() {
    digitalWrite(2, HIGH);

    if (angleTarget < 0){
        angleTarget = -1 * angleTarget;
    }

    //Serial.printf("%d %.2f       |        %d %d %d %d | %d\n", shooterAngle, yaw, basketballX, basketballY, padX, padY, angleTarget);
    

    // Process controllers
    BP32.update();
    if(myControllers && myControllers -> isConnected()){
        processGamepad(myControllers);
    }
    // Count variable packets for sync
    if(count >= 100){
        count = 0;
    } else {
        count++;
    }   

    
    /* Control shooter actuator based on angleTarget
    if (shooterAngle != angleTarget) {
        if (shooterAngle < angleTarget) {
            actuatorState = -1; // Move actuator up
        } else if (shooterAngle > angleTarget) {
            actuatorState = 1; // Move actuator down
        }
    }
    else{
        actuatorState = 0; // Stop the actuator if angle matches target
    }
    */
   
    Serial.printf("Sending packet [%3d] %5d %5d %5d 0x%02x 0x%02x %d %d %5d\n",
         count, X, Y, rot, dpadState, buttons, L1, R1, pwm);


    sendMotorData(X, Y, rot, count, joystickConnected); // Send data to the wheel

    openCVRX.printf("%d\n", shooterAngle); // Send shooter angle back to OpenCV
    openCVRX.flush();
    // Write UART to top body
                                               //var1      , 2 , 3 , 4  , 5          , 6    , 7
    topbodyTX.printf("%d %d %d %d %d %d %d\n", oButtonState, L1, R1, pwm, actuatorState, count, espState);
    topbodyTX.flush();
    digitalWrite(2, LOW);
    delay(50);
}