#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// UART communication pins
#define RX_PIN 12
#define TX_PIN 13

// Motor driver pins
const int INA = 8;
const int INB = 10;
const int PWM = 11;

const int INA_2 = 6;
const int INB_2 = 9;
const int RELAY_PIN = 4;

// Linear actuator
int PWM_1 = 3;
int PWM_2 = 5;

// Current sense
int CS_1 = A0;

// Constants
const unsigned long DEBOUNCE_DELAY = 200;
const unsigned long RELAY_DEBOUNCE_DELAY = 100;
const unsigned long UART_TIMEOUT = 500;
const int COUNTER_MAX = 10;
bool stat2Ignore = false;


// LCD and UART
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial debUART(RX_PIN, TX_PIN);

// Variables
int var1 = 0, var2 = 0, var3 = 0, var4 = 0, var5 = 0, var6 = 0, var7 = 0;
int state = 0; // Motor state: 0 = off, 1 = forward, 2 = backward
int counter = 0;
int invalid = 0, dead = 0;

unsigned long lastPressTime = 0;
unsigned long lastToggleTime = 0;
bool displayState = false;

void showDisp(int x, int stat) {
  lcd.clear();
  if (stat == 0) {
    lcd.setCursor(0, 0);
    lcd.print("ESP no data");
    lcd.setCursor(0, 1);
    lcd.print("ESP32: 5 18");
  } else if(stat == 1){
    lcd.setCursor(0, 0);
    lcd.print("Controller offline");
    lcd.setCursor(0, 1);
    lcd.print("Turn controller on");
  } else if (stat == 2 && stat2Ignore == false) {
    static unsigned long startTime = 0;
    static bool timerStarted = false;

    if(!stat2Ignore){
      if (!timerStarted) {
        startTime = millis();
        timerStarted = true;
      }

      lcd.setCursor(0, 0);
      lcd.print("OpenCV offline");
      lcd.setCursor(0, 1);
      lcd.print("ESP32: 15 4");

      if (millis() - startTime >= 2000) {
        stat2Ignore = true;
        timerStarted = false; // Reset for next time stat==2
      }
    }
    
  }
  else{
    if (x == 0) {
      lcd.setCursor(0, 0);
      lcd.print(var6);
      lcd.setCursor(4, 0);
      lcd.print(var2);
      lcd.setCursor(8, 0);
      lcd.print(var3);
      lcd.setCursor(12, 0);
      lcd.print(var4);

      lcd.setCursor(0, 1);
      switch (state) {
        case 0: lcd.print("Off"); break;
        case 1: lcd.print("Sprint"); break;
        case 2: lcd.print("Suck"); break;
        case 3: lcd.print("Shoot"); break;
      }
    } else {
      lcd.setCursor(0, 0);
      lcd.print(counter);
      if (counter >= 5) {
        lcd.setCursor(4, 1);
        lcd.print("Low pressure");
      }
      lcd.setCursor(13, 0);
      lcd.print(var6);
    }
  }
}

void controlActuator(int var5){
  if (var5 == 0) {
    digitalWrite(PWM_1, LOW);
    digitalWrite(PWM_2, LOW);
  } else if(var5 == -1) {
    digitalWrite(PWM_1, LOW);
    digitalWrite(PWM_2, HIGH);
  } else if(var5 == 1){
    digitalWrite(PWM_1, HIGH);
    digitalWrite(PWM_2, LOW); 
  }
}

void controlShootingMotor() {
  switch (state) {
    case 0: // Off
      digitalWrite(INA, LOW);
      digitalWrite(INB, LOW);

      analogWrite(INA_2, 0);
      analogWrite(INB_2, 0);
      analogWrite(PWM, 0);
      break;

    case 1: // Forward; Sprint
      digitalWrite(INA, HIGH);
      digitalWrite(INB, LOW);

      analogWrite(INA_2, 255); // bottom motor
      analogWrite(INB_2, 0);

      analogWrite(PWM, 80); // top motor
      break;

    case 2: // Backward; Suck
      digitalWrite(INA, LOW);
      digitalWrite(INB, HIGH);

      analogWrite(INA_2, 0);
      analogWrite(INB_2, 120);

      analogWrite(PWM, 120);
      break;
    case 3: // Shoot
      digitalWrite(INA, HIGH);
      digitalWrite(INB, LOW);

      analogWrite(INA_2, var4);
      analogWrite(INB_2, 0);
      analogWrite(PWM, var4);
      break;
  }
}

void controlRelay() {
  static bool incremented = false;
  if (var1 == 1) {
    digitalWrite(RELAY_PIN, HIGH);
    if (!incremented) {
      counter++;
      incremented = true;
    }
  } else {
    digitalWrite(RELAY_PIN, LOW);
    incremented = false;
  }
}

void checkUART() {
  if (debUART.available()) {
    dead = 0;
    String receivedData = debUART.readStringUntil('\n');
    receivedData.trim();

    if (receivedData.length() > 0 && sscanf(receivedData.c_str(), "%d %d %d %d %d %d %d", &var1, &var2, &var3, &var4, &var5, &var6, &var7) == 7) {
      

      Serial.print("[");
      Serial.print(var6);
      Serial.print("]: ");
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
      Serial.print(var6);
      Serial.print(" ");
      Serial.println(var7);

      showDisp(var1, var7);
      controlActuator(var5);

    } else {
      lcd.setCursor(0, 0);
      lcd.print("Invalid data");
      lcd.setCursor(0, 1);
      lcd.print("ask net");
      Serial.println("Invalid data format");

      var1 = 0; var2 = 0; var3 = 0; var4 = 0; var5 = 0; var6 = 0; var7 = 0;
    }


    invalid = var6;
  } else {
    dead++;
    if (dead >= 50) {
      unsigned long currentTime = millis();
      if (currentTime - lastToggleTime >= UART_TIMEOUT) {
        lastToggleTime = currentTime;
        displayState = !displayState;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("No UART Data");
        if (!displayState) {
          lcd.setCursor(0, 1);
          lcd.print("RX: 12, TX: 13");
          tone(8, 1000, 200); // Beep on pin 8 with 1kHz frequency for 200ms
        }
      }
    }
  }
}
void setup() {
  lcd.init();
  lcd.backlight();
  lcd.print("Initializing...");
  Serial.begin(9600);
  debUART.begin(15200);

  pinMode(CS_1, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PWM, OUTPUT);

  pinMode(PWM_1, OUTPUT);
  pinMode(PWM_2, OUTPUT);

  digitalWrite(INA, LOW);
  digitalWrite(INB, LOW);
  analogWrite(PWM, 0);

  // Actuator self test
  //digitalWrite(PWM_1, HIGH);
  //digitalWrite(PWM_2, LOW);
  /*
  bool pwm1_OK = digitalRead(PWM_1);
  bool pwm2_OK = digitalRead(PWM_2);
  if (pwm1_OK && pwm2_OK) {
    lcd.setCursor(0, 0);
    lcd.print("Actuator OK");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Actuator Error");
  }
  */

  lcd.clear();
}

void loop() {
  checkUART();


  // Control shooting motor state
  // Var2 = L1 button, for toggling. Var3 = L2 button, for cycling through states
  unsigned long currentTime = millis();
  if (currentTime - lastPressTime > DEBOUNCE_DELAY) {
    if (var2 == 1) {
      lastPressTime = currentTime;
      state = (state == 0) ? 1 : 0;
    } else if (var3 == 1) {
      lastPressTime = currentTime;
      if (state == 1)
        state = 2;
      else if (state == 2)
        state = 3;
      else if (state == 3)
        state = 1;
      // If state is not 1/2/3, do nothing
      else{
        state = 1;
      }
    }
  }

  
  controlShootingMotor();
  controlRelay();
  delay(30);
}