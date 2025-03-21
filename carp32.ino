#include <Wire.h>
#include <LiquidCrystal.h>
#include "Bluepad32.h"

// LCD setup
LiquidCrystal lcd(5, 18, 19, 21, 22, 23);
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Battery voltage measurement
#define VOLTAGE_PIN 32
#define LED_INDICATOR 2

const float R1 = 20000.0;
const float R2 = 10000.0;
const float ADC_MAX = 4095.0;
const float REF_VOLTAGE = 3.3;
const float BATTERY_FULL = 7.4;
const float BATTERY_EMPTY = 6.0;

// Motor control pins
#define PWM1 12
#define MOTOR1_IN1 26
#define MOTOR1_IN2 25
#define MOTOR1_IN3 27
#define MOTOR1_IN4 14
#define PWM2 13
#define PWM_CHANNEL_1 0
#define PWM_CHANNEL_2 1

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            Serial.printf("Controller connected at index %d\n", i);
            digitalWrite(LED_INDICATOR, HIGH);
            return;
        }
    }
    Serial.println("Controller connected, but no empty slot found.");
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("Controller disconnected from index %d\n", i);
            myControllers[i] = nullptr;
            break;
        }
    }
    
    bool anyConnected = false;
    for (auto controller : myControllers) {
        if (controller) {
            anyConnected = true;
            break;
        }
    }

    if (!anyConnected) {
        digitalWrite(LED_INDICATOR, LOW);
    }
}

void setup() {
    Wire.begin();
    lcd.begin(16, 2);
    Serial.begin(115200);

    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR1_IN3, OUTPUT);
    pinMode(MOTOR1_IN4, OUTPUT);
    pinMode(LED_INDICATOR, OUTPUT);
    digitalWrite(LED_INDICATOR, LOW);

    ledcAttachPin(PWM1, PWM_CHANNEL_1);
    ledcSetup(PWM_CHANNEL_1, 5000, 8);
    ledcAttachPin(PWM2, PWM_CHANNEL_2);
    ledcSetup(PWM_CHANNEL_2, 5000, 8);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
}

float getBatteryVoltage() {
    int totalADC = 0;
    for (int i = 0; i < 10; i++) {
        totalADC += analogRead(VOLTAGE_PIN);
        delay(5);
    }
    float rawValue = totalADC / 10.0;
    float voltageMeasured = rawValue * (REF_VOLTAGE / ADC_MAX);
    return voltageMeasured * ((R1 + R2) / R2);
}

void displayWaiting() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WAITING FOR");
    lcd.setCursor(0, 1);
    lcd.print("CONTROLLER INPUT");
    delay(5000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Battery: ");
    lcd.print(getBatteryVoltage(), 2);
    lcd.print("V");
    lcd.setCursor(0, 1);
    lcd.print("Charge: ");
    lcd.print((int)(((getBatteryVoltage() - BATTERY_EMPTY) / (BATTERY_FULL - BATTERY_EMPTY)) * 100));
    lcd.print("%");
    delay(2000);
}

void stopMotors() {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR1_IN3, LOW);
    digitalWrite(MOTOR1_IN4, LOW);
    ledcWrite(PWM_CHANNEL_1, 0);
    ledcWrite(PWM_CHANNEL_2, 0);
}

void moveForward(int speed, int turn) {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR1_IN3, HIGH);
    digitalWrite(MOTOR1_IN4, LOW);
    ledcWrite(PWM_CHANNEL_1, constrain(speed - turn, 0, 255));
    ledcWrite(PWM_CHANNEL_2, constrain(speed + turn, 0, 255));
}

void moveBackward(int speed, int turn) {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, HIGH);
    digitalWrite(MOTOR1_IN3, LOW);
    digitalWrite(MOTOR1_IN4, HIGH);
    ledcWrite(PWM_CHANNEL_1, constrain(speed - turn, 0, 255));
    ledcWrite(PWM_CHANNEL_2, constrain(speed + turn, 0, 255));
}

// Drift Function
void drift() {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR1_IN3, LOW);
    digitalWrite(MOTOR1_IN4, HIGH);
    ledcWrite(PWM_CHANNEL_1, 255);
    ledcWrite(PWM_CHANNEL_2, 255);
    delay(500);  // Keep drifting for half a second
    stopMotors();
}

// Reverse-Forward Function
void reverseForwardSequence() {
    moveBackward(255, 0);
    delay(500); // Reverse for 1 second
    moveForward(255, 0);
    delay(500); // Forward for 1 second
    stopMotors();
}

void processControllers() {
    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData()) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Battery: ");
            lcd.print(getBatteryVoltage(), 2);
            lcd.print("V");
            lcd.setCursor(0, 1);
            lcd.print("Charge: ");
            lcd.print((int)(((getBatteryVoltage() - BATTERY_EMPTY) / (BATTERY_FULL - BATTERY_EMPTY)) * 100));
            lcd.print("%");

            int throttle = ctl->throttle();
            int brake = ctl->brake();
            int turnValue = map(ctl->axisX(), -511, 512, -255, 255);
            int mappedThrottle = map(throttle, 0, 1023, 0, 255);
            int mappedBrake = map(brake, 0, 1023, 0, 255);

            // Check for drift button (Square on PS / X on Xbox)
            if (ctl->buttons() & 0x0004) {  // Check if Square (X for Xbox) is pressed
             Serial.println("Drift Mode Activated!");
              drift();
            }



            // Check for Reverse-Forward button (Up D-pad)
            if (ctl->dpad()) {
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("WHEELIE :) :)");
                reverseForwardSequence();
            }

            if (throttle > 0) {
                moveForward(mappedThrottle, turnValue);
            } else if (brake > 0) {
                moveBackward(mappedBrake, turnValue);
            } else {
                stopMotors();
            }
        }
    }
}

void loop() {
    BP32.update();

    bool anyConnected = false;
    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected()) {
            anyConnected = true;
            break;
        }
    }

    if (!anyConnected) {
        displayWaiting();
        return;
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("YIPPEE CONTROLLER");
    lcd.setCursor(0, 1);
    lcd.print("FOUND :)");
    delay(2000);

    while (true) {
        BP32.update();
        processControllers();
        delay(100);
    }
}
