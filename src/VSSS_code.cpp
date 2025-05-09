#include <Arduino.h>
#include <Ps3Controller.h>
#include <freertos/task.h>
#include <myESP_bluetooth.h>
#include <MAC_table.h>
#include <cmath>
#include <cstdlib>
#include <stdint.h>

#define MAC MAC_ESP4

#define AIN1 26
#define AIN2 25 
#define PWMA 33

#define BIN1 14
#define BIN2 12
#define PWMB 32

#define STBY 27
#define LED_PIN 2

#define ENC_MOTOR_LEFT_A 39  // Encoder motor izquierdo A C2_M2
#define ENC_MOTOR_LEFT_B 34 // Encoder motor izquierdo B C1_M2
#define ENC_MOTOR_RIGHT_A 11// Encoder motor derecho A C2_M1
#define ENC_MOTOR_RIGHT_B 10// Encoder motor derecho B C1_M1

#define PWMA_CHANNEL 0
#define PWMB_CHANNEL 1

int16_t leftWheel = 0;
int16_t rightWheel = 0;
volatile int leftPulses = 0;
volatile int rightPulses = 0;
unsigned long lastTime = 0;
float leftRPM = 0;
float rightRPM = 0;
int PULSES_PER_REV = 60;
const int DEADZONE = 10;

void IRAM_ATTR countLeftPulses() {
    leftPulses += (digitalRead(ENC_MOTOR_LEFT_A) > digitalRead(ENC_MOTOR_LEFT_B)) ? 1 : -1;
}

void IRAM_ATTR countRightPulses() {
    rightPulses += (digitalRead(ENC_MOTOR_RIGHT_A) > digitalRead(ENC_MOTOR_RIGHT_B)) ? 1 : -1;
}

void calculateRPM() {
    unsigned long currentTime = millis();
    unsigned long timeElapsed = currentTime - lastTime;

    if (timeElapsed >= 1000) {
        leftRPM = ((float)leftPulses / PULSES_PER_REV) * (60.0 * 1000 / timeElapsed);
        rightRPM = ((float)rightPulses / PULSES_PER_REV) * (60.0 * 1000 / timeElapsed);

        leftPulses = 0;
        rightPulses = 0;
        lastTime = currentTime;

        Serial.print("Left RPM: "); Serial.println(leftRPM);
        Serial.print("Right RPM: "); Serial.println(rightRPM);
    }
}

void moveEngine(bool forward, int speed, int num_motor)
{
    speed = constrain(speed, 0, 255);

    if (num_motor == 1) { // Motor izquierdo
        digitalWrite(AIN1, !forward);
        digitalWrite(AIN2, forward);
        ledcWrite(PWMA_CHANNEL, speed);
    } 
    else if (num_motor == 2) { // Motor derecho (invertimos la lógica)
        digitalWrite(BIN1, forward);   // <- Cambiamos !forward por forward
        digitalWrite(BIN2, !forward);  // <- Cambiamos forward por !forward
        ledcWrite(PWMB_CHANNEL, speed);
    }
}

void dataReceiver() {
    if (Ps3.isConnected()) {
        ControllerData data = getControllerData();
        leftWheel = abs(data.ly) > DEADZONE ? data.ly : 0;
        rightWheel = abs(data.ry) > DEADZONE ? data.ry : 0;

        // Debug messages to print joystick data
        Serial.print("Joystick Left: "); Serial.println(leftWheel);
        Serial.print("Joystick Right: "); Serial.println(rightWheel);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println(Ps3.getAddress());
    delay(2000);
    controllerConfig(MAC);

    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(STBY, OUTPUT);

    digitalWrite(STBY, HIGH);

    ledcSetup(PWMA_CHANNEL, 5000, 8);
    ledcAttachPin(PWMA, PWMA_CHANNEL);
    ledcSetup(PWMB_CHANNEL, 5000, 8);
    ledcAttachPin(PWMB, PWMB_CHANNEL);

    pinMode(ENC_MOTOR_LEFT_A, INPUT);
    pinMode(ENC_MOTOR_RIGHT_A, INPUT);
    pinMode(ENC_MOTOR_LEFT_B, INPUT);
    pinMode(ENC_MOTOR_RIGHT_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENC_MOTOR_LEFT_A), countLeftPulses, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_MOTOR_RIGHT_A), countRightPulses, RISING);

    lastTime = millis();
}

void loop() {
    dataReceiver();

    if (leftWheel > 0)
        moveEngine(1, leftWheel * 2, 1);
    else if (leftWheel < 0)
        moveEngine(0, abs(leftWheel) * 2, 1);
    else
        moveEngine(0, 0, 1);

    if (rightWheel > 0)
        moveEngine(1, rightWheel * 2, 2);
    else if (rightWheel < 0)
        moveEngine(0, abs(rightWheel) * 2, 2);
    else
        moveEngine(0, 0, 2);

    calculateRPM();
}
