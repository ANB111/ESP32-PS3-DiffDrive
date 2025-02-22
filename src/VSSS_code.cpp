#include <Arduino.h>
#include <Ps3Controller.h>
#include <freertos/task.h>
#include <myESP_bluetooth.h>
#include <MAC_table.h>
#include <cmath>
#include <cstdlib>
#include <stdint.h>

#define MAC MAC_ESP4

#define AIN1 18
#define AIN2 5 
#define PWMA 4 

#define BIN1 21
#define BIN2 22
#define PWMB 23


#define STBY 19
#define LED_PIN 2

/* Encoder optico-incremental A-B com efeito Hall */
#define ENC_MOTOR_LEFT_A 34  // Pino de encoder optico-incremental do motor esquerdo A
#define ENC_MOTOR_LEFT_B 35 // Pino de encoder optico-incremental do motor esquerdo B
#define ENC_MOTOR_RIGHT_A 12// Pino de encoder optico-incremental do motor direito A
#define ENC_MOTOR_RIGHT_B 13// Pino de encoder optico-incremental do motor direito B

#define PWMA_CHANNEL 0
#define PWMB_CHANNEL 1

int16_t leftWheel = 0;
int16_t rightWheel = 0;
int16_t acc = 0;
int16_t diff = 0;

volatile int leftPulses = 0;
volatile int rightPulses = 0;
unsigned long lastTime = 0;
float leftRPM = 0;
float rightRPM = 0;
int PULSES_PER_REV = 60;

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
        leftRPM = ((float)leftPulses / PULSES_PER_REV) * (60.0*1000/timeElapsed);
        rightRPM = ((float)rightPulses / PULSES_PER_REV) *(60.0*1000/timeElapsed);

        leftPulses = 0;
        rightPulses = 0;
        lastTime = currentTime;

        Serial.print("Left RPM: ");
        Serial.println(leftRPM);
        Serial.print("Right RPM: ");
        Serial.println(rightRPM);
    }
}
void moveEngine(bool forward, int speed, int num_motor)
{
    if (num_motor == 1)
    {
        if (forward)
        {
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
        }
        else
        {
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);
        }
        ledcWrite(PWMA_CHANNEL, speed); // Set motor speed
    }
    else if (num_motor == 2)
    {
        if (forward)
        {
            digitalWrite(BIN1, LOW);
            digitalWrite(BIN2, HIGH);
        }
        else
        {
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
        }
        ledcWrite(PWMB_CHANNEL, speed); // Set motor speed
    }
}

void dataReceiver()
{
    if (Ps3.isConnected())
    {
        ControllerData data = getControllerData();
        acc = data.ly;
        diff = data.rx;
    }
}

void findDifferentialGear()
{
    if (diff < 0)
    {
        if (acc != 0)
        {
            leftWheel = abs(acc) - (abs(diff) - (0.125 * abs(diff)));
            rightWheel = abs(acc);
        }
        else
        {
            leftWheel = rightWheel = 0;
        }
    }
    else if (diff > 0)
    {
        if (acc != 0)
        {
            rightWheel = abs(acc) - (abs(diff) - (0.125 * abs(diff)));
            leftWheel = abs(acc);
        }
        else
        {
            leftWheel = rightWheel = 0;
        }
    }
    else
    {
        leftWheel = rightWheel = abs(acc);
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println(Ps3.getAddress());
    delay(2000);
    // Inicializa o Bluetooth
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

void loop()
{
    dataReceiver();

    findDifferentialGear();

    if (acc < 0)
    {
        moveEngine(1, leftWheel * 2, 1);
        moveEngine(0, rightWheel * 2, 2);
    }
    else if (acc > 0)
    {
        moveEngine(0, leftWheel * 2, 1);
        moveEngine(1, rightWheel * 2, 2);
    }
    else
    {
        moveEngine(0, 0, 1);
        moveEngine(0, 0, 2);
    }
    calculateRPM();
}