#include <Arduino.h>

#define MOTOR_ENCODER_PIN_A 39
#define MOTOR_ENCODER_PIN_B 38

volatile long encoderCount = 0;

void IRAM_ATTR handleEncoderA() {
  int a = digitalRead(MOTOR_ENCODER_PIN_A);
  int b = digitalRead(MOTOR_ENCODER_PIN_B);

  if (a == b) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void IRAM_ATTR handleEncoderB() {
  int a = digitalRead(MOTOR_ENCODER_PIN_A);
  int b = digitalRead(MOTOR_ENCODER_PIN_B);

  if (a != b) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(MOTOR_ENCODER_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_PIN_A), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_PIN_B), handleEncoderB, CHANGE);
}

void loop() {
  static long lastCount = 0;

  if (lastCount != encoderCount) {
    Serial.println(encoderCount);
    lastCount = encoderCount;
  }
}