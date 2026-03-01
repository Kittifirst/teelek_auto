#include <Arduino.h>
#include <stdio.h>

#include <vector>
#include <cmath>
#include <utility>
#include <config.h>
#include <motor.h>
#include <PIDF.h>
#include <Utilize.h>

#include <esp32_Encoder.h> 

#define LIMIT_SWITCH_PIN 15 

void setup() {
  Serial.begin(115200);

  pinMode(LIMIT_SWITCH_PIN, INPUT);  // ปกติ HIGH, ชน = LOW
}
void loop() {
    Serial.println(digitalRead(LIMIT_SWITCH_PIN));
}