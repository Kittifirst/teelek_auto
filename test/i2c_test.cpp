#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// #define SDA_PIN 21
// #define SCL_PIN 22

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);   // สำหรับ ESP32 สามารถกำหนด SDA,SCL ได้ เช่น Wire.begin(21,22)
  Serial.begin(115200);

  Serial.println("\nI2C Scanner");
}

void loop() {
  byte error, address;
  int count = 0;

  Serial.println("Scanning...");

  for(address = 1; address < 127; address++ ) {

    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      count++;
    }

    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (count == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("Done\n");

  delay(5000);
}