#include <Arduino.h>


void setup() {
  Serial.begin(115200);
  Serial.println("SPI OK");
}

void loop() {
  delay(1000);
  Serial.println("SPI OK");
}