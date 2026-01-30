#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial LoRa(2); // UART2: RX2 = GPIO16, TX2 = GPIO17

void setup() {
  Serial.begin(115200);          // Monitor serie
  LoRa.begin(9600, SERIAL_8N1, 16, 17); // UART2 para LoRa (RYLR993)

  Serial.println("Inicializando RYLR993...");
  delay(2000);

  // Ejemplo: verificar comunicación AT
  LoRa.println("AT");
}

void loop() {
  while (LoRa.available()) {
    Serial.write(LoRa.read()); // Imprime respuesta del módulo
  }

  while (Serial.available()) {
    LoRa.write(Serial.read()); // Permite enviar comandos desde el monitor serie
  }
}
