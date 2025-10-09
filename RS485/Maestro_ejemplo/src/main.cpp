/*
  =================================================================================================
  PROYECTO: Maestro Modbus RTU para solicitar datos periódicamente (ESP32)
  AUTOR: Zen451019 & Copilot
  FECHA: 2025-10-04
  
  DESCRIPCIÓN:
  - Actúa como un maestro Modbus.
  - Cada 6 segundos, solicita 15 registros (FC=03) al esclavo con ID=1.
  - Utiliza un enfoque asíncrono: añade peticiones a una cola y procesa las respuestas
    mediante una función de callback (handleData).
  - Imprime los datos recibidos en el monitor serie.
  =================================================================================================
*/

#include <Arduino.h>
#include "ModbusClientRTU.h"

// --- Configuración del Maestro ---
#define SLAVE_ID 1
#define SLAVE_ID_2 2
#define START_ADDRESS 0
#define NUM_REGISTERS 15
#define REQUEST_INTERVAL 6000 // Intervalo de petición en milisegundos (6 segundos)

// Pines para el conversor RS485 (RX, TX)
#define RX_PIN 12
#define TX_PIN 13

ModbusClientRTU MB;
unsigned long lastRequestTime = 0;
uint32_t requestToken = 0; // Token para identificar cada petición

// =======================================================================
// CALLBACK para manejar respuestas de datos exitosas
// =======================================================================
void handleData(ModbusMessage response, uint32_t token) {
  Serial.printf("\nRespuesta recibida para Token %u (Esclavo ID:%u, FC:%u)\n", token, response.getServerID(), response.getFunctionCode());
  Serial.print("  Datos (15 registros): [ ");
  
  // La respuesta contiene los datos como una secuencia de bytes.
  // Leemos cada registro (uint16_t) de la respuesta.
  for (uint8_t i = 0; i < NUM_REGISTERS; ++i) {
    uint16_t data;
    response.get(3 + i * 2, data); // Extraer un uint16_t de la posición correcta
    Serial.printf("%u", data);
    if (i < NUM_REGISTERS - 1) {
      Serial.print(", ");
    }
  }
  Serial.println(" ]");
}

// =======================================================================
// CALLBACK para manejar respuestas de error
// =======================================================================
void handleError(Error error, uint32_t token) {
  ModbusError me(error);
  Serial.printf("\nError en respuesta para Token %u: %02X - %s\n", token, (int)me, (const char *)me);
}

// =======================================================================
// SETUP
// =======================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\nIniciando Maestro Modbus...");

  // Iniciar puerto serie para Modbus RTU
  RTUutils::prepareHardwareSerial(Serial2);
  Serial2.begin(19200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Configurar el cliente Modbus
  MB.onDataHandler(&handleData);      // Asignar callback para datos
  MB.onErrorHandler(&handleError);    // Asignar callback para errores
  MB.setTimeout(2000);                // Timeout de 2 segundos por petición
  MB.begin(Serial2);                  // Iniciar cliente (la tarea de fondo se inicia aquí)

  Serial.println("Setup completado. Enviando peticiones cada 6 segundos...");
}

// =======================================================================
// LOOP
// =======================================================================
void loop() {
  if (millis() - lastRequestTime >= REQUEST_INTERVAL) {
    lastRequestTime = millis();

    // --- Petición para el Esclavo 1 ---
    Serial.printf("\nEnviando petición al Esclavo 1 (Token %u)...", requestToken);
    Error err = MB.addRequest(requestToken, SLAVE_ID, READ_HOLD_REGISTER, START_ADDRESS, NUM_REGISTERS);
    if (err != SUCCESS) {
      ModbusError e(err);
      Serial.printf("Error al crear petición 1: %02X - %s\n", (int)e, (const char *)e);
    }
    requestToken++; // Incrementar el token DESPUÉS de la primera petición

    // --- Petición para el Esclavo 2 ---
    Serial.printf("\nEnviando petición al Esclavo 2 (Token %u)...", requestToken);
    Error err_2 = MB.addRequest(requestToken, SLAVE_ID_2, READ_HOLD_REGISTER, START_ADDRESS, NUM_REGISTERS);
    if (err_2 != SUCCESS) {
      ModbusError e(err_2);
      Serial.printf("Error al crear petición 2: %02X - %s\n", (int)e, (const char *)e);
    }
    requestToken++; // Incrementar el token DESPUÉS de la segunda petición
  }
}