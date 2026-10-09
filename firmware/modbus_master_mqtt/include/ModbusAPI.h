#ifndef MODBUS_API_H
#define MODBUS_API_H

#include <Arduino.h>
#include <cstdint>

// Tamano maximo del payload de datos de una respuesta Modbus.
#define MODBUS_API_MAX_DATA_SIZE 256

/**
 * @brief Posibles errores que la API puede devolver.
 */
enum class ModbusApiError : uint8_t {
    SUCCESS = 0,          // Operacion exitosa.
    ERROR_TIMEOUT,        // No hubo respuesta a tiempo.
    ERROR_CRC,            // CRC invalido (solo RTU).
    ERROR_EXCEPTION,      // El esclavo respondio con una excepcion Modbus.
    ERROR_BUS,            // Otro error de comunicacion (server/FC mismatch, etc.).
};

/**
 * @brief Resultado de una operacion Modbus.
 */
struct ModbusApiResult {
    ModbusApiError error_code;                          // Codigo de error/éxito.
    uint8_t  data[MODBUS_API_MAX_DATA_SIZE];            // Bytes de payload (registros).
    size_t   data_len;                                  // Bytes validos en 'data'.
    uint8_t  slave_id;                                  // ID del esclavo que respondio.
    uint8_t  exception_code;                            // Codigo de excepcion Modbus (si aplica).
};

/**
 * @brief Inicializa la API Modbus (UART + cliente eModbus).
 * @param uart_port          UART a usar (ej. Serial2).
 * @param rx_pin / tx_pin    Pines RS485.
 * @param baud_rate          Velocidad.
 * @param uart_config        SERIAL_8N1, etc.
 * @param default_timeout_ms Timeout por defecto de la interfaz.
 * @param de_re_pin          Pin DE/RE del transceptor (-1 = auto-direccion).
 */
void modbus_api_init(HardwareSerial& uart_port, int rx_pin, int tx_pin,
                     unsigned long baud_rate, uint32_t uart_config,
                     uint32_t default_timeout_ms = 2000,
                     int8_t de_re_pin = -1);

/**
 * @brief Lectura sincrona de registros (bloqueada hasta respuesta o timeout).
 * @param timeout_ms 0 = usar el timeout por defecto de la interfaz.
 */
ModbusApiResult modbus_api_read_registers(uint8_t slave_id, uint8_t function_code,
                                          uint16_t start_address, uint16_t num_registers,
                                          uint32_t timeout_ms = 0);

#endif // MODBUS_API_H
