#ifndef MODBUS_API_H
#define MODBUS_API_H

#include <Arduino.h>
#include <vector>
#include <cstdint>

// Define el tamaño máximo de datos que una respuesta puede contener.
#define MODBUS_API_MAX_DATA_SIZE 128

/**
 * @brief Enumeración de posibles errores que la API puede devolver.
 */
enum class ModbusApiError : uint8_t {
    SUCCESS = 0,            // La operación fue exitosa.
    ERROR_TIMEOUT,          // La solicitud no recibió respuesta a tiempo (timeout de la API).
    ERROR_MODBUS_TIMEOUT,   // La librería Modbus reportó un timeout.
    ERROR_MODBUS_EXCEPTION, // El esclavo respondió con una excepción Modbus.
    ERROR_QUEUE_FULL,       // La cola interna de solicitudes está llena.
    ERROR_INVALID_PARAMS,   // Los parámetros de la solicitud son inválidos.
    ERROR_NOT_FOUND,        // No se encontró el recurso solicitado (ej. sensor no descubierto).
    ERROR_INTERNAL          // Un error interno inesperado.
};

/**
 * @brief Estructura para devolver el resultado de una operación Modbus.
 */
struct ModbusApiResult {
    ModbusApiError error_code;                          // Código de error/éxito.
    uint8_t data[MODBUS_API_MAX_DATA_SIZE];             // Buffer con los datos de respuesta.
    size_t data_len;                                    // Número de bytes válidos en 'data'.
    uint8_t slave_id;                                   // ID del esclavo que respondió.
};

/**
 * @brief Inicializa la API Modbus y las tareas subyacentes.
 * @details Debe ser llamada una vez en el setup().
 * @param uart_port Referencia a la UART a usar (ej. Serial2).
 * @param rx_pin Pin RX para la comunicación RS485.
 * @param tx_pin Pin TX para la comunicación RS485.
 */
void modbus_api_init(HardwareSerial& uart_port, int rx_pin, int tx_pin);

/**
 * @brief Realiza una solicitud Modbus de lectura y espera la respuesta de forma síncrona.
 * @details Esta función se bloquea hasta que se recibe una respuesta o se agota el tiempo de espera.
 *
 * @param slave_id El ID del esclavo Modbus (1-247).
 * @param function_code El código de función Modbus (ej. READ_HOLD_REGISTER).
 * @param start_address La dirección del primer registro a leer.
 * @param num_registers La cantidad de registros a leer.
 * @param timeout_ms El tiempo máximo de espera en milisegundos para esta operación.
 *
 * @return ModbusApiResult Una estructura con el resultado de la operación.
 *         - Si es exitoso, `error_code` será SUCCESS y `data` contendrá los bytes de los registros.
 *         - Si falla, `error_code` indicará la causa del error.
 */
ModbusApiResult modbus_api_read_registers(uint8_t slave_id, uint8_t function_code, uint16_t start_address, uint16_t num_registers, uint32_t timeout_ms);

#endif // MODBUS_API_H