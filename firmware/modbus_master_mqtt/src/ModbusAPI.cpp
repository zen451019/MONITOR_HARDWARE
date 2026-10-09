#include "ModbusAPI.h"
#include "ModbusClientRTU.h"
#include "ModbusError.h"

#include <cstring>
#include <algorithm>

// Cliente Modbus (creado en init para permitir el pin DE/RE del transceptor).
static ModbusClientRTU* MB = nullptr;
static uint32_t s_token = 0;

// Traduce el error de eModbus a nuestro enum. Rellena exception_code si es una excepcion.
static ModbusApiError map_error(Error e, uint8_t& exception_code) {
    switch (e) {
        case SUCCESS:
            return ModbusApiError::SUCCESS;
        case TIMEOUT:
            return ModbusApiError::ERROR_TIMEOUT;
        case CRC_ERROR:
            return ModbusApiError::ERROR_CRC;
        case ILLEGAL_FUNCTION:
        case ILLEGAL_DATA_ADDRESS:
        case ILLEGAL_DATA_VALUE:
        case SERVER_DEVICE_FAILURE:
        case ACKNOWLEDGE:
        case SERVER_DEVICE_BUSY:
        case NEGATIVE_ACKNOWLEDGE:
        case MEMORY_PARITY_ERROR:
        case GATEWAY_PATH_UNAVAIL:
        case GATEWAY_TARGET_NO_RESP:
            exception_code = static_cast<uint8_t>(e);
            return ModbusApiError::ERROR_EXCEPTION;
        default:
            return ModbusApiError::ERROR_BUS;
    }
}

void modbus_api_init(HardwareSerial& uart_port, int rx_pin, int tx_pin,
                     unsigned long baud_rate, uint32_t uart_config,
                     uint32_t default_timeout_ms, int8_t de_re_pin) {
    RTUutils::prepareHardwareSerial(uart_port);
    uart_port.begin(baud_rate, uart_config, rx_pin, tx_pin);

    MB = new ModbusClientRTU(de_re_pin);
    MB->setTimeout(default_timeout_ms);
    MB->begin(uart_port);
}

ModbusApiResult modbus_api_read_registers(uint8_t slave_id, uint8_t function_code,
                                          uint16_t start_address, uint16_t num_registers,
                                          uint32_t timeout_ms) {
    ModbusApiResult result{};
    result.error_code    = ModbusApiError::ERROR_BUS;
    result.data_len      = 0;
    result.slave_id      = slave_id;
    result.exception_code = 0;

    if (MB == nullptr) {
        return result;
    }

    if (timeout_ms > 0) {
        MB->setTimeout(timeout_ms);
    }

    // Peticion sincrona: eModbus bloquea hasta respuesta o timeout.
    ModbusMessage resp = MB->syncRequest(++s_token, slave_id, function_code,
                                         start_address, num_registers);

    Error e = resp.getError();
    result.slave_id = resp.getServerID();

    if (e != SUCCESS) {
        result.error_code = map_error(e, result.exception_code);
        return result;
    }

    // Payload = serverID(1) + FC(1) + byteCount(1) + datos.
    size_t payload_len = (resp.size() >= 3) ? (resp.size() - 3) : 0;
    result.data_len = std::min(payload_len, (size_t)MODBUS_API_MAX_DATA_SIZE);
    if (result.data_len > 0) {
        memcpy(result.data, resp.data() + 3, result.data_len);
    }

    result.error_code = ModbusApiError::SUCCESS;
    return result;
}
