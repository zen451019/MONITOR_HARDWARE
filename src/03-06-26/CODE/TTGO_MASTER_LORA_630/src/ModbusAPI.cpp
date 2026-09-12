#include "ModbusAPI.h"
#include "ModbusClientRTU.h"

// --- Estructuras y variables internas (privadas a este fichero) ---

// Estructura para una solicitud interna, extendida para manejo síncrono
struct ApiRequest {
    uint8_t slave_id;
    uint8_t function_code;
    uint16_t start_address;
    uint16_t num_registers;
    uint32_t request_id;
    ModbusApiResult* result_ptr;     // Puntero para escribir el resultado
};

// Cliente Modbus y colas
static ModbusClientRTU MB;
static QueueHandle_t queueApiRequests;
static QueueHandle_t queueApiResponses;

// Contador de requests para IDs únicos y handle de la tarea que espera respuestas
static uint32_t     s_request_counter = 0;
static TaskHandle_t s_notify_task     = NULL;

// --- Callbacks de la librería Modbus ---

// Callback para respuestas de datos exitosas
static void handle_data_callback(ModbusMessage response, uint32_t token) {
    ModbusApiResult result;
    result.error_code = ModbusApiError::SUCCESS;
    result.slave_id = response.getServerID();

    // Copiamos solo los datos del payload (saltando la cabecera Modbus)
    // Para FC03/04, los datos empiezan en el índice 3.
    // Guardia contra underflow: respuesta malformada de tamaño < 3.
    size_t payload_len = 0;
    if (response.size() >= 3) {
        payload_len = response.size() - 3; // serverID(1) + FC(1) + byteCount(1)
    }
    result.data_len = std::min(payload_len, (size_t)MODBUS_API_MAX_DATA_SIZE);
    if (result.data_len > 0) {
        memcpy(result.data, response.data() + 3, result.data_len);
    }

    // token = request_id único, notificamos con ese valor
    if (s_notify_task != NULL) {
        xQueueSend(queueApiResponses, &result, 0);
        xTaskNotify(s_notify_task, token, eSetValueWithOverwrite);
    }
}

// Callback para errores
static void handle_error_callback(Error error, uint32_t token) {
    ModbusError me(error);
    ModbusApiResult result;
    result.data_len = 0;

    // Traducimos el error de la librería a nuestro tipo de error.
    // eModbus devuelve "Timeout" (T mayúscula, resto minúscula).
    // Usamos strstr ignorando mayúsculas con "imeout".
    const char* msg = (const char *)me;
    if (strstr(msg, "imeout") != nullptr || strstr(msg, "IMEOUT") != nullptr ||
        strstr(msg, "TIMEOUT") != nullptr) {
        result.error_code = ModbusApiError::ERROR_MODBUS_TIMEOUT;
    } else {
        result.error_code = ModbusApiError::ERROR_MODBUS_EXCEPTION;
        Serial.print("[D] Modbus error raw: ");
        Serial.println(msg);
    }

    // token = request_id único
    if (s_notify_task != NULL) {
        xQueueSend(queueApiResponses, &result, 0);
        xTaskNotify(s_notify_task, token, eSetValueWithOverwrite);
    }
}

// --- Tarea de gestión de solicitudes (Worker Task) ---

static void modbus_worker_task(void* pvParameters) {
    while (true) {
        ApiRequest request;
        // Espera a que llegue una nueva solicitud desde la función pública
        if (xQueueReceive(queueApiRequests, &request, portMAX_DELAY) == pdTRUE) {

            // El token es el request_id único. El callback lo usará para notificarnos.
            uint32_t token = request.request_id;

            Error err = MB.addRequest(token, request.slave_id, request.function_code,
                                     request.start_address, request.num_registers);

            if (err != Error::SUCCESS) {
                // Si la librería Modbus no pudo ni siquiera encolar la solicitud
                ModbusApiResult result;
                result.error_code = ModbusApiError::ERROR_QUEUE_FULL;
                result.data_len = 0;
                xQueueSend(queueApiResponses, &result, 0);
                xTaskNotify(s_notify_task, token, eSetValueWithOverwrite);
            }
        }
    }
}

// --- Implementación de las funciones públicas ---

void modbus_api_init(HardwareSerial& uart_port, int rx_pin, int tx_pin,
                     unsigned long baud_rate, uint32_t uart_config) {
    // Configurar UART
    RTUutils::prepareHardwareSerial(uart_port);
    uart_port.begin(baud_rate, uart_config, rx_pin, tx_pin);

    // Configurar cliente Modbus
    MB.onDataHandler(&handle_data_callback);
    MB.onErrorHandler(&handle_error_callback);
    MB.setTimeout(2000); // Timeout por defecto de la librería
    MB.begin(uart_port);

    // Crear colas
    queueApiRequests  = xQueueCreate(5, sizeof(ApiRequest));
    queueApiResponses = xQueueCreate(5, sizeof(ModbusApiResult));

    if (queueApiRequests == NULL || queueApiResponses == NULL) {
        Serial.println("[E] FATAL: No se pudieron crear las colas Modbus (memoria insuficiente)");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // Crear la tarea trabajadora
    xTaskCreate(modbus_worker_task, "ModbusWorker", 4096, NULL, 5, NULL);
}

ModbusApiResult modbus_api_read_registers(uint8_t slave_id, uint8_t function_code, uint16_t start_address, uint16_t num_registers, uint32_t timeout_ms) {
    ModbusApiResult result;

    // 1. Guardar el handle de la tarea llamadora (siempre mainPollingTask)
    s_notify_task = xTaskGetCurrentTaskHandle();

    // 2. ID único para este request
    uint32_t my_id = ++s_request_counter;

    // 3. Drenar notificaciones y respuestas residuales de ciclos anteriores.
    {
        uint32_t stale_val;
        while (xTaskNotifyWait(0, 0xFFFFFFFF, &stale_val, 0) == pdTRUE) { /* vaciar */ }
    }
    {
        ModbusApiResult stale;
        while (xQueueReceive(queueApiResponses, &stale, 0) == pdTRUE) { /* vaciar */ }
    }

    // 4. Preparar la solicitud para la tarea trabajadora.
    ApiRequest request = {
        .slave_id       = slave_id,
        .function_code  = function_code,
        .start_address  = start_address,
        .num_registers  = num_registers,
        .request_id     = my_id,
        .result_ptr     = &result
    };

    // 5. Enviar la solicitud a la cola de la tarea trabajadora.
    if (xQueueSend(queueApiRequests, &request, pdMS_TO_TICKS(100)) != pdTRUE) {
        result.error_code = ModbusApiError::ERROR_QUEUE_FULL;
        return result;
    }

    // 6. Esperar notificación con nuestro ID (o timeout).
    {
        uint32_t notified_id;
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notified_id, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
            if (notified_id == my_id) {
                // Nuestro callback — leer resultado de la cola.
                if (xQueueReceive(queueApiResponses, &result, pdMS_TO_TICKS(10)) != pdTRUE) {
                    result.error_code = ModbusApiError::ERROR_INTERNAL;
                    result.data_len = 0;
                }
            } else {
                // Otro request nos despertó (stale) — reintentar la espera.
                // Guardamos este resultado para no perderlo y seguimos esperando.
                ModbusApiResult deferred;
                bool has_deferred = false;
                if (xQueueReceive(queueApiResponses, &deferred, 0) == pdTRUE) {
                    has_deferred = true;
                }

                if (xTaskNotifyWait(0, 0xFFFFFFFF, &notified_id, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
                    if (notified_id == my_id) {
                        if (xQueueReceive(queueApiResponses, &result, pdMS_TO_TICKS(10)) != pdTRUE) {
                            result.error_code = ModbusApiError::ERROR_INTERNAL;
                            result.data_len = 0;
                        }
                    } else {
                        // Segunda notificación tampoco es nuestra → timeout.
                        result.error_code = ModbusApiError::ERROR_TIMEOUT;
                        result.data_len = 0;
                        // Drenar respuestas pendientes.
                        ModbusApiResult discarded;
                        while (xQueueReceive(queueApiResponses, &discarded, 0) == pdTRUE) { }
                    }
                } else {
                    // Timeout en la segunda espera.
                    result.error_code = ModbusApiError::ERROR_TIMEOUT;
                    result.data_len = 0;
                    // Drenar respuestas pendientes.
                    ModbusApiResult discarded;
                    while (xQueueReceive(queueApiResponses, &discarded, 0) == pdTRUE) { }
                }
            }
        } else {
            // Timeout del API.
            result.error_code = ModbusApiError::ERROR_TIMEOUT;
            result.data_len = 0;
            // Drenar respuestas tardías.
            ModbusApiResult discarded;
            while (xQueueReceive(queueApiResponses, &discarded, 0) == pdTRUE) { }
        }
    }

    return result;
}
