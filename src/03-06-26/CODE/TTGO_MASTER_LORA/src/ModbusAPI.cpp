#include "ModbusAPI.h"
#include "ModbusClientRTU.h"

// --- Estructuras y variables internas (privadas a este fichero) ---

// Estructura para una solicitud interna, extendida para manejo síncrono
struct ApiRequest {
    uint8_t slave_id;
    uint8_t function_code;
    uint16_t start_address;
    uint16_t num_registers;
    TaskHandle_t caller_task_handle; // Task a notificar cuando llegue la respuesta
    ModbusApiResult* result_ptr;     // Puntero para escribir el resultado
};

// Cliente Modbus y colas
static ModbusClientRTU MB;
static QueueHandle_t queueApiRequests;
static QueueHandle_t queueApiResponses;

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

    // El token es el handle de la tarea que espera la respuesta
    TaskHandle_t task_to_notify = (TaskHandle_t)token;
    if (task_to_notify != NULL) {
        xQueueSend(queueApiResponses, &result, 0);
        xTaskNotifyGive(task_to_notify);
    }
}

// Callback para errores
static void handle_error_callback(Error error, uint32_t token) {
    ModbusError me(error);
    ModbusApiResult result;
    result.data_len = 0;

    // Traducimos el error de la librería a nuestro tipo de error
    if (strstr((const char *)me, "TIMEOUT") != nullptr) {
        result.error_code = ModbusApiError::ERROR_MODBUS_TIMEOUT;
    } else {
        result.error_code = ModbusApiError::ERROR_MODBUS_EXCEPTION;
    }

    // El token es el handle de la tarea que espera la respuesta
    TaskHandle_t task_to_notify = (TaskHandle_t)token;
    if (task_to_notify != NULL) {
        xQueueSend(queueApiResponses, &result, 0);
        xTaskNotifyGive(task_to_notify);
    }
}

// --- Tarea de gestión de solicitudes (Worker Task) ---

static void modbus_worker_task(void* pvParameters) {
    while (true) {
        ApiRequest request;
        // Espera a que llegue una nueva solicitud desde la función pública
        if (xQueueReceive(queueApiRequests, &request, portMAX_DELAY) == pdTRUE) {

            // El "token" que usamos es el handle de la tarea que espera.
            // Es un valor único para cada llamada y nos permite despertarla.
            uint32_t token = (uint32_t)request.caller_task_handle;

            Error err = MB.addRequest(token, request.slave_id, request.function_code,
                                     request.start_address, request.num_registers);

            if (err != Error::SUCCESS) {
                // Si la librería Modbus no pudo ni siquiera encolar la solicitud
                ModbusApiResult result;
                result.error_code = ModbusApiError::ERROR_QUEUE_FULL;
                result.data_len = 0;
                xQueueSend(queueApiResponses, &result, 0);
                xTaskNotifyGive(request.caller_task_handle);
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

    // 1. Obtener el handle de la tarea llamadora (siempre mainPollingTask en esta arquitectura)
    TaskHandle_t caller = xTaskGetCurrentTaskHandle();

    // 2. Drenar cualquier notificación o respuesta residual de ciclos anteriores.
    ulTaskNotifyTake(pdTRUE, 0);
    {
        ModbusApiResult stale;
        while (xQueueReceive(queueApiResponses, &stale, 0) == pdTRUE) { /* vaciar */ }
    }

    // 3. Preparar la solicitud para la tarea trabajadora.
    ApiRequest request = {
        .slave_id            = slave_id,
        .function_code       = function_code,
        .start_address       = start_address,
        .num_registers       = num_registers,
        .caller_task_handle  = caller,
        .result_ptr          = &result
    };

    // 4. Enviar la solicitud a la cola de la tarea trabajadora.
    if (xQueueSend(queueApiRequests, &request, pdMS_TO_TICKS(100)) != pdTRUE) {
        result.error_code = ModbusApiError::ERROR_QUEUE_FULL;
        return result;
    }

    // 5. Esperar a que el callback nos notifique (o que se agote el timeout).
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms)) > 0) {
        // El callback fue ejecutado. Recibimos el resultado de la cola.
        if (xQueueReceive(queueApiResponses, &result, pdMS_TO_TICKS(10)) != pdTRUE) {
            // Esto sería muy raro: el callback notificó pero no hay nada en la cola.
            result.error_code = ModbusApiError::ERROR_INTERNAL;
            result.data_len = 0;
        }
    } else {
        // Se agotó el tiempo de espera de la API.
        result.error_code = ModbusApiError::ERROR_TIMEOUT;
        result.data_len = 0;

        // IMPORTANTE: Drenar respuestas tardías. Si el callback se dispara justo
        // después del timeout, pondrá el resultado en la cola y notificará — pero
        // la notificación será drenada en el próximo ciclo (paso 2).
        ModbusApiResult discarded_result;
        while (xQueueReceive(queueApiResponses, &discarded_result, 0) == pdTRUE) {
            // Drenar todas las respuestas tardías para evitar desincronización.
        }
    }

    // Sin vSemaphoreDelete: Task Notifications no necesitan alloc/dealloc,
    // eliminando la race condition del semáforo borrado con token vivo.
    return result;
}
