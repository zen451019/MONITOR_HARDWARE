#include "ModbusAPI.h"
#include "ModbusClientRTU.h"

// --- Estructuras y variables internas (privadas a este fichero) ---

// Estructura para una solicitud interna, extendida para manejo síncrono
struct ApiRequest {
    uint8_t slave_id;
    uint8_t function_code;
    uint16_t start_address;
    uint16_t num_registers;
    SemaphoreHandle_t completion_semaphore; // Semáforo para sincronizar la respuesta
    ModbusApiResult* result_ptr;            // Puntero para escribir el resultado
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
    size_t payload_len = response.size() - 3; // serverID(1) + FC(1) + byteCount(1)
    result.data_len = std::min(payload_len, (size_t)MODBUS_API_MAX_DATA_SIZE);
    memcpy(result.data, response.data() + 3, result.data_len);

    // El token es el puntero al semáforo
    SemaphoreHandle_t sem = (SemaphoreHandle_t)token;
    if (sem != NULL) {
        // Enviamos el resultado a la tarea que espera y la despertamos
        xQueueSend(queueApiResponses, &result, 0);
        xSemaphoreGive(sem);
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

    // El token es el puntero al semáforo
    SemaphoreHandle_t sem = (SemaphoreHandle_t)token;
    if (sem != NULL) {
        xQueueSend(queueApiResponses, &result, 0);
        xSemaphoreGive(sem);
    }
}

// --- Tarea de gestión de solicitudes (Worker Task) ---

static void modbus_worker_task(void* pvParameters) {
    while (true) {
        ApiRequest request;
        // Espera a que llegue una nueva solicitud desde la función pública
        if (xQueueReceive(queueApiRequests, &request, portMAX_DELAY) == pdTRUE) {
            
            // El "token" que usamos es el propio handle del semáforo.
            // Es un valor único para cada llamada y nos permite despertarla.
            uint32_t token = (uint32_t)request.completion_semaphore;

            Error err = MB.addRequest(token, request.slave_id, request.function_code, 
                                     request.start_address, request.num_registers);

            if (err != Error::SUCCESS) {
                // Si la librería Modbus no pudo ni siquiera encolar la solicitud
                ModbusApiResult result;
                result.error_code = ModbusApiError::ERROR_QUEUE_FULL;
                result.data_len = 0;
                xQueueSend(queueApiResponses, &result, 0);
                xSemaphoreGive(request.completion_semaphore);
            }
        }
    }
}

// --- Implementación de las funciones públicas ---

void modbus_api_init(HardwareSerial& uart_port, int rx_pin, int tx_pin) {
    // Configurar UART
    RTUutils::prepareHardwareSerial(uart_port);
    uart_port.begin(19200, SERIAL_8N1, rx_pin, tx_pin);

    // Configurar cliente Modbus
    MB.onDataHandler(&handle_data_callback);
    MB.onErrorHandler(&handle_error_callback);
    MB.setTimeout(2000); // Timeout por defecto de la librería
    MB.begin(uart_port);

    // Crear colas
    queueApiRequests = xQueueCreate(5, sizeof(ApiRequest));
    queueApiResponses = xQueueCreate(5, sizeof(ModbusApiResult));

    // Crear la tarea trabajadora
    xTaskCreate(modbus_worker_task, "ModbusWorker", 4096, NULL, 5, NULL);
}

ModbusApiResult modbus_api_read_registers(uint8_t slave_id, uint8_t function_code, uint16_t start_address, uint16_t num_registers, uint32_t timeout_ms) {
    ModbusApiResult result;

    // 1. Crear un semáforo para esta llamada específica.
    SemaphoreHandle_t completion_sem = xSemaphoreCreateBinary();
    if (completion_sem == NULL) {
        result.error_code = ModbusApiError::ERROR_INTERNAL;
        return result;
    }

    // 2. Preparar la solicitud para la tarea trabajadora.
    ApiRequest request = {
        .slave_id = slave_id,
        .function_code = function_code,
        .start_address = start_address,
        .num_registers = num_registers,
        .completion_semaphore = completion_sem,
        .result_ptr = &result // Pasamos la dirección para escribir el resultado
    };

    // 3. Enviar la solicitud a la cola de la tarea trabajadora.
    if (xQueueSend(queueApiRequests, &request, pdMS_TO_TICKS(100)) != pdTRUE) {
        vSemaphoreDelete(completion_sem);
        result.error_code = ModbusApiError::ERROR_QUEUE_FULL;
        return result;
    }

    // 4. Esperar a que el semáforo sea "liberado" por el callback (o que se agote el tiempo).
    if (xSemaphoreTake(completion_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        // El callback fue ejecutado. Recibimos el resultado de la cola.
        xQueueReceive(queueApiResponses, &result, 0);
    } else {
        // Se agotó el tiempo de espera de la API.
        result.error_code = ModbusApiError::ERROR_TIMEOUT;
        result.data_len = 0;
    }

    // 5. Limpiar y devolver.
    vSemaphoreDelete(completion_sem);
    return result;
}