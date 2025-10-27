/*  Code file for RS485 Driver
    Salinity Probe
    Zuhayr Loonat
    EEE4022S - 2025
*/

// ===== Includes =====
#include "rs485_driver.h"
#include "driver/uart.h"                // UART/RS485 driver
#include "driver/gpio.h"                // GPIO input/output
#include "esp_rom_sys.h"
#include <stdio.h>
#include <string.h>

// ===== Global Vars =====
static QueueHandle_t rs485_queue;
static bool rs485_initialised = false;

// ===== Functions =====

/**
 * @brief initialise RS485 communication using SP3485EN IC
 * @param baud_rate Baud rate for communication (default: 9600)
 * @return esp_err_t Error code
 */
esp_err_t rs485_init(uint32_t baud_rate){
    
    // Check if already initialised
    if (rs485_initialised) {
        return ESP_OK;
    }

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(RS485_UART_NUM, RS485_BUF_SIZE, RS485_BUF_SIZE, RS485_QUEUE_SIZE, &rs485_queue, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(RS485_UART_NUM, &uart_config));

    // Set UART pins (TX, RX, RTS, CTS)
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART_NUM, RS485_DI, RS485_RO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Configure RE/DE pin as output (it's initially in receive mode)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << RS485_RE_DE),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Set to receive mode (RE/DE = LOW)
    rs485_set_receive_mode();
    rs485_initialised = true;
    printf("RS485 initialised at %d baud\n", baud_rate);
    return ESP_OK;
}

/**
 * @brief Deinitialise RS485 and free resources
 * @return esp_err_t Error code
 */
esp_err_t rs485_deinit(void){
    if (!rs485_initialised) {
        return ESP_OK;
    }
    
    esp_err_t ret = uart_driver_delete(RS485_UART_NUM);
    if (ret == ESP_OK) {
        rs485_initialised = false;
        rs485_queue = NULL;
        printf("RS485: Deinitialised\n");
    }
    
    return ret;
}

// Modes (Rx/Rx)
/**
 * @brief Set SP3485EN to transmit mode
 */
void rs485_set_transmit_mode(void){
    gpio_set_level(RS485_RE_DE, 1); // RE/DE = HIGH (transmit)
    esp_rom_delay_us(RS485_TX_DELAY_US); // Wait for transceiver switching
}

/**
 * @brief Set SP3485EN to receive mode  
 */
void rs485_set_receive_mode(void){
    gpio_set_level(RS485_RE_DE, 0); // RE/DE = LOW (receive)
    esp_rom_delay_us(RS485_RX_DELAY_US); // Wait for transceiver switching
}

/**
 * @brief Get current RS485 direction
 * @return true if in transmit mode, false if in receive mode
 */
bool rs485_is_transmit_mode(void){
    return gpio_get_level(RS485_RE_DE) == 1;
}

// Send/Receive
/**
 * @brief Send data over RS485 bus
 * @param data Pointer to data buffer
 * @param length Number of bytes to send
 * @param timeout_ms Timeout in milliseconds
 * @return int Number of bytes sent, -1 on error
 */
int rs485_send_data(const uint8_t *data, size_t length, uint32_t timeout_ms){

    // Check if initialised
    if (!rs485_initialised || data == NULL || length == 0) {
        return -1;
    }

    // Clear any pending RX data (flush UART)
    uart_flush_input(RS485_UART_NUM);

    // Switch to transmit mode
    rs485_set_transmit_mode();

    // Send data
    int bytes_sent = uart_write_bytes(RS485_UART_NUM, data, length);

    // Wait for transmission to complete
    uart_wait_tx_done(RS485_UART_NUM, pdMS_TO_TICKS(timeout_ms));

    // Switch back to receive mode
    rs485_set_receive_mode();

    return bytes_sent;
}

/**
 * @brief Receive data from RS485 bus
 * @param buffer Pointer to receive buffer
 * @param buffer_size Size of receive buffer
 * @param timeout_ms Timeout in milliseconds
 * @return int Number of bytes received, -1 on error
 */
int rs485_receive_data(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms){

    // Check if initialised
    if (!rs485_initialised || buffer == NULL || buffer_size == 0) {
        return -1;
    }

    // Ensure we're in receive mode
    if (rs485_is_transmit_mode()) {
        rs485_set_receive_mode();
    }

    // Read data with timeout
    int bytes_received = uart_read_bytes(RS485_UART_NUM, buffer, buffer_size - 1, pdMS_TO_TICKS(timeout_ms));

    if (bytes_received > 0) {
        buffer[bytes_received] = '\0'; // Null terminate for string operations
    }

    return bytes_received;
}

/**
 * @brief Send string over RS485 bus
 * @param str Null-terminated string to send
 * @param timeout_ms Timeout in milliseconds
 * @return int Number of bytes sent, -1 on error
 */
int rs485_send_string(const char *str, uint32_t timeout_ms){
    if (str == NULL) {
        return -1;
    }
    return rs485_send_data((const uint8_t *)str, strlen(str), timeout_ms);
}

/**
 * @brief Send command and wait for response
 * @param command Command string to send
 * @param response Buffer for response
 * @param response_size Size of response buffer
 * @param timeout_ms Timeout for response in milliseconds
 * @return int Number of bytes received, -1 on error
 */
int rs485_send_command(const char *command, char *response, size_t response_size, uint32_t timeout_ms){

    // Check if there is a command and place to store response
    if (command == NULL || response == NULL) {
        return -1;
    }

    // Send command
    int sent = rs485_send_string(command, 1000);
    if (sent <= 0) {
        return -1;
    }

    // Wait for response
    return rs485_receive_data((uint8_t *)response, response_size, timeout_ms);
}

// Helper functions
/**
 * @brief Check if data is available to read
 * @return size_t Number of bytes available
 */
size_t rs485_available(void){
    
    if (!rs485_initialised) {
        return 0;
    }

    size_t bytes_available;
    uart_get_buffered_data_len(RS485_UART_NUM, &bytes_available);
    return bytes_available;
}

/**
 * @brief Flush RS485 buffers (clear Rx data)
 */
void rs485_flush(void){

    if (rs485_initialised) {
        uart_flush(RS485_UART_NUM);
    }
}