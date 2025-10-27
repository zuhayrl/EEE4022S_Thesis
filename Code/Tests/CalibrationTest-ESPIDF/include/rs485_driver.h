/*  Header file for RS485 Driver
    Salinity Probe
    Zuhayr Loonat
    EEE4022S - 2025
*/

// ===== Includes =====
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <stdbool.h>

// ===== Defines =====
// Pinouts    
#define RS485_RO    39                  // RS485 RO Pin
#define RS485_RE_DE 40                  // RS485 RE/DE Pin
#define RS485_DI    41                  // RS485 RI Pin
// UART Config
#define RS485_UART_NUM      UART_NUM_1  // UART Number
#define RS485_BAUD_RATE     9600        // Adjust as needed
#define RS485_BUF_SIZE      1024        // RX/TX buffer size
#define RS485_QUEUE_SIZE    20          // Event queue size
// Timing (N.B: this is in uSeconds) needed since this is half-duplex
#define RS485_TX_DELAY_US   50          // Delay before transmit
#define RS485_RX_DELAY_US   100         // Delay after transmit before receive

// ===== Functions =====

/**
 * @brief Initialise RS485 communication using SP3485EN IC
 * @param baud_rate Baud rate for communication (default: 9600)
 * @return esp_err_t Error code
 */
esp_err_t rs485_init(uint32_t baud_rate);

/**
 * @brief Deinitialise RS485 and free resources
 * @return esp_err_t Error code
 */
esp_err_t rs485_deinit(void);

// Modes (Rx/Rx)
/**
 * @brief Set SP3485EN to transmit mode
 */
void rs485_set_transmit_mode(void);

/**
 * @brief Set SP3485EN to receive mode  
 */
void rs485_set_receive_mode(void);

/**
 * @brief Get current RS485 direction
 * @return true if in transmit mode, false if in receive mode
 */
bool rs485_is_transmit_mode(void);

// Send/Receive
/**
 * @brief Send data over RS485 bus
 * @param data Pointer to data buffer
 * @param length Number of bytes to send
 * @param timeout_ms Timeout in milliseconds
 * @return int Number of bytes sent, -1 on error
 */
int rs485_send_data(const uint8_t *data, size_t length, uint32_t timeout_ms);

/**
 * @brief Receive data from RS485 bus
 * @param buffer Pointer to receive buffer
 * @param buffer_size Size of receive buffer
 * @param timeout_ms Timeout in milliseconds
 * @return int Number of bytes received, -1 on error
 */
int rs485_receive_data(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms);

/**
 * @brief Send string over RS485 bus
 * @param str Null-terminated string to send
 * @param timeout_ms Timeout in milliseconds
 * @return int Number of bytes sent, -1 on error
 */
int rs485_send_string(const char *str, uint32_t timeout_ms);

/**
 * @brief Send command and wait for response
 * @param command Command string to send
 * @param response Buffer for response
 * @param response_size Size of response buffer
 * @param timeout_ms Timeout for response in milliseconds
 * @return int Number of bytes received, -1 on error
 */
int rs485_send_command(const char *command, char *response, size_t response_size, uint32_t timeout_ms);

// Helper functions
/**
 * @brief Check if data is available to read
 * @return size_t Number of bytes available
 */
size_t rs485_available(void);

/**
 * @brief Flush RS485 buffers (clear Rx data)
 */
void rs485_flush(void);