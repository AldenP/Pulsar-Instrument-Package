/*
    Implementation of driver header file for communicating with a rotation mount
*/
#include <string.h>
#include "waveplate_uart.h"  // Include the header file for macro and function declarations
#include "driver/uart.h"     // ESP32 UART driver for configuring UART communication
#include "driver/gpio.h"
// #include "FreeRTOSConfig.h"  // include whatever portTICK_PERIOD_MS needs

// // Define the TX and RX pins for the waveplate communication => moved to header file, so that main program can share these
// #define WAVEPLATE_TX        GPIO_NUM_21  // Pin 21 for transmitting data (TX)
// #define WAVEPLATE_RX        GPIO_NUM_22  // Pin 22 for receiving data (RX)
// #define UART_NUM            UART_NUM_1       // Use UART1 for communication with the waveplate
// #define WAVEPLATE_BAUD_RATE 9600  // Baud rate spec is 9600 for communication speed
// #define WAVEPLATE_ADDR      '0'     // Assume default address of device is '0' 

// Function to initialize the UART for waveplate communication
void init_waveplate_uart() {
    // UART configuration structure
    const uart_config_t uart_config = {
        .baud_rate = WAVEPLATE_BAUD_RATE,  // Baud rate for communication (115200 bps)
        .data_bits = UART_DATA_8_BITS,     // 8 data bits in each frame (standard configuration)
        .parity    = UART_PARITY_DISABLE,  // No parity bit, to simplify communication
        .stop_bits = UART_STOP_BITS_1,     // Use 1 stop bit to separate data frames
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // Flow control is disabled (simple communication)
        .source_clk = UART_SCLK_APB,       // Use the APB clock as the source clock for UART timing
    };

    // Install the UART driver with a RX buffer size of 1024*2 bytes
    // TX buffer of 0 means that any writes will block the task until finished (writes are small)
    uart_driver_install(UART_NUM, 1024 * 2, 0, 0, NULL, 0);

    // Configure UART parameters (baud rate, data bits, stop bits, etc.)
    uart_param_config(UART_NUM, &uart_config);

    // Set the TX and RX pins for UART communication
    uart_set_pin(UART_NUM, WAVEPLATE_TX, WAVEPLATE_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Function to send a command to the waveplate via UART
void send_waveplate_command(const char* command, uint8_t data) {
    // prepare command before being sent over UART
    char cmd_buf[64] = {0};
    sprintf(cmd_buf, "%c%2s%02x", WAVEPLATE_ADDR, command, data);
    // Write the command to the UART bus
    uart_write_bytes(UART_NUM, command, strlen(cmd_buf));
}

// Function to read the response from the waveplate
void read_waveplate_response() {
    uint8_t data[128];  // Buffer to store incoming data
    int length = uart_read_bytes(UART_NUM, data, sizeof(data), 100 / portTICK_PERIOD_MS);  // Read data from UART with a 100ms timeout
    
    // If data was received, print it to the console
    if (length > 0) {
        data[length] = '\0';  // Null-terminate the received data to treat it as a string
        printf("Waveplate response: %s\n", data);  // Print the response for debugging purposes
    }
}