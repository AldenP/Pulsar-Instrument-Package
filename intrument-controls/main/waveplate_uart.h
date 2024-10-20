#ifndef WAVEPLATE_UART_H  // Header guard to prevent multiple inclusions of this file
#define WAVEPLATE_UART_H

// Define the TX and RX pins for the waveplate communication
#define WAVEPLATE_TX        GPIO_NUM_21  // Pin 21 for transmitting data (TX)
#define WAVEPLATE_RX        GPIO_NUM_22  // Pin 22 for receiving data (RX)
#define UART_NUM            UART_NUM_1       // Use UART1 for communication with the waveplate
#define WAVEPLATE_BAUD_RATE 9600  // Baud rate spec is 9600 for communication speed
#define WAVEPLATE_ADDR      '0'     // Assume default address of device is '0' 
// enum for possible error code responses from rotator
// Error is 2 ASCII character sequence, such as "00" or "09"
enum Rotator_Errors {
    OK = 0,
    CommTO = 1,
    MechTO = 2,
    CmdErr = 3,
    ValueOutOfRange = 4,
    ModIsolated = 5,
    ModOutIsolation = 6,
    InitErr = 7,
    ThermalErr = 8,
    Busy = 9,
    SensorErr = 10,
    MotorEr = 11,
    OutRange = 12,
    OverCur = 13
};

// Function declarations to initialize UART, send commands, and read waveplate responses
/**
 * Initializes the UART for waveplate communication.
 * This sets up the TX/RX pins, baud rate, and other UART parameters.
 */
void init_waveplate_uart();

/**
 * Sends a command to the waveplate via UART.
 * @param command: The command to be sent as a string (e.g., "ROTATE 90\r\n").
 */
void send_waveplate_command(const char* command, uint8_t data);

/**
 * Reads the response from the waveplate via UART.
 * The response is printed to the console for debugging or further processing.
 */
void read_waveplate_response();

#endif // End of header guard
