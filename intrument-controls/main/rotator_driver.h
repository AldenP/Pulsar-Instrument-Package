/*
    A simple driver program for sending UART messages to a rotation mount from Thorlabs
    ESP32 UART API: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/uart.html
    Thorlabs rotator mount: https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=12829
*/

#ifndef ROTATOR_DRIVER_H
#define ROTATOR_DRIVER_H

#include "driver/uart.h"
#include "driver/gpio.h"

// Pins for the rotator mount
#define ROTATOR_TX      GPIO_NUM_21
#define ROTATOR_RX      GPIO_NUM_22
#define ROTATOR_BAUD    9600        // baud rate is fixed at 9600
#define ROTATOR_ADDR    '0'         // default address of devices

// UART port/controller number 
const uart_port_t rotator_uart_num = UART_NUM_1;

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

/**
 * Function to initialize UART port
 * Uses UART port defined here, as well as the pins and baud rate.
 * See ESP32 UART API for configuration functions
 */
void rotator_uart_init();

/**
 * Function to send a command to the rotator mount. Uses a 3-byte header: 1 byte address, 2 byte command, and optional data
 * @param address: Address of rotator mount device. Valid range of 0-F ASCII character for 0x0 to 0xF. 
 * @param cmd: Command to sent to device. Length of 2 bytes, 2 ASCII characters.
 * @param data: Optional set of ASCII characters representing hexadecimal numbers. 
 *              Example: "0A" which is 0x0A, or 10 in decimal
 * @param out: a buffer that recieves the reply from the rotator mount. 
 * @return Returns if cmd sent successfully
 */
int send_rotator_cmd(char address, char cmd[2], char data[], char* out);

/**
 * Function that decodes the response from the rotation mount. 
 * @param response: recieved data over uart from mount
 * @param length: length of the response, in bytes (characters)
 * @param out: pointer to output buffer to hold 
 */
int decode_rotator_response(char *response, size_t length, char *out);


#endif