# Description of Primary Files
Under `main` folder is the code for our embedded application. 
`main_programV2.c` is the code that would read ADC data to sample the detectors. 
`pulse_counter_program.c` is the revised version for pulse counting.
`waveplate_uart.h` and `waveplate_uart.c are drivers for Thorlabs ELL14K rotation mount used in the project.

An LCD library was used for integrating a 20 character by 4 line I2C LCD display.

Downloading this folder should allow easy running of ESP-IDF functions to build, flash, and monitor an attached ESP32.

