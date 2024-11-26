# Pulsar-Instrument-Package
Software Package for calculating the polarization and pulse timing of pulsars from data collected in the .FITS format.
Also includes the programming for the Embedded ESP32 chip.
Part of a Senior Design Project at UCF during the Summer 2024 and Fall 2024 semesters.

## List of Functionality
The following lists the basic functionality that the package provides:
- Takes extracted data and calculates the Pulse Timing of a Pulsar
- Takes extracted data and calculates the Polarization of a Pulsar

On the embedded side:
- Handle user input via physical buttons
- Adjust the sampling rate, and sample duration based on user input
- Count the number of pulses per sample period, for the duration and frequency set by user
- Output data collected via USB/UART to a connected PC

## Directory Information
On the top level, we have 3 folders which take us to the different segments
- Under `Website` are the HTML, CSS, and documents that went to make our website
- Under `Python` are the data input parser, and other data handling tasks
- Under `instrument-controls` is an ESP32 embedded application, written using the ESP-IDF framework

