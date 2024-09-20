# Additional Information
Below details how the data is processed to find desired quantities, like polarization.
## Polarization
Polarization is found by finding the Stokes' Parameter from data collected by a linear Stokes' polarimeter.

# Information on the files
The purpose of the files are described in this section

## plotData.py
This file monitors the serial port that the ESP32 board/chip is connected to. When certain phrases/tags are read, the program will read the ADC data into a data structure to be stored.  

### pythonEnvironment.txt
This file gives the output of `pip freeze`, allowing others to install the necessary packages for the python programs to work.