# plotData.py - gets data over serial port, plots data and stores it for later retrieval.
# David Patenaude - 7.15.2024
# Senior Design Summer '24 - Fall '24 Group 3

import serial
import os
import time
import numpy as np
from matplotlib import pyplot as plt
from array import array

# Configure the serial port
serial_port = 'COM3'  # Change this to your serial port
baud_rate = 1000000      #this can be increased by updating menuconfig UART setup for esp32 chip
num_samples = 8*1024      # this should match the ADC_BUFFER_LEN from ESP32 code. Currently 8KB

BEGIN_TAG = b"BEGIN PY READ"    # 'b' is to interpret as binary
PY_END_TAG = b"END PY SAMPLE"
VMAX = 960  #mV // max when using 0 attenuations
DMAX = pow(2, 12) #12 bit resolution
DEFAULT_DIR = "./adc_data/"

ser = serial.Serial(serial_port, baud_rate, timeout=5)  #5 second timeout on reads

def read_adc_data():
    print("PY > Reading ADC Data\n")
    adc_raw = dict()    #stores channel as key, and list (or better, an array) as value (data) (or np array)
    line = ser.readline()
    print("PY > " + str(line.strip(), "utf-8"))
    numBytes = line.split(b':')[1].strip()    #gets the last index (should be same as 1) and removes whitespace
    # Read and Parse each line of data
    # first = True
    for i in range( int(int(numBytes,base=10) /2)):   #adjust when >1 channel of data is sent
        # ch: #; value: #(in hex)
        # (#),(0x#) // for speed, extra characters are eliminated
        #eventually some kind of time value will be included. Possibly the current clock cycle. 
        line = ser.readline()
        lineS = line.split(b',')
        # print(str(line, "utf-8"))
        # chNum = int(lineS[0].split(b':')[1].strip()) #assumes base 10
        chNum = int(lineS[0].strip(), base=10)
        # val_raw = int(lineS[1].split(b':')[1].strip(), base=16)  #value is hex. 
        val_raw = int(lineS[1].strip(), base=16)
        if chNum not in adc_raw:   #make array for dictionary if key hasn't been entered
            # first = False
            adc_raw[chNum] = array('f')    #'H' is for unsigned integer, which is 2 bytes (16bit). 'f'=float
        val_mV = val_raw * VMAX / DMAX      # Convert raw value to voltage
        adc_raw[chNum].append(val_mV)  #append works with array. 
    
    return adc_raw

def calculate_pulse_timing(adc_data, sampling_rate):
    # Smooth the ADC data using Savitzky-Golay filter
    smoothed_data = savgol_filter(adc_data, window_length=51, polyorder=3)
    # Compute the first derivative
    derivative = np.diff(smoothed_data)
    # Identify the positions of maxima
    maxima = (np.diff(np.sign(np.diff(derivative))) < 0).nonzero()[0] + 1
    # Calculate pulse intervals
    pulse_intervals = np.diff(maxima) / sampling_rate
    return pulse_intervals

timeoutCount = 0
adc_data = list()   # holds dictionaries of the ADC data.
# Prep folder for file saving.
if not os.path.isdir(DEFAULT_DIR):
    os.mkdir(DEFAULT_DIR)

# wait until a tag appears (like 'py_adc_read') to appear and then start reading the data from adc.
while True:
    #Reads data as bytes. Will need to convert to char/string then.
    dataIn = ser.readline()    #timeout set by serial object
    if dataIn == b'':
        print('Py > Timed Out with no new data (%d)\n', timeoutCount)  #print to console if timed out
        timeoutCount +=1

    if dataIn.strip() == BEGIN_TAG:   #if input is the data we want, read it in.
        adc_data_partial = read_adc_data()  #returns a dictionary of chNum->list of values
        adc_data.append(adc_data_partial)
        # esp32 may need >1 loop to send data over. adc_data must be stored on the side until sample is done (wait for tag)
        # Process the ADC data here - process data later, after the full sample has been sent.
        # for chNum in adc_data:
        #     print('Channel #: ', chNum)
        #     x = np.arange(len(adc_data[chNum]))
        #     x = np.divide(x, 1000)  #at 1MHz - 1us per unit. 
        #     plt.plot(x, adc_data[chNum])
        #     plt.xlabel("Time (ms)")
        #     plt.ylabel("raw ADC")
        #     plt.show()
        #     print(adc_data[chNum])
            
        #     #Calculate pulse intervals
        #     sampling_rate = 1e6  # 1 MSPS
        #     pulse_intervals = calculate_pulse_timing(adc_data[chNum], sampling_rate)
        #     print("Pulse intervals (s):", pulse_intervals)
    
    if dataIn.strip() == PY_END_TAG:
        # export the list of dictionaries into one large dictionary (of chNum->array of data)
        # other way to store data: 2D array of chNum and time or cycles. 
        big_data = adc_data[0]  #should be a dictionary
        for i in range(1, len(adc_data)):
            for chNum in big_data:
                big_data[chNum].extend(adc_data[i][chNum])  #extend appends all elements of iterable to the end of original array.

        # save data for later use.
        curTime = time.localtime()
        # YYYYDDMM_hhmmss
        timeStr = str(curTime.tm_year) + str(curTime.tm_mday) + str(curTime.tm_mon)+'_'+ str(curTime.tm_hour)+str(curTime.tm_min)+str(curTime.tm_sec)
        for ch in big_data:
            fileName = DEFAULT_DIR + 'adcCH' + str(ch) + '_' + timeStr + '.txt'
            with open(fileName, 'w') as fd:
                fd.write("Saved ADC Data for CH" + str(ch) + " on " + timeStr + '\n')
                #additional meta data if need be.
                fd.write(str(big_data[chNum]))  #may or may not work
                print("file created: ", fileName)
    else:
        print(dataIn)
