# Simple data processing to see what the data from the ADC looks like
# David Patenaude
# 9.21.2024

# import os
import array
import numpy as np
from matplotlib import pyplot as plt

sample_freq = 0
sample_dur = 0
sample_ch = -1
sample_time = ""

file_path = input("File of ADC data: ")

# open() raises OSError if a problem occurs
with open(file_path, mode="r", encoding="utf-8") as fd:
    # New format will have ch, time, freq, duration on separate lines after the 1st one.
    print(fd.readline())    # prints channel and time data is from.
    sample_ch = int(fd.readline().split(":")[-1].strip())
    sample_time = fd.readline().split(":")[-1].strip()
    sample_freq = int(fd.readline().split(":")[-1].strip())
    sample_dur = int(fd.readline().split(":")[-1].strip())
    sample_len = int(fd.readline().split(":")[-1].strip())
    print('ch', str(sample_ch), '; freq: ', str(sample_freq), ' ; sample_dur: ', str(sample_dur))
    # Now to get the giant array of data. 
    # arr = array.array('f', [1, 2, 3])
    # arr.fromfile
    line = fd.readline().split('[')[1]
    # arr_type = line[0:2]
    data = array.array('f')    # or hard code 'f'
    line = line.split(',')
    for ele in line:
        # first and last elements will have either '[' or '])' at beginning or end respectively. 
        # ele.removeprefix('[')
        ele = ele.removesuffix('])')
        # now parse the element as a float
        data.append(float(ele))
    # with luck, data has been successfully retrieved!
    time = (1.0/sample_freq) * np.arange(len(data))
    plt.plot(time, data)
    plt.xlabel('Time (s)')
    plt.ylabel('Conversion (mV)')
    plt.title('Plot of ADC Data for Ch' + str(sample_ch), loc='center')
    plt.show()

