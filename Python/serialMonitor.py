# serialMonitor.py - Gets data over serial port, sends commands to MCU, and plots data.
# David Patenaude and David Urrego - 7.15.2024
# Senior Design Summer '24 - Fall '24 Group 3

import serial   # import is from pySerial. make sure that is what is installed using pip.
import os
import time
import numpy as np
from matplotlib import pyplot as plt
from array import array
import sys
import threading
import queue


# Configure the serial port
serial_port :str = 'COM9'  # Change this to your serial port
baud_rate :int = 921600      # This can be increased by updating menuconfig UART setup for esp32 chip - limited by CP2102 chip (to 1Mbps)
# num_samples = 16*1024      # this should match the ADC_BUFFER_LEN from ESP32 code. Currently 8KB
chunk_size: int = 8*1024 * 7    # How much data Python will read at once as a chunk. 8k lines from 16KB MCU buffer, each line 7 bytes

# Tags/Flags to look for when receiving data from MCU
BEGIN_TAG = b"BEGIN PY READ"    # 'b' is to interpret as binary
END_TAG = b"END PY READ"        # if the end of a chunk needs to be labeled.
PY_END_TAG = b"END PY SAMPLE"
BEGIN_ADC_TAG = b"BEGIN ADC READ"   # for portability to accept both ADC and PCNT versions of the MCU code.
# PY_ADC_END_TAG = b"END ADC SAMPLE"
META_TAG = b"PY METADATA"       # Used to send meta data information

PY_TAG = "PY>\t"    # something to preface python prints to distinuish from MCU data in.

VMAX = 960  # mV // max when using 0 attenuation
DMAX = pow(2, 12) # 12-bit resolution
DEFAULT_DIR = "./pcnt_data/"

# The serial object for communicating with MCU
ser : serial.Serial = serial.Serial(serial_port, baud_rate, timeout=5)  # 5 second timeout on reads
ser.set_buffer_size(128*1024, None)  # set a buffer size to read from

# Parameters and debug variables
sample_freq :int = 0
sample_dur :int = 250
loopCompletions :int = 0
total_read :int = 0

input_queue = queue.Queue()  # Queue to store user commands
adc_plot_queue = queue.Queue()  # Queue for ADC data to be plotted
pcnt_plot_queue = queue.Queue()  # Queue for PCNT data to be plotted

def read_adc_data() -> dict[int, array]:
    global loopCompletions
    global total_read
    print("PY > Reading ADC Data\n")
    adc_raw :dict[int, array] = dict()  # stores channel as key, and list (or better, an array) as value (data) (or np array)
    for i in range(4):
        adc_raw[i] = array('f')
    # printf("Number of Bytes: %"PRIu32"\n", ret_num);    //pass over number of bytes/lines to read
    line = ser.readline()
    print("PY > " + str(line.strip(), "utf-8"))
    numLines = line.split(b':')[1].strip()    # gets the last index (should be same as 1) and removes whitespace
    # Read and Parse each line of data (2 bytes of data, so numBytes/2 gives number of elements to read)
    for i in range(int(int(numLines, base=10))):  # adjust when >1 channel of data is sent
        # ch: #; value: #(in hex)
        # (#),(0x#) // for speed, extra characters are eliminated
        #eventually some kind of time value will be included. Possibly the current clock cycle.
        # tic = time.perf_counter()
        line :str = ser.readline()
        if line.strip() == b"PY END":
            print(f"Read end tag: {str(line, encoding='utf-8')}")
            return adc_raw
        lineS = line.split(b',')
        # print(str(line, "utf-8"), end="")   # has newline in it already!
        # chNum = int(lineS[0].split(b':')[1].strip()) #assumes base 10
        try:
            chNum = int(lineS[0].strip(), base=10)
            # val_raw = int(lineS[1].split(b':')[1].strip(), base=16)  #value is hex.
            val_raw = int(lineS[1].strip(), base=16)
            # if chNum not in adc_raw:   #make array for dictionary if key hasn't been entered
            #     # first = False
            #     adc_raw[chNum] = array('f')    #'H' is for unsigned integer, which is 2 bytes (16bit). 'f'=float
            val_mV = val_raw * VMAX / DMAX      # Convert raw value to voltage
            adc_raw[chNum].append(val_mV)  #append works with array.
            total_read += 2
            
            # toc = time.perf_counter()
            # print(f"Time processing this line: {(toc-tic)*1000 : 3.3f} ms")
        except ValueError:
            print("<read_adc_data> Error reading a value from serial data in")  
            print(f"This line of data: \"{str(line, encoding="utf-8")}\"")
            print(f"Value of loop iterator {i=} of {numLines=}; {loopCompletions=}; {total_read=} B")
            continue
            exit(-1)
        except Exception as err:
            print(f"Unexpected Error: {err}\nType: {type(err)}")
    loopCompletions += 1
    return adc_raw

def read_adc_chunk():
    """ Reads `chunk_size` bytes from serial 1st and processes it so internal buffer doesn't fill up; prevents data buffer loss/corruption"""
    global total_read
    adc_raw :dict[int, array] = dict()
    
    print("PY > Reading in fixed 8192 bytes of data")
    # ESP will first print the number of bytes it is sending ...
    # printf("Number of Bytes: %"PRIu32"\n", ret_num);    //pass over number of bytes/lines to read
    line = str(ser.readline(), encoding="utf-8")
    print("PY >", line, end='')    # line will have newline in it   
    numBytes = line.split(':')[1].strip()    #gets the last index (should be same as 1) and removes whitespace    
    print(f"PY > numBytes sent from ESP32: {numBytes}")
    # BUG: num bytes is for the data portion, but what's sent over the channel is more than just data (whitespace, and comma). Raw write and read?
    # "[chNum],_[3 digit hex, leading 0's as necessary]\n" Or 7 bytes. 
    
    # read the number of bytes that corresponds to a 16kB chunk of ADC data. 
    chunk = str(ser.read(chunk_size), 'utf-8')  # read returns a byte array, convert to utf-8 encoded string
    # the problem is probably the newline characters. Determine how many bytes each line is to determine amount to read.  
    partitioned :list[str] = chunk.split("\n")   # don't know if serial will output \r with \n  (CRLF/Windows) or just \n (LF/Unix) [probably just LF/Unix]
    # read may cut a line off, and so the 1st split element might need to be removed/sacrificed.   
    for line in partitioned:    # partitioned will have all lines of data.    
        ls = line.split(',') # splits on separator 
        try:
            chNum = int(ls[0].strip(), base=10)
            raw = int(ls[1].strip(), base=16)
            if chNum not in adc_raw:
                adc_raw[chNum] = array('f')
            adc_raw[chNum].append(raw * VMAX / DMAX)
        except ValueError:
            print("Error converting input data into integer type")
            print(f"Data: {line=}")
            print(f"partitioned 1st 5: {partitioned[0:5]}")
        except Exception as err:
            print(f"Unexpected error: {type(err)}\n{err}")
            return
        # finally:
        #     pass
        total_read += 2
    return adc_raw

#pulse counter reader
def read_pcnt_data() -> tuple[array, dict[int, array]]:
    print(PY_TAG, "Reading PCNT Data\n")
    pcnt_data :dict[int, array] = dict()  # stores channel as key, and list (or better, an array) as value (data) (or np array)
    pcnt_data[0] = array('l')      # 4 byte/ 32-bit integer
    pcnt_data[1] = array('l')       # 'L' for unsigned, 'l' for signed
    pcnt_data[2] = array('l')
    pcnt_data[3] = array('l')

    time_data = array('Q')  # 64 bit unsigned integer
    # printf("Number of Bytes: %"PRIu32"\n", ret_num);    //pass over number of bytes/lines to read
    line = ser.readline()
    print(PY_TAG, str(line.strip(), "utf-8"))
    iters = line.split(b':')[1].strip()

    for j in range(int(iters, base=10)):
        # Ensure nothing prints in between! (suppress all the logs!)
        try:
            line = ser.readline()
            print(PY_TAG, str(line.strip(), "utf-8"))
            numLines = line.split(b':')[1].strip()    # gets the last index (should be same as 1) and removes whitespace
        except Exception as err:
            print(PY_TAG, "Error getting number of lines! Error: ", str(err))
            print(f'{str(line,"utf-8")=}')
        loops :int = 0
        # Read and Parse each line of data (2 bytes of data, so numBytes/2 gives number of elements to read)
        for i in range(int(int(numLines, base=10))):  # adjust when >1 channel of data is sent (there is no adjustment)
            # Each line has time, and 4 ints. Store counts for each channel (known by position). Store time in separate struct
            line = ser.readline()   #read line
            lineS = line.split(b',')    #split line
            try:
                # put data right into the storage structure
                time_data.append(int(lineS[0].strip()))    # base-10 default
                pcnt_data[0].append(int(lineS[1].strip()))
                pcnt_data[1].append(int(lineS[2].strip()))
                pcnt_data[2].append(int(lineS[3].strip()))
                pcnt_data[3].append(int(lineS[4].strip()))
                print(PY_TAG, str(line, "utf-8"), end='')   # print the line for debugging
            except:
                print(PY_TAG, 'Error in reading pcnt data')
                print(f'{loops=}, line read: \'{line}\'')
                return (time_data, pcnt_data)
                # return -1   # will this cause an error?
            
            loops += 1
    # print end of function and return tuple of data
    print(PY_TAG, "End of PCNT data read")
    return (time_data, pcnt_data)

def plot_pcnt_data(time_data: array, pcnt_data: dict[int, array], sample_frequency: int):
    """
    Plots pulse counts over time for four channels.

    Args:
        time_data (array): Array of time intervals in 0.1 ms.
        pcnt_data (dict[int, array]): Dictionary containing pulse counts for four channels.
        sample_frequency (int): Sample frequency in Hz.
    """
    import matplotlib.pyplot as plt
    import numpy as np

    # Convert time_data from microseconds to seconds
    time_in_seconds = np.array(time_data) * 1e-4

    # Calculate sample period
    sample_period = 1 / sample_frequency  # In seconds

    # Calculate pulse differences for each channel, including the first point
    channel_pulse_diffs = {}
    for ch in range(4):
        channel_data = np.array(pcnt_data[ch])
        pulse_diffs = np.zeros_like(channel_data)
        pulse_diffs[0] = channel_data[0]  # Keep the first data point as-is
        pulse_diffs[1:] = np.diff(channel_data)  # Compute differences from the second point onward
        channel_pulse_diffs[ch] = pulse_diffs

    # Plot data for each channel
    plt.ion()   # interactive mode
    plt.figure(figsize=(10, 6))
    for ch in range(4):
        plt.plot(
            time_in_seconds,  # Use all time points, including the first
            channel_pulse_diffs[ch],
            label=f"Channel {ch}",
            linewidth=1.5
        )

    plt.title(f"Pulse Counts Over Time (Sample Frequency: {sample_frequency} Hz)")
    plt.xlabel("Time (s)")
    plt.ylabel("Pulse Count")
    plt.legend()
    plt.grid(True)
    plt.show()

    plt.pause(0.001)    # give plot time to update
    plt.ioff            # disable interactive mode
    
def plot_adc_data(big_data: dict[int, array], sample_frequency: int):
    """Plots ADC data for all channels."""
    plt.figure(figsize=(12, 8))
    time_points = np.arange(0, len(next(iter(big_data.values())))) / sample_frequency

    for channel, data in big_data.items():
        plt.plot(time_points, data, label=f"Channel {channel}")

    plt.title("ADC Data")
    plt.xlabel("Time (s)")
    plt.ylabel("ADC Value (Voltage or Raw)")
    plt.legend()
    plt.grid()
    plt.show()

def plot_adc_data_thread():
    """Thread for plotting ADC data."""
    plt.ion()  # Enable interactive plotting
    fig, ax = plt.subplots(figsize=(10, 6))
    lines = {}  # Store line objects for each channel
    ax.set_title("ADC Data Over Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Voltage (mV)")
    ax.grid(True)

    while True:
        if not adc_plot_queue.empty():
            adc_data, sample_frequency = adc_plot_queue.get()

            # Prepare time axis
            num_samples = len(next(iter(adc_data.values())))
            time_axis = np.linspace(0, num_samples / sample_frequency, num_samples)

            # Plot data
            for ch, data in adc_data.items():
                if ch not in lines:
                    lines[ch], = ax.plot([], [], label=f"Channel {ch}")
                lines[ch].set_data(time_axis, data)

            # Update plot limits and redraw
            ax.set_xlim([0, time_axis[-1]])
            ax.set_ylim([0, VMAX])
            ax.legend()
            plt.draw()
            plt.pause(0.001)

def plot_pcnt_data_thread():
    """Thread for plotting PCNT data."""
    plt.ion()  # Enable interactive plotting
    fig, ax = plt.subplots(figsize=(10, 6))
    lines = {}  # Store line objects for each channel
    ax.set_title("Pulse Counts Over Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Pulse Count")
    ax.grid(True)

    while True:
        if not pcnt_plot_queue.empty():
            time_data, pcnt_data, sample_frequency = pcnt_plot_queue.get()

            time_in_seconds = np.array(time_data) * 1e-4  # Convert time data to seconds

            for ch, data in pcnt_data.items():
                if ch not in lines:
                    lines[ch], = ax.plot([], [], label=f"Channel {ch}")
                lines[ch].set_data(time_in_seconds, data)

            ax.set_xlim([0, time_in_seconds[-1]])
            ax.set_ylim([0, max(max(data) for data in pcnt_data.values())])
            ax.legend()
            plt.draw()
            plt.pause(0.001)

# New function to handle user input and display help
def send_user_commands():
    """Handles user input commands asynchronously."""
    while True:
        try:
            user_command = input("Py> Enter command ('help' for options): ").strip().lower()
        except EOFError:    # catch cntl+c / keybord interrupt
            print("Exiting program...")
            sys.exit(0);
        if user_command == "help":
            display_help()
        elif user_command == "exit":
            print("Exiting program...")
            sys.exit(0) # exits the python script entirely
            break
        else:
            input_queue.put(user_command)  # Adds command to the queue for the main thread to process

def display_help():
    print(10*"-" + "\nAvailable Commands:")
    print("help - Show available commands")
    print("start - Begin data acquisition")
    print("setf <frequency [in Hz]> - Sets sample frequency")
    print("setd <duration [in ms]> - Sets sample duration")
    print("set <frequency [in Hz]> <duration [in ms] - Sets sample frequency and duration at once")   
    print("pupil - Engages servos for pupil imaging") 
    # print("stop - Stop data acquisition")
    print("reset samples - Resets variables and data on MCU")
    print("suppress - Suppress logging to INFO level or lower")
    print("unsuppress - return logs back to DEBUG level")
    print("status - Display current system status")
    print("exit - Exit the program\n" + 10*"-")

def process_input():
    """Process user input commands."""
    while not input_queue.empty():
        command = input_queue.get()
        # handle arguement errors here, as opposed to on the MCU
        ser.write(command.encode('utf-8') + b'\n')  # Send command to the MCU
        print(f"Sent command: {command}")

timeoutCount = 1
adc_data :list[dict[int, array]] = list()   # holds dictionaries of the ADC data.   
# Prep folder for file saving.
if not os.path.isdir(DEFAULT_DIR):
    os.mkdir(DEFAULT_DIR)


user_command_thread = threading.Thread(target=send_user_commands)
user_command_thread.daemon = True
user_command_thread.start()

plot_adc_thread = threading.Thread(target=plot_adc_data_thread)
plot_adc_thread.daemon = True
# plot_adc_thread.start()

plot_pcnt_thread = threading.Thread(target=plot_pcnt_data_thread)
plot_pcnt_thread.daemon = True
# plot_pcnt_thread.start()

# main loop / thread
# wait until a tag appears (like 'py_adc_read') to appear and then start reading the data from adc.
while True:
    #Reads data as bytes. Will need to convert to char/string then.

    process_input()  # Handle queued commands from user

    dataIn = ser.readline()    #timeout set by serial object   
    if dataIn == b'':
        print(f'PY > Timed Out with no new data ({timeoutCount})')  #print to console if timed out
        timeoutCount +=1

    elif dataIn.strip() == META_TAG:
        # get metadata from serial/ESP32: printf("%s\nfreq: %"PRIu32"; duration: %"PRIu32"\n", PY_DATA, sample_frequency, sample_duration);   //in Hz and ms
        # line = dataIn.split("|")[1].split(";")
        dataIn = ser.readline()
        line = dataIn.strip().split(b';')
        sample_freq = int(line[0].split(b':')[1], base=10) # Hz 
        sample_dur = int(line[1].split(b':')[1], base=10)  # ms  
        print(f'PY > Recieved: {sample_freq=}; {sample_dur=}')

    elif dataIn.strip() == BEGIN_TAG:   #if input is the data we want, read it in.
        # adc_data_partial = read_adc_data()  #returns a dictionary of chNum->list of values  
        # #TODO: update names of variables! 
        pcnt_data = read_pcnt_data()  # read line by line, use delay on MCU time
        
        # Send the PCNT data to the plotting thread
        # pcnt_plot_queue.put((pcnt_data[0], pcnt_data[1], sample_freq))
        
        # export data to a file
        # save data for later use.
        curTime = time.localtime()
        # YYYYDDMM_hhmmss
        timeStr = str(curTime.tm_year) + str(curTime.tm_mday) + str(curTime.tm_mon)+'_'+ str(curTime.tm_hour)+str(curTime.tm_min)+str(curTime.tm_sec)
        for ch in pcnt_data[1]:
            fileName = DEFAULT_DIR + 'pcntCH' + str(ch) + '_' + timeStr + '.txt'
            with open(fileName, 'w') as fd:
                fd.write("Saved PCNT Data for CH" + str(ch) + " on " + timeStr + '\n')
                #additional meta data if need be. Use lines as separators.
                fd.write("PCNT Channel: " + str(ch) + "\n")
                fd.write("Time: " + timeStr + "\n")
                fd.write("Sample Frequency: " + str(sample_freq) + "\n")
                fd.write("Sample Duration: " + str(sample_dur) + "\n")
                # number of elements would help!    - if can't get right value from below, take it from ESP32 log. 
                fd.write("Time array Size: " + str(len(pcnt_data[0])) + '\n')
                
                fd.write(str(pcnt_data[0]))  #may or may not work
                fd.write('\n' + "Pulse count array size:" + str(len(pcnt_data[1][ch])) + '\n')
                fd.write(str(pcnt_data[1][ch]))
                # alternatively, array can be saved to a binary file with .tofile(fd), and then read using .fromfile(fd, # to read)
                print("PY > file created: ", fileName)
        # for now, plot data here
        plot_pcnt_data(pcnt_data[0], pcnt_data[1], sample_freq) # potentially put in separate thread so monitoring can continue

    elif dataIn.strip() == BEGIN_ADC_TAG:
        adc_data_partial = read_adc_data()  #returns a dictionary of chNum->list of values  
        loopCompletions += 1
        adc_data.append(adc_data_partial)
  
    # elif dataIn.strip() == PY_END_TAG:    #PCNT doesn't need an ending tag
    #     # export data to file
    #     print(PY_TAG, "Sample end not implemented")
    #     pass

    elif dataIn.strip() == PY_END_TAG:
        # export the list of dictionaries into one large dictionary (of chNum->array of data)
        # other way to store data: 2D array of chNum and time or cycles. 
        big_data = adc_data[0]  #should be a dictionary  
        for i in range(1, len(adc_data)):
            for chNum in big_data:
                big_data[chNum].extend(adc_data[i][chNum])  #extend appends all elements of iterable to the end of original array.

        # Send ADC data to the plotting thread
        # adc_plot_queue.put((big_data, sample_freq))
        
        # save data for later use.
        curTime = time.localtime()
        # YYYYDDMM_hhmmss
        timeStr = str(curTime.tm_year) + str(curTime.tm_mday) + str(curTime.tm_mon)+'_'+ str(curTime.tm_hour)+str(curTime.tm_min)+str(curTime.tm_sec)
        for ch in big_data:
            fileName = DEFAULT_DIR + 'adcCH' + str(ch) + '_' + timeStr + '.txt'
            with open(fileName, 'w') as fd:
                fd.write("Saved ADC Data for CH" + str(ch) + " on " + timeStr + '\n')
                #additional meta data if need be. Use lines as separators.
                fd.write("ADC Channel: " + str(ch) + "\n")
                fd.write("Time: " + timeStr + "\n")
                fd.write("Sample Frequency: " + str(sample_freq) + "\n")
                fd.write("Sample Duration: " + str(sample_dur) + "\n")
                # number of elements would help!    - if can't get right value from below, take it from ESP32 log. 
                fd.write("Array Size: " + str(len(big_data[ch])) + '\n')
                
                fd.write(str(big_data[ch]))  #may or may not work
                # alternatively, array can be saved to a binary file with .tofile(fd), and then read using .fromfile(fd, # to read)
                print("PY > file created: ", fileName)
        
        plot_adc_data(big_data, sample_freq)

        loopCompletions = 0 # reset loop counter 
        total_read = 0
    else:
        try:
            print(str(dataIn, encoding='utf-8'), end='')    # using UTF-8 encoding shows the colors of the output. newline in dataIn  
            timeoutCount = 1    #data was received, reset counter
        except UnicodeDecodeError:
            print(str(dataIn), end='')
            timeoutCount = 1    #data was received, reset counter