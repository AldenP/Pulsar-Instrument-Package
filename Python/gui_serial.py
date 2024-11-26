# GUI Program for serial monitor.
# David Urrego
import tkinter as tk
from tkinter import scrolledtext
from threading import Thread
from serialMonitor import SerialMonitor  # Import the serialMonitor module

# Create the main GUI window
root = tk.Tk()
root.title("UART Communication GUI")
root.geometry("700x500")

# Serial Monitor instance
serial_monitor = SerialMonitor()

# Function to read data from the serial monitor and display it in the console
def read_serial_data():
    while True:
        data = serial_monitor.get_serial_data()
        if data:
            console.insert(tk.END, f"{data}\n")
            console.see(tk.END)  # Auto-scroll to the bottom

# Function to send data to the MCU
def send_command():
    print(f"in Function 'send_command'")
    command = input_box.get()
    if command:
        serial_monitor.send_serial_command(command)
        console.insert(tk.END, f"Command Sent: {command}\n")
        console.see(tk.END)
        input_box.delete(0, tk.END)

# Function to handle the "Start" button
def start_acquisition():
    print(f"in Function 'start_acquisition'")
    frequency = freq_entry.get()
    duration = dur_entry.get()
    if frequency > 0 and duration > 0:
        command = f"set {frequency} {duration}"
        serial_monitor.send_serial_command(command)
        serial_monitor.send_serial_command("start")
        console.insert(tk.END, f"Acquisition Started: Frequency={frequency}, Duration={duration}\n")
        console.see(tk.END)

# Console Window
console_label = tk.Label(root, text="Console Window")
console_label.pack()
console = scrolledtext.ScrolledText(root, width=70, height=15, state='disabled')
console.pack()


# User Command Input
input_frame = tk.Frame(root)
input_frame.pack(pady=5)
input_label = tk.Label(input_frame, text="User Command:")
input_label.pack(side=tk.LEFT)
input_box = tk.Entry(input_frame, width=40)
input_box.pack(side=tk.LEFT, padx=5)
send_button = tk.Button(input_frame, text="SEND", command=send_command)
send_button.pack(side=tk.LEFT)

# Control Hot Keys Section
control_frame = tk.Frame(root, borderwidth=2, relief="groove")
control_frame.pack(pady=10)
pupil_label = tk.Label(control_frame, text="Pupil Imager:")
pupil_label.grid(row=0, column=0, padx=5, pady=5)
pupil_on = tk.Radiobutton(control_frame, text="On", value=1)
pupil_on.grid(row=0, column=1)
pupil_off = tk.Radiobutton(control_frame, text="Off", value=0)
pupil_off.grid(row=0, column=2)

freq_label = tk.Label(control_frame, text="Frequency:")
freq_label.grid(row=1, column=0, padx=5, pady=5)
freq_entry = tk.Entry(control_frame, width=10)
freq_entry.grid(row=1, column=1, columnspan=2)

dur_label = tk.Label(control_frame, text="Duration:")
dur_label.grid(row=2, column=0, padx=5, pady=5)
dur_entry = tk.Entry(control_frame, width=10)
dur_entry.grid(row=2, column=1, columnspan=2)

start_button = tk.Button(control_frame, text="START", command=start_acquisition)
start_button.grid(row=3, column=0, columnspan=3, pady=5)

# Start a thread to read serial data
serial_thread = Thread(target=read_serial_data, daemon=True)
serial_thread.start()

# Run the GUI
root.mainloop()