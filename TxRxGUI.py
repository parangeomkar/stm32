from tkinter import *
import serial
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
import os

clear = lambda: os.system('cls')

# Create Tkinter GUI root
root = Tk()
root.geometry("450x400")
root.title("Custom UART Data Acquisition Application")

# Create serial port instance
sp = serial.Serial()
statusText = StringVar()
startDataTransfer = 0
dataBuffer = 0

# This function opens the serial port
def connectClick():
    # get the global serial port instance
    global sp
    
    # set serial port configurations
    sp.baudrate = int(comBaud.get())
    sp.port = "COM"+comName.get()
    sp.timeout = 3
    sp.bytesize=8
    sp.stopbits=1 
    sp.parity="N"
    sp.set_buffer_size(rx_size = 17, tx_size = 6)
    
    # close and reopen the serial port if it is already in use
    if sp.isOpen():
        sp.close()
        sp.open()
        statusText.set("COM port is already open!")
    else:
        sp.open()
        statusText.set("COM port is open!")
    clear()

# This function starts data acquisition
def startClick():
    getData()

# This function stops the data acquisition    
def stopClick():
    global startDataTransfer
    startDataTransfer = 0
    root.update()

# This function implements logic to acquire the data
def getData():
    # close all the plots
    plt.close()
    
    s1 = []
    s2 = []
    s3 = []
    s4 = []
    s5 = []
    t = []
    tm = 0;
    
    # number of data points to acquire
    n = 10000
    
    print("Starting!")
     
    # some user defined sequece to notify the device that host is 
    # ready to receive the data
    tx = [101,0,0]
    sp.write(tx)
    
    # read "n" bytes from serial port
    rxData = list(sp.read(n))
    print(rxData)

    # some user defined sequece to notify the device that host has
    # received all the data
    tx = [102,0,0]
    sp.write(tx)
    
    
    i = 0
    dataOffset = 30000
    
    for _ in range(int(n/inc)):
        if(len(rxData) >= i+inc-1):
            a = (rxData[i] + 256*rxData[i+1] - dataOffset)
            b = (rxData[i+2] + 256*rxData[i+3] - dataOffset)
            c = (rxData[i+4] + 256*rxData[i+5] - dataOffset)
            d = (rxData[i+6] + 256*rxData[i+7] - dataOffset)
            e = (rxData[i+8] + 256*rxData[i+9] - dataOffset)
            
            i += inc
            tm += 1/(16.13*1000)
            
            s1.append(a)
            s2.append(b)
            s3.append(c)
            s4.append(d)
            s5.append(e)
            t.append(tm)
    
    # save the data in CSV file in same folder as this python script
    saveClick(s1,s2,s3,s4,s5)
    print("Done!")
        
    root.update()

# This function closes serial port on window close    
def on_closing():
    plt.close()
    sp.close()
    
# This function plots the variables in GUI plot    
def plot(s1,s2,s3,s4,s5,t):
    plt.cla()
    plt.plot(t,s1)
    plt.plot(t,s2)
    plt.plot(t,s3)
    fig.canvas.draw()

# This function saves the variables as columns in csv file
def saveClick(s1,s2,s3,s4,s5):
    df = pd.DataFrame({"th" : np.array(s1), "a" : np.array(s2), "b" : np.array(s3), "c" : np.array(s4), "d" : np.array(s5)})
    df.to_csv(csvFileName.get()+".csv", index=False, header=False)

# Handler for close button
def closeClick():
    exit()

# This function mimics ST-FOC start motor command from Motor Pilot
def motorStartClick():  
    # start the motor
    tx = [0x29,0x00,0x00,0xE0,0x19,0x00]
    sp.write(tx)   

    # read the acknowledgement
    sp.read(5)
    root.update()

# This function sets the speed of motor in RPM
def motorStartClick(speedInRPM):
    # set the speed to 1000 RPM
    Lbyte = int(hex(speedInRPM & 0xff),16)
    Hbyte = int(hex((speedInRPM >> 8) & 0xff),16)
    
    tx = [0xC9,0x00,0x00,0xC0,0x08,0x00,0xA9,0x01,0x06,0x00,Lbyte,Hbyte,0x00,0x00,0x64,0x00]
    sp.write(tx)
    print("Speed set to "+speedInRPM+" rpm")
    
    # read the acknowledgement
    sp.read(5)
    
    
# This function mimics ST-FOC stop motor command from Motor Pilot  
def motorHaultClick():
    tx = [0x29,0x00,0x00,0xE0,0x21,0x00]
    sp.write(tx)
    root.update()    
      
  
# GUI Frame
frame = Frame(root, relief = 'sunken')
frame.pack(fill = 'both', expand = True,padx = 10, pady = 10)


# COM port
comPortLabel = Label(frame, text="COM Port")
comPortLabel.grid(row=0,column=0,sticky=W)

comName = Entry(frame,width=10)
comName.insert(0,6)
comName.grid(row=0,column=1, padx=5)

# Baud rate
baudLabel = Label(frame, text="Baud Rate")
baudLabel.grid(row=0,column=2,sticky=W)

comBaud = Entry(frame,width=10)
comBaud.insert(0, 1843200)
comBaud.grid(row=0,column=3, padx=5)

# Connect button
connectButton = Button(frame, text="Connect",padx=10,pady=1, command=connectClick, fg="#000",bg="#fff")
connectButton.grid(row=0,column=4, padx=5)


# CSV
csvLabel = Label(frame, text="CSV file name")
csvLabel.grid(row=1,column=0)

csvFileName = Entry(frame,width=10)
csvFileName.insert(0, "data")
csvFileName.grid(row=1,column=1)

csvExtLabel = Label(frame, text=".csv")
csvExtLabel.grid(row=1,column=2,sticky=W)

# Connect button
saveButton = Button(frame, text="Save",padx=10,pady=1, command=saveClick, fg="#000",bg="#fff")
saveButton.grid(row=1,column=3, padx=5)

# Total data points
dataPtsLabel = Label(frame, text="No. of points")
dataPtsLabel.grid(row=2,column=0,sticky=W)

dataPts = Entry(frame,width=10)
dataPts.insert(0, "2000")
dataPts.grid(row=2,column=1)

# Start button
startButton = Button(frame, text="Start",padx=10,pady=1, command=startClick, fg="#000",bg="#fff")
startButton.grid(row=2,column=2, pady=10)

# End button
endButton = Button(frame, text="End",padx=10,pady=1, command=stopClick, fg="#000",bg="#fff")
endButton.grid(row=2,column=3, pady=10)

# Status
statusLabel = Label(frame, textvariable=statusText)
statusLabel.grid(row=3,columnspan=5,sticky=E)

# Close button
closeButton = Button(frame, text="Close",padx=10,pady=1, command=closeClick, fg="#fff",bg="#f00")
closeButton.grid(row=2,column=4, pady=10)


# Run button
runButton = Button(frame, text="Run",padx=10,pady=1, command=motorStartClick, fg="#000",bg="#fff")
runButton.grid(row=1,column=4, pady=10)

# Stop button
stopButton = Button(frame, text="Stop",padx=10,pady=1, command=motorHaultClick, fg="#000",bg="#fff")
stopButton.grid(row=1,column=5, pady=10)

# Chart
fig = plt.figure(1)
canvas = FigureCanvasTkAgg(fig, master=root)
plot_widget = canvas.get_tk_widget()
plot_widget.pack()
plt.plot()
fig.canvas.draw()

frame.mainloop()



