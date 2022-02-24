from tkinter import *
import serial
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
import os
from multiprocessing import Process
import math
import threading

clear = lambda: os.system('cls')

root = Tk()
root.geometry("550x400")
root.title("Custom UART Data Acquisition Application")

sp = serial.Serial()
statusText = StringVar()
startDataTransfer = 0
dataBuffer = 0

def connectClick():
    global sp
    sp.baudrate = int(comBaud.get())
    sp.port = "COM"+comName.get()
    sp.timeout = 10
    sp.bytesize=8
    sp.stopbits=1 
    sp.parity="N"
    sp.set_buffer_size(rx_size = 17, tx_size = 6)
    
    
    if sp.isOpen():
        sp.close()
        sp.open()
        statusText.set("COM port is already open!")
    else:
        sp.open()
        statusText.set("COM port is open!")
    clear()

def startClick():
    getData()

    
def stopClick():
    global startDataTransfer
    startDataTransfer = 0
    root.update()


I_a = 0
I_b = 0
E_a = 0
E_b = 0
s1 = []
s2 = []
s3 = []
s4 = []
t = []


def getData():
    plt.close()
    s1 = []
    s2 = []
    s3 = []
    s4 = []
    s5 = []
    t = []
    tm = 0;
    n = 30000
    
    print("Starting!")
     
    tx = [104,0,0];
    sp.write(tx)
    time.sleep(2)
    
    tx = [101,0,0];
    sp.write(tx)
    
    rxData = list(sp.read(n))

    print(rxData)

    tx = [102,0,0];
    sp.write(tx)
    
    tx = [103,0,0];
    sp.write(tx)
    
    i = 0
    inc = 4
    theta_old = 0
    speed = 0
    told = 0
    x = 1
    pi = 3.14159265
    for _ in range(int(n/4)):
        if(len(rxData) >= i+inc-1):
            Iq = (rxData[i] + 256*rxData[i+1] - 10000)#*pi/180
            Id = (rxData[i+2] + 256*rxData[i+3] - 10000)
            # theta = (rxData[i+4] + 256*rxData[i+5] - 10000)
            # vec = rxData[i+6]
            
            i += inc
            tm += 1/(20*1000)
            
            if(Iq != told):
                if(Iq > told):
                    temp = 0.98*speed + 0.02*(Iq - told)/(x * 0.00005)*60/(4*2*pi)
                else:
                    temp = 0.98*speed + 0.02*(360 + (Iq - told))/(x * 0.00005)*60/(4*2*pi)
                
                if(temp < 5000):
                    speed = temp
                told = Iq
                x = 1
            else:
                x += 1
            
            s1.append(Id)
            s2.append(Iq)
            # s3.append(theta)
            # s4.append(vec)
            t.append(tm)

    plt.plot(t, s1)
    plt.plot(t, s2)
    # plt.plot(t, s3)
    # plt.plot(t, s4)
    # plt.plot(t, s5)
    plt.show()
    saveClick(s1,s2,s3,s4)
    print("Done!")
        
    root.update()
    
def on_closing():
    plt.close()
    sp.close()
    
    
def plot():
    global y1,y2,y3,y4
    print(len(t),len(s1))
    plt.cla()
    plt.plot(t,s1)
    plt.plot(t,s2)
    plt.plot(t,s3)
    fig.canvas.draw()


def saveClick(s1,s2,s3,s4):
    df = pd.DataFrame({"Id" : np.array(s1), "Iq" : np.array(s2), "Th" : np.array(s3), "wt" : s4})
    df.to_csv(csvFileName.get()+".csv", index=False, header=False)


def closeClick():
    exit()


def motorStartClick():
    tx = [0xC9,0x00,0x00,0xC0,0x08,0x00,0xA9,0x01,0x06,0x00,0xE8,0x03,0x00,0x00,0x64,0x00]
    sp.write(tx)
    print("Speed set to 1000 rpm")
    time.sleep(0.1)
    tx = [0x29,0x00,0x00,0xE0,0x19,0x00]
    sp.write(tx)    
    sp.read(5)
    root.update()
    getData()
  
def motorHaultClick():
    tx = [0x29,0x00,0x00,0xE0,0x21,0x00]
    sp.write(tx)
    root.update()    
  
  
def dummyFunction():
    print(sp.read(5))
    
  
# Frame
frame = Frame(root, relief = 'sunken')
frame.pack(fill = 'both', expand = True,padx = 10, pady = 10)


# COM port
comPortLabel = Label(frame, text="COM Port")
comPortLabel.grid(row=0,column=0,sticky=W)

comName = Entry(frame,width=10)
comName.insert(0, 4)
comName.grid(row=0,column=1, padx=5)

# Baud rate
baudLabel = Label(frame, text="Baud Rate")
baudLabel.grid(row=0,column=2,sticky=W)

comBaud = Entry(frame,width=10)
comBaud.insert(0, 2250000)
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

# Dummy button
dummyButton = Button(frame, text="Dummy Button :)",padx=10,pady=1, command=dummyFunction, fg="#000",bg="#fff")
dummyButton.grid(row=2,column=5, pady=10)

# Chart
fig = plt.figure(1)
canvas = FigureCanvasTkAgg(fig, master=root)
plot_widget = canvas.get_tk_widget()
plot_widget.pack()
plt.plot()
fig.canvas.draw()

frame.mainloop()



