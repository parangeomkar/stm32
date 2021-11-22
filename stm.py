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
    sp.timeout = 1
    sp.bytesize=8
    sp.stopbits=1 
    sp.parity="N"
    sp.set_buffer_size(rx_size = 12800, tx_size = 12800)
    
    
    if sp.isOpen():
        sp.close()
        sp.open()
        statusText.set("COM port is already open!")
    else:
        sp.open()
        statusText.set("COM port is open!")
    clear()

def startClick():
    tx = [201,0,0,0,0,0]
    sp.write(tx)
    print(list(sp.read(1)))
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


def getData():
    global I_a, I_b, E_a, E_b,s1,s2,s3,s4
    I_af = 0
    I_bf = 0
    E_af = 0
    E_bf = 0
    x = 0
    a = 0
    
    pi = 3.14159265359
    angle_old = 0
    wt = 0
    V = 0.5*512
    Ts = 512
    
    while(x==0):
        rxData = list(sp.read(17))

        for i in range(9):
            if (rxData[i] == 255):
                I_a = (rxData[i+1] + 256*rxData[i+2])
                I_b = (rxData[i+3] + 256*rxData[i+4])
                E_a = (rxData[i+5] + 256*rxData[i+6]) - 1467
                E_b = (rxData[i+7] + 256*rxData[i+8]) - 1474
                break
        
        I_af = a*I_af + (1-a)*I_a
        I_bf = a*I_bf + (1-a)*I_b
        E_af = a*E_af + (1-a)*E_a
        E_bf = a*E_bf + (1-a)*E_b

         
        angle = math.atan2(E_af, E_bf)*57.29 + 180
        
        n = math.floor(wt/60) + 1

        T1 = math.floor(V*math.sin((n*60 - wt)*pi/180))
        T2 = math.floor(V*math.sin((wt - ((n-1)*60))*pi/180))
        T0 = Ts - (T1+T2) + 5

        if(wt >= 0 and wt < 60):
            Ta = T1 + T2 + (T0/2)
            Tb = T2 + (T0/2)
            Tc = (T0/2)
        elif(wt >= 60 and wt < 120):
            Ta = T1 + (T0/2)
            Tb = T1 + T2 + (T0/2)
            Tc = (T0/2)
        elif(wt >= 120 and wt < 180):
            Ta = (T0/2)
            Tb = T1 + T2 + (T0/2)
            Tc = T2 + (T0/2)
        elif(wt >= 180 and wt < 240):
            Ta = (T0/2)
            Tb = T1 + (T0/2)
            Tc = T1 + T2 + (T0/2)
        elif(wt >= 240 and wt < 300):
            Ta = T2 + (T0/2)
            Tb = (T0/2)
            Tc = T1 + T2 + (T0/2)
        elif(wt >= 300 and wt < 360):
            Ta = T1 + T2 + (T0/2)
            Tb = (T0/2)
            Tc = T1 + (T0/2)
        else:
            Ta = 0
            Tb = 0
            Tc = 0

        _ta0 = int(Ta) & 0xff
        _tb0 = int(Tb) & 0xff
        _tc0 = int(Tc) & 0xff

        _ta1 = (int(Ta)>>8) & 0xff
        _tb1 = (int(Tb)>>8) & 0xff
        _tc1 = (int(Tc)>>8) & 0xff

        tx = [_ta0,_ta1,_tb0,_tb1,_tc0,_tc1]
        sp.write(tx)

        wt += 1

        if wt == 360:
            wt = 0
        
        s1.append(E_af)
        s2.append(E_bf)
        s3.append(angle)
        
        if(len(s1) >= 5000):
            saveClick()
            s1 = []
            s2 = []
            s3 = []

        
        root.update()
    
def on_closing():
    plt.close()
    sp.close()
    
    
def plot():
    global y1,y2,y3,y4
    
    plt.cla()
    plt.plot(x,s1)
    plt.plot(x,s2)
    # plt.plot(x,s3)
    fig.canvas.draw()


def saveClick():
    global s1,s2,s3,s4
    sf1 = np.array(s1)
    sf2 = np.array(s2)
    sf3 = np.array(s3)
    sf4 = np.array(s4)
    print(len(sf1))
    print(len(sf2))
    df = pd.DataFrame({"I_a" : sf1, "I_b" : sf2, "E_a" : sf3})#, "E_b" : sf4})
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



