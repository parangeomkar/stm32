from tkinter import *
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
from datetime import datetime
import pandas as pd
import numpy as np
import os

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
    sp.timeout = 0.1
    sp.bytesize=8
    sp.stopbits=1 
    sp.parity="N"

    if sp.isOpen():
        sp.close()
        sp.open()
        statusText.set("COM port is already open!")
    else:
        sp.open()
        statusText.set("COM port is open!")

    tx = [0x05,0xC7,0x01,0x14]
    sp.write(tx)
    print(sp.read(4))
    
    tx = [0x06,0x00,0x00,0x60]
    sp.write(tx)
    print(sp.read(4))
    
    tx = [0xA9,0x00,0x00,0x50,0x11,0x00,0x28,0x00,0x69,0x00,0xA9,0x00,0xE9,0x00]
    sp.write(tx)
    print(sp.read(109))
    
    tx = [0xD9,0x00,0x00,0x40,0x08,0x00,0x28,0x05,0x07,0x00,0x00,0x08,0x00,0x00,0xFF,0x00,0x00]
    sp.write(tx)
    print(sp.read(5))
    
    clear()

def startClick():
    global startDataTransfer
    startDataTransfer = 1
    getData()

    
def stopClick():
    global startDataTransfer
    startDataTransfer = 0
    root.update()

plotTime =[]
speed=[]
IA=[]
IB=[]
Ialpha=[]
Ibeta=[]
Valpha=[]
Vbeta=[]
BEMFalpha=[]
BEMFbeta=[]
def getData():
    global dataBuffer,plotTime,speed1,speed2,speed3,IA,IB,Ialpha,Ibeta,Valpha,Vbeta,BEMFalpha,BEMFbeta
    speed1=[]
    speed2=[]
    speed3=[]
    IA=[]
    IB=[]
    Ialpha=[]
    Ibeta=[]
    Valpha=[]
    Vbeta=[]
    BEMFalpha=[]
    BEMFbeta=[]
    recordedBuffers=[]
    dataBuffer=[]
    plotTime = []
    
    # tx = [0x89,0x00,0x00,0x20,0x11,0x00,0x19,0x00,0x59,0x00,0x49,0x00]
    # tx=[0xA9,0x01,0x00,0xE0,0x11,0x00,0x19,0x00,0x59,0x00,0x99,0x00,0xD1,0x07,0x11,0x08,0x51,0x08,0x91,0x08,0x51,0x0A,0x91,0x0A,0x51,0x0C,0x91,0x0C,0x49,0x00]
    # tx2=[0xC9,0x00,0x00,0xC0,0x08,0x00,0xA9,0x01,0x06,0x00,0xDC,0x05,0x00,0x00,0xE8,0x03]
    # tx3 = [0xC9,0x00,0x00,0xC0,0x08,0x00,0xA9,0x01,0x06,0x00,0xD0,0x07,0x00,0x00,0xE8,0x03]
    
    
    # while startDataTransfer:
    plotTime =[]
    # statusText.set("Data transfer in progress!")
    sp.flush()
    sp.flushInput()
    sp.flushOutput()
    root.update()
    
    t1 = time.time_ns()
    
    asd = []
    totalPts=int(dataPts.get())
    
    for i in range(totalPts):
        # if i == (round(totalPts/3)):
            # sp.write(tx2)
            # sp.read(5)
            
        # if i == (round(2*totalPts/3)):
            # sp.write(tx3)
            # sp.read(5)
            
        # sp.write(tx)
        
        # plotTime.append((time.time_ns()-t1))
        # t2 = time.time_ns()
        # recordedBuffers.append(list(sp.read(14)))
        # asd.append(time.time_ns() - t1)
        recordedBuffers.append(list(sp.read(6)))
        # t3 = time.time_ns()
        # asd.append((t3-t2)/1000000000)
    i=0
    aa=0
    bb=0
    cc=0
    for dataBuffer in recordedBuffers:
        # if((dataBuffer[0] + 256*dataBuffer[1]) < 4096):
        i+=1
        speed1.append((dataBuffer[0] + 256*dataBuffer[1]))# + 512*dataBuffer[2] + 1024*dataBuffer[3])
        speed2.append((dataBuffer[2] + 256*dataBuffer[3]))
        speed3.append((dataBuffer[4] + 256*dataBuffer[5]))
        asd.append(i)
        plotTime.append(i)
        # aa = 0.94*aa+0.061*(dataBuffer[0] + 256*dataBuffer[1])
        # speed1.append(aa)# + 512*dataBuffer[2] + 1024*dataBuffer[3])
        
        # bb = 0.94*bb+0.061*(dataBuffer[2] + 256*dataBuffer[3])
        # speed2.append(bb)
        
        # cc = 0.94*cc+0.061*(dataBuffer[3] + 256*dataBuffer[4])
        # speed3.append(cc)
        
        # IA.append(dataBuffer[16] + 256*dataBuffer[17])
        # IB.append(dataBuffer[18] + 256*dataBuffer[19])
        # Ialpha.append(dataBuffer[20] + 256*dataBuffer[21])
        # Ibeta.append(dataBuffer[22] + 256*dataBuffer[23])
        # Valpha.append(dataBuffer[24] + 256*dataBuffer[25])
        # Vbeta.append(dataBuffer[26] + 256*dataBuffer[27])
        # BEMFalpha.append(dataBuffer[28] + 256*dataBuffer[29])
        # BEMFbeta.append(dataBuffer[30] + 256*dataBuffer[31])
        
        
    # print((time.time_ns()-t1)/1000000000)
    # statusText.set("Data transfer complete!")
   
    # print("Plotting graph!!!")
    print(speed)
    plot(plotTime,speed1,speed2,speed3)
    root.update()

    
def on_closing():
    plt.close()
    sp.close()
    
    
def plot(x,y1,y2,y3):
    plt.cla()
    plt.plot(x,y1)
    plt.plot(x,y2)
    plt.plot(x,y3)
    fig.canvas.draw()


def saveClick():
    s1 = np.array(speed1)
    s2 = np.array(speed2)
    s3 = np.array(speed3)
    t = np.array(plotTime)
    # idm1 = np.array(idm)
    # iqm2 = np.array(iqm)
    # srf = np.array(speedref)
    df = pd.DataFrame({"time" : t, "s1" : s1, "s2" : s2, "s3" : s3})#,"speed reference" : srf,"id measured" : idm1,"iq measured" : iqm2,})
    df.to_csv(csvFileName.get()+".csv", index=False)


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



