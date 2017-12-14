import serial
import matplotlib.pyplot as plt
import numpy as np
import time

#ser=serial.Serial('/dev/ttyUSB0')
ser=serial.Serial('/dev/ttyACM0')

ser.reset_input_buffer()
time.sleep(2)

roll=[]
pitch=[]
a=[]
b=[]

for i in range(1,1000):
        comm=ser.readline()
        comm=comm.split( )
        if len(comm)==2:
                roll.append(comm[0])
                pitch.append(comm[1])
        else:
                print(i, ': error:', comm)


ser.close()

plt.plot(roll[1:-1],label='roll'), plt.plot(pitch[1:-1], label='pitch'), plt.title('Gyroscope')
plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

plt.show()
