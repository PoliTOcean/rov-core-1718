#!/usr/bin/env python
'''
This node receives joystick data and sends them through SPI to the ATmega from which
it receives back the sensors data'''
import rospy
from politocean.msg import *
from errmess_publisher import *
import spidev

spi = spidev.SpiDev()
spi.open(0, 0)

#sensors publisher definition
sensors_pub = rospy.Publisher('sensors', sensors_data, queue_size=0)
#set node name
rospy.init_node("spi_talker", anonymous=False)

#function that receives joystick data
def joystickButtCallback(data):
    global comm
    global bitArray
    global mode

    bitArray[6] = 0
    bitArray[5] = bitArray[7] = 1

    if data.ID == "thumb": #stop
        bitArray[4] = data.status
    if data.ID == "thumb2": #start
        bitArray[3] = data.status
    if data.ID == "trigger2": #fast up
        bitArray[0] = data.status
    if data.ID == "trigger": #slow up
        bitArray[1] = data.status
    if data.ID == "pinkie": #down
        bitArray[2] = data.status
    if data.ID == "mode_1" & data.status == True:
        mode = 1
    if data.ID == "mode_2" & data.status == True:
        mode = 0.6
    if data.ID == "mode_3" & data.status == True:
        mode = 0.3
    
    comm[3] = 0
    for i in range(8):
        comm[3] += bitArray[i]<<i
        
def joystickAxisCallback(data):
    global comm
    global mode
    
    if data.ID == "x": #laterale
        comm[1] = int((data.status*127*mode)+127)
    if data.ID == "y": #anvanti
        comm[0] = int((data.status*127*mode)+127)
    if data.ID == "rz": #rotazione
        comm[2] = int((data.status*127*mode)+127)

def main():
    errMessInit() #init topics
    
    # command array: [y x rz [trigger2 trigger pinkie thumb2 thumb 1 0 1]]
    # (avanti, lato, rotazione, [fastUp slowUp down start stop 1 0 1])
    global comm
    global bitArray
    global mode
    
    comm = [127, 127, 127, 160] # 127: mean value (0:255) for axis, 160: byte corresponding to the bit array
    bitArray = [0, 0, 0, 0, 0, 1, 0, 1]

    mode = 1
    
    values = sensors_data()

    #subscriber
    joystick_butt_sub = rospy.Subscriber("joystick_buttons", joystick_buttons, joystickButtCallback)
    joystick_axis_sub = rospy.Subscriber("joystick_axis", joystick_axis, joystickAxisCallback)
    
    rate = rospy.Rate(50) # 50 Hz
    
    while not rospy.is_shutdown():
                
        resp = spi.xfer2(list(comm))
        
        values.roll = resp[3]
        values.pitch = resp[2]
        values.pressure = resp[1]
        values.temperature = resp[0]
        
        try: #publish commands
            sensors_pub.publish(values)
        except rospy.ROSInterruptException as e:
            publishErrors("spi_talker", "Sensors data publisher: "+str(e))
        
        rate.sleep()

if __name__ == '__main__':
    main()
