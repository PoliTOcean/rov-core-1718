#!/usr/bin/env python

import rospy
from politocean.msg import *
from errmess_publisher import *
import serial

ser=serial.Serial(
port='/dev/ttyAMA0',
baudrate=300,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1
)

#set node name
rospy.init_node("ultrasounds", anonymous=False)

def joystickButtCallback(data):
    global sgancio

    if data.ID == "b_butt": #stop only the ROV
        sgancio = data.status

def main():
    errMessInit() #init topics
    
    global sgancio
    sgancio = 0
    
    joystick_butt_sub = rospy.Subscriber("joystick_buttons", joystick_buttons, joystickButtCallback)
    
    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        if sgancio:
            ser.write('A')        #open
            
        rate.sleep()

if __name__ == '__main__':
    main()
