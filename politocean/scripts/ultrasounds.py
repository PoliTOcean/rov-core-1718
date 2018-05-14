#!/usr/bin/env python

import rospy
from politocean.msg import *
from errmess_publisher import *
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
TX=14
GPIO.setup(TX, GPIO.OUT)
GPIO.output(TX, 1)  
#set node name
rospy.init_node("ultrasounds", anonymous=False)

def joystickButtCallback(data):
    global sgancio

    if data.ID == "top": #stop only the ROV
        sgancio = data.status

def main():
    errMessInit() #init topics
    
    global sgancio
    sgancio = 0
    
    joystick_butt_sub = rospy.Subscriber("joystick_buttons", joystick_buttons, joystickButtCallback)
    
    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        if sgancio:
	     GPIO.output(TX, 0)
      	     time.sleep(0.0034)
             GPIO.output(TX, 1)
             time.sleep(0.0034)
             GPIO.output(TX, 0)
             time.sleep(0.017)
             GPIO.output(TX, 1)
             time.sleep(0.0034)
             GPIO.output(TX, 0)
             time.sleep(0.0034)
             GPIO.output(TX, 1)            
        rate.sleep()

if __name__ == '__main__':
    main()
