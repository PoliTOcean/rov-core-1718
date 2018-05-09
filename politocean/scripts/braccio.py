#!/usr/bin/env python
from time import sleep
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
from politocean.msg import *
from errmess_publisher import *
from ADG import micro_stepping

DIR1 = 20   # Direction GPIO Pin
STEP1 = 16  # Step GPIO Pin
EN_n1 = 21  # Enable Pin active low

CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation

delay = 0.001

GPIO.setmode(GPIO.BCM)

GPIO.setup(DIR1, GPIO.OUT)
GPIO.setup(STEP1, GPIO.OUT)
GPIO.setup(EN_n1, GPIO.OUT)
GPIO.output(DIR1, CCW)
GPIO.output(EN_n1, 1)


def joystickButtCallback(data):
    global Status3
    global Status4

    if data.ID == "back_up":
        Status3 = data.status
    if data.ID == "back_down":
        Status4 = data.status

def main():
    global Status3
    global Status4
    Status3 = 0
    Status4 = 0

    # set node name
    rospy.init_node("Braccio", anonymous=False)
    #subscriber
    joystick_butt_sub = rospy.Subscriber("joystick_buttons", joystick_buttons, joystickButtCallback)
    errMessInit() #init topics
    micro_stepping(1,2)

    while not rospy.is_shutdown(): 
        while (Status3):
		GPIO.output(EN_n1, 0)
	 	GPIO.output(DIR1, CW)		#CV-->su

	 	GPIO.output(STEP1, GPIO.HIGH)
    	 	sleep(delay)
    	 	GPIO.output(STEP1, GPIO.LOW)
    	 	sleep(delay)

        GPIO.output(EN_n1, 1)

      	while (Status4):
		GPIO.output(EN_n1, 0)
	 	GPIO.output(DIR1, CCW)		#CVV-->giu

    	 	GPIO.output(STEP1, GPIO.HIGH)
    	 	sleep(delay)
    	 	GPIO.output(STEP1, GPIO.LOW)
    	 	sleep(delay)

        GPIO.output(EN_n1,1)
        sleep(0.1)

if __name__ == '__main__':
    main()
    
GPIO.cleanup()
