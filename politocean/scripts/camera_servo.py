#!/usr/bin/env python
'''
This node moves the servo camera according to joystick commands'''

import rospy
from std_msgs.msg import String
from politocean.msg import *
from errmess_publisher import *

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685(0x7F)

# Configure min and max servo pulse lengths
servo_min = 130  # Min pulse length out of 4096
servo_max = 730 # Max pulse length out of 4096

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

#function that receives joystick data
def joystickButtCallback(data):
    global up
    global down
    
    if data.ID == "base1":
        up = data.status
        down = 0
    elif data.ID == "base2":
        up = 0
        down = data.status

def main():
    global up
    global down
    up = 0
    down = 0
    
    servo_val = 400 # Initialize the servo in a certain position
    interval = 2 # resolution of movement
    
    # set node name
    rospy.init_node("camera_servo", anonymous=False)
    #subscriber
    joystick_butt_sub = rospy.Subscriber("joystick_buttons", joystick_buttons, joystickButtCallback)
    
    errMessInit() #init topics
    
    rate = rospy.Rate(50) # 50 Hz
    
    while not rospy.is_shutdown():
        try:
            # Move servo on channel 3
            pwm.set_pwm(3, 0, servo_val)
        except:
            pass
        
        if up:
            if servo_val < servo_max - interval:
                servo_val = servo_val + interval
            else:
                publishMessages("camera_servo", "max value reached")
        elif down:
            if servo_val > servo_min + interval:
                servo_val = servo_val - interval
            else:
                publishMessages("camera_servo", "min value reached")
        
        rate.sleep()

if __name__ == '__main__':
    main()