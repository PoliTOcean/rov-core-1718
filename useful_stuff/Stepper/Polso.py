# -*- coding: utf-8 -*-
"""
Created on Sat Apr  7 23:08:07 2018

@author: DETJON
"""

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
from politocean.msg import *
from time import sleep
import timeit
from errmess_publisher import *

DIR = 20   # Direction GPIO Pin
STEP = 21  # Step GPIO Pin
CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
SPR = 200  # Steps per Revolution (360 /1.7) Sensibility of nema 1.7deg


GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)

delay = .01

def Rotate_CW():
    
    GPIO.output(DIR, CW)
    
    while ( Joy.ID == 'wheel_bott' ) & ( Joy.status ):
        
        if ((Joy.ID == 'trottle') & (Joy.status > 0)):
            
            status = Joy.status
            delay_new = delay/(status*50)
            GPIO.output(STEP, GPIO.HIGH)
            sleep(delay_new)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay_new)
            
        else:
            return 
        
def Rotate_CCW():
    
    GPIO.output(DIR, CCW)
    
    while ( Joy.ID == 'wheel_bott' ) & ( Joy.status ):
        
        if ((Joy.ID == 'trottle') & (Joy.status < 0)):
            
            status = Joy.status
            delay_new = delay/(Joy.status*50)
            GPIO.output(STEP, GPIO.HIGH)
            sleep(delay_new)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay_new)
            
        else: 
            return 
        
def main():
    
   while not rospy.is_shutdown(): 
    
        if ( Joy.ID == 'wheel_bott' ) & ( Joy.status ) :
            if ((Joy.ID == 'trottle') & (Joy.status > 0)):
                Rotate_CW()
            else: 
                Rotate_CCW()
        
GPIO.cleanup()
    
    
    
