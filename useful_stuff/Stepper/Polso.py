# -*- coding: utf-8 -*-
"""
Created on Sat Apr  7 23:08:07 2018

@author: DETJON
"""
#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
from politocean.msg import *
from time import sleep
import timeit
from errmess_publisher import *

DIR = 27   # Direction GPIO Pin
STEP = 17  # Step GPIO Pin
CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
SPR = 200  # Steps per Revolution (360 /1.7) Sensibility of nema 1.7deg
EN_n1 = 22


GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(EN_n1, GPIO.OUT)
GPIO.output(EN_n1,0)

delay = .01

def Rotate_CW():
    global e_butt
    global ry
    
    GPIO.output(DIR, CW)
    
    while ( e_butt == 1 ):
        
        if (ry > 0):
            
            delay_new = delay/(ry*50)
            GPIO.output(STEP, GPIO.HIGH)
            sleep(delay_new)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay_new)
            
        else:
            return 
        
def Rotate_CCW():
    global e_butt
    global ry
    
    GPIO.output(DIR, CCW)
    
    while ( e_butt == 1 ):
        
        if (ry < 0):
            
            delay_new = -delay/(ry*50)
            GPIO.output(STEP, GPIO.HIGH)
            sleep(delay_new)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay_new)
            
        else: 
            return 
        
def joystickButtCallback(data):
    
    global e_butt
    global ry
    
    if data.ID == "e_butt":
        e_butt = data.status
    if data.ID == "ry":
        e_butt = data.status
        
def main():
    
    global e_butt
    global ry
    
    # set node name
    rospy.init_node("Polso", anonymous=False)
    #subscriber
    joystick_butt_sub = rospy.Subscriber("joystick_butt", joystick_butt, joystickButtCallback)
    
    errMessInit() #init topics
    
    while not rospy.is_shutdown(): 
    
        if ( e_butt == 1 ) :
            GPIO.output(EN_n1,1)
            if (ry > 0):
                Rotate_CW()
            else: 
                Rotate_CCW()
        else :
            GPIO.output(EN_n1,0)
            sleep(1)
        
GPIO.cleanup()
    
    
    
