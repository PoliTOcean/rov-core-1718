# -*- coding: utf-8 -*-
"""
Created on Tue Apr  3 18:52:44 2018

@author: DETJON
"""

from time import sleep
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
from politocean.msg import *
import timeit
from errmess_publisher import *


CALIBRATE = 6 # Joystick botton to give 'start calibration'
DIR = 20   # Direction GPIO Pin
STEP = 21  # Step GPIO Pin
CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
SPR = 400   # Steps per Revolution (360 / 0.9) Sensibility of nema 17H is 0.9deg

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.output(DIR, CCW) #controllare senso per inizializzazione

step_count = SPR/2
delay = .01 # 4/400 seconds/steps --> I'm saing that I need 4 seconds for a complete rotation
            #                     --> 2sec for 180deg and so on

def NormMode(Old , New):
        
        if New > Old:
                GPIO.output(DIR, CW)                                  
                for i in range((New-Old)/(0.9)):
                    GPIO.output(STEP, GPIO.HIGH)
                    sleep(delay)
                    GPIO.output(STEP, GPIO.LOW)
                    sleep(delay)
                    if Joy.ID == 'z':
                        i = (New-Old)/(0.9)

        elif New < Old:
                GPIO.output(DIR, CCW)                                  
                for i in range((Old-New)/(0.9)):
                    GPIO.output(STEP, GPIO.HIGH)
                    sleep(delay)
                    GPIO.output(STEP, GPIO.LOW)
                    sleep(delay)
                    if Joy.ID == 'z':
                        i = (Old-New)/(0.9)
                    
def main():
## CALIBRATION
    Error = 1
    while ((Joy.ID != 'z' )  & (Joy.status != -1)):
        
    if ((Joy.ID == 'z' )  & (Joy.status == -1)):
        for x in range(step_count):
            GPIO.output(STEP, GPIO.HIGH)
            sleep(delay)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay)
            Sensor = Val_from_magnetic_sensor 
            
            if Sensor == 1:       # I find the initial position
                x = step_count
                Angle = 0
                Error = 0
                
# Waiting command from Joystick 
    while (!Error)  :
        
        if (isFloat(Joy.staus) & (Joy.ID == 'z' )):
        Joy_Manopola = Joy.status
        if Joy_Manopola > 1 | Joy_Manopola < -1:
            Error = 1
            print ("Errore lettura Joystick\n")
        else:
            
        Angle_new = 180*((Joy_Manopola/2)+0.5) # --> Joy rage [-1,+1] ; Angle range [0,180]
                                                # Joy=-1 --> angle=0 i.e. nitial position
        NormMode(Angle , Angle_new)
        
        Angle = Angle_new
        
        
        
GPIO.cleanup()
    





















