#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
from politocean.msg import *
from time import sleep
from errmess_publisher import *
from ADG import micro_stepping

DIR = 27   # Direction GPIO Pin
STEP = 17  # Step GPIO Pin
CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
SPR = 200  # Steps per Revolution (360 /1.7) Sensibility of nema 1.7deg
EN_n1 = 22
cont = False

 
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(EN_n1, GPIO.OUT)
GPIO.output(EN_n1, 1) # En attivo basso

delay = 0.0005

def Rotate_CW():
    global ry
    
    GPIO.output(DIR, CW)
            
    if (ry > 0):
        delay_new = delay/ry
        GPIO.output(STEP, GPIO.HIGH)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay_new)
        
def Rotate_CCW():
    global ry
    
    GPIO.output(DIR, CCW)
            
    if (ry < 0):
        delay_new = -delay/ry
        GPIO.output(STEP, GPIO.HIGH)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay_new)
        
def joystickButtCallback(data):
    global e_butt
    global c_butt
    global cont

    if data.ID == "e_butt":
        e_butt = data.status
    if data.ID == "c_butt" and data.status == True :
	cont = not cont #cambio il valore ogni volta che il pulsante trigghera verso l'alto
	

def joystickAxisCallback(data):
    global ry
    

    if data.ID == "ry":
        ry = data.status
    
        
def main():
    global e_butt
    global ry
    global cont
    global c_butt

    e_butt = 0
    ry = 0
    cont = False

    # set node name
    rospy.init_node("Polso", anonymous=False)
    #subscriber
    joystick_butt_sub = rospy.Subscriber("joystick_buttons", joystick_buttons, joystickButtCallback)
    joystick_axis_sub = rospy.Subscriber("joystick_axis", joystick_axis, joystickAxisCallback)

    
    errMessInit() #init topics
    
    while not rospy.is_shutdown(): 
       
        if ( e_butt == 1 ) :
            micro_stepping(1,1)
            GPIO.output(EN_n1, 0)
            if (ry > 0):
                Rotate_CW()
            else: 
                Rotate_CCW()
        else:
            GPIO.output(EN_n1, not cont)
            sleep(0.1)

if __name__ == '__main__':
    main()
    
GPIO.cleanup()
