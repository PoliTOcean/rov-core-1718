#!/usr/bin/env python

from time import sleep
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
from politocean.msg import *
from errmess_publisher import *
from ADG import micro_stepping

CALIBRATE = 6 # Joystick botton to give 'start calibration'
DIR = 20   # Direction GPIO Pin
STEP = 16  # Step GPIO Pin
EN_n1 = 21 # Enable driver pin
CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
SPR = 400   # Steps per Revolution (360 / 0.9) Sensibility of nema 17H is 0.9deg

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(EN_n1, GPIO.OUT)
GPIO.output(DIR, CCW) #controllare senso per inizializzazione
GPIO.output(EN_n1, 1) # disable driver

step_count = SPR/2
delay = .001

def joystickAxisCallback(data):
    global Status
    global Angle_new
    
    if data.ID == "z":
        Status = data.status
    
        Angle_new = 180*((Status/2)+0.5)       # --> Joy rage [-1,+1] ; Angle range [0,180]
                                               # Joy=-1 --> angle=0 i.e. nitial position

def NormMode():
    global Status
    global Angle
    global Angle_new
        
    if Angle_new-0.45 > Angle:
        GPIO.output(DIR, CW)                                  
        while Angle < Angle_new:
            GPIO.output(STEP, GPIO.HIGH)
            GPIO.output(STEP, GPIO.LOW)
            Angle += 0.9
            sleep(delay)

    elif Angle_new < Angle:
        GPIO.output(DIR, CCW)                                  
        while Angle > Angle_new+0.45:
            GPIO.output(STEP, GPIO.HIGH)
            GPIO.output(STEP, GPIO.LOW)
            Angle -= 0.9
            sleep(delay)
                    
def main():
## CALIBRATION
    global Status
    global Angle
    
    Status = 0
    Actual_status = 0
    Error = 1
    
    # set node name
    rospy.init_node("Braccio", anonymous=False)
    #subscriber
    joystick_axis_sub = rospy.Subscriber("joystick_axis", joystick_axis, joystickAxisCallback)
    
    errMessInit() #init topics
    
    while (Status != -1):
        sleep(.1)
        
    if (Status == -1):
        GPIO.output(DIR, CCW)
        GPIO.output(EN_n1, 0)
        
        while 1:
            GPIO.output(STEP, GPIO.HIGH)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay)
            Sensor = 1 # value from magnetic sensor
            
            if Sensor == 1:       # I find the initial position
                Angle = 0
                Error = 0
                GPIO.output(EN_n1, 1)
                break

# Waiting command from Joystick 
    while Error == 0  :
        if Status > 1 or Status < -1:
            Error = 1
            publishErrors("Braccio", "Errore lettura Joystick\n")
            
        else:
            micro_stepping(0,2)
            GPIO.output(EN_n1, 0)
            NormMode()
            GPIO.output(EN_n1, 1)

if __name__ == '__main__':
    main()
        
GPIO.cleanup()