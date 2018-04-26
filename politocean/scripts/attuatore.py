#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
from politocean.msg import *
from time import sleep
from errmess_publisher import *
#limitare la corrente a 100 mA
#chiedere agli elettronici dove leggere la corrente 

global pwm_pin=24
global dir_pin=23
GPIO.setmode(GPIO.BCM)
GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(dir_pin, GPIO.OUT)
my_pwm=GPIO.PWM(pwm_pin,100)
my_pwm.start(0)

def apri(d_butt)
	global d_butt
	global fast
	GPIO.output(dir_pin, 0) #0 indica la direzione dell'attuatore
        my_pwm.ChangeDutyCycle(fast)
	#leggi il val di rx imposta la corrente e apri
	

def chiudi (i_butt)
	global i_butt
	global fast
	GPIO.output(dir_pin, 1) #1 indica la direzione dell'attuatore
	my_pwm.ChangeDutyCycle(fast)
	#leggi il val di rx imposta la corrente e chiudi 
	



def joystickButtCallback(data):
    global i_butt
    global d_butt

    if data.ID == "i_butt":
        i_butt = data.status
    elif data.ID == "d_butt":
	d_butt=data.status 
	
def joystickAxisCallback(data):
    global rx
    global fast
    
    if data.ID == "rx":
        rx = data.status
	fast=(rx+1)*40+20
        
def main():
    
    # set node name
    rospy.init_node("Attuatore", anonymous=False)
    #subscriber
    joystick_butt_sub = rospy.Subscriber("joystick_buttons", joystick_buttons, joystickButtCallback)
    joystick_axis_sub = rospy.Subscriber("joystick_axis", joystick_axis, joystickAxisCallback)
    ry=0
    i_butt=0
    d_butt=0

    errMessInit() #init topics
    
    while not rospy.is_shutdown(): 

	if ( d_butt == 1 ) :
           apri(d_butt)
	
	if ( i_butt == 1) :
	   chiudi(i_butt)

            
if __name__ == '__main__':
    main()
    
GPIO.cleanup()
    
    
    
