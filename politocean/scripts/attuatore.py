#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
from politocean.msg import *
from time import sleep
from errmess_publisher import *
#limitare la corrente a 100 mA
#chiedere agli elettronici dove leggere la corrente 

global pwm_pin
global dir_pin
global fast
fast=0
pwm_pin=24
dir_pin=23
GPIO.setmode(GPIO.BCM)
GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(dir_pin, GPIO.OUT)
my_pwm=GPIO.PWM(pwm_pin,100)
my_pwm.start(0)

def apri():
        global d_butt
        global fast
        while(d_butt):
                GPIO.output(dir_pin, 1) #0 indica la direzione dell'attuatore
                my_pwm.ChangeDutyCycle(fast)
        my_pwm.stop()
        #leggi il val di rx imposta la corrente e apri


def chiudi ():
        global i_butt
        global fast
        while(i_butt):
                GPIO.output(dir_pin, 0) #1 indica la direzione dell'attuatore
		my_pwm.ChangeDutyCycle(fast)
        my_pwm.stop()
        #leggi il val di rx imposta la corrente e chiudi 




def joystickButtCallback(data):
    global i_butt
    global d_butt

    if data.ID == "i_butt":
        i_butt = data.status
    if data.ID == "d_butt":
        d_butt=data.status

def joystickAxisCallback(data):
    global rx
    global fast

    if data.ID == "rx":
        rx = data.status
        fast=(rx+1)*40+20

def main():
    global i_butt
    global d_butt
    global rx

    # set node name
    rospy.init_node("Attuatore", anonymous=False)
    #subscriber
    joystick_butt_sub = rospy.Subscriber("joystick_buttons", joystick_buttons, joystickButtCallback)
    joystick_axis_sub = rospy.Subscriber("joystick_axis", joystick_axis, joystickAxisCallback)
    rx=0
    i_butt=0
    d_butt=0

    errMessInit() #init topics

 while not rospy.is_shutdown():

        if ( d_butt == 1 ) :
           apri()

        if ( i_butt == 1) :
           chiudi()


if __name__ == '__main__':
    main()

GPIO.cleanup()



	

