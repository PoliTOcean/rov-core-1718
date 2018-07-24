#!/usr/bin/env python
'''
This node receives joystick data and converts them into string commands.
If the variable "check" remains the same for some time, it sends a STOP signal
to the ROV, since it means that the joystick (or the ROV Raspberry) has been disconnected'''
import rospy
from std_msgs.msg import String
from politocean.msg import *
from time import sleep
import timeit
from errmess_publisher import *

#set node name
rospy.init_node(NODE.JOYSUB, anonymous=False)

#ROS publishers
errors_pub = rospy.Publisher('errors', String, queue_size=10)
commands_pub = rospy.Publisher('commands', String, queue_size=9)

#check variable
check = 0

#function that receives joystick data
def joystickButtCallback(data):
    global check
    global buttComm
    global thumb2
    global trigger
    global pinkie
    global mode_1
    global mode_3
    check+=1; #update check
    if(check>=1000):
        check=0;    #reset check
    #prepare command string
    buttComm=""
    
    if data.ID == "thumb2":
        thumb2 = data.status
    if data.ID == "trigger":
        trigger = data.status
    if data.ID == "pinkie":
        pinkie = data.status
    if data.ID == "mode_1":
        mode_1 = data.status
    if data.ID == "mode_3":
        mode_3 = data.status
    
    if thumb2:
        buttComm+="G " #go
    if trigger:
        buttComm+="U " #up
    if pinkie:
        buttComm+="D " #down
    if (trigger or pinkie) and (mode_1 or mode_3): #speed control
        buttComm+="V";
        if mode_1: #speed down
            buttComm+="-"
        elif mode_3: #speed up
            buttComm+="+"
        buttComm+=" "
        
    if data.ID == "thumb"  and data.status == True:
        buttComm="SSS" #stop
        
def joystickAxisCallback(data):
    global axisComm
    axisComm=""
    
    global x
    global y
    global y_pad
    global x_pad
    
    if data.ID == "x":
        x = int(data.status*100)
    if data.ID == "y":
        y = int(data.status*100)
    if data.ID == "y_pad":
        y_pad = int(data.status*100)
    if data.ID == "x_pad":
        x_pad = int(data.status*100)
        
    axisComm+="RX"+str(x)+"  "
    axisComm+="RY"+str(y)+"  "
    axisComm+="LY"+str(y_pad)+"  "
    axisComm+="LX"+str(x_pad)+"  "

def main():
    errMessInit() #init topics
    
    global check
    global buttComm
    global axisComm
    
    global thumb2
    global trigger
    global pinkie
    global mode_1
    global mode_3
    
    global x
    global y
    global y_pad
    global x_pad
    
    thumb2 = False
    trigger = False
    pinkie = False
    mode_1 = False
    mode_3 = False
    
    x = 0
    y = 0
    x_pad = 0
    y_pad = 0
    
    buttComm = ""
    axisComm = ""

    #subscriber
    joystick_butt_sub = rospy.Subscriber("joystick_buttons", joystick_buttons, joystickButtCallback)
    joystick_axis_sub = rospy.Subscriber("joystick_axis", joystick_axis, joystickAxisCallback)
    
    prec_check = -1
    delayed = False
    while not rospy.is_shutdown():
        #check for check variable
#        if prec_check==check: #if it has been the same too long, we're not receiving joystick data
#            if not delayed: #if it's the first time we see this
#                commands_pub.publish("SSS") #send stop signal (DO NOT send it always if you want to have the possibility to use command line)
#            delayed = True
#        else:
#            delayed = False
#        prec_check = check
        
        if buttComm != "SSS":
            comm = buttComm + axisComm
        else:
            comm = buttComm
        
        try: #publish commands
            commands_pub.publish(comm)
        except rospy.ROSInterruptException as e:
            publishErrors(NODE.JOYSUB, "Commands topic publisher: "+str(e))
        
        sleep(0.1) #wait for 0.1 seconds

if __name__ == '__main__':
    main()
