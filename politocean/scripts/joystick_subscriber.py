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
def joystickCallback(data):
    global check
    check+=1; #update check
    if(check>=1000)
        check=0;    #reset check
    #prepare command string
    comm=""
    if data.start:
        comm+="G " #go
    if data.up:
        comm+="U " #up
    if data.down:
        comm+="D " #down
    if (data.up or data.down) and (data.r1 or data.r2): #speed control
        comm+="V";
        if data.r1: #speed down
            comm+="-"
        elif data.r2: #speed up
            comm+="+"
        comm+=" "
    comm+="RX"+str(data.rx)+"  "
    comm+="RY"+str(data.ry)+"  "
    comm+="LY"+str(data.ly)+"  "
    comm+="LX"+str(data.lx)+"  "
    if data.select:
        comm="SSS" #stop
    try: #publish commands
        commands_pub.publish(comm)
    except rospy.ROSInterruptException as e:
        publishErrors(NODE.JOYSUB, "Commands topic publisher: "+str(e))

def main():
    errMessInit() #init topics

    #subscriber
    joystick_sub = rospy.Subscriber("joystick", joystick_data, joystickCallback)

    global check
    prec_check = -1
    delayed = False
    while not rospy.is_shutdown():
        #check for check variable
        if prec_check==check: #if it has been the same too long, we're not receiving joystick data
            if not delayed: #if it's the first time we see this
                commands_pub.publish("SSS") #send stop signal (DO NOT send it always if you want to have the possibility to use command line)
            delayed = True
        else:
            delayed = False
        prec_check = check
        sleep(1) #wait for another second

if __name__ == '__main__':
    main()
