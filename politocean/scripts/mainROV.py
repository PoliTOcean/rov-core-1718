#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''This is the main node that communicates with ROS over the ROV Raspberry.
It receives commands to write over atmega and sends sensors data (or errors).
There is a system to wait for Arduino plug in and manage its disconnection.
'''
import roslib
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import serial
import timeit
import signal
import sys
import os
from time import sleep
from politocean.msg import *
from errmess_publisher import *

#atmega init
#atmega=serial.Serial("/dev/ttyUSB0",9600)
#atmega=serial.Serial("/dev/cu.usbserial-A900XW2W",9600)

#set node name
rospy.init_node(NODE.ROV,anonymous=False)

#sensors publisher
sensors_pub = rospy.Publisher('sensors', sensors_data, queue_size=3)

#global variables
atmega=None
ready=False #tell if ATMega is enabled
received = False #tell if ATMega is sending something (or if it's able to receive)

#function called when received something over components topic
def checkATMega(data):
    #check if it's a request for the ATMega component
    if data.ID==ID.ATMEGA and data.status==STATUS.REQUEST:
        #say if it's enabled or not
        if ready:
            publishComponent(NODE.ROV, ID.ATMEGA, STATUS.ENABLED)
        else:
            publishComponent(NODE.ROV, ID.ATMEGA, STATUS.DISABLED)

comp_sub = rospy.Subscriber('components', component_data, checkATMega)

#take commands and send to ATMega
def sendCommand(data):
    global received
    data = str(data).upper() #transform all the string uppercase
    if "W" in str(data): #awake command. It's for the ROV in general, not for the ATMega
        publishMessages(NODE.ROV, "???")
    elif ready: #if ready, write on the Serial
        try:
            atmega.write("/ "+str(data).replace('DATA: ', '').encode()+" \\")
            received = True
        except Exception as e:
            publishErrors(NODE.ROV, "Unable to write on ATMega: "+str(e))
            publishComponent(NODE.ROV, ID.ATMEGA, STATUS.BUSY) #say it's busy
            received = False

#publish sensors data over the topic
def publishSensors(sens):
    try:
        sensors = sensors_data() #init variable
        values = sens.split(' ') #split values on the string
        try: #catch exceptions for some data errors
            sensors.pressure = float(values[0])
            sensors.temperature = float(values[1])
            sensors.pitch = float(values[2])
            sensors.roll = float(values[3])
        except:
            sensors.pressure = 0.0
            sensors.temperature = 0.0
            sensors.pitch = 0.0
            sensors.roll = 0.0
        sensors_pub.publish(sensors) #publish
        #safety checks (we hope useless)
        if sensors.temperature >= 60:
            publishErrors(NODE.ROV, "ROV will stop! Temperature too high: "+str(sensors.temperature)+" degrees")
            sendCommand("SSS")
        elif sensors.temperature >= 42:
            publishMessages(NODE.ROV, "Temperature warning: "+str(sensors.temperature)+" degrees")
    except rospy.ROSInterruptException as e:
        publishErrors(NODE.ROV, "Closed sensors topic: "+str(e))

#subscribers
#commands topic
commands_sub = rospy.Subscriber("commands", String, sendCommand)

#function to connect to the ATMega
def connectToATMega():
    global ready
    ready = False #set atmega as not enabled
    cont=4
    atmega = None
    while atmega is None and not rospy.is_shutdown():
        try: #try to attach atmega
            atmega=serial.Serial("/dev/ttyACM0",9600)
            break
        except:
            #send message
            if cont>=4:
                cont=0
                publishMessages(NODE.ROV, "Trying to connect to ATMega...")
            else:
                cont+=1
            sleep(2) #wait for next attempt
    #send message and publish ATMega status
    publishMessages(NODE.ROV, "ATMega connected and enabled.")
    publishComponent(NODE.ROV, ID.ATMEGA, STATUS.ENABLED)
    ready = True #now it's ready
    return atmega


#function called at the exit
def exit():
    sendCommand("SSS") #send stop signal
    global atmega
    atmega = None
    #kill other nodes
    os.system("rosnode kill "+NODE.JOYSUB)
    os.system("rosnode kill "+NODE.CAM0)
    os.system("rosnode kill "+NODE.CAM1)
    os.system("rosnode kill "+NODE.CAM2)
    os.system("rosnode kill "+NODE.ROV) #even itself, to be safe
    sys.exit(0)

#function called when catched a signal SIGINT or SIGTERM
def exit_signal(signum, frame):
    exit() #call custom exit function


def main():
    #prepare to catch termination signals
    signal.signal(signal.SIGINT, exit_signal)
    signal.signal(signal.SIGTERM, exit_signal)
    global atmega
    global received

    errMessInit() #init topics

    os.system("rosrun politocean joystick_subscriber.py &")
    PATH_ROS = os.environ['ROS_PACKAGE_PATH'].split(':')[0]
    os.system("roslaunch "+PATH_ROS+"/politocean/launches/video_stream.launch &")

    s = sensors_data()
    s.pressure = 0.0
    s.temperature = 0.0
    s.pitch = 0.0
    s.roll = 0.0
    for i in range(1,6):
        sensors_pub.publish(s)
        sleep(0.2)

    atmega = connectToATMega()

    tic = timeit.default_timer() #set the timer
    received=False
    while not rospy.is_shutdown() and atmega is not None:
        try: #try to read from atmega
            if atmega is not None and atmega.in_waiting: #if there is something
                temp=atmega.readline().decode("utf-8") #read
                if "!!" in temp: #if it is an error
                    publishErrors(NODE.ROV, temp.replace('!!', '')) #send over errors topic
                else: #else, over sensors topic
                    publishSensors(temp)
                if not received:
                    publishComponent(NODE.ROV, ID.ATMEGA, STATUS.ENABLED)
                received = True
        except Exception as e: #if it couldn't read from atmega, then it has to be re-attached
            #publish an error and the disabled status of the component
            publishErrors(NODE.ROV, "ATMega disconnected.")
            publishComponent(NODE.ROV, ID.ATMEGA, STATUS.DISABLED)
            atmega = connectToATMega() #try to connect again
        if timeit.default_timer()-tic >= 3: #if 3 seconds are passed
            tic = timeit.default_timer()
            if not received: #and it didn't respond
                #send an error message and publish the busy status
                publishErrors(NODE.ROV, "ATMega is not responding")
                publishComponent(NODE.ROV, ID.ATMEGA, STATUS.BUSY)
            received = False #set to false to see if it will respond in next 3 seconds

    exit() #call custom exit function

if __name__ == '__main__':
    main()
