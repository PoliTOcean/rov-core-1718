#!/usr/bin/env python
'''
This node has the window and ROV objects. Window is the GUI responsible,
while ROV is the responsible of ROS communications.
It has three threads:
 - ROV awakeness checker
 - string commands sender
 - gui
'''
from PyQt4 import QtGui
import sys
from ROV import ROV
from gui import Window
from time import sleep
from std_msgs.msg import String
import timeit
import thread
import rospy
import os
import signal
from errmess_publisher import *
from politocean.msg import *


#function to init ROV and send commands
def initAndSend(window, rov):

    rate = rospy.Rate(2) # 2 Hz
    #while to check if the ROV is awake, as well as tell the ROV that GUI is awake
    while not rospy.is_shutdown():
        tic = timeit.default_timer()   #tell the ROV that GUI is awake
        while not rospy.is_shutdown() and not rov.isAwake(): #if it's not awake
            sleep(0.1)
            if timeit.default_timer()-tic >= 5: #after a bit
                tic = timeit.default_timer()
                publishErrors(NODE.GUI, "ROV is not responding") #print on console
                publishComponent(NODE.GUI, ID.ATMEGA, STATUS.DISABLED) #publish ATMega status
            sleep(0.1)
        rate.sleep()

def main():

    #set the node name
    rospy.init_node(NODE.GUI, anonymous=False)

    app = QtGui.QApplication(sys.argv) #gui app init

    #window object
    window = Window(None)
    window.show() #Initialize Window, with disabled objects

    #init rov object
    rov = ROV(window)

    #init a thread passing window object as argument
    try:
        thread.start_new_thread(initAndSend, (window, rov, ))
    except:
        print("Unable to start the commands thread. Exit")
        exit(-1)

    #start the gui app
    n=app.exec_()

    sys.exit(n)

if __name__ == '__main__':
    main()
