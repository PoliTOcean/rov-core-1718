#!/usr/bin/env python

import rospy
from politocean.msg import *

def callback(data):
    rospy.loginfo('Start: %d rx: %d ry: %d lx: %d ly: %d' % (data.start, data.rx, data.ry, data.lx, data.ly))

def joystick_subscriber():
    rospy.init_node('joystick_subscriber', anonymous=True)

    rospy.Subscriber('joystick', joystick_data, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    joystick_subscriber()
