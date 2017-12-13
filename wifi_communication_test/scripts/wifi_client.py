#!/usr/bin/env python

import rospy
from politocean.srv import wifi
import matplotlib.pyplot as plt

def wifi_client():
    rospy.wait_for_service('wifi_read')
    try:
        wifi_read = rospy.ServiceProxy('wifi_read', wifi)
        resp = wifi_read()
        return resp
    except rospy.ServiceException, e:
        print('Service call failed: %s'%e)

if __name__ == "__main__":
    print('Requesting data')
    data = wifi_client()
    plt.plot(data.x, data.y)
    plt.xlabel('Time')
    plt.ylabel('Amplitude')
    plt.title('Sismograph')
    plt.grid()
    plt.show()
