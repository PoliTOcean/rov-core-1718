#!/usr/bin/env python

import rospy
from politocean.srv import wifi

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
    print(wifi_client().data)
