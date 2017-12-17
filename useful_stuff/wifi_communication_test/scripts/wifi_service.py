#!/usr/bin/env python

import rospy
from politocean.srv import wifi
from std_msgs.msg import String
import requests

def read_data(req):
    global err_pub
    
    url_get='http://192.168.4.1/test_new'

    try:
        data = requests.get(url_get, timeout=1)
    except requests.exceptions.ConnectionError:
        mess='Connection error'
        rospy.loginfo(mess)
        err_pub.publish(mess)
        return [[0], [0]]

    rospy.loginfo('Data readed')
    
    data_div = data.text.split('\n')
        
    x = []
    y = []
    
    for couple in data_div:
        [X,Y] = couple.split()
        x.append(float(X))
        y.append(float(Y))

    return [x, y]

def wifi_server():
    global err_pub
    
    rospy.init_node('wifi_server', anonymous=False)
    s = rospy.Service('wifi_read', wifi, read_data)
    err_pub = rospy.Publisher('errors', String, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    wifi_server()
