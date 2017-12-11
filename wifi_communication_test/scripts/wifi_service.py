#!/usr/bin/env python

import rospy
from politocean.srv import wifi
from std_msgs.msg import String
from wifi import Cell, Scheme
import requests

def read_data(req):
    global err_pub
    
    url_get='http://192.168.4.1/test'

    nets = list(Cell.all('wlp2s0')) # check the hardware
    if nets[0].ssid != 'wifi':
        mess='Please connect to \'wifi\''
        err_pub.publish(mess)
        rospy.loginfo(mess)
        return 0 # check

    try:
        data = requests.get(url_get)
    except requests.exceptions.ConnectionError as conn_err:
        mess='Errore di connessione'
        err_pub.publish(mess)
        rospy.loginfo(mess)

    try:
        rospy.loginfo(data.text)
        return int(data.text)
    except NameError as ne: # data does not exist
        pass # already advised, coming from 'Errore di connessione'

def wifi_server():
    rospy.init_node('wifi_server', anonymous=False)
    s = rospy.Service('wifi_read', wifi, read_data)
    global err_pub
    err_pub = rospy.Publisher('errors', String, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    wifi_server()
