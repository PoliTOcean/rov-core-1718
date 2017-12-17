#!/usr/bin/env python

''' The wifi data published on the url specified is an uint8, and so the topic data '''

import rospy
from std_msgs.msg import UInt8
from std_msgs.msg import String
from wifi import Cell, Scheme
import requests

def wifi_publisher():
    wifi_pub = rospy.Publisher('wifi_data', UInt8, queue_size=1)
    err_pub = rospy.Publisher('errors', String, queue_size=1)
    rospy.init_node('wifi_publisher', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    
    url_get='http://192.168.4.1/test'

    nets = list(Cell.all('wlp2s0')) # check the hardware
    if nets[0].ssid != 'wifi':
        mess='Please connect to \'wifi\''
        err_pub.publish(mess)
        rospy.loginfo(mess)
        return 1
    
    while not rospy.is_shutdown():
        try:
            data = requests.get(url_get)
        except requests.exceptions.ConnectionError as conn_err:
            mess='Errore di connessione'
            err_pub.publish(mess)
            rospy.loginfo(mess)
            
        try:
            rospy.loginfo(data.text)
            wifi_pub.publish(int(data.text))
        except NameError as ne: # data does not exist
            pass # already advised, coming from 'Errore di connessione'
        rate.sleep()

if __name__ == '__main__':
    try:
        wifi_publisher()
    except rospy.ROSInterruptException:
        pass
