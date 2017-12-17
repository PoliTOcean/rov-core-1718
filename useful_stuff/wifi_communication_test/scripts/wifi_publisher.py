#!/usr/bin/env python

''' The wifi data published on the url specified is an uint8, and so the topic data '''

import rospy
from std_msgs.msg import UInt8
from std_msgs.msg import String
import requests

def wifi_publisher():
    wifi_pub = rospy.Publisher('wifi_data', UInt8, queue_size=1)
    err_pub = rospy.Publisher('errors', String, queue_size=1)
    rospy.init_node('wifi_publisher', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    
    url_get='http://192.168.4.1/test'
    
    while not rospy.is_shutdown():
        try:
            data = requests.get(url_get, timeout=1)
        except requests.exceptions.ConnectionError:
            mess='Connection error'
            rospy.loginfo(mess)
            err_pub.publish(mess)
            return 0

        rospy.loginfo('Data readed')
            
        rospy.loginfo(data.text)
        wifi_pub.publish(int(data.text))

        rate.sleep()

if __name__ == '__main__':
    try:
        wifi_publisher()
    except rospy.ROSInterruptException:
        pass
