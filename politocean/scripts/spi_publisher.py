#!/usr/bin/env python

import rospy
from errmess_publisher import *
from politocean.msg import *
import spidev

spi = spidev.SpiDev()
spi.open(0, 0)

def spi_publisher():
    spi_pub = rospy.Publisher('spi_rec_data', Int8, queue_size=1)
    rospy.init_node('spi_publisher', anonymous=False)
    rate = rospy.Rate(50) # 50hz

    while not rospy.is_shutdown():
        resp = spi.xfer2([0])
        if resp[0] > 127:
                data = (256-resp[0])*(-1)
        else:
                data = resp[0]

        try:
            rospy.loginfo(data)
            spi_pub.publish(data)
        except:
            pass

        rate.sleep()

if __name__ == '__main__':
    try:
        spi_publisher()
    except rospy.ROSInterruptException:
        pass
