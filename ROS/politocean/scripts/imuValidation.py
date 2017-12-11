#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

angles=[0,0]

def rotx(a):
    Rx=[[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]]
    return Rx

def roty(b):
    Ry=[[np.cos(b),0,np.sin(b)],[0,1,0],[-np.sin(b),0,np.cos(b)]]
    return Ry

def rotz(c):
    Rz=[[np.cos(c),-np.sin(c),0],[np.sin(c),np.cos(c),0],[0,0,1]]
    return Rz

def rot2rpy(Rot):
    if len(Rot) == 3 & len(Rot[0]) == 3:
        yaw = np.arctan2(Rot[1][0], Rot[0][0])
        pitch= np.arctan2(-Rot[2][0], np.sqrt(Rot[2][1]*Rot[2][1]+Rot[2][2]*Rot[2][2]))
        roll = np.arctan2(Rot[2][1], Rot[2][2])
    else:
        print("Error in dim(R)")
        yaw = 0
        pitch = 0
        roll = 0
    return [roll, pitch, yaw]

def callback(data):
    print("Working")
    global angles
    roll=angles[0]
    pitch=angles[1]
    Ry=roty(pitch)
    Rx=rotx(roll)
    R=np.dot(Ry, Rx)
    # 3->5 gyro
    Rx=rotx(data.data[3]*0.01/1000)
    Ry=roty(data.data[4]*0.01/1000)
    Rz=rotz(data.data[5]*0.01/1000)
    dR=np.dot(Rz, Ry)
    dR=np.dot(dR, Rx)
    R=np.dot(R,dR)
    [roll_gyro, pitch_gyro, yaw_gyro]=rot2rpy(R)
    # 0->2 acc
    acc_tot=np.sqrt(data.data[0]*data.data[0]+data.data[1]*data.data[1]+data.data[2]*data.data[2])
    pitch_acc=np.arcsin(-data.data[0]/np.abs(acc_tot))
    roll_acc=np.arctan2(data.data[1], data.data[2])
    # filter
    if acc_tot>0.95 and acc_tot<1.05:
        roll=roll_gyro*0.9+roll_acc*0.1
        pitch=pitch_gyro*0.9+pitch_acc*0.1
    else:
        roll=roll_gyro
        pitch=pitch_gyro
    angles[0]=roll
    angles[1]=pitch

def imu_validation():
    pub = rospy.Publisher('rp_angles', Float32MultiArray, queue_size=0)
    rospy.init_node('imu_validation', anonymous=True)
    rospy.Subscriber('imu', Float32MultiArray, callback)
    rp_out = Float32MultiArray()
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        rp_out.data=angles
        pub.publish(rp_out)
        rate.sleep()



if __name__ == '__main__':
    try:
        imu_validation()
    except rospy.ROSInterruptException:
        pass
