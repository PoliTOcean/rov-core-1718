#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
ISTRUZIONI PER L'USO:

    - Ricordarsi di settarlo come eseguibile

    - Avviare SOLO questo nodo e la GUI, quindi:
        lato ROV
            rosrun politocean camera_publisher.py

        lato GUI:
            rosrun politocean mainGUI.py
'''

import rospy
import numpy as np
from politocean.msg import *
import os, struct, array
from time import sleep
from errmess_publisher import *
import thread
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

#nome del nodo (eventualmente da aggiungere nel msg NODE)
rospy.init_node("CameraPublisher", anonymous=False)

#inizializzo l'array di frame
frames = [None, None, None]

def updateFrames(index):
    global frames

    # "cap" deve contenere la camera desiderata. Non so se è da modificare, ma
    # in teoria dovrebbe rilevare da solo le telecamere connesse e assegnare loro
    # indici progressivi (la mia del PC aveva index=0 ma ovviamente dopo non ce n'erano altre)
    cap = cv2.VideoCapture(index)

    while not rospy.is_shutdown():
        # Catturo il frame (OGNI frame)
        ret, frame = cap.read()

        # salvo la conversione in Image, con encoding bgr8 (lo stesso usato nella GUI)
        frames[index] = frame

    # release delle risorse
    cap.release()


def initCameras():
    global frames
    #dichiaro l'array di publisher
    pub = [ None, None, None ]
    for i in range(3): #inizializzo i thread e i publisher
        try:
            #passo "i" come argomento (il parametro "index" della funzione -> vedi su)
            thread.start_new_thread( updateFrames, (i, ) )
        except:
            print "Error: unable to start thread" #qui sarebbe da mettere un "publishError"
            return

        if i==0:
            pub[i] = rospy.Publisher('/cam_fhd/image_rect_color', Image, queue_size=0)
        else:
            pub[i] = rospy.Publisher('/cam'+str(i)+'/image_raw', Image, queue_size=0)

    #inizializzo index a 0
    index=0
    bridge = CvBridge() #bridge per conversione al tipo "Image" di ROS
    #45Hz, ovvero 45fps, ovvero 15fps*3 (15fps per ogni camera)
    rate = rospy.Rate(45) # 45 Hz
    while not rospy.is_shutdown():
        #pubblico sui topic, alternativamente, uno alla volta
        if frames[index] is not None:
            pub[index].publish(bridge.cv2_to_imgmsg(frames[index], encoding="bgr8"))
        #aggiorno l'index (0, 1, 2)
        index=(index+1)%3
        rate.sleep()


if __name__ == '__main__':
    initCameras()
