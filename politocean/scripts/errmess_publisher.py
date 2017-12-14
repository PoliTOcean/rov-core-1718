'''
This script is imported from almost every other node, either GUI or ROV side.
It provides the interface to communicate over errors and messages topics, as
well as the components one, which is the topic that updates about components
status.
'''
import roslib
import rospy
from std_msgs.msg import String
from time import sleep
from politocean.msg import *

#publishers
#errors
errors_pub = rospy.Publisher('errors', String, queue_size=9)
#messages
messages_pub = rospy.Publisher('messages', String, queue_size=5)
#components
components_pub = rospy.Publisher('components', component_data, queue_size=0)

#publish errors over the topic
def publishErrors(nodeName, err):
    try:
        errors_pub.publish(nodeName+" ::: "+err)
    except:
        print("!!! Closed errors topic !!!")

#publish messages over topic
def publishMessages(nodeName, data):
    try:
        messages_pub.publish(nodeName+' ::: '+str(data))
    except rospy.ROSInterruptException as e:
        publishErrors(nodeName, "Messages publishing error: "+str(e))

#publish component ID status
def publishComponent(nodeName, ID, status):
    try:
        comp = component_data()
        comp.ID = ID
        comp.status = status
        components_pub.publish(comp)
    except rospy.ROSInterruptException as e:
        publishErrors(nodeName, "Components status publishing error: "+str(e))

#init topics
def errMessInit():
    #send packets to "active" topics
    for i in range(1,5):
        publishErrors("", "???")
        publishMessages("", "???")
        #in particular, send status requests for joystick and ATMEGA,
        #in order to update status on the GUI
        publishComponent("", ID.JOYSTICK, STATUS.REQUEST)
        sleep(0.5)
        publishComponent("", ID.ATMEGA, STATUS.REQUEST)
