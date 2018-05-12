#!/usr/bin/env python
'''
this node is responsible to get data from a joystick and send them
over the 'joystick' topic. It runs in parallel with joystick_checker, which
tells him the state of the joystick. If it has been disabled, it sops sending
data until it has been re-attached. '''
import rospy
from politocean.msg import *
import os, struct, array
from fcntl import ioctl
import time
from errmess_publisher import *
import thread
from std_msgs.msg import Bool

#set the name of the node
rospy.init_node(NODE.JOYPUB, anonymous=False)
#awake topic publisher
pub_awake = rospy.Publisher('joystick_awake', Bool, queue_size=0)

#check inout devices and initialize the joystick
def joy_init():
    # wait for available devices
    while True:
        if any(fn.startswith('js') for fn in os.listdir('/dev/input')):
            break
    time.sleep(1)

    publishComponent(NODE.JOYCHK, ID.JOYSTICK, STATUS.ENABLED)

    # We'll store the states here.
    axis_states = {}
    button_states = {}

    # mappng of the joystick axis and buttons
    axis_names = {
        0x00 : 'x',
        0x01 : 'y',
        0x02 : 'z',
        0x03 : 'rx',
        0x04 : 'ry',
        0x05 : 'rz',
        0x06 : 'trottle',
        0x10 : 'x_pad',
        0x11 : 'y_pad',
        0x28 : 'y_mouse',
        0x29 : 'x_mouse',
    }

    button_names = {
        0x000 : 'wheel',
        0x120 : 'trigger',
        0x121 : 'thumb',
        0x122 : 'thumb2',
        0x123 : 'top',
        0x124 : 'c_butt',
        0x125 : 'pinkie',
        0x126 : 'd_butt',
        0x127 : 'e_butt',
        0x128 : 'base1',
        0x129 : 'base2',
        0x12a : 'base3',
        0x12b : 'base4',
        0x12c : 'base5',
        0x12d : 'base6',
        0x12e : 'trigger2',

        0x12f : 'cpad_up',
        0x2c0 : 'cpad_right',
        0x2c1 : 'cpad_down',
        0x2c2 : 'cpad_left',

        0x2c3 : 'back_up',
        0x2c4 : 'back_right',
        0x2c5 : 'back_down',
        0x2c6 : 'back_left',

        0x2c7 : 'mode_1',
        0x2c8 : 'mode_2',
        0x2c9 : 'mode_3',

        0x2ca : 'function',
        0x2cb : 'start-stop',
        0x2cc : 'reset',
        0x2cd : 'i_butt',
        0x2ce : 'mouse_butt',
        0x2cf : 'wheel_butt',
    }

    axis_map = []
    button_map = []

    # Open the joystick device.
    fn = '/dev/input/js0'
    publishMessages(NODE.JOYPUB, 'Opening %s...' % fn)
    jsdev = open(fn, 'rb')

    # Get the device name.
    buf = array.array('c', ['\0'] * 64)
    ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
    publishMessages(NODE.JOYPUB, 'Device name: %s' % buf.tostring().strip('\0'))

    # Get number of axes and buttons.
    buf = array.array('B', [0])
    ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
    num_axes = buf[0]

    buf = array.array('B', [0])
    ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
    num_buttons = buf[0]

    # Get the axis map.
    buf = array.array('B', [0] * 0x40)
    ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

    for axis in buf[:num_axes]:
        axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
        axis_map.append(axis_name)
        axis_states[axis_name] = 0.0

    # Get the button map.
    buf = array.array('H', [0] * 200)
    ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

    for btn in buf[:num_buttons]:
        btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
        button_map.append(btn_name)
        button_states[btn_name] = 0

    return jsdev, button_map, button_states, axis_map, axis_states

#publish joystick data
def joystick_publisher(jsdev, button_map, button_states, axis_map, axis_states):
    #set joystick publisher
    pub_axis = rospy.Publisher('joystick_axis', joystick_axis, queue_size=0)
    pub_butt = rospy.Publisher('joystick_buttons', joystick_buttons, queue_size=0)

    #prepare variable that has to be sent
    axis_command = joystick_axis()
    butt_command = joystick_buttons()

    #loop until ros is active
    while not rospy.is_shutdown():

        try: # if the joystick is connected
            evbuf = jsdev.read(8)

            if evbuf:
                jtime, value, type, number = struct.unpack('IhBB', evbuf)

                # button events
                if type & 0x01:
                    button = button_map[number]
                    if button:
                        button_states[button] = value
                        butt_command.ID = button
                        butt_command.status = value

                        try:
                            pub_butt.publish(butt_command) #send data over the topic
                            publishComponent(NODE.JOYCHK, ID.JOYSTICK, STATUS.ENABLED)

                        except rospy.ROSInterruptException as e:
                            publishErrors(NODE.JOYPUB, "Joystick publisher error: "+str(e))

                # axis events
                if type & 0x02:
                    axis = axis_map[number]
                    if axis:
                        fvalue = value / 32767.0
                        axis_states[axis] = fvalue
                        axis_command.ID = axis
                        axis_command.status = fvalue

                        try:
                            pub_axis.publish(axis_command) #send data over the topic
                            publishComponent(NODE.JOYCHK, ID.JOYSTICK, STATUS.ENABLED)

                        except rospy.ROSInterruptException as e:
                            publishErrors(NODE.JOYPUB, "Joystick publisher error: "+str(e))

        except IOError: # if the joystick is disconnected
            publishMessages(NODE.JOYPUB, "Joystick has been disconnected.")
            publishComponent(NODE.JOYCHK, ID.JOYSTICK, STATUS.DISABLED)

            jsdev, button_map, button_state, axis_map, axis_state = joy_init()

            publishMessages(NODE.JOYPUB, "Joystick reconnected.")


def awakeCheck():
    global pub_awake    #publisher
    #while to tell the subscriber that the publisher is awake
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        pub_awake.publish(True)
        rate.sleep()




if __name__ == '__main__':
    errMessInit() #init topics

    #jsdev, button_map, button_state, axis_map, axis_state = joy_init()

    #init a thread to send check signals
    try:
        thread.start_new_thread(awakeCheck, ())
    except Exception as e:
        publishErrors(NODE.JOYPUB, "Unable to start the thread: "+str(e))
        print("THREAD ERROR!!!!")

    while not rospy.is_shutdown():
        pass
    #joystick_publisher(jsdev, button_map, button_state, axis_map, axis_state)
