#!/usr/bin/env python
'''
This node receives joystick data and sends them through SPI to the ATmega from which
it receives back the sensors data'''
import rospy
from politocean.msg import *
from errmess_publisher import *
import spidev
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Bool

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12,GPIO.OUT)         #12V RC
GPIO.setup(7,GPIO.OUT)          #reset atMega

GPIO.output(7,1) #do not reset the atMega

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000

#set node name
rospy.init_node(NODE.JOYSUB, anonymous=False)
#sensors publisher definition
sensors_pub = rospy.Publisher('sensors', sensors_data, queue_size=0)
#check variable
check = 0

#function that receives joystick data
def joystickButtCallback(data):
    global comm
    global bitArray
    global mode
    global resp
    global ind
    global check

    check=0 #reset check variable

    bitArray[6] = 0
    bitArray[5] = bitArray[7] = 1

    if (data.ID == "thumb"): #stop
        bitArray[4] = data.status #stop
        GPIO.output(12,0) #disable 12V
    if data.ID == "thumb2": #start
        GPIO.output(12,1) #enable 12V
        bitArray[3] = data.status
    if data.ID == "trigger2": #fast up
        bitArray[0] = data.status
    if data.ID == "trigger": #slow up
        bitArray[1] = data.status
    if data.ID == "pinkie": #down
        bitArray[2] = data.status
    if (data.ID == "mode_1") & (data.status == True): #fast mode
        mode = 1
    if (data.ID == "mode_2") & (data.status == True): #normal mode
        mode = 0.6
    if (data.ID == "mode_3") & (data.status == True): #slow mode
        mode = 0.3

    if (data.ID == "top") & (data.status == True): #liftbag release
        ser.write('A')

    if (data.ID == "base6"): #stop only the atMega
        bitArray[4] = data.status #stop
    if (data.ID == "base5") & (data.status == True): #reset the atMega
        publishComponent(NODE.ROV, ID.ATMEGA, STATUS.DISABLED)
        publishMessages(NODE.ROV, "Resetting the ATMega...")
        GPIO.output(7,0) # reset the atMega
        time.sleep(0.1)
        GPIO.output(7,1)
        time.sleep(2)

        ind = 0
        publishMessages(NODE.ROV, "ATMega connected and enabled.")
        publishComponent(NODE.ROV, ID.ATMEGA, STATUS.ENABLED)

    comm[3] = 0
    for i in range(8):
        comm[3] += bitArray[i]<<i

def joystickAxisCallback(data):
    global comm
    global mode
    global check

    check=0 #reset check variable

    if data.ID == "x": #laterale
        comm[1] = int((data.status*127*mode)+127)
    if data.ID == "y": #anvanti
        comm[0] = int((data.status*127*mode)+127)
    if data.ID == "rz": #rotazione
        comm[2] = int((data.status*127*mode)+127)

def joystick_is_awake(data):
    global check
    check = 0   #reset check variable


def main():
    errMessInit() #init topics

    # command array: [y x rz [trigger2 trigger pinkie thumb2 thumb 1 0 1]]
    # (avanti, lato, rotazione, [fastUp slowUp down start stop 1 0 1])
    global comm
    global bitArray
    global mode
    global ind

    comm = [127, 127, 127, 160] # 127: mean value (0:255) for axis, 160: byte corresponding to the bit array
    bitArray = [0, 0, 0, 0, 0, 1, 0, 1]

    mode = 1
    ind = 0
    resp = [0,0,0,0b10100000]

    values = sensors_data()

    #subscriber
    joystick_butt_sub = rospy.Subscriber("joystick_buttons", joystick_buttons, joystickButtCallback)
    joystick_axis_sub = rospy.Subscriber("joystick_axis", joystick_axis, joystickAxisCallback)
    joystick_awake = rospy.Subscriber("joystick_awake", Bool, joystick_is_awake)

    rate = rospy.Rate(50) # 50 Hz

    publishMessages(NODE.ROV, "ATMega connected and enabled.") # per ora ci fidiamo funzioni
    publishComponent(NODE.ROV, ID.ATMEGA, STATUS.ENABLED)

    global check

    while not rospy.is_shutdown():
        #check if the publisher is still sending the checks
        if check>120: #if it's more than 2 seconds
            b = joystick_buttons()
            b.ID = "thumb"
            b.status = True
            joystickButtCallback(b) #send stop signal
            publishErrors("joystick_subscriber", "No income data from joystick")
        else:
            check+=1   #increment check


        resp[ind] = spi.xfer2([comm[ind]])[0]
        if ind < 3:
            ind += 1
        else:
            ind = 0

        values.roll = resp[0]-127
        values.pitch = resp[1]-127
        values.pressure = resp[2]
        values.temperature = (resp[3]&0b00011111) + 20

        if ((resp[3]&0b11100000)>>5) - 0b00000101 != 0: #if the 3 known bits do not match, reset
            publishErrors("spi_talker", "SPI communication error: data mismatched")
            publishComponent(NODE.ROV, ID.ATMEGA, STATUS.DISABLED)
            publishMessages(NODE.ROV, "Trying to connect to ATMega...")
            GPIO.output(7,0) # reset the atMega
            time.sleep(0.1)
            GPIO.output(7,1)
            time.sleep(2)

            resp[3] = 0b10100000
            ind = 0
            publishMessages(NODE.ROV, "ATMega connected and enabled.")
            publishComponent(NODE.ROV, ID.ATMEGA, STATUS.ENABLED)

        try: #publish commands
            sensors_pub.publish(values)
            publishComponent(NODE.ROV, ID.ATMEGA, STATUS.ENABLED)
        except rospy.ROSInterruptException as e:
            publishErrors("spi_talker", "Sensors data publisher: "+str(e))

        rate.sleep()

if __name__ == '__main__':
    main()
