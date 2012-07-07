#! /usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import roslib; roslib.load_manifest('rxtxserver')
import time
import rospy
from rxtxserver.srv import *

serialPort = "/dev/ttyUSB0"
baudrate = 38400

try:
    ser = serial.Serial(serialPort, baudrate, timeout = 1) 
    print("Serial at %s") % (serialPort)
except:
    print("Serial Unavailable at %s!") % (serialPort)
    quit()

def handle_send_byte(req):
    try:
        ser.write(chr(req.outData))
        return True
    except:
        print "sendData failed"
        return False

def handle_read_byte(req):
    if ser.inWaiting() != 0:
        req.inData = ser.read()
        return True
    else:
        return False


def InitRxTxServer():

    rospy.init_node('rxtx_server')
    s = rospy.Service('send_byte', Byte, handle_send_byte)
    r = rospy.Service('read_byte', Read, handle_read_byte)
    print "Serial Ready"
    rospy.spin()

if __name__ == "__main__":
    InitRxTxServer()





