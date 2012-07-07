#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('rxtxserver')
import time
import rospy
from rxtxserver.srv import *


def send_byte_client(data):
    rospy.wait_for_service('send_byte')
    try:
        send_byte = rospy.ServiceProxy('send_byte', Byte)
        rval = send_byte(data)
        return rval.Response
    except:
        "Service call failed"

def read_byte_client(indata):
    rospy.wait_for_service('read_byte')
    try:
        read_byte = rospy.ServiceProxy('read_byte', Read)
        rval = read_byte(indata)
        return rval.inResponse
    except:
         "Service call failed"

if __name__ == "__main__":
    self = True
    data = 96
    Rx = [0,0,0,0,0,0,0,0,0,0]
    i = 0
    while self:
        #send_byte_client(data)
        if(read_byte_client(Rx[0])):
            print chr(Rx[0])
