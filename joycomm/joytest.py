#! /usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import pygame
import threading
from threading import Timer, Thread
import time

header = chr(255)
dogBone = chr(254) # Feed the watchdog

updateInterval = 0.01


serialPort = "/dev/ttyUSB0"
baudRate = 9600
joystickNumber = 0

joy = []
msg = ""
num = ""
updatedAxis = 0
axisValue = [0, 0, 0, 0, 0, 0, 0]
axisSign  = [1, 1, 1, 1, 1, 1, 1]
vecR = 0
vecX = 0
vecY = 0

# Open serial connection
try:
    ser = serial.Serial(serialPort, baudRate, timeout=1)
    print("Serial at %s") % (serialPort)
except:
    print("Serial unavailable at %s!") % (serialPort)

# Send data
def sendData():
    try:
        ser.write(chr(axisValue[0]) + chr(axisValue[1]) + chr(axisValue[2]) + chr(axisValue[3]))
    except:
        print "sendData failed"

# Read data
def readData():
    try:
        while ser.inWaiting() != 0:
            RX = ser.readline()
        print RX
    except:
        return 0
        # print "Serial read failed!"

# TODO: combine all read/write messages and print on one line; align RX to left, append TX to right.

# def returnData():
#     try: return chr(axisValue[0]) + chr(axisValue[1]) + chr(axisValue[2]) + chr(axisValue[3])
#     except:
#         print "returnData failed"

# def feedDog():
#     try:
#         ser.write(dogBone) # Feed the watchdog
#     except:
#         pass

def startJoy():
    pygame.joystick.init()
    pygame.display.init()

    if not pygame.joystick.get_count():
        print "No joystick."
        quit()

    stick = pygame.joystick.Joystick(joystickNumber)
    stick.init()
    print "Joystick:", stick.get_name()

def getJoyEvents():
    event = pygame.event.poll()
#     feedDog()
    if (event.type == pygame.JOYAXISMOTION):
        global updatedAxis, axisValue
        updatedAxis = event.dict['axis']
        axisValue[updatedAxis] = int((axisSign[updatedAxis]*event.dict['value']+1)/2*250+1)
        # print "Axis %d: %d" % (updatedAxis, axisValue[updatedAxis])
        sendData()

class myJoy(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
    def run(self):
        while self.running:
            getJoyEvents()
            readData()
            time.sleep(updateInterval)


if __name__ == "__main__":
    startJoy()
    
    joyful = myJoy()
    joyful.start()
    raw_input("Hit <enter> to quit.")

    joyful.running = False
    joyful.join()

