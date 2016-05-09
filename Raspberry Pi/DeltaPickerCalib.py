import sys
from time import sleep
from random import random
import time
import math
sys.path.append("..")
import RPi.GPIO as GPIO
import os
from ImPro import ImPro
import numpy as np
import serial

# - - - - - - - - - - - - - - - - 
# - - - - - GPIO SETUP  - - - - -
# - - - - - - - - - - - - - - - -
GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.IN)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.setup(6,GPIO.IN)
GPIO.setup(13,GPIO.IN)
GPIO.setup(19,GPIO.IN)
GPIO.setup(26,GPIO.IN)
GPIO.setup(12,GPIO.IN)
GPIO.setup(16,GPIO.IN)
GPIO.setup(20,GPIO.IN)

# - - - - - - - - - - - - - - - - 
# - - - GLOBAL OBJECTS  - - - - -
# - - - - - - - - - - - - - - - -

imageProObj = ImPro(256, 256)
imageProObj.camera._set_led(False)
port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=3.0)

# - - - - - - - - - - - - - - - - 
# - - - SERIAL TEST SET UP  - - -
# - - - - - - - - - - - - - - - -
def calculateGoalPos(xPix, yPix):
    
    # The pixelposition reached when moving DeltaRobot with 0.0, 0.0
    offset_x = 131  #orig: 134
    offset_y = 145  #orig: 145

    # Calculating the relative position (with the middle as origin)
    relPixPosX = offset_x - xPix
    relPixPosY = offset_y - yPix

    print "relPixPosX: ", relPixPosX
    print "relPixPosY: ", relPixPosY
    
    if relPixPosY > 0.0 and relPixPosX > 0.0:
        # Y positive    X positive
        OutX = -  (0.123  * relPixPosX / offset_x - relPixPosY * 0.004 / 112.0)
        OutY =  0.123 * relPixPosY / 145.0  
    elif (relPixPosY > 0.0 and relPixPosX < 0.0):
        # Y positive    X negative
        OutX = -  (0.123  * relPixPosX / offset_x - relPixPosY * 0.004 / 112.0)
        OutY =  0.123 * relPixPosY / 145.0 
    elif relPixPosY < 0.0 and relPixPosX > 0.0:
        # Y negative    X positive
        OutX = -  (0.123  * relPixPosX / offset_x - relPixPosY * 0.008 / 112.0)
        OutY =  0.139 * relPixPosY / 145.0 
    else:
        # Y negative    X negative
        OutX = -  (0.117  * relPixPosX / offset_x + relPixPosY * 0.00 / 112.0)
        OutY =  0.141 * relPixPosY / 145.0 + relPixPosX * 0.002 / 40 
    return (OutX, OutY)

# - - - - - - - - - - - - - - - - 
# - - - - WRITE TO SERIAL - - - -
# - - - - - - - - - - - - - - - -
def writeToSerial(coordX, coordY):

    time.sleep(1)
    print "sending data 1"         
    port.write("{:.6f}".format(coordX))
    port.write("\n")
    #port.write("0.1\n")
    time.sleep(1)
    print "sending data 2"         
    port.write("{:.6f}".format(coordY))
    port.write("\n")
    time.sleep(1)
    print "sending data 3"         
    port.write("0.0")
    port.write("\n")

# - - - - - - - - - - - - - - - - 
# - - WRITE TO SERIAL CALIB - - -
# - - - - - - - - - - - - - - - -
def writeToSerialCALIB(coordX, coordY):
    
    time.sleep(1)
    print "pos 1"         
    port.write("0.0\n")
    port.write("-0.05\n")
    port.write("0.0\n")
    time.sleep(2)
    imageProObj.captureFrame()
    print "pos 2"         
    port.write("0.0\n")
    port.write("0.0\n")
    port.write("0.0\n")
    time.sleep(2)
    imageProObj.captureFrame()
    print "pos 3"         
    port.write("0.0\n")
    port.write("0.05\n")
    port.write("0.0\n")
    time.sleep(2)
    imageProObj.captureFrame()
 

        
# - - - - - - - - - - - - - - - - 
# - - - - - - - - - - - - - - - - 
# - - - MAIN PROG START - - - - -
# - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - 

print "doing the thing"    


#CAPTURE FRAME and GET PIXEL OBJECTS
imageProObj.blink()
imageProObj.captureFrame()
print "nr. of obj in pixel obj list: ", len(imageProObj.pixelObjList)
for ent in imageProObj.pixelObjList:
    if ent.numberOfPixels > 50:
        xVal, yVal = calculateGoalPos(ent.coord_x, ent.coord_y)
        print "xVal: ", xVal
        print "yVal: ", yVal
        writeToSerial(xVal, yVal)
        #writeToSerialCALIB(xVal, yVal)
        break; # break after one write to serial (just for testing purposes)


