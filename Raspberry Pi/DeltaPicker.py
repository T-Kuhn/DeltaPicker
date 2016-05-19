import sys
sys.path.append("..")
import RPi.GPIO as GPIO
import os
import time
from ImPro import ImPro
from DeltaPickerClass import DeltaPickerClass


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

DeltaPickerObj = DeltaPickerClass()
   
# - - - - - - - - - - - - - - - - 
# - - - - - - - - - - - - - - - - 
# - - - MAIN PROG START - - - - -
# - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - 

print "doing the thing"    

DeltaPickerObj.calib()
print "done calibrating"
DeltaPickerObj.PickUpThings()
time.sleep(10)
print "done doing the thing."


# So here's what happened lately:
# We found a way to put the vacuumpump on and off in sync with the position loading mechanism

# Know one big thing we want to think about is this:
# How can we get the thing to pick up more then 1 stone 

# The only problem really is, that we need to adjust the lower level pixelpos algorithms.
# At the moment, is is using the first object with more than 50 pixels.
# However, it should be more like this: 
# Use the piece which is closest to the place where it is expected.

# First, we try to make the lower level pixelpos algorithms better.

