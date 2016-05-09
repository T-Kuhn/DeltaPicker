import sys
sys.path.append("..")
import RPi.GPIO as GPIO
import os
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
while True:
    DeltaPickerObj.PickUpThings()
    print "done doing the thing."
