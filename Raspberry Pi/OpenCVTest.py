import picamera
import picamera.array
import cv2
import serial
import time
import io
import numpy as np
from Vector import Vector 

# - - - - - - - - - - - - - - - - 
# - - OPEN CV TEST CLASS  - - - -
# - - - - - - - - - - - - - - - -
class OpenCVTest:
        
    # - - - - - - - - - - - - - - - - 
    # - - - - - -  INIT - - - - - - -
    # - - - - - - - - - - - - - - - -
    def __init__(self, height, width):
        """Constructor"""
 
        self.camera =  picamera.PiCamera()
        self.stream =  picamera.array.PiYUVArray(self.camera)
        
        self.height = height
        self.width = width
        
        self.camera.resolution = (self.height, self.width)

    # - - - - - - - - - - - - - - - - 
    # - - - - Take Picture  - - - - -
    # - - - - - - - - - - - - - - - - 
    def takePicture(self):
        self.stream = io.BytesIO() 
        self.camera.capture(self.stream, format='jpeg')
        self.data = np.fromstring(self.stream.getvalue(), dtype=np.uint8)
        self.camera._set_led(True)
        self.image = cv2.imdecode(self.data, 0) 
        self.edges = cv2.Canny(opencv.image, 200, 250)
        
        cv2.imshow('image1', self.image)        
        cv2.imshow('image2', self.edges)        
        cv2.waitKey(0)

# - - - - - - - - - - - - - - - - 
# - - - - - TEST AREA - - - - - -
# - - - - - - - - - - - - - - - -


opencv = OpenCVTest(256, 256)
opencv.takePicture()






