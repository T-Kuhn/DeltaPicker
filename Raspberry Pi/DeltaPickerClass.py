import serial
import time
from Vector import Vector 
from ImPro import ImPro

# - - - - - - - - - - - - - - - - 
# - - DELTA PICKER CLASS  - - - -
# - - - - - - - - - - - - - - - -
class DeltaPickerClass:
        
    # - - - - - - - - - - - - - - - - 
    # - - - - - -  INIT - - - - - - -
    # - - - - - - - - - - - - - - - -
    def __init__(self):
        """Constructor"""
        
        self.imageProObj = ImPro(256, 256)
        self.imageProObj.camera._set_led(False)
        # This serial port is used to send 3D coordinates to the Arduino. 
        self.port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=3.0)

        # This offset marks the pixelposition on the image which corresponds to the physical position when the
        # DeltaPicker move down to 0.0, 0.0, -0.466. Only x and y components are used.
        self.upperLevelOffset = Vector(0, 0, 0)
        # This offset is the low level offset. It gets taken at a hight of -0.41        
        self.lowerLevelOffset = Vector(0, 0, 0)

         
 
    # - - - - - - - - - - - - - - - - 
    # - - - STARTUP CALBIRATION - - -
    # - - - - - - - - - - - - - - - -
    def calib(self):

        self.writeToSerial(0.0, 0.0, -0.28)
        self.writeToSerial(0.0, 0.0, -0.36)
        self.writeToSerial(0.0, 0.0, -0.41)
        self.writeToSerial(0.0, 0.0, -0.45)
        self.writeToSerial(0.0, 0.0, -0.472)
        self.endSerialDataTrans()    
        time.sleep(12)
        self.writeToSerial(0.0, 0.0, -0.41)
        self.endSerialDataTrans()    
        time.sleep(3)
        print "take image of lower position"
        tmp = self.getPixelPositions()
        if tmp:
            self.lowerLevelOffset = tmp[0]
        print "lower level offset is    x: ", self.lowerLevelOffset.x, "   y: ", self.lowerLevelOffset.y 
        time.sleep(5)
        self.writeToSerial(0.0, 0.0, -0.36)
        self.writeToSerial(0.0, 0.0, -0.28)
        self.endSerialDataTrans()
        time.sleep(6)
        print "take image of upper position"
        tmp = self.getPixelPositions()
        if tmp:
            self.upperLevelOffset = tmp[0]
        print "upper level offset is    x: ", self.upperLevelOffset.x, "   y: ", self.upperLevelOffset.y 
        time.sleep(4)

    # - - - - - - - - - - - - - - - - 
    # - - - GET PIXELPOSITIONS - - -
    # - - - - - - - - - - - - - - - -
    def getPixelPositions(self):
        """Get's the Pixelpositions of the realworld objects which are laying below the DeltaPicker"""
        
        tmpList = [] 
    
        self.imageProObj.blink()
        # First we take a picture
        self.imageProObj.captureFrame()

        print "nr. of obj in pixel obj list: ", len(self.imageProObj.pixelObjList)
        for ent in self.imageProObj.pixelObjList:
            if ent.numberOfPixels > 50:
                print "Adding Object at PixelPos:"
                print "x: ", ent.coord_x
                print "y: ", ent.coord_y
                tmpList.append(Vector(ent.coord_x, ent.coord_y, 0))
        return tmpList

    # - - - - - - - - - - - - - - - - 
    # - - CALCULATE MODEL GOALPOS - -
    # - - - - - - - - - - - - - - - -
    def calculateModelGoalPos(self, xPix, yPix):
        
        # Calculating the relative position (with the middle as origin)
        relPixPosX = self.upperLevelOffset.x - xPix
        relPixPosY = self.upperLevelOffset.y - yPix

        print "relPixPosX: ", relPixPosX
        print "relPixPosY: ", relPixPosY
        
        if relPixPosY > 0.0 and relPixPosX > 0.0:
            # Y positive    X positive
            OutX = -  (0.123  * relPixPosX / self.upperLevelOffset.x - relPixPosY * 0.004 / 112.0)
            OutY =  0.123 * relPixPosY / 145.0  
        elif (relPixPosY > 0.0 and relPixPosX < 0.0):
            # Y positive    X negative
            OutX = -  (0.123  * relPixPosX / self.upperLevelOffset.x - relPixPosY * 0.004 / 112.0)
            OutY =  0.123 * relPixPosY / 145.0 
        elif relPixPosY < 0.0 and relPixPosX > 0.0:
            # Y negative    X positive
            OutX = -  (0.123  * relPixPosX / self.upperLevelOffset.x - relPixPosY * 0.008 / 112.0)
            OutY =  0.139 * relPixPosY / 145.0 
        else:
            # Y negative    X negative
            OutX = -  (0.117  * relPixPosX / self.upperLevelOffset.x + relPixPosY * 0.00 / 112.0)
            OutY =  0.141 * relPixPosY / 145.0 + relPixPosX * 0.002 / 40 

        if relPixPosX == 0:
            OutX = 0.0
        
        if relPixPosY == 0:
            OutY = 0.0

        return Vector(OutX, OutY, 0.0)

    # - - - - - - - - - - - - - - - - - - 
    # CALCULATE LOWER LEVEL CORRECTION  -
    # - - - - - - - - - - - - - - - - - -
    def calculateLowerLevelCorrection(self, xPix, yPix):
                
        # Calculating the relative position (with the middle as origin)
        relPixPosX = self.lowerLevelOffset.x - xPix
        relPixPosY = self.lowerLevelOffset.y - yPix
        
        print "relPixPosX: ", relPixPosX
        print "relPixPosY: ", relPixPosY
        
        if relPixPosY > 0.0 and relPixPosX > 0.0:
            # Y positive    X positive
            OutX = -0.04  * relPixPosX / self.upperLevelOffset.x
            OutY =  0.04 * relPixPosY / 145.0  
        elif (relPixPosY > 0.0 and relPixPosX < 0.0):
            # Y positive    X negative
            OutX = -0.04  * relPixPosX / self.upperLevelOffset.x
            OutY =  0.04 * relPixPosY / 145.0 
        elif relPixPosY < 0.0 and relPixPosX > 0.0:
            # Y negative    X positive
            OutX = -0.04  * relPixPosX / self.upperLevelOffset.x
            OutY =  0.04 * relPixPosY / 145.0 
        else:
            # Y negative    X negative
            OutX = -0.04  * relPixPosX / self.upperLevelOffset.x 
            OutY =  0.04 * relPixPosY / 145.0  
        
        if relPixPosX == 0:
            OutX = 0.0
        
        if relPixPosY == 0:
            OutY = 0.0
        
        
        return Vector(OutX, OutY, 0.0)
     
    # - - - - - - - - - - - - - - - - 
    # - - - GOT TO LOWER LEVEL  - - -
    # - - - - - - - - - - - - - - - -
    def goToLowerLevel(self, vect):
        self.writeToSerial(0.0, 0.0, -0.28)
        self.writeToSerial(0.0, 0.0, -0.36)
        self.writeToSerial(vect.x, vect.y, -0.41)
        self.endSerialDataTrans()    
        time.sleep(7)

    # - - - - - - - - - - - - - - - - 
    # - - CORRECT LOWER LEVEL POS - -
    # - - - - - - - - - - - - - - - -
    def correctLowerLevelPos(self, vect):
        self.writeToSerial(vect.x, vect.y, -0.41)
        self.endSerialDataTrans()    
        time.sleep(1)
    
    # - - - - - - - - - - - - - - - - 
    # - - - - - - PICK UP - - - - - -
    # - - - - - - - - - - - - - - - -
    def pickUp(self, vect):
        
        self.writeToSerial(vect.x, vect.y, -0.41)
        self.writeToSerial(vect.x, vect.y, -0.466)
        self.writeToSerial(vect.x, vect.y, -0.477)
        self.endSerialDataTrans()    
        time.sleep(3)
        self.writeToSerial(0.0, 0.0, -0.36)
        self.writeToSerial(0.0, 0.0, -0.28)
        self.endSerialDataTrans()    

    # - - - - - - - - - - - - - - - - 
    # - - - - PICK UP THINGS  - - - -
    # - - - - - - - - - - - - - - - -
    def PickUpThings(self):

        tmpList = []

        # First: Get the pixelpositions of all the objects with more than 50 pixels.
        tmpList = self.getPixelPositions();
        
        # Second: iterate through all the pixelpositions and get the corresponding realworld model coordinates
        for ent in tmpList:        
            tmp = []
            tmpVect = self.calculateModelGoalPos(ent.x, ent.y)
            print "tmpVect:   x: ", tmpVect.x, "    y: ", tmpVect.y
            self.goToLowerLevel(tmpVect)
            for i in range(4):            
                tmp = self.getPixelPositions()
                if tmp:
                    corVect = self.calculateLowerLevelCorrection(tmp[0].x, tmp[0].y)
                    print "corVect:   x: ", corVect.x, "    y: ", corVect.y
                    tmpVect.x += corVect.x
                    tmpVect.y += corVect.y
                    self.correctLowerLevelPos(Vector(tmpVect.x, tmpVect.y, 0))
                    if i == 3:
                        self.pickUp(Vector(tmpVect.x, tmpVect.y, 0))

    # - - - - - - - - - - - - - - - - 
    # - - - - WRITE TO SERIAL - - - -
    # - - - - - - - - - - - - - - - -
    def writeToSerial(self, coordX, coordY, coordZ):

        print "sending data 1"         
        self.port.write("{:.6f}".format(coordX))
        self.port.write("\n")
        print "sending data 2"         
        self.port.write("{:.6f}".format(coordY))
        self.port.write("\n")
        print "sending data 3"         
        self.port.write("{:.6f}".format(coordZ))
        self.port.write("\n")

    # - - - - - - - - - - - - - - - - 
    # - - END SERIAL DATA TRANS - - -
    # - - - - - - - - - - - - - - - -
    def endSerialDataTrans(self):

        print "ending Serial Data Trans"         
        self.port.write("e")
     
    









