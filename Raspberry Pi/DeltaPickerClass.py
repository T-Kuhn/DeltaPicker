import serial
import time
import math
import picamera
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
        self.folderCount = 0;
        self.cam = picamera.PiCamera()
        self.imageProUpperLevel = ImPro(self.cam, 256, 256)
        self.imageProUpperLevel.camera._set_led(False)
        
        self.imageProLowerLevel = ImPro(self.cam, 256, 256, True, 96, 96)
        
        # This serial port is used to send 3D coordinates to the Arduino. 
        self.port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=3.0)

        # This offset marks the pixelposition on the image which corresponds to the physical position when the
        # DeltaPicker move down to 0.0, 0.0, -0.466. Only x and y components are used.
        self.upperLevelOffset = Vector(0, 0, 0)
        # This offset is the low level offset. It gets taken at a hight of -0.41        
        self.lowerLevelOffset = Vector(0, 0, 0)

        # Crop Offset
        self.cropOffset = Vector(80, 0, 0)
        

    # - - - - - - - - - - - - - - - - 
    # - - - - GO TO STARTPOS  - - - -
    # - - - - - - - - - - - - - - - -
    def goToStartPos(self):

        self.writeToSerial(0.0, 0.0, -0.36)
        self.writeToSerial(0.0, 0.0, -0.28)
        self.endSerialDataTrans()    
        time.sleep(7)

    # - - - - - - - - - - - - - - - - 
    # - - - STARTUP CALBIRATION - - -
    # - - - - - - - - - - - - - - - -
    def calib(self):

        self.writeToSerial(0.0, 0.0, -0.28)
        self.writeToSerial(0.0, 0.0, -0.36)
        self.writeToSerial(0.0, 0.0, -0.41)
        self.writeToSerial(0.0, 0.0, -0.45)
        self.writeToSerial(0.0, 0.0, -0.473)
        self.endSerialDataTrans()    
        time.sleep(12)
        self.writeToSerial(0.0, 0.0, -0.41)
        self.endSerialDataTrans()    
        time.sleep(3)
        print "take image of lower position"
        tmp = self.getPixelPositionsLowerLevel()
        if tmp:
            self.lowerLevelOffset = tmp[0]
        print "lower level offset is    x: ", self.lowerLevelOffset.x, "   y: ", self.lowerLevelOffset.y 
        time.sleep(5)
        self.writeToSerial(0.0, 0.0, -0.36)
        self.writeToSerial(0.0, 0.0, -0.28)
        self.endSerialDataTrans()
        time.sleep(6)
        print "take image of upper position"
        tmp = self.getPixelPositionsUpperLevel()
        if tmp:
            self.upperLevelOffset = tmp[0]
        print "upper level offset is    x: ", self.upperLevelOffset.x, "   y: ", self.upperLevelOffset.y 
        time.sleep(4)

    # - - - - - - - - - - - - - - - - 
    # - - GET PIXELPOSITIONS LOW  - -
    # - - - - - - - - - - - - - - - -
    def getPixelPositionsLowerLevel(self):
        """Get's the Pixelpositions of the realworld objects which are laying below the DeltaPicker in the Lower Level Position"""
        
        tmpList = [] 
    
        # First we take a picture
        self.imageProLowerLevel.captureFrame(self.cropOffset, self.folderCount)
        self.folderCount += 1

        print "nr. of obj in pixel obj list: ", len(self.imageProLowerLevel.pixelObjList)
        for ent in self.imageProLowerLevel.pixelObjList:
            if ent.numberOfPixels > 50:
                print "Adding Object at PixelPos:"
                print "x: ", ent.coord_x
                print "y: ", ent.coord_y
                tmpList.append(Vector(ent.coord_x, ent.coord_y, 0))
        return tmpList

        

    # - - - - - - - - - - - - - - - - 
    # - - GET PIXELPOSITIONS UP - - -
    # - - - - - - - - - - - - - - - -
    def getPixelPositionsUpperLevel(self):
        """Get's the Pixelpositions of the realworld objects which are laying below the DeltaPicker in the Upper Level Position"""
        
        tmpList = [] 
    
        #self.imageProUpperLevel.blink()
        # First we take a picture
        self.imageProUpperLevel.captureFrame(folderName=self.folderCount)
        self.folderCount += 1

        print "nr. of obj in pixel obj list: ", len(self.imageProUpperLevel.pixelObjList)
        for ent in self.imageProUpperLevel.pixelObjList:
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
            OutX = -0.06  * relPixPosX / self.upperLevelOffset.x
            OutY =  0.06 * relPixPosY / 145.0  
        elif (relPixPosY > 0.0 and relPixPosX < 0.0):
            # Y positive    X negative
            OutX = -0.06  * relPixPosX / self.upperLevelOffset.x
            OutY =  0.06 * relPixPosY / 145.0 
        elif relPixPosY < 0.0 and relPixPosX > 0.0:
            # Y negative    X positive
            OutX = -0.06  * relPixPosX / self.upperLevelOffset.x
            OutY =  0.06 * relPixPosY / 145.0 
        else:
            # Y negative    X negative
            OutX = -0.06  * relPixPosX / self.upperLevelOffset.x 
            OutY =  0.06 * relPixPosY / 145.0  
        
        if relPixPosX == 0:
            OutX = 0.0
        
        if relPixPosY == 0:
            OutY = 0.0
        
        
        return Vector(OutX, OutY, 0.0)
     
    # - - - - - - - - - - - - - - - - 
    # - CHOOSE CLOSEST TO EXPECTED  -
    # - - - - - - - - - - - - - - - -
    def chooseClosestToExpected(self, listOfPositions):
        lowestVal = 999
        for idx, ent in enumerate(listOfPositions):
            tmp = math.sqrt((ent.x - self.lowerLevelOffset.x)**2 + (ent.y - self.lowerLevelOffset.y)**2)
            if(tmp < lowestVal):
                lowestVal = tmp
                lowestValIndex = idx
        return listOfPositions[lowestValIndex]

    # - - - - - - - - - - - - - - - - 
    # - - - GOT TO LOWER LEVEL  - - -
    # - - - - - - - - - - - - - - - -
    def goToLowerLevel(self):
        self.writeToSerial(0.0, 0.0, -0.28)
        self.writeToSerial(0.0, 0.0, -0.36)
        self.writeToSerial(0.0, 0.0, -0.41)
        self.endSerialDataTrans()    
        time.sleep(5)

    # - - - - - - - - - - - - - - - - 
    # - GOT TO LOWER LEVEL POSITION -
    # - - - - - - - - - - - - - - - -
    def goToLowerLevelPosition(self, vect):
        self.writeToSerial(vect.x, vect.y, -0.41)
        self.endSerialDataTrans()    
        time.sleep(2)

    # - - - - - - - - - - - - - - - - 
    # - - - - - - - - - - - - - - - - 
    # - - CORRECT LOWER LEVEL POS - -
    # - - - - - - - - - - - - - - - -
    def correctLowerLevelPos(self, vect):
        self.writeToSerial(vect.x, vect.y, -0.41)
        self.endSerialDataTrans()    
        time.sleep(1)
   
    # - - - - - - - - - - - - - - - - 
    # - - - - PICK AND PLACE  - - - -
    # - - - - - - - - - - - - - - - -
    def pickAndPlace(self, vect):

        self.writeToSerial(vect.x, vect.y, -0.41)
        self.writeToSerial(vect.x, vect.y, -0.466)
        self.writeToSerial(vect.x, vect.y, -0.477)
        self.writeToSerial(vect.x, vect.y, -0.477, 1)
        self.endSerialDataTrans()    
        time.sleep(3)
        self.writeToSerial(vect.x, vect.y, -0.45 + self.placedThingsNumber * 0.0055, 1)
        self.writeToSerial(self.dropPos.x, self.dropPos.y, -0.468 + self.placedThingsNumber * 0.0055, 1)
        self.writeToSerial(self.dropPos.x, self.dropPos.y, -0.468 + self.placedThingsNumber * 0.0055)
        self.endSerialDataTrans()    
        time.sleep(4)
        #self.writeToSerial(0.0, 0.0, -0.41)
        #self.endSerialDataTrans()    
        #time.sleep(2)
        self.placedThingsNumber += 1        
        print "pick and place finished!"

 
    # - - - - - - - - - - - - - - - - 
    # - - - - - - PICK UP - - - - - -
    # - - - - - - - - - - - - - - - -
    def pickUp(self, vect):
        
        self.writeToSerial(vect.x, vect.y, -0.41)
        self.writeToSerial(vect.x, vect.y, -0.466)
        self.writeToSerial(vect.x, vect.y, -0.477)
        self.writeToSerial(vect.x, vect.y, -0.477, 1)
        self.endSerialDataTrans()    
        time.sleep(3)
        self.writeToSerial(vect.x, vect.y, -0.466, 1)
        self.writeToSerial(vect.x, vect.y, -0.41, 1)
        self.writeToSerial(0.0, 0.0, -0.36, 1)
        self.writeToSerial(0.0, 0.0, -0.36)
        self.endSerialDataTrans()    
        time.sleep(8)
        print "pickUp finished!"

    # - - - - - - - - - - - - - - - - 
    # - - - - PICK UP THINGS  - - - -
    # - - - - - - - - - - - - - - - -
    def PickUpThings(self):

        tmpList = []
        self.placedThingsNumber = 0

        # First: Get the pixelpositions of all the objects with more than 50 pixels.
        tmpList = self.getPixelPositionsUpperLevel();
        
        # Calculate distance to upperLevelOffset for all pixelOjects
        for ent in tmpList:
            ent.setDistFromOffset(self.upperLevelOffset)       

        # We need to rearrange the pixelobjects. near to the UpperLevelOffset objects should be first in the list.
        tmpList.sort(key=lambda ent: ent.getDistFromOffset(), reverse=False)     
 
        print "entries in tmplist:"
        for ent in tmpList:        
            print "ent:   x: ", ent.x, "    y: ", ent.y
        print "entries in tmplist:"
        
        # take the first entry in the list, refine the coordinates and use it as drop down zone.
        self.dropPos = firstEntry = tmpList.pop(0)
       
        # Calculate distance to dropPos for all pixelOjects
        for ent in tmpList:
            ent.setDistFromOffset(self.dropPos)      
        
        # We need to rearrange the pixelobjects. near to the dropPos objects should be first in the list.
        tmpList.sort(key=lambda ent: ent.getDistFromOffset(), reverse=False)     
        
        for ent in tmpList:        
            print "ent:   x: ", ent.x, "    y: ", ent.y
        
        self.goToLowerLevel()
        
        # refine the dropPos coordinates!
        dropPosVect = self.calculateModelGoalPos(self.dropPos.x, self.dropPos.y)
        print "dropPosVect:   x: ", dropPosVect.x, "    y: ", dropPosVect.y
        print "go to lower level position"
        self.goToLowerLevelPosition(dropPosVect)
        for i in range(2):            
            tmp = self.getPixelPositionsLowerLevel()
            if tmp:
                closestToExpected = self.chooseClosestToExpected(tmp)
                corVect = self.calculateLowerLevelCorrection(closestToExpected.x, closestToExpected.y)
                print "corVect:   x: ", corVect.x, "    y: ", corVect.y
                dropPosVect.x += corVect.x
                dropPosVect.y += corVect.y
                self.correctLowerLevelPos(Vector(dropPosVect.x, dropPosVect.y, 0))
                if i == 1:
                    self.dropPos.x = dropPosVect.x
                    self.dropPos.y = dropPosVect.y

        # Second: iterate through all the pixelpositions and get the corresponding realworld model coordinates
        for ent in tmpList:        
            tmp = []
            tmpVect = self.calculateModelGoalPos(ent.x, ent.y)
            print "tmpVect:   x: ", tmpVect.x, "    y: ", tmpVect.y
            print "go to lower level position"
            self.goToLowerLevelPosition(tmpVect)
            for i in range(2):            
                tmp = self.getPixelPositionsLowerLevel()
                if tmp:
                    closestToExpected = self.chooseClosestToExpected(tmp)
                    corVect = self.calculateLowerLevelCorrection(closestToExpected.x, closestToExpected.y)
                    print "corVect:   x: ", corVect.x, "    y: ", corVect.y
                    tmpVect.x += corVect.x
                    tmpVect.y += corVect.y
                    self.correctLowerLevelPos(Vector(tmpVect.x, tmpVect.y, 0))
                    if i == 1:
                        self.pickAndPlace(Vector(tmpVect.x, tmpVect.y, 0))
        print "finished PickUpThings!" 
        self.goToStartPos()

    # - - - - - - - - - - - - - - - - 
    # - - - - WRITE TO SERIAL - - - -
    # - - - - - - - - - - - - - - - -
    def writeToSerial(self, coordX, coordY, coordZ, state = 0):

        print "sending data 1"         
        self.port.write("{:.6f}".format(coordX))
        self.port.write("\n")
        print "sending data 2"         
        self.port.write("{:.6f}".format(coordY))
        self.port.write("\n")
        print "sending data 3"         
        self.port.write("{:.6f}".format(coordZ))
        self.port.write("\n")
        print "sending data 4"
        self.port.write("{:.6f}".format(state))
        self.port.write("\n")



    # - - - - - - - - - - - - - - - - 
    # - - END SERIAL DATA TRANS - - -
    # - - - - - - - - - - - - - - - -
    def endSerialDataTrans(self):

        print "ending Serial Data Trans"         
        self.port.write("e")
     
    









