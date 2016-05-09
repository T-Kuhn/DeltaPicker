import picamera
import picamera.array
import png
from time import sleep
import math
import string
import PixelObj

# - - - - - - - - - - - - - - - - 
# - - - -  IMPRO CLASS  - - - - -
# - - - - - - - - - - - - - - - -
class ImPro:
        
    # - - - - - - - - - - - - - - - - 
    # - - - - - -  INIT - - - - - - -
    # - - - - - - - - - - - - - - - -
    def __init__(self, height, width):
        self.camera =  picamera.PiCamera()
        self.stream =  picamera.array.PiYUVArray(self.camera)
        
        self.height = height
        self.width = width
        
        self.camera.resolution = (self.height, self.width)

        self.pixelObjList = []
        self.objIDCntr = 0
        self.pixelObjList.append(PixelObj.PixelObj(self.getNextObjId(), self.height))
    
    # - - - - - - - - - - - - - - - - 
    # - - - GET NEXT OBJ ID - - - - -
    # - - - - - - - - - - - - - - - -
    def getNextObjId(self):
        self.objIDCntr += 1
        return self.objIDCntr

    # - - - - - - - - - - - - - - - - 
    # - - - - CAPTURE FRAME - - - - -
    # - - - - - - - - - - - - - - - -
    def captureFrame(self):    
        self.stream =  picamera.array.PiYUVArray(self.camera)
        self.camera.capture(self.stream, 'yuv')
        self.camera._set_led(True)

        self.pixelObjList = []
        self.objIDCntr = 0
        self.pixelObjList.append(PixelObj.PixelObj(self.getNextObjId(), self.height))

        rows = []
        for _ in range(self.height):
            rows.append(range(self.width))
        for j, j_ in enumerate(range(self.width - 1,-1,-1)): #flip image horizontally
            for i, i_ in enumerate(range(self.height - 1, -1, -1)): #flip vertically
                rows[j][i] = self.stream.array[j_][i_][0]
        
        self.savePNG('raw.png', rows)
        self.processFrame_5(self.processFrame_4(self.processFrame_3(self.processFrame_1(rows), self.processFrame_2(rows))))
    
    # - - - - - - - - - - - - - - - - 
    # - - - PROCESS FRAME 1 - - - - -
    # - - - - - - - - - - - - - - - -
    def processFrame_1(self, rawrows):
        '''get horizontal edges'''
        rows = []
        for _ in range(self.height):
            rows.append(range(self.width))  
        for j in range(self.height):
            for i in range(self.width):
                if i + 1 <= self.width - 1:
                    rows[j][i] = self.getDif(rawrows[j][i], rawrows[j][i+1])
                else:
                    rows[j][i] = self.getDif(rawrows[j][i], rawrows[j][i-1])
        self.savePNG('processed_1.png', rows)
        return rows

    # - - - - - - - - - - - - - - - - 
    # - - - PROCESS FRAME 2 - - - - -
    # - - - - - - - - - - - - - - - -
    def processFrame_2(self, rawrows):
        '''get vertical edges'''
        rows = []
        for _ in range(self.height):
            rows.append(range(self.width))  
        for j in range(self.height):
            for i in range(self.width):
                if j + 1 <= self.width - 1:
                    rows[j][i] = self.getDif(rawrows[j][i], rawrows[j+1][i])
                else:
                    rows[j][i] = self.getDif(rawrows[j][i], rawrows[j-1][i])
        self.savePNG('processed_2.png', rows)
        return rows
    
    # - - - - - - - - - - - - - - - - 
    # - - - PROCESS FRAME 3 - - - - -
    # - - - - - - - - - - - - - - - -
    def processFrame_3(self, hrows, vrows):
        '''fuse the horizontal edge-image with the vertical edge-image'''
        rows = []
        for _ in range(self.height):
            rows.append(range(self.width))  
        for j in range(self.height):
            for i in range(self.width):
                rows[j][i] = self.getFusion(hrows[j][i], vrows[j][i])
        self.savePNG('processed_3.png', rows)
        return rows

    # - - - - - - - - - - - - - - - - 
    # - - - PROCESS FRAME 4 - - - - -
    # - - - - - - - - - - - - - - - -
    def processFrame_4(self, edgeRows):
        '''make the image dual in color (black and white)'''
        threshhold = 20
        
        rows = []
        for _ in range(self.height):
            rows.append(range(self.width))  
        for j in range(self.height):
            for i in range(self.width):
                if edgeRows[j][i] >= threshhold:
                    rows[j][i] = 255
                else:
                    rows[j][i] = 0
        self.savePNG('processed_4.png', rows)
        return rows
    
    # - - - - - - - - - - - - - - - - 
    # - - -  PROCESS FRAME 4.5  - - -
    # - - - - - - - - - - - - - - - -
    def processFrame_4_5(self, bwRows):
        '''make all the white pixel spread out 1 more pixel! '''
        rows = []
        for _ in range(self.height):
            rows.append(range(self.width))  
        for j in range(self.height):
            for i in range(self.width):
                if bwRows[j][i] == 255:
                    tmpList = self.getNeighbours((i, j), self.height, self.width)
                    for ent in tmpList:
                        tmpX, tmpY = ent
                        rows[tmpY][tmpX] = 255
                else:
                    rows[j][i] = 0
        self.savePNG('processed_4_5.png', rows)
        
        self.processFrame_5(rows)

    # - - - - - - - - - - - - - - - - 
    # - - - PROCESS FRAME 5 - - - - -
    # - - - - - - - - - - - - - - - -
    def processFrame_5(self, bwRows):
        '''make PixelObjects by looking which pixels are direct 8-neighbours of each other'''
        for j in range(self.height):
            for i in range(self.width):
                if bwRows[j][i] == 255: #if the pixel is white
                    tmpList = []
                    for ent in self.getNeighbours((i, j), self.height, self.width):
                        tmp_x, tmp_y = ent
                        if bwRows[tmp_y][tmp_x] == 255: #if the pixel is white
                            tmpList.append(ent)
                    #print tmpList
                    flag = False
                    for obj in self.pixelObjList:
                        if obj.checkXYset(tmpList) is True: #make a new PixelObj whenever a Pixel isn't connected to an object
                            flag = True
                    if flag is False:
                        self.pixelObjList.append(PixelObj.PixelObj(self.getNextObjId(), self.height))
                        for obj in self.pixelObjList:
                            obj.checkXYset(tmpList) 
        for obj in self.pixelObjList:
            rows = []
            for _ in range(self.height):
                rows.append(range(self.width))  
            for j in range(self.height):
                for i in range(self.width):
                    if (i, j) in obj.XYset:
                        rows[j][i] = 255
                    else:
                        rows[j][i] = 0
            #self.savePNG(string.join([str(obj.id_), 'processed_5.png'], ''), rows)
        self.processPixelObj_1()
    
    # - - - - - - - - - - - - - - - - 
    # - - - PROCESS PIXEL OBJ 1 - - -
    # - - - - - - - - - - - - - - - -
    def processPixelObj_1(self):
        '''merge objects with overlapping x-y tuples together'''
        
        cntr = 0
        maxEntry = len(self.pixelObjList) -1
        oldlen = len(self.pixelObjList)
        flag = False
        while(cntr < maxEntry):
            tmp = self.checkForOverlap(cntr) 
            if tmp is False:
                cntr += 1
            else:
                for ent in self.pixelObjList[tmp].XYset:
                    self.pixelObjList[cntr].XYset.add(ent)
                del self.pixelObjList[tmp]
                maxEntry = len(self.pixelObjList) -1
                
        for obj in self.pixelObjList:
            obj.countPixel()
            obj.computeMeanCoord()
            #print obj.XYset
            rows = []
            for _ in range(self.height):
                rows.append(range(self.width))  
            for j in range(self.height):
                for i in range(self.width):
                    if (i, j) in obj.XYset:
                        rows[j][i] = 255
                    else:
                        rows[j][i] = 0
            #self.savePNG(string.join([str(obj.id_), 'pixpro.png'], ''), rows)
        print "nmbr of pre Objects: ", oldlen
        self.processPixelObj_3()

    # - - - - - - - - - - - - - - - - 
    # - - - PROCESS PIXEL OBJ  - - -
    # - - - - - - - - - - - - - - - -
    def processPixelObj_2(self):
        '''make a new png with 1 pixel per object at their respective center'''
        #IS BEEING SKIPPED!
        tmpPosList = []

        for obj in self.pixelObjList:
            for ent in obj.XYset:
                tmpPosList.append(ent)
            
        rows = []
        for _ in range(self.height):
            rows.append(range(self.width))  
        for j in range(self.height):
            for i in range(self.width):
                if (i, j) in tmpPosList:
                    rows[j][i] = 255
                else:
                    rows[j][i] = 0
        self.savePNG('PixelObj.png', rows)
        self.processPixelObj_3()

    # - - - - - - - - - - - - - - - - 
    # - - - PROCESS PIXEL OBJ 3 - - -
    # - - - - - - - - - - - - - - - -
    def processPixelObj_3(self):
        '''make a new png with 1 pixel per object at their respective center'''
        tmpPosList = []

        for obj in self.pixelObjList:
            print "X: ", obj.coord_x, " Y: ", obj.coord_y
            tmpPosList.append((obj.coord_x, obj.coord_real_y))
            
        rows = []
        for _ in range(self.height):
            rows.append(range(self.width))  
        for j in range(self.height):
            for i in range(self.width):
                if (i, j) in tmpPosList:
                    rows[j][i] = 255
                else:
                    rows[j][i] = 0
        self.savePNG('PixelObjPos.png', rows)
    
    # - - - - - - - - - - - - - - - - 
    # - - -  CHECK FOR OVERLAP  - - -
    # - - - - - - - - - - - - - - - -
    def checkForOverlap(self, counter):
        '''check for overlapping x-y tuples in sets in 2 distinct objects
        return the listNumber for the object with overlapping pixels if there are overlappning pixels
        return False if not'''
        maxEntry = len(self.pixelObjList) -1
        for ent1 in self.pixelObjList[counter].XYset:
            for i in range(counter+1, maxEntry + 1, 1):
                for ent2 in self.pixelObjList[i].XYset:
                    if ent1 == ent2:
                        return i
        return False
        
    # - - - - - - - - - - - - - - - - 
    # - - - - - SAVE PNG  - - - - - -
    # - - - - - - - - - - - - - - - -
    def savePNG(self, filename, rws):
        f = open(string.join(['img/', filename]), 'wb')
        w = png.Writer(self.height, self.width, greyscale=True)
        w.write(f, rws) 
        f.close()

    # - - - - - - - - - - - - - - - - 
    # - - - BLINK CAMERA LED  - - - -
    # - - - - - - - - - - - - - - - -
    def blink(self):
        for _ in range(4):
            self.camera._set_led(True)
            sleep(0.1)
            self.camera._set_led(False)
            sleep(0.1)

    # - - - - - - - - - - - - - - - - 
    # - - - - GET DIFFERENCE  - - - -
    # - - - - - - - - - - - - - - - -
    def getDif(self, a, b):
        if a >= b:
            return a - b
        else:
            return b - a 

    # - - - - - - - - - - - - - - - - 
    # - - - -  GET FUSION - - - - - -
    # - - - - - - - - - - - - - - - -
    def getFusion(self, a, b):
        a_ = int(a)
        b_ = int(b)
        tmp =  round(math.sqrt(a_*a_ + b_*b_))
        if tmp <= 255:    
            return int(tmp) 
        else:
            return 255
    
    # - - - - - - - - - - - - - - - - 
    # - - - - GET NEIGHBOURS  - - - -
    # - - - - - - - - - - - - - - - -
    def getNeighbours(self, (x, y), maxX, maxY):
        nList = []
        xx, yy = (x, y)
        for y_ in range(-1, 2, 1):
            for x_ in range(-1, 2, 1):
                resX = xx + x_
                resY = yy + y_
                if(resX < maxX and resX >= 0 and resY < maxY and resY >= 0):
                    nList.append((resX, resY))
        #print nList
        return nList

