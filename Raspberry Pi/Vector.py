import math
# - - - - - - - - - - - - - - - - 
# - - - - VECTOR CLASS  - - - - -
# - - - - - - - - - - - - - - - -
class Vector:
        
    # - - - - - - - - - - - - - - - - 
    # - - - - - -  INIT - - - - - - -
    # - - - - - - - - - - - - - - - -
    def __init__(self, x = 0, y = 0, z = 0):
        '''Constructor'''
        self.x = x
        self.y = y
        self.z = z
    

    # - - - - - - - - - - - - - - - - 
    # - - - SET DIST FROM OFFSET  - -
    # - - - - - - - - - - - - - - - -
    def setDistFromOffset(self, offset):
        '''Distance from upperLevel Offset'''
        self.DistFromOffset = math.sqrt((self.x - offset.x)**2 + (self.y - offset.y)**2)
        

    # - - - - - - - - - - - - - - - - 
    # - - - GET DIST FROM OFFSET  - -
    # - - - - - - - - - - - - - - - -
    def getDistFromOffset(self):
        return self.DistFromOffset 


