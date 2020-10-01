import math

class Vector:
    x1 = 0
    y1 = 0
    x2 = 0
    y2 = 0

    p1 = (0,0)
    p2 = (0,0)
    vector = (0,0)
    angle = 0
    magnitude = 0

    def calculateVector(self):
        self.vector = (self.x2 - self.x1, self.y2 - self.y1)

    def calculateAngle(self):
        self.angle = Math.atan((self.y2-self.y1)/(self.x2-self.x1))

    def calculateMagnitude(self):
        self.magnitude = Math.sqrt(((self.y2-self.y1)**2) + ((self.x2-self.x1)**2))



    def __init__(self,x1,y1,x2,y2):
        self.p1 = (x1,y1)
        self.x1 = x1
        self.y1 = y1

        self.p2 = (x2,y2)
        self.x2 = x2
        self.y2 = y2

        self.calculateVector()
        self.calculateAngle()
        self.calculateMagnitude()

    def getVector(self):
        return self.vector

    def getAngle(self):
        return self.angle

    def getMagnitude(self):
        return self.magnitude

    def setPoint1(self,x,y):
        self.p1 = (x,y)
        self.x1 = x
        self.y1 = y

        self.calculateVector()
        self.calculateAngle()
        self.calculateMagnitude()

    def setPoint2(self,x,y):
        self.p2 = (x,y)
        self.x2 = x
        self.y2 = y

        self.calculateVector()
        self.calculateAngle()
        self.calculateMagnitude()
