from Commands import *
from dfsonmap import Graph

class Engine:
    def run(self):
        longMin,longMax,latMin,latMax = getLongLatMinMaxFromUser()
        graph = Graph(longMin, longMax, latMin, latMax)
        graph.startDFSTraversalAtCoordinate(1,1)

         
engine = Engine()
engine.run()