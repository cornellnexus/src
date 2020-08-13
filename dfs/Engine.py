from Commands import *
from dfsonmap import Graph

class Engine:
    def run(self):
        longMin,longMax,latMin,latMax = getLongLatMinMaxFromUser()
        graph = Graph(longMin, longMax, latMin, latMax)
        graph.startDFSTraversalAtCoordinate(longMin,latMin)

         
engine = Engine()
engine.run()