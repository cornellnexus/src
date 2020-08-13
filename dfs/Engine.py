from UserUtils import *
from FileUtils import *
from Graph import Graph
from GUI import GUI

class Engine:

    def run(self):
        longMin,longMax,latMin,latMax = getLongLatMinMaxFromUser()
        graph = Graph(longMin, longMax, latMin, latMax)
        coordsList = graph.startDFSTraversalAtCoordinate(longMin,latMin)
        writeCoordsToCSVFile(coordsList)

        gui = GUI(longMax,latMax)
        gui.animateRobotMovingGivenCoordList(coordsList)

         
engine = Engine()
engine.run()