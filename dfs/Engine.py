from UserUtils import *
from FileUtils import *
from Graph import Graph
from GUI import GUI

class Engine:

    def run(self):
        longMin,longMax,latMin,latMax = getLongLatMinMaxFromUser()
        graph = Graph(longMin, longMax, latMin, latMax)
        #graph.printAllCoordinates()

        coordsList = graph.startDFSTraversalAtCoordinate(latMin,longMin)
        writeCoordsToCSVFile(coordsList)

        shouldDisplayGUI = getDisplayGuiFromUserInput()

        if (shouldDisplayGUI):
            gui = GUI(longMax,latMax)
            gui.animateRobotMovingGivenCoordList(coordsList)

         
engine = Engine()
engine.run()