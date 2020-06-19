import tkinter as tk
import tkinter


"""
must be in the format :   "0.0,1.2,1.3"
"""
def parseAndFormatCsvLine(line):
    firstComma = line.index(",")
    secondComma = line.index(",", firstComma+1)

    xCoord = float(line[0:firstComma])
    yCoord = float(line[firstComma+1:secondComma])
    angle = float(line[secondComma+1:]) #ignore for now b.c. we do not need it.

    return (xCoord, yCoord)


def readCoordsFromCsv(file):
    coordList = []
    f = open(file, "r")
    for line in f:
        coords = parseAndFormatCsvLine(line)
        coordList.append(coords)

    return coordList

#Take values from CSV file and plot them onto the screen
def pasteCoords(file):
    coordList = readCoordsFromCsv(file)

    #Take tuples from coordList
    for coordinate in coordList:

        #Convert tuple to string to get actual values
        StringCoordinate = str(coordinate)

        #Getting x and y coordinates from tuple string
        xCoord = StringCoordinate[1:StringCoordinate.index(",")]
        yCoord = StringCoordinate[StringCoordinate.index(" ")+1:StringCoordinate.index(")")]

        #Convert to floats
        xCoord = float(xCoord)
        yCoord = float(yCoord)

        #Create oval based on coordinates
        c.create_oval(xCoord-10, yCoord-10, xCoord+10 , yCoord+10, fill='red')



def createGrid(event=None):
    w = c.winfo_width() # Get current width of canvas
    h = c.winfo_height() # Get current height of canvas
    c.delete('grid_line') # Will only remove the grid_line

    # Creates all vertical lines at intevals of 100
    for i in range(0, w, 100):
        c.create_line([(i, 0), (i, h)], tag='grid_line')

    # Creates all horizontal lines at intevals of 100
    for i in range(0, h, 100):
        c.create_line([(0, i), (w, i)], tag='grid_line')




root = tk.Tk()
readCoordsFromCsv("coordinates.txt")
c = tk.Canvas(root, height=500, width=500, bg='white')
c.pack(fill=tk.BOTH, expand=True)

c.bind('<Configure>', createGrid)
pasteCoords("coordinates.txt")
c.pack(fill =tk.BOTH, expand = True)


root.mainloop()
