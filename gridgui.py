import tkinter as tk
import tkinter
import time


class Coordinate(object):
    x = 0
    y = 0
    isObstacle = False

    def __init__(self, x,y,obby):
        self.x = x
        self.y = y
        isObstacle=obby

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
def pasteCoords(coords):
    #Take tuples from coordList

    for (x,y) in coords:

        #Create oval based on coordinates
        c.create_oval(x-3, y-3, x+3 , y+3, fill='green')

def generatediagonalCoordsFile():
    f= open("coords.txt","w+")
    for i in range (0,500):
        f.write(""+ str(i+1)+","+str(i+1)+","+str(i+1)+"\n")

def moveRobot(coords):

    robot = c.create_rectangle(0, 0, 10, 10, fill="red")

    for (x,y) in coords:
        print("moving to: "+str((x,y)))
        c.coords(robot,x-5,y-5,x+5,y+5)
        c.update()
        c.after(50)

coordinates = {}
def getCoordinates():
    for c in range(0,500):
        for r in range(0,500):
            coordinates[str(r)+","+str(c)] = Coordinate(r,c,False)

visited = []
hori_min = 0
vert_min = 0
hori_max = 500
vert_max = 500

#coordinates represents dictionary w all coordinates
#key is the string of coordinate you want to access
def dfs(visited, coordinates, key):
    if key not in visited:
        if (coordinates[key].x == hori_min and coordinates[key].y == vert_min):
            neighbors=[[coordinates[key].x+1,0],[coordinates[key].x+1,coordinates[key].y+1],[0,coordinates[key].y+1]]
        elif(coordinates[key].x== hori_min and coordinates[key].y == vert_max):
            neighbors=[[coordinates[key].x+1,coordinates[key].y],[coordinates[key].x+1,coordinates[key].y-1],[coordinates[key].x,coordinates[key].y-1]]
        elif(coordinates[key].x == hori_max and coordinates[key].y == vert_max):
            neighbors=[[coordinates[key].x-1,coordinates[key].y],[coordinates[key].x-1,coordinates[key].y-1],[coordinates[key].x,coordinates[key].y-1]]
        elif(coordinates[key].x == hori_max and coordinates[key].y == vert_min):
            neighbors=[[coordinates[key].x-1,coordinates[key].y],[coordinates[key].x-1,coordinates[key].y-1],[coordinates[key].x,coordinates[key].y-1]]
        elif(coordinates[key].x==hori_min):
            neighbors=[[coordinates[key].x,coordinates[key].y-1],[coordinates[key].x+1,coordinates[key].y+1],[coordinates[key].x+1,coordinates[key].y],[coordinates[key].x+1,coordinates[key].y-1],[coordinates[key].x,coordinates[key].y-1]]
        elif(coordinates[key].x == hori_max):
            neighbors=[[coordinates[key].x,coordinates[key].y+1],[coordinates[key].x,coordinates[key].y-1],[coordinates[key].x-1,coordinates[key].y+1],[coordinates[key].x-1,coordinates[key].y],[coordinates[key].x-1,coordinates[key].y-1]]
        elif(coordinates[key].y==vert_min):
            neighbors=[[coordinates[key].x-1,coordinates[key].y],[coordinates[key].x-1,coordinates[key].y+1],[coordinates[key].x,coordinates[key].y+1],[coordinates[key].x+1,coordinates[key].y+1],[coordinates[key].x+1,coordinates[key].y]]
        elif(coordinates[key].y ==vert_max):
            neighbors=[[coordinates[key].x-1,coordinates[key].y],[coordinates[key].x-1,coordinates[key].y-1],[coordinates[key].x,coordinates[key].y-1],[coordinates[key].x+1,coordinates[key].y-1],[coordinates[key].x+1,coordinates[key].y]]
        else:
            neighbors=[[coordinates[key].x-1,coordinates[key].y-1],[coordinates[key].x,coordinates[key].y-1],[coordinates[key].x+1,coordinates[key].y-1],[coordinates[key].x+1,coordinates[key].y],[coordinates[key].x+1,coordinates[key].y+1],[coordinates[key].x,coordinates[key].y+1],[coordinates[key].x-1,coordinates[key].y+1],[coordinates[key].x-1,coordinates[key].y]]
        visited.append(key)
        for neighbor in neighbors:
            dfs(visited, coordinates, neighbor)

def createGrid(event=None):
    w = c.winfo_width() # Get current width of canvas
    h = c.winfo_height() # Get current height of canvas
    c.delete('grid_line') # Will only remove the grid_line

    # Creates all vertical lines at intevals of 100
    for i in range(0, w, 25):
        c.create_line([(i, 0), (i, h)], tag='grid_line')

    # Creates all horizontal lines at intevals of 100
    for i in range(0, h, 25):
        c.create_line([(0, i), (w, i)], tag='grid_line')



root = tk.Tk()
c = tk.Canvas(root, height=500, width=500, bg='white')
c.pack(fill=tk.BOTH, expand=True)

c.bind('<Configure>', createGrid)

generatediagonalCoordsFile()

#practice plotting points from CSV file to GUI
practiceCoords = readCoordsFromCsv("samplepoints.txt")
pasteCoords(practiceCoords)
getCoordinates()
print(coordinates["24,355"].x)
dfs(visited, coordinates, "24,355")

#move robot
coords = readCoordsFromCsv("coords.txt")
moveRobot(coords)

c.pack(fill =tk.BOTH, expand = True)
root.mainloop()
