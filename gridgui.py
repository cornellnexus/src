import tkinter as tk
import tkinter
import time

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

allCoordinates = []
def getCoordinates():
    for c in range(0,500):
        for r in range(0,500):
            allCoordinates.append([r,c])#Add coordinates in pairs, forming 2D array

    #print(allCoordinates)
visited = set()
hori_min = 0
vert_min = 0
hori_max = 500
vert_max = 500
def dfs(visited, coordinates, node):
    if node not in visited:
        if (node[0] == hori_min && node[1] == vert_min):
            neighbors=[[node[0]+1,0],[node[0]+1,node[1]+1],[0,node[1]+1]]
        elif(node[0]== hori_min && node[1] == vert_max):
            neighbors=[[node[0]+1,node[1]],[node[0]+1,node[1]-1],[node[0],node[1]-1]]
        elif(node[0] == hori_max && node[1] == vert_max):
            neighbors=[[node[0]-1,node[1]],[node[0]-1,node[1]-1],[node[0],node[1]-1]]
        elif(node[0] == hori_max && node[1] == vert_min):
            neighbors=[[node[0]-1,node[1]],[node[0]-1,node[1]-1],[node[0],node[1]-1]]
        elif(node[0]==hori_min):
            neighbors=[[node[0],node[1]-1],[node[0]+1,node[1]+1],[node[0]+1,node[1]],[node[0]+1,node[1]-1],[node[0],node[1]-1]]
        elif(node[0] == hori_max):
            neighbors=[[node[0],node[1]+1],[node[0],node[1]-1],[node[0]-1,node[1]+1],[node[0]-1,node[1]],[node[0]-1,node[1]-1]]
        elif(node[1]==vert_min):
            neighbors=[[node[0]-1,node[1]],[node[0]-1,node[1]+1],[node[0],node[1]+1],[node[0]+1,node[1]+1],[node[0]+1,node[1]]]
        elif(node[1] ==vert_max):
            neighbors=[[node[0]-1,node[1]],[node[0]-1,node[1]-1],[node[0],node[1]-1],[node[0]+1,node[1]-1],[node[0]+1,node[1]]]
        else:
            neighbors=[[node[0]-1,node[1]-1],[node[0],node[1]-1],[node[0]+1,node[1]-1],[node[0]+1,node[1]],[node[0]+1,node[1]+1],[node[0],node[1]+1],[node[0]-1,node[1]+1],[node[0]-1,node[1]]]
        print(node)
        visited.add(node)
        for neighbor in graph
            dfs(visited, graph, neighbor)

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

#move robot
coords = readCoordsFromCsv("coords.txt")
moveRobot(coords)

c.pack(fill =tk.BOTH, expand = True)
root.mainloop()
