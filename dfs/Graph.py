from Coordinate import Coordinate

class Graph:
    def performDFSAlgorithm(self,start):
        start.setTraversed(True)
        print(start)

        for neighbor in start.getNeighbors():
            if not neighbor.getTraversed() and not neighbor.getObstacle():
                self.performDFSAlgorithm(neighbor)


g = Graph()
c1 = Coordinate(1,1,[])
c2 = Coordinate(1,2,[])
c3 = Coordinate(1,3,[])
c4 = Coordinate(1,4,[])

c5 = Coordinate(2,1,[])
c6 = Coordinate(2,2,[])
c7 = Coordinate(2,3,[])
c8 = Coordinate(2,4,[])

c1.addNeighbor(c2)
c1.addNeighbor(c5)
c1.addNeighbor(c6)

c2.addNeighbor(c1)
c2.addNeighbor(c6)
c2.addNeighbor(c3)
c2.addNeighbor(c5)
c2.addNeighbor(c7)

c3.addNeighbor(c2)
c3.addNeighbor(c4)
c3.addNeighbor(c7)
c3.addNeighbor(c8)
c3.addNeighbor(c6)

c4.addNeighbor(c3)
c4.addNeighbor(c8)
c4.addNeighbor(c7)

c5.addNeighbor(c1)
c5.addNeighbor(c2)
c5.addNeighbor(c6)

c6.addNeighbor(c5)
c6.addNeighbor(c7)
c6.addNeighbor(c2)
c6.addNeighbor(c1)
c6.addNeighbor(c3)

c7.addNeighbor(c6)
c7.addNeighbor(c8)
c7.addNeighbor(c3)
c7.addNeighbor(c2)
c7.addNeighbor(c4)

c8.addNeighbor(c7)
c8.addNeighbor(c4)
c8.addNeighbor(c3)

g.performDFSAlgorithm(c1)
