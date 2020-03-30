from cell_based_forward_search import CellBasedForwardSearch
from Queue import PriorityQueue
import math
from cell import *
import rospy

# This class implements Dijkstra's forward search algorithm

class DijkstraPlanner(CellBasedForwardSearch):
    
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.priorityQueue = PriorityQueue()
        self.frontier = None
        self.frontierGot = False

    # This method determines if a cell is a frontier cell or not. A
    # frontier cell is open and has at least one neighbour which is
    # unknown.
    def isFrontierCell(self, x, y):

        # Check the cell to see if it's open
        if self.occupancyGrid.getCell(x, y) != 0:
            return False

        # Check the neighbouring cells; if at least one of them is unknown, it's a frontier
        return self.checkIfCellIsUnknown(x, y, -1, -1) | self.checkIfCellIsUnknown(x, y, 0, -1) \
            | self.checkIfCellIsUnknown(x, y, 1, -1) | self.checkIfCellIsUnknown(x, y, 1, 0) \
            | self.checkIfCellIsUnknown(x, y, 1, 1) | self.checkIfCellIsUnknown(x, y, 0, 1) \
            | self.checkIfCellIsUnknown(x, y, -1, 1) | self.checkIfCellIsUnknown(x, y, -1, 0)
            
    def checkIfCellIsUnknown(self, x, y, offsetX, offsetY):
        newX = x + offsetX
        newY = y + offsetY
        return (newX >= 0) & (newX < self.occupancyGrid.getWidthInCells()) \
            & (newY >= 0) & (newY < self.occupancyGrid.getHeightInCells()) \
            & (self.occupancyGrid.getCell(newX, newY) == 0.5)

    # Put the cell on the queue, using the path cost as the key to
    # determine the search order
    def pushCellOntoQueue(self, cell):
    
        if (cell.parent is not None):
            # Work out the cost of the action from the parent to self cell
            d = self.computeLStageAdditiveCost(cell.parent, cell)
            cell.pathCost = cell.parent.pathCost + d
        else:
            cell.pathCost = 0
            
        self.priorityQueue.put((cell.pathCost, cell))

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.priorityQueue.empty()

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        tuple = self.priorityQueue.get()
        return tuple[1]

    def resolveDuplicate(self, cell, parentCell):

        # See if the cost from the parent cell to this cell is shorter
        # than the existing path. If so, use it instead.
        dX = cell.coords[0] - parentCell.coords[0]
        dY = cell.coords[1] - parentCell.coords[1]
        d = math.sqrt(dX * dX + dY * dY)
        pathCostThroughNewParent = parentCell.pathCost + d
        if (pathCostThroughNewParent < cell.pathCost):
            cell.parent = parentCell
            cell.pathCost = pathCostThroughNewParent
            self.reorderPriorityQueue()

    # Reorder the queue. I don't see another way to do this, other than
    # create a new queue and copy over tuple-by-tuple. This rebuilds
    # the heap trees.
    def reorderPriorityQueue(self):
        newQueue = PriorityQueue()

        while self.priorityQueue.empty() is False:
            tuple = self.priorityQueue.get()
            newQueue.put(tuple)
             
        self.priorityQueue = newQueue

    # search the closest frontier
    def searchFrontier(self, start, blackList):
        
        # change the pose to the coords
        startCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates([start.x, start.y])
        
        self.handleChangeToOccupancyGrid()
        
        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.isQueueEmpty() == False):
            self.popCellFromQueue()

        # Check the start and end are not occupied. Note that "0.5" means
        # "don't know" which is why it is used as the threshold for detection.
        if (self.occupancyGrid.getCell(startCoords[0], startCoords[1]) > 0.5):
            return False

        # Get the start cell object and label it as such. Also set its
        # path cost to 0.
        self.start = self.searchGrid.getCellFromCoords(startCoords)

        #if self.start.label is CellLabel.OBSTRUCTED:
        #    return False
        
        self.start.pathCost = 0
        
        #if self.goal.label is CellLabel.OBSTRUCTED:
        #    return False

        if rospy.is_shutdown():
            return False

        # Insert the start on the queue to start the process going.
        self.markCellAsVisitedAndRecordParent(self.start, None)
        self.pushCellOntoQueue(self.start)

        # Reset the count
        self.numberOfCellsVisited = 0

        # initialize return variable
        self.frontierGot = False
        self.frontier = None

        # Iterate until we have run out of live cells to try or we reached the goal
        while (self.isQueueEmpty() == False):

            # Check if ROS is shutting down; if so, abort. This stops the
            # planner from hanging
            if rospy.is_shutdown():
                return False
            
            cell = self.popCellFromQueue()
            # check if the cell is frontier and not the start
            if (self.isFrontierCell(cell.coords[0], cell.coords[1]) == True and cell.coords != startCoords):
                flag = False
                # if the cell is not in the blacklist, it is the new destination
                for k in range(0, len(blackList)):
                    if blackList[k] == cell.coords:
                        flag = True
                        break
                if not flag:
                    self.frontier = cell.coords
                    self.frontierGot = True
                    break

            cells = self.getNextSetOfCellsToBeVisited(cell)
            for nextCell in cells:
                if (self.hasCellBeenVisitedAlready(nextCell) == False):
                    self.markCellAsVisitedAndRecordParent(nextCell, cell)
                    self.pushCellOntoQueue(nextCell)
                    self.numberOfCellsVisited = self.numberOfCellsVisited + 1
                else:
                    self.resolveDuplicate(nextCell, cell)

            # Now that we've checked all the actions for this cell,
            # mark it as dead
            self.markCellAsDead(cell)

        return self.frontierGot, self.frontier