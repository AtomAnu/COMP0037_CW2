import rospy

from explorer_node_base import ExplorerNodeBase
from comp0037_reactive_planner_controller.fifo_planner import FIFOPlanner

from comp0037_reactive_planner_controller.cell import *
import numpy as np

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode2(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)
        self.planner = FIFOPlanner('FIFO', self.occupancyGrid)
        self.blackList = []

    def updateFrontiers(self):
        pass

    def chooseNewDestination(self):


        print 'blackList:'
        for coords in self.blackList:
            print str(coords)

        candidateGood = False
        destination = None
        """
        smallestD2 = float('inf')
        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                candidate = (x, y)
                if self.isFrontierCell(x, y) is True:
                    candidateGood = True
                    for k in range(0, len(self.blackList)):
                        if self.blackList[k] == candidate:
                            candidateGood = False
                            break
                    
                    if candidateGood is True:
                        d2 = candidate[0]**2+(candidate[1]-0.5*self.occupancyGrid.getHeightInCells())**2

                        if (d2 < smallestD2):
                            destination = candidate
                            smallestD2 = d2
        """
        frontiers = []

        startCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates([self.pose.x, self.pose.y])
        
        self.planner.handleChangeToOccupancyGrid()
        
        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.planner.isQueueoEmpty() == False):
            self.planner.popCellFromQueueo()

        # Check the start and end are not occupied. Note that "0.5" means
        # "don't know" which is why it is used as the threshold for detection.
        if (self.occupancyGrid.getCell(startCoords[0], startCoords[1]) > 0.5):
            return False

        # Get the start cell object and label it as such. Also set its
        # path cost to 0.
        self.planner.start = self.planner.searchGrid.getCellFromCoords(startCoords)

        #if self.start.label is CellLabel.OBSTRUCTED:
        #    return False
        
        self.planner.start.pathCost = 0
        
        #if self.goal.label is CellLabel.OBSTRUCTED:
        #    return False

        if rospy.is_shutdown():
            return False

        # Insert the start on the queue to start the process going.
        self.planner.markCell(self.planner.start, CellLabel.map_open_list)
        self.planner.pushCellOntoQueueo(self.planner.start)

        # Iterate until we have run out of live cells to try or we reached the goal
        while (self.planner.isQueueoEmpty() == False):

            # Check if ROS is shutting down; if so, abort. This stops the
            # planner from hanging
            if rospy.is_shutdown():
                return False
            
            cell = self.planner.popCellFromQueueo()

            if cell.Flabel == CellLabel.map_close_list:
                continue

            if self.isFrontierCell(cell.coords[0], cell.coords[1]) == True:
                flag = False
                for k in range(0, len(self.blackList)):
                    if self.blackList[k]== cell.coords:
                        flag = True
                        break
                if flag:
                    break
                # Make sure the queue is empty. We do this so that we can keep calling
                # the same method multiple times and have it work.
                while (self.planner.isQueueiEmpty() == False):
                    self.planner.popCellFromQueuei()

                if rospy.is_shutdown():
                    return False

                # Insert the start on the queue to start the process going.
                self.planner.pushCellOntoQueuei(cell)
                self.planner.markCell(cell, CellLabel.frontier_open_list)           

                newFrontier = []

                while (self.planner.isQueueiEmpty() == False):
                    # Check if ROS is shutting down; if so, abort. This stops the
                    # planner from hanging
                    if rospy.is_shutdown():
                        return False
                    
                    cell_2 = self.planner.popCellFromQueuei()
                    #print("Cell2: {}, {}".format(cell_2.coords, cell_2.Flabel))
                    if cell_2.Flabel == CellLabel.map_close_list or cell_2.Flabel == CellLabel.frontier_close_list:
                        continue

                    if self.isFrontierCell(cell_2.coords[0], cell_2.coords[1]) == True:
                        flag = False
                        for k in range(0, len(self.blackList)):
                            if self.blackList[k] == cell_2.coords:
                                flag = True
                                break
                        if flag:
                            break
                        newFrontier.append(cell_2)
                        cells = self.planner.getNextSetOfCellsToBeVisited(cell_2)

                        for nextCell in cells:
                            if (nextCell.Flabel != CellLabel.frontier_open_list) and (nextCell.Flabel != CellLabel.frontier_close_list) and \
                                (nextCell.Flabel != CellLabel.map_close_list):
                                self.planner.pushCellOntoQueuei(nextCell) 
                                self.planner.markCell(nextCell, CellLabel.frontier_open_list)

                    self.planner.markCell(nextCell, CellLabel.frontier_close_list)

                for c in newFrontier:
                    self.planner.markCell(c, CellLabel.map_close_list)

                frontiers.append(newFrontier)

            cells = self.planner.getNextSetOfCellsToBeVisited(cell)

            for nextCell in cells:
                if (nextCell.Flabel != CellLabel.map_open_list) and (nextCell.Flabel != CellLabel.map_close_list):
                    neighbors = self.planner.getNextSetOfCellsToBeVisited(cell)
                    for nextNeighbor in neighbors:
                        if (self.occupancyGrid.getCell(nextNeighbor.coords[0], nextNeighbor.coords[1]) < 0.5):
                            self.planner.pushCellOntoQueueo(nextCell)
                            self.planner.markCell(nextCell, CellLabel.map_open_list)
                            break
            
            self.planner.markCell(cell, CellLabel.map_close_list)

        #d = 0
        #for a in frontiers:
        #    for b in a:
        #        print("{} Cell: {}, {}".format(d, b.coords, b.Flabel))
        #    d += 1
        if len(frontiers) != 0:
            frontiersCoords = []
            #medians = []
            for frontier in max(frontiers, key=len):
                frontiersCoords.append(frontier.coords)
            temp = np.round(np.median(np.array(frontiersCoords), axis=0))
            destination = (temp[0], temp[1])
            candidateGood = True

        return candidateGood, destination

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
            print 'Adding ' + str(goal) + ' to the naughty step'
            # self.occupancyGrid.grid[goal[0]][goal[1]] = 1
            self.blackList.append(goal)
