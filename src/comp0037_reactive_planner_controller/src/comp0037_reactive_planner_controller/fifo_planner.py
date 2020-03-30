# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque

# This class implements the FIFO - or breadth first search - planning
# algorithm. It works by using a double ended queue: cells are pushed
# onto the back of the queue, and are popped from the front of the
# queue.

class FIFOPlanner(CellBasedForwardSearch):

     # Construct the new planner object
    
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.fifoQueueo = deque()
        self.fifoQueuei = deque()

    # Simply put on the end of the queue
    def pushCellOntoQueueo(self, cell):
        self.fifoQueueo.append(cell)

    # Check the queue size is zero
    def isQueueoEmpty(self):
        return not self.fifoQueueo

    # Simply pull from the front of the list
    def popCellFromQueueo(self):
        cell = self.fifoQueueo.popleft()
        return cell

    def pushCellOntoQueuei(self, cell):
        self.fifoQueuei.append(cell)

    # Check the queue size is zero
    def isQueueiEmpty(self):
        return not self.fifoQueuei

    # Simply pull from the front of the list
    def popCellFromQueuei(self):
        cell = self.fifoQueuei.popleft()
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass

    # Mark the cell in WFD
    def markCell(self, cell, label):
        cell.Flabel = label
