import rospy

from explorer_node_base import ExplorerNodeBase
from comp0037_reactive_planner_controller.dijkstra_planner import DijkstraPlanner

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)
        self.planner = DijkstraPlanner('Planner', self.occupancyGrid)
        self.blackList = []

    def updateFrontiers(self):
        pass

    def chooseNewDestination(self):

        print 'blackList:'
        for coords in self.blackList:
            print str(coords)

        candidateGood = False
        destination = None

        # Use the Dijkstra planner find the closest frontier
        candidateGood, destination = self.planner.searchFrontier(self.pose, self.blackList)
        # If we got a good candidate, use it

        return candidateGood, destination

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
            print 'Adding ' + str(goal) + ' to the naughty step'
            # self.occupancyGrid.grid[goal[0]][goal[1]] = 1
            self.blackList.append(goal)
