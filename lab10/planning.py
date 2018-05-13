
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo



def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """

    start = grid._start

    # Make the assumption that there is only one goal
    goal = grid._goals[0]

    # The nodes that are finished
    closedSet = set()

    # The nodes that need to be evaluated
    openSet = set()
    openSet.add(start)

    # The previous node where the key node can be reached most efficiently
    previousStep = {}

    # Distance from start to node
    gScore = {start : 0}

    # Distance from start to goal when passing through a node
    fScore = {start : heuristic(start, goal)}

    # The current node that will traverse the grid
    current = None

    while len(openSet):
        # Get the node in openSet with the lowest fScore value
        lowestFScore = math.inf
        for node in openSet:
            if node in fScore and fScore[node] < lowestFScore:
                current = node
                lowestFScore = fScore[node]

        # Stop the loop and set the path
        if current == goal:
            break
        
        # Move current node to closed set
        openSet.remove(current)
        closedSet.add(current)
        grid.addVisited(current)

        for neighborObj in grid.getNeighbors(current):
            neighbor = neighborObj[0]
            distanceToNeighbor = neighborObj[1]

            if neighbor in closedSet:
                # Don't do anything if the node is already closed
                continue

            if neighbor not in openSet:
                # Add neighbor if it hasn't been discovered yet
                openSet.add(neighbor)

            gScoreCurrent = gScore[current] if current in gScore else math.inf
            gScoreNeighbor = gScore[neighbor] if neighbor in gScore else math.inf
            tentativeGScore = gScoreCurrent +  distanceToNeighbor
            if tentativeGScore >= gScoreNeighbor:
                # This new path is not the most optimal
                continue

            # This new path is the best so far
            previousStep[neighbor] = current
            gScore[neighbor] = tentativeGScore
            fScore[neighbor] = gScore[neighbor] + heuristic(neighbor, goal)

    #Let's construct the path
    path = [current]
    while current in previousStep:
        current = previousStep[current]
        path.insert(0, current)
    grid.setPath(path)

def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """

    return math.sqrt(math.pow(current[0] - goal[0],2) + math.pow(current[1] - goal[1],2))


def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """
        
    global grid, stopevent
    
    while not stopevent.is_set():
        pass # Your code here


######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

