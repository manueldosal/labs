
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
from cozmo.util import degrees, Pose
import asyncio

def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """

    start = grid.getStart()

    # Make the assumption that there is only one goal
    goal = grid.getGoals()[0]

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

    width = grid.width
    height = grid.height
    scale = grid.scale

    startX = grid.getStart()[0]
    startY = grid.getStart()[1]
    
    #Find Goal
    robot.move_lift(-3)
    robot.set_head_angle(degrees(0)).wait_for_completed()
    # Boolean that determines if Cozmo is already in the center of the grid
    inCenter = False
    cube1 = None

    #The intial goal is set to the center of the grid, in case the cube is not found initially
    grid.addGoal((width/2, height/2))

    while not stopevent.is_set():
        #Update start coordinate to be current robot's pose
        xR = int(robot.pose.position.x / scale) + startX
        yR = int(robot.pose.position.y / scale) + startY
        grid.setStart((xR, yR))

        cubeObstacle = None
        try:
            # TODO: Detect more than one obstacle.
            cubeObstacle = robot.world.wait_for_observed_light_cube(timeout=2)
        except asyncio.TimeoutError:
            print("No Cube obstacle was found")
        if cubeObstacle is not None:
            if cubeObstacle.object_id == 1 and cube1 is None:
                cube1 = cubeObstacle

                # We found cube 1. Clear the old goal of going to center.
                grid.clearGoals()

                #Get cube's 1 coordinates and rotation
                print("Found Cube 1:", cube1)
                #Assumption: Cube's coordinates is global coordiantes (where Robot originally started)
                xC = int(cube1.pose.position.x / scale) + startX
                yC = int(cube1.pose.position.y / scale) + startY
                angle_zC = cube1.pose.rotation.angle_z

                # Make cube 1 an obstacle to avoid hitting it
                addCubeObstacle(grid, (xC, yC))

                #Compute the final goal since we found Cube 1
                #This is the final distance where we want to finish from the cube
                distanceToCube = 4
                goalX = int(round(xC + distanceToCube * math.cos(angle_zC.radians)))
                goalY = int(round(yC + distanceToCube * math.sin(angle_zC.radians)))
                grid.addGoal((goalX, goalY))
                print("Goal:", grid.getGoals()[0])
            else:
                #We found cube 2 or 3. Let's make them obstacles
                xObstacle = int(cubeObstacle.pose.position.x / scale) + startX
                yObstacle = int(cubeObstacle.pose.position.y / scale) + startY
                print("Obstacle found:(", xObstacle, ",", yObstacle, ")")
                addCubeObstacle(grid, (xObstacle, yObstacle))

        #Stop robot if we are close enough to goal.
        numSquaresThreshold = 1
        goalX = grid.getGoals()[0][0]
        goalY = grid.getGoals()[0][1]
        distanceToGoal = math.sqrt(math.pow(goalX - xR, 2) + math.pow(goalY - yR, 2))
        if cube1 is not None:
            if distanceToGoal < numSquaresThreshold:
                #Rotate the robot to face cube's face and finish
                degreesToTurn = normalizeAngle(cube1.pose.rotation.angle_z.degrees + 180 - robot.pose.rotation.angle_z.degrees)
                print("degrees to turn:", degreesToTurn)
                robot.turn_in_place(degrees(degreesToTurn), speed=degrees(40)).wait_for_completed()
                break
        elif distanceToGoal < numSquaresThreshold or inCenter:
            # We haven't found cube1 yet but we are in the center of the grid. Just rotate to look around
            inCenter = True
            robot.turn_in_place(degrees(20), speed=degrees(40)).wait_for_completed()
            continue

        # Use A* to find the path to cube
        astar(grid, heuristic)

        path = grid.getPath()
        if len(path) >= 2:
            movStart = path[0]
            movEnd = path[1]
            robotAngle = math.degrees(math.atan2(movEnd[1] - movStart[1], movEnd[0] - movStart[0]))
            newRobotX = robot.pose.position.x + (movEnd[0] - movStart[0]) * scale
            newRobotY = robot.pose.position.y + (movEnd[1] - movStart[1]) * scale
            robot.go_to_pose(Pose(newRobotX, newRobotY, 0, angle_z=degrees(robotAngle)), relative_to_robot=False).wait_for_completed()

def addCubeObstacle(grid, cubePosition):
    for i in range(-2,3,1):
        for j in range(-2,3,1):
            obstacle = (cubePosition[0] + i, cubePosition[1] + j)
            if grid.coordInBounds(obstacle):
                grid.addObstacle(obstacle)

# return angle always in range (-180, 180] in deg
def normalizeAngle(heading):
    while heading > 180:
        heading -= 360
    while heading <= -180:
        heading += 360
    return heading

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

