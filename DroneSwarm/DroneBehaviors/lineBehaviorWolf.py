import airsim
from math import sqrt
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from airsim_ros_pkgs.msg import droneData
from ServiceRequestors.wolfGetWolfData import getWolfState
import ServiceRequestors.overseerGetOverseerData as overseerGetOverseerData
from ServiceRequestors.overseerGetWolfData import getWolfDataOfCluster
import ServiceRequestors.wolfGetWolfData as wolfService

AVOID_FACTOR = 0.01
DIRECTION_FACTOR = 5
WOLF_SLOW_DOWN = 3
OVERSEER_DIRECTION_SPEED_UP = 16
OVERSEER_DIRECTION_SLOW_DOWN = 5
OVERSEER_TO_WOLF_GROUP_RADIUS = 0.0003
REPULSION_RADIUS = 0.0003
Cluster = ""

def repulsion(client, curDroneIndex):
    wolfInfoArray = getWolfState()        # Get droneWolfState state array from service

    # Variables used for separation for drones too close
    close_dx = 0
    close_dy = 0
    finalVX = 0
    finalVY = 0

    wolfDroneCount = len(wolfInfoArray)

    for nearbyDroneIndex in range(wolfDroneCount):
        # Get difference in location from drones
        xDifference = wolfInfoArray[curDroneIndex].longitude - wolfInfoArray[nearbyDroneIndex].longitude
        yDifference = wolfInfoArray[curDroneIndex].latitude - wolfInfoArray[nearbyDroneIndex].latitude

        # Check if any nearby drones are too close using difference values
        if ((nearbyDroneIndex != curDroneIndex) and (abs(xDifference) < REPULSION_RADIUS) and (abs(yDifference) < REPULSION_RADIUS)):
            close_dx += xDifference
            close_dy += yDifference

    # Account for updated repulsion equation, if closer have more weight, if father have less weight
    if ((close_dx != 0) and (close_dy != 0)):
        close_dx = ((REPULSION_RADIUS/abs(close_dx)) - 1) * ((abs(close_dx))/close_dx)
        close_dy = ((REPULSION_RADIUS/abs(close_dy)) - 1) * ((abs(close_dy))/close_dy)

    # Adds in repulsion factor to final vectors
    finalVX += close_dx*AVOID_FACTOR
    finalVY += close_dy*AVOID_FACTOR
    return [finalVX, finalVY]

# Creates directional vector towards waypoint
def waypointDirection(client, curDroneIndex, waypoint):
    # Gets data from all drones
    wolfInfoArray = getWolfState()

    # Gets x and y difference between drone and waypoint
    xDifference = float(waypoint[0]) - wolfInfoArray[curDroneIndex].longitude
    yDifference = float(waypoint[1]) - wolfInfoArray[curDroneIndex].latitude

    distance = sqrt(xDifference**2 + yDifference**2)

    # If within certain distance of waypoint, don't move
    if ((abs(xDifference) < 0.00005) and (abs(yDifference) < 0.00005)):

        finalVelocity = [0, 0]
        return finalVelocity, True

    # Slow down when close
    elif (distance < 0.0001):
        # Gets normalized difference values and adds in directional factor
        xNormalized = (xDifference / sqrt(xDifference**2 + yDifference**2))*WOLF_SLOW_DOWN
        yNormalized = (yDifference / sqrt(xDifference**2 + yDifference**2))*WOLF_SLOW_DOWN

        # Saves final weighted vector to final velocity
        finalVelocity = [xNormalized, yNormalized]

    # Else move to waypoint
    else:
        # Gets normalized difference values and adds in directional factor
        xNormalized = (xDifference / sqrt(xDifference**2 + yDifference**2))*DIRECTION_FACTOR
        yNormalized = (yDifference / sqrt(xDifference**2 + yDifference**2))*DIRECTION_FACTOR

        # Saves final weighted vector to final velocity
        finalVelocity = [xNormalized, yNormalized]

    # return false for not being at waypoint
    return finalVelocity, False

# Uses repulsion and waypoint direction to move between waypoints
def lineBehavior(client, curDroneIndex, waypoint_coords):
    # Gets current wolf data
    wolfInfoArray = getWolfState()

    # If there currently is not droneData
    if (wolfInfoArray[0].droneName == ""):
        print("No drone data")
        return [0, 0]

    # Gets repulsion and direction vectors and adds them up
    velocityR = repulsion(client, curDroneIndex)
    velocityD, atCurrentWaypoint = waypointDirection(client, curDroneIndex, waypoint_coords)
    finalVelocityX = velocityD[0] + velocityR[0]
    finalVelocityY = velocityD[1] + velocityR[1]
    vector = [finalVelocityX, finalVelocityY]

    # Returns false because drone has not made it to waypoint
    return vector, atCurrentWaypoint

def subWaypointCalculator(currentWaypoint, previousWaypoint, radius, droneName, droneCluster):
    global Cluster
    Cluster = droneCluster
    # 0 is longitude, 1 is latitude
    newWaypoint = []

    # Finds vector between waypoints
    waypointDiffX = float(currentWaypoint[0]) - float(previousWaypoint[0])
    waypointDiffY = float(currentWaypoint[1]) - float(previousWaypoint[1])

    # Gets normalized difference vector
    vectorVal = sqrt(waypointDiffX**2 + waypointDiffY**2)
    xDirection = (waypointDiffX/vectorVal) * radius
    yDirection = (waypointDiffY/vectorVal) * radius

    # Calculates horizontal and vertical changes
    horizonalChange = xDirection - yDirection
    verticalChange = xDirection + yDirection

    # Get cluster and size information
    wolfCluster = wolfService.getWolfDataOfClusterWCurWolf(Cluster)
    clusterSize = len(wolfCluster)

    # LANES       3  1  0  2  4
    # DISTANCE    2  1  0  1  2
    subwaypointList = []

    # Calculates list of subwaypoint
    # Append middle waypoint
    subwaypointList.append(currentWaypoint)

    distanceFromCenterLeft = 1
    distanceFromCenterRight = 1
    for lane in range(1, clusterSize):

        # Odd go to the left of center
        if (lane % 2 == 1):
            newWaypointX = float(currentWaypoint[0]) + (horizonalChange * distanceFromCenterLeft)
            newWaypointY = float(currentWaypoint[1]) + (verticalChange * distanceFromCenterLeft)
            distanceFromCenterLeft = distanceFromCenterLeft + 1
            newWaypoint = [newWaypointX, newWaypointY]
            subwaypointList.append(newWaypoint)
        # Even go to the right of center
        elif (lane % 2 == 0):
            newWaypointX = float(currentWaypoint[0]) - (horizonalChange * distanceFromCenterRight)
            newWaypointY = float(currentWaypoint[1]) - (verticalChange * distanceFromCenterRight)
            distanceFromCenterRight = distanceFromCenterRight + 1
            newWaypoint = [newWaypointX, newWaypointY]
            subwaypointList.append(newWaypoint)

    # Gets lane and subwaypoint from calculated list
    lane = int(droneName) % clusterSize
    newWaypoint = subwaypointList[lane]

    return newWaypoint

def getSubwaypointList(currentWaypoint, previousWaypoint, radius, droneCluster):
    global Cluster
    Cluster = droneCluster
    # 0 is longitude, 1 is latitude

    # Finds vector between waypoints
    waypointDiffX = float(currentWaypoint[0]) - float(previousWaypoint[0])
    waypointDiffY = float(currentWaypoint[1]) - float(previousWaypoint[1])

    # Gets normalized difference vector
    vectorVal = sqrt(waypointDiffX**2 + waypointDiffY**2)
    xDirection = (waypointDiffX/vectorVal) * radius
    yDirection = (waypointDiffY/vectorVal) * radius

    # Calculates horizontal and vertical changes
    horizonalChange = xDirection - yDirection
    verticalChange = xDirection + yDirection

    # Get cluster and size information
    wolfCluster = wolfService.getWolfDataOfClusterWCurWolf(Cluster)
    clusterSize = len(wolfCluster)

    # LANES       3  1  0  2  4
    # DISTANCE    2  1  0  1  2
    subwaypointList = []

    # Calculates list of subwaypoint
    # Append middle waypoint
    subwaypointList.append(currentWaypoint)

    # Distance from center, example above subwaypoint list
    distanceFromCenterLeft = 1
    distanceFromCenterRight = 1
    for lane in range(1, clusterSize):

        # Odd go to the left of center
        if (lane % 2 == 1):
            newWaypointX = float(currentWaypoint[0]) + (horizonalChange * distanceFromCenterLeft)
            newWaypointY = float(currentWaypoint[1]) + (verticalChange * distanceFromCenterLeft)
            distanceFromCenterLeft = distanceFromCenterLeft + 1
            newWaypoint = [newWaypointX, newWaypointY]
            subwaypointList.append(newWaypoint)
        # Even go to the right of center
        elif (lane % 2 == 0):
            newWaypointX = float(currentWaypoint[0]) - (horizonalChange * distanceFromCenterRight)
            newWaypointY = float(currentWaypoint[1]) - (verticalChange * distanceFromCenterRight)
            distanceFromCenterRight = distanceFromCenterRight + 1
            newWaypoint = [newWaypointX, newWaypointY]
            subwaypointList.append(newWaypoint)

    # Gets lane and subwaypoint from calculated list
    # str(subwaypointList)
    # debugPrint(subwaypointList)
    return subwaypointList