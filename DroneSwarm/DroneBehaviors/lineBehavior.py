import airsim
from math import sqrt
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from airsim_ros_pkgs.msg import droneData
from ServiceRequestors.wolfGetWolfData import getWolfState
from ServiceRequestors.overseerGetOverseerData import getOverseerState
from ServiceRequestors.overseerGetWolfData import getWolfDataOfCluster

AVOID_FACTOR = 0.01
DIRECTION_FACTOR = 9
OVERSEER_DIRECTION_SPEED_UP = 16
OVERSEER_DIRECTION_SLOW_DOWN = 5
OVERSEER_TO_WOLF_GROUP_RADIUS = 0.0003
REPULSION_RADIUS = 0.0003

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

    # If within certain distance of waypoint, don't move
    if ((abs(xDifference) < 0.0001) and (abs(yDifference) < 0.0001)):
        finalVelocity = [0, 0]

    # Else move to waypoint
    else:
        # Gets normalized difference values and adds in directional factor
        xNormalized = (xDifference / sqrt(xDifference**2 + yDifference**2))*DIRECTION_FACTOR
        yNormalized = (yDifference / sqrt(xDifference**2 + yDifference**2))*DIRECTION_FACTOR

        # Saves final weighted vector to final velocity
        finalVelocity = [xNormalized, yNormalized]

    return finalVelocity

# Uses repulsion and waypoint direction to move between waypoints
def lineBehavior(client, curDroneIndex, DM_Wolfs_Cluster, waypoint_coords):
    # Gets current wolf data
    wolfInfoArray = getWolfState()

    # If there currently is not droneData, return 0
    if (wolfInfoArray[0].droneName == ""):
        print("No drone data")
        return [0, 0]

    # Gets repulsion and direction vectors and adds them up
    velocityR = repulsion(client, curDroneIndex)
    velocityD = waypointDirection(client, curDroneIndex, waypoint_coords)
    finalVelocityX = velocityD[0] + velocityR[0]
    finalVelocityY = velocityD[1] + velocityR[1]
    vector = [finalVelocityX, finalVelocityY]

    return vector

# Creates directional vector towards waypoint
def overseerWaypoint(client, curDroneIndex, waypoint):
    # Gets data from all drones
    overseerInfoArray = getOverseerState()

    # Gets x and y difference between drone and waypoint
    xDifference = float(waypoint[0]) - overseerInfoArray[curDroneIndex].longitude
    yDifference = float(waypoint[1]) - overseerInfoArray[curDroneIndex].latitude

    # If within certain distance of waypoint, don't move
    if ((abs(xDifference) < 0.00005) and (abs(yDifference) < 0.00005)):
        finalVelocity = [0, 0]

    # Else move to waypoint
    else:
        directionFactor = DIRECTION_FACTOR

        # Get wolf data of cluster
        clusterName = "Overseer_" + str(curDroneIndex)
        cluster = getWolfDataOfCluster(clusterName)

        # Calculate average wolf drone cluster location
        averageLongitude = 0
        averageLatitude = 0
        for drone in cluster:
            averageLongitude += drone.longitude
            averageLatitude += drone.latitude
        averageLatitude = averageLatitude / len(cluster)
        averageLongitude = averageLongitude / len(cluster)

        # Calculate difference between overseer and cluster
        curLongitude = overseerInfoArray[curDroneIndex].longitude
        curLatitide = overseerInfoArray[curDroneIndex].latitude
        distance = sqrt( (curLongitude - averageLongitude)**2 + (curLatitide - averageLatitude)**2 )
        # print(clusterName, "distance from group", distance)

        # If wolf cluster distance is closer to next waypoint, speed up overseer
        overseerToWaypointDistance = sqrt( (xDifference)**2 + (yDifference)**2 )
        groupToWaypointDistance = sqrt( (float(waypoint[0]) - averageLongitude)**2 + (float(waypoint[1]) - averageLatitude)**2 )
        if (overseerToWaypointDistance > groupToWaypointDistance):
            directionFactor = OVERSEER_DIRECTION_SPEED_UP
            # print("OVERSEER", curDroneIndex, "Speeding up, wolfs average ahead")

        # If too close to group, speed up the overseer
        elif (distance < OVERSEER_TO_WOLF_GROUP_RADIUS):
            directionFactor = OVERSEER_DIRECTION_SPEED_UP
            # print("OVERSEER", curDroneIndex, "Speeding up")

        # If too far from group, slow down the overseer
        elif (distance > OVERSEER_TO_WOLF_GROUP_RADIUS):
            directionFactor = OVERSEER_DIRECTION_SLOW_DOWN
            # print("OVERSEER", curDroneIndex, "Slowing down")

        # Gets normalized difference values and adds in directional factor
        xNormalized = (xDifference / sqrt(xDifference**2 + yDifference**2))*directionFactor
        yNormalized = (yDifference / sqrt(xDifference**2 + yDifference**2))*directionFactor

        # Saves final weighted vector to final velocity
        finalVelocity = [xNormalized, yNormalized]

    return finalVelocity
