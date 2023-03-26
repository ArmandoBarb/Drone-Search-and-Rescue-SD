import airsim
from math import sqrt
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from airsim_ros_pkgs.msg import droneData
from ServiceRequestors.wolfGetWolfData import getWolfState
import ServiceRequestors.overseerGetOverseerData as overseerGetOverseerData
from ServiceRequestors.overseerGetWolfData import getWolfDataOfCluster
import ServiceRequestors.wolfGetWolfData as wolfService
from math import sqrt
import airsim

AVOID_FACTOR = 0.01
DIRECTION_FACTOR = 5
WOLF_SLOW_DOWN = 3
OVERSEER_DIRECTION_SPEED_UP = 16
OVERSEER_DIRECTION_SLOW_DOWN = 5
OVERSEER_TO_WOLF_GROUP_RADIUS = 0.0003
REPULSION_RADIUS = 0.0003

# Creates directional vector towards waypoint
def overseerWaypoint(client, curDroneIndex, waypoint):
    # Gets data from all drones
    overseerInfoArray = overseerGetOverseerData.getOverseerState()

    # Gets x and y difference between drone and waypoint
    xDifference = float(waypoint[0]) - overseerInfoArray[curDroneIndex].longitude
    yDifference = float(waypoint[1]) - overseerInfoArray[curDroneIndex].latitude

    # Get wolf data of cluster
    clusterName = "Overseer_" + str(curDroneIndex)
    cluster = getWolfDataOfCluster(clusterName)


    # If within certain distance of waypoint, don't move
    if ((abs(xDifference) < 0.00005) and (abs(yDifference) < 0.00005) or (0 == len(cluster))):
        finalVelocity = [0, 0]

    # Else move to waypoint
    else:
        directionFactor = DIRECTION_FACTOR


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

        elif (distance > OVERSEER_TO_WOLF_GROUP_RADIUS * 1.5):
            directionFactor = 0

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