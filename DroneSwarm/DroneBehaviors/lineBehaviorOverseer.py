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
import Constants.configDrones as configDrones

AVOID_FACTOR = 0.01
DIRECTION_FACTOR = 5
WOLF_SLOW_DOWN = 3
OVERSEER_DIRECTION_SPEED_UP = 8
OVERSEER_DIRECTION_SLOW_DOWN = 2
OVERSEER_POINT_ON_LINE_FACTOR = 5
OVERSEER_TO_WOLF_GROUP_RADIUS = 0.0003
REPULSION_RADIUS = 0.0003
AT_WAYPOINT_RADIUS = 0.00015
MAX_DIRECTION_FACTOR = 10
COHESION_FACTOR = 0.00005
MIN_CIRCLE_RADIUS_GPS = configDrones.MIN_CIRCLE_RADIUS_GPS

# Directional vector towards next spiral waypoint
def waypointDirection(client, curDroneIndex, waypoint):
    # Gets data from overseers
    overseerInfoArray = overseerGetOverseerData.getOverseerState()

    # Gets x and y difference between drone and waypoint
    xDifference = float(waypoint[0]) - overseerInfoArray[curDroneIndex].longitude
    yDifference = float(waypoint[1]) - overseerInfoArray[curDroneIndex].latitude

    distance = sqrt(xDifference**2 + yDifference**2)

    # If within certain distance of waypoint, don't move
    if (distance < AT_WAYPOINT_RADIUS):

        finalVelocity = [0, 0]
        # print("At waypoint stopping")
        return finalVelocity

    # Slow down when close
    elif (distance < AT_WAYPOINT_RADIUS * 2):
        # Gets normalized difference values and adds in directional factor
        xNormalized = (xDifference / sqrt(xDifference**2 + yDifference**2))*OVERSEER_DIRECTION_SLOW_DOWN
        yNormalized = (yDifference / sqrt(xDifference**2 + yDifference**2))*OVERSEER_DIRECTION_SLOW_DOWN
        # print("Getting close slowing down")

        # Saves final weighted vector to final velocity
        finalVelocity = [xNormalized, yNormalized]

    # Else move to waypoint
    else:
        # Gets normalized difference values and adds in directional factor
        xNormalized = (xDifference / sqrt(xDifference**2 + yDifference**2))*OVERSEER_DIRECTION_SPEED_UP
        yNormalized = (yDifference / sqrt(xDifference**2 + yDifference**2))*OVERSEER_DIRECTION_SPEED_UP

        # Saves final weighted vector to final velocity
        finalVelocity = [xNormalized, yNormalized]

    return finalVelocity

# Pull towards center gps point
def cohesionPointOnLine(client, curDroneIndex, pointOnLine):
     # Gets data from overseers
    overseerInfoArray = overseerGetOverseerData.getOverseerState()

    # Gets x and y difference between drone and point on line
    xDifference = float(pointOnLine[0]) - overseerInfoArray[curDroneIndex].longitude
    yDifference = float(pointOnLine[1]) - overseerInfoArray[curDroneIndex].latitude

    # TODO: Add in difference between waypoint to get speed

    # Get distance between point on line and overseer
    distance = sqrt(xDifference**2 + yDifference**2)

    # if (distance > MIN_CIRCLE_RADIUS_GPS):
    #     print("Too far from waypoint. Distance is: ", distance)

    # Factor for movement based on distance to point
    directionFactor = distance / COHESION_FACTOR
    # print("Our direction factor for cohesion is: ", directionFactor)

    if (directionFactor > MAX_DIRECTION_FACTOR):
        directionFactor = MAX_DIRECTION_FACTOR

    # Gets normalized difference values and adds in directional factor
    xNormalized = (xDifference / sqrt(xDifference**2 + yDifference**2))*directionFactor
    yNormalized = (yDifference / sqrt(xDifference**2 + yDifference**2))*directionFactor

    # Saves final weighted vector to final velocity
    finalVelocity = [xNormalized, yNormalized]

    return finalVelocity


# Creates directional vector towards waypoint
def overseerWaypoint(client, curDroneIndex, pointOnLine, nextSpiralWaypoint):
    # Calculates waypoint and cohesion velocity
    velocityW = waypointDirection(client, curDroneIndex, nextSpiralWaypoint)
    velocityC = cohesionPointOnLine(client, curDroneIndex, pointOnLine)

    # Adds together velocity for waypoint and cohesion
    finalVelocityX = velocityW[0] + velocityC[0]
    finalVelocityY = velocityW[1] + velocityC[1]

    # Vector movement
    finalVelocity = [finalVelocityX, finalVelocityY]

    return finalVelocity

