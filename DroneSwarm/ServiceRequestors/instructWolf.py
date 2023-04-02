
import rospy
import math
# custum message import
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.msg import requestLineBehavior
from airsim_ros_pkgs.msg import requestWolfSearchBehavior
from airsim_ros_pkgs.msg import requestConsensusDecisionBehavior
from airsim_ros_pkgs.msg import GPS
from airsim_ros_pkgs.srv import getDroneData
from airsim_ros_pkgs.srv import sendCommand
# helper imports
import HelperFunctions.calcHelper as calcHelper
import HelperFunctions.pathWaypoints as pathWaypoints
# service imports
import ServiceRequestors.wolfGetWolfData as wolfGetWolfData
# import constants
import Constants.configDrones as configDrones
import Constants.ros as ros
# dynamic services:
WOLF_DRONE_SERVICE = ros.WOLF_DRONE_SERVICE
EMPTY_TASK_GROUP = ros.EMPTY_TASK_GROUP
# config imports
WOLF_SEARCH_REQUEST_HELP_DISTANCE_MULTIPLE = configDrones.WOLF_SEARCH_REQUEST_HELP_DISTANCE_MULTIPLE
CONSENSUS_DECISION_REQUEST_HELP_DISTANCE_MULTIPLE = configDrones.CONSENSUS_DECISION_REQUEST_HELP_DISTANCE_MULTIPLE
MIN_CIRCLE_RADIUS_GPS = configDrones.MIN_CIRCLE_RADIUS_GPS 

# Send service to start live behavior with given waypoints
def sendLinebehaviorRequest(serviceName, clusterName):
    # Create messages needed with parameters
    messageType = "RequestLineBehavior"
    linebehaviorMsg = requestLineBehavior()
    linebehaviorMsg.cluster = clusterName
    # Add empty data for unused message
    wolfSearchBehaviorMsg = requestWolfSearchBehavior()
    consensusDecisionBehaviorMsg = requestConsensusDecisionBehavior()

    # Sends service
    print("requestLineBehavior from wolf: ", serviceName)
    rospy.wait_for_service(serviceName)
    response = rospy.ServiceProxy(serviceName, sendCommand)
    resp = response(messageType, linebehaviorMsg, wolfSearchBehaviorMsg, consensusDecisionBehaviorMsg)

    # Prints bool on whether behavior was passed or not
    print("Message status: ", resp, " wolf: ", serviceName)
    return response

def sendWolfSearchBehaviorRequest(serviceName, circleCenterGPS, circleRadiusGPS, circleRadiusMeters, spreadTimeS, searchTimeS,  taskGroup):
    # Create messages needed with parameters
    messageType = "RequestWolfSearch"
    wolfSearchBehaviorMsg = requestWolfSearchBehavior()
    wolfSearchBehaviorMsg.circleCenterGPS = circleCenterGPS
    # circleCenterGPS
    # wolfSearchBehaviorMsg.circleCenterGPS.latitude = circleCenterGPS.latitude
    # wolfSearchBehaviorMsg.circleCenterGPS.longitude = circleCenterGPS.longitude
    # wolfSearchBehaviorMsg.circleCenterGPS.altitude = circleCenterGPS.altitude
    # wolfSearchBehaviorMsg.circleCenterGPS.yaw = circleCenterGPS.yaw

    wolfSearchBehaviorMsg.circleRadiusGPS = circleRadiusGPS
    wolfSearchBehaviorMsg.circleRadiusMeters = circleRadiusMeters
    wolfSearchBehaviorMsg.spreadTimeS = spreadTimeS
    wolfSearchBehaviorMsg.searchTimeS = searchTimeS
    wolfSearchBehaviorMsg.taskGroup = taskGroup

    # Add empty data for unused message
    linebehaviorMsg = requestLineBehavior()
    consensusDecisionBehaviorMsg = requestConsensusDecisionBehavior()

    # Sends service
    print("requestWolfSearchBehavior from wolf: ", serviceName)
    rospy.wait_for_service(serviceName)
    response = rospy.ServiceProxy(serviceName, sendCommand)
    resp = response(messageType, linebehaviorMsg, wolfSearchBehaviorMsg, consensusDecisionBehaviorMsg)

    # Prints bool on whether behavior was passed or not
    print("Message status: ", resp, " wolf: ", serviceName)
    return response

def sendConsensusDecisionBehaviorRequest(serviceName, circleCenterGPS, circleRadiusGPS, circleRadiusMeters, searchTimeS, taskGroup):
    # Create messages needed with parameters
    messageType = "RequestConsensusDecision"
    consensusDecisionBehaviorMsg = requestConsensusDecisionBehavior()

    consensusDecisionBehaviorMsg.circleCenterGPS = circleCenterGPS
    # circleCenterGPS
    # consensusDecisionBehaviorMsg.circleCenterGPS.latitude = circleCenterGPS.latitude
    # consensusDecisionBehaviorMsg.circleCenterGPS.longitude = circleCenterGPS.longitude
    # consensusDecisionBehaviorMsg.circleCenterGPS.altitude = circleCenterGPS.altitude
    # consensusDecisionBehaviorMsg.circleCenterGPS.yaw = circleCenterGPS.yaw

    consensusDecisionBehaviorMsg.circleRadiusGPS = circleRadiusGPS
    consensusDecisionBehaviorMsg.circleRadiusMeters = circleRadiusMeters
    consensusDecisionBehaviorMsg.searchTimeS = searchTimeS
    consensusDecisionBehaviorMsg.taskGroup = taskGroup

    # Add empty data for unused message
    linebehaviorMsg = requestLineBehavior()
    wolfSearchBehaviorMsg = requestWolfSearchBehavior()

    # Sends service
    print("requestConsensusDecisionBehavior from wolf: ", serviceName)
    rospy.wait_for_service(serviceName)
    response = rospy.ServiceProxy(serviceName, sendCommand)
    resp = response(messageType, linebehaviorMsg, wolfSearchBehaviorMsg, consensusDecisionBehaviorMsg)

    # Prints bool on whether behavior was passed or not
    print("Message status: ", resp, " wolf: ", serviceName)


# Requests nearby drones to do search
def requestNearbyDronesWolfSearch(circleCenterGPS, circleRadiusGPS, circleRadiusMeters, spreadTimeS, searchTimeS, taskGroup, clusterName, dmDroneName, waypointIndex, waypointCoords):
    endWaypoint = pathWaypoints.getNewWaypointWolf(dmDroneName, waypointIndex, waypointCoords, clusterName)
    startWaypoint = pathWaypoints.getLastWaypointWolf(dmDroneName, waypointIndex, waypointCoords, clusterName)
    startGPS = calcHelper.fixDegenerateCoordinate(startWaypoint)
    endGPS = calcHelper.fixDegenerateCoordinate(endWaypoint)
    isEmpty, clusterCenterGPS = wolfGetWolfData.getWolfClusterCenterGPS(clusterName=clusterName)

    # Checks if there is no cluster center
    if (isEmpty):
        return

    AvgClusterGPSOnLine = calcHelper.mapGPSPointOnLine(startGPS, endGPS, clusterCenterGPS)
    
    
    wolfDataArray = wolfGetWolfData.getWolfDataOfCluster(wolfNameToExclude=dmDroneName, clusterName=clusterName)

    # Go through each drones and request to nearby drones in cluster
    assignedDroneNum = 1
    for drone in wolfDataArray:
        # limit drone assignment number
        assignedDroneNum
        maxDroneAssignment = math.ceil(circleRadiusGPS / MIN_CIRCLE_RADIUS_GPS) # round up
        if (assignedDroneNum >= maxDroneAssignment):
            break;

        droneGPSOnLine = calcHelper.mapGPSPointOnLine(startGPS, endGPS, drone)
        droneDistanceFromStart = calcHelper.calcDistanceBetweenGPS(startGPS, droneGPSOnLine)
        avgClusterDistanceFromStart = calcHelper.calcDistanceBetweenGPS(startGPS, AvgClusterGPSOnLine)

        distanceFromAverage = calcHelper.calcDistanceBetweenGPS(droneGPSOnLine, AvgClusterGPSOnLine)
        distance = calcHelper.calcDistanceBetweenGPS(circleCenterGPS, drone)

        if (droneDistanceFromStart > avgClusterDistanceFromStart):
            distance -= distanceFromAverage
        else:
            distance += distanceFromAverage
        
        minDistanceFromWaypoint = circleRadiusMeters * WOLF_SEARCH_REQUEST_HELP_DISTANCE_MULTIPLE
        # If a drone is within a certain radius of requestor 
        if ((distance < minDistanceFromWaypoint) and (drone.taskGroup == EMPTY_TASK_GROUP)):
            serviceName = WOLF_DRONE_SERVICE + drone.droneName
            requestStatus = sendWolfSearchBehaviorRequest(serviceName, circleCenterGPS, circleRadiusGPS, circleRadiusMeters, spreadTimeS, searchTimeS,  taskGroup)
            if(requestStatus):
                assignedDroneNum += 1

def requestNearbyDronesConsensusDecision(circleCenterGPS, circleRadiusGPS, circleRadiusMeters, searchTimeS, taskGroup, clusterName, dmDroneName, waypointIndex, waypointCoords):
    endWaypoint = pathWaypoints.getNewWaypointWolf(dmDroneName, waypointIndex, waypointCoords, clusterName)
    startWaypoint = pathWaypoints.getLastWaypointWolf(dmDroneName, waypointIndex, waypointCoords, clusterName)
    startGPS = calcHelper.fixDegenerateCoordinate(startWaypoint)
    endGPS = calcHelper.fixDegenerateCoordinate(endWaypoint)
    isEmpty, clusterCenterGPS = wolfGetWolfData.getWolfClusterCenterGPS(clusterName=clusterName)

    if (isEmpty):
        return
    
    AvgClusterGPSOnLine = calcHelper.mapGPSPointOnLine(startGPS, endGPS, clusterCenterGPS)
    
    
    wolfDataArray = wolfGetWolfData.getWolfDataOfCluster(wolfNameToExclude=dmDroneName, clusterName=clusterName)

    # Go through each drones and request to nearby drones in cluster
    
    for drone in wolfDataArray:
        droneGPSOnLine = calcHelper.mapGPSPointOnLine(startGPS, endGPS, drone)
        droneDistanceFromStart = calcHelper.calcDistanceBetweenGPS(startGPS, droneGPSOnLine)
        avgClusterDistanceFromStart = calcHelper.calcDistanceBetweenGPS(startGPS, AvgClusterGPSOnLine)

        distanceFromAverage = calcHelper.calcDistanceBetweenGPS(droneGPSOnLine, AvgClusterGPSOnLine)
        distance = calcHelper.calcDistanceBetweenGPS(circleCenterGPS, drone)

        if (droneDistanceFromStart > avgClusterDistanceFromStart):
            distance -= distanceFromAverage
        else:
            distance += distanceFromAverage
        
        minDistanceFromWaypoint = circleRadiusMeters * CONSENSUS_DECISION_REQUEST_HELP_DISTANCE_MULTIPLE
        # If a drone is within a certain radius of requestor 
        if ((distance < minDistanceFromWaypoint)):
            serviceName = WOLF_DRONE_SERVICE + drone.droneName
            requestStatus = sendConsensusDecisionBehaviorRequest(serviceName, circleCenterGPS, circleRadiusGPS, circleRadiusMeters, searchTimeS, taskGroup)
            if(requestStatus):
                assignedDroneNum += 1