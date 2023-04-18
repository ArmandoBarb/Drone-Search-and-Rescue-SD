# Overseer drone node
# Subscribes to the (Command)
# Publishes to (OverseerDroneData)
# TODO: Subscribes to (Command) topic
# TODO: Publish to (CommandResult) topic (may be removed as not critical)
# TODO: Add method for Overseer to cordinate
# Uses services for positions on (OverseerStateAPI) and (WolfStateAPI)

import setup_path # If setup_path gives issue, comment out
import airsim
import rospy
import time
import json
import ast
import math
from math import sqrt
# import constants
import Constants.configDrones as configDrones
import Constants.ros as ros
# TODO: Investigate if we need to use a Lock while writing or reading global variables
from threading import Timer # Use for interval checks with minimal code
from threading import Thread # USe for important code running constantly
# TODO: Add custom message types
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
# Get service calls here
from ServiceRequestors.instructWolf import sendLinebehaviorRequest
from ServiceRequestors.overseerGetOverseerData import getOverseerState
from ServiceRequestors.overseerGetWolfData import getOverseerGetWolfState
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.msg import wolfCommunication
from airsim_ros_pkgs.msg import updateMap
from ServiceRequestors.wolfGetWolfData import getWolfState
import ServiceRequestors.overseerGetWolfData as overseerGetWolfData 
import ServiceRequestors.instructWolf as instructWolf
import DroneBehaviors.lineBehaviorOverseer as lineBehaviorOverseer
from airsim_ros_pkgs.msg import GPS
# ros publish helfper
import RosPublishHelper.MapHandlerPublishHelper as mapHandlerPublishHelper


from ImageProcessing import getInfo
from ImageProcessing import clustering
from HelperFunctions import clusterHelper
from HelperFunctions import algoHelper
from HelperFunctions import calcHelper
import ServiceRequestors.checkGPU as checkGPU
import warnings
import os


# for a clearner output
warnings.filterwarnings("ignore", category=DeprecationWarning)

# Environmental Variables
LOOP_NUMBER = configDrones.LOOP_NUMBER
LOCAL_IP = configDrones.LOCAL_IP
MAX_TIME = configDrones.MAX_TIME
MIN_CIRCLE_RADIUS_GPS = configDrones.MIN_CIRCLE_RADIUS_GPS 
MIN_CIRCLE_RADIUS_METERS = configDrones.MIN_CIRCLE_RADIUS_METERS
DISTANCE_LEAD_OVERSEER_GPS = configDrones.DISTANCE_LEAD_OVERSEER_GPS
MIN_CIRCLE_PADDING_FOR_SEARCH_HISTORY = configDrones.MIN_CIRCLE_PADDING_FOR_SEARCH_HISTORY
MAX_WAYPOINT_SAVE_TIME = configDrones.MAX_WAYPOINT_SAVE_TIME

# ros: topics
OVERSEER_DATA_TOPIC = ros.OVERSEER_DATA_TOPIC
OVERSEER_COMMUNICATION_TOPIC = ros.OVERSEER_COMMUNICATION_TOPIC
COMMAND_TOPIC = ros.COMMAND_TOPIC
COMMAND_RESULT_TOPIC = ros.COMMAND_RESULT_TOPIC
MAP_HANDLER_TOPIC = ros.MAP_HANDLER_TOPIC

# ros: services
PROXIMITY_OVERSEER_SERVICE = ros.PROXIMITY_OVERSEER_SERVICE
# Dynamic service append number
WOLF_DRONE_SERVICE = ros.WOLF_DRONE_SERVICE
SEARCH_TASK_GROUP = ros.SEARCH_TASK_GROUP
AT_SPIRAL_WAYPOINT_SIGNAL = ros.AT_SPIRAL_WAYPOINT_SIGNAL
EMPTY_CLUSTER = ros.EMPTY_CLUSTER
EMPTY_TASK_GROUP = ros.EMPTY_TASK_GROUP

# Internal Wolf Drone Memory Start -------------------------------------------
# Current pattern is ussing Global variable to allow access across threads (open to change)
# Global variables
DM_Drone_Name = None
DM_Wolfs_Cluster = []
WAYPOINT_COORDS = []
WAYPOINT_INDEX = 0
GROUP_0_SEARCH = 'Constants/Group0Spiral.txt'
GROUP_1_SEARCH = 'Constants/Group1Spiral.txt'
Cluster = EMPTY_CLUSTER
End_Loop = False
Waypoint_History = []

# Main Process Start ----------------------------------------------
# Main function for the overseer drone
def overseerDroneController(droneName, overseerCount, wolfCount):
    global DM_Drone_Name
    global Cluster
    DM_Drone_Name = droneName
    Cluster = droneName
    # use this code to make print calls allowing you to know what process made the print statemnt
    debugPrint("Process started")
    droneNum = ''.join(filter(lambda i: i.isdigit(), droneName)) # Get overseer number from droneName

    # Create Node for Overseer
    nodeName = droneName
    rospy.init_node(nodeName, anonymous = True)

    # Wait until GPU is loaded
    checkGPU.checkGPUStatus()
    debugPrint("GPU Loaded")

    # Get wolf cluster

    # Reads in coords for drone
    if (droneName == "Overseer_0"):
        readCoordFile(GROUP_0_SEARCH)
    else:
        readCoordFile(GROUP_1_SEARCH)

    # Start all threads here (if you have to make one somwhere else bring it up with the team)
    t = Thread(target = overseerCommunicationSubscriber, args=())
    t.start()

    # Create topic publishers
    # (TODO: ADD IN COMMAND RESULT PUBLISHERS)
    overseerDataPublish = rospy.Publisher(OVERSEER_DATA_TOPIC, droneData, latch=True, queue_size=1)
    overseerCommunicationPublish = rospy.Publisher(OVERSEER_COMMUNICATION_TOPIC, String, latch=True, queue_size=1)

    # Sets client and takes off drone
    client = takeOff(droneName)
    client.moveToZAsync(z=-40, velocity=8, vehicle_name = droneName).join()

    # client = takeOff("TestOverseer")
    # client.moveToZAsync(z=-10, velocity=8, vehicle_name = "TestOverseer").join()

    # thread for infared waypoint detection
    t1 = Thread(target = overseerInfraredDetection, args=({droneName}))
    t1.start()

    # Call startup service on each wolf
    clusterSize = math.floor(wolfCount / overseerCount)
    groupStartDroneNum = int(droneNum) * clusterSize  # Get number of first wolf in cluster
    
    if (int(droneNum) == (overseerCount - 1)):
        reminder = wolfCount % overseerCount
        if reminder != 0:
            clusterSize = clusterSize + reminder
    debugPrint(str(clusterSize))
    for num in range(clusterSize):
        wolfNum = num + groupStartDroneNum
        wolfDroneService = WOLF_DRONE_SERVICE + str(wolfNum)
        sendLinebehaviorRequest(wolfDroneService, droneName)

    # Overseer Drone search loop Start
    i = 0
    debugPrint("Starting Search and Rescue loop")
    timeSpent = 0
    runtime = time.time() # USED FOR TESTING
    while (i < LOOP_NUMBER):
        if (End_Loop):
            print(droneName, "Ending loop")
            break
        timeDiff = time.time() - runtime
        start=time.time() # gather time data
        if (timeDiff > MAX_TIME):
            break
        # Checks if made it through all waypoints
        if (WAYPOINT_INDEX == (len(WAYPOINT_COORDS) - 1)):
            break;
            # print(droneName, "Made it to end of waypoint spiral search")

        # Publishes to (OverseerData) topic
        overseerDataPublisher(overseerDataPublish, client, droneName)
        # Publishes to (OverseerCommunication) topic
        # TODO: cordinate drone clustering
        overseerCommunicationPublisher(overseerCommunicationPublish, client, droneName)
        # TODO: Update assigned wolf drones on search area

        # Gets waypoint and calculates movement vector to next waypoint
        endWaypoint = getNewWaypoint(droneName)
        startWaypoint = getLastWaypoint(droneName)
        startGPS = calcHelper.fixDegenerateCoordinate(startWaypoint)
        endGPS = calcHelper.fixDegenerateCoordinate(endWaypoint)
        # debugPrint("Start Longitude: " + str(startGPS.longitude) + "Start Latitude: " + str(startGPS.latitude) + "End Longitude: "+ str(endGPS.longitude) + "End Latitude: " + str(endGPS.latitude))
        isEmpty, clusterCenterGPS = overseerGetWolfData.getWolfClusterCenterGPS(droneName)

        waypoint = [0, 0]
        if(isEmpty):
            waypoint = endWaypoint
        else:
            # print("We in else")
            GPSOnLine = calcHelper.mapGPSPointOnLine(startGPS, endGPS, clusterCenterGPS)
            # debugPrint("GPSOnLine to add: " + str(GPSOnLine))
            # print("We in else")
            dVector = calcHelper.calcVectorBetweenGPS(GPSOnLine, endGPS)
            # todo  DEAL WITH calcVectorBetweenGPS
            dVector = [dVector[1], dVector[0]]
            # debugPrint("dVector to add: " + str(dVector))
            
            vectorAdd = calcHelper.setVectorMagnitude(dVector, DISTANCE_LEAD_OVERSEER_GPS)
            # debugPrint("Vector to add: " + str(vectorAdd))
            # debugPrint("Longitude: " + str(GPSOnLine.longitude) + "Latitude: " + str(GPSOnLine.latitude))

            waypoint2 = [GPSOnLine.longitude + vectorAdd[0], GPSOnLine.latitude + vectorAdd[1]]
            # waypoint2 = [GPSOnLine.longitude + 0, GPSOnLine.latitude + 0]

            outputForWaypoint = "Dynamic Waypoint " + str(waypoint2) + " Waypoint: " + str(endWaypoint)
            # debugPrint(outputForWaypoint)
            waypoint = waypoint2
            # waypoint = GPSOnLine


        outputForWaypoint = "Waypoint to move to: " + str(waypoint) + " END GPS: " + str(endGPS)
        # debugPrint(outputForWaypoint)
        vector = lineBehaviorOverseer.overseerWaypoint(client, int(droneNum), waypoint, endWaypoint)

        client.moveByVelocityZAsync(vector[1], vector[0], -40, duration = 1, vehicle_name=droneName)

        # client.moveByVelocityZAsync(vector[1], vector[0], -10, duration = 1, vehicle_name="TestOverseer")

        # If all drones make it to the waypoint, more to next waypoint
        allDronesAtWaypoint(droneName)

        # TODO: Add in Overseer behavior
        # TODO: Creeping Line lead behavior
            # Overseer will slighly go ahead its drone cluster to search for waypoints

        # TODO: Make Airsim call with desired action

        # Add in artifical loop delay (How fast the loop runs dictates the drones reaction speed)
        # time.sleep(0.5)
        i+=1
        end = time.time()
        timeSpent += end-start
    debugPrint("Average Loop Time: " + str(timeSpent / i))
    # Overseer Drone search loop End
# Main Process End ----------------------------------------------

# Theads Start ===========================================
# Subscribes to (Command) topic
def commandSub():
    rospy.Subscriber(COMMAND_TOPIC, String, handleCommand, (overseerCount, client))
    rospy.spin()

# TODO: overseer communication listen
def overseerCommunicationSubscriber():
    rospy.Subscriber(OVERSEER_COMMUNICATION_TOPIC, String, handleOverseerCommunication, ())
    rospy.Subscriber(ros.END_LOOP_TOPIC, String, handleEnd)
    rospy.Subscriber(ros.WOLF_COMMUNICATION_TOPIC, wolfCommunication, handleWolfCommunication)
    rospy.spin()

def handleWolfCommunication(data):
    # Grabs strings from data object
    global DM_Drone_Name
    cluster = data.cluster
    taskGroup = data.taskGroup
    command = data.command
    spiralIndex = data.genericInt
    global WAYPOINT_INDEX
    #debugPrint("overseer listend to wolf comm")
    # Check if we got at spiral waypoint signal
    if ((command == AT_SPIRAL_WAYPOINT_SIGNAL) and (cluster == DM_Drone_Name)):
        # If our current waypoint index is less than the one we received, use the most up to data spiral index
        # text = "Overseer recieved current waypoint: " + str(spiralIndex) + "Cluster: " + DM_Drone_Name
        # debugPrint(text)
        if(WAYPOINT_INDEX < spiralIndex):
            # text = "Current index out of data, setting to recieved waypoint: " + str(spiralIndex)
            # debugPrint("Current index out of data, setting to recieved waypoint")
            WAYPOINT_INDEX = spiralIndex

def overseerInfraredDetection(droneName):
    global Cluster
    threadClient = airsim.MultirotorClient(LOCAL_IP)
    debugPrint("Starting overseerCameraDetection loop")
    i = 0
    timeSpent = 0

    wolfMapPublisher = rospy.Publisher(MAP_HANDLER_TOPIC, updateMap, latch=True, queue_size=100)

    runtime = time.time()
    # print("Our current drone name is: ", droneName)
    while (i < LOOP_NUMBER):
        timeDiff = time.time() - runtime
        if (timeDiff > MAX_TIME):
            return

        # If we receive end command, end the loop
        if (End_Loop):
            exit()
            debugPrint("Ending loop")
            return;
          
        start=time.time() # gather time data

        #---Waypoint Detection---
        # # get response object and retrieve segmentation
        responses = getInfo.getInfrared(threadClient, droneName)
        currentGPS = threadClient.getGpsData(gps_name = "", vehicle_name = droneName)

        height, width, segRGB = getInfo.getSegInfo(responses)

        # cluster heat signatures from segmenation map
        clusters = clustering.pixelClustering(height, width, segRGB)

        # print("Cluster len:", len(clusters))
        
        # if no detections then avoid unecessary calculations
        if len(clusters) > 0:
            # debugPrint("Got a detection!")
            # get centroids of each pixel cluster
            # this info will be in longitude and latitude form
            centroidsGPS = getInfo.getCentroids(clusters, threadClient, droneName, height, width)

            # filter centroids against past waypoints
            filteredCentroidsGPS = []
            for centroid in centroidsGPS:
                if isValidCentroid(centroid):
                    filteredCentroidsGPS.append(centroid)

            # generate search circles
            # contains [radius, avg circle center, list of circle centers]
            radius = MIN_CIRCLE_RADIUS_GPS
            circleList = []
            centroidsGPS = filteredCentroidsGPS
            for centroid in centroidsGPS:
                circle = clustering.circle(radius, centroid, [centroid])
                circleList = clustering.addCircle(circle, circleList)

            wolfDataList = overseerGetWolfData.getWolfDataOfCluster(Cluster)
            cleanWaypointHistory(wolfDataList)
            # for circle in circleList:
            #     print(str(i) + ": " + "center, " + str(circle.avgCenter) + "radius, " + str(circle.radius))


            for x in range(len(circleList)):
                waypoint = circleList[x].avgCenter
                radius = circleList[x].radius

                mapHandlerPublishHelper.updateOverseerDronePrediction(wolfMapPublisher=wolfMapPublisher, imageNumber=x, currentGPS=currentGPS.gnss.geo_point, targetLat=waypoint[1], targetLon=waypoint[0])

                # see whose available
                optimalDroneName = algoHelper.getOptimalWolf(waypoint, wolfDataList, droneName)

                # Sends if we have a valid waypoint and have an optimal drone
                if (optimalDroneName != ""):
                    # add new waypoint to history
                    taskGroup = SEARCH_TASK_GROUP + optimalDroneName
                    updateWayPointHistory(waypoint, taskGroup, radius)

                    gpsDataObject = GPS()
                    gpsDataObject.longitude = waypoint[0]
                    gpsDataObject.latitude = waypoint[1]

                    # check who gets assigned to what
                    print("Wolf " + optimalDroneName + " assigned " + "(" + str(waypoint[0]) + ", " + str(waypoint[1]) + ")")

                    # SEND MESSAGE TO OPTIMAL DRONE
                    serviceName = WOLF_DRONE_SERVICE + str(optimalDroneName)
                    circleCenterGPS =  gpsDataObject
                    circleRadiusGPS = circleList[x].radius
                    circleRadiusMeters = (circleList[x].radius*MIN_CIRCLE_RADIUS_METERS)/MIN_CIRCLE_RADIUS_GPS
                    spreadTimeS = 30
                    searchTimeS = (circleList[x].radius*15)/MIN_CIRCLE_RADIUS_GPS
                    taskGroup = EMPTY_CLUSTER  # Let's wolf know it comes from an overseer

                    # IF TASK GROUP IS EMPTY, THE REQUEST IS FROM THE OVERSEER
                    # IF HAS NAME, IS FROM WOLF
                    overseerLocation = threadClient.getGpsData(gps_name = "", vehicle_name = droneName)
                    calcDistanceBetweenGPS = calcHelper.calcDistanceBetweenGPS(overseerLocation.gnss.geo_point, gpsDataObject)

                    # print("Overseerr to GPS Difference:", calcDistanceBetweenGPS)
                    requestStatus = instructWolf.sendWolfSearchBehaviorRequest(serviceName, circleCenterGPS, circleRadiusGPS, circleRadiusMeters, spreadTimeS, searchTimeS,  taskGroup)
                    print("Request bool:", requestStatus, "From Overseer:", droneName, "To:", optimalDroneName)
        else:
            mapHandlerPublishHelper.updateOverseerDronePosition(wolfMapPublisher=wolfMapPublisher, currentGPS=currentGPS.gnss.geo_point)

        time.sleep(0.5)
        end = time.time()
        timeSpent += end-start
        i+=1
    debugPrint(" CameraDetection: Average Loop Time: " + str(timeSpent / i))
    return
# Theads END ===========================================

# DON'T SPLIT THE NEXT THREE FUNCTIONS UP!!! :(
def updateWayPointHistory(waypoint, taskGroup, radius):
    global Waypoint_History
    Waypoint_History.append([waypoint, taskGroup, radius, time.time()])

def cleanWaypointHistory(wolfDroneDataList):
    global Waypoint_History
    filteredWaypointHistory = []
    for waypoint in Waypoint_History:
        timeDiff = time.time() - waypoint[3]
        if (timeDiff < MAX_WAYPOINT_SAVE_TIME):
            filteredWaypointHistory.append(waypoint)
        # for wolf in wolfDroneDataList:
        #     if wolf.taskGroup == waypoint[1]:
        #         filteredWaypointHistory.append(waypoint)
    Waypoint_History = filteredWaypointHistory

def isValidCentroid(centroid):
    global Waypoint_History
    for waypoint in Waypoint_History:
        distance = sqrt((float(waypoint[0][0]) - float(centroid[0]))**2 + (float(waypoint[0][1]) - float(centroid[1]))**2)
        radius = MIN_CIRCLE_PADDING_FOR_SEARCH_HISTORY + waypoint[2]
        if (distance < radius):             
            return False
    return True

def handleEnd(data):
    global End_Loop
    if (data.data == "e"):
        End_Loop = True

# Takes in strings from the (Command) topic for processing
def handleCommand(data, args):
    print()
    # TODO: ADD IN CODE TO HANDLE COMMAND RESULT TOPIC

# Sets up publisher and calls function for (OverseerDroneData)
def overseerCommunicationPublisher(pub, client, droneName):
    message = "Hello, I'm drone " + droneName
    pub.publish(message) # publish lcoation

# TODO: overseer communication listen
def handleOverseerCommunication(data, args):
    message = str(data.data)
    # print(message)

# Sets up publisher and calls function for (OverseerDroneData)
def overseerDataPublisher(pub, client, droneName):
    position = client.getMultirotorState(vehicle_name = droneName)
    velocity = client.getGpsData(vehicle_name = droneName)
    global Cluster

    # Creates droneMsg object and inserts values from AirSim apis
    droneMsg = droneData()
    droneMsg.droneName = droneName
    droneMsg.longitude = position.gps_location.longitude
    droneMsg.latitude = position.gps_location.latitude
    droneMsg.velocityX = velocity.gnss.velocity.x_val
    droneMsg.velocityY = velocity.gnss.velocity.y_val
    droneMsg.cluster = droneName

    # Publishes to topic
    pub.publish(droneMsg)

# Publishes data to OverseerDroneData topic
def updateDroneData(pub, client):
    # TODO: UPDATE STATE FUNCTIONALITY
    stateData = "True" # Temporary state data for testing
    pub.publish(jsonLocation)

# Enables api control, takes off drone, returns the client
def takeOff(droneName):
    client = airsim.MultirotorClient(LOCAL_IP)
    debugPrint("Checking if connected to MulirotorClient")
    client.confirmConnection()
    debugPrint("Connected to MulirotorClient")
    client.enableApiControl(True, droneName)
    client.armDisarm(True, droneName)
    client.takeoffAsync(vehicle_name=droneName).join()

    return client

def debugPrint (debugMessage):
    global DM_Drone_Name
    print("Overseer: ", DM_Drone_Name, " : " ,  debugMessage)

def getNewWaypoint(droneName):
    # Created global waypoints
    global WAYPOINT_INDEX
    # print("DroneName: ", droneName, "Current waypoint index", WAYPOINT_INDEX)
    currentWaypoint = WAYPOINT_COORDS[WAYPOINT_INDEX]

    return currentWaypoint

def getLastWaypoint(droneName):
    # Created global waypoints
    global WAYPOINT_INDEX
    # print("DroneName: ", droneName, "Current waypoint index", WAYPOINT_INDEX)
    currentWaypoint = WAYPOINT_COORDS[WAYPOINT_INDEX - 1]

    return currentWaypoint

# Reads values in SpiralSearch.txt and sets it to global variable
def readCoordFile(filename):
    file = open(filename, 'r')
    f = file.readlines()
    i = 0

    # Creates an array for the coordinates and strips the newlines
    newList = []
    for line in f:
        newLine = line.strip()
        newLine = newLine.split(' ')
        newList.append(newLine)

    global WAYPOINT_COORDS
    WAYPOINT_COORDS = newList

def allDronesAtWaypoint(overseerName):
    global WAYPOINT_INDEX
    wolfClusterInfo = overseerGetWolfData.getWolfDataOfCluster(overseerName)
    for drone in wolfClusterInfo:
        xDifference = drone.longitude - float(WAYPOINT_COORDS[WAYPOINT_INDEX][0])
        yDifference = drone.latitude - float(WAYPOINT_COORDS[WAYPOINT_INDEX][1])

        # If any of the drones are out of bounds, return false
        if ((abs(xDifference) > 0.0003) or (abs(yDifference) > 0.0003)):
            # print(droneNum, "X difference:", xDifference, "Y Difference:", yDifference)
            return 0

    WAYPOINT_INDEX = WAYPOINT_INDEX + 1
    # print("Drones:", DM_Wolfs_Cluster, "Made it to waypoint:", WAYPOINT_INDEX)
    return 1