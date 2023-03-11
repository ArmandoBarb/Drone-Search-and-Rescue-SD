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
from DroneBehaviors.lineBehavior import overseerWaypoint
from airsim_ros_pkgs.msg import droneData
from ServiceRequestors.wolfGetWolfData import getWolfState
import ServiceRequestors.overseerGetWolfData as overseerGetWolfData 
import ServiceRequestors.instructWolf as instructWolf
from airsim_ros_pkgs.msg import GPS
from ImageProcessing import getInfo
from ImageProcessing import clustering
from HelperFunctions import clusterHelper
from HelperFunctions import algoHelper
from HelperFunctions import calcHelper
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
WAYPOINT_HISTORY_DISTANCE_MULT = configDrones.WAYPOINT_HISTORY_DISTANCE_MULT
DISTANCE_LEAD_OVERSEER_GPS = configDrones.DISTANCE_LEAD_OVERSEER_GPS

# ros: topics
OVERSEER_DATA_TOPIC = ros.OVERSEER_DATA_TOPIC
OVERSEER_COMMUNICATION_TOPIC = ros.OVERSEER_COMMUNICATION_TOPIC
COMMAND_TOPIC = ros.COMMAND_TOPIC
COMMAND_RESULT_TOPIC = ros.COMMAND_RESULT_TOPIC

# ros: services
PROXIMITY_OVERSEER_SERVICE = ros.PROXIMITY_OVERSEER_SERVICE
# Dynamic service append number
WOLF_DRONE_SERVICE = ros.WOLF_DRONE_SERVICE
SEARCH_TASK_GROUP = ros.SEARCH_TASK_GROUP

# Internal Wolf Drone Memory Start -------------------------------------------
# Current pattern is ussing Global variable to allow access across threads (open to change)
# Global variables
DM_Drone_Name = None
DM_Wolfs_Cluster = []
WAYPOINT_COORDS = []
WAYPOINT_INDEX = 0
GROUP_0_SEARCH = 'Constants/Group0Spiral.txt'
GROUP_1_SEARCH = 'Constants/Group1Spiral.txt'
Cluster = ""
Task_Group = ""
End_Loop = False
Waypoint_History = []

# Main Process Start ----------------------------------------------
# Main function for the overseer drone
def overseerDroneController(droneName, droneCount):
    global DM_Drone_Name
    DM_Drone_Name = droneName

    # use this code to make print calls allowing you to know what process made the print statemnt
    debugPrint("Process started")
    droneNum = ''.join(filter(lambda i: i.isdigit(), droneName)) # Get overseer number from droneName

    # Create Node for Overseer
    nodeName = droneName
    rospy.init_node(nodeName, anonymous = True)

    # Get wolf cluster
    wolfClusterCreation(droneName)

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
    client.moveToZAsync(z=-35, velocity=8, vehicle_name = droneName).join()

    # thread for infared waypoint detection
    t1 = Thread(target = overseerInfraredDetection, args=({droneName}))
    t1.start()

    # Call startup service on each wolf
    # THis is Hardcoded need to replace
    droneLimit = int(droneNum) * 3
    for num in range(3):
        wolfNum = num + droneLimit
        wolfDroneService = WOLF_DRONE_SERVICE + str(wolfNum)
        # sendWolfCommandClusterInfo(wolfDroneService)
        sendLinebehaviorRequest(wolfDroneService, droneName)

    # Overseer Drone search loop Start
    i = 0
    debugPrint("Starting Search and Rescue loop")
    timeSpent = 0
    runtime = time.time() # USED FOR TESTING
    while (i < LOOP_NUMBER):
        if (End_Loop):
            print(droneName, "Ending loop")
            return
        timeDiff = time.time() - runtime
        start=time.time() # gather time data
        if (timeDiff > MAX_TIME):
            break
        # Checks if made it through all waypoints
        if (WAYPOINT_INDEX == (len(WAYPOINT_COORDS) - 1)):
            print(droneName, "Made it to end of waypoint spiral search")
        # Get Airsim Data and procesess it here
        # TODO: add infared image detector code here (if runtime is to long Seprate into thread that runs on intervals)
            # getDataFromAirsim -> imageProcessing ->
            # if Node detected calulate estimated node position ->
            # update internal drone state

        # TODO: run drone node assignment if needed and message wolf node

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

            waypoint2 = [GPSOnLine.longitude + vectorAdd[0], GPSOnLine.latitude + vectorAdd[1]]

            outputForWaypoint = "Dynamic Waypoint " + str(waypoint2) + " Waypoint: " + str(waypoint)
            # debugPrint(outputForWaypoint)
            waypoint = waypoint2


        outputForWaypoint = "Waypoint to move to: " + str(waypoint) + " END GPS: " + str(endGPS)
        # debugPrint(outputForWaypoint)
        vector = overseerWaypoint(client, int(droneNum), waypoint)


        # If all drones make it to the waypoint, more to next waypoint
        allDronesAtWaypoint()

        client.moveByVelocityZAsync(vector[1], vector[0], -35, duration = 1, vehicle_name=droneName)

        # TODO: Add in Overseer behavior
        # TODO: Creeping Line lead behavior
            # Overseer will slighly go ahead its drone cluster to search for waypoints

        # TODO: Make Airsim call with desired action

        # Add in artifical loop delay (How fast the loop runs dictates the drones reaction speed)
        time.sleep(0.5)
        i+=1
        end = time.time()
        timeSpent += end-start
    debugPrint("Average Loop Time: " + str(timeSpent / i))
    # Overseer Drone search loop End
# Main Process End ----------------------------------------------

# Theads Start ===========================================
# Subscribes to (Command) topic
def commandSub():
    rospy.Subscriber(COMMAND_TOPIC, String, handleCommand, (droneCount, client))
    rospy.spin()

# TODO: overseer communication listen
def overseerCommunicationSubscriber():
    rospy.Subscriber(OVERSEER_COMMUNICATION_TOPIC, String, handleOverseerCommunication, ())
    rospy.Subscriber(ros.END_LOOP_TOPIC, String, handleEnd)
    rospy.spin()

def overseerInfraredDetection(droneName):
    global Cluster
    threadClient = airsim.MultirotorClient(LOCAL_IP)
    debugPrint("Starting overseerCameraDetection loop")
    i = 0
    timeSpent = 0
    runtime = time.time()
    while (i < LOOP_NUMBER):
        timeDiff = time.time() - runtime
        if (timeDiff > MAX_TIME):
            break
        start=time.time() # gather time data

        #---Waypoint Detection---
        # # get response object and retrieve segmentation
        responses = getInfo.getInfrared(threadClient, droneName)
        height, width, segRGB = getInfo.getSegInfo(responses)

        dataDir='/home/testuser/AirSim/PythonClient/multirotor/Drone-Search-and-Rescue-SD/DroneSwarm/infraredDebug'
        # isExist=os.path.exists(dataDir)

        # if not isExist:
        #     # make directory if not already there
        #     os.makedirs(dataDir)
        #     print('Created: ' + dataDir)

        # j=0
        # while os.path.exists(dataDir + "/" + ('%s' % j)+"test.jpg"):
        #     print("Found "+('%s' % j))
        #     j+=1

        # # debug infrared
        # airsim.write_png(dataDir + '/' + str(j)+'test.jpg', segRGB)

        # cluster heat signatures from segmenation map
        clusters = clustering.pixelClustering(height, width, segRGB)

        # if no detections then avoid unecessary calculations
        if len(clusters) > 0:
            debugPrint("Got a detection!")
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
            for centroid in centroidsGPS:
                circle = clustering.circle(radius, centroid, [centroid])
                circleList = clustering.addCircle(circle, circleList)

            # calculate circle groups and get average centers per group
            # since the centroids were already in gps form (lon, lat) the
            # avgCentroids calculation are also in terms of (lon, lat) 
            #intersectGroups, averageCentroidsGPS = clustering.circleGroups(filteredCentroidsGPS, MIN_CIRCLE_RADIUS_GPS)

            # calculate search circle
            # for search circles we have a list of radii measured in meters
            # along with an associated tuple of the form (lon, lat)
            #searchRadii = getInfo.getSearchCircles(intersectGroups, averageCentroidsGPS, MIN_CIRCLE_RADIUS_GPS)

            wolfDataList = overseerGetWolfData.getWolfDataOfCluster(Cluster)
            cleanWaypointHistory(wolfDataList)

            for x in range(len(circleList)):
                waypoint = circleList[x].avgCenter
                radius = circleList[x].radius

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

                    # SEND MESSAGE TO OPTIMAL DRONE
                    serviceName = WOLF_DRONE_SERVICE + str(optimalDroneName)
                    circleCenterGPS =  gpsDataObject
                    circleRadiusGPS = circleList[x].radius
                    circleRadiusMeters = (circleList[x].radius*MIN_CIRCLE_RADIUS_METERS)/MIN_CIRCLE_RADIUS_GPS
                    spreadTimeS = 30
                    searchTimeS = (circleList[x].radius*20)/MIN_CIRCLE_RADIUS_GPS
                    taskGroup = ""  # Let's wolf know it comes from an overseer

                    # IF TASK GROUP IS EMPTY, THE REQUEST IS FROM THE OVERSEER
                    # IF HAS NAME, IS FROM WOLF
                    overseerLocation = threadClient.getGpsData(gps_name = "", vehicle_name = droneName)
                    calcDistanceBetweenGPS = calcHelper.calcDistanceBetweenGPS(overseerLocation.gnss.geo_point, gpsDataObject)

                    # print("Overseerr to GPS Difference:", calcDistanceBetweenGPS)
                    requestStatus = instructWolf.sendWolfSearchBehaviorRequest(serviceName, circleCenterGPS, circleRadiusGPS, circleRadiusMeters, spreadTimeS, searchTimeS,  taskGroup)
                    print("Request bool:", requestStatus, "From Overseer:", droneName, "To:", optimalDroneName)

        time.sleep(1)
        end = time.time()
        timeSpent += end-start
        i+=1
    return
    debugPrint(" CameraDetection: Average Loop Time: " + str(timeSpent / i))
# Theads END ===========================================

# DON'T SPLIT THE NEXT THREE FUNCTIONS UP!!! :(
def updateWayPointHistory(waypoint, taskGroup, radius):
    global Waypoint_History
    Waypoint_History.append([waypoint, taskGroup, radius])

def cleanWaypointHistory(wolfDroneDataList):
    global Waypoint_History
    filteredWaypointHistory = []
    for waypoint in Waypoint_History:
        for wolf in wolfDroneDataList:
            if wolf.taskGroup == waypoint[1]:
                filteredWaypointHistory.append(waypoint)
    Waypoint_History = filteredWaypointHistory

def isValidCentroid(centroid):
    global Waypoint_History
    for waypoint in Waypoint_History:
        distance = sqrt((float(waypoint[0][0]) - float(centroid[0]))**2 + (float(waypoint[0][1]) - float(centroid[1]))**2)
        if (distance < WAYPOINT_HISTORY_DISTANCE_MULT*waypoint[2]):             
            return False
    return True

def handleEnd(data):
    global End_Loop
    if (data.data == "End"):
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
    global Task_Group

    # Creates droneMsg object and inserts values from AirSim apis
    droneMsg = droneData()
    droneMsg.droneName = droneName
    droneMsg.longitude = position.gps_location.longitude
    droneMsg.latitude = position.gps_location.latitude
    droneMsg.velocityX = velocity.gnss.velocity.x_val
    droneMsg.velocityY = velocity.gnss.velocity.y_val
    droneMsg.cluster = droneName
    droneMsg.taskGroup = Task_Group

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

# Creates drone groups based on wolf number
def wolfClusterCreation(droneName):
    global DM_Wolfs_Cluster
    global Cluster
    Cluster = droneName
    if (droneName == "Overseer_0"):
        DM_Wolfs_Cluster = [0, 1, 2]
    else:
        DM_Wolfs_Cluster = [3, 4, 5]

def allDronesAtWaypoint():
    global WAYPOINT_INDEX
    wolfInfoArray = overseerGetWolfData.getOverseerGetWolfState()
    for droneNum in DM_Wolfs_Cluster:
        xDifference = wolfInfoArray[droneNum].longitude - float(WAYPOINT_COORDS[WAYPOINT_INDEX][0])
        yDifference = wolfInfoArray[droneNum].latitude - float(WAYPOINT_COORDS[WAYPOINT_INDEX][1])

        # If any of the drones are out of bounds, return false
        if ((abs(xDifference) > 0.0003) or (abs(yDifference) > 0.0003)):
            # print(droneNum, "X difference:", xDifference, "Y Difference:", yDifference)
            return 0

    WAYPOINT_INDEX = WAYPOINT_INDEX + 1
    # print("Drones:", DM_Wolfs_Cluster, "Made it to waypoint:", WAYPOINT_INDEX)
    return 1