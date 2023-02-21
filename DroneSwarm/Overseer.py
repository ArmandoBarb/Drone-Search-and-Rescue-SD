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
from ServiceRequestors.instructWolf import sendStartLineBehavior
from ServiceRequestors.overseerGetOverseerData import getOverseerState
from ServiceRequestors.overseerGetWolfData import getOverseerGetWolfState
from DroneBehaviors.lineBehavior import overseerWaypoint
from airsim_ros_pkgs.msg import droneData
from ServiceRequestors.wolfGetWolfData import getWolfState

# Environmental Variables
RUNTIME = configDrones.RUNTIME
LOCAL_IP = configDrones.LOCAL_IP

# ros: topics
OVERSEER_DATA_TOPIC = ros.OVERSEER_DATA_TOPIC
OVERSEER_COMMUNICATION_TOPIC = ros.OVERSEER_COMMUNICATION_TOPIC
COMMAND_TOPIC = ros.COMMAND_TOPIC
COMMAND_RESULT_TOPIC = ros.COMMAND_RESULT_TOPIC

# ros: services
PROXIMITY_OVERSEER_SERVICE = ros.PROXIMITY_OVERSEER_SERVICE
# Dynamic service append number
WOLF_DRONE_SERVICE = ros.WOLF_DRONE_SERVICE

# Internal Wolf Drone Memory Start -------------------------------------------
# Current pattern is ussing Global variable to allow access across threads (open to change)
# Global variables
DM_Drone_Name = None
DM_Wolfs_Cluster = []
WAYPOINT_COORDS = []
WAYPOINT_INDEX = 0
GROUP_0_SEARCH = 'Constants/Group0Spiral.txt'
GROUP_1_SEARCH = 'Constants/Group1Spiral.txt'

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

    # Call startup service on each wolf
    droneLimit = int(droneNum) * 3
    for num in range(3):
        wolfNum = num + droneLimit
        wolfDroneService = WOLF_DRONE_SERVICE + str(wolfNum)
        # sendWolfCommandClusterInfo(wolfDroneService)
        sendStartLineBehavior(wolfDroneService, GROUP_0_SEARCH, GROUP_1_SEARCH)

    # Overseer Drone search loop Start
    i = 0
    debugPrint("Starting Search and Rescue loop")
    while (i < RUNTIME):
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

        waypoint = getNewWaypoint(droneName)
        vector = overseerWaypoint(client, int(droneNum), waypoint)

        # If all drones make it to the waypoint, more to next waypoint
        allDronesAtWaypoint()

        client.moveByVelocityZAsync(vector[1], vector[0], -35, duration = 1, vehicle_name=droneName)

        # TODO: Add in Overseer behavior
        # TODO: Creeping Line lead behavior
            # Overseer will slighly go ahead its drone cluster to search for waypoints

        # TODO: Make Airsim call with desired action

        # Add in artifical loop delay (How fast the loop runs dictates the drones reaction speed)
        time.sleep(1)
        i+=1
    debugPrint("Ending Search and Rescue loop")
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
    rospy.spin()

# Theads END ===========================================

# TODO: Functions need to Refatctor +++++++++++++++++++++++++++++++++++

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

    # Creates droneMsg object and inserts values from AirSim apis
    droneMsg = droneData()
    droneMsg.droneName = droneName
    droneMsg.longitude = position.gps_location.longitude
    droneMsg.latitude = position.gps_location.latitude
    droneMsg.velocityX = velocity.gnss.velocity.x_val
    droneMsg.velocityY = velocity.gnss.velocity.y_val

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
    if (droneName == "Overseer_0"):
        DM_Wolfs_Cluster = [0, 1, 2]
    else:
        DM_Wolfs_Cluster = [3, 4, 5]

def allDronesAtWaypoint():
    global WAYPOINT_INDEX
    wolfInfoArray = getWolfState()
    for droneNum in DM_Wolfs_Cluster:
        xDifference = wolfInfoArray[droneNum].longitude - float(WAYPOINT_COORDS[WAYPOINT_INDEX][0])
        yDifference = wolfInfoArray[droneNum].latitude - float(WAYPOINT_COORDS[WAYPOINT_INDEX][1])

        # If any of the drones are out of bounds, return false
        if ((abs(xDifference) > 0.00015) or (abs(yDifference) > 0.00015)):
            return 0

    WAYPOINT_INDEX = WAYPOINT_INDEX + 1
    # print("Drones:", DM_Wolfs_Cluster, "Made it to waypoint:", WAYPOINT_INDEX)
    return 1
