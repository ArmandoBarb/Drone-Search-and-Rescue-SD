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
from ServiceRequestors.instructWolf import sendWolfCommandClusterInfo
from ServiceRequestors.overseerGetOverseerData import getOverseerState
from ServiceRequestors.overseerGetWolfData import getOverseerGetWolfState

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


    # Call startup service on each wolf
    droneLimit = int(droneNum) * 3
    for num in range(3):
        wolfNum = num + droneLimit
        wolfDroneService = WOLF_DRONE_SERVICE + str(wolfNum)
        sendWolfCommandClusterInfo(wolfDroneService)

    # Start all threads here (if you have to make one somwhere else bring it up with the team)
    t = Thread(target = overseerCommunicationSubscriber, args=())
    t.start()

    # Create topic publishers
    # (TODO: ADD IN COMMAND RESULT PUBLISHERS)
    overseerDataPublish = rospy.Publisher(OVERSEER_DATA_TOPIC, String, latch=True, queue_size=1)
    overseerCommunicationPublish = rospy.Publisher(OVERSEER_COMMUNICATION_TOPIC, String, latch=True, queue_size=1)

    # Sets client and takes off drone
    client = takeOff(droneName)

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

    jsonWolfInfo = json.dumps({"DroneName": droneName, "Longitude": position.gps_location.longitude, "Latitude": position.gps_location.latitude, "vx": velocity.gnss.velocity.x_val, "vy": velocity.gnss.velocity.y_val}, indent=4)
    pub.publish(jsonWolfInfo) # publish lcoation

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
