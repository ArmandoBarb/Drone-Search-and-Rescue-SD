# Overseer node
# Subscribes to the (Command)
# Publishes to (OverseerDroneData)
# Publishes to the (CommandResult)
# Can use the (OverseerXActionHandlerAPI) service to give goals to their drones
# Adds or removes drones from their assignment
# Uses services for positions on (OverseerStateAPI) and (WolfStateAPI)

import setup_path # If setup_path gives issue, comment out
import airsim
import rospy
import time
import json
import ast
import configDrones
from threading import Timer
from threading import Thread
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from ServiceRequestors.instructWolf import sendWolfCommandClusterInfo
from ServiceRequestors.overseerGetOverseerData import getOverseerState
from ServiceRequestors.overseerGetWolfData import getOverseerGetWolfState
# from instructWolf import sendWolfCommandClusterInfo
# from ActionHandler import actionHandlerController

OVERSEER_DATA_TOPIC = "OverseerData"
COMMAND_TOPIC = "Command"
LOCAL_IP = configDrones.LOCAL_IP
COMMAND_RESULT_TOPIC = "CommandResult"
PROXIMITY_OVERSEER_SERVICE = "ProximityOverseerService"
OVERSEER_COMMUNICATION_TOPIC = "OverseerCommunication"

# Main function for the overseer drone
def overseerDroneController(droneName, droneCount):
    print("This is overseer", droneName)
    droneNum = ''.join(filter(lambda i: i.isdigit(), droneName)) # Get overseer number from droneName

    # One node for "MissionControl"
    nodeName = droneName
    rospy.init_node(nodeName, anonymous = True)

    # Sets client and takes off drone
    client = takeOff(droneName)
    print("Overseer drone", droneName)

    # Call startup service on each wolf
    droneLimit = int(droneNum) * 3
    for num in range(3):
        wolfNum = num + droneLimit
        wolfDroneService = "wolf_service_" + str(wolfNum)
        sendWolfCommandClusterInfo(wolfDroneService)

    # Start overseer communication subscriber thread service
    t = Thread(target = overseerCommunicationSubscriber, args=())
    t.start()

    overseerDataPublish = rospy.Publisher(OVERSEER_DATA_TOPIC, String, latch=True, queue_size=1)
    overseerCommunicationPublish = rospy.Publisher(OVERSEER_COMMUNICATION_TOPIC, String, latch=True, queue_size=1)

    # Add in loop logic
    i = 0
    print("Starting overseer loop")
    while (i < 10):

        # Publishes to (OverseerData) topic
        overseerDataPublisher(overseerDataPublish, client, droneName)

        # Publishes to (OverseerCommunication) topic
        overseerCommunicationPublisher(overseerCommunicationPublish, client, droneName)

        # Tests out overseerGetOverseerData
        # print(getOverseerState())

        # Tests out getOverseerGetWolfState
        # print(getOverseerGetWolfState())

        time.sleep(1)
        i+=1


# Subscribes to (Command) topic
def commandSub():
    rospy.Subscriber(COMMAND_TOPIC, String, handleCommand, (droneCount, client))
    rospy.spin()

# Takes in strings from the (Command) topic for processing
def handleCommand(data, args):
    print()
    # TODO: ADD IN CODE TO HANDLE COMMAND RESULT TOPIC

# Sets up publisher and calls function for (OverseerDroneData)
def overseerCommunicationPublisher(pub, client, droneName):
    message = "Hello, I'm drone " + droneName
    pub.publish(message) # publish lcoation

# Subscribes to (Command) topic
def overseerCommunicationSubscriber():
    rospy.Subscriber(OVERSEER_COMMUNICATION_TOPIC, String, handleOverseerCommunication, ())
    rospy.spin()

# Takes in strings from the (Command) topic for processing
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
    client.confirmConnection()
    client.enableApiControl(True, droneName)
    client.armDisarm(True, droneName)
    client.takeoffAsync(vehicle_name=droneName).join()

    return client
