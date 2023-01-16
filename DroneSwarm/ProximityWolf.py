# Nodes: "ProximityWolf" node for wolfs
# Will subscribe to (WolfData) topic for wolfs
# (WolfStateAPI) will reply nearby position data / states to requester
# Will have two separate responses for overseer and wolfs
# TODO: TOTEST : Test if this becomes bottelneck

import rospy
import json
# import Constants
import Constants.ros as ros

from threading import Thread
from std_msgs.msg import String
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.srv import getDroneData, getDroneDataResponse
from std_srvs.srv import Trigger, TriggerResponse

# Environmental Variables
# ros: topics
WOLF_DATA_TOPIC = ros.WOLF_DATA_TOPIC
# ros: services
PROXIMITY_WOLF_SERVICE = ros.PROXIMITY_WOLF_SERVICE

def globalVarSetup(droneCount):
    global WOLF_POSITIONS
    WOLF_POSITIONS = [droneData()]*droneCount

# Main promixity wolf function, returns list of wolf locations
# TODO: ADD IN RADIUS

# Main Process Start ----------------------------------------------
def startProximityWolf(droneCount):
    debugPrint("Starting proximity wolf")
    # Create node for "ProximityWolf"
    nodeName = "ProximityWolf"
    rospy.init_node(nodeName, anonymous = True)

    # Start wolf service
    t = Thread(target = wolfProximityService, args=())
    t.start()

    globalVarSetup(droneCount)
    # TODO: handle overseer position Data writing to storage
    debugPrint("Subscribing to " + WOLF_DATA_TOPIC)
    wolfDataSub(droneCount)
    # is this needed? # wolfProximityService()

# Main Process end -----------------------------------------------

# Theads Start ===========================================
def wolfProximityService():
    serviceName = "PromixityWolfService"
    service = rospy.Service(serviceName, getDroneData, wolf_position_response)
    rospy.spin()

# Connects subcriber listen to WolfData
def wolfDataSub(droneCount):
    print(WOLF_DATA_TOPIC)
    rospy.Subscriber(WOLF_DATA_TOPIC, String, updateWolfData, ())
    rospy.spin()

# Theads END ===========================================

# TODO: Functions need to Refatctor +++++++++++++++++++++++++++++++++++

# # TODO: handle data retrieval for service calls
def wolf_position_response(request):
    wolfPositionsArray = [WOLF_POSITIONS]
    # print(WOLF_POSITIONS[0])
    return [WOLF_POSITIONS]

# Connects subcriber listen to WolfData
def wolfDataSub(droneCount):
    print(WOLF_DATA_TOPIC)
    rospy.Subscriber(WOLF_DATA_TOPIC, droneData, updateWolfData, ())
    rospy.spin()

# Stores drone position to array, moves drone towards average location\
# TODO: handle data storage
def updateWolfData(data, args):
    global WOLF_POSITIONS
    curDroneIndex = int(data.droneName) # Gets droneName to know which index in array to put drone Ex: A drone named "0" would go in the 0 index
    WOLF_POSITIONS[curDroneIndex] = data # Inserts droneData to respective array index

def debugPrint (debugMessage):
    print("ProximityWolf: ",  debugMessage)
