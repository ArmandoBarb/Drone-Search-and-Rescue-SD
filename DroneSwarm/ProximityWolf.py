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
from std_srvs.srv import Trigger, TriggerResponse


# Environmental Variables
# ros: topics
WOLF_DATA_TOPIC = ros.WOLF_DATA_TOPIC
# ros: services
PROXIMITY_WOLF_SERVICE = ros.PROXIMITY_WOLF_SERVICE

WOLF_POSITIONS = []

def globalVarSetup(droneCount):
    global WOLF_POSITIONS
    WOLF_POSITIONS = [None]*droneCount

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
    serviceName = PROXIMITY_WOLF_SERVICE
    service = rospy.Service(serviceName, Trigger, wolf_position_response)
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
    wolfPositionsString = str(WOLF_POSITIONS)
    return TriggerResponse(
        success=True,
        message=wolfPositionsString
    )


# Stores drone position to array, moves drone towards average location\
# TODO: handle data storage
def updateWolfData(data, args):
    global WOLF_POSITIONS
    # Gets drone numbers and saves json data
    topicData = str(data.data)
    droneLocationJson = json.loads(topicData)

    # Adds respective drone information to array
    curDroneIndex = int(droneLocationJson['DroneName'])
    WOLF_POSITIONS[curDroneIndex] = [droneLocationJson['DroneName'], droneLocationJson['Latitude'], droneLocationJson['Longitude'], droneLocationJson['vx'], droneLocationJson['vy']]
    # WOLF_POSITIONS[curDroneIndex] = (droneLocationJson['DroneName'], droneLocationJson['Latitude'], droneLocationJson['Longitude'], droneLocationJson['vx'], droneLocationJson['vy'])
    # print("Wolf topic data", topicData)
    # print("curDroneIndex", curDroneIndex)
    # print("Wolf positions", WOLF_POSITIONS)


def debugPrint (debugMessage):
    print("ProximityWolf: ",  debugMessage)
