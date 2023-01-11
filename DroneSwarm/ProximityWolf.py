# Nodes: "ProximityWolf" node for wolfs
# Will subscribe to (WolfData) topic for wolfs
# (WolfStateAPI) will reply nearby position data / states to requester
# Will have two separate responses for overseer and wolfs

import rospy
import json
from threading import Thread
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

WOLF_DATA_TOPIC = "WolfData"
WOLF_POSITIONS = []

def globalVarSetup(droneCount):
    global WOLF_POSITIONS
    WOLF_POSITIONS = [None]*droneCount

# Main promixity wolf function, returns list of wolf locations
# TODO: ADD IN RADIUS


def startProximityWolf(droneCount):
    # One node for "ProximityWolf"
    nodeName = "ProximityWolf"
    rospy.init_node(nodeName, anonymous = True)

    # Start wolf service
    t = Thread(target = wolfProximityService, args=())
    t.start()

    # print("Starting proximity wolf")

    globalVarSetup(droneCount)
    wolfDataSub(droneCount)
    wolfProximityService()


def wolfProximityService():
    serviceName = "PromixityWolfService"
    service = rospy.Service(serviceName, Trigger, wolf_position_response)
    rospy.spin()

def wolf_position_response(request):
    wolfPositionsString = str(WOLF_POSITIONS)
    return TriggerResponse(
        success=True,
        message=wolfPositionsString
    )

# Connects subcriber listen to WolfData
def wolfDataSub(droneCount):
    print(WOLF_DATA_TOPIC)
    rospy.Subscriber(WOLF_DATA_TOPIC, String, updateWolfData, ())
    rospy.spin()


# Stores drone position to array, moves drone towards average location
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
