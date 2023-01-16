# Nodes: "ProximityWolf" node for wolfs
# Will subscribe to (WolfData) topic for wolfs
# (WolfStateAPI) will reply nearby position data / states to requester
# Will have two separate responses for overseer and wolfs

import rospy
import json
from threading import Thread
from std_msgs.msg import String
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.srv import getDroneData, getDroneDataResponse
from std_srvs.srv import Trigger, TriggerResponse

WOLF_DATA_TOPIC = "WolfData"
# WOLF_POSITIONS = []

def globalVarSetup(droneCount):
    global WOLF_POSITIONS
    WOLF_POSITIONS = [droneData()]*droneCount

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
    service = rospy.Service(serviceName, getDroneData, wolf_position_response)
    rospy.spin()

def wolf_position_response(request):
    wolfPositionsArray = [WOLF_POSITIONS]
    # print(WOLF_POSITIONS[0])
    return [WOLF_POSITIONS]

# Connects subcriber listen to WolfData
def wolfDataSub(droneCount):
    print(WOLF_DATA_TOPIC)
    rospy.Subscriber(WOLF_DATA_TOPIC, droneData, updateWolfData, ())
    rospy.spin()


# Stores drone position to array, moves drone towards average location
def updateWolfData(data, args):
    global WOLF_POSITIONS
    curDroneIndex = int(data.droneName) # Gets droneName to know which index in array to put drone Ex: A drone named "0" would go in the 0 index
    WOLF_POSITIONS[curDroneIndex] = data # Inserts droneData to respective array index
