# Nodes: "ProximityOverseer" node
# This is a mock for communication
# we could move this to each individual drone to handle but this is cleaner
# TODO: TOTEST : Test if this becomes bottelneck
# subscribes to (OverseerData) topic for overseers
# listens for service request to send data to

import rospy
import json
# import Constants
import Constants.ros as ros
# TODO: Investigate if we need to use a Lock while writing or reading global variables
from threading import Timer # Use for interval checks with minimal code
from threading import Thread # USe for important code running constantly
# TODO: Add custom message types
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

# Environmental Variables
# ros: topics
OVERSEER_DATA_TOPIC = ros.OVERSEER_DATA_TOPIC
# ros: services
PROXIMITY_OVERSEER_SERVICE = ros.PROXIMITY_OVERSEER_SERVICE

# Internal Drone memory shared across threads
OVERSEER_POSITIONS = []

def globalVarSetup(droneCount):
    global OVERSEER_POSITIONS
    OVERSEER_POSITIONS = [None]*droneCount

# Main Process Start ----------------------------------------------
def startProximityOverseer(droneCount):
    debugPrint(' started overseer proximity')

    # Create Node for "ProximityOverseer" (Only one should exist)
    nodeName = "ProximityOverseer"
    rospy.init_node(nodeName, anonymous = True)

    # Start overseer service
    t = Thread(target = overseerProximityService, args=())
    t.start()

    globalVarSetup(droneCount)
    # TODO: handle overseer position Data writing to storage
    debugPrint("Subscribing to " + OVERSEER_DATA_TOPIC)
    overseerDataSub(droneCount)
    # is this needed? # overseerProximityService()

# Main Process End ----------------------------------------------

# Theads Start ===========================================
def overseerProximityService():
    service = rospy.Service(PROXIMITY_OVERSEER_SERVICE, Trigger, overseer_position_response)
    rospy.spin()

# Connects subcriber listen to OverseerData
def overseerDataSub(droneCount):
    rospy.Subscriber(OVERSEER_DATA_TOPIC, String, updateOverseerData, ())
    rospy.spin()
# Theads End ===========================================


# TODO: Functions need to Refatctor +++++++++++++++++++++++++++++++++++


# TODO: handle data retrieval for service calls
def overseer_position_response(request):
    overseerPositionsString = str(OVERSEER_POSITIONS)
    return TriggerResponse(
        success=True,
        message=overseerPositionsString
    )

# Stores drone position to array, moves drone towards average location\
# TODO: handle data storage
def updateOverseerData(data, args):
    global OVERSEER_POSITIONS
    # Gets drone numbers and saves json data
    topicData = str(data.data)
    droneLocationJson = json.loads(topicData)
    # Adds respective drone information to array
    # curDroneIndex = int(droneLocationJson['DroneName'])
    droneNum = ''.join(filter(lambda i: i.isdigit(), droneLocationJson['DroneName'])) # Get overseer number from droneName
    curDroneIndex = int(droneNum)
    OVERSEER_POSITIONS[curDroneIndex] = [droneLocationJson['DroneName'], droneLocationJson['Latitude'], droneLocationJson['Longitude'], droneLocationJson['vx'], droneLocationJson['vy']]


def debugPrint (debugMessage):
    print("ProximityOverseer: ",  debugMessage)