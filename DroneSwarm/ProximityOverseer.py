# Nodes: "ProximityOverseer" node
# Will subscribe to (OverseerData) topic for overseers
# (OverseerStateAPI) will reply nearby position data / states to requesters

import rospy
import json
from threading import Thread
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

OVERSEER_DATA_TOPIC = "OverseerData"
OVERSEER_POSITIONS = []

def globalVarSetup(droneCount):
    global OVERSEER_POSITIONS
    OVERSEER_POSITIONS = [None]*droneCount

# Main proximity overseer function
def startProximityOverseer(droneCount):
    print("We're in overseer proximity")

    # One node for "ProximityOverseer"
    nodeName = "ProximityOverseer"
    rospy.init_node(nodeName, anonymous = True)

    # Start overseer service
    t = Thread(target = overseerProximityService, args=())
    t.start()

    print("Starting proximity overseer")

    globalVarSetup(droneCount)
    overseerDataSub(droneCount)
    overseerProximityService()

def overseerProximityService():
    serviceName = "ProximityOverseerService"
    service = rospy.Service(serviceName, Trigger, overseer_position_response)
    rospy.spin()

def overseer_position_response(request):
    overseerPositionsString = str(OVERSEER_POSITIONS)
    return TriggerResponse(
        success=True,
        message=overseerPositionsString
    )

# Connects subcriber listen to OverseerData
def overseerDataSub(droneCount):
    rospy.Subscriber(OVERSEER_DATA_TOPIC, String, updateOverseerData, ())
    rospy.spin()


# Stores drone position to array, moves drone towards average location
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
