# Wolf drone nodes
# Would publish to (SlamMerge) topic
# Would publish to (WolfData) topic
# Would publish to (CommandResult) topic
# Subscribes to (Command) topic
# Publishes and subscribes to (WolfCommunicationX) topic
# Requests from the (WolfStateAPI)
# Recieves and provides feedback to respective action handler

import setup_path # If setup_path gives issue, comment out
import airsim
import rospy
import time
import json
import ast
from threading import Timer
from threading import Thread
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

RUNTIME = 10

SLAM_MERGE_TOPIC = "SlamMerge"
WOLF_DATA_TOPIC = "WolfData"
COMMAND_RESULT_TOPIC = "CommandResult"
COMMAND_TOPIC = "Command"
LOCAL_IP = "192.168.86.40"
PROXIMITY_WOLF_SERVICE = "PromixityWolfService"
WOLFS_CLUSTER = []

def wolfDroneController(droneName, droneCount):
    print("This is wolf", droneName)

    # Node for wolf
    nodeName = "Wolf" + droneName
    rospy.init_node(nodeName, anonymous = True)

    # Start wolf service
    t = Thread(target = wolfService, args=(droneName))
    t.start()

    # Sets client and takes off drone
    client = takeOff(droneName)

    # Create topic publishers
    # (TODO: ADD IN SLAM MERGE AND COMMAND RESULT PUBLISHERS)
    wolfDataPublish = rospy.Publisher(WOLF_DATA_TOPIC, String, latch=True, queue_size=1)


    # Add in loop logic
    i = 0
    print("Starting wolf loop")
    while (i < 10):

        # Publishes to (WolfData) topic
        wolfDataPublisher(wolfDataPublish, client, droneName)

        # Get wolf data using a service
        rospy.wait_for_service(PROXIMITY_WOLF_SERVICE)

        # Gets service response and messsage from WolfData
        response = rospy.ServiceProxy(PROXIMITY_WOLF_SERVICE, Trigger)
        resp = response()
        responseText = resp.message
        convertedResponseArray = ast.literal_eval(responseText)


        # # TODO:
        # AREA CAN BE USED TO CALL BEHAVIOR FUNCTIONS FOR TESTING

        time.sleep(1)
        i+=1

def wolfService(droneName):
    serviceName = "wolf_service_" + droneName
    service = rospy.Service(serviceName, Trigger, trigger_response)
    rospy.spin()

def trigger_response(request):
    return TriggerResponse(
        success=True,
        message="Got service response from wolf"
    )

# Publishes wolf data to (WolfData) topic
def wolfDataPublisher(pub, client, droneName):
    position = client.getMultirotorState(vehicle_name = droneName)
    velocity = client.getGpsData(vehicle_name = droneName)

    jsonWolfInfo = json.dumps({"DroneName": droneName, "Longitude": position.gps_location.longitude, "Latitude": position.gps_location.latitude, "vx": velocity.gnss.velocity.x_val, "vy": velocity.gnss.velocity.y_val}, indent=4)
    pub.publish(jsonWolfInfo) # publish lcoation

# Enables api control, takes off drone, returns the client
def takeOff(droneName):
    client = airsim.MultirotorClient(LOCAL_IP)
    client.confirmConnection()
    client.enableApiControl(True, droneName)
    client.armDisarm(True, droneName)
    client.takeoffAsync(vehicle_name=droneName).join()

    return client
