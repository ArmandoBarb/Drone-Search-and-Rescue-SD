# Wolf drone nodes
# TODO: (SlamMerge) topic
# TODO: Subscribes to (Command) topic
# TODO: Publish to (CommandResult) topic (may be removed as not critical)

# imports
import setup_path # If setup_path gives issue, comment out
import airsim
import rospy
import time
import json
import ast
# import constants
import Constants.configDrones as configDrones
import Constants.ros as ros
# import drone behavior
from DroneBehaviors.wolfSearchBehavior import wolfSearchBehavior;
from DroneBehaviors.lineBehavior import lineBehavior
# TODO: Investigate if we need to use a Lock while writing or reading global variables
from threading import Timer # Use for interval checks with minimal code
from threading import Thread # USe for important code running constantly
# TODO: Add custom message types
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from airsim_ros_pkgs.msg import droneData
from ServiceRequestors.wolfGetWolfData import getWolfState

# Environmental Variables
RUNTIME = configDrones.RUNTIME
LOCAL_IP = configDrones.LOCAL_IP
# ros: topics
SLAM_MERGE_TOPIC = ros.SLAM_MERGE_TOPIC # TODO
WOLF_DATA_TOPIC = ros.WOLF_DATA_TOPIC
COMMAND_RESULT_TOPIC = ros.COMMAND_RESULT_TOPIC # TODO
COMMAND_TOPIC = ros.COMMAND_TOPIC # TODO
# ros: services: service calls should be in the ServiceRequesteros folder
PROXIMITY_WOLF_SERVICE = ros.PROXIMITY_WOLF_SERVICE
# dynamic services:
WOLF_DRONE_SERVICE = ros.WOLF_DRONE_SERVICE

# Internal Wolf Drone Memory Start -------------------------------------------
# Current pattern is ussing Global variable to allow access across threads (open to change)
# Global variables
DM_Drone_Name = None
DM_Wolfs_Cluster = [] # Drone will beassigned a group of drones to work with
WAYPOINT_COORDS = [None]*9
WAYPOINT_INDEX = 0
GROUP_0_SEARCH = 'Constants/Group0Spiral.txt'
GROUP_1_SEARCH = 'Constants/Group1Spiral.txt'
# Internal Wolf Drone Memory End -------------------------------------------

# TODO: add tunning variables for behaviors (would be cool if we can train them)

# Main Process Start ----------------------------------------------
def wolfDroneController(droneName, droneCount):
    # set global vairable
    global DM_Drone_Name
    DM_Drone_Name = droneName
    global WAYPOINT_INDEX

    # Sets global values for wolf cluster and coordinate
    wolfClusterCreation(droneName)
    droneBoundary = (int(droneCount) / 2)
    if (int(droneName) <= 2):
        readCoordFile(GROUP_0_SEARCH)
    else:
        readCoordFile(GROUP_1_SEARCH)

    # use this code to make print calls allowing you to know what process made the print statemnt
    debugPrint("Process started")

    # Create Node for wolf
    nodeName = "Wolf" + droneName
    rospy.init_node(nodeName, anonymous = True)
    debugPrint("Node initiated")

    # Start all threads here (if you have to make one somwhere else bring it up with the team)
    t = Thread(target = wolfServiceListeners, args=(droneName))
    t.start()

    # Create topic publishers
    # (TODO: ADD IN SLAM MERGE AND COMMAND RESULT PUBLISHERS)
    wolfDataPublish = rospy.Publisher(WOLF_DATA_TOPIC, droneData, latch=True, queue_size=1)


    # Sets and connects to client and takes off drone
    client = takeOff(droneName)
    client.moveToZAsync(z=-10, velocity=8, vehicle_name = droneName).join()


    # Wolf Drone search loop Start
    i = 0
    debugPrint("Starting Search and Rescue loop")
    while (i < 30):
        # Publishes to (WolfData) topic
        wolfDataPublisher(wolfDataPublish, client, droneName)

        # TEST OUT WOLF SERVICE, wolfGetWolfData
        # wolfInfoArray = getWolfState()        # Get droneWolfState state array from service
        # print(wolfInfoArray[0])               # Example of printing wolf drone 3's information

        # Get Airsim Data and procesess it here
        # TODO: add Yolo person Detector (if runtime is to long Seprate into thread that runs on intervals)
            # getDataFromAirsim -> yolo detect -> update internal drone state or publish data to other drones
        # TODO: Publishes to (WolfData) topic
        wolfDataPublisher(wolfDataPublish, client, droneName) # Publish drones state at each loop so other drones can cordiante behaviors
        # TODO: add Colliion detecotr
            # getNeededAirSimData -> checkForCollision -> update collision behavior
        collisionAvoidance = False # set to true if need to do collision avoidance (open to better integration method)


        # # TODO: Add in Drone behavior desion making
        # TODO: Line formation behavior, toss in waypoint drone will move to
        # TODO: Wolf Search behavior

        vector = lineBehavior(client, int(droneName), DM_Wolfs_Cluster, WAYPOINT_COORDS[WAYPOINT_INDEX])
        # If all drones make it to the waypoint, more to next waypoint
        allDronesAtWaypoint()
        # print("Wolf", droneName, "At waypoint", WAYPOINT_INDEX)

        # vector = wolfSearchBehavior(currentGPS=position.gps_location, targetGPS=targetP.gps_location) # may need to refactor to use other gps format
        client.moveByVelocityZAsync(vector[1], vector[0], -10, duration = 1, vehicle_name=droneName)
        # TODO: Consensus Descion behavior
        # TODO: Apply turning to desired action
        # TODO: Overide other behaviors if collisionAvoidance is needed

        # TODO: Make Airsim call with desired action

        # Add in artifical loop delay (How fast the loop runs dictates the drones reaction speed)

        time.sleep(1)
        i+=1
    debugPrint("Ending Search and Rescue loop")
    # Wolf Drone search loop End

# Main Process End ----------------------------------------------

# Theads Start ===========================================
def wolfServiceListeners(droneName):
    serviceName = WOLF_DRONE_SERVICE + droneName
    service = rospy.Service(serviceName, Trigger, trigger_response)
    rospy.spin()

# Theads END ===========================================

# TODO: Functions need to Refatctor +++++++++++++++++++++++++++++++++++

# Creates drone groups based on wolf number
def wolfClusterCreation(droneName):
    droneNum = int(droneName)
    global DM_Wolfs_Cluster
    if (droneNum <= 2):
        DM_Wolfs_Cluster = [0, 1, 2]
    else:
        DM_Wolfs_Cluster = [3, 4, 5]

def trigger_response(request):
    return TriggerResponse(
        success=True,
        message="Got service response from wolf"
    )

# Publishes wolf data to (WolfData) topic
def wolfDataPublisher(pub, client, droneName):
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

def allDronesAtWaypoint():
    global WAYPOINT_INDEX
    wolfInfoArray = getWolfState()
    for droneNum in DM_Wolfs_Cluster:
        xDifference = wolfInfoArray[droneNum].longitude - float(WAYPOINT_COORDS[WAYPOINT_INDEX][0])
        yDifference = wolfInfoArray[droneNum].latitude - float(WAYPOINT_COORDS[WAYPOINT_INDEX][1])

        # If any of the drones are out of bounds, return false
        if ((abs(xDifference) > 0.0001) or (abs(yDifference) > 0.0001)):
            return 0

    WAYPOINT_INDEX = WAYPOINT_INDEX + 1
    print("Drones:", DM_Wolfs_Cluster, "Made it to waypoint:", WAYPOINT_INDEX)
    return 1


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

def debugPrint ( debugMessage):
    global DM_Drone_Name
    print("Wolf: ", DM_Drone_Name, " : " ,  debugMessage)
