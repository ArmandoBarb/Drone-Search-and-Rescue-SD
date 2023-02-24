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
import math
import math
from math import sqrt
# import constants
import Constants.configDrones as configDrones
import Constants.ros as ros
# import drone behavior
import DroneBehaviors.wolfSearchBehavior as wolfSearchBehavior;
from DroneBehaviors.lineBehavior import lineBehavior
# TODO: Investigate if we need to use a Lock while writing or reading global variables
from threading import Timer # Use for interval checks with minimal code
from threading import Thread # USe for important code running constantly
# TODO: Add custom message types
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.msg import requestLineBehavior
from airsim_ros_pkgs.msg import requestWolfSearchBehavior
from airsim_ros_pkgs.msg import requestConsensusDecisionBehavior
from airsim_ros_pkgs.srv import getDroneData
from airsim_ros_pkgs.srv import sendCommand
import ServiceRequestors.wolfGetWolfData as wolfService
import ServiceRequestors.instructWolf as instructWolf
import HelperFunctions.calcHelper as helper

# Environmental Variables
LOOP_NUMBER = configDrones.LOOP_NUMBER
MAX_TIME = configDrones.MAX_TIME
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
WAYPOINT_COORDS = []
WAYPOINT_INDEX = 0
GROUP_0_SEARCH = 'Constants/Group0Spiral.txt'
GROUP_1_SEARCH = 'Constants/Group1Spiral.txt'
# Internal Wolf Drone Memory End -------------------------------------------
Line_Behavior = False
# Memory for circle behavior
Wolf_Search_Behavior = False
Consensus_Decision_Behavior = False
Circle_Center_GPS = [] # gps cordinate
Circle_Radius_GPS = 0 # radius distance in gps
Min_Circle_Radius_GPS = 0.00008983152373552244 # 10 in x direction converted to gps
Min_Circle_Radius_Meters = 6.988048291572515 # 10 in x direction converted to Meters
Circle_Radius_Meters = 0 # radius distance in meters
Start_Time = 0 # time
Spread_Time = 0 #  time in seconds # time to get in position 
Search_Time = 0 #  time in seconds # time to search
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
    client.moveToZAsync(z=-3, velocity=8, vehicle_name = droneName).join()
    
    # start camera thread here
    t = Thread(target = wolfCameraDetection, args=(droneName, client))
    t.start()

    # Test Code startWolfSearch
    targetP = client.getMultirotorState(vehicle_name = "target")
    targetC = client.getMultirotorState(vehicle_name = "circle")
    # calc radius
    radiusC = abs(targetP.gps_location.longitude - targetC.gps_location.longitude);
    radiusM = helper.calcDistanceInMetersBetweenGPS(targetP.gps_location, targetC.gps_location);

    # test Wolf Search
    # startWolfSearch( circleCenterGPS=targetP.gps_location, circleRadiusGPS=radiusC*7, circleRadiusMeters=radiusM*7, spreadTimeS=40, searchTimeS=70 );
    # startLineBehavior(group0Waypoints = 'Constants/Group0Spiral.txt', group1Waypoints = 'Constants/Group1Spiral.txt')
    # startWolfSearch( circleCenterGPS=targetP.gps_location, circleRadiusGPS=radiusC*7, circleRadiusMeters=radiusM*7, spreadTimeS=40, searchTimeS=70 );
    # startLineBehavior(group0Waypoints = 'Constants/Group0Spiral.txt', group1Waypoints = 'Constants/Group1Spiral.txt')
    # test Consensus Decision
    # startConsensusDecision( circleCenterGPS=targetP.gps_location, circleRadiusGPS=Min_Circle_Radius_GPS*2, circleRadiusMeters=Min_Circle_Radius_Meters*2, searchTimeS=100 );
    # startConsensusDecision( circleCenterGPS=targetP.gps_location, circleRadiusGPS=Min_Circle_Radius_GPS*2, circleRadiusMeters=Min_Circle_Radius_Meters*2, searchTimeS=100 );

    # Wolf Drone search loop Start
    i = 1
    debugPrint("Starting Search and Rescue loop")
    timeSpent = 0;
    runtime = time.time()
    while (i < LOOP_NUMBER):
        timeDiff = time.time() - runtime
        if (timeDiff > MAX_TIME):
            endLineBehavior()
            break;

        vector = [0, 0] # dont move if nothing to do
        yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=(0)) # Set yaw to zero

        # test consensus behavior
        if(Consensus_Decision_Behavior):
            timeDiff = time.time() - Start_Time;
            stageLength = Search_Time / 5
            newP = None
            if (timeDiff < stageLength):
                newP = client.getMultirotorState(vehicle_name = "target")
                # if (droneName == "0"):
                    # print('target');
            elif (timeDiff < stageLength*2):
                newP = client.getMultirotorState(vehicle_name = "targetR")
                # if (droneName == "0"):
                    # print('targetR');
            elif (timeDiff < stageLength*3):
                newP = client.getMultirotorState(vehicle_name = "circle")
                # if (droneName == "0"):
                    # print('circle');
            elif (timeDiff < stageLength*4):
                newP = client.getMultirotorState(vehicle_name = "circle2")
                # if (droneName == "0"):
                    # print('circle2');
            elif (timeDiff < stageLength*5):
                newP = client.getMultirotorState(vehicle_name = "circle3")
                # if (droneName == "0"):
                    # print('circle3');
            else:
                newP = client.getMultirotorState(vehicle_name = "target")
                # if (droneName == "0"):
                    # print('circle3');
            if (newP != None):
                # if (droneName == "0"):
                    # print("Gps: " + str(newP.gps_location));
                updateConsensusDecisionCenter(circleCenterGPS=newP.gps_location);

        
        start=time.time() # gather time data

        # Publishes to (WolfData) topic
        wolfDataPublisher(wolfDataPublish, client, droneName)
        
        # TEST OUT WOLF SERVICE, wolfGetWolfData
        # wolfInfoArray = getWolfState()        # Get droneWolfState state array from service
        # print(wolfInfoArray[0])               # Example of printing wolf drone 3's information

        # Get Airsim Data and procesess it here
        # TODO: add Yolo person Detector (if runtime is to long Seprate into thread that runs on intervals)
            # getDataFromAirsim -> yolo detect -> update internal drone state or publish data to other drones
        # TODO: Publishes to (WolfData) topic
        # #####wolfDataPublisher(wolfDataPublish, client, droneName) # Publish drones state at each loop so other drones can cordiante behaviors
        # TODO: add Collision detecotr
            # getNeededAirSimData -> checkForCollision -> update collision behavior
        collisionAvoidance = False # set to true if need to do collision avoidance (open to better integration method)


        # # TODO: Add in Drone behavior desion making
        if (Consensus_Decision_Behavior): # Consensus Descion behavior
            currentDroneData = client.getMultirotorState(vehicle_name = droneName);
            
            vector = consensusDecisionBehaviorGetVector(currentDroneData);

            yawDegrees = wolfSearchBehavior.calcYaw(currentGPS=currentDroneData.gps_location, targetGPS=Circle_Center_GPS);
            yaw_mode  = airsim.YawMode(is_rate=False, yaw_or_rate=(yawDegrees));

            # check if time to end consensus Desension
            timeDiff = time.time() - Start_Time
            if (timeDiff > (Search_Time)):
                endConsensusDecision();

        elif (Wolf_Search_Behavior): # Wolf Search behavior
            currentDroneData = client.getMultirotorState(vehicle_name = droneName);

            vector = wolfSearchBehaviorGetVector(currentDroneData);

            yawDegrees = wolfSearchBehavior.calcYaw(currentGPS=currentDroneData.gps_location, targetGPS=Circle_Center_GPS);
            yaw_mode  = airsim.YawMode(is_rate=False, yaw_or_rate=(yawDegrees));

            timeDiff = time.time() - Start_Time;
            if (timeDiff > (Spread_Time + Search_Time)):
                endWolfSearch();

        elif (Line_Behavior): # Line_Behavior
            # Gets drones waypoint and vector movement
            newWaypoint = getNewWaypoint(droneName)
            vector = lineBehavior(client, int(droneName), DM_Wolfs_Cluster, newWaypoint)
            vectorTemp = 0

            vectorTemp = vector[0]
            vector[0] = vector[1]
            vector[1] = vectorTemp

            # Calculates camera direction based on velocity      
            if (vector[1] != 0):
                yaw = math.atan2(vector[1], vector[0])
                degrees = math.degrees(yaw)
                yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=(degrees));
            else:
                degrees = 0
                yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=(degrees));


        # TODO: Apply turning to desired action
        # TODO: Overide other behaviors if collisionAvoidance is needed


        # TODO: Make Airsim call with desired action
        client.moveByVelocityZAsync(vector[0], vector[1], -8, duration = 0.5, yaw_mode=yaw_mode, vehicle_name=droneName)
        
        # Add in artifical loop delay (How fast the loop runs dictates the drones reaction speed)

        # time.sleep(0.18)
        end = time.time();
        timeSpent += end-start;

        # Checks if drones have made iot to the next waypoint
        allDronesAtWaypoint()

        i+=1
    # debugPrint("Ending Search and Rescue loop: ")
    debugPrint("Average Loop Time in seconds: " + str(timeSpent / i))
    # Wolf Drone search loop End

# Main Process End ----------------------------------------------

# Theads Start ===========================================
def wolfServiceListeners(droneName):
    serviceName = WOLF_DRONE_SERVICE + droneName
    service = rospy.Service(serviceName, sendCommand, commandResponse)
    rospy.spin()

# checks drone camera with yolo detection
def wolfCameraDetection(droneName, client):
    debugPrint("Starting wolfCameraDetection loop")
    i = 0;
    timeSpent = 0;
    runtime = time.time()
    while (i < LOOP_NUMBER):
        timeDiff = time.time() - runtime
        if (timeDiff > MAX_TIME):
            break;
        start=time.time() # gather time data
        # todo: marry add camera checkl nad yolo detector

        # mock detection
        timeDiff = time.time() - runtime
        if(not(Consensus_Decision_Behavior)):
            if(timeDiff > 18 and droneName == '1'):
                # targetP is estimated gps position
                targetP = client.getMultirotorState(vehicle_name = "target")
                circleCenterGPS = targetP.gps_location
                circleRadiusGPS = Min_Circle_Radius_GPS*2
                circleRadiusMeters = Min_Circle_Radius_Meters*2
                searchTimeS = 100
                startConsensusDecision( circleCenterGPS=circleCenterGPS, circleRadiusGPS=circleRadiusGPS, circleRadiusMeters=circleRadiusMeters, searchTimeS=searchTimeS )
                # ToDO addd function call to return list of availalbe drones
                # THis is Hardcoded need to replace
                instructWolf.sendConsensusDecisionBehaviorRequest(WOLF_DRONE_SERVICE + '2', circleCenterGPS, circleRadiusGPS, circleRadiusMeters, searchTimeS)
                instructWolf.sendConsensusDecisionBehaviorRequest(WOLF_DRONE_SERVICE + '0', circleCenterGPS, circleRadiusGPS, circleRadiusMeters, searchTimeS)

        time.sleep(1);
        end = time.time();
        timeSpent += end-start;
        i+=1
    return;
# startConsensusDecision( circleCenterGPS=targetP.gps_location, circleRadiusGPS=Min_Circle_Radius_GPS*2, circleRadiusMeters=Min_Circle_Radius_Meters*2, searchTimeS=100 );
    debugPrint(" CameraDetection: Average Loop Time: " + str(timeSpent / i))


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

def commandResponse(request):
    messageType = request.messageType
    lineInfo = request.lineBehaviorStart
    wolfSearchInfo = request.wolfSearchBehaviorStart
    consensusDecisionInfo = request.consensusDecisionBehaviorStart

    print("Got a command")
    
    lineString = str(lineInfo)
    # Find datatype with info, execute command based on who has data
    if (messageType == "RequestLineBehavior"):
        startLineBehavior(group0Waypoints = lineInfo.group0Waypoints, group1Waypoints = lineInfo.group1Waypoints)
        # debugPrint("Do line behavior")
        return True

    elif (messageType == "RequestWolfSearch"):
        # debugPrint("Do wolf search")
        return True

    elif (messageType == "RequestConsensusDecision"):
        # debugPrint("Do consensus decision")
        circleCenterGPS = consensusDecisionInfo.circleCenterGPS
        circleRadiusGPS = consensusDecisionInfo.circleRadiusGPS
        circleRadiusMeters = consensusDecisionInfo.circleRadiusMeters
        searchTimeS = consensusDecisionInfo.searchTimeS
        startConsensusDecision( circleCenterGPS=circleCenterGPS, circleRadiusGPS=circleRadiusGPS, circleRadiusMeters=circleRadiusMeters, searchTimeS=searchTimeS );
        return True
    
    return False

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

    # TODO: NEEDS TO BE ADDED
    droneMsg.cluster = "0"
    droneMsg.taskGroup = "0"

    # Publishes to topic
    pub.publish(droneMsg)

# Function get drones subwaypoint based on index
def getNewWaypoint(droneName):
    # Created global waypoints
    global WAYPOINT_INDEX
    # print("DroneName: ", droneName, "Current waypoint index", WAYPOINT_INDEX)
    currentWaypoint = WAYPOINT_COORDS[WAYPOINT_INDEX]
    newWaypoint = currentWaypoint

    if (WAYPOINT_INDEX >= 1):
        # Radius in charge of distance between drones
        radius = 0.0001
        previousWaypoint = WAYPOINT_COORDS[WAYPOINT_INDEX-1]

        # Finds vector between waypoints
        waypointDiffX = float(currentWaypoint[0]) - float(previousWaypoint[0])
        waypointDiffY = float(currentWaypoint[1]) - float(previousWaypoint[1])

        # Gets normalized difference vector
        vectorVal = sqrt(waypointDiffX**2 + waypointDiffY**2)
        xDirection = (waypointDiffX/vectorVal) * radius
        yDirection = (waypointDiffY/vectorVal) * radius

        # Creates lanes for 3 group clusters
        if (len(DM_Wolfs_Cluster) == 3):
            # Moves first drone left of the waypoint
            if ((int(droneName) % 3) == 0):
                newWaypointX = float(currentWaypoint[0]) - yDirection
                newWaypointY = float(currentWaypoint[1]) + xDirection
                newWaypoint = [float(newWaypointX), float(newWaypointY)]
                # print("Drone", droneName, "Int dronename", (int(droneName)), "Moving to ", newWaypoint)
            # Moves second drone directly to waypoint
            elif((int(droneName) % 3) == 1):
                newWaypoint = currentWaypoint
                # print("Drone", droneName, "Moving to ", newWaypoint)
            # Moves third drone right of the waypoint
            elif((int(droneName) % 3) == 2):
                newWaypointX = float(currentWaypoint[0]) + yDirection
                newWaypointY = float(currentWaypoint[1]) - xDirection
                newWaypoint = [float(newWaypointX), float(newWaypointY)]
                # print("Drone", droneName, "Int dronename", (int(droneName)), "Moving to ", newWaypoint)

        # Creates lanes for 4 group clusters
        if (len(DM_Wolfs_Cluster) == 4):
            # Moves first drone left of the waypoint
            if ((int(droneName) % 4) == 0):
                newWaypointX = float(currentWaypoint[0]) - (yDirection * 1.5)
                newWaypointY = float(currentWaypoint[1]) + (xDirection * 1.5)
                newWaypoint = [float(newWaypointX), float(newWaypointY)]
                # print("Drone", droneName, "Int dronename", (int(droneName)), "Moving to ", newWaypoint)
            # Moves second drone directly to waypoint
            elif((int(droneName) % 4) == 1):
                newWaypointX = float(currentWaypoint[0]) - (yDirection * 0.5)
                newWaypointY = float(currentWaypoint[1]) + (xDirection * 0.5)
                newWaypoint = [float(newWaypointX), float(newWaypointY)]
            elif ((int(droneName) % 4) == 2):
                newWaypointX = float(currentWaypoint[0]) + (yDirection * 0.5)
                newWaypointY = float(currentWaypoint[1]) - (xDirection * 0.5)
                newWaypoint = [float(newWaypointX), float(newWaypointY)]
                # print("Drone", droneName, "Moving to ", newWaypoint)
            # Moves third drone right of the waypoint
            elif((int(droneName) % 4) == 3):
                newWaypointX = float(currentWaypoint[0]) + (yDirection * 1.5)
                newWaypointY = float(currentWaypoint[1]) - (xDirection * 1.5)
                newWaypoint = [float(newWaypointX), float(newWaypointY)]
                # print("Drone", droneName, "Int dronename", (int(droneName)), "Moving to ", newWaypoint)


    return newWaypoint


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
    wolfInfoArray = wolfService.getWolfState()
    for droneNum in DM_Wolfs_Cluster:
        xDifference = wolfInfoArray[droneNum].longitude - float(WAYPOINT_COORDS[WAYPOINT_INDEX][0])
        yDifference = wolfInfoArray[droneNum].latitude - float(WAYPOINT_COORDS[WAYPOINT_INDEX][1])

        # If any of the drones are out of bounds, return false
        if ((abs(xDifference) > 0.00015) or (abs(yDifference) > 0.00015)):
            return 0

    WAYPOINT_INDEX = WAYPOINT_INDEX + 1
    # print("Drones:", DM_Wolfs_Cluster, "Made it to waypoint:", WAYPOINT_INDEX)
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
    # client.moveByAngleRatesZAsync(roll_rate=0, pitch_rate=0, yaw_rate=2, z=-3, duration=1, vehicle_name=droneName).join()

    return client

def startLineBehavior(group0Waypoints, group1Waypoints):
    global GROUP_0_SEARCH
    global GROUP_1_SEARCH
    global Line_Behavior
    GROUP_0_SEARCH = group0Waypoints
    GROUP_1_SEARCH = group1Waypoints
    Line_Behavior = True

def endLineBehavior():
    global GROUP_0_SEARCH
    global GROUP_1_SEARCH
    global Line_Behavior
    GROUP_0_SEARCH = None
    GROUP_1_SEARCH = None
    Line_Behavior = False

def startWolfSearch( circleCenterGPS, circleRadiusGPS, circleRadiusMeters, spreadTimeS, searchTimeS ):
    global Wolf_Search_Behavior;
    global Circle_Center_GPS;
    global Circle_Radius_GPS, Circle_Radius_Meters;
    global Start_Time, Spread_Time, Search_Time;


    Circle_Center_GPS = circleCenterGPS;
    Circle_Radius_GPS, Circle_Radius_Meters = circleRadiusGPS, circleRadiusMeters;
    Search_Time = searchTimeS;
    Spread_Time = spreadTimeS;
    Start_Time = time.time();
    Wolf_Search_Behavior = True;

def wolfSearchBehaviorGetVector(currentDroneData):
    radius = Circle_Radius_GPS
    radiusM = Circle_Radius_Meters

    timeDiff = time.time() - Start_Time;
    if (timeDiff > Spread_Time):
        timeDiv = (Search_Time - (timeDiff - Spread_Time)) / Search_Time
        radius = (radius - Min_Circle_Radius_GPS)*timeDiv + Min_Circle_Radius_GPS;
        radiusM = (radiusM - Min_Circle_Radius_Meters)*timeDiv + Min_Circle_Radius_Meters;
    
    wolfDataArray = wolfService.getWolfDataExC(DM_Drone_Name);
    # calcSpeedVector function variables
    averageAlignmentSpeed = 12;
    bonusAlignmentSpeed = 0;
    maxCohSepSpeed = 4;
    maxSpeed = 13;

    vector = wolfSearchBehavior.calcSpeedVector(currentDroneData=currentDroneData, targetGPS=Circle_Center_GPS, \
                radius=radius, radiusM=radiusM, wolfData=wolfDataArray, \
                averageAlignmentSpeed=averageAlignmentSpeed, bonusAlignmentSpeed=bonusAlignmentSpeed, \
                maxCohSepSpeed=maxCohSepSpeed, maxSpeed=maxSpeed);
    
    return vector;


def endWolfSearch():
    global Wolf_Search_Behavior;
    global Circle_Center_GPS;
    global Circle_Radius_GPS, Circle_Radius_Meters;
    global Start_Time, Spread_Time, Search_Time;


    Circle_Center_GPS = None;
    Circle_Radius_GPS, Circle_Radius_Meters = None, None;
    Start_Time, Spread_Time, Search_Time = None, None, None;
    Wolf_Search_Behavior = False;

def startConsensusDecision( circleCenterGPS, circleRadiusGPS, circleRadiusMeters, searchTimeS ):
    global Consensus_Decision_Behavior;
    global Circle_Center_GPS;
    global Circle_Radius_GPS, Circle_Radius_Meters;
    global Start_Time, Spread_Time, Search_Time;


    Circle_Center_GPS = circleCenterGPS;
    Circle_Radius_GPS, Circle_Radius_Meters = circleRadiusGPS, circleRadiusMeters;
    Search_Time = searchTimeS;
    Start_Time = time.time();
    Consensus_Decision_Behavior = True;

def consensusDecisionBehaviorGetVector(currentDroneData):
    radius = Circle_Radius_GPS
    radiusM = Circle_Radius_Meters
    targetGPS = Circle_Center_GPS
    wolfDataArray = wolfService.getWolfDataExC(DM_Drone_Name);
    # calcSpeedVector function variables
    averageAlignmentSpeed = 12;
    bonusAlignmentSpeed = 0;
    maxCohSepSpeed = 4;
    maxSpeed = 13
    
    vector = wolfSearchBehavior.calcSpeedVector(currentDroneData=currentDroneData, targetGPS=targetGPS, \
                radius=radius, radiusM=radiusM, wolfData=wolfDataArray, \
                averageAlignmentSpeed=averageAlignmentSpeed, bonusAlignmentSpeed=bonusAlignmentSpeed, \
                maxCohSepSpeed=maxCohSepSpeed, maxSpeed=maxSpeed);

    return vector;

def updateConsensusDecisionCenter(circleCenterGPS):
    global Circle_Center_GPS;

    Circle_Center_GPS = circleCenterGPS

def endConsensusDecision():
    global Consensus_Decision_Behavior;
    global Circle_Center_GPS;
    global Circle_Radius_GPS, Circle_Radius_Meters;
    global Start_Time, Spread_Time, Search_Time;

    Circle_Center_GPS = None;
    Circle_Radius_GPS, Circle_Radius_Meters = None, None;
    Start_Time, Search_Time = None, None;
    Consensus_Decision_Behavior = False;


def debugPrint( debugMessage):
    global DM_Drone_Name
    print("Wolf: ", DM_Drone_Name, " : " ,  debugMessage)