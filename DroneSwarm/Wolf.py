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
import os
from math import sqrt
from ImageProcessing import getInfo
from ImageProcessing import yolov5
import torch
# import constants
import Constants.configDrones as configDrones
import Constants.ros as ros
# import drone behavior
import DroneBehaviors.circleBehavior as circleBehavior;
import DroneBehaviors.lineBehaviorWolf as lineBehaviorWolf
import DroneBehaviors.collisionDetectionBehavior as collisionDetectionBehavior
# TODO: Investigate if we need to use a Lock while writing or reading global variables
import threading # for important # code running constantly
# TODO: Add custom message types
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.msg import requestLineBehavior
from airsim_ros_pkgs.msg import requestWolfSearchBehavior
from airsim_ros_pkgs.msg import requestConsensusDecisionBehavior
from airsim_ros_pkgs.msg import wolfCommunication
from airsim_ros_pkgs.msg import updateMap
from airsim_ros_pkgs.srv import getDroneData
from airsim_ros_pkgs.srv import sendCommand
from airsim_ros_pkgs.msg import GPS
# import helper function
import HelperFunctions.calcHelper as calcHelper
import HelperFunctions.pathWaypoints as pathWaypoints
import HelperFunctions.airSimHelper as airSimHelper
# import service requesters
import ServiceRequestors.wolfGetWolfData as wolfGetWolfData
import ServiceRequestors.instructWolf as instructWolf
import ServiceRequestors.checkGPU as checkGPU
# import rosHelper
import RosPublishHelper.MapHandlerPublishHelper as mapHandlerPublishHelper

# Environmental Variables
LOOP_NUMBER = configDrones.LOOP_NUMBER
MAX_TIME = configDrones.MAX_TIME
LOCAL_IP = configDrones.LOCAL_IP
MIN_CIRCLE_RADIUS_GPS = configDrones.MIN_CIRCLE_RADIUS_GPS 
MIN_CIRCLE_RADIUS_METERS = configDrones.MIN_CIRCLE_RADIUS_METERS
MAX_TURN_ANGLE = configDrones.MAX_TURN_ANGLE
SPEED_CHANGE = configDrones.SPEED_CHANGE
MIN_SPEED_FACTOR = configDrones.MIN_SPEED_FACTOR
MIN_DIFFRENCE_IN_RADIUS = configDrones.MIN_DIFFRENCE_IN_RADIUS
REQUIRED_SEPERATION_PERCENT = configDrones.REQUIRED_SEPERATION_PERCENT
MAX_CONSENSUS_ITERATION_NUMBER = configDrones.MAX_CONSENSUS_ITERATION_NUMBER
CONSENSUS_THRESHOLD = configDrones.CONSENSUS_THRESHOLD
CONSENSUS_ITERATION_LENGTH_SECONDS = configDrones.CONSENSUS_ITERATION_LENGTH_SECONDS
YOLO_CONFIDENCE = configDrones.YOLO_CONFIDENCE
MAX_COLLISION_TIME =configDrones.MAX_COLLISION_TIME
MIN_COLLISION_TIME = configDrones.MIN_COLLISION_TIME
# ros: topics
SLAM_MERGE_TOPIC = ros.SLAM_MERGE_TOPIC # TODO
WOLF_DATA_TOPIC = ros.WOLF_DATA_TOPIC
COMMAND_RESULT_TOPIC = ros.COMMAND_RESULT_TOPIC # TODO
COMMAND_TOPIC = ros.COMMAND_TOPIC # TODO
WOLF_COMMUNICATION_TOPIC = ros.WOLF_COMMUNICATION_TOPIC
MAP_HANDLER_TOPIC = ros.MAP_HANDLER_TOPIC
# ros: updateMapCommand types
FINAL_TARGET_POSITION = ros.FINAL_TARGET_POSITION
NEW_GPS_PREDICTION = ros.NEW_GPS_PREDICTION
UPDATE_DRONE_POSITION =  ros.UPDATE_DRONE_POSITION
# ros: topics: SIGNAL
IN_POSITION_SIGNAL = ros.IN_POSITION_SIGNAL
CONSENSUS_DECISION_SIGNAL = ros.CONSENSUS_DECISION_SIGNAL
AT_SPIRAL_WAYPOINT_SIGNAL = ros.AT_SPIRAL_WAYPOINT_SIGNAL
MAX_DRONE_WAIT_TIMER = ros.MAX_DRONE_WAIT_TIMER
# ros: services: service calls should be in the ServiceRequesteros folder
PROXIMITY_WOLF_SERVICE = ros.PROXIMITY_WOLF_SERVICE
# dynamic services:
WOLF_DRONE_SERVICE = ros.WOLF_DRONE_SERVICE
# task group name
SEARCH_TASK_GROUP = ros.SEARCH_TASK_GROUP
EMPTY_TASK_GROUP = ros.EMPTY_TASK_GROUP
EMPTY_CLUSTER = ros.EMPTY_CLUSTER

# global constant
GROUP_0_SEARCH = 'Constants/Group0Spiral.txt'
GROUP_1_SEARCH = 'Constants/Group1Spiral.txt'
# Internal Wolf Drone Memory Start -------------------------------------------
# Current pattern is ussing Global variable to allow access across threads (open to change)
# Global variables
lock = threading.Lock()
lockConsenus = threading.Lock()
DM_Drone_Name = None
DM_Wolfs_Cluster = [] # Drone will beassigned a group of drones to work with
WAYPOINT_COORDS = []
WAYPOINT_INDEX = 0
# Internal Wolf Drone Memory End -------------------------------------------
Line_Behavior = False
Cluster = EMPTY_CLUSTER
Task_Group = EMPTY_TASK_GROUP
Collision_Mode_Time = 0
# Memory for circle behavior
Wolf_Search_Behavior = False
Consensus_Decision_Behavior = False
Circle_Center_GPS = GPS()# gps cordinate
Circle_Radius_GPS = 0 # radius distance in gps
Circle_Radius_Meters = 0 # radius distance in meters
Start_Time = 0 # time
Spread_Time = 0 #  time in seconds # time to get in position 
Search_Time = 0 #  time in seconds # time to search
Cur_Consensus_Iteration_Number = 0
In_Position_WS = False
End_Loop = False
# consensus decion behavior
In_Position_CD = False
Success_Det_Count = 0
Fail_Det_Count = 0
Avg_Consensus_Decion_GPS = GPS() # gps data type
THRESHOLD = 16  # TODO: Needs to be scalable
COLLISION_DIRECTION_FACTOR = 5
Speed_Factor = 1
Previously_Had_Collision = False
Collision_Mode_Time_Length = 1
Drone_Max_Wait_Time_Start = time.time()
Consensus_Waypoint_History = []
Final_Target = GPS()
Final_Target_Found = False
# TODO: add tunning variables for behaviors (would be cool if we can train them)

# TODO: COMMENT AND REVIEW

# Main Process Start ----------------------------------------------
def wolfDroneController(droneName, droneCount, overseerCount):
    # set global vairable
    global DM_Drone_Name
    DM_Drone_Name = droneName
    global WAYPOINT_INDEX
    global End_Loop
    global Drone_Max_Wait_Time_Start
    global Collision_Mode_Time
    global Speed_Factor
    global Previously_Had_Collision
    global Collision_Mode_Time_Length
        # Globals for consensus
    global In_Position_WS, In_Position_CD, Start_Time
    global Cur_Consensus_Iteration_Number, Circle_Center_GPS
    depthImageCount = 0

    # loading yolov5
    # cwd = os.getcwd()
    # yoloPT = os.path.join(str(cwd), 'best.pt')
    # model = torch.hub.load('ultralytics/yolov5', 'custom', path=yoloPT, trust_repo=True)

    # Sets global values for wolf cluster and coordinate
    droneBoundary = math.floor(droneCount / overseerCount)
    remainder = droneCount % overseerCount

    # Assigns drone to a group, reads in respective coordinates
    if (int(droneName) < droneBoundary):
        debugPrint("Using group 0 search")
        readCoordFile(GROUP_0_SEARCH)
    else:
        debugPrint("Using group 1 search")
        readCoordFile(GROUP_1_SEARCH)

    # use this code to make print calls allowing you to know what process made the print statemnt
    debugPrint("Process started")

    # Create Node for wolf
    nodeName = "Wolf" + droneName
    rospy.init_node(nodeName, anonymous = True)
    debugPrint("Node initiated")

    # Wait until GPU is loaded
    checkGPU.checkGPUStatus()
    debugPrint("GPU Loaded")

    # Start all threads here (if you have to make one somwhere else bring it up with the team)
    # Starts service listeners for commands
    t = threading.Thread(target = wolfRosListeners, args=(droneName))
    t.start()

    # Setup collision directory
    imgDir = collisionDetectionBehavior.setupCollisionDirectory(droneName)

    # Create topic publishers
    wolfDataPublish = rospy.Publisher(WOLF_DATA_TOPIC, droneData, latch=True, queue_size=1)
    wolfCommPublish = rospy.Publisher(WOLF_COMMUNICATION_TOPIC, wolfCommunication, latch=True, queue_size=1)

    # Sets and connects to client and takes off drone
    client = takeOff(droneName)
    client.moveToZAsync(z=-5, velocity=8, vehicle_name = droneName).join()

    # start camera thread here
    t3 = threading.Thread(target = wolfCameraDetection, args=(droneName))
    t3.start()

    # Used to set up collision detection
    # collisionDetectionBehavior.setUpLidar(client, droneName)

    # Wolf Drone search loop Start
    i = 1
    debugPrint("Starting Search and Rescue loop")
    timeSpent = 0;
    runtime = time.time()
    while (i < LOOP_NUMBER):
        # If we receive end command, end the loop
        if (End_Loop):
            debugPrint("Ending loop")

            # Move to brian
            global Final_Target, Final_Target_Found 
            if (Final_Target_Found):
                client.moveToGPSAsync(Final_Target.latitude, Final_Target.longitude, altitude = 0, velocity = 10, vehicle_name=droneName)

            exit()
            break;
        elif (WAYPOINT_INDEX == (len(WAYPOINT_COORDS) - 1)):         # Checks if made it through all waypoints
            debugPrint("WAYPOINT_COORDS end")
            break;
        timeDiff = time.time() - runtime
        if (timeDiff > MAX_TIME):         # Check if we made it past max time
            debugPrint("max time")
            break;

        # Default movement and yaw set at 0
        vector = [0, 0] # dont move if nothing to do
        yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=(0)) # Set yaw to zero
        # Previously_Had_Collision = False

        start=time.time() # gather time data

        # Publishes to (WolfData) topic
        wolfDataPublisher(wolfDataPublish, client, droneName)

        isChangeVelocity = True
        droneSpeed = airSimHelper.getDroneSpeed(client, droneName)
        threshold = droneSpeed * 2.5
        tempTree = None

        # # Check if threshold is under min
        # if (threshold < 5):
        #     threshold = 5
            
        doCollision, closestObjectDistance, closestTree,closestTreeName= collisionDetectionBehavior.collisionAvoidanceCheck(client, droneName, threshold)
        timeDiff = time.time() - Collision_Mode_Time
        if((doCollision) and (closestTreeName != tempTree)):
            # debugPrint("Doing collision")
            Previously_Had_Collision = True
            Collision_Mode_Time = time.time()
            
            # distanceForTimeCalc = 0
            # if (closestObjectDistance < slightDeviationDistance):
            distanceForTimeCalc = closestObjectDistance
            # else:
            #     distanceForTimeCalc = slightDeviationDistance

            if(droneSpeed != 0):
                totalTime = distanceForTimeCalc / droneSpeed

                Collision_Mode_Time_Length = totalTime

            yaw_mode = airsim.YawMode(is_rate=True, yaw_or_rate=(0))
            colTime = time.time()
            vector = collisionDetectionBehavior.collisionAlgo(client,imgDir,droneName,closestObjectDistance,COLLISION_DIRECTION_FACTOR,closestTree)
            endTime = time.time() - colTime
            depthImageCount += 1
            tempTree = closestTreeName
            # text = "Collision avoidance time: " + str(Collision_Mode_Time_Length) + " Depth image count: " + str(depthImageCount) + "Collision algo time: " + str(endTime)
            # debugPrint(text)

            # client.moveByVelocityZAsync(vector[0], vector[1], -4, duration = COLLISION_DIRECTION_FACTOR, yaw_mode=yaw_mode, vehicle_name=droneName)

        elif (timeDiff < Collision_Mode_Time_Length):
            # debugPrint("Still doing collision, under time")
            isChangeVelocity = False

        # # TODO: Add in Drone behavior desion making
        elif (Consensus_Decision_Behavior): # Consensus Descion behavior
            currentDroneData = client.getMultirotorState(vehicle_name = droneName);
            
            vector = consensusDecisionBehaviorGetVector(currentDroneData);

            yawDegrees = circleBehavior.calcYaw(currentGPS=currentDroneData.gps_location, targetGPS=Circle_Center_GPS);
            yawDegrees = yawDegrees - 80
            yaw_mode  = airsim.YawMode(is_rate=False, yaw_or_rate=(yawDegrees));

            # check if time to end consensus Desension
            timeDiff = time.time() - Start_Time
            gpsCenter = Circle_Center_GPS
            currIterationNum = Cur_Consensus_Iteration_Number
            if (In_Position_CD):
                timeDiff = time.time() - Start_Time;
                # conisder locking this completly
                if (timeDiff > Search_Time): # make cosenus decion
                    with lockConsenus:
                        threshold = CONSENSUS_THRESHOLD
                        wolfDataArray = wolfGetWolfData.getWolfDataOfTaskGroup( Task_Group );
                        # check stage of conensus
                        if(currIterationNum < MAX_CONSENSUS_ITERATION_NUMBER):
                            # make new cosensus dec
                            passThreshold, newGPSCenter = calcHelper.calcNewConsenusGPS(wolfDataArray, gpsCenter, threshold, droneName, currIterationNum)
                            
                            if(currIterationNum == Cur_Consensus_Iteration_Number):
                                currIterationNum += 1
                                wolfSignalPublisherGPS(wolfCommPublish, client, str(Cluster), str(Task_Group), CONSENSUS_DECISION_SIGNAL, \
                                    signalGPS=newGPSCenter, iterationNumber=currIterationNum, result=passThreshold)

                                # updateConsensusDecisionCenter may change taskGroup name
                                # assign values locally
                                if(Consensus_Decision_Behavior):
                                    updateConsensusDecisionCenter(newGPSCenter, currIterationNum, result=passThreshold);
                            else:
                                debugPrint("alreadu updated consnsus")
                            
                        else:
                            # consenus decion target found
                            passThreshold, newGPSCenter = calcHelper.calcNewConsenusGPS(wolfDataArray, gpsCenter, threshold, droneName, currIterationNum)
                            wolfSignalPublisherGPS(wolfCommPublish, client, str(Cluster), str(Task_Group), CONSENSUS_DECISION_SIGNAL, \
                                    signalGPS=newGPSCenter, iterationNumber=currIterationNum, result=passThreshold)
                            
                            if(Consensus_Decision_Behavior):
                                    updateConsensusDecisionCenter(newGPSCenter, currIterationNum, result=passThreshold);
            elif (not In_Position_CD):
                wolfDataArray = wolfGetWolfData.getWolfDataOfTaskGroupExSelf(droneName, Task_Group)

                # ToDo : modify so consenus and search are diffrent
                radius = Circle_Radius_GPS
                isInPositon = circleBehavior.IsInPosition(currentGPS=currentDroneData.gps_location, \
                            targetGPS=Circle_Center_GPS, radius=radius, wolfData=wolfDataArray,  \
                            minDiffrenceInRadius=MIN_DIFFRENCE_IN_RADIUS, requiredSeperationPercent=REQUIRED_SEPERATION_PERCENT)
                
                isSameIteration = wolfGetWolfData.isTaskGroupSameIteration(Task_Group)
                if (isInPositon and isSameIteration):
                    # ToDO droneComminication
                    wolfSignalPublisher(wolfCommPublish, client, str(Cluster), str(Task_Group), IN_POSITION_SIGNAL, IsWS=False)
                    with lock:
                        Start_Time = time.time();
                        In_Position_CD = True

        elif (Wolf_Search_Behavior): # Wolf Search behavior
            
            currentDroneData = client.getMultirotorState(vehicle_name = droneName);

            vector = wolfSearchBehaviorGetVector(wolfCommPublish, client, currentDroneData);
            
            yawDegrees = circleBehavior.calcYaw(currentGPS=currentDroneData.gps_location, targetGPS=Circle_Center_GPS);
            yawDegrees = yawDegrees - 80
            
            yaw_mode  = airsim.YawMode(is_rate=False, yaw_or_rate=(yawDegrees));

            if (In_Position_WS):
                timeDiff = time.time() - Start_Time;
                if (timeDiff > Search_Time):
                    debugPrint("end wolf search")
                    endWolfSearch();

        elif (Line_Behavior): # Line_Behavior
            # Gets drones waypoint and vector movement
            newWaypoint = pathWaypoints.getNewWaypointWolf(droneName, WAYPOINT_INDEX, WAYPOINT_COORDS, Cluster)
            vector, curDroneAtWaypoint = lineBehaviorWolf.lineBehavior(client, int(droneName), newWaypoint)
            vectorTemp = 0

            # Changes to necessary values
            vectorTemp = vector[0]
            vector[0] = vector[1]
            vector[1] = vectorTemp
            
            # Updates speed based on factor
            vector[0] = vector[0] * Speed_Factor
            vector[1] = vector[1] * Speed_Factor

            # Calculates camera direction based on velocity      
            if (vector[1] != 0):
                yaw = math.atan2(vector[1], vector[0])
                degrees = math.degrees(yaw)
                yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=(degrees));
            else:
                degrees = 0
                yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=(degrees));

            # Checks if all drones made it to waypoint if cur drone is at waypoint
            if (curDroneAtWaypoint):
                # Calculates time from waypoint arrival, check threshold to move on anyway
                timeDiff = time.time() - Drone_Max_Wait_Time_Start;
                if (timeDiff > MAX_DRONE_WAIT_TIMER):
                    text = "Moving to waypoint took too long, moving to: " + str(WAYPOINT_INDEX + 1)
                    debugPrint(text)
                    cleanConsensusWaypointHistory()
                    wolfSignalWaypointPublisher(wolfCommPublish, client, str(Cluster), "", AT_SPIRAL_WAYPOINT_SIGNAL, WAYPOINT_INDEX + 1)

                # Otherwise we check the whole group
                else:
                    # Save our index and calculate if all drones are at waypoint
                    indexBeforeCalculation = WAYPOINT_INDEX
                    allDronesAtWaypoint, newWaypointIndex = lineBehaviorWolf.allDronesAtWaypoint(wolfCommPublish, client, WAYPOINT_INDEX, Cluster, WAYPOINT_COORDS)

                    # Checks if global changed during calculation, if not and all drones are at the waypoint, we update the waypoint
                    if ((indexBeforeCalculation == WAYPOINT_INDEX) and (allDronesAtWaypoint)):
                        with lock:
                            WAYPOINT_INDEX = newWaypointIndex
                        # debugPrint("Update waypoint to: " + str(WAYPOINT_INDEX))
                        cleanConsensusWaypointHistory()
            else:
                with lock:
                    Drone_Max_Wait_Time_Start = time.time();

        # Calculates acceleration based on previous collision detections
        # Speed_Factor = calcHelper.accelCalculator(Speed_Factor, Previously_Had_Collision, SPEED_CHANGE, MIN_SPEED_FACTOR)
        # Apply acceleration based on trees
        # vector = [vector[0] * Speed_Factor, vector[1] * Speed_Factor]

        # Calculates turning
         # Grabs current drones velocity in x and y
        curDroneData = client.getGpsData(vehicle_name = droneName)
        curDroneVelocity = [curDroneData.gnss.velocity.x_val, curDroneData.gnss.velocity.y_val]
        vector = calcHelper.turningCalculation(curDroneVelocity, vector, MAX_TURN_ANGLE)

        if (isChangeVelocity):
            client.moveByVelocityZAsync(vector[0], vector[1], -5, duration = 10, yaw_mode=yaw_mode, vehicle_name=droneName)

        # artifical loop delay (How fast the loop runs dictates the drones reaction speed)
        end = time.time();
        loopTime = end-start 
        timeSpent += loopTime;
        if (loopTime < 0.25):
            time.sleep(0.25 - loopTime)

        i+=1
    debugPrint("Average Loop Time in seconds: " + str(timeSpent / i))
    # Wolf Drone search loop End

# Main Process End ----------------------------------------------

# Theads Start ===========================================
def wolfRosListeners(droneName):
    serviceName = WOLF_DRONE_SERVICE + droneName
    service = rospy.Service(serviceName, sendCommand, commandResponse)
    rospy.Subscriber(ros.END_LOOP_TOPIC, String, handleEnd)
    rospy.Subscriber(ros.WOLF_COMMUNICATION_TOPIC, wolfCommunication, handleWolfSignal)
    rospy.spin()

# checks drone camera with yolo detection
def wolfCameraDetection(droneName):
    # Create topic publishers
    wolfMapPublisher = rospy.Publisher(MAP_HANDLER_TOPIC, updateMap, latch=True, queue_size=100)

    threadClient = airsim.MultirotorClient(LOCAL_IP)
    debugPrint("Starting wolfCameraDetection loop")
    i = 0
    timeSpent = 0
    runtime = time.time()

    cwd = os.getcwd()
    cwd = os.path.join(cwd[:cwd.index("DroneSwarm")],'DroneSwarm')
    dataDir_pass=os.path.join(str(cwd),'yolov5Images_pass')
    
    isExist=os.path.exists(dataDir_pass)

    if not isExist:
        # make directory if not already there
        os.makedirs(dataDir_pass)

    dataDir_fail=os.path.join(str(cwd),'yolov5Images_fails')
    isExist=os.path.exists(dataDir_fail)

    if not isExist:
        # make directory if not already there
        os.makedirs(dataDir_fail)

    while (i < LOOP_NUMBER):
        timeDiff = time.time() - runtime
        if (timeDiff > MAX_TIME):
            debugPrint(" Images taken: " + str(i))
            return
        elif (End_Loop):
            debugPrint(" Images taken: " + str(i))
            debugPrint("Ending loop")
            return
       
        start=time.time() # gather time data

        cameraName = None
        response, repsonseF, responseR = None, None, None

        # Retrieve image from airsim
        if (Consensus_Decision_Behavior and not In_Position_CD):
            time.sleep(0.3); # wait for positon
            continue;
        response = getInfo.getResponse(threadClient, droneName, "front-center")

        # wolf gps 
        gps = getInfo.getDroneGPS(droneName, threadClient)
        wolfEstimate, validDetection, passedConfidence = yolov5.runYolov5(response, dataDir_pass, dataDir_fail, vehicleName=droneName, confidanceMin=YOLO_CONFIDENCE, gps=gps, updateMapPublisher=wolfMapPublisher)

        # handle image detection result
        if(passedConfidence):
            formattedWolfEstimateGPS = calcHelper.fixDegenerateCoordinate(wolfEstimate)

            if(not Consensus_Decision_Behavior):
                # detection with no consensus behavior
                isSearched = isAlreadySearched(formattedWolfEstimateGPS, MIN_CIRCLE_RADIUS_GPS)
                if (not isSearched):
                    # request nearby drones
                    instructWolf.requestNearbyDronesConsensusDecision(circleCenterGPS=formattedWolfEstimateGPS, circleRadiusGPS=MIN_CIRCLE_RADIUS_GPS, \
                        circleRadiusMeters=MIN_CIRCLE_RADIUS_METERS, searchTimeS=CONSENSUS_ITERATION_LENGTH_SECONDS, taskGroup=(droneName + "Con"), \
                        clusterName=Cluster, dmDroneName=DM_Drone_Name, waypointIndex=WAYPOINT_INDEX, waypointCoords=WAYPOINT_COORDS )
                    # current Drone switch to consenus
                    startConsensusDecision( circleCenterGPS=formattedWolfEstimateGPS, circleRadiusGPS=MIN_CIRCLE_RADIUS_GPS, circleRadiusMeters=MIN_CIRCLE_RADIUS_METERS, \
                        searchTimeS=CONSENSUS_ITERATION_LENGTH_SECONDS, taskGroup=(droneName + "Con") )
            else:
                # detection update gps estimation
                updateConsensusDecisionStatus(isSuccess=True, successEstimateGPS=formattedWolfEstimateGPS)

        elif(validDetection):
            if(Consensus_Decision_Behavior):
                # no detection - currently consensus Behavior
                updateConsensusDecisionStatus(isSuccess=False, successEstimateGPS=None)
            # do nothing if not doing consenus

        # mock detection
        timeDiff = time.time() - runtime
        # if(not(Consensus_Decision_Behavior)):
        
        end = time.time();
        loopLen = end-start
        if (loopLen < 1):
            time.sleep(1 - loopLen);
        timeSpent += loopLen;
        i+=1
    debugPrint(" CameraDetection: Average Loop Time: " + str(timeSpent / i))
    return;


# Theads END ===========================================
# Ros handlers Start ++++++++++++++++++++++++++++++++++++
def handleWolfSignal(data):
    # Grabs strings from data object
    cluster = data.cluster
    taskGroup = data.taskGroup
    command = data.command
    signalGPS = data.signalGPS
    result = data.result

    global In_Position_WS, In_Position_CD, Start_Time, WAYPOINT_INDEX
    global Cur_Consensus_Iteration_Number
    #debugPrint("Wolf listend to wolf comm: " +  str(command))

    # Check if we are in the same cluster, or if cluster is empty
    if ((cluster != Cluster) and (cluster != EMPTY_CLUSTER)):
        return;

    # Check if we got at spiral waypoint signal
    if ((command == AT_SPIRAL_WAYPOINT_SIGNAL)):
        # Clear current consensus 
        cleanConsensusWaypointHistory()

        # If our current waypoint index is less than the one we received, use the most up to data spiral index
        spiralIndex = data.genericInt
        if(WAYPOINT_INDEX < spiralIndex):
            with lock:
                WAYPOINT_INDEX = spiralIndex

    if ((taskGroup != Task_Group) and (Task_Group != EMPTY_TASK_GROUP)):
        return;

    if(command == IN_POSITION_SIGNAL):
        # handle received is in position signal
        if (result):
            if(not In_Position_WS):
                Start_Time = time.time();
                #debugPrint("IN_Position set to true")
                with lock:
                    In_Position_WS = True
        else:
            if(not In_Position_CD):
                Start_Time = time.time();
                #debugPrint("IN_Position set to true")
                with lock:
                    In_Position_CD = True
        if(command == CONSENSUS_DECISION_SIGNAL):
            with lockConsenus:
                iterationNum = data.genericInt
                if (Consensus_Decision_Behavior and iterationNum > Cur_Consensus_Iteration_Number ):
                    updateConsensusDecisionCenter(signalGPS, iterationNum, result);


def handleEnd(data):
    global End_Loop
    if (data.data == "e"):
        debugPrint("Triggered end")
        End_Loop = True

def commandResponse(request):
    global DM_Drone_Name
    global Task_Group
    messageType = request.messageType
    lineInfo = request.lineBehaviorStart
    wolfSearchInfo = request.wolfSearchBehaviorStart
    consensusDecisionInfo = request.consensusDecisionBehaviorStart

    print("Got a command")
    
    lineString = str(lineInfo)
    # Find datatype with info, execute command based on who has data
    if (messageType == "RequestLineBehavior"):
        startLineBehavior(lineInfo.cluster)
        # debugPrint("Do line behavior")
        return True

    elif (messageType == "RequestWolfSearch"):
        # debugPrint("Do wolf search")

        # Return false is wolf is already in a task group
        if (Task_Group != EMPTY_TASK_GROUP):
            debugPrint("Unable to complete wolf search request")
            return False

        # Check if we got message from overseer
        if (wolfSearchInfo.taskGroup == EMPTY_TASK_GROUP):
            # if so create task group with wolf name
            taskGroup = SEARCH_TASK_GROUP + DM_Drone_Name
            debugPrint("Got request wolf search from Overseer: " + str(taskGroup))

            # Request nearby drones
            instructWolf.requestNearbyDronesWolfSearch(circleCenterGPS=wolfSearchInfo.circleCenterGPS, \
                circleRadiusGPS=wolfSearchInfo.circleRadiusGPS, circleRadiusMeters=wolfSearchInfo.circleRadiusMeters, \
                spreadTimeS=wolfSearchInfo.spreadTimeS, searchTimeS=wolfSearchInfo.searchTimeS, taskGroup=taskGroup,  \
                clusterName=Cluster, dmDroneName=DM_Drone_Name, waypointIndex=WAYPOINT_INDEX, waypointCoords=WAYPOINT_COORDS )

            # Start wolf search current drone
            startWolfSearch(wolfSearchInfo.circleCenterGPS, wolfSearchInfo.circleRadiusGPS, wolfSearchInfo.circleRadiusMeters, wolfSearchInfo.spreadTimeS, wolfSearchInfo.searchTimeS,  taskGroup)
            return True      
        # Got message from wolf, no need to request from nearby
        else:
            debugPrint("Got request wolf search from wolf: " + str(wolfSearchInfo.taskGroup))
            startWolfSearch(wolfSearchInfo.circleCenterGPS, wolfSearchInfo.circleRadiusGPS, wolfSearchInfo.circleRadiusMeters, wolfSearchInfo.spreadTimeS, wolfSearchInfo.searchTimeS,  wolfSearchInfo.taskGroup)
            return True

        return False

    elif (messageType == "RequestConsensusDecision"):
        circleCenterGPS = consensusDecisionInfo.circleCenterGPS
        isSearched = isAlreadySearched(circleCenterGPS, MIN_CIRCLE_RADIUS_GPS)
        if(Consensus_Decision_Behavior or isSearched):
            return False # already doing consenus behavior
        # debugPrint("Do consensus decision")

        circleRadiusGPS = consensusDecisionInfo.circleRadiusGPS
        circleRadiusMeters = consensusDecisionInfo.circleRadiusMeters
        searchTimeS = consensusDecisionInfo.searchTimeS
        taskGroup = consensusDecisionInfo.taskGroup
        startConsensusDecision( circleCenterGPS=circleCenterGPS, circleRadiusGPS=circleRadiusGPS, circleRadiusMeters=circleRadiusMeters, searchTimeS=searchTimeS, taskGroup=taskGroup );
        return True
    
    return False

# Ros handler functions end +++++++++++++++++++++++++++++++++++
# Ros Publishers start +++++++++++++++++++++++++++++++++++
def wolfSignalPublisher(pub, client, cluster, taskGroup, command, IsWS):
    # Creates droneMsg object and inserts values from AirSim apis
    wolfCommMessage = wolfCommunication()
    wolfCommMessage.cluster = cluster
    wolfCommMessage.taskGroup = taskGroup
    wolfCommMessage.command = command
    wolfCommMessage.result = IsWS

    # Publishes to topic
    pub.publish(wolfCommMessage)

def wolfSignalPublisherGPS(pub, client, cluster, taskGroup, command, signalGPS, iterationNumber, result):
    # Creates droneMsg object and inserts values from AirSim apis
    wolfCommMessage = wolfCommunication()
    wolfCommMessage.cluster = cluster
    wolfCommMessage.taskGroup = taskGroup
    wolfCommMessage.command = command
    wolfCommMessage.signalGPS = signalGPS
    wolfCommMessage.genericInt = iterationNumber
    wolfCommMessage.result = result
    # Publishes to topic
    pub.publish(wolfCommMessage)

# wolfCommunicationPublisher
def wolfSignalWaypointPublisher(pub, client, cluster, taskGroup, command, waypointIndex):
    # Creates droneMsg object and inserts values from AirSim apis
    wolfCommMessage = wolfCommunication()
    wolfCommMessage.cluster = cluster
    wolfCommMessage.taskGroup = taskGroup
    wolfCommMessage.command = command
    wolfCommMessage.genericInt = waypointIndex

    # Publishes to topic
    pub.publish(wolfCommMessage)

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
    droneMsg.cluster = Cluster
    droneMsg.taskGroup = Task_Group
    droneMsg.successDetCount = Success_Det_Count
    droneMsg.failDetCount = Fail_Det_Count
    droneMsg.avgConsensusDecionGPS = Avg_Consensus_Decion_GPS
    droneMsg.iterationNumber = Cur_Consensus_Iteration_Number

    # Publishes to topic
    pub.publish(droneMsg)
# Ros Publishers End +++++++++++++++++++++++++++++++++++

# TODO: Functions need to Refatctor +++++++++++++++++++++++++++++++++++

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
    with lock:
        WAYPOINT_COORDS = newList

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

# code for updating behaviors status
def startLineBehavior(clusterName):
    global Cluster
    global Line_Behavior
    with lock:
        Cluster = clusterName
        Line_Behavior = True

def endLineBehavior():
    global Cluster
    global Line_Behavior
    with lock:
        Cluster = EMPTY_CLUSTER
        Line_Behavior = False

def startWolfSearch( circleCenterGPS, circleRadiusGPS, circleRadiusMeters, spreadTimeS, searchTimeS, taskGroup):
    global Wolf_Search_Behavior;
    global Circle_Center_GPS;
    global Circle_Radius_GPS, Circle_Radius_Meters;
    global Start_Time, Spread_Time, Search_Time;
    global Task_Group;
    global In_Position_WS

    global DM_Drone_Name
    with lock:
        Circle_Center_GPS = circleCenterGPS;
        Circle_Radius_GPS, Circle_Radius_Meters = circleRadiusGPS, circleRadiusMeters;
        Search_Time = searchTimeS;
        Spread_Time = spreadTimeS;
        Task_Group = taskGroup;
        In_Position_WS = False
        Wolf_Search_Behavior = True;

def wolfSearchBehaviorGetVector(wolfCommPublish, client, currentDroneData):
    radius = Circle_Radius_GPS
    radiusM = Circle_Radius_Meters
    global In_Position_WS, Start_Time, DM_Drone_Name
    
    wolfDataArray = wolfGetWolfData.getWolfDataOfTaskGroupExSelf(DM_Drone_Name, Task_Group);

    if(not In_Position_WS):
        isInPositon = False
        try:
            isInPositon = circleBehavior.IsInPosition(currentGPS=currentDroneData.gps_location, \
                        targetGPS=Circle_Center_GPS, radius=radius, wolfData=wolfDataArray,  \
                        minDiffrenceInRadius=MIN_DIFFRENCE_IN_RADIUS, requiredSeperationPercent=REQUIRED_SEPERATION_PERCENT)
        except:
            debugPrint("isinPos error-----------------------------------------------")

        if (isInPositon):
            # ToDO droneComminication
            wolfSignalPublisher(wolfCommPublish, client, str(Cluster), str(Task_Group), IN_POSITION_SIGNAL, IsWS=True)
            Start_Time = time.time();
            debugPrint("IN Positionset to true")
            with lock:
                In_Position_WS = True
    
    if (In_Position_WS):
        try:
            timeDiff = time.time() - Start_Time;
        except:
            debugPrint("None Type error-----------------------------------------------")
        
        timeDiv = (Search_Time - timeDiff) / Search_Time
        radius = (radius - MIN_CIRCLE_RADIUS_GPS)*timeDiv + MIN_CIRCLE_RADIUS_GPS;
        radiusM = (radiusM - MIN_CIRCLE_RADIUS_METERS)*timeDiv + MIN_CIRCLE_RADIUS_METERS;

    # calcSpeedVector function variables
    averageAlignmentSpeed = 5 * Speed_Factor
    bonusAlignmentSpeed = 0 * Speed_Factor
    maxCohSepSpeed = 3 * Speed_Factor
    maxSpeed = 8 * Speed_Factor

    vectorR = lineBehaviorWolf.repulsion(client, int(DM_Drone_Name));
    vectorR = [vectorR[1], vectorR[0]]
    vector = circleBehavior.calcSpeedVector(currentDroneData=currentDroneData, targetGPS=Circle_Center_GPS, \
                radius=radius, radiusM=radiusM, wolfData=wolfDataArray, \
                averageAlignmentSpeed=averageAlignmentSpeed, bonusAlignmentSpeed=bonusAlignmentSpeed, \
                maxCohSepSpeed=maxCohSepSpeed, maxSpeed=maxSpeed);

    # vector = [vector[0] + vectorR[0], vector[1] + vectorR[1]]

    return calcHelper.applyMaxSpeed(vector, maxSpeed);


def endWolfSearch():
    global Wolf_Search_Behavior;
    global Circle_Center_GPS;
    global Circle_Radius_GPS, Circle_Radius_Meters;
    global Start_Time, Spread_Time, Search_Time;
    global Task_Group;
    with lock:
        Wolf_Search_Behavior = False;
        Circle_Center_GPS = None;
        Circle_Radius_GPS, Circle_Radius_Meters = None, None;
        Start_Time, Spread_Time, Search_Time = None, None, None;
        Task_Group = EMPTY_TASK_GROUP;

def startConsensusDecision( circleCenterGPS, circleRadiusGPS, circleRadiusMeters, searchTimeS, taskGroup):
    global Consensus_Decision_Behavior, Wolf_Search_Behavior;
    global Circle_Center_GPS;
    global Circle_Radius_GPS, Circle_Radius_Meters;
    global Start_Time, Spread_Time, Search_Time;
    global Task_Group;
    global In_Position_CD, Cur_Consensus_Iteration_Number;
    global Success_Det_Count, Fail_Det_Count

    updateConsensusWaypointHistory(circleCenterGPS)
    with lock:
        Circle_Center_GPS = circleCenterGPS;
        Circle_Radius_GPS, Circle_Radius_Meters = circleRadiusGPS, circleRadiusMeters;
        Search_Time = searchTimeS;
        Start_Time = time.time(); # TODO : remove
        Task_Group = taskGroup;
        Cur_Consensus_Iteration_Number = 0
        Success_Det_Count = 0
        Fail_Det_Count = 0
        In_Position_CD = False
        # debugPrint("start consenus decion")
        Consensus_Decision_Behavior = True;
        Wolf_Search_Behavior = False;

def updateConsensusDecisionStatus(isSuccess, successEstimateGPS):
    global Success_Det_Count, Fail_Det_Count, Avg_Consensus_Decion_GPS
    with lock:
        if (isSuccess):
            totalLon = Avg_Consensus_Decion_GPS.longitude * Success_Det_Count
            totalLat = Avg_Consensus_Decion_GPS.latitude * Success_Det_Count
            Success_Det_Count += 1
            Avg_Consensus_Decion_GPS.longitude = (totalLon + successEstimateGPS.longitude) / Success_Det_Count;
            Avg_Consensus_Decion_GPS.latitude = (totalLat + successEstimateGPS.latitude) / Success_Det_Count;
        else:
            Fail_Det_Count += 1

def consensusDecisionBehaviorGetVector(currentDroneData):
    global Speed_Factor
    radius = Circle_Radius_GPS
    radiusM = Circle_Radius_Meters
    targetGPS = Circle_Center_GPS
    wolfDataArray = wolfGetWolfData.getWolfDataOfTaskGroupExSelf(DM_Drone_Name, Task_Group);

    # code for updating consensus position
    # updateConsensusDecisionCenter(circleCenterGPS)
    # if(not In_Position_WS):

    # calcSpeedVector function variables
    averageAlignmentSpeed = 5 * Speed_Factor
    bonusAlignmentSpeed = 0 * Speed_Factor
    maxCohSepSpeed = 4 * Speed_Factor
    maxSpeed = 7 * Speed_Factor
    
    vector = circleBehavior.calcSpeedVector(currentDroneData=currentDroneData, targetGPS=targetGPS, \
                radius=radius, radiusM=radiusM, wolfData=wolfDataArray, \
                averageAlignmentSpeed=averageAlignmentSpeed, bonusAlignmentSpeed=bonusAlignmentSpeed, \
                maxCohSepSpeed=maxCohSepSpeed, maxSpeed=maxSpeed);

    return vector;

def updateConsensusDecisionCenter(circleCenterGPS, currIterationNum, result):
    global Circle_Center_GPS, Cur_Consensus_Iteration_Number, Start_Time, In_Position_CD;
    global Task_Group
    global Success_Det_Count, Fail_Det_Count
    global End_Loop
    global Final_Target, Final_Target_Found 
    if (result and currIterationNum < MAX_CONSENSUS_ITERATION_NUMBER):
        with lock:
            In_Position_CD = False
            Circle_Center_GPS = circleCenterGPS
            Start_Time = time.time()
            Cur_Consensus_Iteration_Number = currIterationNum
            Success_Det_Count = 0
            Fail_Det_Count = 0
            In_Position_CD = False
    else:
        if(currIterationNum >= MAX_CONSENSUS_ITERATION_NUMBER):
            if (result):
                # ToDo: success
                debugPrint("target GPS: " + str(circleCenterGPS))
                debugPrint("======================================================================================== \n \
                            Another day in paradise \n \
                            ======================================================================================== \n ")

                # update map with target
                mapHandlerPublishHelper.updateFinalTargetPosition( droneName=DM_Drone_Name, targetGPS=circleCenterGPS)

                # Updates globals
                Final_Target = circleCenterGPS
                Final_Target_Found = True

                # end code executiom
                endTaskPublish = rospy.Publisher(ros.END_LOOP_TOPIC, String, latch=True, queue_size=1)
                endTaskPublish.publish("e")
                with lock:
                    End_Loop = True
        # consenus decion no target found
        global Consensus_Decision_Behavior;
        if (Consensus_Decision_Behavior):
            endConsensusDecision();
        else:
            debugPrint("consenus already ended")
        

def endConsensusDecision():
    global Consensus_Decision_Behavior;
    global Circle_Center_GPS;
    global Circle_Radius_GPS, Circle_Radius_Meters;
    global Start_Time, Spread_Time, Search_Time;
    global Task_Group;
    with lock:
        debugPrint("End consenus: " + str(Wolf_Search_Behavior) + " taskGroup: " + str(Task_Group))
        Consensus_Decision_Behavior = False;
        # Circle_Center_GPS = None;
        # Circle_Radius_GPS, Circle_Radius_Meters = None, None;
        # Start_Time, Search_Time = None, None;
        Task_Group = "";
    
def updateConsensusWaypointHistory(waypoint):
    global Consensus_Waypoint_History
    with lock:
        Consensus_Waypoint_History.append(waypoint)

def cleanConsensusWaypointHistory():
    global Consensus_Waypoint_History
    with lock:
        Consensus_Waypoint_History = []

def isAlreadySearched(nextWaypoint, radius):
    global Consensus_Waypoint_History
    seperationRadius = radius / 2
    for pastWaypoint in Consensus_Waypoint_History:
        distance = calcHelper.calcDistanceBetweenGPS(nextWaypoint, pastWaypoint)
        if (distance < seperationRadius):
            return True;
    return False;

def debugPrint( debugMessage):
    global DM_Drone_Name
    print("Wolf: ", DM_Drone_Name, " : " ,  debugMessage)
