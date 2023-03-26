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
import os
import time
from math import sqrt
# import constants
import Constants.configDrones as configDrones
import Constants.ros as ros
# import drone behavior
import DroneBehaviors.circleBehavior as circleBehavior;
import DroneBehaviors.lineBehavior as lineBehavior
import DroneBehaviors.lineBehaviorWolf as lineBehaviorWolf
# TODO: Investigate if we need to use a Lock while writing or reading global variables
from threading import Timer # Use for interval checks with minimal code
from threading import Thread # USe for important code running constantly
# TODO: Add custom message types
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from airsim_ros_pkgs.msg import droneData
from airsim_ros_pkgs.msg import requestLineBehavior
from airsim_ros_pkgs.msg import requestWolfSearchBehavior
from airsim_ros_pkgs.msg import requestConsensusDecisionBehavior
from airsim_ros_pkgs.msg import wolfCommunication
from airsim_ros_pkgs.srv import getDroneData
from airsim_ros_pkgs.srv import sendCommand
from airsim_ros_pkgs.msg import GPS
import ServiceRequestors.wolfGetWolfData as wolfService
import ServiceRequestors.instructWolf as instructWolf
from ImageProcessing import getInfo
from ImageProcessing import yolov5
import torch
import HelperFunctions.calcHelper as calcHelper
import HelperFunctions.airSimHelper as airSimHelper
import DroneBehaviors.collisionDetectionBehavior as collisionDetectionBehavior

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
WOLF_SEARCH_REQUEST_HELP_DISTANCE_MULTIPLE = configDrones.WOLF_SEARCH_REQUEST_HELP_DISTANCE_MULTIPLE
CONSENSUS_DECISION_REQUEST_HELP_DISTANCE_MULTIPLE = configDrones.CONSENSUS_DECISION_REQUEST_HELP_DISTANCE_MULTIPLE
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

    # Start all threads here (if you have to make one somwhere else bring it up with the team)
    # Starts service listeners for commands
    t = Thread(target = wolfServiceListeners, args=(droneName))
    t.start()
    # Starts topic listeners for signals and end command
    t2 = Thread(target = wolfTopicListener)
    t2.start()

    # Setup collision directory
    imgDir = collisionDetectionBehavior.setupCollisionDirectory(droneName)

    # Create topic publishers
    wolfDataPublish = rospy.Publisher(WOLF_DATA_TOPIC, droneData, latch=True, queue_size=1)
    wolfCommPublish = rospy.Publisher(WOLF_COMMUNICATION_TOPIC, wolfCommunication, latch=True, queue_size=1)

    # Sets and connects to client and takes off drone
    client = takeOff(droneName)
    client.moveToZAsync(z=-3, velocity=8, vehicle_name = droneName).join()

    # start camera thread here
    # t3 = Thread(target = wolfCameraDetection, args=(droneName))
    # t3.start()

    # Globals for consensus
    global In_Position_WS, In_Position_CD, Start_Time
    global Cur_Consensus_Iteration_Number, Circle_Center_GPS

    # Wolf Drone search loop Start
    i = 1
    debugPrint("Starting Search and Rescue loop")
    timeSpent = 0;
    runtime = time.time()
    while (i < LOOP_NUMBER):
        # If we receive end command, end the loop
        if (End_Loop):
            debugPrint("Ending loop")
            exit()
            break;

        # Checks if made it through all waypoints
        if (WAYPOINT_INDEX == (len(WAYPOINT_COORDS) - 1)):
            debugPrint("WAYPOINT_COORDS end")
            break;

        # Check if we made it past max time
        timeDiff = time.time() - runtime
        if (timeDiff > MAX_TIME):
            debugPrint("max time")
            break;

        # Default movement and yaw set at 0
        vector = [0, 0] # dont move if nothing to do
        yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=(0)) # Set yaw to zero
        Previously_Had_Collision = False

        start=time.time() # gather time data

        # Publishes to (WolfData) topic
        wolfDataPublisher(wolfDataPublish, client, droneName)

        
        collisionAvoidance = False # set to true if need to do collision avoidance (open to better integration method)
        isChangeVelocity = True
        droneSpeed = airSimHelper.getDroneSpeed(client, droneName)
        threshold = droneSpeed * 2.5
        slightDeviation = airSimHelper.getDroneSpeed(client, droneName)*1.5 

        # # Check if threshold is under min
        # if (threshold < 5):
        #     threshold = 5
            
        collisionAvoidance, closestObjectDistance,slightDeviationDistance, slightFlag,sensorName = collisionDetectionBehavior.collisionAvoidanceCheck(client, droneName, threshold,slightDeviation)
        timeDiff = time.time() - Collision_Mode_Time
        if(collisionAvoidance):
            # debugPrint("Doing collision")
            Previously_Had_Collision = True
            Collision_Mode_Time = time.time()
            
            # distanceForTimeCalc = 0
            # if (closestObjectDistance < slightDeviationDistance):
            if(slightFlag):
                distanceForTimeCalc = slightDeviationDistance
            else:
                distanceForTimeCalc = closestObjectDistance
            # else:
            #     distanceForTimeCalc = slightDeviationDistance

            totalTime = distanceForTimeCalc / droneSpeed
            if (totalTime > MAX_COLLISION_TIME):
                totalTime = MAX_COLLISION_TIME
            elif (totalTime < MIN_COLLISION_TIME):
                totalTime = MIN_COLLISION_TIME

            Collision_Mode_Time_Length = totalTime

            yaw_mode = airsim.YawMode(is_rate=True, yaw_or_rate=(10));
            colTime = time.time()
            vector = collisionDetectionBehavior.collisionAlgo(client,imgDir,droneName,closestObjectDistance,slightDeviationDistance,droneSpeed,slightFlag,sensorName)
            endTime = time.time() - colTime
            depthImageCount += 1
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
            yawDegrees = yawDegrees - 90
            yaw_mode  = airsim.YawMode(is_rate=False, yaw_or_rate=(yawDegrees));

            # check if time to end consensus Desension
            timeDiff = time.time() - Start_Time
            gpsCenter = Circle_Center_GPS
            currIterationNum = Cur_Consensus_Iteration_Number
            if (In_Position_CD):
                timeDiff = time.time() - Start_Time;
                if (timeDiff > Search_Time): # make cosenus decion
                    threshold = CONSENSUS_THRESHOLD
                    wolfDataArray = wolfService.getWolfDataOfTaskGroup( Task_Group );
                    # check stage of conensus
                    if(currIterationNum < MAX_CONSENSUS_ITERATION_NUMBER):
                        # make new cosensus dec
                        passThreshold, newGPSCenter = calcHelper.calcNewConsenusGPS(wolfDataArray, gpsCenter, threshold, droneName)
                        
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
                        passThreshold, newGPSCenter = calcHelper.calcNewConsenusGPS(wolfDataArray, gpsCenter, threshold, droneName)
                        wolfSignalPublisherGPS(wolfCommPublish, client, str(Cluster), str(Task_Group), CONSENSUS_DECISION_SIGNAL, \
                                signalGPS=newGPSCenter, iterationNumber=currIterationNum, result=passThreshold)
                        
                        if(Consensus_Decision_Behavior):
                                updateConsensusDecisionCenter(newGPSCenter, currIterationNum, result=passThreshold);
            elif (not In_Position_CD):
                wolfDataArray = wolfService.getWolfDataOfTaskGroupExSelf(droneName, Task_Group)

                # ToDo : modify so consenus and search are diffrent
                radius = Circle_Radius_GPS
                isInPositon = circleBehavior.IsInPosition(currentGPS=currentDroneData.gps_location, \
                            targetGPS=Circle_Center_GPS, radius=radius, wolfData=wolfDataArray,  \
                            minDiffrenceInRadius=MIN_DIFFRENCE_IN_RADIUS, requiredSeperationPercent=REQUIRED_SEPERATION_PERCENT)
                if (isInPositon):
                    # ToDO droneComminication
                    wolfSignalPublisher(wolfCommPublish, client, str(Cluster), str(Task_Group), IN_POSITION_SIGNAL, IsWS=False)
                    Start_Time = time.time();
                    # debugPrint("IN Positionsetfor Consus: " + str(currIterationNum))
                    In_Position_CD = True

        elif (Wolf_Search_Behavior): # Wolf Search behavior
            
            currentDroneData = client.getMultirotorState(vehicle_name = droneName);

            vector = wolfSearchBehaviorGetVector(wolfCommPublish, client, currentDroneData);
            
            yawDegrees = circleBehavior.calcYaw(currentGPS=currentDroneData.gps_location, targetGPS=Circle_Center_GPS);
            yawDegrees = yawDegrees - 90
            
            yaw_mode  = airsim.YawMode(is_rate=False, yaw_or_rate=(yawDegrees));

            if (In_Position_WS):
                timeDiff = time.time() - Start_Time;
                if (timeDiff > Search_Time):
                    debugPrint("end wolf search")
                    endWolfSearch();

        elif (Line_Behavior): # Line_Behavior
            # Gets drones waypoint and vector movement
            newWaypoint = getNewWaypoint(droneName)
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
                    wolfSignalWaypointPublisher(wolfCommPublish, client, str(Cluster), '', AT_SPIRAL_WAYPOINT_SIGNAL, WAYPOINT_INDEX + 1)

                # Otherwise we check the whole group
                else:
                    allDronesAtWaypoint(wolfCommPublish, client)
            else:
                Drone_Max_Wait_Time_Start = time.time();

        # TODO: Apply turning to desired action
        # TODO: Overide other behaviors if collisionAvoidance is needed

        # Grabs current drones velocity in x and y
        curDroneData = client.getGpsData(vehicle_name = droneName)
        curDroneVelocity = [curDroneData.gnss.velocity.x_val, curDroneData.gnss.velocity.y_val]

        # Calculates acceleration based on previous collision detections
        Speed_Factor = calcHelper.accelCalculator(Speed_Factor, Previously_Had_Collision, SPEED_CHANGE, MIN_SPEED_FACTOR)

        # Apply acceleration based on trees
        # vector = [vector[0] * Speed_Factor, vector[1] * Speed_Factor]

        # Calculates turning
        vector = calcHelper.turningCalculation(curDroneVelocity, vector, MAX_TURN_ANGLE)

        if (isChangeVelocity):
            client.moveByVelocityZAsync(vector[0], vector[1], -3, duration = 10, yaw_mode=yaw_mode, vehicle_name=droneName)
        
        # Add in artifical loop delay (How fast the loop runs dictates the drones reaction speed)
        
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
def wolfServiceListeners(droneName):
    serviceName = WOLF_DRONE_SERVICE + droneName
    service = rospy.Service(serviceName, sendCommand, commandResponse)
    rospy.spin()

def wolfTopicListener():
    rospy.Subscriber(ros.END_LOOP_TOPIC, String, handleEnd)
    rospy.Subscriber(ros.WOLF_COMMUNICATION_TOPIC, wolfCommunication, handleWolfSignal)
    rospy.spin()

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
            WAYPOINT_INDEX = spiralIndex

    if ((taskGroup != Task_Group) and (Task_Group != EMPTY_TASK_GROUP)):
        return;

    if(command == IN_POSITION_SIGNAL):
        # handle received is in position signal
        if (result):
            if(not In_Position_WS):
                Start_Time = time.time();
                #debugPrint("IN_Position set to true")
                In_Position_WS = True
        else:
            if(not In_Position_CD):
                Start_Time = time.time();
                #debugPrint("IN_Position set to true")
                In_Position_CD = True

    if(command == CONSENSUS_DECISION_SIGNAL):
        iterationNum = data.genericInt
        if (Consensus_Decision_Behavior and iterationNum > Cur_Consensus_Iteration_Number ):
            updateConsensusDecisionCenter(signalGPS, iterationNum, result);


def handleEnd(data):
    global End_Loop
    if (data.data == "e"):
        debugPrint("Triggered end")
        End_Loop = True

# checks drone camera with yolo detection
def wolfCameraDetection(droneName):
    threadClient = airsim.MultirotorClient(LOCAL_IP)
    debugPrint("Starting wolfCameraDetection loop")
    i = 0
    timeSpent = 0
    runtime = time.time()

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

        # Switches camera for object detector depending on behavior
        if Wolf_Search_Behavior:
            cameraName="frontright"
            responseF = getInfo.getResponse(threadClient, droneName, "front")
            responseR = getInfo.getResponse(threadClient, droneName, "right")
        elif Consensus_Decision_Behavior:
            if (not In_Position_CD):
                time.sleep(0.3); # 
                continue;
            cameraName="right"
            response = getInfo.getResponse(threadClient, droneName, "right")
        else: # If at spawn or in line, use front camera
            cameraName="front"
            response = getInfo.getResponse(threadClient, droneName, "front")


        validDetection=False


        if cameraName=="frontright":
            cameraName="front"
            wolfEstimate, validDetection, passedConfidence = yolov5.runYolov5(threadClient, responseF, cameraName, droneName, YOLO_CONFIDENCE)
            # if front camera retrieves null detection, then run yolo on right camera
            if(wolfEstimate[0]==None and wolfEstimate[1]==None):
                cameraName="right"
                wolfEstimate, validDetection, passedConfidence = yolov5.runYolov5(threadClient, responseR, cameraName, droneName, YOLO_CONFIDENCE)
        else:
            wolfEstimate, validDetection, passedConfidence = yolov5.runYolov5(threadClient, response, cameraName, droneName, YOLO_CONFIDENCE)

        if(passedConfidence):
            formattedWolfEstimateGPS = calcHelper.fixDegenerateCoordinate(wolfEstimate)

            if(not Consensus_Decision_Behavior):
                # detection with no consensus behavior
                isSearched = isAlreadySearched(formattedWolfEstimateGPS, MIN_CIRCLE_RADIUS_GPS)
                if (not isSearched):
                    circleRadiusGPS = MIN_CIRCLE_RADIUS_GPS
                    circleRadiusMeters = MIN_CIRCLE_RADIUS_METERS
                    searchTimeS = CONSENSUS_ITERATION_LENGTH_SECONDS
                    taskGroup = droneName + "Con"
                    # request nearby drones
                    requestNearbyDronesConsensusDecision(circleCenterGPS=formattedWolfEstimateGPS, circleRadiusGPS=circleRadiusGPS, circleRadiusMeters=circleRadiusMeters, searchTimeS=searchTimeS,  taskGroup=taskGroup)
                    # current Drones
                    startConsensusDecision( circleCenterGPS=formattedWolfEstimateGPS, circleRadiusGPS=circleRadiusGPS, circleRadiusMeters=circleRadiusMeters, searchTimeS=searchTimeS, taskGroup=taskGroup )


            else:
                # ToDo: use locl
                global Success_Det_Count, Avg_Consensus_Decion_GPS
                totalLon = Avg_Consensus_Decion_GPS.longitude * Success_Det_Count
                totalLat = Avg_Consensus_Decion_GPS.latitude * Success_Det_Count
                Success_Det_Count += 1
                Avg_Consensus_Decion_GPS.longitude = (totalLon + formattedWolfEstimateGPS.longitude) / Success_Det_Count;
                Avg_Consensus_Decion_GPS.latitude = (totalLat + formattedWolfEstimateGPS.latitude) / Success_Det_Count;

        elif(validDetection):
            if(Consensus_Decision_Behavior):
                # no detection - currently consensus Behavior
                global Fail_Det_Count
                Fail_Det_Count += 1


        # mock detection
        timeDiff = time.time() - runtime
        # if(not(Consensus_Decision_Behavior)):
        if(False):
            if(timeDiff > 18 and droneName == '1'):
                # targetP is estimated gps position
                targetP = threadClient.getMultirotorState(vehicle_name = "target")
                circleCenterGPS = targetP.gps_location
                circleRadiusGPS = MIN_CIRCLE_RADIUS_GPS
                circleRadiusMeters = MIN_CIRCLE_RADIUS_METERS
                searchTimeS = 100
                taskGroup = droneName + "Con"
                startConsensusDecision( circleCenterGPS=circleCenterGPS, circleRadiusGPS=circleRadiusGPS, circleRadiusMeters=circleRadiusMeters, searchTimeS=searchTimeS, taskGroup=taskGroup )
                # ToDO addd function call to return list of availalbe drones
                # THis is Hardcoded need to replace
                requestNearbyDronesConsensusDecision(circleCenterGPS, circleRadiusGPS, circleRadiusMeters, searchTimeS,  taskGroup)

        end = time.time();
        loopLen = end-start
        if (loopLen < 1):
            time.sleep(1 - loopLen);
        timeSpent += loopLen;
        i+=1
    debugPrint("Actual loop size:" + str(i))
    debugPrint("Expected loop size:" + str(LOOP_NUMBER))
# startConsensusDecision( circleCenterGPS=targetP.gps_location, circleRadiusGPS=MIN_CIRCLE_RADIUS_GPS*2, circleRadiusMeters=MIN_CIRCLE_RADIUS_METERS*2, searchTimeS=100 );
    debugPrint(" CameraDetection: Average Loop Time: " + str(timeSpent / i))
    return;


# Theads END ===========================================

# TODO: Functions need to Refatctor +++++++++++++++++++++++++++++++++++

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
            #debugPrint("Requesting neaby wolfs")
            requestNearbyDronesWolfSearch(wolfSearchInfo.circleCenterGPS, wolfSearchInfo.circleRadiusGPS, wolfSearchInfo.circleRadiusMeters, wolfSearchInfo.spreadTimeS, wolfSearchInfo.searchTimeS, taskGroup)
            

            # Start wolf search
            #debugPrint("Doing search")
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


# wolfCommunicationPublisher
# signal
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

    # Publishes to topic
    pub.publish(droneMsg)

# TODO: MIGRATE OUT SOMEWHERE

# Requests nearby drones to do search
# NEEds to be tested
def requestNearbyDronesWolfSearch(circleCenterGPS, circleRadiusGPS, circleRadiusMeters, spreadTimeS, searchTimeS,  taskGroup):
    endWaypoint = getNewWaypoint(DM_Drone_Name)
    startWaypoint = getLastWaypoint(DM_Drone_Name)
    startGPS = calcHelper.fixDegenerateCoordinate(startWaypoint)
    endGPS = calcHelper.fixDegenerateCoordinate(endWaypoint)
    isEmpty, clusterCenterGPS = wolfService.getWolfClusterCenterGPS(clusterName=Cluster)

    # Checks if there is no cluster center
    if (isEmpty):
        return

    AvgClusterGPSOnLine = calcHelper.mapGPSPointOnLine(startGPS, endGPS, clusterCenterGPS)
    
    
    wolfDataArray = wolfService.getWolfDataOfCluster(wolfNameToExclude=DM_Drone_Name, clusterName=Cluster)

    # Go through each drones and request to nearby drones in cluster
    assignedDroneNum = 1
    for drone in wolfDataArray:
        # limit drone assignment number
        assignedDroneNum
        maxDroneAssignment = math.ceil(circleRadiusGPS / MIN_CIRCLE_RADIUS_GPS) # round up
        if (assignedDroneNum >= maxDroneAssignment):
            debugPrint("max number of drones assigned Wolf Search: " + str(maxDroneAssignment))
            break;

        droneGPSOnLine = calcHelper.mapGPSPointOnLine(startGPS, endGPS, drone)
        droneDistanceFromStart = calcHelper.calcDistanceBetweenGPS(startGPS, droneGPSOnLine)
        avgClusterDistanceFromStart = calcHelper.calcDistanceBetweenGPS(startGPS, AvgClusterGPSOnLine)

        distanceFromAverage = calcHelper.calcDistanceBetweenGPS(droneGPSOnLine, AvgClusterGPSOnLine)
        distance = calcHelper.calcDistanceBetweenGPS(circleCenterGPS, drone)

        if (droneDistanceFromStart > avgClusterDistanceFromStart):
            distance -= distanceFromAverage
        else:
            distance += distanceFromAverage
        
        minDistanceFromWaypoint = circleRadiusMeters * WOLF_SEARCH_REQUEST_HELP_DISTANCE_MULTIPLE
        # If a drone is within a certain radius of requestor 
        if ((distance < minDistanceFromWaypoint) and (drone.taskGroup == EMPTY_TASK_GROUP)):
            serviceName = WOLF_DRONE_SERVICE + drone.droneName
            requestStatus = instructWolf.sendWolfSearchBehaviorRequest(serviceName, circleCenterGPS, circleRadiusGPS, circleRadiusMeters, spreadTimeS, searchTimeS,  taskGroup)
            if(requestStatus):
                assignedDroneNum += 1

# TODO: MIGRATE OUT SOMEWHERE

def requestNearbyDronesConsensusDecision(circleCenterGPS, circleRadiusGPS, circleRadiusMeters, searchTimeS,  taskGroup):
    endWaypoint = getNewWaypoint(DM_Drone_Name)
    startWaypoint = getLastWaypoint(DM_Drone_Name)
    startGPS = calcHelper.fixDegenerateCoordinate(startWaypoint)
    endGPS = calcHelper.fixDegenerateCoordinate(endWaypoint)
    isEmpty, clusterCenterGPS = wolfService.getWolfClusterCenterGPS(clusterName=Cluster)

    if (isEmpty):
        return
    
    AvgClusterGPSOnLine = calcHelper.mapGPSPointOnLine(startGPS, endGPS, clusterCenterGPS)
    
    
    wolfDataArray = wolfService.getWolfDataOfCluster(wolfNameToExclude=DM_Drone_Name, clusterName=Cluster)

    # Go through each drones and request to nearby drones in cluster
    
    for drone in wolfDataArray:

        droneGPSOnLine = calcHelper.mapGPSPointOnLine(startGPS, endGPS, drone)
        droneDistanceFromStart = calcHelper.calcDistanceBetweenGPS(startGPS, droneGPSOnLine)
        avgClusterDistanceFromStart = calcHelper.calcDistanceBetweenGPS(startGPS, AvgClusterGPSOnLine)

        distanceFromAverage = calcHelper.calcDistanceBetweenGPS(droneGPSOnLine, AvgClusterGPSOnLine)
        distance = calcHelper.calcDistanceBetweenGPS(circleCenterGPS, drone)

        if (droneDistanceFromStart > avgClusterDistanceFromStart):
            distance -= distanceFromAverage
        else:
            distance += distanceFromAverage
        
        minDistanceFromWaypoint = circleRadiusMeters * CONSENSUS_DECISION_REQUEST_HELP_DISTANCE_MULTIPLE
        # If a drone is within a certain radius of requestor 
        if ((distance < minDistanceFromWaypoint)):
            serviceName = WOLF_DRONE_SERVICE + drone.droneName
            requestStatus = instructWolf.sendConsensusDecisionBehaviorRequest(serviceName, circleCenterGPS, circleRadiusGPS, circleRadiusMeters, searchTimeS, taskGroup)
            if(requestStatus):
                assignedDroneNum += 1
            
            # print("Request bool:", requestStatus, "From drone", droneName)

# TODO: MOVE TO HELPER

# Function get drones subwaypoint based on index
def getNewWaypoint(droneName):
    # Created global waypoints
    global WAYPOINT_INDEX
    # print("DroneName: ", droneName, "Current waypoint index", WAYPOINT_INDEX)
    currentWaypoint = WAYPOINT_COORDS[WAYPOINT_INDEX]

    # Calculates subwaypoint if past first spawn waypoint
    if (WAYPOINT_INDEX >= 1):
        previousWaypoint = WAYPOINT_COORDS[WAYPOINT_INDEX-1]
        radius = 0.0001
        currentWaypoint = lineBehaviorWolf.subWaypointCalculator(currentWaypoint, previousWaypoint, radius, droneName, Cluster)

    return currentWaypoint

# TODO: MOVE TO HELPER

# Grabs subwaypoint based on waypoints and droneName
def getLastWaypoint(droneName):
    # Created global waypoints
    global WAYPOINT_INDEX
    # print("DroneName: ", droneName, "Current waypoint index", WAYPOINT_INDEX)
    if (WAYPOINT_INDEX < 2):
        currentWaypoint = WAYPOINT_COORDS[0]
    else:
        currentWaypoint = WAYPOINT_COORDS[WAYPOINT_INDEX - 1]

        previousWaypoint = WAYPOINT_COORDS[WAYPOINT_INDEX - 2]
        radius = 0.0001
        currentWaypoint = lineBehaviorWolf.subWaypointCalculator(currentWaypoint, previousWaypoint, radius, droneName, Cluster)

    return currentWaypoint

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

# TODO: PUSH TO LINE BEHAVIOR, RETURN INDEX VALUE

def allDronesAtWaypoint(wolfCommPublish, client):
    global WAYPOINT_INDEX
    global Cluster
    wolfInfoArray = wolfService.getWolfState()

    currentWaypoint = WAYPOINT_COORDS[WAYPOINT_INDEX]
    newWaypoint = currentWaypoint
    waypointIndexBeforeCheck = WAYPOINT_INDEX

    # Check if made it to spawn and drone has a cluster
    if ((WAYPOINT_INDEX >= 1) and (Cluster != EMPTY_CLUSTER)):
        radius = 0.0001
        previousWaypoint = WAYPOINT_COORDS[WAYPOINT_INDEX-1]

        # Gets list of subwaypoints
        subWaypointList = lineBehaviorWolf.getSubwaypointList(currentWaypoint, previousWaypoint, radius, Cluster)

        # Get cluster and size information
        wolfCluster = wolfService.getWolfDataOfClusterWCurWolf(Cluster)
        clusterSize = len(wolfCluster)

        # Checks if all wolf in cluster made it to waypoint
        for wolf in wolfCluster:
            # Gets wolfs subwaypoint based on lane
            lane = int(wolf.droneName) % clusterSize
            wolfSubwaypoint = subWaypointList[lane]

            # Get difference between waypoint and drones actual location
            distance = sqrt( ((wolf.longitude - float(wolfSubwaypoint[0])) ** 2) + ((wolf.latitude - float(wolfSubwaypoint[1])) ** 2))

            # debugPrint(str(distance))
            # If any of the drones are not near waypoint, return false
            if (distance > 0.00015):
                return 0
    # Check that drones are spawned in an contain a cluster before moving on
    elif((Cluster != EMPTY_CLUSTER) and (WAYPOINT_INDEX == 0)):
        wolfCluster = wolfService.getWolfDataOfClusterWCurWolf(Cluster)

        for wolf in wolfCluster:
            xDifference = wolf.longitude - float(WAYPOINT_COORDS[WAYPOINT_INDEX][0])
            yDifference = wolf.latitude - float(WAYPOINT_COORDS[WAYPOINT_INDEX][1])

            # If any of the drones are out of bounds, return false
            if ((abs(xDifference) > 0.0004) or (abs(yDifference) > 0.0004)):
                return 0

        # Check if our global value has changed
        if (waypointIndexBeforeCheck == WAYPOINT_INDEX):    
            WAYPOINT_INDEX = WAYPOINT_INDEX + 1
            cleanConsensusWaypointHistory()
            # Communicate to other drones in cluster new waypoint 
            wolfSignalWaypointPublisher(wolfCommPublish, client, str(Cluster), EMPTY_TASK_GROUP, AT_SPIRAL_WAYPOINT_SIGNAL, WAYPOINT_INDEX)

        return 1
        # debugPrint("Drone spawned")

    # If drone is in cluster and passed checks, increment waypoint
    if ((Cluster != EMPTY_CLUSTER)):
        # Check if our global value has changed
        if (waypointIndexBeforeCheck == WAYPOINT_INDEX):    
            WAYPOINT_INDEX = WAYPOINT_INDEX + 1
            cleanConsensusWaypointHistory()
            # Communicate to other drones in cluster new waypoint 
            wolfSignalWaypointPublisher(wolfCommPublish, client, str(Cluster), EMPTY_TASK_GROUP, AT_SPIRAL_WAYPOINT_SIGNAL, WAYPOINT_INDEX)
            # debugPrint("Made it to waypoint")

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

def startLineBehavior(clusterName):
    global Cluster
    global Line_Behavior
    Cluster = clusterName
    Line_Behavior = True

def endLineBehavior():
    global Cluster
    global Line_Behavior
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
    
    wolfDataArray = wolfService.getWolfDataOfTaskGroupExSelf(DM_Drone_Name, Task_Group);

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

def consensusDecisionBehaviorGetVector(currentDroneData):
    global Speed_Factor
    radius = Circle_Radius_GPS
    radiusM = Circle_Radius_Meters
    targetGPS = Circle_Center_GPS
    wolfDataArray = wolfService.getWolfDataOfTaskGroupExSelf(DM_Drone_Name, Task_Group);

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
    if (result and currIterationNum < MAX_CONSENSUS_ITERATION_NUMBER):
        Start_Time = time.time();
        Cur_Consensus_Iteration_Number = currIterationNum
        Success_Det_Count = 0
        Fail_Det_Count = 0
        Circle_Center_GPS = circleCenterGPS
        In_Position_CD = False
    else:
        if(currIterationNum >= MAX_CONSENSUS_ITERATION_NUMBER):
            if (result):
                # ToDo: success
                debugPrint("target GPS: " + str(circleCenterGPS))
                debugPrint("======================================================================================== \n \
                            Another day in paradise \n \
                            ======================================================================================== \n ")
                endTaskPublish = rospy.Publisher(ros.END_LOOP_TOPIC, String, latch=True, queue_size=1)
                endTaskPublish.publish("e")
        # consenus decion no target found
        # debugPrint("Cluster: " + str(Cluster))
        # debugPrint("taskGroup: " + str(Task_Group))
        # debugPrint("consenus failed")
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
    debugPrint("End consenus: " + str(Wolf_Search_Behavior) + " taskGroup: " + str(Task_Group))
    Consensus_Decision_Behavior = False;
    # Circle_Center_GPS = None;
    # Circle_Radius_GPS, Circle_Radius_Meters = None, None;
    # Start_Time, Search_Time = None, None;
    Task_Group = "";
    
def updateConsensusWaypointHistory(waypoint):
    global Consensus_Waypoint_History
    Consensus_Waypoint_History.append(waypoint)

def cleanConsensusWaypointHistory():
    global Consensus_Waypoint_History
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
