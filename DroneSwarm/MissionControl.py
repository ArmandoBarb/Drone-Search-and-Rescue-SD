# NECESSARY NODES:
# One node for "MissionControl"
# Subscribes to (CommandResult) topic
# Subscribes to (SlamMerge) topic
# Publishes to (Command) topic

import airsim
import time
import rospy
import torch
import os
from threading import Timer
from threading import Thread
from Overseer import overseerDroneController
from Wolf import wolfDroneController
from ProximityWolf import startProximityWolf
from ProximityOverseer import startProximityOverseer
from YoloGPU import startYoloGPU
from ctypes import Structure, c_int
from multiprocessing.sharedctypes import Array
import multiprocessing as mp
from std_msgs.msg import String
from DroneBehaviors.spiralSearchCreator import createWaypoints
# import constants
import Constants.configDrones as configDrones
import Constants.ros as ros
from HelperFunctions import clusterHelper
from airsim_ros_pkgs.srv import requestGPU

# Environmental Variables
LOOP_NUMBER = configDrones.LOOP_NUMBER
LOCAL_IP = configDrones.LOCAL_IP

# ros topics
COMMAND_TOPIC = ros.COMMAND_TOPIC
COMMAND_RESULT_TOPIC = ros.COMMAND_RESULT_TOPIC
SLAM_MERGE_TOPIC = ros.SLAM_MERGE_TOPIC

# Main Process Start ----------------------------------------------
# Main function for mission control
print('Starting Mission Control')
if __name__ == '__main__': # Only runs if this is main processes
    mp.set_start_method('fork') # windows specific. Change based on OS.

    createWaypoints()

    # overseerCount = mp.cpu_count() - 5

    overseerCount = 2
    wolfCount = 8

    # apply infrared to overseers
    client = airsim.MultirotorClient(LOCAL_IP)
    clusterHelper.applyInfrared(client)

    # # loading yolov5
    # cwd = os.getcwd()
    # yoloPT = os.path.join(str(cwd), 'best.pt')
    # model = torch.hub.load('ultralytics/yolov5', 'custom', path=yoloPT, trust_repo=True)

    # TODO: start all procecess for ros Nodes here

    # Starts node for gpu yolo processing
    mp.Process(target=startYoloGPU, args=()).start()
    time.sleep(10);

    rospy.wait_for_service(GPU_SERVICE)
    response = rospy.ServiceProxy(GPU_SERVICE, requestGPU)
    responseObject = response("")
    if (responseObject.success == True):
        print("GPU model is ready")

    # Start wolf proximity subscriber and wolf nodes
    mp.Process(target=startProximityWolf, args=(wolfCount,)).start()
    time.sleep(1);
    for wolf in range(wolfCount): # str(x) = the vechical_name of the drone
        droneName = str(wolf)
        mp.Process(target=wolfDroneController, args=(droneName,wolfCount,overseerCount)).start()

    # Start overseer proximity subscriber and overseer nodes

    mp.Process(target=startProximityOverseer, args=(overseerCount,)).start()
    for overseer in range(overseerCount):
        droneNum = str(overseer)
        droneName = "Overseer_" + droneNum
        mp.Process(target=overseerDroneController, args=(droneName,overseerCount, wolfCount)).start()


    # One node for "MissionControl"
    nodeName = "MissionControl"
    rospy.init_node(nodeName, anonymous = True)

    name = input('Enter "End" to finish program?\n')
    endTaskPublish = rospy.Publisher(ros.END_LOOP_TOPIC, String, latch=True, queue_size=1)
    if (name == "e"):
        endTaskPublish.publish("e")

    # TODO: PUBLISHERS AND SUBSCRIBERS FOR MISSION CONTROL
    # # Subscribes to (Command) topic
    # t = Timer(1, commandResultSub, args=(nodeName)) # every 1.0 seconds
    # t.start()
    #
    # # Subscribes to (SlamMerge) topic
    # t2 = Timer(1, slamMergeSub, args=(nodeName)) # every 1.0 seconds
    # t2.start()
    #
    # # Publishes to (Command) topic
    # commandPublisher()

# Main Process End ----------------------------------------------

# TODO: Functions need to Refatctor +++++++++++++++++++++++++++++++++++

# Publishes to command topic
def commandPublisher():
    pub = rospy.Publisher(COMMAND_TOPIC, String, latch=True, queue_size=1)
    updateState(pub, client)

    i = 0
    while (not rospy.is_shutdown() and i < LOOP_NUMBER):
        updateState(pub, client, droneName)
        time.sleep(1)
        i+=1

# Publishes data to command topic
def updateState(pub, client):
    # TODO: UPDATE STATE FUNCTIONALITY
    stateData = "True" # Temporary state data for testing
    pub.publish(stateData)

# Subscribes to (Command) topic
def commandResultSub():
    rospy.Subscriber(COMMAND_RESULT_TOPIC, String, updateCommandResult, (droneCount, client))
    rospy.spin()

# Takes in strings from the (Command) topic for processing
def updateCommandResult(data, args):
    print()
    # TODO: ADD IN CODE TO HANDLE COMMAND RESULT TOPIC

# Subscribes to (SlamMerge) topic
def slamMergeSub():
    rospy.Subscriber(SLAM_MERGE_TOPIC, String, updateCommandResult, (droneCount, client))
    rospy.spin()

# Takes in strings from the (SlamMerge) topic for processing
def updateCommandResult(data, args):
    print()
    # TODO: ADD IN CODE TO HANDLE SLAM MERGE
