import rospy
import Constants.ros as ros
from airsim_ros_pkgs.srv import requestGPU0
from airsim_ros_pkgs.srv import requestGPU1
from airsim_ros_pkgs.srv import requestGPU2
from airsim_ros_pkgs.srv import requestGPU3
# ros service
GPU_SERVICE0 = ros.GPU_SERVICE0
GPU_SERVICE1 = ros.GPU_SERVICE1
GPU_SERVICE2 = ros.GPU_SERVICE2
GPU_SERVICE3 = ros.GPU_SERVICE3

def checkGPUStatus(droneName):
    # Does first check on if gpu is loaded
    print("Checking if yolo loaded")

    if(droneName=="0"):
        rospy.wait_for_service(GPU_SERVICE0)
        response = rospy.ServiceProxy(GPU_SERVICE0, requestGPU0)
    elif (droneName=="1"):
        rospy.wait_for_service(GPU_SERVICE0)
        response = rospy.ServiceProxy(GPU_SERVICE0, requestGPU0)
    elif (droneName=="2"):
        rospy.wait_for_service(GPU_SERVICE0)
        response = rospy.ServiceProxy(GPU_SERVICE0, requestGPU0)
    elif (droneName=="3"):
        rospy.wait_for_service(GPU_SERVICE0)
        response = rospy.ServiceProxy(GPU_SERVICE0, requestGPU0)
    # if(droneName=="1"):
    #     rospy.wait_for_service(GPU_SERVICE1)
    #     response = rospy.ServiceProxy(GPU_SERVICE1, requestGPU1)
    # if(droneName=="2"):
    #     rospy.wait_for_service(GPU_SERVICE2)
    #     response = rospy.ServiceProxy(GPU_SERVICE2, requestGPU2)
    # if(droneName=="3"):
    #     rospy.wait_for_service(GPU_SERVICE3)
    #     response = rospy.ServiceProxy(GPU_SERVICE3, requestGPU3)

    responseObject = response("", 0, 0)

    # Stays in while until yolo is loaded on the gpu
    while (not responseObject.success):
        time.sleep(1);
        
        if(droneName=="0"):
            rospy.wait_for_service(GPU_SERVICE0)
            response = rospy.ServiceProxy(GPU_SERVICE0, requestGPU0)
        elif (droneName=="1"):
            rospy.wait_for_service(GPU_SERVICE0)
            response = rospy.ServiceProxy(GPU_SERVICE0, requestGPU0)
        elif (droneName=="2"):
            rospy.wait_for_service(GPU_SERVICE0)
            response = rospy.ServiceProxy(GPU_SERVICE0, requestGPU0)
        elif (droneName=="3"):
            rospy.wait_for_service(GPU_SERVICE0)
            response = rospy.ServiceProxy(GPU_SERVICE0, requestGPU0)
        # if(droneName=="1"):
        #     rospy.wait_for_service(GPU_SERVICE1)
        #     response = rospy.ServiceProxy(GPU_SERVICE1, requestGPU1)
        # if(droneName=="2"):
        #     rospy.wait_for_service(GPU_SERVICE2)
        #     response = rospy.ServiceProxy(GPU_SERVICE2, requestGPU2)
        # if(droneName=="3"):
        #     rospy.wait_for_service(GPU_SERVICE3)
        #     response = rospy.ServiceProxy(GPU_SERVICE3, requestGPU3)

        responseObject = response("", 0, 0)

    return