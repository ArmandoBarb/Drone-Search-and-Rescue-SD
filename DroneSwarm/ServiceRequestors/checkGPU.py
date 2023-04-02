import rospy
import Constants.ros as ros
from airsim_ros_pkgs.srv import requestGPU
# ros service
GPU_SERVICE = ros.GPU_SERVICE

def checkGPUStatus():
    # Does first check on if gpu is loaded
    print("Checking if yolo loaded")
    rospy.wait_for_service(GPU_SERVICE)
    response = rospy.ServiceProxy(GPU_SERVICE, requestGPU)
    responseObject = response("", 0, 0)

    # Stays in while until yolo is loaded on the gpu
    while (not responseObject.success):
        time.sleep(1);
        rospy.wait_for_service(GPU_SERVICE)
        response = rospy.ServiceProxy(GPU_SERVICE, requestGPU)
        responseObject = response("", 0, 0)

    return