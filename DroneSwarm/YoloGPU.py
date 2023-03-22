# import rospy
# import json
# # import Constants
import Constants.ros as ros

#from std_msgs.msg import String
#from airsim_ros_pkgs.msg import droneData
#from airsim_ros_pkgs.srv import getDroneData, getDroneDataResponse
# from std_srvs.srv import Trigger, TriggerResponse
from airsim_ros_pkgs.srv import requestGPU


global MODEL

# Environmental Variables
# ros: services
GPU_SERVICE = ros.GPU_SERVICE
isModelLoaded = False

# Main Process Start ----------------------------------------------
def startYoloGPU():
    global isModelLoaded
    global MODEL
    debugPrint("StartingYoloGPU")
    # Create node for "ProximityWolf"
    nodeName = "YoloGPU"
    rospy.init_node(nodeName, anonymous = True)

    # start yolo GPU
    cwd = os.getcwd()
    yoloPT = os.path.join(str(cwd), 'best.pt')

    # Model is loaded global to be used by service functions
    try:
        MODEL = torch.hub.load('ultralytics/yolov5', 'custom', path=yoloPT, trust_repo=True)
        isModelLoaded = True
    except:
        isModelLoaded = False

    # Spins up gpu service
    print("Starting gpu service")
    startGPUService()

# Main Process end -----------------------------------------------

# Starts of gpu service for handling requests, requests are then done through the handleGPU function
def startGPUService():
    serviceName = GPU_SERVICE
    service = rospy.Service(serviceName, requestGPU, handleGPU)
    rospy.spin()


# # TODO: handle data retrieval for service calls
def handleGPU(request):
    global isModelLoaded

    responseString = request.responseString

    # Send back gpu model status if reponse string is empty
    if (reponseString == ""):
        print("Got the service request")
        return (isModelLoaded, 0, 0, 0, 0)
        # TODO:Send back is model loaded status in success bool

    # Else we handle the response string for processing
    else:
        segArr = np.fromstring(responseString, dtype=np.uint8)
        sceneRGB1 = segArr.reshape(height, width, 3)

        #print("RESULTS yolov5:")
        model.classes = [0] # detect only for person class (0)
        results=model(sceneRGB2)
        #results.print()

        # get the bounding boxes and confidence scores for single image
        validDetection = False
        resultsPandas = results.pandas().xyxy[0]
        confidenceArr = resultsPandas.confidence

        # Check if we found no bounding boxes, if so return false with empty values
        if (len(confidenceArr) == 0):
            return (False, 0, 0, 0, 0)
            # TODO:Send back false in success bool


        # TODO: Send over object with this stuff
        success = responseObject.success
        xMin = resultsPandas.xmin
        yMin = resultsPandas.ymin
        xMax = resultsPandas.xmax
        yMax = resultsPandas.ymax
        
        return (success, xMin, yMin, xMax, yMax)
        # load yolo model

