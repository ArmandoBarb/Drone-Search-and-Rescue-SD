import rospy
import Constants.ros as ros

from airsim_ros_pkgs.msg import GPS
from airsim_ros_pkgs.msg import updateMap
MAP_HANDLER_TOPIC = ros.MAP_HANDLER_TOPIC
# ros: updateMapCommand types
FINAL_TARGET_POSITION = ros.FINAL_TARGET_POSITION
NEW_GPS_PREDICTION = ros.NEW_GPS_PREDICTION
UPDATE_DRONE_POSITION =  ros.UPDATE_DRONE_POSITION

def updateWolfDronePosition(wolfMapPublisher, droneName, currentGPS):
    updateMapMessage = updateMap()
    updateMapMessage.updateType = UPDATE_DRONE_POSITION

    updateMapMessage.wolfNumber = int(droneName)
    updateMapMessage.imageNumber = 0
    updateMapMessage.isOverseer = False

    # Gets the wolf position object
    wolfPosition = GPS()
    wolfPosition.longitude = currentGPS[2]
    wolfPosition.latitude = currentGPS[1]
    updateMapMessage.wolfPosition = wolfPosition

    # Gets the target position object
    targetPosition = GPS()
    updateMapMessage.targetPosition = targetPosition

    wolfMapPublisher.publish(updateMapMessage)

def updateWolfDronePrediction(wolfMapPublisher, droneName, imageNumber, currentGPS, targetLat, targetLon):
    updateMapMessage = updateMap()
    updateMapMessage.updateType = NEW_GPS_PREDICTION

    updateMapMessage.wolfNumber = int(droneName)
    updateMapMessage.imageNumber = imageNumber
    updateMapMessage.isOverseer = False

    # Gets the wolf position object
    wolfPosition = GPS()
    wolfPosition.longitude = currentGPS[2]
    wolfPosition.latitude = currentGPS[1]
    updateMapMessage.wolfPosition = wolfPosition

    # Gets the target position object
    targetPosition = GPS()
    targetPosition.longitude = targetLon
    targetPosition.latitude = targetLat
    updateMapMessage.targetPosition = targetPosition

    wolfMapPublisher.publish(updateMapMessage)

def updateWolfDroneFailPrediction(wolfMapPublisher, droneName, imageNumber, currentGPS, targetLat, targetLon):
    updateMapMessage = updateMap()
    updateMapMessage.updateType = NEW_GPS_PREDICTION

    updateMapMessage.wolfNumber = int(droneName)
    updateMapMessage.imageNumber = -imageNumber
    updateMapMessage.isOverseer = False

    # Gets the wolf position object
    wolfPosition = GPS()
    wolfPosition.longitude = currentGPS[2]
    wolfPosition.latitude = currentGPS[1]
    updateMapMessage.wolfPosition = wolfPosition

    # Gets the target position object
    targetPosition = GPS()
    targetPosition.longitude = targetLon
    targetPosition.latitude = targetLat
    updateMapMessage.targetPosition = targetPosition

    wolfMapPublisher.publish(updateMapMessage)



def updateFinalTargetPosition(droneName, targetGPS):
    wolfMapPublisher = rospy.Publisher(MAP_HANDLER_TOPIC, updateMap, latch=True, queue_size=100)
    
    updateMapMessage = updateMap()
    updateMapMessage.updateType = FINAL_TARGET_POSITION

    updateMapMessage.wolfNumber = int(droneName)
    updateMapMessage.imageNumber = 0
    updateMapMessage.isOverseer = False

    # Gets the wolf position object
    wolfPosition = GPS()
    updateMapMessage.wolfPosition = wolfPosition

    # Gets the target position object
    targetPosition = GPS()
    targetPosition.longitude = targetGPS.longitude
    targetPosition.latitude = targetGPS.latitude
    updateMapMessage.targetPosition = targetPosition

    wolfMapPublisher.publish(updateMapMessage)

def updateOverseerDronePosition(wolfMapPublisher, currentGPS):
    updateMapMessage = updateMap()
    updateMapMessage.updateType = UPDATE_DRONE_POSITION

    updateMapMessage.wolfNumber = -1
    updateMapMessage.imageNumber = 0
    updateMapMessage.isOverseer = True

    # Gets the wolf position object
    wolfPosition = GPS()
    wolfPosition.longitude = currentGPS.longitude
    wolfPosition.latitude = currentGPS.latitude
    updateMapMessage.wolfPosition = wolfPosition

    # Gets the target position object
    targetPosition = GPS()
    updateMapMessage.targetPosition = targetPosition

    wolfMapPublisher.publish(updateMapMessage)

def updateOverseerDronePrediction(wolfMapPublisher, imageNumber, currentGPS, targetLat, targetLon):
    updateMapMessage = updateMap()
    updateMapMessage.updateType = NEW_GPS_PREDICTION

    updateMapMessage.wolfNumber = 0
    updateMapMessage.imageNumber = imageNumber
    updateMapMessage.isOverseer = True

    # Gets the wolf position object
    wolfPosition = GPS()
    wolfPosition.longitude = currentGPS.longitude
    wolfPosition.latitude = currentGPS.latitude
    updateMapMessage.wolfPosition = wolfPosition

    # Gets the target position object
    targetPosition = GPS()
    targetPosition.longitude = targetLon
    targetPosition.latitude = targetLat
    updateMapMessage.targetPosition = targetPosition

    wolfMapPublisher.publish(updateMapMessage)
