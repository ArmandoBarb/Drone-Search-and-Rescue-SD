import airsim
import os
import numpy as np
import pprint
import time
import shutil

from PIL import Image
import warnings

warnings.filterwarnings("ignore", category=DeprecationWarning) 

vehicle_name = 'DUMMY'

# set up client object to access multirotor drone
client = airsim.MultirotorClient('10.171.204.219')

# connect to AirSim simulator
client.confirmConnection()

# Note: API Control must be off if you want to manually fly drone
client.enableApiControl(False, vehicle_name)
client.armDisarm(True, vehicle_name)


gps_data = client.getGpsData(gps_name = "", vehicle_name = vehicle_name)

currentDroneData = client.getMultirotorState(vehicle_name = vehicle_name)


print(gps_data)
print(currentDroneData.gps_location)