import math
import numpy as np

def perpendicularVector( vector ) :
    perpendicularVector = np.empty_like(vector)
    perpendicularVector[0] = -vector[1]
    perpendicularVector[1] = vector[0]
    return perpendicularVector

def normalizeVector(vector):
    vector = np.array(vector)
    vectorNormalized =  vector/np.linalg.norm(vector)
    return vectorNormalized;

def calcVectorBetweenGPS(currentGPS, targetGPS):
    # vector goes from current to target
    currentLatitude = currentGPS.latitude;
    currentLongitude = currentGPS.longitude;

    targetLatitude = targetGPS.latitude;
    targetLongitude = targetGPS.longitude;

    latitudeDifference = (targetLatitude - currentLatitude);
    longitudeDifference = (targetLongitude - currentLongitude);

    return [latitudeDifference, longitudeDifference];


    
def calcDistanceBetweenGPS(currentGPS, targetGPS):
    vector = calcVectorBetweenGPS(currentGPS, targetGPS);
    distanceTarget = math.sqrt( (vector[0] ** 2) + (vector[1] ** 2) )

    return distanceTarget;

def calcVectorMagnitude(vector):
    magnitude = math.sqrt( (vector[0] ** 2) + (vector[1] ** 2) )
    return magnitude;