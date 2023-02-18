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

def calcDistanceInMetersBetweenGPS(currentGPS, targetGPS):
    # this is not an accurate conversion more of an estiamte
    currentLatitude = currentGPS.latitude;
    currentLongitude = currentGPS.longitude;
    targetLatitude = targetGPS.latitude;
    targetLongitude = targetGPS.longitude;
    degree = math.pi / 180;
    R = 6371e3 # earrth radius in meters
    lat1 = currentLatitude * degree;
    lat2 = targetLatitude * degree;
    lon1 = currentLongitude * degree;
    lon2 = targetLongitude * degree;
    latDif = (targetLatitude - currentLatitude) * degree;
    lonDif = (targetLongitude - currentLongitude) * degree;

    a = (math.sin(latDif / 2) ** 2) + (math.cos(lat1) * math.cos(lat2)) \
        * (math.sin(lonDif / 2) ** 2);
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a));

    distanceInMeters = R * c;

    return distanceInMeters - 3;

def calcCentripetalVelocity(velocityM, radiusM):
    cv = velocityM ** 2 / radiusM;
    return cv;
    
def calcDistanceBetweenGPS(currentGPS, targetGPS):
    vector = calcVectorBetweenGPS(currentGPS, targetGPS);
    distanceTarget = math.sqrt( (vector[0] ** 2) + (vector[1] ** 2) )

    return distanceTarget;

def calcVectorMagnitude(vector):
    magnitude = math.sqrt( (vector[0] ** 2) + (vector[1] ** 2) )
    return magnitude;

def calcCircleChord(radius, angle):
    chord = 2 * radius * math.sin((angle * math.pi) / 360)
    return chord;

def calcPolygonSideLength(radius, sideNumber):
    sideLength = 2 * radius * math.sin(math.radians(180 / sideNumber))
    return sideLength;

def applyMaxSpeed(vector, maxSpeed):
    magnitude = calcVectorMagnitude(vector)
    if (magnitude > maxSpeed):
        return setVectorMagnitude(vector, maxSpeed)
    return vector;

def setVectorMagnitude(vector, magnitude):
    oldMagnitude = calcVectorMagnitude(vector)
    multiplyXY = magnitude / oldMagnitude

    newVX = vector[0] * multiplyXY
    newVY = vector[1] * multiplyXY
    return [newVX, newVY];

def calcVectorAngle(vector1, vector2):
    top = np.dot(vector1, vector2);
    bottom = calcVectorMagnitude(vector1) * calcVectorMagnitude(vector2);
    angleRadians = math.acos(top / bottom);
    angleDegrees =  (angleRadians * 180) / math.pi;
    return angleDegrees;


# _a__
# \  |
# c\θ|b
#   \|___v_____
# a = Opposite b = Adjacent
# c = hypothenus
def calcRTriangleOpposite(hypothenus, angleD):
    return math.sin(math.radians(angleD)) * hypothenus;

def calcRTriangleAdjacent(hypothenus, angleD):
    return math.cos(math.radians(angleD)) * hypothenus;
