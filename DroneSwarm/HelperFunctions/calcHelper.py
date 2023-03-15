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

def mapGPSPointOnLine(startLineGPS, endLineGPS, pointGPS):
    # vector goes from current to target
    startLineLatitude = startLineGPS.latitude;
    startLineLongitude = startLineGPS.longitude;
    
    endLineLatitude = endLineGPS.latitude;
    endLineLongitude = endLineGPS.longitude;

    pointLatitude = pointGPS.latitude;
    pointLongitude = pointGPS.longitude;

    startEndLineLatitude = endLineLatitude - startLineLatitude;
    startEndLineLongitude = endLineLongitude - startLineLongitude;
    
    startEndSquared = lineLatitude ** 2 + lineLongitude ** 2

    if (startEndSquared == 0): # start and end gps cordinate are the same
        return startLineGPS;
    else:
        pointStartLineLatitude = endLineLatitude - startLineLatitude;
        pointStartLineLongitude = endLineLongitude - startLineLongitude;
        t = (pointStartLineLatitude ** 2 + pointStartLineLongitude ** 2) / startEndSquared;
        if (t < 0):
            return startLineGPS;
        elif (t > 1):
            return startLineGPS;
        else:
            # NearestPointGPS = [startLineLatitude + (t * startEndLineLatitude), startLineLongitude + (t * startEndLineLongitude)];
            NearestPointGPS.latitude = startLineLatitude + (t * startEndLineLatitude);
            NearestPointGPS.longitude = startLineLongitude + (t * startEndLineLongitude);
            return NearestPointGPS;

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

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def radianCalculatorWNegatives(v1, v2):
    x1 = v1[0]
    x2 = v2[0]
    y1 = v2[1]
    y2 = v2[1]
    degrees = math.atan2(x1*y2-y1*x2,x1*x2+y1*y2);

    return degrees

# Checks updated magnitude based on speed change threshold and vectors
def calcUpdatedMagnitude(currentVector, desiredVector, maxIncreaseSpeed, maxDecreaseSpeed):
    # Get magnitude of current and desired vector
    currentVectorMagnitude = calcVectorMagnitude(currentVector)
    currentDesiredMagnitude = calcVectorMagnitude(desiredVector)

    # Set our calculated magnitude to our desired one by default
    calculatedMagnitude = currentDesiredMagnitude

    # Get the difference between magnitudes
    magnitudeDifference = currentDesiredMagnitude - currentVectorMagnitude

    # Handle speed up
    if (magnitudeDifference > 0 and magnitudeDifference > maxIncreaseSpeed):
        # print("Speeding up")
        calculatedMagnitude = currentVectorMagnitude + maxIncreaseSpeed

    # Handle slow down
    elif (magnitudeDifference < 0 and abs(magnitudeDifference) > maxDecreaseSpeed):
        # print("Slowing down")
        calculatedMagnitude = currentVectorMagnitude - maxDecreaseSpeed

    return calculatedMagnitude


def turningCalculation(currentVector, desiredVector, maxTurnAngle):
    # Set final vector to our desired one by default
    finalVector = desiredVector
    currentMagnitude = calcVectorMagnitude(currentVector)
    desiredMagnitude = calcVectorMagnitude(desiredVector)

    # Calculates difference between vectors
    calculatedRadianBetweenVectors = radianCalculatorWNegatives(desiredVector, currentVector)
    calculatedVecDifDegrees = math.degrees(calculatedRadianBetweenVectors)

    # print("Calculated turn:", calculatedVecDifDegrees)

    # Handles turning current vector if dif is greater than max angle
    if (abs(calculatedVecDifDegrees) > maxTurnAngle):
        curXVector = currentVector[0]
        curYVector = currentVector[1]

        # If we have a negative degree, turn a negative angle
        if (calculatedVecDifDegrees < 0):
            maxTurnAngle = maxTurnAngle * -1

        # Get rotate
        cs = math.cos(maxTurnAngle)
        sn = math.sin(maxTurnAngle)

        # Rotates vector
        rotatedXVector = curXVector * cs - curYVector * sn
        rotatedYVector = curXVector * sn + curYVector * cs

        # Rotate current vector a certain degree
        rotatedCurrentVector = [rotatedXVector, rotatedYVector]

        # Return rotated vector
        finalVector = rotatedCurrentVector

    # Calculate speed up and slowdown
    speedChangeThreshold = 3
    maxIncreaseSpeed = 3
    maxDecreaseSpeed = 3
    desiredMagnitude = calcUpdatedMagnitude(currentVector, desiredVector, maxIncreaseSpeed, maxDecreaseSpeed)
    finalVectorMagnitude = calcVectorMagnitude(finalVector)
    
    # Set desired magnitude is greater than 0
    if (desiredMagnitude > 0 and finalVectorMagnitude > 0):
        finalVector = setVectorMagnitude(finalVector, desiredMagnitude)

    # Turn is within max turn angle
    return finalVector
    

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
