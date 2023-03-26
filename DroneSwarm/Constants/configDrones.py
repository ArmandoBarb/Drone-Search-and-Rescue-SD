# shared non export
min_circle_radius_gps = 0.00008983152373552244
# shared end

LOOP_NUMBER = 100000
MAX_TIME = 1000
LOCAL_IP = "10.171.204.214"
# collision start
COLLISION_MODE_TIME_LENGTH = 1
MAX_COLLISION_TIME = 3
MIN_COLLISION_TIME = 0.25
# collision end
# vector control start
MAX_TURN_ANGLE = 30
SPEED_CHANGE = 0.1
MIN_SPEED_FACTOR = 0.4
# vector control end
# linebehavior start
DISTANCE_LEAD_OVERSEER_GPS = min_circle_radius_gps * 2
# waypoint 
MAX_WAYPOINT_SAVE_TIME = 100    # measured in seconds
WAYPOINT_HISTORY_DISTANCE_MULT = 1
WAPOINT_HISTORY_DISTANCE_ERROR = 0.0001
# linebehavior end
# circling start
MIN_CIRCLE_RADIUS_GPS = min_circle_radius_gps # 10 in x direction converted to gps
MIN_CIRCLE_RADIUS_METERS = 6.988048291572515 # 10 in x direction converted to Meters
MIN_DIFFRENCE_IN_RADIUS = min_circle_radius_gps * 0.2
REQUIRED_SEPERATION_PERCENT = 0.8
MIN_CIRCLE_PADDING_FOR_SEARCH_HISTORY = min_circle_radius_gps * 1.1
# circling end
# wolf search start
WOLF_SEARCH_REQUEST_HELP_DISTANCE_MULTIPLE = 1.5
CIRCLE_SPACING = min_circle_radius_gps * 0.2
# MIN_CIRCLE_RADIUS_GPS
# MIN_CIRCLE_RADIUS_METERS
# wolf search end
# consenus start
CONSENSUS_DECISION_REQUEST_HELP_DISTANCE_MULTIPLE = 3
MAX_CONSENSUS_ITERATION_NUMBER = 1
CONSENSUS_ITERATION_LENGTH_SECONDS = 15
CONSENSUS_THRESHOLD = 0.1
YOLO_CONFIDENCE = 0.65
# MIN_CIRCLE_RADIUS_GPS
# MIN_CIRCLE_RADIUS_METERS
# consenus end