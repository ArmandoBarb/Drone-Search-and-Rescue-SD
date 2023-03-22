# ros: topics
OVERSEER_DATA_TOPIC = "OverseerData"
SLAM_MERGE_TOPIC = "SlamMerge" # TODO
WOLF_DATA_TOPIC = "WolfData"
COMMAND_RESULT_TOPIC = "CommandResult" # TODO
COMMAND_TOPIC = "Command" # TODO
OVERSEER_COMMUNICATION_TOPIC = "OverseerCommunication"
END_LOOP_TOPIC = "End"
WOLF_COMMUNICATION_TOPIC = "WolfCommunication"
# ros: topics: SIGNAL
IN_POSITION_SIGNAL = "In_Position"
CONSENSUS_DECISION_SIGNAL = "CONSENSUS_DECISION"
AT_SPIRAL_WAYPOINT_SIGNAL = "At_Spiral_Waypoint"
# ros: services
PROXIMITY_OVERSEER_SERVICE = "ProximityOverseerService"
PROXIMITY_WOLF_SERVICE = "PromixityWolfService"
GPU_SERVICE = "GPUService"
# dynamic services : need tp apend value
WOLF_DRONE_SERVICE = "wolf_service_"
# task group name beginning
SEARCH_TASK_GROUP = "Task_Group_Search_"
MAX_DRONE_WAIT_TIMER = 90