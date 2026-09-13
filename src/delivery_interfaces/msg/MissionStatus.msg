uint8 IDLE=0
uint8 RUNNING=1
uint8 CANCELING=2
uint8 SUCCEEDED=3
uint8 CANCELED=4
uint8 FAILED=5

uint8 PHASE_IDLE=0
uint8 PREFLIGHT=1
uint8 TAKEOFF=2
uint8 TRANSIT=3
uint8 SEARCH=4
uint8 APPROACH=5
uint8 DESCEND=6
uint8 LAND=7
uint8 RELEASE=8
uint8 RETURN_PREFLIGHT=9
uint8 RETURN_TAKEOFF=10
uint8 RETURN_TRANSIT=11
uint8 RETURN_LAND=12
uint8 FINISHED=13

uint16 NONE=0
uint16 INVALID_GOAL=1
uint16 TELEMETRY_LOST=2
uint16 ESTIMATOR_RESET=3
uint16 COMMAND_FAILED=4
uint16 MARKER_LOST=5
uint16 SEARCH_EXHAUSTED=6
uint16 PAYLOAD_FAILED=7
uint16 OPERATOR_TAKEOVER=8
uint16 TIMEOUT=9
uint16 BATTERY_LOW=10
uint16 PREFLIGHT_FAILED=11
uint16 RANGE_LOST=12

std_msgs/Header header
unique_identifier_msgs/UUID mission_id
uint32 sequence
uint8 execution_state
uint8 phase
# Completed milestones, not an ETA. One only on full success.
float32 progress
builtin_interfaces/Duration elapsed
bool payload_released
uint16 error_code
string detail
