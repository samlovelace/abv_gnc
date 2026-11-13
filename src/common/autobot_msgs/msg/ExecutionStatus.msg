uint8 IN_PROGRESS = 0
uint8 SUCCEEDED = 1
uint8 FAILED = 2

uint8 status
int32 waypoint_index    # current waypoint (0-based) when IN_PROGRESS
int32 total_waypoints
string message          # failure detail, or "ok" on success
