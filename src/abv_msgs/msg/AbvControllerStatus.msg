# node name and current state
AbvNodeStatus status

# Applied forces/torque 
float64 fx
float64 fy 
float64 tz

# must match abv_controller/Vehicle::Status
uint8 IDLE=0
uint8 RUNNING=1
uint8 ARRIVED=2

uint8 arrival

# mirrors Vehicle::isNavOk() at publish time
bool nav_ok