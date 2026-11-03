# node name and current state
NodeStatus status

# must match autobot_goal_manager/GoalManager::Status
uint8 IDLE=0
uint8 RUNNING=1
uint8 ARRIVED=2

uint8 arrival

# mirrors GoalManager::isNavOk() at publish time
bool nav_ok