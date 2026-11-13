uint8 JOINT_TRAJECTORY = 0
uint8 PRE_GRASP        = 1
uint8 GRASP            = 2
uint8 TOGGLE           = 3
uint8 LIFT             = 4

uint8 type

# type == JOINT_TRAJECTORY: one point of the OMPL joint-space path, ordered
# per ConfigManager::Config::ctrlJointNames.
float64[] joint_positions

# type != JOINT_TRAJECTORY: the task-space goal pose for this stage.
geometry_msgs/PoseStamped task_pose

# Populated only for the four task stages (open for PRE_GRASP/GRASP, closed
# for TOGGLE/LIFT, from ConfigManager::Config::gripper's openPos/closedPos);
# empty for JOINT_TRAJECTORY, meaning "leave the gripper wherever it is".
float64[] gripper_positions
