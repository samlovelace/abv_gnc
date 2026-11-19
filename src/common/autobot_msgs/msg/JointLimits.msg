string[] joint_names
float64[] lower
float64[] upper
float64[] velocity

string[] gripper_joint_names
float64[] gripper_open   # per-joint position when fully open (may be lower or higher than closed, e.g. a mimic finger pair)
float64[] gripper_closed # per-joint position when fully closed
