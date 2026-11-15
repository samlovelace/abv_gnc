std_msgs/Header header # stamp + frame_id = kinematic tree root (the "global" frame)
string name # manipulator name

float64[] position # [base..., arm...] joint positions
float64[] velocity # [base..., arm...] joint velocities

geometry_msgs/Pose task_pose # end link pose in header.frame_id
geometry_msgs/Twist task_velocity # end link twist (ref point at the end link origin) in header.frame_id

uint8 state # mirrors StateMachine::STATE, re-derived from isEnabled()/isArrived() since Manipulator has no reference back to StateMachine

uint8 DISABLED = 0
uint8 IDLE = 1
uint8 MOVING = 2
