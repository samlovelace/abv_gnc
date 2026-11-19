export interface State {
  valid: boolean;
  position: { x: number; y: number; yaw: number };
  velocity: { x: number; y: number; yaw: number };
}

export interface PolicyEntry {
  policy_name: string;
  alive: boolean;
}

export interface PolicyRegistry {
  policies: PolicyEntry[];
}

export interface PolicyActive {
  policy_name: string;
}

export interface SwitchPolicyRequest {
  policy_name: string;
}

export interface SwitchPolicyResponse {
  success: boolean;
  message: string;
}

export interface ThrusterStatus {
  thrusters: string;

  fx: number;
  fy: number;
  fz: number;
}

export interface Obstacle {
  x: number;
  y: number;
  radius: number;
}

export interface ObstacleArray {
  obstacles: Obstacle[];
}

export interface GuidanceCommand {
  type: string;
  duration: number;
  goal_state: State;
  arrival_tolerance: State;
}

export interface Vec3 {
  x: number;
  y: number;
  yaw: number;
}

export interface Path {
  waypoints: Vec3[];
}

export interface ControllerCommand {
  type: string;
  is_global: boolean;
  data: Vec3;
  tolerance: Vec3;
  thrusters: string;
}
export interface CompressedImage {
  header: {
    stamp: {
      sec: number;
      nanosec: number;
    };
    frame_id: string;
  };
  format: string;
  data: string | number[];
}

export interface RegionOfInterest {
  x_offset: number;
  y_offset: number;
  height: number;
  width: number;

  do_rectify: boolean;
}

export interface Time {
  sec: number; // int32
  nanosec: number; // uint32
}

export interface Header {
  stamp: Time; // acquisition time of the image
  frame_id: string; // optical frame of the camera
}

export interface Image {
  header: Header;
  height: number; // uint32, number of rows
  width: number; // uint32, number of columns
  encoding: string; // e.g. "rgb8", "bgr8", "mono8", "16UC1", "32FC1"
  is_bigendian: number; // uint8
  step: number; // uint32, full row length in bytes
  data: Uint8Array; // uint8[], length = step * height
}
export interface Detection2D {
  label: string;
  score: number;
  box: RegionOfInterest;
  mask: Image;
}

export interface Detections2D {
  header: Header;
  detections: Detection2D[];
}

export interface Vec3xyz {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface Pose {
  position: Vec3xyz;
  orientation: Quaternion;
}

export interface PoseStamped {
  header: Header;
  pose: Pose;
}

export interface Twist {
  linear: Vec3xyz;
  angular: Vec3xyz;
}

export const ManipHardwareState_DISABLED = 0;
export const ManipHardwareState_IDLE = 1;
export const ManipHardwareState_MOVING = 2;

export interface ManipHardwareState {
  header: Header;
  name: string;
  position: number[]; // [base..., arm...]
  velocity: number[]; // [base..., arm...]
  task_pose: Pose;
  task_velocity: Twist;
  state: number; // ManipHardwareState_DISABLED | _IDLE | _MOVING
}

export interface ManipHardwareTelemetry {
  name: string;
  position: number[];
  velocity: number[];
}

export interface JointPositionWaypoint {
  positions: number[];
  tolerances: number[];
}

export interface JointVelocityWaypoint {
  velocities: number[];
  tolerances: number[];
}

export interface TaskPositionWaypoint {
  pose: PoseStamped;
  tolerance: number[];
}

export interface TaskVelocityWaypoint {
  goal: Twist;
  tolerance: Twist;
}

export interface JointLimits {
  joint_names: string[];
  lower: number[];
  upper: number[];
  velocity: number[];
  gripper_joint_names: string[];
  gripper_open: number[];
  gripper_closed: number[];
}

export interface Enable {
  enabled: boolean;
}
