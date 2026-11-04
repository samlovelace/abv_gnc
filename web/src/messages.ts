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
