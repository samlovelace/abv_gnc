export interface AbvState {
  valid: boolean;
  position: { x: number; y: number; yaw: number };
  velocity: { x: number; y: number; yaw: number };
}

export interface AbvPolicyEntry {
  policy_name: string;
  alive: boolean;
}

export interface AbvPolicyRegistry {
  policies: AbvPolicyEntry[];
}

export interface AbvPolicyActive {
  policy_name: string;
}

export interface SwitchPolicyRequest {
  policy_name: string;
}

export interface SwitchPolicyResponse {
  success: boolean;
  message: string;
}

export interface AbvThrusterStatus {
  thrusters: string;

  fx: number;
  fy: number;
  fz: number;
}

export interface AbvObstacle {
  x: number;
  y: number;
  radius: number;
}

export interface AbvObstacleArray {
  obstacles: AbvObstacle[];
}

export interface AbvGuidanceCommand {
  type: string;
  duration: number;
  goal_state: AbvState;
  arrival_tolerance: AbvState;
}

export interface AbvVec3 {
  x: number;
  y: number;
  yaw: number;
}

export interface AbvPath {
  waypoints: AbvVec3[];
}
