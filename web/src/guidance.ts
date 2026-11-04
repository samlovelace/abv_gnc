// Shared GuidanceCommand helpers, used by both TableInteractionHandler's
// drag-to-set-goal flow and CommandPanelHandler's Pose mode, so there's one
// place that decides the "line" vs "avoid" type and the zeroed defaults -
// mirrors CommandPanel::sendPoseCommand in abv_gui.

import type { RosClient } from "./ros";
import type { State, GuidanceCommand } from "./messages";
import { getObstaclesPresent } from "./obstacle-state";

const GUIDANCE_COMMAND_TOPIC = "abv/guidance/command";
const GUIDANCE_COMMAND_TYPE = "autobot_msgs/msg/GuidanceCommand";

export function zeroState(): State {
  return {
    valid: true,
    position: { x: 0, y: 0, yaw: 0 },
    velocity: { x: 0, y: 0, yaw: 0 },
  };
}

export function publishPoseGoal(
  ros: RosClient,
  x: number,
  y: number,
  yaw: number,
): void {
  const goalState = zeroState();
  goalState.position = { x, y, yaw };

  const cmd: GuidanceCommand = {
    type: getObstaclesPresent() ? "avoid" : "line",
    duration: -1,
    goal_state: goalState,
    arrival_tolerance: zeroState(),
  };
  ros.publish(GUIDANCE_COMMAND_TOPIC, GUIDANCE_COMMAND_TYPE, cmd);
}

export function publishPathCommand(
  ros: RosClient,
  type: string,
  duration: number,
): void {
  const cmd: GuidanceCommand = {
    type,
    duration,
    goal_state: zeroState(),
    arrival_tolerance: zeroState(),
  };
  ros.publish(GUIDANCE_COMMAND_TOPIC, GUIDANCE_COMMAND_TYPE, cmd);
}
