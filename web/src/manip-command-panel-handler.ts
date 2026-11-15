// Manipulator analog of CommandPanelHandler (command-panel-handler.ts): a
// mode-switching panel for commanding the arm directly in Joint Position,
// Joint Velocity, Task Position or Task Velocity space, plus an always-on
// gripper slider and an Enable/Disable control (every waypoint type on
// CommandHandler is silently dropped unless arm/enable has been sent true -
// see CommandHandler::handleWaypoint/gripperPosWaypointCallback).
//
// Joint names/limits aren't known until arm/joint_limits arrives, so the
// per-joint rows (Joint Position/Velocity) are built dynamically the first
// time that topic delivers a message; Task Position/Velocity and the
// gripper row are fixed-shape and built eagerly.
//
// Every mode's inputs are kept synced to live telemetry continuously,
// regardless of which mode is visually active (same dirty-tracking idea as
// CommandPanelHandler's SyncedVec3Fields), so switching modes never reveals
// a stale snapshot and never causes a stray waypoint to fire.

import type { Handler } from "./handler";
import type { RosClient } from "./ros";
import { must } from "./dom";
import type {
  Enable,
  JointLimits,
  JointPositionWaypoint,
  JointVelocityWaypoint,
  ManipHardwareState,
  ManipHardwareTelemetry,
  Pose,
  TaskPositionWaypoint,
  TaskVelocityWaypoint,
  Twist,
} from "./messages";
import { ManipHardwareState_DISABLED } from "./messages";

const ARM_ENABLE_TOPIC = "arm/enable";
const ARM_ENABLE_TYPE = "autobot_msgs/msg/Enable";

const ARM_HW_STATE_TOPIC = "arm/hardware/state";
const ARM_HW_STATE_TYPE = "autobot_msgs/msg/ManipHardwareState";

const ARM_GRIPPER_TELEM_TOPIC = "arm/hardware/gripper_telemetry";
const ARM_GRIPPER_TELEM_TYPE = "autobot_msgs/msg/ManipHardwareTelemetry";

const ARM_JOINT_LIMITS_TOPIC = "arm/joint_limits";
const ARM_JOINT_LIMITS_TYPE = "autobot_msgs/msg/JointLimits";

const ARM_JOINT_POS_WP_TOPIC = "arm/joint_position_waypoint";
const ARM_JOINT_POS_WP_TYPE = "autobot_msgs/msg/JointPositionWaypoint";

const ARM_JOINT_VEL_WP_TOPIC = "arm/joint_velocity_waypoint";
const ARM_JOINT_VEL_WP_TYPE = "autobot_msgs/msg/JointVelocityWaypoint";

const ARM_TASK_POS_WP_TOPIC = "arm/task_position_waypoint";
const ARM_TASK_POS_WP_TYPE = "autobot_msgs/msg/TaskPositionWaypoint";

const ARM_TASK_VEL_WP_TOPIC = "arm/task_velocity_waypoint";
const ARM_TASK_VEL_WP_TYPE = "autobot_msgs/msg/TaskVelocityWaypoint";

const ARM_GRIPPER_WP_TOPIC = "arm/gripper_position_waypoint";
const ARM_GRIPPER_WP_TYPE = "autobot_msgs/msg/JointPositionWaypoint";

const JOINT_TOLERANCE = 0.01; // rad
const GRIPPER_TOLERANCE = 0.002; // m
const TASK_POS_TOLERANCE = [0.005, 0.005, 0.005, 0.02, 0.02, 0.02]; // xyz (m), rpy (rad)
const LINEAR_JOG_MPS = 0.05;
const ANGULAR_JOG_RADPS = 0.2;
const FIRE_INTERVAL_MS = 100;

type Axis6 = "x" | "y" | "z" | "roll" | "pitch" | "yaw";
const AXIS6: Axis6[] = ["x", "y", "z", "roll", "pitch", "yaw"];

function zeroTwist(): Twist {
  return { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
}

// Standard ZYX (roll about x, pitch about y, yaw about z, applied
// extrinsically) Euler<->quaternion conversions - matches the convention
// KDL::Rotation::GetRPY / geometry_msgs use, so this round-trips cleanly
// with what TaskPositionWaypoint's converter (utils::toFrame) expects.
function quaternionToRPY(q: {
  x: number;
  y: number;
  z: number;
  w: number;
}): { roll: number; pitch: number; yaw: number } {
  const { x, y, z, w } = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const sinp = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
  const pitch = Math.asin(sinp);
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return { roll, pitch, yaw };
}

function rpyToQuaternion(
  roll: number,
  pitch: number,
  yaw: number,
): { x: number; y: number; z: number; w: number } {
  const cr = Math.cos(roll / 2);
  const sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2);
  const sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2);
  const sy = Math.sin(yaw / 2);
  return {
    w: cr * cp * cy + sr * sp * sy,
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy,
  };
}

// One input element that doubles as a command input and a live readout -
// stays "dirty" (ignores incoming sync()s) from the moment the user starts
// editing until the browser's native "change" event fires (drag release for
// a range slider, blur-after-edit for a number input), then resumes
// tracking live telemetry.
class SyncedInput {
  dirty = false;
  el: HTMLInputElement;

  constructor(el: HTMLInputElement, onInput: () => void) {
    this.el = el;
    el.addEventListener("input", () => {
      this.dirty = true;
      onInput();
    });
    el.addEventListener("change", () => {
      this.dirty = false;
    });
  }

  sync(value: number, decimals = 3): void {
    if (this.dirty || document.activeElement === this.el) return;
    this.el.value = value.toFixed(decimals);
  }

  get value(): number {
    return this.el.valueAsNumber;
  }
}

// Press-and-hold jog button, shared mutual-exclusion firing logic - ports
// CommandPanelHandler's #makeFireButton/#startFiring/#stopFiring pattern
// (only one jog fires at a time across the whole panel).
class JogFire {
  #handle: ReturnType<typeof setInterval> | null = null;
  #pendingZero: (() => void) | null = null;

  start(fire: () => void, zero: () => void): void {
    this.stop();
    fire();
    this.#handle = setInterval(fire, FIRE_INTERVAL_MS);
    this.#pendingZero = zero;
  }

  stop(): void {
    if (this.#handle === null) return;
    clearInterval(this.#handle);
    this.#handle = null;
    const zero = this.#pendingZero;
    this.#pendingZero = null;
    zero?.();
  }
}

function makeJogRow(
  label: string,
  fire: JogFire,
  onJog: (direction: 1 | -1) => void,
  onZero: () => void,
): { row: HTMLElement; valueEl: HTMLElement } {
  const row = document.createElement("div");
  row.className = "jog-row";

  const labelEl = document.createElement("label");
  labelEl.textContent = label;

  const minusBtn = document.createElement("button");
  minusBtn.className = "btn-warning";
  minusBtn.textContent = "-";

  const plusBtn = document.createElement("button");
  plusBtn.className = "btn-warning";
  plusBtn.textContent = "+";

  const valueEl = document.createElement("span");
  valueEl.className = "jog-value";
  valueEl.textContent = "0.000";

  for (const [btn, dir] of [
    [minusBtn, -1],
    [plusBtn, 1],
  ] as [HTMLButtonElement, 1 | -1][]) {
    btn.addEventListener("pointerdown", (e) => {
      e.preventDefault();
      fire.start(
        () => onJog(dir),
        () => onZero(),
      );
    });
    btn.addEventListener("pointerup", () => fire.stop());
    btn.addEventListener("pointerleave", () => fire.stop());
  }

  row.append(labelEl, minusBtn, valueEl, plusBtn);
  return { row, valueEl };
}

export class ManipCommandPanel implements Handler {
  #ros: RosClient;

  #enablePill: HTMLButtonElement;
  #modeSelect: HTMLSelectElement;
  #modePanels: HTMLElement[];

  #jointPosRowsEl: HTMLElement;
  #jointVelRowsEl: HTMLElement;
  #taskVelRowsEl: HTMLElement;

  #gripperSlider: SyncedInput;
  #gripperValue: HTMLElement;

  #jointNames: string[] = [];
  #jointLower: number[] = [];
  #jointUpper: number[] = [];
  #jointVelLimit: number[] = [];
  #jointPosSliders: SyncedInput[] = [];
  #jointVelValueEls: HTMLElement[] = [];

  #taskPos: Record<Axis6, SyncedInput>;
  #taskVelValueEls: Record<Axis6, HTMLElement> | null = null;

  #latestState: ManipHardwareState | undefined;
  #fire = new JogFire();

  constructor(ros: RosClient, root: ParentNode) {
    this.#ros = ros;

    this.#enablePill = must<HTMLButtonElement>(root, "#manip-enable-pill");
    this.#enablePill.addEventListener("click", this.#onToggleEnable);

    this.#modeSelect = must<HTMLSelectElement>(root, "#manip-mode-select");
    this.#modePanels = Array.from(
      root.querySelectorAll<HTMLElement>("[data-mode-panel]"),
    );
    this.#modeSelect.addEventListener("change", this.#onModeChange);
    this.#onModeChange();

    this.#jointPosRowsEl = must<HTMLElement>(root, "#joint-pos-rows");
    this.#jointVelRowsEl = must<HTMLElement>(root, "#joint-vel-rows");
    this.#taskVelRowsEl = must<HTMLElement>(root, "#task-vel-rows");

    const taskPosEl = (axis: Axis6) =>
      must<HTMLInputElement>(root, `#task-pos-${axis}`);
    this.#taskPos = {
      x: new SyncedInput(taskPosEl("x"), this.#onTaskPositionInput),
      y: new SyncedInput(taskPosEl("y"), this.#onTaskPositionInput),
      z: new SyncedInput(taskPosEl("z"), this.#onTaskPositionInput),
      roll: new SyncedInput(taskPosEl("roll"), this.#onTaskPositionInput),
      pitch: new SyncedInput(taskPosEl("pitch"), this.#onTaskPositionInput),
      yaw: new SyncedInput(taskPosEl("yaw"), this.#onTaskPositionInput),
    };

    this.#gripperSlider = new SyncedInput(
      must<HTMLInputElement>(root, "#gripper-cmd"),
      this.#onGripperInput,
    );
    this.#gripperValue = must<HTMLElement>(root, "#gripper-value");

    this.#buildTaskVelRows();

    ros.watch<ManipHardwareState>(ARM_HW_STATE_TOPIC, ARM_HW_STATE_TYPE, this.#onHardwareState);
    ros.watch<ManipHardwareTelemetry>(
      ARM_GRIPPER_TELEM_TOPIC,
      ARM_GRIPPER_TELEM_TYPE,
      this.#onGripperTelemetry,
    );
    ros.watch<JointLimits>(ARM_JOINT_LIMITS_TOPIC, ARM_JOINT_LIMITS_TYPE, this.#onJointLimits);
  }

  update(): void {}

  dispose(): void {
    this.#fire.stop();
  }

  #onModeChange = (): void => {
    this.#fire.stop();
    const mode = this.#modeSelect.value;
    for (const panel of this.#modePanels) {
      panel.style.display = panel.dataset.modePanel === mode ? "" : "none";
    }
  };

  #onToggleEnable = (): void => {
    const currentlyDisabled =
      (this.#latestState?.state ?? ManipHardwareState_DISABLED) ===
      ManipHardwareState_DISABLED;
    const msg: Enable = { enabled: currentlyDisabled };
    this.#ros.publish(ARM_ENABLE_TOPIC, ARM_ENABLE_TYPE, msg);
  };

  #onHardwareState = (msg: ManipHardwareState): void => {
    this.#latestState = msg;

    const disabled = msg.state === ManipHardwareState_DISABLED;
    this.#enablePill.dataset.state = disabled ? "disabled" : "enabled";
    this.#enablePill.textContent = disabled ? "Disabled" : "Enabled";

    // Full kinematic array is [base..., arm...]; ctrl/arm joints are always
    // the tail of it, same offset ManipConfigManager::Config::ctrlOffset
    // applies server-side - recovered here from the two known lengths
    // rather than needing a dedicated field on arm/joint_limits.
    const offset = Math.max(0, msg.position.length - this.#jointNames.length);
    this.#jointPosSliders.forEach((s, i) => s.sync(msg.position[offset + i] ?? 0));
    this.#jointVelValueEls.forEach(
      (el, i) => (el.textContent = (msg.velocity[offset + i] ?? 0).toFixed(3)),
    );

    const rpy = quaternionToRPY(msg.task_pose.orientation);
    this.#taskPos.x.sync(msg.task_pose.position.x);
    this.#taskPos.y.sync(msg.task_pose.position.y);
    this.#taskPos.z.sync(msg.task_pose.position.z);
    this.#taskPos.roll.sync(rpy.roll);
    this.#taskPos.pitch.sync(rpy.pitch);
    this.#taskPos.yaw.sync(rpy.yaw);

    if (this.#taskVelValueEls) {
      const t = msg.task_velocity;
      this.#taskVelValueEls.x.textContent = t.linear.x.toFixed(3);
      this.#taskVelValueEls.y.textContent = t.linear.y.toFixed(3);
      this.#taskVelValueEls.z.textContent = t.linear.z.toFixed(3);
      this.#taskVelValueEls.roll.textContent = t.angular.x.toFixed(3);
      this.#taskVelValueEls.pitch.textContent = t.angular.y.toFixed(3);
      this.#taskVelValueEls.yaw.textContent = t.angular.z.toFixed(3);
    }
  };

  #onGripperTelemetry = (msg: ManipHardwareTelemetry): void => {
    this.#gripperSlider.sync(msg.position[0] ?? 0);
    this.#gripperValue.textContent = (msg.position[0] ?? 0).toFixed(3);
  };

  #onJointLimits = (msg: JointLimits): void => {
    if (this.#jointNames.length === 0 && msg.joint_names.length > 0) {
      this.#jointNames = msg.joint_names;
      this.#jointLower = msg.lower;
      this.#jointUpper = msg.upper;
      this.#jointVelLimit = msg.velocity;
      this.#buildJointPosRows();
      this.#buildJointVelRows();
    }

    if (msg.gripper_joint_names.length > 0 && msg.gripper_lower.length > 0) {
      const slider = this.#gripperSlider.el;
      slider.min = String(msg.gripper_lower[0]);
      slider.max = String(msg.gripper_upper[0]);
      slider.disabled = false;
    }
  };

  #buildJointPosRows(): void {
    this.#jointPosRowsEl.innerHTML = "";
    this.#jointPosSliders = this.#jointNames.map((name, i) => {
      const row = document.createElement("div");
      row.className = "joint-slider-row";

      const label = document.createElement("label");
      label.textContent = `${name}:`;

      const input = document.createElement("input");
      input.type = "range";
      input.min = String(this.#jointLower[i] ?? -3.14);
      input.max = String(this.#jointUpper[i] ?? 3.14);
      input.step = "0.005";
      input.value = "0";

      const valueEl = document.createElement("span");
      valueEl.className = "joint-value";

      const synced = new SyncedInput(input, () => {
        valueEl.textContent = synced.value.toFixed(3);
        this.#sendJointPosition();
      });
      valueEl.textContent = "0.000";

      row.append(label, input, valueEl);
      this.#jointPosRowsEl.appendChild(row);
      return synced;
    });
  }

  #buildJointVelRows(): void {
    this.#jointVelRowsEl.innerHTML = "";
    this.#jointVelValueEls = this.#jointNames.map((name, i) => {
      const { row, valueEl } = makeJogRow(
        `${name}:`,
        this.#fire,
        (direction) => this.#sendJointVelocity(i, direction),
        () => this.#sendJointVelocity(null, 0),
      );
      this.#jointVelRowsEl.appendChild(row);
      return valueEl;
    });
  }

  #buildTaskVelRows(): void {
    const labels: Record<Axis6, string> = {
      x: "x (m/s):",
      y: "y (m/s):",
      z: "z (m/s):",
      roll: "roll (rad/s):",
      pitch: "pitch (rad/s):",
      yaw: "yaw (rad/s):",
    };
    const valueEls: Partial<Record<Axis6, HTMLElement>> = {};
    for (const axis of AXIS6) {
      const { row, valueEl } = makeJogRow(
        labels[axis],
        this.#fire,
        (direction) => this.#sendTaskVelocity(axis, direction),
        () => this.#sendTaskVelocity(null, 0),
      );
      this.#taskVelRowsEl.appendChild(row);
      valueEls[axis] = valueEl;
    }
    this.#taskVelValueEls = valueEls as Record<Axis6, HTMLElement>;
  }

  #sendJointPosition = (): void => {
    if (this.#jointPosSliders.length === 0) return;
    const msg: JointPositionWaypoint = {
      positions: this.#jointPosSliders.map((s) => s.value),
      tolerances: this.#jointPosSliders.map(() => JOINT_TOLERANCE),
    };
    this.#ros.publish(ARM_JOINT_POS_WP_TOPIC, ARM_JOINT_POS_WP_TYPE, msg);
  };

  #sendJointVelocity(activeIndex: number | null, direction: number): void {
    const n = this.#jointNames.length;
    if (n === 0) return;
    const velocities = new Array(n).fill(0);
    if (activeIndex !== null) {
      velocities[activeIndex] = direction * (this.#jointVelLimit[activeIndex] ?? 0);
    }
    const msg: JointVelocityWaypoint = {
      velocities,
      tolerances: new Array(n).fill(JOINT_TOLERANCE),
    };
    this.#ros.publish(ARM_JOINT_VEL_WP_TOPIC, ARM_JOINT_VEL_WP_TYPE, msg);
  }

  #onTaskPositionInput = (): void => {
    const roll = this.#taskPos.roll.value;
    const pitch = this.#taskPos.pitch.value;
    const yaw = this.#taskPos.yaw.value;

    const pose: Pose = {
      position: {
        x: this.#taskPos.x.value,
        y: this.#taskPos.y.value,
        z: this.#taskPos.z.value,
      },
      orientation: rpyToQuaternion(roll, pitch, yaw),
    };

    const msg: TaskPositionWaypoint = {
      pose: {
        header: {
          stamp: { sec: 0, nanosec: 0 },
          frame_id: this.#latestState?.header.frame_id ?? "",
        },
        pose,
      },
      tolerance: TASK_POS_TOLERANCE,
    };
    this.#ros.publish(ARM_TASK_POS_WP_TOPIC, ARM_TASK_POS_WP_TYPE, msg);
  };

  #sendTaskVelocity(axis: Axis6 | null, direction: number): void {
    const goal = zeroTwist();
    if (axis !== null) {
      const speed =
        axis === "x" || axis === "y" || axis === "z" ? LINEAR_JOG_MPS : ANGULAR_JOG_RADPS;
      const value = direction * speed;
      switch (axis) {
        case "x":
          goal.linear.x = value;
          break;
        case "y":
          goal.linear.y = value;
          break;
        case "z":
          goal.linear.z = value;
          break;
        case "roll":
          goal.angular.x = value;
          break;
        case "pitch":
          goal.angular.y = value;
          break;
        case "yaw":
          goal.angular.z = value;
          break;
      }
    }
    const msg: TaskVelocityWaypoint = { goal, tolerance: zeroTwist() };
    this.#ros.publish(ARM_TASK_VEL_WP_TOPIC, ARM_TASK_VEL_WP_TYPE, msg);
  }

  #onGripperInput = (): void => {
    const msg: JointPositionWaypoint = {
      positions: [this.#gripperSlider.value],
      tolerances: [GRIPPER_TOLERANCE],
    };
    this.#ros.publish(ARM_GRIPPER_WP_TOPIC, ARM_GRIPPER_WP_TYPE, msg);
  };
}
