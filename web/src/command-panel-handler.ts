// Ports abv_gui's CommandPanel (abv_gnc/src/abv_gui/src/CommandPanel.cpp) -
// a mode-switching panel for commanding the vehicle directly: Pose,
// Velocity, Path, and Thruster (which also holds the per-thruster T1-T8 and
// per-axis +/- fire buttons). Pose/Velocity/Path mirror CommandPanel's
// "read fields, publish once" send buttons; Thruster/Axis mirror its
// press-and-hold ButtonAdapter + 100ms rclcpp::wall_timer pattern.

import type { Handler } from "./handler";
import type { RosClient } from "./ros";
import type { State, ControllerCommand, Vec3 } from "./messages";
import { must } from "./dom";
import { publishPoseGoal, publishPathCommand } from "./guidance";

const CONTROLLER_COMMAND_TOPIC = "abv/controller/command";
const CONTROLLER_COMMAND_TYPE = "autobot_msgs/msg/ControllerCommand";

const THRUSTER_COUNT = 8;

// +X, -X, +Y, -Y, +Yaw, -Yaw - same labels/order as
// CommandPanel.cpp's makeAxisButton loop
const AXIS_SPECS: { label: string; vec: Vec3 }[] = [
  { label: "+X", vec: { x: 1, y: 0, yaw: 0 } },
  { label: "-X", vec: { x: -1, y: 0, yaw: 0 } },
  { label: "+Y", vec: { x: 0, y: 1, yaw: 0 } },
  { label: "-Y", vec: { x: 0, y: -1, yaw: 0 } },
  { label: "+Yaw", vec: { x: 0, y: 0, yaw: 1 } },
  { label: "-Yaw", vec: { x: 0, y: 0, yaw: -1 } },
];

function zeroVec(): Vec3 {
  return { x: 0, y: 0, yaw: 0 };
}

function thrusterBitstring(onIndex: number | null): string {
  return Array.from({ length: THRUSTER_COUNT }, (_, i) =>
    i === onIndex ? "1" : "0",
  ).join("");
}

type Axis3 = "x" | "y" | "yaw";

// A trio of x/y/yaw input boxes that double as both a command input (user
// types a value, Send reads it) and a live readout (tracks the latest
// State field) - matches CommandPanel::onPoseSync's rule: skip the live
// update for a field that's currently focused or has an edit that hasn't
// been sent yet, so typing never gets clobbered by telemetry.
class SyncedVec3Fields {
  #x: HTMLInputElement;
  #y: HTMLInputElement;
  #yaw: HTMLInputElement;
  #dirty: Record<Axis3, boolean> = { x: false, y: false, yaw: false };

  constructor(x: HTMLInputElement, y: HTMLInputElement, yaw: HTMLInputElement) {
    this.#x = x;
    this.#y = y;
    this.#yaw = yaw;
    for (const [axis, el] of [
      ["x", x],
      ["y", y],
      ["yaw", yaw],
    ] as [Axis3, HTMLInputElement][]) {
      el.addEventListener("input", () => (this.#dirty[axis] = true));
    }
  }

  sync(value: Vec3): void {
    this.#syncOne(this.#x, "x", value.x);
    this.#syncOne(this.#y, "y", value.y);
    this.#syncOne(this.#yaw, "yaw", value.yaw);
  }

  #syncOne(el: HTMLInputElement, axis: Axis3, value: number): void {
    if (document.activeElement === el || this.#dirty[axis]) return;
    el.value = value.toFixed(3);
  }

  read(): Vec3 {
    return {
      x: this.#x.valueAsNumber,
      y: this.#y.valueAsNumber,
      yaw: this.#yaw.valueAsNumber,
    };
  }

  clearDirty(): void {
    this.#dirty = { x: false, y: false, yaw: false };
  }
}

export class CommandPanelHandler implements Handler {
  #ros: RosClient;

  #modeSelect: HTMLSelectElement;
  #modePanels: HTMLElement[];
  #bodyFrameCheckbox: HTMLInputElement;

  #poseFields: SyncedVec3Fields;
  #velFields: SyncedVec3Fields;

  #pathTypeSelect: HTMLSelectElement;
  #pathDuration: HTMLInputElement;

  // press-and-hold "fire" state, shared by every thruster/axis button -
  // only one can be active at a time (see #startFiring)
  #fireHandle: ReturnType<typeof setInterval> | null = null;
  #pendingZero: (() => ControllerCommand) | null = null;

  constructor(ros: RosClient, root: ParentNode) {
    this.#ros = ros;

    this.#modeSelect = must<HTMLSelectElement>(root, "#command-mode-select");
    this.#modePanels = Array.from(
      root.querySelectorAll<HTMLElement>("[data-mode-panel]"),
    );
    this.#bodyFrameCheckbox = must<HTMLInputElement>(
      root,
      "#body-frame-checkbox",
    );
    this.#modeSelect.addEventListener("change", this.#onModeChange);
    this.#onModeChange();

    this.#poseFields = new SyncedVec3Fields(
      must<HTMLInputElement>(root, "#pose-cmd-x"),
      must<HTMLInputElement>(root, "#pose-cmd-y"),
      must<HTMLInputElement>(root, "#pose-cmd-yaw"),
    );
    must<HTMLButtonElement>(root, "#pose-cmd-btn").addEventListener(
      "click",
      this.#onSendPose,
    );

    this.#velFields = new SyncedVec3Fields(
      must<HTMLInputElement>(root, "#vel-cmd-x"),
      must<HTMLInputElement>(root, "#vel-cmd-y"),
      must<HTMLInputElement>(root, "#vel-cmd-yaw"),
    );
    must<HTMLButtonElement>(root, "#vel-cmd-btn").addEventListener(
      "click",
      this.#onSendVelocity,
    );

    this.#pathTypeSelect = must<HTMLSelectElement>(root, "#path-type-select");
    this.#pathDuration = must<HTMLInputElement>(root, "#path-duration");
    must<HTMLButtonElement>(root, "#path-cmd-btn").addEventListener(
      "click",
      this.#onSendPath,
    );

    this.#buildThrusterGrid(must<HTMLElement>(root, "#thruster-grid"));
    this.#buildAxisGrid(must<HTMLElement>(root, "#axis-grid"));

    // Pose and Velocity fields track the live vehicle state unless focused
    // or edited but not yet sent - matches CommandPanel::onPoseSync exactly,
    // extended here to Velocity too
    ros.watch<State>("abv/state", "autobot_msgs/msg/State", (m) => {
      this.#poseFields.sync(m.position);
      this.#velFields.sync(m.velocity);
    });

    let stopBtn = must<HTMLButtonElement>(root, "#stop-btn");
    stopBtn.addEventListener("click", () => {
      const cmd: ControllerCommand = {
        type: "STOP",
        is_global: !this.#bodyFrameCheckbox.checked,
        data: this.#velFields.read(),
        tolerance: zeroVec(),
        thrusters: "",
      };
      this.#publishController(cmd);
    });
  }

  update(): void {}

  dispose(): void {
    this.#stopFiring();
  }

  #onModeChange = (): void => {
    const mode = this.#modeSelect.value;
    for (const panel of this.#modePanels) {
      panel.style.display = panel.dataset.modePanel === mode ? "" : "none";
    }
  };

  #onSendPose = (): void => {
    const { x, y, yaw } = this.#poseFields.read();
    publishPoseGoal(this.#ros, x, y, yaw);
    this.#poseFields.clearDirty();
  };

  #onSendVelocity = (): void => {
    const cmd: ControllerCommand = {
      type: "vel",
      is_global: !this.#bodyFrameCheckbox.checked,
      data: this.#velFields.read(),
      tolerance: zeroVec(),
      thrusters: "",
    };
    this.#publishController(cmd);
    this.#velFields.clearDirty();
  };

  #onSendPath = (): void => {
    publishPathCommand(
      this.#ros,
      this.#pathTypeSelect.value,
      this.#pathDuration.valueAsNumber,
    );
  };

  #buildThrusterGrid(grid: HTMLElement): void {
    for (let i = 1; i <= THRUSTER_COUNT; i++) {
      const index = i - 1;
      const button = this.#makeFireButton(
        `T${i}`,
        () => ({
          type: "thruster",
          is_global: false,
          data: zeroVec(),
          tolerance: zeroVec(),
          thrusters: thrusterBitstring(index),
        }),
        () => ({
          type: "thruster",
          is_global: false,
          data: zeroVec(),
          tolerance: zeroVec(),
          thrusters: thrusterBitstring(null),
        }),
      );
      grid.appendChild(button);
    }
  }

  #buildAxisGrid(grid: HTMLElement): void {
    for (const { label, vec } of AXIS_SPECS) {
      const button = this.#makeFireButton(
        label,
        () => ({
          type: "direction",
          is_global: !this.#bodyFrameCheckbox.checked,
          data: vec,
          tolerance: zeroVec(),
          thrusters: "",
        }),
        () => ({
          type: "direction",
          is_global: !this.#bodyFrameCheckbox.checked,
          data: zeroVec(),
          tolerance: zeroVec(),
          thrusters: "",
        }),
      );
      grid.appendChild(button);
    }
  }

  #makeFireButton(
    label: string,
    cmd: () => ControllerCommand,
    zero: () => ControllerCommand,
  ): HTMLButtonElement {
    const button = document.createElement("button");
    button.className = "btn-warning";
    button.textContent = label;
    button.addEventListener("pointerdown", (e) => {
      e.preventDefault();
      this.#startFiring(cmd, zero);
    });
    // both pointerup and pointerleave stop firing - unlike a physical mouse
    // button, a pointer can leave the element without ever releasing on it,
    // which would otherwise leave this button stuck firing
    button.addEventListener("pointerup", () => this.#stopFiring());
    button.addEventListener("pointerleave", () => this.#stopFiring());
    return button;
  }

  #startFiring(
    cmd: () => ControllerCommand,
    zero: () => ControllerCommand,
  ): void {
    // mutual exclusion - only one button fires at a time, even if a
    // previous press's release was somehow missed
    this.#stopFiring();
    this.#fireHandle = setInterval(() => this.#publishController(cmd()), 100);
    this.#pendingZero = zero;
  }

  #stopFiring(): void {
    if (this.#fireHandle === null) return;
    clearInterval(this.#fireHandle);
    this.#fireHandle = null;
    const zero = this.#pendingZero;
    this.#pendingZero = null;
    if (zero) this.#publishController(zero());
  }

  #publishController(cmd: ControllerCommand): void {
    this.#ros.publish(CONTROLLER_COMMAND_TOPIC, CONTROLLER_COMMAND_TYPE, cmd);
  }
}
