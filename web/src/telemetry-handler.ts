import type { Handler } from "./handler";
import type { AbvState, AbvThrusterStatus } from "./messages";
import type { RosClient } from "./ros";
import { XYZPlot } from "./xyz-plot";
import { must } from "./dom"; // wherever your helper lives

type Readout = { el: HTMLInputElement; get: (m: AbvState) => number };

const READOUTS: Record<string, (m: AbvState) => number> = {
  "x-pose": (m) => m.position.x,
  "y-pose": (m) => m.position.y,
  "yaw-pose": (m) => m.position.yaw,
  "x-vel": (m) => m.velocity.x,
  "y-vel": (m) => m.velocity.y,
  "yaw-vel": (m) => m.velocity.yaw,
};

export class TelemetryHandler implements Handler {
  #plots: XYZPlot[];
  #readouts: Readout[] = [];
  #pending?: AbvState;
  #vehicle: SVGRectElement;

  constructor(ros: RosClient, root: ParentNode) {
    const pos = new XYZPlot(root, "#plot-pos", "Position");
    const vel = new XYZPlot(root, "#plot-vel", "Velocity");
    const inp = new XYZPlot(root, "#plot-input", "Control Input");
    this.#plots = [pos, vel, inp];

    for (const [id, get] of Object.entries(READOUTS)) {
      const el = must<HTMLInputElement>(root, `#${id}`);
      el.readOnly = true; // display only
      this.#readouts.push({ el, get });
    }

    ros.watch<AbvState>("abv/state", "abv_msgs/msg/AbvState", (m) => {
      const t = performance.now() / 1000;
      pos.push(t, m.position.x, m.position.y, m.position.yaw);
      vel.push(t, m.velocity.x, m.velocity.y, m.velocity.yaw);
      this.#pending = m; // render later, in update()
    });

    ros.watch<AbvThrusterStatus>(
      "abv/thrusters/status",
      "abv_msgs/msg/AbvThrusterStatus",
      (s) => inp.push(performance.now() / 1000, s.fx, s.fy, s.fz),
    );

    this.#vehicle = must<SVGGElement>(root, "#vehicle");
  }

  update() {
    for (const p of this.#plots) p.redraw();

    const m = this.#pending;
    if (!m) return;
    this.#pending = undefined;
    for (const { el, get } of this.#readouts) el.value = get(m).toFixed(3);

    const { x, y, yaw } = m.position;
    const deg = (yaw * 180) / Math.PI; // assuming yaw is in radians
    this.#vehicle.setAttribute(
      "transform",
      `translate(${x} ${y}) rotate(${deg})`,
    );
  }

  dispose(): void {}
}
