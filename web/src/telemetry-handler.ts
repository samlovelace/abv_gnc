import type { Handler } from "./handler";
import type { State, ThrusterStatus, Path } from "./messages";
import type { RosClient } from "./ros";
import { XYZPlot } from "./xyz-plot";
import { must } from "./dom";
import { buildVehicleGlyph, type VehicleGlyph } from "./vehicle-glyph";
import { svgEl } from "./svg";

// "Tesla-style" planned-path line - a soft, wide translucent glow pass
// underneath a crisp solid line on top, ported from TableTopView::drawPath's
// QPen literals (glow: QColor(60,140,255,60) width 13.5; line:
// QColor(70,150,255) width 4.5 - converted from screen pixels to the SVG's
// world-unit scale here)
const PATH_GLOW_COLOR = "#3c8cff";
const PATH_GLOW_WIDTH = "0.08";
const PATH_GLOW_OPACITY = "0.235";
const PATH_LINE_COLOR = "#4696ff";
const PATH_LINE_WIDTH = "0.027";

// TODO: pull from the abv config (RobotLength/RobotWidth) instead of
// hardcoding - matches the table-dims TODO already on the topdown SVG.
// Exported since TableInteractionHandler's goal ghost needs the same
// dimensions as the vehicle glyph.
export const ROBOT_HALF_LENGTH = 0.175;
export const ROBOT_HALF_WIDTH = 0.175;
const THRUSTER_OFF_COLOR = "#323235";
const THRUSTER_ON_COLOR = "#f19346";

export class TelemetryHandler implements Handler {
  #plots: XYZPlot[];
  #pending?: State;
  #vehicle: SVGGElement;
  #thrusters: VehicleGlyph;
  #pathWaypoints: { x: number; y: number }[] = [];
  #pathGlow: SVGPolylineElement;
  #pathLine: SVGPolylineElement;

  constructor(ros: RosClient, root: ParentNode) {
    const pos = new XYZPlot(must(root, "#plot-pos"), "Position");
    const vel = new XYZPlot(must(root, "#plot-vel"), "Velocity");
    const inp = new XYZPlot(must(root, "#plot-input"), "Control Input");
    this.#plots = [pos, vel, inp];

    ros.watch<State>("abv/state", "autobot_msgs/msg/State", (m) => {
      const t = performance.now() / 1000;
      pos.push(t, m.position.x, m.position.y, m.position.yaw);
      vel.push(t, m.velocity.x, m.velocity.y, m.velocity.yaw);
      this.#pending = m; // render later, in update()
    });

    this.#vehicle = must<SVGGElement>(root, "#vehicle");

    this.#pathGlow = svgEl("polyline");
    this.#pathGlow.setAttribute("fill", "none");
    this.#pathGlow.setAttribute("stroke", PATH_GLOW_COLOR);
    this.#pathGlow.setAttribute("stroke-opacity", PATH_GLOW_OPACITY);
    this.#pathGlow.setAttribute("stroke-width", PATH_GLOW_WIDTH);
    this.#pathGlow.setAttribute("stroke-linecap", "round");
    this.#pathGlow.setAttribute("stroke-linejoin", "round");

    this.#pathLine = svgEl("polyline");
    this.#pathLine.setAttribute("fill", "none");
    this.#pathLine.setAttribute("stroke", PATH_LINE_COLOR);
    this.#pathLine.setAttribute("stroke-width", PATH_LINE_WIDTH);
    this.#pathLine.setAttribute("stroke-linecap", "round");
    this.#pathLine.setAttribute("stroke-linejoin", "round");

    // drawn before (behind) the vehicle - same order as TableTopView.cpp's
    // paintEvent (drawPath runs before drawRobot) so the vehicle renders on
    // top of the path it's following
    const world = this.#vehicle.parentElement!;
    world.insertBefore(this.#pathGlow, this.#vehicle);
    world.insertBefore(this.#pathLine, this.#vehicle);

    ros.watch<Path>("abv/guidance/path", "autobot_msgs/msg/Path", (p) => {
      this.#pathWaypoints = p.waypoints.map((wp) => ({ x: wp.x, y: wp.y }));
    });

    this.#thrusters = buildVehicleGlyph(
      this.#vehicle,
      ROBOT_HALF_LENGTH,
      ROBOT_HALF_WIDTH,
    );

    ros.watch<ThrusterStatus>(
      "abv/thrusters/status",
      "autobot_msgs/msg/ThrusterStatus",
      (s) => {
        inp.push(performance.now() / 1000, s.fx, s.fy, s.fz);

        for (let i = 0; i < s.thrusters.length; i++) {
          let fillColor: string = THRUSTER_OFF_COLOR;
          if ("1" == s.thrusters.at(i)) {
            fillColor = THRUSTER_ON_COLOR;
          }
          this.#thrusters.thrusters[i].setAttribute("fill", fillColor);
        }
      },
    );
  }

  update() {
    for (const p of this.#plots) p.redraw();

    const m = this.#pending;
    if (!m) return;
    this.#pending = undefined;

    const { x, y, yaw } = m.position;
    const deg = (yaw * 180) / Math.PI; // assuming yaw is in radians
    this.#vehicle.setAttribute(
      "transform",
      `translate(${x} ${y}) rotate(${deg})`,
    );

    // anchor the path to the vehicle's own live position rather than
    // whatever (possibly stale) first point abv_guidance published - see
    // TableTopView::drawPath's comment on the same choice. Empty when there's
    // no path, matching drawPath's `if (mPath.isEmpty()) return;`
    const pathPoints =
      this.#pathWaypoints.length > 0
        ? [{ x, y }, ...this.#pathWaypoints]
            .map((p) => `${p.x},${p.y}`)
            .join(" ")
        : "";
    this.#pathGlow.setAttribute("points", pathPoints);
    this.#pathLine.setAttribute("points", pathPoints);
  }

  dispose(): void {}
}
