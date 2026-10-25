// Commanding the vehicle from the tabletop view: left-click-drag proposes a
// goal pose, right-click-drag proposes an obstacle, a plain right-click (no
// drag) offers to clear obstacles. Ported from abv_gui's TableTopView.cpp
// (mousePressEvent/mouseMoveEvent/mouseReleaseEvent +
// drawGoalGhost/drawObstacleGhost/drawObstacles) and the confirm-menu +
// publish wiring in its main.cpp.
//
// Both the goal and obstacle ghost are positioned at the drag's START point
// and only grow/rotate as the drag continues - dragging does not move the
// position, only the heading (goal) or radius (obstacle). This matches
// TableTopView.cpp exactly: mGoalX/mGoalY and mObstacleX/mObstacleY are set
// once in mousePressEvent and never reassigned in mouseMoveEvent.

import type { Handler } from "./handler";
import type { RosClient } from "./ros";
import type { AbvObstacle, AbvObstacleArray } from "./messages";
import { must } from "./dom";
import { showConfirmPopup } from "./confirm-popup";
import { ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH } from "./telemetry-handler";
import { svgEl } from "./svg";
import { publishPoseGoal } from "./guidance";
import { setObstaclesPresent } from "./obstacle-state";

// matches TableTopView.cpp's kMinDragPixels - below this, a drag reads as
// "no real intent" (yaw stays 0 / a right-click reads as "clear obstacles"
// instead of "add obstacle")
const MIN_DRAG_PIXELS = 18;

const OBSTACLES_TOPIC = "abv/scene/obstacles";
const OBSTACLES_TYPE = "abv_msgs/msg/AbvObstacleArray";

// ghost/obstacle colors, ported from TableTopView.cpp's
// drawGoalGhost/drawObstacleGhost/drawObstacles QColor literals
const GOAL_GHOST_COLOR = "#78c8ff";
const OBSTACLE_GHOST_COLOR = "#ff8c64";
const OBSTACLE_COLOR = "#dc503c";

export class TableInteractionHandler implements Handler {
  #ros: RosClient;
  #svg: SVGSVGElement;
  #world: SVGGElement;

  // goal drag state
  #draggingGoal = false;
  #goalStartClient: [number, number] = [0, 0];
  #goalX = 0;
  #goalY = 0;
  #goalYaw = 0;

  // obstacle drag state
  #draggingObstacle = false;
  #obstacleStartClient: [number, number] = [0, 0];
  #obstacleX = 0;
  #obstacleY = 0;
  #obstacleRadius = 0;
  #obstacleDragExceededThreshold = false;

  // locally-tracked committed obstacles - this client is the source of
  // truth for what IT has placed, same single-writer assumption abv_gui's
  // own main.cpp makes (not synced from the topic)
  #obstacles: AbvObstacle[] = [];

  // ghost/committed-obstacle elements, built once inside #world (after
  // #vehicle, so they paint on top - same order as TableTopView.cpp's
  // paintEvent) and updated in place rather than recreated per drag
  #goalGhost: SVGGElement;
  #obstacleGhost: SVGCircleElement;
  #obstaclesGroup: SVGGElement;

  constructor(ros: RosClient, root: ParentNode) {
    this.#ros = ros;
    this.#svg = must<SVGSVGElement>(root, "#topdown-svg");
    this.#world = must<SVGGElement>(root, "#world");

    this.#obstaclesGroup = svgEl("g");
    this.#goalGhost = this.#buildGoalGhost();
    this.#obstacleGhost = this.#buildObstacleGhost();
    this.#world.appendChild(this.#obstaclesGroup);
    this.#world.appendChild(this.#goalGhost);
    this.#world.appendChild(this.#obstacleGhost);

    this.#svg.addEventListener("mousedown", this.#onMouseDown);
    this.#svg.addEventListener("mousemove", this.#onMouseMove);
    this.#svg.addEventListener("mouseup", this.#onMouseUp);
    // right-click-drag places obstacles - suppress the browser's native
    // context menu so it doesn't pop up alongside our own confirm popup
    // (the web equivalent of Qt::NoContextMenu in TableTopView.cpp)
    this.#svg.addEventListener("contextmenu", (e) => e.preventDefault());
  }

  update(): void {}
  dispose(): void {}

  #buildGoalGhost(): SVGGElement {
    const g = svgEl("g");
    g.style.display = "none";

    const rect = svgEl("rect");
    rect.setAttribute("x", String(-ROBOT_HALF_LENGTH));
    rect.setAttribute("y", String(-ROBOT_HALF_WIDTH));
    rect.setAttribute("width", String(ROBOT_HALF_LENGTH * 2));
    rect.setAttribute("height", String(ROBOT_HALF_WIDTH * 2));
    rect.setAttribute("fill", GOAL_GHOST_COLOR);
    rect.setAttribute("fill-opacity", "0.16");
    rect.setAttribute("stroke", GOAL_GHOST_COLOR);
    rect.setAttribute("stroke-width", "0.012");
    rect.setAttribute("stroke-dasharray", "0.03 0.02");
    g.appendChild(rect);

    const heading = svgEl("line");
    heading.setAttribute("x1", "0");
    heading.setAttribute("y1", "0");
    heading.setAttribute("x2", String(ROBOT_HALF_LENGTH * 1.4));
    heading.setAttribute("y2", "0");
    heading.setAttribute("stroke", GOAL_GHOST_COLOR);
    heading.setAttribute("stroke-width", "0.018");
    g.appendChild(heading);

    return g;
  }

  #buildObstacleGhost(): SVGCircleElement {
    const circle = svgEl("circle");
    circle.style.display = "none";
    circle.setAttribute("cx", "0");
    circle.setAttribute("cy", "0");
    circle.setAttribute("r", "0");
    circle.setAttribute("fill", OBSTACLE_GHOST_COLOR);
    circle.setAttribute("fill-opacity", "0.2");
    circle.setAttribute("stroke", OBSTACLE_GHOST_COLOR);
    circle.setAttribute("stroke-width", "0.012");
    circle.setAttribute("stroke-dasharray", "0.03 0.02");
    return circle;
  }

  #screenToWorld(clientX: number, clientY: number): { x: number; y: number } {
    const ctm = this.#world.getScreenCTM();
    if (!ctm) return { x: 0, y: 0 };
    const p = new DOMPoint(clientX, clientY).matrixTransform(ctm.inverse());
    return { x: p.x, y: p.y };
  }

  #onMouseDown = (e: MouseEvent): void => {
    if (e.button !== 0 && e.button !== 2) return;
    e.preventDefault();
    const world = this.#screenToWorld(e.clientX, e.clientY);

    if (e.button === 0) {
      this.#draggingGoal = true;
      this.#goalStartClient = [e.clientX, e.clientY];
      this.#goalX = world.x;
      this.#goalY = world.y;
      this.#goalYaw = 0;
      this.#updateGoalGhost();
      this.#goalGhost.style.display = "";
    } else {
      // right-drag: no ghost until the drag exceeds the threshold, so a
      // plain right-click reads as "clear obstacles" on release instead
      this.#draggingObstacle = true;
      this.#obstacleStartClient = [e.clientX, e.clientY];
      this.#obstacleX = world.x;
      this.#obstacleY = world.y;
      this.#obstacleRadius = 0;
      this.#obstacleDragExceededThreshold = false;
    }
  };

  #onMouseMove = (e: MouseEvent): void => {
    if (!this.#draggingGoal && !this.#draggingObstacle) return;
    const world = this.#screenToWorld(e.clientX, e.clientY);

    if (this.#draggingGoal) {
      const [startX, startY] = this.#goalStartClient;
      const screenDist = Math.hypot(e.clientX - startX, e.clientY - startY);
      if (screenDist >= MIN_DRAG_PIXELS) {
        const dx = world.x - this.#goalX;
        const dy = world.y - this.#goalY;
        this.#goalYaw = Math.atan2(dy, dx);
      }
      this.#updateGoalGhost();
    } else if (this.#draggingObstacle) {
      const [startX, startY] = this.#obstacleStartClient;
      const screenDist = Math.hypot(e.clientX - startX, e.clientY - startY);
      this.#obstacleRadius = Math.hypot(
        world.x - this.#obstacleX,
        world.y - this.#obstacleY,
      );
      if (screenDist >= MIN_DRAG_PIXELS) {
        this.#obstacleDragExceededThreshold = true;
        this.#obstacleGhost.style.display = "";
      }
      this.#updateObstacleGhost();
    }
  };

  #onMouseUp = (e: MouseEvent): void => {
    if (this.#draggingGoal && e.button === 0) {
      this.#draggingGoal = false;
      const x = this.#goalX;
      const y = this.#goalY;
      const yaw = this.#goalYaw;
      const deg = Math.round((yaw * 180) / Math.PI);

      showConfirmPopup(
        `Send Goal (${x.toFixed(2)}, ${y.toFixed(2)}, ${deg}°)`,
        e.clientX,
        e.clientY,
        "Send",
        () => {
          // left as a static marker of the commanded goal - only cleared by
          // starting a new drag (which repositions this same ghost), matching
          // abv_gui's main.cpp behavior
          this.#publishGoal(x, y, yaw);
        },
        () => {
          this.#goalGhost.style.display = "none";
        },
      );
    } else if (this.#draggingObstacle && e.button === 2) {
      this.#draggingObstacle = false;

      if (this.#obstacleDragExceededThreshold) {
        const x = this.#obstacleX;
        const y = this.#obstacleY;
        const radius = this.#obstacleRadius;

        showConfirmPopup(
          `Add Obstacle (${x.toFixed(2)}, ${y.toFixed(2)}, r=${radius.toFixed(2)})`,
          e.clientX,
          e.clientY,
          "Add",
          () => {
            this.#obstacles.push({ x, y, radius });
            this.#publishObstacles();
            this.#refreshObstaclesView();
            this.#obstacleGhost.style.display = "none";
          },
          () => {
            this.#obstacleGhost.style.display = "none";
          },
        );
      } else {
        // plain right-click, no real drag - offer to clear instead
        showConfirmPopup(
          "Clear Obstacles",
          e.clientX,
          e.clientY,
          "Clear",
          () => {
            this.#obstacles = [];
            this.#publishObstacles();
            this.#refreshObstaclesView();
          },
        );
      }
    }
  };

  #updateGoalGhost(): void {
    const deg = (this.#goalYaw * 180) / Math.PI;
    this.#goalGhost.setAttribute(
      "transform",
      `translate(${this.#goalX} ${this.#goalY}) rotate(${deg})`,
    );
  }

  #updateObstacleGhost(): void {
    this.#obstacleGhost.setAttribute("cx", String(this.#obstacleX));
    this.#obstacleGhost.setAttribute("cy", String(this.#obstacleY));
    this.#obstacleGhost.setAttribute("r", String(this.#obstacleRadius));
  }

  #refreshObstaclesView(): void {
    this.#obstaclesGroup.replaceChildren(
      ...this.#obstacles.map((o) => {
        const circle = svgEl("circle");
        circle.setAttribute("cx", String(o.x));
        circle.setAttribute("cy", String(o.y));
        circle.setAttribute("r", String(o.radius));
        circle.setAttribute("fill", OBSTACLE_COLOR);
        circle.setAttribute("fill-opacity", "0.35");
        circle.setAttribute("stroke", OBSTACLE_COLOR);
        circle.setAttribute("stroke-width", "0.012");
        return circle;
      }),
    );
  }

  #publishGoal(x: number, y: number, yaw: number): void {
    publishPoseGoal(this.#ros, x, y, yaw);
  }

  #publishObstacles(): void {
    setObstaclesPresent(this.#obstacles.length > 0);
    const msg: AbvObstacleArray = { obstacles: this.#obstacles };
    this.#ros.publish(OBSTACLES_TOPIC, OBSTACLES_TYPE, msg);
  }
}
