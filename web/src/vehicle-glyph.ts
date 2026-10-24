// Builds the static parts of the vehicle glyph that live inside #vehicle:
// the 8 thruster nozzles and the camera housing + FOV wedge. Ported from
// abv_gui's TableTopView::drawRobot (abv_gnc/src/abv_gui/src/TableTopView.cpp)
// - same formulas, same T1..T8 corner/direction layout, just computed here
// instead of in C++ so we don't have to hand-place each one in the SVG.
//
// Everything is expressed in the same local body frame already used by the
// hand-authored body rect: +x is forward (long axis), origin at the
// vehicle's center. Each thruster is its own generated <path> (not a <use>
// referencing a shared def) specifically so its fill/stroke can be set
// directly per-element - a shared def's own fill/stroke would otherwise
// win over anything a referencing <use> tries to set, which would get in
// the way of live firing-state coloring later.

import { svgEl } from "./svg";

interface ThrusterSpec {
  pos: [number, number];
  outward: [number, number];
}

// one thruster per corner (not per side) - mirrors thrusterLocal in
// TableTopView.cpp exactly, so index i here lines up with
// AbvThrusterCommand/AbvThrusterStatus.thrusters[i] (T1..T8)
function thrusterLayout(halfLength: number, halfWidth: number): ThrusterSpec[] {
  const leftRightOffset = halfLength * 0.6;
  const frontBackOffset = halfWidth * 0.6;

  return [
    { pos: [-leftRightOffset, -halfWidth], outward: [0, -1] }, // T1
    { pos: [leftRightOffset, -halfWidth], outward: [0, -1] }, // T2
    { pos: [halfLength, -frontBackOffset], outward: [1, 0] }, // T3
    { pos: [halfLength, frontBackOffset], outward: [1, 0] }, // T4
    { pos: [leftRightOffset, halfWidth], outward: [0, 1] }, // T5
    { pos: [-leftRightOffset, halfWidth], outward: [0, 1] }, // T6
    { pos: [-halfLength, frontBackOffset], outward: [-1, 0] }, // T7
    { pos: [-halfLength, -frontBackOffset], outward: [-1, 0] }, // T8
  ];
}

// angle (degrees) that rotates a shape whose default direction is
// `from` so it instead points along `to` - both unit vectors in the same
// local frame used for `rotate()` on this element
function angleBetween(from: [number, number], to: [number, number]): number {
  const a = Math.atan2(to[1], to[0]) - Math.atan2(from[1], from[0]);
  return (a * 180) / Math.PI;
}

// local thruster shape: a small triangle with its apex pointing along -y
// and its base centered on the origin - buildVehicleGlyph rotates/translates
// a copy of this per thruster to place it and point it outward
const THRUSTER_DEFAULT_DIR: [number, number] = [0, -1];
const THRUSTER_HALF_WIDTH = 0.0175;
const THRUSTER_LENGTH = 0.095;
const THRUSTER_PATH = `M ${-THRUSTER_HALF_WIDTH} 0 L 0 ${-THRUSTER_LENGTH} L ${THRUSTER_HALF_WIDTH} 0 Z`;
const THRUSTER_FILL = "#3f3f34";
const THRUSTER_STROKE = "#2c2c31";
const THRUSTER_STROKE_WIDTH = "0.003";

export interface VehicleGlyph {
  // index 0..7 => T1..T8, for wiring up live firing-state coloring later -
  // each is a <path>, so recoloring is just setAttribute("fill", ...)
  thrusters: SVGPathElement[];
}

export function buildVehicleGlyph(
  vehicle: SVGGElement,
  halfLength: number,
  halfWidth: number,
): VehicleGlyph {
  // rendered BEHIND the body rect (see insertBefore below) so the body's
  // own fill occludes each nozzle's base - only the part that pokes past
  // the hull edge is visible, which means the base can sit comfortably
  // buried under the body without needing to fine-tune the inset to avoid
  // a visible gap at the edge
  const bodyRect = vehicle.querySelector("rect");
  const thrusterInset = Math.min(halfLength, halfWidth) * 0.15;

  const thrusters = thrusterLayout(halfLength, halfWidth).map(
    ({ pos, outward }) => {
      const x = pos[0] - outward[0] * thrusterInset;
      const y = pos[1] - outward[1] * thrusterInset;
      const angle = angleBetween(THRUSTER_DEFAULT_DIR, outward);

      const thruster = svgEl("path");
      thruster.setAttribute("d", THRUSTER_PATH);
      thruster.setAttribute(
        "transform",
        `translate(${x} ${y}) rotate(${angle})`,
      );
      thruster.setAttribute("fill", THRUSTER_FILL);
      thruster.setAttribute("stroke", THRUSTER_STROKE);
      thruster.setAttribute("stroke-width", THRUSTER_STROKE_WIDTH);
      vehicle.insertBefore(thruster, bodyRect);
      return thruster;
    },
  );

  // darker corner-bracket accents, one per body corner - mirrors
  // TableTopView.cpp's `corners`/`accent` construction (a small square
  // inset from each corner, suggesting bolted extrusion corner plates)
  const cornerAccent = Math.min(halfLength, halfWidth) * 0.28;
  const corners: [number, number][] = [
    [halfLength, halfWidth],
    [halfLength, -halfWidth],
    [-halfLength, -halfWidth],
    [-halfLength, halfWidth],
  ];
  for (const [cx, cy] of corners) {
    const bracket = svgEl("rect");
    bracket.setAttribute("x", String(cx > 0 ? cx - cornerAccent : cx));
    bracket.setAttribute("y", String(cy > 0 ? cy - cornerAccent : cy));
    bracket.setAttribute("width", String(cornerAccent));
    bracket.setAttribute("height", String(cornerAccent));
    bracket.setAttribute("fill", "#5f5f64");
    vehicle.appendChild(bracket);
  }

  // camera housing + FOV wedge - mirrors TableTopView.cpp's housingLocal/
  // wedgeLocal construction, widened a bit and with the wedge's origin
  // pulled back slightly so it doesn't start flush with the housing's face
  const housingDepth = Math.min(halfLength, halfWidth) * 0.32;
  const housingHalfWidth = halfWidth * 0.3;
  const housingInset = housingDepth * 0.45;
  const housingNear = halfLength - housingInset;
  const housingFar = housingNear + housingDepth;

  const housing = svgEl("polygon");
  housing.setAttribute(
    "points",
    [
      [housingNear, housingHalfWidth],
      [housingFar, housingHalfWidth],
      [housingFar, -housingHalfWidth],
      [housingNear, -housingHalfWidth],
    ]
      .map((p) => p.join(","))
      .join(" "),
  );
  housing.setAttribute("fill", "#323235");
  housing.setAttribute("stroke", "#3c3c41");
  housing.setAttribute("stroke-width", "0.004");
  vehicle.appendChild(housing);

  const wedgeOriginInset = housingDepth * 0.2;
  const wedgeOriginX = housingFar - wedgeOriginInset;
  const wedgeHalfAngle = (32 * Math.PI) / 180;
  const wedgeLength = halfLength * 2 * 0.9;
  const wedgeReach = wedgeLength * Math.sin(wedgeHalfAngle);
  const wedgeTipX = wedgeOriginX + wedgeLength * Math.cos(wedgeHalfAngle);

  const wedge = svgEl("polygon");
  wedge.setAttribute(
    "points",
    [
      [wedgeOriginX, 0],
      [wedgeTipX, wedgeReach],
      [wedgeTipX, -wedgeReach],
    ]
      .map((p) => p.join(","))
      .join(" "),
  );
  wedge.setAttribute("fill", "#d2e6ff");
  wedge.setAttribute("fill-opacity", "0.12");
  // drawn first (behind the body/housing), same order as TableTopView.cpp
  vehicle.insertBefore(wedge, vehicle.firstChild);

  return { thrusters };
}
