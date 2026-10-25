// Shared obstacle-presence flag, bridging TableInteractionHandler's locally
// tracked obstacle set to anything else that needs to know whether obstacle
// avoidance should be requested (e.g. CommandPanelHandler's Pose send) -
// mirrors main.cpp's CommandPanel::setObstaclesPresent wiring in abv_gui.

let obstaclesPresent = false;

export function setObstaclesPresent(present: boolean): void {
  obstaclesPresent = present;
}

export function getObstaclesPresent(): boolean {
  return obstaclesPresent;
}
