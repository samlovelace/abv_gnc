import { RosClient } from "./ros";
import { PolicyHandler } from "./policy-handler";
import type { Handler } from "./handler";
import { TelemetryHandler } from "./telemetry-handler";
import { TableInteractionHandler } from "./table-interaction-handler";
import { CommandPanelHandler } from "./command-panel-handler";

// TODO: get ip from the abv config
const ros = new RosClient(`ws://192.168.1.242:9090`);
const handlers: Handler[] = [
  new PolicyHandler(ros, document.body),
  new TelemetryHandler(ros, document.body),
  new TableInteractionHandler(ros, document.body),
  new CommandPanelHandler(ros, document.body),
];

function frame() {
  for (const h of handlers) {
    try {
      h.update();
    } catch (e) {
      console.error(e);
    }
  }
  requestAnimationFrame(frame);
}

requestAnimationFrame(frame);
