import type { Handler } from "./handler";
import type { RosClient } from "./ros";
import { PolicyHandler } from "./policy-handler";
import { TelemetryHandler } from "./telemetry-handler";
import { TableInteractionHandler } from "./table-interaction-handler";
import { CommandPanelHandler } from "./command-panel-handler";
import { ImageHandler } from "./image-handler";

// The fixed set of panel kinds a robot's panels.json may name. Each one maps
// to exactly one <template> in index.html and one Handler class - robots
// pick a subset + order from this registry rather than supplying arbitrary
// markup, so an unknown/misspelled name degrades gracefully (see main.ts)
// instead of producing a broken layout.
export interface PanelDef {
  templateId: string;
  // Where the cloned template content gets mounted in the live document.
  mount: (content: DocumentFragment) => void;
  Handler: new (ros: RosClient, root: ParentNode) => Handler;
}

// The "main" area (topdown/telemetry/camera) sits left of the always-present
// .right-column, which holds the command panel and, below it, the policy
// selector - inserting before .right-column (rather than appending to
// .main-layout directly) keeps that column pinned to the right regardless
// of what else is enabled.
function mountInMainLayout(content: DocumentFragment): void {
  const mainLayout = document.querySelector(".main-layout")!;
  mainLayout.insertBefore(content, mainLayout.querySelector(".right-column"));
}

function mountInRightColumn(content: DocumentFragment): void {
  document.querySelector(".right-column")!.appendChild(content);
}

export const PANEL_REGISTRY: Record<string, PanelDef> = {
  policy: {
    templateId: "panel-policy",
    mount: mountInRightColumn,
    Handler: PolicyHandler,
  },
  telemetry: {
    templateId: "panel-telemetry",
    mount: mountInMainLayout,
    Handler: TelemetryHandler,
  },
  topdown: {
    templateId: "panel-topdown",
    mount: mountInMainLayout,
    Handler: TableInteractionHandler,
  },
  command: {
    templateId: "panel-command",
    mount: mountInRightColumn,
    Handler: CommandPanelHandler,
  },
  camera: {
    templateId: "panel-camera",
    mount: mountInMainLayout,
    Handler: ImageHandler,
  },
};

export const DEFAULT_PANELS = ["telemetry", "topdown", "command", "policy", "camera"];
