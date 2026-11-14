import { RosClient } from "./ros";
import type { Handler } from "./handler";
import { PANEL_REGISTRY, DEFAULT_PANELS } from "./panel-registry";
import { ActivePolicyHandler } from "./active-policy-handler";

const hostname = window.location.hostname;
const ros = new RosClient(`ws://${hostname}:9090`);

async function loadPanelNames(): Promise<string[]> {
  try {
    const res = await fetch("./panels.json");
    if (!res.ok) throw new Error(`panels.json: ${res.status}`);
    const parsed = await res.json();
    if (!Array.isArray(parsed.panels)) throw new Error("panels.json: missing \"panels\" array");
    return parsed.panels;
  } catch (e) {
    console.warn("falling back to default panel set:", e);
    return DEFAULT_PANELS;
  }
}

function mountPanels(names: string[]): Handler[] {
  const known = names.filter((name) => {
    if (PANEL_REGISTRY[name]) return true;
    console.warn(`unknown panel "${name}" in panels.json, skipping`);
    return false;
  });

  // Clone + mount every panel's markup first, so cross-panel element lookups
  // (e.g. CommandPanelHandler reaching out to the always-present #stop-btn)
  // resolve correctly once Handlers are constructed below.
  for (const name of known) {
    const def = PANEL_REGISTRY[name];
    const template = document.getElementById(def.templateId) as HTMLTemplateElement;
    def.mount(template.content.cloneNode(true) as DocumentFragment);
  }

  // Construct each Handler independently - one panel failing to initialize
  // (e.g. a missing dependency, a third-party library throwing) must not
  // prevent the rest of the page from working.
  const handlers: Handler[] = [];
  for (const name of known) {
    try {
      handlers.push(new PANEL_REGISTRY[name].Handler(ros, document.body));
    } catch (e) {
      console.error(`panel "${name}" failed to initialize:`, e);
    }
  }
  return handlers;
}

async function main() {
  const panelNames = await loadPanelNames();
  const handlers: Handler[] = [new ActivePolicyHandler(ros, document.body)];
  handlers.push(...mountPanels(panelNames));

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
}

main();
