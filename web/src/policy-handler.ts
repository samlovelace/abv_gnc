import { must } from "./dom";
import type { Handler } from "./handler";

import type { RosClient } from "./ros";
import {
  type SwitchPolicyRequest,
  type PolicyEntry,
  type PolicyRegistry,
  type SwitchPolicyResponse,
} from "./messages";

const POLICY_REGISTRY_TOPIC = "abv/policy/registry";
const POLICY_REGISTRY_TYPE = "autobot_msgs/msg/PolicyRegistry";
const POLICY_SWITCH_TYPE = "autobot_msgs/srv/PolicySwitch.srv";
const POLICY_SWITCH_TOPIC = "abv/policy/switch";

// The policy selector panel - lets the operator pick a registered policy and
// switch to it. The always-visible "currently active policy" display is a
// separate, unconditional piece of chrome - see ActivePolicyHandler.
export class PolicyHandler implements Handler {
  private registry = new Map<string, PolicyEntry>();
  private lastKey = "";

  private selectEl: HTMLSelectElement;
  private switchBtn: HTMLButtonElement;

  constructor(
    private ros: RosClient,
    root: ParentNode,
  ) {
    this.selectEl = must<HTMLSelectElement>(root, "#policy-select");
    this.switchBtn = must<HTMLButtonElement>(root, "#policy-switch-btn");

    ros.watch(POLICY_REGISTRY_TOPIC, POLICY_REGISTRY_TYPE);

    this.switchBtn.addEventListener("click", this.onSwitchClick);
  }

  update(): void {
    this.updateRegistry();
  }

  private updateRegistry() {
    // update the policy registry
    const policyReg = this.ros.latest<PolicyRegistry>(POLICY_REGISTRY_TOPIC);
    for (const p of policyReg?.policies ?? []) {
      this.registry.set(p.policy_name, p);
    }

    const aliveNames = [...this.registry.values()]
      .filter((p) => p.alive)
      .map((p) => p.policy_name);

    const key = aliveNames.join("\n");
    if (key === this.lastKey) return; // nothing changed, leave the DOM alone
    this.lastKey = key;

    const selected = this.selectEl.value; // remember the user's choice
    this.selectEl.replaceChildren(
      ...aliveNames.map((name) => new Option(name, name)),
    );
    if (aliveNames.includes(selected)) this.selectEl.value = selected;
  }

  private onSwitchClick = async () => {
    const name = this.selectEl.value;
    if (!name) return;

    this.switchBtn.disabled = true;
    try {
      const res = await this.ros.callService<
        SwitchPolicyRequest,
        SwitchPolicyResponse
      >(POLICY_SWITCH_TOPIC, POLICY_SWITCH_TYPE, { policy_name: name });

      if (!res.success) console.warn("switch rejected:", res.message);
    } catch (e) {
      console.error("switch call failed:", e);
    } finally {
      this.switchBtn.disabled = false;
    }
  };

  dispose(): void {}
}
