import { must } from "./dom";
import type { Handler } from "./handler";

import type { RosClient } from "./ros";
import {
  type SwitchPolicyRequest,
  type PolicyActive,
  type PolicyEntry,
  type PolicyRegistry,
  type SwitchPolicyResponse,
} from "./messages";

const POLICY_ACTIVE_TOPIC = "abv/policy/active";
const POLICY_ACTIVE_TYPE = "autobot_msgs/msg/PolicyActive";
const POLICY_REGISTRY_TOPIC = "abv/policy/registry";
const POLICY_REGISTRY_TYPE = "autobot_msgs/msg/PolicyRegistry";
const POLICY_SWITCH_TYPE = "autobot_msgs/srv/PolicySwitch.srv";
const POLICY_SWITCH_TOPIC = "abv/policy/switch";

// @brief handles
//    * updating active policy banner
//    * updating policy registry selections
//    * sending active policy switch command
export class PolicyHandler implements Handler {
  private registry = new Map<string, PolicyEntry>();
  private lastKey = "";

  private activeEl: HTMLElement;
  private selectEl: HTMLSelectElement;
  private switchBtn: HTMLButtonElement;

  constructor(
    private ros: RosClient,
    root: HTMLElement,
  ) {
    this.activeEl = must<HTMLElement>(root, ".active-policy");
    this.selectEl = must<HTMLSelectElement>(root, "#policy-select");
    this.switchBtn = must<HTMLButtonElement>(root, "#policy-switch-btn");

    ros.watch(POLICY_ACTIVE_TOPIC, POLICY_ACTIVE_TYPE);
    ros.watch(POLICY_REGISTRY_TOPIC, POLICY_REGISTRY_TYPE);

    this.switchBtn.addEventListener("click", this.onSwitchClick);
  }

  update(): void {
    this.updateActive();
    this.updateRegistry();
  }

  private updateActive() {
    const policy = this.ros.latest<PolicyActive>(POLICY_ACTIVE_TOPIC);
    const name = policy?.policy_name;
    if (this.activeEl) this.activeEl.textContent = name ?? "--";
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
