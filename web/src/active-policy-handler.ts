import { must } from "./dom";
import type { Handler } from "./handler";
import type { RosClient } from "./ros";
import type { PolicyActive } from "./messages";

const POLICY_ACTIVE_TOPIC = "abv/policy/active";
const POLICY_ACTIVE_TYPE = "autobot_msgs/msg/PolicyActive";

// Always-on topbar display of the currently active policy - unlike the
// policy selector (an optional panel, see PolicyHandler), this is core
// chrome: every robot gets it regardless of its panels.json, the same way
// the connection indicator and stop button are always present.
export class ActivePolicyHandler implements Handler {
  private activeEl: HTMLElement;

  constructor(
    private ros: RosClient,
    root: ParentNode,
  ) {
    this.activeEl = must<HTMLElement>(root, "#active-policy");
    ros.watch(POLICY_ACTIVE_TOPIC, POLICY_ACTIVE_TYPE);
  }

  update(): void {
    const policy = this.ros.latest<PolicyActive>(POLICY_ACTIVE_TOPIC);
    this.activeEl.textContent = policy?.policy_name ?? "--";
  }

  dispose(): void {}
}
