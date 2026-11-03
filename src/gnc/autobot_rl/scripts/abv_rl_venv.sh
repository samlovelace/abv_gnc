#!/bin/bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate safetyrl-env
exec python3 -m autobot_rl.rl_policy_node "$@"