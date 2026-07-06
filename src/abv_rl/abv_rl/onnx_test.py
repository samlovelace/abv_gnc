import torch
import numpy as np
import onnxruntime as ort
import sys
sys.path.insert(0, '/abv_gnc/src/abv_rl/abv_rl')

from model import GaussianPolicy
from config import ceConfig

# ── config (match your actual training config) ────────────────────────────────
STATE_DIM  = 15   # 6 + 3*3 obstacles
ACTION_DIM = 3
ARCH       = [256, 256]

cfg = ceConfig(ACTIVATION="ReLU", A_ARCHITECTURE=ARCH)
action_space = None  # uses default scale=1, bias=0

# ── load .pt model ────────────────────────────────────────────────────────────
pro_pt = GaussianPolicy(cfg, [STATE_DIM] + ARCH + [ACTION_DIM], ACTION_DIM,
                        action_space, conditioned_sigma=True)
pro_pt.load_state_dict(torch.load(
    "best_models/protagonist.pt",
    map_location="cpu"))
pro_pt.eval()

# ── load ONNX model ───────────────────────────────────────────────────────────
sess = ort.InferenceSession("abv_rl_onnx/protagonist.onnx",
                            providers=["CPUExecutionProvider"])

# ── test on identical inputs ──────────────────────────────────────────────────
np.random.seed(42)
inputs = [
    np.zeros((1, STATE_DIM), dtype=np.float32),          # all zeros
    np.ones((1, STATE_DIM),  dtype=np.float32),          # all ones
    np.random.randn(1, STATE_DIM).astype(np.float32),    # random
]

print(f"{'Input':<8} {'PT output':<40} {'ONNX output':<40} {'Max diff'}")
print("-" * 100)

for i, x_np in enumerate(inputs):
    x_t = torch.from_numpy(x_np)
    with torch.no_grad():
        mean = pro_pt.mu_head(x_t)
        mean = torch.tanh(mean)
        # action_scale/bias are 1.0/0.0 if action_space=None, so this is just tanh(mean)
        pt_out = mean.numpy()

    onnx_out = sess.run(None, {"state": x_np})[0]

    diff = np.abs(pt_out - onnx_out).max()
    print(f"Input {i:<3} PT={pt_out}  ONNX={onnx_out}  max_diff={diff:.2e}")
    if diff > 1e-4:
        print(f"  *** MISMATCH on input {i} ***")