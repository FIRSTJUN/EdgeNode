import time
import torch

from utils.config import Config
from model.model_culane import parsingNet

CFG_PATH = "configs/culane_res18.py"
WEIGHT_PATH = "weights/culane_res18.pth"

print("1) Loading config...")
cfg = Config.fromfile(CFG_PATH)

print("2) Creating UFLD-v2 model...")
net = parsingNet(
    pretrained=False,
    backbone=cfg.backbone,
    num_grid_row=cfg.num_cell_row,
    num_cls_row=cfg.num_row,
    num_grid_col=cfg.num_cell_col,
    num_cls_col=cfg.num_col,
    num_lane_on_row=cfg.num_lanes,
    num_lane_on_col=cfg.num_lanes,
    use_aux=cfg.use_aux,
    input_height=cfg.train_height,
    input_width=cfg.train_width,
    fc_norm=cfg.fc_norm,
).cuda()

print("3) Loading pretrained checkpoint...")
checkpoint = torch.load(WEIGHT_PATH, map_location="cpu")["model"]

state_dict = {}
for key, value in checkpoint.items():
    if key.startswith("module."):
        key = key[7:]
    state_dict[key] = value

net.load_state_dict(state_dict, strict=True)
net.eval()

print("4) Model loaded successfully")
print("   GPU:", torch.cuda.get_device_name(0))
print("   Input:", cfg.train_width, "x", cfg.train_height)

dummy = torch.zeros(
    (1, 3, cfg.train_height, cfg.train_width),
    dtype=torch.float32,
    device="cuda",
)

print("5) Running GPU inference...")

with torch.no_grad():
    for _ in range(3):
        pred = net(dummy)

    torch.cuda.synchronize()
    start = time.perf_counter()

    pred = net(dummy)

    torch.cuda.synchronize()
    elapsed = time.perf_counter() - start

print("\n=== UFLD-v2 GPU SMOKE TEST SUCCESS ===")
print(f"Inference time: {elapsed * 1000:.2f} ms")
print(f"Approx FPS: {1.0 / elapsed:.2f}")

for name, tensor in pred.items():
    print(name, tuple(tensor.shape))

print(
    "GPU memory:",
    f"{torch.cuda.memory_allocated() / 1024**2:.1f} MiB"
)
