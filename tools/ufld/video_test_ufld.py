import cv2
import torch
import numpy as np
from PIL import Image
import torchvision.transforms as transforms

from utils.config import Config
from model.model_culane import parsingNet


CFG_PATH = "configs/culane_res18.py"
WEIGHT_PATH = "weights/culane_res18.pth"
VIDEO_PATH = "example.mp4"
OUTPUT_PATH = "ufld_result.avi"


def pred2coords(
    pred,
    row_anchor,
    col_anchor,
    local_width=1,
    original_image_width=1640,
    original_image_height=590,
):
    _, num_grid_row, num_cls_row, _ = pred["loc_row"].shape
    _, num_grid_col, num_cls_col, _ = pred["loc_col"].shape

    max_indices_row = pred["loc_row"].argmax(1).cpu()
    valid_row = pred["exist_row"].argmax(1).cpu()

    max_indices_col = pred["loc_col"].argmax(1).cpu()
    valid_col = pred["exist_col"].argmax(1).cpu()

    loc_row = pred["loc_row"].cpu()
    loc_col = pred["loc_col"].cpu()

    coords = []

    # Inner two lanes
    for lane_idx in [1, 2]:
        lane = []

        if valid_row[0, :, lane_idx].sum() > num_cls_row / 2:
            for k in range(valid_row.shape[1]):
                if valid_row[0, k, lane_idx]:
                    center = int(max_indices_row[0, k, lane_idx])

                    indices = torch.tensor(
                        list(
                            range(
                                max(0, center - local_width),
                                min(num_grid_row - 1, center + local_width) + 1,
                            )
                        )
                    )

                    value = (
                        loc_row[0, indices, k, lane_idx].softmax(0)
                        * indices.float()
                    ).sum() + 0.5

                    x = value / (num_grid_row - 1) * original_image_width
                    y = row_anchor[k] * original_image_height

                    lane.append((int(x), int(y)))

        coords.append(lane)

    # Outer two lanes
    for lane_idx in [0, 3]:
        lane = []

        if valid_col[0, :, lane_idx].sum() > num_cls_col / 4:
            for k in range(valid_col.shape[1]):
                if valid_col[0, k, lane_idx]:
                    center = int(max_indices_col[0, k, lane_idx])

                    indices = torch.tensor(
                        list(
                            range(
                                max(0, center - local_width),
                                min(num_grid_col - 1, center + local_width) + 1,
                            )
                        )
                    )

                    value = (
                        loc_col[0, indices, k, lane_idx].softmax(0)
                        * indices.float()
                    ).sum() + 0.5

                    y = value / (num_grid_col - 1) * original_image_height
                    x = col_anchor[k] * original_image_width

                    lane.append((int(x), int(y)))

        coords.append(lane)

    return coords


print("Loading config...")
cfg = Config.fromfile(CFG_PATH)

row_anchor = np.linspace(0.42, 1.0, cfg.num_row)
col_anchor = np.linspace(0.0, 1.0, cfg.num_col)

print("Creating model...")
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

print("Loading weights...")
checkpoint = torch.load(
    WEIGHT_PATH,
    map_location="cpu"
)["model"]

state_dict = {}

for key, value in checkpoint.items():
    if key.startswith("module."):
        key = key[7:]

    state_dict[key] = value

net.load_state_dict(state_dict, strict=True)
net.eval()


transform = transforms.Compose([
    transforms.Resize(
        (
            int(cfg.train_height / cfg.crop_ratio),
            cfg.train_width
        )
    ),
    transforms.ToTensor(),
    transforms.Normalize(
        (0.485, 0.456, 0.406),
        (0.229, 0.224, 0.225),
    ),
])


cap = cv2.VideoCapture(VIDEO_PATH)

if not cap.isOpened():
    raise RuntimeError(f"Cannot open {VIDEO_PATH}")

fps = cap.get(cv2.CAP_PROP_FPS)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

print("Video:", width, "x", height, "@", fps, "FPS")

fourcc = cv2.VideoWriter_fourcc(*"MJPG")

writer = cv2.VideoWriter(
    OUTPUT_PATH,
    fourcc,
    fps,
    (width, height),
)

frame_count = 0

with torch.no_grad():
    while True:
        ok, frame = cap.read()

        if not ok:
            break

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil = Image.fromarray(rgb)

        img = transform(pil)

        # Same crop behavior used by UFLD-v2 test loader
        img = img[:, -cfg.train_height:, :]

        img = img.unsqueeze(0).cuda()

        pred = net(img)

        coords = pred2coords(
            pred,
            row_anchor,
            col_anchor,
            original_image_width=width,
            original_image_height=height,
        )

        vis = frame.copy()

        for lane in coords:
            for x, y in lane:
                if 0 <= x < width and 0 <= y < height:
                    cv2.circle(
                        vis,
                        (x, y),
                        5,
                        (0, 255, 0),
                        -1,
                    )

        writer.write(vis)

        frame_count += 1

        if frame_count % 100 == 0:
            print("Processed:", frame_count)


cap.release()
writer.release()

print("DONE")
print("Output:", OUTPUT_PATH)
