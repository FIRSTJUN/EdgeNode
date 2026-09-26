import time

import cv2
import numpy as np
import rclpy
import torch
import torchvision.transforms as transforms

from PIL import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from utils.config import Config
from model.model_culane import parsingNet


CFG_PATH = "/workspace/Ultra-Fast-Lane-Detection-v2/configs/culane_res18.py"
WEIGHT_PATH = "/workspace/Ultra-Fast-Lane-Detection-v2/weights/culane_res18.pth"


def pred2coords(
    pred,
    row_anchor,
    col_anchor,
    local_width=1,
    original_image_width=640,
    original_image_height=480,
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

    # Inner lanes
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

    # Outer lanes
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


class MoraiUFLDTest(Node):

    def __init__(self):
        super().__init__("morai_ufld_test")

        self.get_logger().info("Loading UFLD-v2...")

        self.cfg = Config.fromfile(CFG_PATH)

        self.row_anchor = np.linspace(
            0.42, 1.0, self.cfg.num_row
        )

        self.col_anchor = np.linspace(
            0.0, 1.0, self.cfg.num_col
        )

        self.net = parsingNet(
            pretrained=False,
            backbone=self.cfg.backbone,
            num_grid_row=self.cfg.num_cell_row,
            num_cls_row=self.cfg.num_row,
            num_grid_col=self.cfg.num_cell_col,
            num_cls_col=self.cfg.num_col,
            num_lane_on_row=self.cfg.num_lanes,
            num_lane_on_col=self.cfg.num_lanes,
            use_aux=self.cfg.use_aux,
            input_height=self.cfg.train_height,
            input_width=self.cfg.train_width,
            fc_norm=self.cfg.fc_norm,
        ).cuda()

        checkpoint = torch.load(
            WEIGHT_PATH,
            map_location="cpu",
        )["model"]

        state_dict = {}

        for key, value in checkpoint.items():
            if key.startswith("module."):
                key = key[7:]

            state_dict[key] = value

        self.net.load_state_dict(
            state_dict,
            strict=True,
        )

        self.net.eval()

        torch.backends.cudnn.benchmark = True

        self.transform = transforms.Compose([
            transforms.Resize(
                (
                    int(
                        self.cfg.train_height /
                        self.cfg.crop_ratio
                    ),
                    self.cfg.train_width,
                )
            ),
            transforms.ToTensor(),
            transforms.Normalize(
                (0.485, 0.456, 0.406),
                (0.229, 0.224, 0.225),
            ),
        ])

        self.debug_pub = self.create_publisher(
            CompressedImage,
            "/perception/debug/ufld_image/compressed",
            1,
        )

        self.create_subscription(
            CompressedImage,
            "/camera/image/compressed",
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.last_time = None
        self.fps = 0.0

        self.get_logger().info(
            "UFLD-v2 ready on "
            + torch.cuda.get_device_name(0)
        )

        self.get_logger().info(
            "Waiting for /camera/image/compressed ..."
        )

    def image_callback(self, msg):
        image = cv2.imdecode(
            np.frombuffer(
                msg.data,
                dtype=np.uint8,
            ),
            cv2.IMREAD_COLOR,
        )

        if image is None:
            return

        height, width = image.shape[:2]

        rgb = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2RGB,
        )

        pil = Image.fromarray(rgb)

        tensor = self.transform(pil)

        tensor = tensor[
            :,
            -self.cfg.train_height:,
            :
        ]

        tensor = (
            tensor
            .unsqueeze(0)
            .cuda(non_blocking=True)
        )

        start = time.perf_counter()

        with torch.no_grad():
            pred = self.net(tensor)

        valid_row = pred["exist_row"].argmax(1)
        valid_col = pred["exist_col"].argmax(1)

        row_counts = [
            int(valid_row[0, :, i].sum().item())
            for i in range(4)
        ]

        col_counts = [
            int(valid_col[0, :, i].sum().item())
            for i in range(4)
        ]    

        torch.cuda.synchronize()

        inference_ms = (
            time.perf_counter() - start
        ) * 1000.0

        coords = pred2coords(
            pred,
            self.row_anchor,
            self.col_anchor,
            original_image_width=width,
            original_image_height=height,
        )

        debug = image.copy()

        lane_count = 0

        for lane in coords:
            if len(lane) < 2:
                continue

            lane_count += 1

            points = np.array(
                lane,
                dtype=np.int32,
            )

            cv2.polylines(
                debug,
                [points],
                False,
                (0, 255, 0),
                3,
            )

            for x, y in lane:
                cv2.circle(
                    debug,
                    (x, y),
                    4,
                    (0, 255, 0),
                    -1,
                )

        now = time.perf_counter()

        if self.last_time is not None:
            current_fps = 1.0 / max(
                now - self.last_time,
                1e-6,
            )

            if self.fps == 0.0:
                self.fps = current_fps
            else:
                self.fps = (
                    0.9 * self.fps +
                    0.1 * current_fps
                )

        self.last_time = now

        cv2.putText(
            debug,
            f"UFLD lanes={lane_count} "
            f"infer={inference_ms:.1f}ms "
            f"fps={self.fps:.1f}",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            debug,
            f"row={row_counts} col={col_counts}",
            (15, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )

        ok, encoded = cv2.imencode(
            ".jpg",
            debug,
            [int(cv2.IMWRITE_JPEG_QUALITY), 80],
        )

        if not ok:
            return

        output = CompressedImage()

        output.header = msg.header
        output.format = "jpeg"
        output.data = encoded.tobytes()

        self.debug_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)

    node = MoraiUFLDTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()