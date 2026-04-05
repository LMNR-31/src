import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class DatasetCapture(Node):
    def __init__(self):
        super().__init__("dataset_capture")

        # ===== Params =====
        self.declare_parameter("split", "train")  # train or val
        self.declare_parameter("base_dir", str(Path.home() / "datasets/landing_pad/dataset"))
        self.declare_parameter("front_topic", "/uav1/rgbd_front/color/image_raw")
        self.declare_parameter("down_topic", "/uav1/rgbd_down/color/image_raw")

        # Capture settings
        self.declare_parameter("max_fps_per_camera", 30.0)   # throttle per camera
        self.declare_parameter("save_png", True)             # True=png, False=jpg
        self.declare_parameter("jpg_quality", 95)            # if save_png=False
        self.declare_parameter("prefix_front", "front_")
        self.declare_parameter("prefix_down", "down_")
        self.declare_parameter("start_index", 0)

        self.split = self.get_parameter("split").value
        self.base_dir = Path(self.get_parameter("base_dir").value)

        self.front_topic = self.get_parameter("front_topic").value
        self.down_topic = self.get_parameter("down_topic").value

        self.max_fps = float(self.get_parameter("max_fps_per_camera").value)
        self.min_dt = 0.0 if self.max_fps <= 0 else (1.0 / self.max_fps)

        self.save_png = bool(self.get_parameter("save_png").value)
        self.jpg_quality = int(self.get_parameter("jpg_quality").value)

        self.prefix_front = self.get_parameter("prefix_front").value
        self.prefix_down = self.get_parameter("prefix_down").value
        self.index = int(self.get_parameter("start_index").value)

        # Output dirs
        self.img_dir = self.base_dir / "images" / self.split
        self.lbl_dir = self.base_dir / "labels" / self.split
        self.img_dir.mkdir(parents=True, exist_ok=True)
        self.lbl_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()

        # per-camera throttle timestamps
        self.last_saved_front = 0.0
        self.last_saved_down = 0.0

        qos = rclpy.qos.qos_profile_sensor_data
        self.sub_front = self.create_subscription(Image, self.front_topic, self.cb_front, qos)
        self.sub_down = self.create_subscription(Image, self.down_topic, self.cb_down, qos)

        self.get_logger().info(f"Capturing split={self.split}")
        self.get_logger().info(f"Front topic: {self.front_topic}")
        self.get_logger().info(f"Down topic:  {self.down_topic}")
        self.get_logger().info(f"Saving into: {self.img_dir} and {self.lbl_dir}")
        self.get_logger().info("Press Ctrl+C to stop.")

    def _save_pair(self, cv_img_bgr, prefix: str):
        ext = "png" if self.save_png else "jpg"
        name = f"{prefix}{self.index:06d}"
        img_path = self.img_dir / f"{name}.{ext}"
        lbl_path = self.lbl_dir / f"{name}.txt"

        # save image
        if self.save_png:
            ok = cv2.imwrite(str(img_path), cv_img_bgr)
        else:
            ok = cv2.imwrite(str(img_path), cv_img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpg_quality])

        if not ok:
            self.get_logger().warning(f"Failed to write {img_path}")
            return

        # create empty label file (if not exists)
        if not lbl_path.exists():
            lbl_path.write_text("", encoding="utf-8")

        self.index += 1

    def cb_front(self, msg: Image):
        now = time.time()
        if (now - self.last_saved_front) < self.min_dt:
            return
        self.last_saved_front = now

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning(f"cv_bridge front error: {e}")
            return

        self._save_pair(bgr, self.prefix_front)

    def cb_down(self, msg: Image):
        now = time.time()
        if (now - self.last_saved_down) < self.min_dt:
            return
        self.last_saved_down = now

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warning(f"cv_bridge down error: {e}")
            return

        self._save_pair(bgr, self.prefix_down)


def main():
    rclpy.init()
    node = DatasetCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
