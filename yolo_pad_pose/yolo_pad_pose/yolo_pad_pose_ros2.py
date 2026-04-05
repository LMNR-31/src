import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge

import numpy as np

from message_filters import Subscriber, ApproximateTimeSynchronizer

from ultralytics import YOLO

import tf2_ros
from tf2_geometry_msgs import do_transform_point


def robust_depth_m(depth_m: np.ndarray, u: int, v: int, win: int = 7) -> float:
    h, w = depth_m.shape[:2]
    r = win // 2
    u0, u1 = max(0, u - r), min(w, u + r + 1)
    v0, v1 = max(0, v - r), min(h, v + r + 1)
    patch = depth_m[v0:v1, u0:u1]
    vals = patch[np.isfinite(patch) & (patch > 0.0)]
    if vals.size == 0:
        return float("nan")
    return float(np.median(vals))


class YoloPadPose(Node):
    def __init__(self):
        super().__init__("yolo_pad_pose")

        # Params
        self.declare_parameter("model_path", "best.pt")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("class_pad", 0)
        self.declare_parameter("target_frame", "uav1/base_link")
        self.declare_parameter("fixed_z", 1.5)

        # Topics (from your system)
        self.declare_parameter("front_rgb",  "/uav1/rgbd_front/color/image_raw")
        self.declare_parameter("front_depth","/uav1/rgbd_front/depth/image_raw")
        self.declare_parameter("front_info", "/uav1/rgbd_front/color/camera_info")

        self.declare_parameter("down_rgb",   "/uav1/rgbd_down/color/image_raw")
        self.declare_parameter("down_depth", "/uav1/rgbd_down/depth/image_raw")
        self.declare_parameter("down_info",  "/uav1/rgbd_down/color/camera_info")

        self.model_path = self.get_parameter("model_path").value
        self.conf = float(self.get_parameter("conf").value)
        self.class_pad = int(self.get_parameter("class_pad").value)
        self.target_frame = self.get_parameter("target_frame").value
        self.fixed_z = float(self.get_parameter("fixed_z").value)

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher
        self.pub = self.create_publisher(PointStamped, "/landing_pad/relative_position", 10)

        qos = rclpy.qos.qos_profile_sensor_data

        # FRONT sync
        self.sub_f_rgb = Subscriber(self, Image, self.get_parameter("front_rgb").value, qos_profile=qos)
        self.sub_f_depth = Subscriber(self, Image, self.get_parameter("front_depth").value, qos_profile=qos)
        self.sub_f_info = Subscriber(self, CameraInfo, self.get_parameter("front_info").value, qos_profile=qos)
        self.ts_front = ApproximateTimeSynchronizer([self.sub_f_rgb, self.sub_f_depth, self.sub_f_info], 10, 0.10)
        self.ts_front.registerCallback(lambda rgb, depth, info: self.process(rgb, depth, info, "front"))

        # DOWN sync
        self.sub_d_rgb = Subscriber(self, Image, self.get_parameter("down_rgb").value, qos_profile=qos)
        self.sub_d_depth = Subscriber(self, Image, self.get_parameter("down_depth").value, qos_profile=qos)
        self.sub_d_info = Subscriber(self, CameraInfo, self.get_parameter("down_info").value, qos_profile=qos)
        self.ts_down = ApproximateTimeSynchronizer([self.sub_d_rgb, self.sub_d_depth, self.sub_d_info], 10, 0.10)
        self.ts_down.registerCallback(lambda rgb, depth, info: self.process(rgb, depth, info, "down"))

        self.get_logger().info("Publishing /landing_pad/relative_position (x=right, y=front, z=fixed).")

    def process(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo, source: str):
        fx = info_msg.k[0]
        fy = info_msg.k[4]
        cx = info_msg.k[2]
        cy = info_msg.k[5]
        if fx <= 0.0 or fy <= 0.0:
            return

        # Convert
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warning(f"[{source}] cv_bridge error: {e}")
            return

        # Your sim is 32FC1 meters
        if depth_msg.encoding == "32FC1":
            depth_m = depth.astype(np.float32)
        elif depth_msg.encoding == "16UC1":
            depth_m = depth.astype(np.float32) * 0.001
        else:
            depth_m = depth.astype(np.float32)

        # YOLO
        res = self.model.predict(source=bgr, conf=self.conf, verbose=False)[0]
        if res.boxes is None or len(res.boxes) == 0:
            return

        # best pad
        best = None
        for b in res.boxes:
            cls = int(b.cls.item())
            if cls != self.class_pad:
                continue
            conf = float(b.conf.item())
            x1, y1, x2, y2 = map(float, b.xyxy[0].tolist())
            if best is None or conf > best[0]:
                best = (conf, x1, y1, x2, y2)
        if best is None:
            return

        conf, x1, y1, x2, y2 = best
        u = int(round((x1 + x2) * 0.5))
        v = int(round((y1 + y2) * 0.5))

        h, w = depth_m.shape[:2]
        u = int(np.clip(u, 0, w - 1))
        v = int(np.clip(v, 0, h - 1))

        Z = robust_depth_m(depth_m, u, v, win=7)
        if not np.isfinite(Z) or Z <= 0.0:
            return

        # Camera optical frame: X right, Y down, Z forward
        Xc = (u - cx) * Z / fx
        Yc = (v - cy) * Z / fy
        Zc = Z

        p_cam = PointStamped()
        p_cam.header = depth_msg.header  # frame_id: uav1/.../depth_optical
        p_cam.point = Point(x=float(Xc), y=float(Yc), z=float(Zc))

        # TF to base_link
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                p_cam.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            p_base = do_transform_point(p_cam, tf)
        except Exception as e:
            self.get_logger().warn(f"[{source}] TF failed {p_cam.header.frame_id} -> {self.target_frame}: {e}")
            return

        # base_link (usual): x forward, y left, z up
        front = p_base.point.x
        right = -p_base.point.y

        out = PointStamped()
        out.header = p_base.header
        out.header.frame_id = self.target_frame
        out.point.x = float(right)     # x = right (your convention)
        out.point.y = float(front)     # y = front
        out.point.z = float(self.fixed_z)

        self.pub.publish(out)

        self.get_logger().info(
            f"[{source}] conf={conf:.2f} right(x)={out.point.x:.2f}m front(y)={out.point.y:.2f}m depthZ={Z:.2f}m"
        )


def main():
    rclpy.init()
    node = YoloPadPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()