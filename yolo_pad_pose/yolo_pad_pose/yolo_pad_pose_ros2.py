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
        # base_class_id / h_class_id map YOLO class indices to semantic roles.
        # Defaults match the dataset convention: base=0, h=1, borda=2.
        # class_pad is kept for backward compatibility: when class_pad != 0 and
        # base_class_id is at its default (0), class_pad is used as base_class_id.
        self.declare_parameter("class_pad", 0)       # deprecated alias
        self.declare_parameter("base_class_id", 0)   # YOLO class id for base
        self.declare_parameter("h_class_id", 1)      # YOLO class id for H marker
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
        self.base_class_id = int(self.get_parameter("base_class_id").value)
        self.h_class_id = int(self.get_parameter("h_class_id").value)
        # Honour legacy class_pad parameter: if it was set to a non-zero value
        # and base_class_id was not explicitly overridden from its default (0),
        # treat class_pad as the base class id and emit a deprecation warning.
        # Note: if both class_pad and base_class_id are changed from their
        # defaults, base_class_id takes precedence.
        _class_pad = int(self.get_parameter("class_pad").value)
        if _class_pad != 0 and self.base_class_id == 0:
            self.base_class_id = _class_pad
            self.get_logger().warning(
                "[DEPRECATED] 'class_pad' parameter is deprecated; "
                f"use 'base_class_id' instead. Using class_pad={_class_pad}."
            )
        self.target_frame = self.get_parameter("target_frame").value
        self.fixed_z = float(self.get_parameter("fixed_z").value)

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        # /landing_pad/base_relative_position — primary topic for base (class 0)
        self.pub_base = self.create_publisher(
            PointStamped, "/landing_pad/base_relative_position", 10)
        # /landing_pad/h_relative_position — primary topic for H marker (class 1)
        self.pub_h = self.create_publisher(
            PointStamped, "/landing_pad/h_relative_position", 10)
        # /landing_pad/relative_position — backward-compatible alias of base topic
        self.pub = self.create_publisher(PointStamped, "/landing_pad/relative_position", 10)
        # Per-camera optical-frame debug topics (keep for backward compat)
        self.pub_front = self.create_publisher(PointStamped, "/landing_pad/front_optical_point", 10)
        self.pub_down = self.create_publisher(PointStamped, "/landing_pad/down_optical_point", 10)

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

        self.get_logger().info(
            f"Publishing /landing_pad/base_relative_position (class {self.base_class_id}) and "
            f"/landing_pad/h_relative_position (class {self.h_class_id}). "
            "Legacy /landing_pad/relative_position mirrors base topic."
        )

    def _detection_to_base_link(
        self,
        depth_m: np.ndarray,
        depth_header,
        u: int,
        v: int,
        fx: float,
        fy: float,
        cx_param: float,
        cy_param: float,
        source: str,
    ):
        """Convert a pixel detection at (u, v) to a PointStamped in target_frame.

        Returns a PointStamped or None on failure.
        """
        h_img, w_img = depth_m.shape[:2]
        u = int(np.clip(u, 0, w_img - 1))
        v = int(np.clip(v, 0, h_img - 1))

        Z = robust_depth_m(depth_m, u, v, win=7)
        if not np.isfinite(Z) or Z <= 0.0:
            return None

        # Camera optical frame: X right, Y down, Z forward
        Xc = (u - cx_param) * Z / fx
        Yc = (v - cy_param) * Z / fy
        Zc = Z

        p_cam = PointStamped()
        p_cam.header = depth_header
        p_cam.point = Point(x=float(Xc), y=float(Yc), z=float(Zc))

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                p_cam.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            p_base = do_transform_point(p_cam, tf)
        except Exception as e:
            self.get_logger().warn(
                f"[{source}] TF failed {p_cam.header.frame_id} -> {self.target_frame}: {e}"
            )
            return None

        # base_link (usual): x forward, y left, z up
        front = p_base.point.x
        right = -p_base.point.y

        out = PointStamped()
        out.header = p_base.header
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.target_frame
        out.point.x = float(right)     # x = right (drone body convention)
        out.point.y = float(front)     # y = front
        out.point.z = float(self.fixed_z)
        return out, Z

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

        # Collect best detection per class id
        best_by_class: dict = {}
        for b in res.boxes:
            cls = int(b.cls.item())
            conf_val = float(b.conf.item())
            x1, y1, x2, y2 = map(float, b.xyxy[0].tolist())
            if cls not in best_by_class or conf_val > best_by_class[cls][0]:
                best_by_class[cls] = (conf_val, x1, y1, x2, y2)

        # Process base detection (class base_class_id)
        if self.base_class_id in best_by_class:
            conf_val, x1, y1, x2, y2 = best_by_class[self.base_class_id]
            u = int(round((x1 + x2) * 0.5))
            v = int(round((y1 + y2) * 0.5))
            result = self._detection_to_base_link(
                depth_m, depth_msg.header, u, v, fx, fy, cx, cy, source
            )
            if result is not None:
                out_base, Z = result
                self.pub_base.publish(out_base)
                self.pub.publish(out_base)   # backward-compatible alias
                if source == "front":
                    self.pub_front.publish(out_base)
                elif source == "down":
                    self.pub_down.publish(out_base)
                self.get_logger().info(
                    f"[{source}] BASE conf={conf_val:.2f} "
                    f"right(x)={out_base.point.x:.2f}m "
                    f"front(y)={out_base.point.y:.2f}m depthZ={Z:.2f}m"
                )

        # Process H detection (class h_class_id)
        if self.h_class_id in best_by_class:
            conf_val, x1, y1, x2, y2 = best_by_class[self.h_class_id]
            u = int(round((x1 + x2) * 0.5))
            v = int(round((y1 + y2) * 0.5))
            result = self._detection_to_base_link(
                depth_m, depth_msg.header, u, v, fx, fy, cx, cy, source
            )
            if result is not None:
                out_h, Z = result
                self.pub_h.publish(out_h)
                self.get_logger().info(
                    f"[{source}] H conf={conf_val:.2f} "
                    f"right(x)={out_h.point.x:.2f}m "
                    f"front(y)={out_h.point.y:.2f}m depthZ={Z:.2f}m"
                )


def main():
    rclpy.init()
    node = YoloPadPose()
    node.get_logger().info("yolo_pad_pose node started.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()