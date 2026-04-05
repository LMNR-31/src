import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, PoseArray, Pose
from std_msgs.msg import Bool

import tf2_ros
from tf2_geometry_msgs import do_transform_point


class PadWaypointSupervisor(Node):
    """Supervisor that uses YOLO pad detections to command the drone to visit
    all landing bases (up to MAX_BASES) without repeating, then return home."""

    MAX_BASES = 6
    MIN_SEP = 0.6    # metres in map frame — closer than this = same base
    FIXED_Z = 1.5    # fixed flight altitude
    COOLDOWN = 2.0   # seconds between accepted detections

    def __init__(self):
        super().__init__("pad_waypoint_supervisor")

        # TF
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=10.0)
        )
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.visited_bases = []   # list of (x, y) in map frame
        self.waiting_for_finish: bool = False
        self.all_bases_done: bool = False
        self._home_published: bool = False
        self._last_detection_time: float = 0.0

        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribers
        self.create_subscription(
            PointStamped,
            "/landing_pad/relative_position",
            self._pad_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/trajectory_finished",
            self._finished_callback,
            reliable_qos,
        )

        # Publisher
        self.waypoints_pub = self.create_publisher(
            PoseArray,
            "/waypoints",
            reliable_qos,
        )

        self.get_logger().info(
            "PadWaypointSupervisor started. "
            "Listening on /landing_pad/relative_position and /trajectory_finished."
        )

    # ------------------------------------------------------------------
    # /trajectory_finished callback
    # ------------------------------------------------------------------

    def _finished_callback(self, msg: Bool) -> None:
        if not msg.data:
            return

        self.get_logger().info(
            "[FINISH] /trajectory_finished=true — ready for next command."
        )
        self.waiting_for_finish = False

        if self.all_bases_done and not self._home_published:
            self.get_logger().info(
                f"[HOME] All {self.MAX_BASES} bases visited. "
                "Publishing return-to-origin trajectory."
            )
            self._publish_return_home()
            self._home_published = True

    # ------------------------------------------------------------------
    # /landing_pad/relative_position callback
    # ------------------------------------------------------------------

    def _pad_callback(self, msg: PointStamped) -> None:
        if self.all_bases_done:
            return

        if self.waiting_for_finish:
            self.get_logger().debug(
                "[WAIT] Waiting for trajectory to finish — skipping detection."
            )
            return

        # Detection-rate cooldown
        now = time.monotonic()
        if now - self._last_detection_time < self.COOLDOWN:
            return

        # ------------------------------------------------------------------
        # Undo the x=right / y=front encoding and recover real base_link coords.
        # In uav1/base_link: x=forward, y=left, z=up.
        # The publisher stores:  msg.x = right = −base_link.y
        #                        msg.y = front =  base_link.x
        # ------------------------------------------------------------------
        corrected = PointStamped()
        corrected.header = msg.header          # frame_id = uav1/base_link
        corrected.point.x = msg.point.y        # front  → base_link x (forward)
        corrected.point.y = -msg.point.x       # −right → base_link y (left)
        corrected.point.z = 0.0                # will be overridden with FIXED_Z

        # Transform to map frame
        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                corrected.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            p_map = do_transform_point(corrected, tf)
        except Exception as exc:
            self.get_logger().warn(
                f"[TF] Transform to map failed: {exc}"
            )
            return

        pad_x = p_map.point.x
        pad_y = p_map.point.y

        # Check whether this detection belongs to an already-visited base
        for vx, vy in self.visited_bases:
            dist = math.sqrt((pad_x - vx) ** 2 + (pad_y - vy) ** 2)
            if dist < self.MIN_SEP:
                self.get_logger().info(
                    f"[SKIP] Detection ({pad_x:.2f}, {pad_y:.2f}) m is near "
                    f"visited base ({vx:.2f}, {vy:.2f}) m — dist={dist:.2f} m."
                )
                return

        # Get current drone position in map
        try:
            drone_tf = self.tf_buffer.lookup_transform(
                "map",
                "uav1/base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"[TF] Could not get drone pose in map: {exc}"
            )
            return

        drone_x = drone_tf.transform.translation.x
        drone_y = drone_tf.transform.translation.y

        # Accept this detection
        self._last_detection_time = now
        n = len(self.visited_bases) + 1

        self.get_logger().info(
            f"[SELECT] Base #{n}/{self.MAX_BASES} at map "
            f"({pad_x:.2f}, {pad_y:.2f}) m. Publishing trajectory."
        )

        self.visited_bases.append((pad_x, pad_y))
        self._publish_trajectory(
            drone_x, drone_y, self.FIXED_Z,
            pad_x,   pad_y,   self.FIXED_Z,
        )
        self.waiting_for_finish = True

        if len(self.visited_bases) >= self.MAX_BASES:
            self.get_logger().info(
                f"[DONE] {self.MAX_BASES} bases selected. "
                "Waiting for last trajectory to finish before returning home."
            )
            self.all_bases_done = True

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _make_pose(self, x: float, y: float, z: float) -> Pose:
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        p.orientation.w = 1.0
        return p

    def _publish_trajectory(
        self,
        x0: float, y0: float, z0: float,
        x1: float, y1: float, z1: float,
    ) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.poses = [
            self._make_pose(x0, y0, z0),
            self._make_pose(x1, y1, z1),
        ]
        self.waypoints_pub.publish(msg)
        self.get_logger().info(
            f"[TRAJ] Published: "
            f"WP0=({x0:.2f}, {y0:.2f}, {z0:.2f}) → "
            f"WP1=({x1:.2f}, {y1:.2f}, {z1:.2f})"
        )

    def _publish_return_home(self) -> None:
        try:
            drone_tf = self.tf_buffer.lookup_transform(
                "map",
                "uav1/base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            drone_x = drone_tf.transform.translation.x
            drone_y = drone_tf.transform.translation.y
        except Exception as exc:
            self.get_logger().warn(
                f"[TF] Could not get drone pose for return home: {exc}. "
                "Using (0.0, 0.0) as WP0."
            )
            drone_x, drone_y = 0.0, 0.0

        self.get_logger().info(
            f"[HOME] Return trajectory: "
            f"({drone_x:.2f}, {drone_y:.2f}) → (0.0, 0.0) at z={self.FIXED_Z}"
        )
        self._publish_trajectory(
            drone_x, drone_y, self.FIXED_Z,
            0.0,     0.0,     self.FIXED_Z,
        )


def main():
    rclpy.init()
    node = PadWaypointSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
