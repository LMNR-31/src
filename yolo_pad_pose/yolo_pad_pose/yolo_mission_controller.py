"""
yolo_mission_controller.py
==========================
ROS 2 mission controller that integrates the trained YOLO landing-pad detector
with the MRS drone simulation.

Overview
--------
This node subscribes to the detection stream published by ``yolo_pad_pose``
(/landing_pad/relative_position) and to the drone's odometry topic.  It builds
a memory of unique landing-pad positions (in the global/map frame) by converting
each relative detection with the current drone yaw — **without** relying on TF2.

A simple state machine then commands the drone to visit up to MAX_BASES landing
pads (default 6) in order of proximity, publishing a two-point PoseArray
trajectory on /waypoints after each transition, and finally returns the drone to
the origin.

Coordinate conventions
-----------------------
* /landing_pad/relative_position  (geometry_msgs/PointStamped)
    x = right of drone  (body +right)
    y = front of drone  (body +forward)
    z = fixed altitude  (ignored here; we use FIXED_Z)

* /uav1/mavros/local_position/odom  (nav_msgs/Odometry)
    Pose in the robot's local frame (ENU or NED depending on MAVROS config).
    Yaw is extracted from the orientation quaternion using the same formula as
    drone_controller_completo.cpp so the transforms are consistent.

* /waypoints  (geometry_msgs/PoseArray)
    frame_id : derived from the first odom message header (e.g. ``map`` or
               ``uav1/map``).  Falls back to ``map`` if odom is not received
               before the first publish.
    poses[0] : current drone position (WP0)
    poses[1] : target pad position    (WP1)
    Both points use FIXED_Z = 1.5 m.

State machine
-------------
BUILD_BASE_LIST
    Accumulate pad detections.  Each accepted detection is clustered into an
    existing candidate or added as a new candidate (if distance > CLUSTER_TOL).
    Periodically (every BUILD_PERIOD s) check whether there is an unvisited
    candidate close enough (< MAX_APPROACH_DIST) and, if so, transition to
    NAVIGATING by publishing a trajectory.

NAVIGATING
    Wait for /trajectory_finished = True.  No new detections are consumed in
    this state (the drone is flying).  On the finished signal, increment the
    visited counter and return to BUILD_BASE_LIST (or RETURN_HOME if 6 done).

RETURN_HOME
    Publish a single two-point trajectory from the current position to the map
    origin (x=0, y=0, z=FIXED_Z), then wait for trajectory_finished.

Parameters (ROS 2 node parameters)
------------------------------------
max_bases          int   6        Total number of landing pads to visit.
fixed_z            float 1.5      Fixed flight altitude [m].
cluster_tol        float 1.0      Radius within which detections belong to the
                                  same candidate base [m].
min_sep            float 0.8      Minimum separation between accepted distinct
                                  bases [m]. Must be >= cluster_tol.
max_approach_dist  float 30.0     Maximum distance at which a candidate will be
                                  selected as next target [m].
build_period       float 0.5      Timer period [s] for the BUILD_BASE_LIST state.
detection_cooldown float 0.3      Minimum seconds between accepted detections.
odom_topic         str   /uav1/mavros/local_position/odom
pad_topic          str   /landing_pad/relative_position
finished_topic     str   /trajectory_finished
waypoints_topic    str   /waypoints

Required external nodes
-----------------------
* yolo_pad_pose     — publishes /landing_pad/relative_position
* drone_controller_completo — subscribes /waypoints, publishes /trajectory_finished
"""

import math
import enum
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool


# ---------------------------------------------------------------------------
# State enumeration
# ---------------------------------------------------------------------------

class State(enum.Enum):
    BUILD_BASE_LIST = "BUILD_BASE_LIST"
    NAVIGATING = "NAVIGATING"
    RETURN_HOME = "RETURN_HOME"
    DONE = "DONE"


# ---------------------------------------------------------------------------
# Helper: yaw from quaternion  (same formula as drone_controller_completo.cpp)
# ---------------------------------------------------------------------------

def _yaw_from_quat(q) -> float:
    """Extract yaw [rad] from a geometry_msgs/Quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class YoloMissionController(Node):
    """Navigate the drone to visit every detected landing pad, then return home."""

    def __init__(self):
        super().__init__("yolo_mission_controller")

        # ── declare parameters ──────────────────────────────────────────────
        self.declare_parameter("max_bases", 6)
        self.declare_parameter("fixed_z", 1.5)
        self.declare_parameter("cluster_tol", 1.0)
        self.declare_parameter("min_sep", 0.8)
        self.declare_parameter("max_approach_dist", 30.0)
        self.declare_parameter("build_period", 0.5)
        self.declare_parameter("detection_cooldown", 0.3)
        self.declare_parameter("odom_topic", "/uav1/mavros/local_position/odom")
        self.declare_parameter("pad_topic", "/landing_pad/relative_position")
        self.declare_parameter("finished_topic", "/trajectory_finished")
        self.declare_parameter("waypoints_topic", "/waypoints")

        self._max_bases: int = int(self.get_parameter("max_bases").value)
        self._fixed_z: float = float(self.get_parameter("fixed_z").value)
        self._cluster_tol: float = float(self.get_parameter("cluster_tol").value)
        self._min_sep: float = float(self.get_parameter("min_sep").value)
        self._max_approach_dist: float = float(
            self.get_parameter("max_approach_dist").value
        )
        self._build_period: float = float(self.get_parameter("build_period").value)
        self._detection_cooldown: float = float(
            self.get_parameter("detection_cooldown").value
        )

        # ── internal state ───────────────────────────────────────────────────
        # Odometry (updated on every odom message)
        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_yaw: float = 0.0
        self._odom_frame_id: str = "map"   # derived from first odom header
        self._odom_received: bool = False

        # Candidate bases in the global/map frame: list of [x, y, count]
        # 'count' is the number of detections merged into this candidate.
        self._candidates: list = []

        # Set of visited candidate indices (into self._candidates)
        self._visited_indices: set = set()

        # State machine
        self._state: State = State.BUILD_BASE_LIST
        self._visited_count: int = 0

        # Timestamps
        self._last_detection_time: float = 0.0

        # ── QoS ─────────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── subscribers ──────────────────────────────────────────────────────
        odom_topic = self.get_parameter("odom_topic").value
        pad_topic = self.get_parameter("pad_topic").value
        finished_topic = self.get_parameter("finished_topic").value

        self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            pad_topic,
            self._pad_callback,
            10,
        )
        self.create_subscription(
            Bool,
            finished_topic,
            self._finished_callback,
            reliable_qos,
        )

        # ── publisher ────────────────────────────────────────────────────────
        waypoints_topic = self.get_parameter("waypoints_topic").value
        self._waypoints_pub = self.create_publisher(
            PoseArray,
            waypoints_topic,
            reliable_qos,
        )

        # ── timer: periodic check in BUILD_BASE_LIST state ───────────────────
        self._build_timer = self.create_timer(
            self._build_period, self._build_tick
        )

        self.get_logger().info(
            f"[YoloMissionController] started — max_bases={self._max_bases}, "
            f"fixed_z={self._fixed_z} m, cluster_tol={self._cluster_tol} m, "
            f"min_sep={self._min_sep} m, odom={odom_topic}, pad={pad_topic}"
        )

    # ========================================================================
    # Callbacks
    # ========================================================================

    def _odom_callback(self, msg: Odometry) -> None:
        """Store current drone position and yaw (used for coordinate conversion)."""
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_yaw = _yaw_from_quat(msg.pose.pose.orientation)

        if not self._odom_received:
            # Derive the map frame id from the odometry header once.
            frame = msg.header.frame_id.strip()
            if frame:
                self._odom_frame_id = frame
                self.get_logger().info(
                    f"[ODOM] Map frame derived from odometry header: "
                    f"'{self._odom_frame_id}'"
                )
            self._odom_received = True

    def _pad_callback(self, msg: PointStamped) -> None:
        """Process a YOLO pad detection and update the candidate list."""
        if self._state != State.BUILD_BASE_LIST:
            return

        if not self._odom_received:
            self.get_logger().debug(
                "[PAD] Odometry not received yet — skipping detection."
            )
            return

        # Rate-limit: avoid flooding the candidate list with rapid duplicates.
        now = time.monotonic()
        if now - self._last_detection_time < self._detection_cooldown:
            return
        self._last_detection_time = now

        # ── convert relative (right, front) → global (map) ──────────────────
        # Detection convention: msg.x = right, msg.y = front
        # base_link: x = forward, y = left, z = up
        #   base_link.x = front = msg.y
        #   base_link.y = left  = -right = -msg.x
        #
        # Map frame rotation by yaw (angle of drone forward from map x-axis):
        #   dx_map = base_link.x * cos(yaw) - base_link.y * sin(yaw)
        #          = front * cos(yaw) + right * sin(yaw)
        #   dy_map = base_link.x * sin(yaw) + base_link.y * cos(yaw)
        #          = front * sin(yaw) - right * cos(yaw)
        front = float(msg.point.y)
        right = float(msg.point.x)
        yaw = self._odom_yaw

        dx = front * math.cos(yaw) + right * math.sin(yaw)
        dy = front * math.sin(yaw) - right * math.cos(yaw)

        gx = self._odom_x + dx
        gy = self._odom_y + dy

        self._update_candidates(gx, gy)

    def _finished_callback(self, msg: Bool) -> None:
        """Handle trajectory completion signal from drone_controller_completo."""
        if not msg.data:
            return

        self.get_logger().info(
            f"[FINISH] /trajectory_finished=true — state={self._state.value}"
        )

        if self._state == State.NAVIGATING:
            self._visited_count += 1
            self.get_logger().info(
                f"[VISITED] Base #{self._visited_count}/{self._max_bases} done."
            )

            if self._visited_count >= self._max_bases:
                self.get_logger().info(
                    f"[DONE] All {self._max_bases} bases visited. "
                    "Publishing return-to-home trajectory."
                )
                self._state = State.RETURN_HOME
                self._publish_return_home()
            else:
                # Return to detection loop for next base.
                self._state = State.BUILD_BASE_LIST
                self.get_logger().info(
                    "[STATE] → BUILD_BASE_LIST (collecting next base)"
                )

        elif self._state == State.RETURN_HOME:
            self.get_logger().info("[STATE] → DONE. Mission complete.")
            self._state = State.DONE

    # ========================================================================
    # Timer tick (BUILD_BASE_LIST state)
    # ========================================================================

    def _build_tick(self) -> None:
        """Periodic check: try to select and navigate to the nearest unvisited base."""
        if self._state != State.BUILD_BASE_LIST:
            return

        if not self._odom_received:
            return

        target = self._pick_nearest_unvisited()
        if target is None:
            return

        idx, cx, cy = target
        self._visited_indices.add(idx)

        dist = math.hypot(cx - self._odom_x, cy - self._odom_y)
        self.get_logger().info(
            f"[SELECT] Heading to candidate #{idx} at map "
            f"({cx:.2f}, {cy:.2f}) m — dist={dist:.2f} m "
            f"[{self._visited_count + 1}/{self._max_bases}]"
        )

        self._publish_trajectory(
            self._odom_x, self._odom_y, self._fixed_z,
            cx, cy, self._fixed_z,
        )
        self._state = State.NAVIGATING
        self.get_logger().info("[STATE] → NAVIGATING")

    # ========================================================================
    # Candidate management
    # ========================================================================

    def _update_candidates(self, gx: float, gy: float) -> None:
        """Merge a new detection into the nearest existing candidate or create one."""
        best_idx = None
        best_dist = float("inf")

        for i, (cx, cy, _count) in enumerate(self._candidates):
            d = math.hypot(gx - cx, gy - cy)
            if d < best_dist:
                best_dist = d
                best_idx = i

        if best_idx is not None and best_dist < self._cluster_tol:
            # Merge: update candidate position as running mean.
            cx, cy, count = self._candidates[best_idx]
            new_count = count + 1
            self._candidates[best_idx] = [
                (cx * count + gx) / new_count,
                (cy * count + gy) / new_count,
                new_count,
            ]
            self.get_logger().debug(
                f"[CLUSTER] Merged into candidate #{best_idx} "
                f"({self._candidates[best_idx][0]:.2f}, "
                f"{self._candidates[best_idx][1]:.2f}) — "
                f"n={new_count}"
            )
        else:
            # Check minimum separation from all existing candidates to avoid
            # registering closely-spaced duplicates.
            for cx, cy, _count in self._candidates:
                if math.hypot(gx - cx, gy - cy) < self._min_sep:
                    self.get_logger().debug(
                        f"[CLUSTER] Dropped ({gx:.2f}, {gy:.2f}): "
                        "too close to existing candidate."
                    )
                    return
            # Add new candidate.
            self._candidates.append([gx, gy, 1])
            self.get_logger().info(
                f"[CLUSTER] New candidate #{len(self._candidates) - 1} "
                f"at ({gx:.2f}, {gy:.2f}) m — total candidates: "
                f"{len(self._candidates)}"
            )

    def _pick_nearest_unvisited(self):
        """Return (index, x, y) of nearest unvisited candidate, or None."""
        best = None
        best_dist = float("inf")

        for i, (cx, cy, count) in enumerate(self._candidates):
            if i in self._visited_indices:
                continue
            dist = math.hypot(cx - self._odom_x, cy - self._odom_y)
            if dist < best_dist and dist <= self._max_approach_dist:
                best_dist = dist
                best = (i, cx, cy)

        return best

    # ========================================================================
    # Publishing helpers
    # ========================================================================

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
        """Publish a 2-point PoseArray trajectory."""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame_id
        msg.poses = [
            self._make_pose(x0, y0, z0),
            self._make_pose(x1, y1, z1),
        ]
        self._waypoints_pub.publish(msg)
        self.get_logger().info(
            f"[TRAJ] frame={self._odom_frame_id} "
            f"WP0=({x0:.2f}, {y0:.2f}, {z0:.2f}) → "
            f"WP1=({x1:.2f}, {y1:.2f}, {z1:.2f})"
        )

    def _publish_return_home(self) -> None:
        """Publish a return-to-origin (x=0, y=0) trajectory."""
        self.get_logger().info(
            f"[HOME] Return: ({self._odom_x:.2f}, {self._odom_y:.2f}) → "
            f"(0.0, 0.0) at z={self._fixed_z}"
        )
        self._publish_trajectory(
            self._odom_x, self._odom_y, self._fixed_z,
            0.0, 0.0, self._fixed_z,
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = YoloMissionController()
    node.get_logger().info("yolo_mission_controller node started.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
