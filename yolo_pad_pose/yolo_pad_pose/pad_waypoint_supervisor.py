#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32
from drone_control.msg import Waypoint4D, Waypoint4DArray


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    # yaw (Z) from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class BaseCandidate:
    x: float
    y: float
    last_seen_s: float
    seen_count: int
    visited: bool = False


class PadWaypointSupervisor(Node):
    """
    Subscribes to YOLO detections from front+down cameras (optical frame points),
    converts to right/front, projects to map using odom yaw, deduplicates bases,
    and publishes /waypoints_4d (Waypoint4DArray) with 2 waypoints: current + target
    (z fixed). The target waypoint encodes an explicit yaw pointing toward the
    detected H pad center so the drone faces the base on arrival.
    """

    def __init__(self):
        super().__init__("pad_waypoint_supervisor")

        # Parameters
        self.declare_parameter("odom_topic", "/uav1/mavros/local_position/odom")
        self.declare_parameter("front_det_topic", "/landing_pad/front_optical_point")
        self.declare_parameter("down_det_topic", "/landing_pad/down_optical_point")
        self.declare_parameter("waypoints_topic", "/waypoints_4d")

        self.declare_parameter("trajectory_finished_topic", "/trajectory_finished")
        self.declare_parameter("use_trajectory_finished", True)
        self.declare_parameter("mission_cycle_done_topic", "/mission_cycle_done")
        self.declare_parameter("yaw_scan_done_topic", "/yaw_scan_done")
        self.declare_parameter("scan_duration_s", 35.0)   # fallback: auto-advance SCAN after this many seconds
        self.declare_parameter("controller_state_topic", "/drone_controller/state_voo")

        self.declare_parameter("map_frame", "uav1/map")     # used in Waypoint4DArray.header.frame_id
        self.declare_parameter("z_fixed", 1.5)
        self.declare_parameter("bases_to_visit", 6)

        self.declare_parameter("cluster_tol_m", 0.7)        # merge detections within this radius
        self.declare_parameter("min_seen_count", 3)         # require stable detection before accepting a base
        self.declare_parameter("candidate_timeout_s", 5.0)  # drop candidates not seen recently

        self.declare_parameter("publish_period_s", 0.25)    # how often to republish target while navigating
        self.declare_parameter("reach_tol_m", 0.6)           # consider reached when within this XY distance
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("use_home_xy", True)          # forwarded to pouso: use initial odom XY as home
        self.declare_parameter("enable_auto_land", False)    # forwarded to pouso: call AUTO.LAND mode
        self.declare_parameter("xy_hold_tol", 0.05)          # forwarded to pouso: XY tolerance for landing (m)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.front_det_topic = self.get_parameter("front_det_topic").value
        self.down_det_topic = self.get_parameter("down_det_topic").value
        self.waypoints_topic = self.get_parameter("waypoints_topic").value

        self.trajectory_finished_topic = self.get_parameter("trajectory_finished_topic").value
        self.use_trajectory_finished = bool(self.get_parameter("use_trajectory_finished").value)
        self.mission_cycle_done_topic = self.get_parameter("mission_cycle_done_topic").value
        self.yaw_scan_done_topic = self.get_parameter("yaw_scan_done_topic").value
        self.scan_duration_s = float(self.get_parameter("scan_duration_s").value)
        self.controller_state_topic = self.get_parameter("controller_state_topic").value

        self.map_frame = self.get_parameter("map_frame").value
        self.z_fixed = float(self.get_parameter("z_fixed").value)
        self.bases_to_visit = int(self.get_parameter("bases_to_visit").value)

        self.cluster_tol_m = float(self.get_parameter("cluster_tol_m").value)
        self.min_seen_count = int(self.get_parameter("min_seen_count").value)
        self.candidate_timeout_s = float(self.get_parameter("candidate_timeout_s").value)

        self.publish_period_s = float(self.get_parameter("publish_period_s").value)
        self.reach_tol_m = float(self.get_parameter("reach_tol_m").value)
        self.home_x = float(self.get_parameter("home_x").value)
        self.home_y = float(self.get_parameter("home_y").value)
        self.use_home_xy = bool(self.get_parameter("use_home_xy").value)
        self.enable_auto_land = bool(self.get_parameter("enable_auto_land").value)
        self.xy_hold_tol = float(self.get_parameter("xy_hold_tol").value)

        # State
        self.have_odom = False
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0
        # Home (H pad) coordinates: latched from first odom when use_home_xy=True.
        self._home_latched = False

        self.trajectory_finished = False
        self._last_finished_msg_t = 0.0
        self.mission_cycle_done = False
        self.yaw_scan_done = False
        self._scan_start_t = time.time()
        self.ready_for_commands = (not self.use_trajectory_finished)
        if self.ready_for_commands:
            self.get_logger().info("[READY] use_trajectory_finished=False, enabling waypoint publishing immediately.")
        else:
            self.get_logger().info("[WAIT] use_trajectory_finished=True, waiting for /trajectory_finished to enable waypoint publishing.")
        self.state_voo = None           # controller state from /drone_controller/state_voo
        self._last_state_warn_t = 0.0   # for throttled warning when state not in (2,3)
        self._last_zero_stamp_warn_t = 0.0  # for throttled warning on zero-stamp messages

        self.candidates: List[BaseCandidate] = []
        self.active_target: Optional[Tuple[float, float]] = None
        self.visited_count = 0
        self.state = "SCAN"   # SCAN -> DISCOVER -> NAVIGATE -> WAIT_MISSION_DONE -> ... -> RETURN_HOME -> DONE

        # ROS I/O
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 10)
        self.sub_front = self.create_subscription(PointStamped, self.front_det_topic, lambda m: self.cb_det(m, "front"), 10)
        self.sub_down = self.create_subscription(PointStamped, self.down_det_topic, lambda m: self.cb_det(m, "down"), 10)

        if self.use_trajectory_finished:
            self.sub_finished = self.create_subscription(Bool, self.trajectory_finished_topic, self.cb_finished, 10)
        else:
            self.sub_finished = None

        self.sub_mission_cycle_done = self.create_subscription(
            Bool, self.mission_cycle_done_topic, self.cb_mission_cycle_done, 10)

        self.sub_yaw_scan_done = self.create_subscription(
            Bool, self.yaw_scan_done_topic, self.cb_yaw_scan_done, 10)

        self.sub_state_voo = self.create_subscription(
            Int32, self.controller_state_topic, self.cb_state_voo,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            ),
        )

        self.pub_waypoints = self.create_publisher(Waypoint4DArray, self.waypoints_topic, 10)

        self.timer = self.create_timer(self.publish_period_s, self.tick)

        self.get_logger().info(
            "pad_waypoint_supervisor started.\n"
            f" odom_topic={self.odom_topic}\n"
            f" front_det_topic={self.front_det_topic}\n"
            f" down_det_topic={self.down_det_topic}\n"
            f" waypoints_topic={self.waypoints_topic}\n"
            f" controller_state_topic={self.controller_state_topic}\n"
            f" mission_cycle_done_topic={self.mission_cycle_done_topic}\n"
            f" map_frame={self.map_frame} z_fixed={self.z_fixed}\n"
            f" bases_to_visit={self.bases_to_visit}\n"
            f" cluster_tol_m={self.cluster_tol_m} min_seen_count={self.min_seen_count}\n"
            f" use_home_xy={self.use_home_xy} enable_auto_land={self.enable_auto_land}"
            f" xy_hold_tol={self.xy_hold_tol}"
        )

    def cb_finished(self, msg: Bool):
        self.trajectory_finished = bool(msg.data)
        self._last_finished_msg_t = time.time()
        if msg.data and not self.ready_for_commands:
            self.ready_for_commands = True
            self.get_logger().info("[READY] Controller signaled /trajectory_finished. Enabling waypoint publishing.")

    def cb_mission_cycle_done(self, msg: Bool):
        if msg.data and self.state == "WAIT_MISSION_DONE":
            self.mission_cycle_done = True
            self.get_logger().info("[HANDSHAKE] /mission_cycle_done=true received — mission cycle complete.")
        elif msg.data:
            self.get_logger().debug(
                f"[HANDSHAKE] /mission_cycle_done=true received but ignored (state={self.state})."
            )

    def cb_yaw_scan_done(self, msg: Bool):
        if msg.data and not self.yaw_scan_done:
            self.yaw_scan_done = True
            self.ready_for_commands = True
            confirmed = [c for c in self.candidates if c.seen_count >= self.min_seen_count]
            self.get_logger().info(
                f"[SCAN] /yaw_scan_done=true — 360° scan complete. "
                f"{len(confirmed)} confirmed base(s) discovered (of {len(self.candidates)} candidate(s)):"
            )
            for i, c in enumerate(confirmed):
                self.get_logger().info(
                    f"[SCAN]   base #{i}: map ({c.x:.2f}, {c.y:.2f}) m  seen={c.seen_count}"
                )

    def cb_state_voo(self, msg: Int32):
        self.state_voo = int(msg.data)

    def cb_odom(self, msg: Odometry):
        self.have_odom = True
        self.cur_x = float(msg.pose.pose.position.x)
        self.cur_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.cur_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        # Latch first odom position as home (H pad) when use_home_xy is enabled.
        if self.use_home_xy and not self._home_latched:
            self.home_x = self.cur_x
            self.home_y = self.cur_y
            self._home_latched = True
            self.get_logger().info(
                f"[HOME] use_home_xy=True — home (H) latched from first odom: "
                f"x={self.home_x:.3f} y={self.home_y:.3f}"
            )

        # Update map frame from odom if it looks namespaced (optional)
        if msg.header.frame_id:
            self.map_frame = msg.header.frame_id

    def cb_det(self, msg: PointStamped, source: str):
        # Need odom to project to map
        if not self.have_odom:
            return

        # Do not accumulate candidates until the controller has signaled readiness,
        # except during the initial SCAN phase when we intentionally collect data.
        if not self.ready_for_commands and self.state != "SCAN":
            return

        # Gate on controller state: allow HOVER (2) and NAVIGATING (3).
        # During the initial SCAN phase the drone is rotating in place (HOVER),
        # so we skip the state gate entirely to ensure detections are captured.
        if self.state not in ("SCAN",) and self.state_voo not in (2, 3):
            now = time.time()
            if now - self._last_state_warn_t >= 2.0:
                self._last_state_warn_t = now
                self.get_logger().warning(
                    f"[{source}] Detection received but controller state={self.state_voo}"
                    " (need 2=HOVER or 3=NAVIGATING). Ignoring."
                )
            return

        # msg.point is in optical frame. Our detector currently publishes:
        #  - x ~ X_optical (right)
        #  - y ~ Y_optical (down)   [not used]
        #  - z ~ Z_optical (forward)

        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            now_t = time.time()
            if now_t - self._last_zero_stamp_warn_t >= 2.0:
                self._last_zero_stamp_warn_t = now_t
                self.get_logger().warning(
                    f"[{source}] Received PointStamped with zero stamp; using current time as fallback."
                )

        right = float(msg.point.x)
        front = float(msg.point.y)

        # Project to map (2D)
        yaw = self.cur_yaw
        dx_map = front * math.cos(yaw) + right * math.sin(yaw)
        dy_map = front * math.sin(yaw) - right * math.cos(yaw)

        bx = self.cur_x + dx_map
        by = self.cur_y + dy_map

        self._update_candidates(bx, by)

        now = time.time()
        if not hasattr(self, "_last_det_info_t"):
            self._last_det_info_t = 0.0
        if now - self._last_det_info_t >= 1.0:
            self._last_det_info_t = now
            max_seen = max((c.seen_count for c in self.candidates), default=0)
            self.get_logger().info(
                f"[{source}] det ok: bx={bx:.2f} by={by:.2f} "
                f"candidates={len(self.candidates)} max_seen={max_seen} "
                f"state={self.state} state_voo={self.state_voo}"
            )

    def _update_candidates(self, bx: float, by: float):
        now = time.time()

        # Drop stale candidates — but NOT during the initial scan so that bases
        # seen early in the 360° rotation aren't pruned before the scan ends.
        if self.state != "SCAN":
            self.candidates = [
                c for c in self.candidates
                if (now - c.last_seen_s) <= self.candidate_timeout_s
            ]

        # Merge into existing if close
        for c in self.candidates:
            if math.hypot(c.x - bx, c.y - by) <= self.cluster_tol_m:
                # exponential moving average update
                alpha = 0.3
                c.x = (1 - alpha) * c.x + alpha * bx
                c.y = (1 - alpha) * c.y + alpha * by
                c.last_seen_s = now
                c.seen_count += 1
                return

        # New candidate
        self.candidates.append(BaseCandidate(x=bx, y=by, last_seen_s=now, seen_count=1))

    def _pick_next_target(self) -> Optional[Tuple[float, float]]:
        # Choose nearest unvisited candidate with enough evidence
        best = None
        best_d = None
        for c in self.candidates:
            if c.visited:
                continue
            if c.seen_count < self.min_seen_count:
                continue
            d = math.hypot(c.x - self.cur_x, c.y - self.cur_y)
            if best is None or d < best_d:
                best = c
                best_d = d
        if best is None:
            return None
        return (best.x, best.y)

    def _mark_visited(self, x: float, y: float):
        for c in self.candidates:
            if math.hypot(c.x - x, c.y - y) <= self.cluster_tol_m:
                c.visited = True

    def _publish_two_pose_waypoints(self, tx: float, ty: float):
        # Yaw at the target: point the drone toward the detected H center (base).
        yaw_to_target = math.atan2(ty - self.cur_y, tx - self.cur_x)

        msg = Waypoint4DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        wp0 = Waypoint4D()
        wp0.pose.position.x = float(self.cur_x)
        wp0.pose.position.y = float(self.cur_y)
        wp0.pose.position.z = float(self.z_fixed)
        wp0.yaw = float("nan")  # keep current heading for the first (origin) wp

        wp1 = Waypoint4D()
        wp1.pose.position.x = float(tx)
        wp1.pose.position.y = float(ty)
        wp1.pose.position.z = float(self.z_fixed)
        wp1.yaw = float(yaw_to_target)  # face the H pad center on arrival

        msg.waypoints = [wp0, wp1]
        self.pub_waypoints.publish(msg)

    def tick(self):
        if not self.have_odom:
            return

        # State machine
        if self.state == "SCAN":
            # Collect detections passively while supervisor_T runs the yaw scan.
            # Advance when /yaw_scan_done is received OR the fallback timer expires.
            elapsed = time.time() - self._scan_start_t
            scan_timed_out = self.scan_duration_s > 0 and elapsed >= self.scan_duration_s
            if self.yaw_scan_done or scan_timed_out:
                if scan_timed_out and not self.yaw_scan_done:
                    self.get_logger().warning(
                        f"[SCAN] Timeout ({elapsed:.1f}s) — /yaw_scan_done not received. "
                        "Advancing to DISCOVER anyway.")
                    self.ready_for_commands = True
                confirmed = [c for c in self.candidates if c.seen_count >= self.min_seen_count]
                self.get_logger().info(
                    f"[SCAN→DISCOVER] {len(confirmed)} confirmed base(s) found. "
                    "Starting mission sequencing.")
                self.state = "DISCOVER"
            return

        if self.state == "DISCOVER":
            if self.visited_count >= self.bases_to_visit:
                self.get_logger().info("All bases visited. Switching to RETURN_HOME.")
                self.state = "RETURN_HOME"
                self.active_target = (self.home_x, self.home_y)
                return

            nxt = self._pick_next_target()
            if nxt is None:
                # Still discovering bases
                return

            self.active_target = nxt
            self.get_logger().info(f"Target selected: x={nxt[0]:.2f} y={nxt[1]:.2f} (visited={self.visited_count}/{self.bases_to_visit})")
            self.state = "NAVIGATE"

        if self.state == "NAVIGATE":
            if self.active_target is None:
                self.state = "DISCOVER"
                return

            tx, ty = self.active_target
            self._publish_two_pose_waypoints(tx, ty)

            d = math.hypot(tx - self.cur_x, ty - self.cur_y)
            if d <= self.reach_tol_m:
                self.get_logger().info(
                    f"[STATE] Reached base within {self.reach_tol_m}m (d={d:.2f}). "
                    "Waiting for mission cycle to complete (WAIT_MISSION_DONE).")
                self.state = "WAIT_MISSION_DONE"
                # Reset handshake flag to discard any stale signal that arrived
                # before we entered this state (e.g. from the previous cycle).
                self.mission_cycle_done = False
                return

        if self.state == "WAIT_MISSION_DONE":
            # Hold position — do not publish new navigation targets.
            # Wait until supervisor_T signals that missao_P_T has finished.
            if not self.mission_cycle_done:
                return

            # Mission cycle done: mark this base as visited and select next.
            tx, ty = self.active_target if self.active_target else (None, None)
            if tx is not None:
                self._mark_visited(tx, ty)

            self.visited_count += 1
            # Reset for next cycle (the pre-state reset at entry guarded against
            # stale signals; this reset ensures a clean state going forward).
            self.mission_cycle_done = False
            self.get_logger().info(
                f"[STATE] Mission cycle done — base marked visited. "
                f"visited={self.visited_count}/{self.bases_to_visit}. "
                "Transitioning to DISCOVER.")
            self.active_target = None
            self.state = "DISCOVER"

        if self.state == "RETURN_HOME":
            # publish until reached home, then DONE
            tx, ty = (self.home_x, self.home_y)
            self._publish_two_pose_waypoints(tx, ty)
            d = math.hypot(tx - self.cur_x, ty - self.cur_y)
            if d <= self.reach_tol_m:
                self.get_logger().info("Returned home. DONE.")
                self.state = "DONE"

        if self.state == "DONE":
            return


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


if __name__ == "__main__":
    main()
