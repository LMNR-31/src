#!/usr/bin/env python3
import math
import subprocess
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32


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
    and publishes /waypoints (PoseArray) with 2 poses: current + target (z fixed).
    """

    def __init__(self):
        super().__init__("pad_waypoint_supervisor")

        # Parameters
        self.declare_parameter("odom_topic", "/uav1/mavros/local_position/odom")
        self.declare_parameter("base_det_topic", "/landing_pad/base_relative_position")
        # Legacy per-camera optical-frame topics; set to "" to disable
        self.declare_parameter("front_det_topic", "")
        self.declare_parameter("down_det_topic", "")
        self.declare_parameter("waypoints_topic", "/waypoints")

        self.declare_parameter("trajectory_finished_topic", "/trajectory_finished")
        self.declare_parameter("use_trajectory_finished", True)
        self.declare_parameter("mission_cycle_done_topic", "/mission_cycle_done")
        self.declare_parameter("yaw_scan_done_topic", "/yaw_scan_done")
        self.declare_parameter("scan_duration_s", 35.0)   # fallback: auto-advance SCAN after this many seconds
        self.declare_parameter("controller_state_topic", "/drone_controller/state_voo")

        self.declare_parameter("world_frame_id", "map")  # output PoseArray header frame_id; treated numerically as odom
        self.declare_parameter("z_fixed", 1.5)
        self.declare_parameter("bases_to_visit", 6)

        self.declare_parameter("cluster_tol_m", 1.2)        # merge detections within this radius
        self.declare_parameter("min_seen_count", 6)         # require stable detection before accepting a base
        self.declare_parameter("candidate_timeout_s", 30.0) # drop candidates not seen recently

        self.declare_parameter("publish_period_s", 0.25)    # how often to republish target while navigating
        self.declare_parameter("reach_tol_m", 0.6)           # consider reached when within this XY distance
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("repeat_block_m", 1.2)       # skip targets within this radius of any visited position
        self.declare_parameter("aim_at_home", False)        # orient waypoints to face toward home
        self.declare_parameter("aim_at_h", True)            # orient waypoints to face toward detected H marker
        self.declare_parameter("h_det_topic", "/landing_pad/h_relative_position")
        self.declare_parameter("h_timeout_s", 3.0)          # discard H position if older than this
        self.declare_parameter("max_h_base_separation_m", 0.5)  # discard H if H_rel vs base_rel distance exceeds this

        # Axis correction: flip the incoming right (x) axis to fix systematic lateral bias
        self.declare_parameter("invert_right_axis", True)
        # Fine-tuning offsets applied in base_link before world projection
        self.declare_parameter("target_offset_right_m", 0.0)
        self.declare_parameter("target_offset_front_m", 0.0)

        # Outlier rejection
        self.declare_parameter("max_detection_range_m", 3.0)  # discard if farther than this
        self.declare_parameter("max_jump_m", 0.8)  # discard if jump from last accepted > this

        # "Lost" recovery: trigger yaw-360 scan if no valid candidate for this long
        self.declare_parameter("lost_timeout_s", 10.0)
        self.declare_parameter("yaw_scan_cooldown_s", 30.0)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.base_det_topic = self.get_parameter("base_det_topic").value
        self.front_det_topic = self.get_parameter("front_det_topic").value
        self.down_det_topic = self.get_parameter("down_det_topic").value
        self.waypoints_topic = self.get_parameter("waypoints_topic").value

        self.trajectory_finished_topic = self.get_parameter("trajectory_finished_topic").value
        self.use_trajectory_finished = bool(self.get_parameter("use_trajectory_finished").value)
        self.mission_cycle_done_topic = self.get_parameter("mission_cycle_done_topic").value
        self.yaw_scan_done_topic = self.get_parameter("yaw_scan_done_topic").value
        self.scan_duration_s = float(self.get_parameter("scan_duration_s").value)
        self.controller_state_topic = self.get_parameter("controller_state_topic").value

        self.world_frame_id = self.get_parameter("world_frame_id").value
        self.z_fixed = float(self.get_parameter("z_fixed").value)
        self.bases_to_visit = int(self.get_parameter("bases_to_visit").value)

        self.cluster_tol_m = float(self.get_parameter("cluster_tol_m").value)
        self.min_seen_count = int(self.get_parameter("min_seen_count").value)
        self.candidate_timeout_s = float(self.get_parameter("candidate_timeout_s").value)

        self.publish_period_s = float(self.get_parameter("publish_period_s").value)
        self.reach_tol_m = float(self.get_parameter("reach_tol_m").value)
        self.home_x = float(self.get_parameter("home_x").value)
        self.home_y = float(self.get_parameter("home_y").value)
        self.repeat_block_m = float(self.get_parameter("repeat_block_m").value)
        self.aim_at_home = bool(self.get_parameter("aim_at_home").value)
        self.aim_at_h = bool(self.get_parameter("aim_at_h").value)
        self.h_det_topic = self.get_parameter("h_det_topic").value
        self.h_timeout_s = float(self.get_parameter("h_timeout_s").value)
        self.max_h_base_separation_m = float(self.get_parameter("max_h_base_separation_m").value)

        self.invert_right_axis = bool(self.get_parameter("invert_right_axis").value)
        self.target_offset_right_m = float(self.get_parameter("target_offset_right_m").value)
        self.target_offset_front_m = float(self.get_parameter("target_offset_front_m").value)
        self.max_detection_range_m = float(self.get_parameter("max_detection_range_m").value)
        self.max_jump_m = float(self.get_parameter("max_jump_m").value)
        self.lost_timeout_s = float(self.get_parameter("lost_timeout_s").value)
        self.yaw_scan_cooldown_s = float(self.get_parameter("yaw_scan_cooldown_s").value)

        # State
        self.have_odom = False
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0

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
        self.visited_positions: List[Tuple[float, float]] = []
        self.state = "SCAN"   # SCAN -> DISCOVER -> NAVIGATE -> WAIT_MISSION_DONE -> ... -> RETURN_HOME -> DONE

        # Home captured from the first odom reading (used for aim_at_home)
        self._have_home_from_odom = False
        self._home_from_odom_x = 0.0
        self._home_from_odom_y = 0.0

        # Latest H-marker position received in base_link frame (right, front) + timestamp
        self._h_right: float = 0.0
        self._h_front: float = 0.0
        self._h_last_t: float = 0.0   # monotonic time of last H detection

        # Latest base detection in base_link frame (right, front) used for H gating
        self._base_right: float = 0.0
        self._base_front: float = 0.0
        self._base_last_t: float = 0.0  # monotonic time of last base detection

        # Last accepted detection position (body frame, after inversion/offsets) for jump filtering
        self._have_last_accepted: bool = False
        self._last_accepted_right: float = 0.0
        self._last_accepted_front: float = 0.0

        # Lost-recovery state
        self._last_valid_candidate_t: float = time.time()
        self._yaw_scan_active: bool = False
        self._yaw_scan_proc: Optional[subprocess.Popen] = None  # type: ignore[type-arg]
        self._last_yaw_scan_t: float = 0.0

        # ROS I/O
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 10)
        self.sub_base = self.create_subscription(
            PointStamped, self.base_det_topic, lambda m: self.cb_det(m, "base"), 10)
        # Legacy per-camera optical-frame topics (only subscribed if topic is non-empty)
        if self.front_det_topic:
            self.sub_front = self.create_subscription(
                PointStamped, self.front_det_topic, lambda m: self.cb_det(m, "front"), 10)
        if self.down_det_topic:
            self.sub_down = self.create_subscription(
                PointStamped, self.down_det_topic, lambda m: self.cb_det(m, "down"), 10)
        self.sub_h_det = self.create_subscription(
            PointStamped, self.h_det_topic, self.cb_h_det, 10)

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

        self.pub_waypoints = self.create_publisher(PoseArray, self.waypoints_topic, 10)

        self.timer = self.create_timer(self.publish_period_s, self.tick)

        self.get_logger().info(
            "pad_waypoint_supervisor started.\n"
            f" odom_topic={self.odom_topic}\n"
            f" base_det_topic={self.base_det_topic}\n"
            f" front_det_topic={self.front_det_topic or '(disabled)'}\n"
            f" down_det_topic={self.down_det_topic or '(disabled)'}\n"
            f" h_det_topic={self.h_det_topic} aim_at_h={self.aim_at_h} "
            f"h_timeout_s={self.h_timeout_s} max_h_base_separation_m={self.max_h_base_separation_m}\n"
            f" waypoints_topic={self.waypoints_topic}\n"
            f" controller_state_topic={self.controller_state_topic}\n"
            f" mission_cycle_done_topic={self.mission_cycle_done_topic}\n"
            f" world_frame_id={self.world_frame_id} (NOTE: treated numerically as the odom frame)\n"
            f" z_fixed={self.z_fixed}\n"
            f" bases_to_visit={self.bases_to_visit}\n"
            f" cluster_tol_m={self.cluster_tol_m} min_seen_count={self.min_seen_count}\n"
            f" invert_right_axis={self.invert_right_axis} "
            f"target_offset_right_m={self.target_offset_right_m} "
            f"target_offset_front_m={self.target_offset_front_m}\n"
            f" max_detection_range_m={self.max_detection_range_m} "
            f"max_jump_m={self.max_jump_m}\n"
            f" lost_timeout_s={self.lost_timeout_s} "
            f"yaw_scan_cooldown_s={self.yaw_scan_cooldown_s}"
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

        # Capture home position from the very first odom message
        if not self._have_home_from_odom:
            self._have_home_from_odom = True
            self._home_from_odom_x = self.cur_x
            self._home_from_odom_y = self.cur_y
            self.get_logger().info(
                f"[HOME] Home captured from first odom: "
                f"({self._home_from_odom_x:.2f}, {self._home_from_odom_y:.2f})"
            )

    def cb_h_det(self, msg: PointStamped):
        """Store the latest H-marker relative position (right=x, front=y in base_link)."""
        right = float(msg.point.x)
        front = float(msg.point.y)
        if self.invert_right_axis:
            right = -right
        right += self.target_offset_right_m
        front += self.target_offset_front_m
        self._h_right = right
        self._h_front = front
        self._h_last_t = time.monotonic()

    def cb_det(self, msg: PointStamped, source: str):
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

        # msg.point convention (same for base_relative_position and legacy optical topics):
        #  - x = right (drone body frame, +right)
        #  - y = front (drone body frame, +forward)
        #  - z = fixed altitude (unused here)

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

        # Axis inversion: flip right (x) sign to correct systematic lateral bias
        if self.invert_right_axis:
            right = -right
            self.get_logger().debug(
                f"[{source}] invert_right_axis: x flipped to {right:.3f}"
            )

        # Fine-tuning offsets (applied in base_link before world projection)
        if self.target_offset_right_m != 0.0 or self.target_offset_front_m != 0.0:
            right += self.target_offset_right_m
            front += self.target_offset_front_m
            self.get_logger().debug(
                f"[{source}] offsets applied: right={right:.3f} front={front:.3f}"
            )

        # Outlier rejection: range check
        det_range = math.hypot(right, front)
        if det_range > self.max_detection_range_m:
            self.get_logger().warning(
                f"[{source}] Detection discarded: range={det_range:.2f}m > "
                f"max_detection_range_m={self.max_detection_range_m}m "
                f"(right={right:.2f} front={front:.2f})"
            )
            return

        # Outlier rejection: jump check from last accepted detection
        if self._have_last_accepted:
            jump = math.hypot(
                right - self._last_accepted_right,
                front - self._last_accepted_front,
            )
            if jump > self.max_jump_m:
                self.get_logger().warning(
                    f"[{source}] Detection discarded: jump={jump:.2f}m > "
                    f"max_jump_m={self.max_jump_m}m "
                    f"(right={right:.2f} front={front:.2f})"
                )
                return

        # Detection accepted — update last accepted position
        self._have_last_accepted = True
        self._last_accepted_right = right
        self._last_accepted_front = front

        # Store latest base relative position for H gating
        self._base_right = right
        self._base_front = front
        self._base_last_t = time.monotonic()

        # Project to world frame (2D) using odom yaw — no TF required.
        # dx_map = front*cos(yaw) + right*sin(yaw)
        # dy_map = front*sin(yaw) - right*cos(yaw)
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
                # Track when we last had a detection that contributes to a candidate
                self._last_valid_candidate_t = now
                return

        # New candidate
        self.candidates.append(BaseCandidate(x=bx, y=by, last_seen_s=now, seen_count=1))
        self._last_valid_candidate_t = now

    def _try_yaw_scan_recovery(self):
        """Trigger a non-blocking yaw-360 recovery scan if not already scanning."""
        now = time.time()
        # Poll existing subprocess first
        if self._yaw_scan_active and self._yaw_scan_proc is not None:
            ret = self._yaw_scan_proc.poll()
            if ret is not None:
                self._yaw_scan_active = False
                self._yaw_scan_proc = None
                if ret == 0:
                    self.get_logger().info("[LOST] drone_yaw_360 completed successfully.")
                else:
                    self.get_logger().warning(
                        f"[LOST] drone_yaw_360 exited with code {ret}."
                    )
            else:
                # Still running
                return
        if self._yaw_scan_active:
            return
        if now - self._last_yaw_scan_t < self.yaw_scan_cooldown_s:
            return
        self.get_logger().warning(
            "[LOST] No valid base candidate found for "
            f"{self.lost_timeout_s:.1f}s — triggering yaw-360 recovery scan."
        )
        try:
            proc = subprocess.Popen(
                ["ros2", "run", "drone_control", "drone_yaw_360"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            self._yaw_scan_active = True
            self._yaw_scan_proc = proc
            self._last_yaw_scan_t = now
            self.get_logger().info(
                f"[LOST] drone_yaw_360 launched (pid={proc.pid})."
            )
        except Exception as exc:
            self.get_logger().error(
                f"[LOST] Failed to launch drone_yaw_360: {exc}"
            )

    def _pick_next_target(self) -> Optional[Tuple[float, float]]:
        # Debug: log full candidate list each time we pick
        now = time.time()
        self.get_logger().info(
            f"[PICK] Candidate list ({len(self.candidates)} total, "
            f"visited_positions={len(self.visited_positions)}):"
        )
        for i, c in enumerate(self.candidates):
            blocked = any(
                math.hypot(c.x - vx, c.y - vy) <= self.repeat_block_m
                for vx, vy in self.visited_positions
            )
            age_s = now - c.last_seen_s
            self.get_logger().info(
                f"[PICK]   #{i}: map=({c.x:.2f},{c.y:.2f}) "
                f"seen={c.seen_count} age={age_s:.1f}s "
                f"visited={c.visited} blocked={blocked}"
            )

        # Choose nearest unvisited candidate with enough evidence
        best = None
        best_d = None
        for c in self.candidates:
            if c.visited:
                continue
            if c.seen_count < self.min_seen_count:
                continue
            # Anti-repeat guard: skip candidates too close to any previously
            # visited position (based on actual drone odom at visit time).
            if any(
                math.hypot(c.x - vx, c.y - vy) <= self.repeat_block_m
                for vx, vy in self.visited_positions
            ):
                continue
            d = math.hypot(c.x - self.cur_x, c.y - self.cur_y)
            if best is None or d < best_d:
                best = c
                best_d = d
        if best is None:
            return None
        self.get_logger().info(
            f"[PICK] Selected: map=({best.x:.2f},{best.y:.2f}) "
            f"seen={best.seen_count} dist={best_d:.2f}m"
        )
        return (best.x, best.y)

    def _mark_visited(self, x: float, y: float):
        for c in self.candidates:
            if math.hypot(c.x - x, c.y - y) <= self.cluster_tol_m:
                c.visited = True

    def _yaw_toward(
        self, from_x: float, from_y: float, to_x: float, to_y: float
    ) -> Tuple[float, float]:
        """Return (qz, qw) quaternion components for a yaw that faces (to_x,to_y) from (from_x,from_y)."""
        theta = math.atan2(to_y - from_y, to_x - from_x)
        return math.sin(theta / 2.0), math.cos(theta / 2.0)

    def _publish_two_pose_waypoints(self, tx: float, ty: float):
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = self.world_frame_id

        p0 = Pose()
        p0.position.x = float(self.cur_x)
        p0.position.y = float(self.cur_y)
        p0.position.z = float(self.z_fixed)

        p1 = Pose()
        p1.position.x = float(tx)
        p1.position.y = float(ty)
        p1.position.z = float(self.z_fixed)

        # Determine orientation: aim_at_h takes priority over aim_at_home.
        h_fresh = (time.monotonic() - self._h_last_t) <= self.h_timeout_s
        base_fresh = (time.monotonic() - self._base_last_t) <= self.h_timeout_s
        # Gate H: discard if H relative position is too far from the latest base detection.
        h_sep = math.hypot(
            self._h_right - self._base_right,
            self._h_front - self._base_front,
        ) if (h_fresh and base_fresh) else float("inf")
        h_valid = h_fresh and base_fresh and (h_sep <= self.max_h_base_separation_m)
        if self.aim_at_h and h_fresh and not h_valid:
            self.get_logger().debug(
                f"[aim_at_h] H discarded: sep={h_sep:.2f}m > "
                f"max_h_base_separation_m={self.max_h_base_separation_m}m"
            )
        if self.aim_at_h and h_valid:
            # Project H relative position (right, front in base_link) to world frame
            # using the current drone yaw from MAVROS odom (no TF required).
            #   world_dx = front * cos(yaw) + right * sin(yaw)
            #   world_dy = front * sin(yaw) - right * cos(yaw)
            yaw = self.cur_yaw
            h_dx = self._h_front * math.cos(yaw) + self._h_right * math.sin(yaw)
            h_dy = self._h_front * math.sin(yaw) - self._h_right * math.cos(yaw)
            h_map_x = self.cur_x + h_dx
            h_map_y = self.cur_y + h_dy
            # yaw_target = atan2(h_world.y - drone.y, h_world.x - drone.x)
            # Use the same drone-to-H direction for both current and target poses so
            # the camera stays locked on H throughout navigation.
            qz, qw = self._yaw_toward(self.cur_x, self.cur_y, h_map_x, h_map_y)
            p0.orientation.z = qz
            p0.orientation.w = qw
            p1.orientation.z = qz
            p1.orientation.w = qw
            self.get_logger().debug(
                f"[aim_at_h] H at world ({h_map_x:.2f},{h_map_y:.2f}); "
                f"yaw toward H set on both poses."
            )
        elif self.aim_at_h and not h_valid:
            self.get_logger().debug(
                "[aim_at_h] No valid H detection (timeout or gated); using identity orientation."
            )
            p0.orientation.w = 1.0
            p1.orientation.w = 1.0
        elif self.aim_at_home and self._have_home_from_odom:
            qz0, qw0 = self._yaw_toward(
                self.cur_x, self.cur_y,
                self._home_from_odom_x, self._home_from_odom_y,
            )
            p0.orientation.z = qz0
            p0.orientation.w = qw0
            qz1, qw1 = self._yaw_toward(
                tx, ty,
                self._home_from_odom_x, self._home_from_odom_y,
            )
            p1.orientation.z = qz1
            p1.orientation.w = qw1
        elif self.aim_at_home and not self._have_home_from_odom:
            self.get_logger().warning(
                "[aim_at_home] Home position not yet captured from odom; "
                "using identity orientation until first odom is received."
            )
            p0.orientation.w = 1.0
            p1.orientation.w = 1.0
        else:
            p0.orientation.w = 1.0
            p1.orientation.w = 1.0

        pa.poses = [p0, p1]
        self.pub_waypoints.publish(pa)

    def tick(self):
        if not self.have_odom:
            return

        # Poll the yaw-scan subprocess to detect completion
        if self._yaw_scan_active and self._yaw_scan_proc is not None:
            ret = self._yaw_scan_proc.poll()
            if ret is not None:
                self._yaw_scan_active = False
                self._yaw_scan_proc = None
                if ret == 0:
                    self.get_logger().info("[LOST] drone_yaw_360 completed successfully.")
                else:
                    self.get_logger().warning(
                        f"[LOST] drone_yaw_360 exited with code {ret}."
                    )
                # Reset lost timer so we don't immediately re-trigger
                self._last_valid_candidate_t = time.time()

        # Update lost-timer whenever a confirmed candidate exists
        confirmed_any = any(
            c.seen_count >= self.min_seen_count for c in self.candidates
        )
        if confirmed_any:
            self._last_valid_candidate_t = time.time()

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
                # Check if all confirmed candidates are already visited/blocked —
                # if so there is nothing left to find and we should go home rather
                # than waiting indefinitely (fixes "stuck at last base" regression).
                confirmed = [c for c in self.candidates if c.seen_count >= self.min_seen_count]
                all_done = bool(confirmed) and all(
                    c.visited or any(
                        math.hypot(c.x - vx, c.y - vy) <= self.repeat_block_m
                        for vx, vy in self.visited_positions
                    )
                    for c in confirmed
                )
                if all_done:
                    self.get_logger().info(
                        f"[DISCOVER] All {len(confirmed)} confirmed candidate(s) visited/blocked "
                        f"but only {self.visited_count}/{self.bases_to_visit} counted. "
                        "No more targets available — switching to RETURN_HOME."
                    )
                    self.state = "RETURN_HOME"
                    self.active_target = (self.home_x, self.home_y)
                    return
                # No valid target yet — check for "lost" condition and trigger yaw recovery
                if not confirmed:
                    elapsed_lost = time.time() - self._last_valid_candidate_t
                    if elapsed_lost > self.lost_timeout_s:
                        self._try_yaw_scan_recovery()
                # Still discovering bases (or waiting for more detections)
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

            # Mission cycle done: mark this base as visited using the actual
            # drone odom position (not the target coordinates) so that the
            # repeat_block_m guard correctly covers the real landing spot.
            visit_x = self.cur_x
            visit_y = self.cur_y
            self._mark_visited(visit_x, visit_y)
            self.visited_positions.append((visit_x, visit_y))

            self.visited_count += 1
            # Reset for next cycle (the pre-state reset at entry guarded against
            # stale signals; this reset ensures a clean state going forward).
            self.mission_cycle_done = False
            self.get_logger().info(
                f"[STATE] Mission cycle done — base marked visited at odom "
                f"({visit_x:.2f}, {visit_y:.2f}). "
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
