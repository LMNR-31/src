#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32


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
    Simplified pad waypoint supervisor.

    Builds a deduplicated list of landing bases from YOLO detections
    (/landing_pad/base_relative_position), confirms candidates by
    min_seen_count, visits up to bases_to_visit bases in nearest-neighbour
    order, and returns home (first odom position) after all bases are visited.

    Waypoints (/waypoints, PoseArray with 2 poses: current + target) are
    published only when state_voo == 2.  A configurable pause
    (inter_base_wait_s, default 5 s) is observed between consecutive bases;
    during this pause the node stays quiet (no waypoint publishes) but
    continues accumulating detections.

    When enable_h_centering is True, the final approach to each base uses
    the H marker topic (/landing_pad/h_relative_position) to refine the
    commanded target and ensure the drone centers on the pad before marking
    the base as reached.

    FSM states: COLLECT → NAVIGATE → WAIT_BETWEEN_BASES → RETURN_HOME → DONE
    """

    def __init__(self):
        super().__init__("pad_waypoint_supervisor")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("odom_topic", "/uav1/mavros/local_position/odom")
        self.declare_parameter("base_det_topic", "/landing_pad/base_relative_position")
        self.declare_parameter("h_det_topic", "/landing_pad/h_relative_position")
        self.declare_parameter("waypoints_topic", "/waypoints")
        self.declare_parameter("controller_state_topic", "/drone_controller/state_voo")

        self.declare_parameter("world_frame_id", "map")
        self.declare_parameter("z_fixed", 1.5)
        self.declare_parameter("bases_to_visit", 6)

        self.declare_parameter("cluster_tol_m", 0.8)    # merge detections within this radius
        self.declare_parameter("min_seen_count", 5)      # confirmations before accepting a base

        self.declare_parameter("publish_period_s", 0.25)  # waypoint re-publish interval (s)
        self.declare_parameter("reach_tol_m", 0.15)       # XY distance to consider "reached"
        self.declare_parameter("inter_base_wait_s", 5.0)  # pause between consecutive bases (s)

        # Outlier rejection
        self.declare_parameter("max_detection_range_m", 6.0)  # reject if range > this
        self.declare_parameter("max_jump_m", 2.0)              # reject if body-frame jump > this

        # Anti-repeat: skip candidates too close to already-visited positions
        self.declare_parameter("repeat_block_m", 1.2)

        # Stale candidate pruning (unvisited only)
        self.declare_parameter("candidate_timeout_s", 120.0)

        # H-marker centering parameters
        self.declare_parameter("enable_h_centering", True)
        self.declare_parameter("centering_start_dist_m", 0.6)
        self.declare_parameter("centering_reach_tol_m", 0.05)
        self.declare_parameter("h_timeout_s", 0.5)
        self.declare_parameter("max_h_range_m", 6.0)

        # ── Read parameters ───────────────────────────────────────────────────
        self.odom_topic = self.get_parameter("odom_topic").value
        self.base_det_topic = self.get_parameter("base_det_topic").value
        self.h_det_topic = self.get_parameter("h_det_topic").value
        self.waypoints_topic = self.get_parameter("waypoints_topic").value
        self.controller_state_topic = self.get_parameter("controller_state_topic").value

        self.world_frame_id = self.get_parameter("world_frame_id").value
        self.z_fixed = float(self.get_parameter("z_fixed").value)
        self.bases_to_visit = int(self.get_parameter("bases_to_visit").value)

        self.cluster_tol_m = float(self.get_parameter("cluster_tol_m").value)
        self.min_seen_count = int(self.get_parameter("min_seen_count").value)

        self.publish_period_s = float(self.get_parameter("publish_period_s").value)
        self.reach_tol_m = float(self.get_parameter("reach_tol_m").value)
        self.inter_base_wait_s = float(self.get_parameter("inter_base_wait_s").value)

        self.max_detection_range_m = float(
            self.get_parameter("max_detection_range_m").value)
        self.max_jump_m = float(self.get_parameter("max_jump_m").value)
        self.repeat_block_m = float(self.get_parameter("repeat_block_m").value)
        self.candidate_timeout_s = float(self.get_parameter("candidate_timeout_s").value)

        self.enable_h_centering = bool(self.get_parameter("enable_h_centering").value)
        self.centering_start_dist_m = float(
            self.get_parameter("centering_start_dist_m").value)
        self.centering_reach_tol_m = float(
            self.get_parameter("centering_reach_tol_m").value)
        self.h_timeout_s = float(self.get_parameter("h_timeout_s").value)
        self.max_h_range_m = float(self.get_parameter("max_h_range_m").value)

        # ── Runtime state ─────────────────────────────────────────────────────
        self.have_odom: bool = False
        self.cur_x: float = 0.0
        self.cur_y: float = 0.0
        self.cur_yaw: float = 0.0

        self.home_x: float = 0.0
        self.home_y: float = 0.0
        self._have_home: bool = False

        self.state_voo: Optional[int] = None

        self.candidates: List[BaseCandidate] = []
        self.active_target: Optional[Tuple[float, float]] = None
        self.visited_count: int = 0
        self.visited_positions: List[Tuple[float, float]] = []  # odom XY at each visit

        # Last accepted detection in body frame (for jump filter)
        self._last_det_right: Optional[float] = None
        self._last_det_front: Optional[float] = None

        # Latest H-marker detection (body frame) and its timestamp
        self._last_h_right: Optional[float] = None
        self._last_h_front: Optional[float] = None
        self._last_h_t: float = 0.0

        # FSM: COLLECT → NAVIGATE → WAIT_BETWEEN_BASES → RETURN_HOME → DONE
        self.state: str = "COLLECT"
        self._wait_start_t: float = 0.0   # monotonic time when WAIT_BETWEEN_BASES started

        self._last_det_info_t: float = 0.0      # throttle detection log to 1 Hz
        self._last_centering_info_t: float = 0.0  # throttle centering-active log
        self._last_h_stale_warn_t: float = 0.0   # throttle stale-H warning

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self.cb_odom, 10)
        self.sub_base = self.create_subscription(
            PointStamped, self.base_det_topic, self.cb_det, 10)
        self.sub_h = self.create_subscription(
            PointStamped, self.h_det_topic, self.cb_h_det, 10)
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
            "pad_waypoint_supervisor started (simplified).\n"
            f"  odom_topic={self.odom_topic}\n"
            f"  base_det_topic={self.base_det_topic}\n"
            f"  h_det_topic={self.h_det_topic}\n"
            f"  waypoints_topic={self.waypoints_topic}\n"
            f"  controller_state_topic={self.controller_state_topic}\n"
            f"  world_frame_id={self.world_frame_id}  z_fixed={self.z_fixed}\n"
            f"  bases_to_visit={self.bases_to_visit}\n"
            f"  cluster_tol_m={self.cluster_tol_m}  min_seen_count={self.min_seen_count}\n"
            f"  reach_tol_m={self.reach_tol_m}  inter_base_wait_s={self.inter_base_wait_s}\n"
            f"  max_detection_range_m={self.max_detection_range_m}"
            f"  max_jump_m={self.max_jump_m}\n"
            f"  repeat_block_m={self.repeat_block_m}"
            f"  candidate_timeout_s={self.candidate_timeout_s}\n"
            f"  enable_h_centering={self.enable_h_centering}"
            f"  centering_start_dist_m={self.centering_start_dist_m}\n"
            f"  centering_reach_tol_m={self.centering_reach_tol_m}"
            f"  h_timeout_s={self.h_timeout_s}"
            f"  max_h_range_m={self.max_h_range_m}"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cb_odom(self, msg: Odometry):
        self.have_odom = True
        self.cur_x = float(msg.pose.pose.position.x)
        self.cur_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.cur_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        if not self._have_home:
            self._have_home = True
            self.home_x = self.cur_x
            self.home_y = self.cur_y
            if self.world_frame_id == "map" and msg.header.frame_id:
                self.world_frame_id = msg.header.frame_id
                self.get_logger().info(
                    f"[FRAME] world_frame_id set from odom header: '{self.world_frame_id}'"
                )
            self.get_logger().info(
                f"[HOME] Captured home from first odom: ({self.home_x:.2f}, {self.home_y:.2f})"
            )

    def cb_state_voo(self, msg: Int32):
        self.state_voo = int(msg.data)

    def cb_det(self, msg: PointStamped):
        """Accumulate base detections into the candidate list."""
        if not self.have_odom:
            return
        if self.state == "DONE":
            return
        # Collect detections while hovering (2) or navigating (3)
        if self.state_voo not in (2, 3):
            return

        # Detection convention: msg.point.x = right, msg.point.y = front (drone body frame)
        right = float(msg.point.x)
        front = float(msg.point.y)

        # ── Outlier rejection ─────────────────────────────────────────────────
        det_range = math.hypot(right, front)
        if det_range > self.max_detection_range_m:
            self.get_logger().warn(
                f"[det] REJECTED range={det_range:.2f}m > max={self.max_detection_range_m}m "
                f"right={right:.2f} front={front:.2f}"
            )
            return

        if self._last_det_right is not None:
            jump = math.hypot(right - self._last_det_right, front - self._last_det_front)
            if jump > self.max_jump_m:
                self.get_logger().warn(
                    f"[det] REJECTED jump={jump:.2f}m > max={self.max_jump_m}m "
                    f"right={right:.2f} front={front:.2f}"
                )
                return

        self._last_det_right = right
        self._last_det_front = front

        # Project to world frame (ENU) using odom yaw (no TF required).
        #   dx_world = cos(yaw)*front + sin(yaw)*right
        #   dy_world = sin(yaw)*front - cos(yaw)*right
        yaw = self.cur_yaw
        bx = self.cur_x + front * math.cos(yaw) + right * math.sin(yaw)
        by = self.cur_y + front * math.sin(yaw) - right * math.cos(yaw)

        self._update_candidates(bx, by)

        now = time.time()
        if now - self._last_det_info_t >= 1.0:
            self._last_det_info_t = now
            max_seen = max((c.seen_count for c in self.candidates), default=0)
            self.get_logger().info(
                f"[det] right={right:.2f} front={front:.2f} yaw={yaw:.3f}rad "
                f"bx={bx:.2f} by={by:.2f} "
                f"candidates={len(self.candidates)} max_seen={max_seen} "
                f"state={self.state} state_voo={self.state_voo}"
            )

    def cb_h_det(self, msg: PointStamped):
        """Store the latest H-marker detection (body frame) for centering."""
        if not self.have_odom:
            return
        if self.state == "DONE":
            return
        self._last_h_right = float(msg.point.x)
        self._last_h_front = float(msg.point.y)
        self._last_h_t = time.monotonic()

    def _is_h_fresh(self) -> bool:
        """Return True if a recent H detection is available within h_timeout_s."""
        if self._last_h_right is None:
            return False
        return (time.monotonic() - self._last_h_t) <= self.h_timeout_s

    # ── Candidate management ───────────────────────────────────────────────────

    def _update_candidates(self, bx: float, by: float):
        now = time.time()
        # Prune stale unvisited candidates (never drop visited ones)
        if self.candidate_timeout_s > 0.0:
            before = len(self.candidates)
            self.candidates = [
                c for c in self.candidates
                if c.visited or (now - c.last_seen_s) < self.candidate_timeout_s
            ]
            dropped = before - len(self.candidates)
            if dropped:
                self.get_logger().warn(
                    f"[candidates] Pruned {dropped} stale unvisited candidate(s) "
                    f"(timeout={self.candidate_timeout_s:.0f}s)"
                )
        # Merge into the closest existing cluster within cluster_tol_m
        for c in self.candidates:
            if math.hypot(c.x - bx, c.y - by) <= self.cluster_tol_m:
                alpha = 0.3
                c.x = (1 - alpha) * c.x + alpha * bx
                c.y = (1 - alpha) * c.y + alpha * by
                c.last_seen_s = now
                c.seen_count += 1
                return
        # New candidate cluster
        self.candidates.append(BaseCandidate(x=bx, y=by, last_seen_s=now, seen_count=1))
        self.get_logger().info(
            f"[candidates] New candidate #{len(self.candidates)} at ({bx:.2f}, {by:.2f})"
        )

    def _pick_next_target(self, log: bool = False) -> Optional[Tuple[float, float]]:
        """Return (x, y) of the nearest confirmed unvisited base, or None."""
        best: Optional[BaseCandidate] = None
        best_d: float = math.inf
        for c in self.candidates:
            if c.visited or c.seen_count < self.min_seen_count:
                continue
            # Anti-repeat: skip if too close to any already-visited odom position
            if any(
                math.hypot(c.x - vx, c.y - vy) < self.repeat_block_m
                for vx, vy in self.visited_positions
            ):
                continue
            d = math.hypot(c.x - self.cur_x, c.y - self.cur_y)
            if d < best_d:
                best = c
                best_d = d
        if best is None:
            return None
        if log:
            self.get_logger().info(
                f"[PICK] Next target: ({best.x:.2f}, {best.y:.2f}) "
                f"dist={best_d:.2f}m seen={best.seen_count} "
                f"visited={self.visited_count}/{self.bases_to_visit}"
            )
        return (best.x, best.y)

    def _mark_visited_at_odom(
        self, active_target: Optional[Tuple[float, float]] = None
    ):
        """Mark candidates near the current odom position as visited.

        Uses repeat_block_m as the search radius (same value used in
        _pick_next_target) so both anti-repeat layers are consistent.
        When active_target is provided, candidates within cluster_tol_m of
        that position are also flagged, covering the case where H-centering
        placed the drone at the pad centre while the EMA candidate is offset.
        The active_target position is appended to visited_positions so that
        future new clusters near the same physical base are also blocked.
        """
        self.visited_positions.append((self.cur_x, self.cur_y))
        if active_target is not None:
            ax, ay = active_target
            # Record the base EMA position only if not already covered by odom
            if math.hypot(ax - self.cur_x, ay - self.cur_y) > self.cluster_tol_m:
                self.visited_positions.append((ax, ay))
        for c in self.candidates:
            if c.visited:
                continue
            # Mark if within repeat_block_m of current odom (drone's actual position)
            if math.hypot(c.x - self.cur_x, c.y - self.cur_y) <= self.repeat_block_m:
                c.visited = True
            # Also mark if within cluster_tol_m of the active base EMA center
            elif active_target is not None and math.hypot(
                c.x - active_target[0], c.y - active_target[1]
            ) <= self.cluster_tol_m:
                c.visited = True

    # ── Waypoint publisher ─────────────────────────────────────────────────────

    def _publish_waypoint(self, tx: float, ty: float):
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = self.world_frame_id

        p0 = Pose()
        p0.position.x = float(self.cur_x)
        p0.position.y = float(self.cur_y)
        p0.position.z = float(self.z_fixed)
        p0.orientation.w = 1.0

        p1 = Pose()
        p1.position.x = float(tx)
        p1.position.y = float(ty)
        p1.position.z = float(self.z_fixed)
        p1.orientation.w = 1.0

        pa.poses = [p0, p1]
        self.pub_waypoints.publish(pa)

    # ── Main FSM tick ──────────────────────────────────────────────────────────

    def tick(self):
        if not self.have_odom:
            return

        if self.state == "COLLECT":
            # Wait until at least one confirmed unvisited base is available.
            nxt = self._pick_next_target(log=True)
            if nxt is None:
                return
            self.active_target = nxt
            self.get_logger().info(
                f"[STATE] COLLECT → NAVIGATE target=({nxt[0]:.2f}, {nxt[1]:.2f})"
            )
            self.state = "NAVIGATE"

        if self.state == "NAVIGATE":
            if self.active_target is None:
                self.state = "COLLECT"
                return

            # Re-evaluate every tick: always fly to the nearest confirmed
            # unvisited base, updating immediately when a closer one appears.
            # active_target also tracks EMA position drift of the current base.
            nxt = self._pick_next_target()
            if nxt is not None:
                old_tx, old_ty = self.active_target
                if math.hypot(nxt[0] - old_tx, nxt[1] - old_ty) > self.cluster_tol_m:
                    # Target has switched to a genuinely different base
                    new_d = math.hypot(nxt[0] - self.cur_x, nxt[1] - self.cur_y)
                    cur_d = math.hypot(old_tx - self.cur_x, old_ty - self.cur_y)
                    self.get_logger().info(
                        f"[NAVIGATE] Closer base found: switching target "
                        f"({old_tx:.2f}, {old_ty:.2f}) d={cur_d:.2f}m → "
                        f"({nxt[0]:.2f}, {nxt[1]:.2f}) d={new_d:.2f}m"
                    )
                # Always update to latest EMA position of the best candidate
                self.active_target = nxt

            tx, ty = self.active_target

            # ── H-marker centering ────────────────────────────────────────────
            # When close to the active base and H is fresh, refine the commanded
            # target using the H marker world-frame position.
            d_base = math.hypot(tx - self.cur_x, ty - self.cur_y)
            centering_active = False
            cmd_x, cmd_y = tx, ty
            hx, hy = tx, ty  # will be overwritten if centering is active

            if self.enable_h_centering and d_base <= self.centering_start_dist_m:
                if self._is_h_fresh():
                    h_range = math.hypot(self._last_h_right, self._last_h_front)
                    if h_range <= self.max_h_range_m:
                        yaw = self.cur_yaw
                        hx = (self.cur_x
                              + self._last_h_front * math.cos(yaw)
                              + self._last_h_right * math.sin(yaw))
                        hy = (self.cur_y
                              + self._last_h_front * math.sin(yaw)
                              - self._last_h_right * math.cos(yaw))
                        cmd_x, cmd_y = hx, hy
                        centering_active = True
                        now_m = time.monotonic()
                        if now_m - self._last_centering_info_t >= 1.0:
                            self._last_centering_info_t = now_m
                            self.get_logger().info(
                                f"[CENTER] Centering on H marker: "
                                f"hx={hx:.2f} hy={hy:.2f} "
                                f"d_base={d_base:.2f}m"
                            )
                else:
                    now_m = time.monotonic()
                    if now_m - self._last_h_stale_warn_t >= 2.0:
                        self._last_h_stale_warn_t = now_m
                        self.get_logger().warn(
                            f"[CENTER] H detection stale - cannot center "
                            f"(d_base={d_base:.2f}m). Falling back to base center."
                        )

            # Only publish when the controller is in HOVER (state_voo == 2)
            if self.state_voo == 2:
                self._publish_waypoint(cmd_x, cmd_y)

            # Reach check: use H centering tolerance when centering, else base tol
            if centering_active:
                d_center = math.hypot(hx - self.cur_x, hy - self.cur_y)
                if d_center <= self.centering_reach_tol_m:
                    self._mark_visited_at_odom(self.active_target)
                    self.visited_count += 1
                    self.get_logger().info(
                        f"[STATE] Reached base #{self.visited_count} via H centering "
                        f"(d_center={d_center:.3f}m ≤ {self.centering_reach_tol_m}m) "
                        f"at odom ({self.cur_x:.2f}, {self.cur_y:.2f}). "
                        f"Entering inter-base wait ({self.inter_base_wait_s:.1f}s)."
                    )
                    self._wait_start_t = time.monotonic()
                    self.active_target = None
                    self.state = "WAIT_BETWEEN_BASES"
            else:
                d = math.hypot(cmd_x - self.cur_x, cmd_y - self.cur_y)
                if d <= self.reach_tol_m:
                    # Mark base as visited using the actual odom position
                    self._mark_visited_at_odom(self.active_target)
                    self.visited_count += 1
                    self.get_logger().info(
                        f"[STATE] Reached base #{self.visited_count} within {self.reach_tol_m}m "
                        f"(d={d:.2f}m) at odom ({self.cur_x:.2f}, {self.cur_y:.2f}). "
                        f"Entering inter-base wait ({self.inter_base_wait_s:.1f}s)."
                    )
                    self._wait_start_t = time.monotonic()
                    self.active_target = None
                    self.state = "WAIT_BETWEEN_BASES"
            return

        if self.state == "WAIT_BETWEEN_BASES":
            # Hold quiet — detections still accumulate via cb_det.
            elapsed = time.monotonic() - self._wait_start_t
            if elapsed < self.inter_base_wait_s:
                return
            self.get_logger().info(
                f"[STATE] Inter-base wait complete ({elapsed:.1f}s). "
                f"visited={self.visited_count}/{self.bases_to_visit}. "
                "Selecting next target."
            )
            if self.visited_count >= self.bases_to_visit:
                self.get_logger().info("[STATE] All bases visited. Switching to RETURN_HOME.")
                self.state = "RETURN_HOME"
            else:
                self.state = "COLLECT"
            return

        if self.state == "RETURN_HOME":
            if not self._have_home:
                return
            if self.state_voo == 2:
                self._publish_waypoint(self.home_x, self.home_y)
            d = math.hypot(self.home_x - self.cur_x, self.home_y - self.cur_y)
            if d <= self.reach_tol_m:
                self.get_logger().info(
                    f"[STATE] Returned home (d={d:.2f}m). Mission complete. DONE."
                )
                self.state = "DONE"
            return

        # state == "DONE": nothing to do


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
