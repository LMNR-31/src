#!/usr/bin/env python3
"""base_waypoint_publisher — nearest-neighbour landing-pad waypoint publisher.

Replaces ``pad_waypoint_supervisor`` with a simpler node focused on the
6-base nearest-neighbour mission:

1. Collect up to ``max_bases`` (default 6) landing-pad base positions from
   YOLO detections on ``det_topic``; convert each body-frame detection to
   world frame using current odom yaw.

2. Merge detections that represent the same physical base.  Merge radius is
   derived from ``merge_area_m2`` (default 2.25 m²) as::

       merge_radius_m = sqrt(merge_area_m2 / pi)   # ≈ 0.846 m for 2.25 m²

   If a new world-frame detection is within ``merge_radius_m`` of an existing
   base, update that base with an EMA (α = 0.3).  If it is farther from all
   existing bases:
   - if fewer than ``max_bases`` bases exist, add it;
   - otherwise discard (never replace an existing base once 6 are known).

   Multiple bases are discovered and updated concurrently at any time,
   including while the drone is flying to another target.  The old body-frame
   jump filter (``max_jump_m``) has been replaced by this world-frame
   clustering, which is compatible with YOLO switching between bases.

3. Publish ``geometry_msgs/PoseArray`` (2 poses: current + target) on
   ``waypoints_topic`` **only when** the controller reports state == 2.

4. Visiting order: nearest-neighbour from current odom position among
   unvisited bases.  When the drone is within ``reach_tol_m`` (XY) of the
   target, start a ``dwell_s``-second timer.  After dwell, select the next
   nearest unvisited base.  If no confirmed unvisited base is available after
   dwell, the FSM falls back to COLLECT to wait for more bases to be confirmed.

5. After all ``max_bases`` bases (or all currently-known bases if fewer;
   see ``require_all_bases`` parameter) have been visited, fly to the home
   position captured from the first odom message.  Mission is complete when
   within ``reach_tol_m`` of home.

Parameters (ROS)
----------------
odom_topic              string   /uav1/mavros/local_position/odom
det_topic               string   /landing_pad/base_relative_position
waypoints_topic         string   /waypoints
controller_state_topic  string   /drone_controller/state_voo
world_frame_id          string   map  (auto-overridden from first odom header)
z_fixed                 float    1.75  (m; all waypoints use this Z)
max_bases               int      6
merge_area_m2           float    2.25  (merge radius = sqrt(area/pi) ≈ 0.846 m)
min_seen_count          int      3     (detections required before base is confirmed)
reach_tol_m             float    0.10  (XY distance to consider target reached)
dwell_s                 float    5.0   (seconds to wait at each visited base)
publish_period_s        float    0.25  (waypoint re-publish interval)
max_detection_range_m   float    6.0   (reject body-frame detections beyond this)
max_jump_m              float    2.0   (DEPRECATED – no longer applied; kept for
                                        backward compatibility only; set to 0 to
                                        silence the deprecation warning at startup)
require_all_bases       bool     true  (if true, require max_bases before returning home;
                                        if false, return home after all *known* bases visited)
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

# ---------------------------------------------------------------------------
# Pure helper functions (testable without ROS)
# ---------------------------------------------------------------------------


def merge_radius_from_area(area_m2: float) -> float:
    """Compute the merge radius (m) from a circular merge area (m²).

    Interprets ``area_m2`` as the area of a circle and solves for the radius::

        r = sqrt(area_m2 / pi)

    Examples:
        >>> merge_radius_from_area(2.25)   # ≈ 0.846 m
        0.8462843753216482
        >>> merge_radius_from_area(0.0)
        0.0
    """
    if area_m2 <= 0.0:
        return 0.0
    return math.sqrt(area_m2 / math.pi)


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw (Z-axis rotation) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def body_to_world(
    front: float, right: float, cur_x: float, cur_y: float, yaw: float
) -> Tuple[float, float]:
    """Project a body-frame detection to world frame (ENU) using odom yaw.

    Detection convention: ``right`` = +starboard, ``front`` = +nose.

    Equivalent projection (maps body → ENU world)::

        dx_world = cos(yaw)*front + sin(yaw)*right
        dy_world = sin(yaw)*front - cos(yaw)*right

    Args:
        front: Distance along drone's nose direction (m).
        right: Distance along drone's starboard direction (m).
        cur_x: Drone's current world-frame X (m).
        cur_y: Drone's current world-frame Y (m).
        yaw: Drone's heading in ENU frame (rad, CCW positive).

    Returns:
        ``(wx, wy)`` world-frame position of the detection.
    """
    wx = cur_x + front * math.cos(yaw) + right * math.sin(yaw)
    wy = cur_y + front * math.sin(yaw) - right * math.cos(yaw)
    return wx, wy


@dataclass
class BaseEntry:
    """Running estimate of one physical landing-pad base."""

    x: float
    y: float
    seen_count: int = 1
    visited: bool = False
    # EMA smoothing factor (not a constructor parameter; fixed)
    _ema_alpha: float = field(default=0.3, init=False, repr=False)

    def update(self, nx: float, ny: float) -> None:
        """Merge a new world-frame observation into this estimate via EMA."""
        a = self._ema_alpha
        self.x = (1.0 - a) * self.x + a * nx
        self.y = (1.0 - a) * self.y + a * ny
        self.seen_count += 1


def cluster_update(
    bases: List[BaseEntry],
    nx: float,
    ny: float,
    merge_radius: float,
    max_bases: int,
) -> Tuple[List[BaseEntry], str]:
    """Update the base list with a new world-frame detection.

    Returns the updated list and an action string (one of ``'merged'``,
    ``'added'``, or ``'discarded'``).

    Merge policy:
        * Find the closest existing base within ``merge_radius``.
        * If found: update (EMA) that base → action = ``'merged'``.
        * If not found and ``len(bases) < max_bases``: add new → action = ``'added'``.
        * If not found and list is full: discard → action = ``'discarded'``.

    Args:
        bases: Current list of known bases (mutated in-place).
        nx: New detection X in world frame (m).
        ny: New detection Y in world frame (m).
        merge_radius: Maximum distance (m) to merge into an existing base.
        max_bases: Maximum number of bases to track.

    Returns:
        ``(bases, action)`` where ``action`` is ``'merged'``, ``'added'``, or
        ``'discarded'``.
    """
    best_idx: Optional[int] = None
    best_d: float = math.inf
    for i, b in enumerate(bases):
        d = math.hypot(b.x - nx, b.y - ny)
        if d < best_d:
            best_idx = i
            best_d = d

    if best_d <= merge_radius:
        bases[best_idx].update(nx, ny)
        return bases, 'merged'

    if len(bases) < max_bases:
        bases.append(BaseEntry(x=nx, y=ny))
        return bases, 'added'

    return bases, 'discarded'


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------


class BaseWaypointPublisher(Node):
    """Nearest-neighbour landing-pad waypoint publisher.

    FSM states:
        COLLECT        – waiting for at least 1 confirmed base and state_voo == 2.
        NAVIGATE       – flying to a target base; publishing waypoints.
        DWELL          – reached target; waiting dwell_s before next base.
        RETURN_HOME    – all bases visited; flying back to origin.
        DONE           – mission complete.
    """

    def __init__(self):
        super().__init__("base_waypoint_publisher")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("odom_topic", "/uav1/mavros/local_position/odom")
        self.declare_parameter("det_topic", "/landing_pad/base_relative_position")
        self.declare_parameter("waypoints_topic", "/waypoints")
        self.declare_parameter("controller_state_topic", "/drone_controller/state_voo")

        self.declare_parameter("world_frame_id", "map")
        self.declare_parameter("z_fixed", 1.75)
        self.declare_parameter("max_bases", 6)

        # merge_area_m2 = 2.25 → merge_radius = sqrt(2.25/pi) ≈ 0.846 m
        self.declare_parameter("merge_area_m2", 2.25)
        self.declare_parameter("min_seen_count", 3)

        self.declare_parameter("reach_tol_m", 0.10)
        self.declare_parameter("dwell_s", 5.0)
        self.declare_parameter("publish_period_s", 0.25)

        # Outlier rejection
        self.declare_parameter("max_detection_range_m", 6.0)
        self.declare_parameter("max_jump_m", 2.0)

        # If True, wait until max_bases are visited before returning home.
        # If False, return home after all *currently known* bases are visited.
        self.declare_parameter("require_all_bases", True)

        # ── Read parameters ───────────────────────────────────────────────────
        self.odom_topic: str = self.get_parameter("odom_topic").value
        self.det_topic: str = self.get_parameter("det_topic").value
        self.waypoints_topic: str = self.get_parameter("waypoints_topic").value
        self.controller_state_topic: str = (
            self.get_parameter("controller_state_topic").value
        )

        self.world_frame_id: str = self.get_parameter("world_frame_id").value
        self.z_fixed: float = float(self.get_parameter("z_fixed").value)
        self.max_bases: int = int(self.get_parameter("max_bases").value)

        merge_area = float(self.get_parameter("merge_area_m2").value)
        self.merge_radius: float = merge_radius_from_area(merge_area)

        self.min_seen_count: int = int(self.get_parameter("min_seen_count").value)
        self.reach_tol_m: float = float(self.get_parameter("reach_tol_m").value)
        self.dwell_s: float = float(self.get_parameter("dwell_s").value)
        self.publish_period_s: float = float(
            self.get_parameter("publish_period_s").value
        )
        self.max_detection_range_m: float = float(
            self.get_parameter("max_detection_range_m").value
        )
        # max_jump_m is kept for backward compatibility but is no longer applied.
        # World-frame clustering (cluster_update) replaces the body-frame jump filter.
        self.max_jump_m: float = float(self.get_parameter("max_jump_m").value)
        self.require_all_bases: bool = bool(
            self.get_parameter("require_all_bases").value
        )

        # ── Runtime state ─────────────────────────────────────────────────────
        self.have_odom: bool = False
        self.cur_x: float = 0.0
        self.cur_y: float = 0.0
        self.cur_yaw: float = 0.0

        self.home_x: float = 0.0
        self.home_y: float = 0.0
        self._have_home: bool = False

        self.state_voo: Optional[int] = None

        self.bases: List[BaseEntry] = []
        self.active_target: Optional[Tuple[float, float]] = None

        # FSM
        self.fsm_state: str = "COLLECT"
        self._dwell_start_t: float = 0.0
        # Guard: ensure DWELL-complete is logged only once per dwell event.
        self._dwell_completed: bool = False

        # Throttle timestamps for warnings
        self._last_range_warn_t: float = -math.inf
        self._last_det_log_t: float = -math.inf

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self.cb_odom, 10
        )
        self.sub_det = self.create_subscription(
            PointStamped, self.det_topic, self.cb_det, 10
        )
        self.sub_state_voo = self.create_subscription(
            Int32,
            self.controller_state_topic,
            self.cb_state_voo,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            ),
        )

        self.pub_waypoints = self.create_publisher(PoseArray, self.waypoints_topic, 10)

        self.timer = self.create_timer(self.publish_period_s, self.tick)

        self.get_logger().info(
            "base_waypoint_publisher started.\n"
            f"  odom_topic={self.odom_topic}\n"
            f"  det_topic={self.det_topic}\n"
            f"  waypoints_topic={self.waypoints_topic}\n"
            f"  controller_state_topic={self.controller_state_topic}\n"
            f"  world_frame_id={self.world_frame_id}  z_fixed={self.z_fixed}\n"
            f"  max_bases={self.max_bases}\n"
            f"  merge_area_m2={merge_area:.3f}  merge_radius={self.merge_radius:.4f} m\n"
            f"  min_seen_count={self.min_seen_count}\n"
            f"  reach_tol_m={self.reach_tol_m}  dwell_s={self.dwell_s}\n"
            f"  max_detection_range_m={self.max_detection_range_m}\n"
            f"  require_all_bases={self.require_all_bases}"
        )
        if self.max_jump_m > 0.0:
            self.get_logger().warn(
                f"[DEPRECATED] max_jump_m={self.max_jump_m} is set but no longer "
                "applied. Body-frame jump filtering has been replaced by world-frame "
                "clustering (cluster_update). Set max_jump_m:=0 to silence this warning."
            )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cb_odom(self, msg: Odometry) -> None:
        """Update current pose; capture home from the first message."""
        self.have_odom = True
        self.cur_x = float(msg.pose.pose.position.x)
        self.cur_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.cur_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        if not self._have_home:
            self._have_home = True
            self.home_x = self.cur_x
            self.home_y = self.cur_y
            # Override world_frame_id from first odom header (like existing supervisor)
            if self.world_frame_id == "map" and msg.header.frame_id:
                self.world_frame_id = msg.header.frame_id
                self.get_logger().info(
                    f"[FRAME] world_frame_id set from odom: '{self.world_frame_id}'"
                )
            self.get_logger().info(
                f"[HOME] Captured home: ({self.home_x:.2f}, {self.home_y:.2f})"
            )

    def cb_state_voo(self, msg: Int32) -> None:
        self.state_voo = int(msg.data)

    def cb_det(self, msg: PointStamped) -> None:
        """Accumulate base detections; always update bases while flying.

        Outlier rejection uses only:
        - ``max_detection_range_m``: reject detections too far in body frame.
        - World-frame clustering (``cluster_update``): merge close detections
          into the same base, add new ones up to ``max_bases``, or discard.

        The old body-frame jump filter (``max_jump_m``) has been removed.
        It prevented YOLO from discovering/updating bases #2..#6 whenever
        the detector switched between pads (typical jump ~3 m > 2 m limit).
        """
        if not self.have_odom:
            return
        if self.fsm_state == "DONE":
            return

        # Detection convention: point.x = right (+starboard), point.y = front (+nose)
        right = float(msg.point.x)
        front = float(msg.point.y)

        # ── Outlier rejection: range ──────────────────────────────────────────
        det_range = math.hypot(right, front)
        if self.max_detection_range_m > 0.0 and det_range > self.max_detection_range_m:
            now = time.monotonic()
            if now - self._last_range_warn_t >= 1.0:
                self._last_range_warn_t = now
                self.get_logger().warn(
                    f"[det] REJECTED range={det_range:.2f} m > "
                    f"max={self.max_detection_range_m:.2f} m  "
                    f"right={right:.2f} front={front:.2f}"
                )
            return

        # Project body frame → world frame
        wx, wy = body_to_world(front, right, self.cur_x, self.cur_y, self.cur_yaw)

        # Update base list via clustering
        self.bases, action = cluster_update(
            self.bases, wx, wy, self.merge_radius, self.max_bases
        )

        if action == 'added':
            self.get_logger().info(
                f"[BASE] New base #{len(self.bases)} added at "
                f"({wx:.2f}, {wy:.2f})"
            )
        elif action == 'merged':
            now = time.monotonic()
            if now - self._last_det_log_t >= 1.0:
                self._last_det_log_t = now
                idx = min(
                    range(len(self.bases)),
                    key=lambda i: math.hypot(
                        self.bases[i].x - wx, self.bases[i].y - wy
                    ),
                )
                b = self.bases[idx]
                self.get_logger().debug(
                    f"[BASE] Merged into base #{idx + 1} at "
                    f"({b.x:.2f}, {b.y:.2f})  seen={b.seen_count}  "
                    f"det=({wx:.2f}, {wy:.2f})"
                )
        # 'discarded' is silent (6 known bases → any new unique cluster ignored)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _confirmed_bases(self) -> List[BaseEntry]:
        """Return confirmed (seen_count >= min_seen_count) bases."""
        return [b for b in self.bases if b.seen_count >= self.min_seen_count]

    def _pick_nearest_unvisited(self) -> Optional[Tuple[float, float]]:
        """Return (x, y) of the nearest confirmed unvisited base, or None."""
        best: Optional[BaseEntry] = None
        best_d: float = math.inf
        for b in self._confirmed_bases():
            if b.visited:
                continue
            d = math.hypot(b.x - self.cur_x, b.y - self.cur_y)
            if d < best_d:
                best = b
                best_d = d
        if best is None:
            return None
        return (best.x, best.y)

    def _mark_active_visited(self) -> None:
        """Mark the base nearest to the current position as visited."""
        if not self.active_target:
            return
        tx, ty = self.active_target
        best: Optional[BaseEntry] = None
        best_d: float = math.inf
        for b in self.bases:
            if b.visited:
                continue
            d = math.hypot(b.x - tx, b.y - ty)
            if d < best_d:
                best = b
                best_d = d
        if best is not None:
            best.visited = True

    def _all_bases_visited(self) -> bool:
        """Return True when the mission-completion criterion is met.

        If ``require_all_bases`` is True, all ``max_bases`` confirmed bases
        must have been visited.  If False, all *currently known* confirmed
        bases must have been visited (i.e. no unvisited confirmed bases remain).
        """
        confirmed = self._confirmed_bases()
        if self.require_all_bases:
            visited_count = sum(1 for b in confirmed if b.visited)
            return visited_count >= self.max_bases
        return all(b.visited for b in confirmed) and len(confirmed) > 0

    def _publish_waypoint(self, tx: float, ty: float) -> None:
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

    def tick(self) -> None:
        """Timer callback; drives the FSM and publishes waypoints."""
        if not self.have_odom:
            return
        # Publish waypoints only when controller is in state 2
        if self.state_voo != 2:
            return

        if self.fsm_state == "COLLECT":
            target = self._pick_nearest_unvisited()
            if target is None:
                return  # No confirmed base yet; keep collecting
            self.active_target = target
            self.get_logger().info(
                f"[NAV] Selecting first target: ({target[0]:.2f}, {target[1]:.2f})"
            )
            self.fsm_state = "NAVIGATE"
            self._publish_waypoint(*self.active_target)

        elif self.fsm_state == "NAVIGATE":
            # Update target to current EMA (bases keep improving while flying)
            target = self._pick_nearest_unvisited()
            if target is not None:
                self.active_target = target

            tx, ty = self.active_target
            dist = math.hypot(self.cur_x - tx, self.cur_y - ty)

            if dist <= self.reach_tol_m:
                self.get_logger().info(
                    f"[REACH] Reached base at ({tx:.2f}, {ty:.2f})  "
                    f"dist={dist:.3f} m. Starting {self.dwell_s:.1f} s dwell."
                )
                self._mark_active_visited()
                self._dwell_start_t = time.monotonic()
                self._dwell_completed = False
                self.fsm_state = "DWELL"
            else:
                self._publish_waypoint(tx, ty)

        elif self.fsm_state == "DWELL":
            elapsed = time.monotonic() - self._dwell_start_t
            if elapsed >= self.dwell_s:
                if not self._dwell_completed:
                    self._dwell_completed = True
                    self.get_logger().info(
                        f"[DWELL] Dwell complete ({elapsed:.1f} s)."
                    )
                if self._all_bases_visited():
                    self.get_logger().info(
                        "[MISSION] All bases visited. Returning home "
                        f"({self.home_x:.2f}, {self.home_y:.2f})."
                    )
                    self.active_target = (self.home_x, self.home_y)
                    self.fsm_state = "RETURN_HOME"
                    self._publish_waypoint(self.home_x, self.home_y)
                else:
                    next_t = self._pick_nearest_unvisited()
                    if next_t is None:
                        # No confirmed unvisited base yet; fall back to COLLECT
                        # to wait for more bases to be confirmed (avoids DWELL spam).
                        self.get_logger().info(
                            "[COLLECT] No confirmed unvisited base after dwell; "
                            "waiting for more bases to be discovered."
                        )
                        self.fsm_state = "COLLECT"
                        return
                    self.active_target = next_t
                    self.get_logger().info(
                        f"[NAV] Next target: ({next_t[0]:.2f}, {next_t[1]:.2f})"
                    )
                    self.fsm_state = "NAVIGATE"
                    self._publish_waypoint(*self.active_target)
            # Still dwelling; no waypoint published (drone holds position)

        elif self.fsm_state == "RETURN_HOME":
            dist = math.hypot(self.cur_x - self.home_x, self.cur_y - self.home_y)
            if dist <= self.reach_tol_m:
                self.get_logger().info(
                    f"[DONE] Reached home ({self.home_x:.2f}, {self.home_y:.2f})  "
                    f"dist={dist:.3f} m. Mission complete."
                )
                self.fsm_state = "DONE"
            else:
                self._publish_waypoint(self.home_x, self.home_y)

        # DONE: do nothing


def main():
    rclpy.init()
    node = BaseWaypointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
