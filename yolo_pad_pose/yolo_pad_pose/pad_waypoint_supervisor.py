#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


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
        self.declare_parameter("front_det_topic", "/landing_pad/front_optical_point")
        self.declare_parameter("down_det_topic", "/landing_pad/down_optical_point")
        self.declare_parameter("waypoints_topic", "/waypoints")

        self.declare_parameter("trajectory_finished_topic", "/trajectory_finished")
        self.declare_parameter("use_trajectory_finished", True)

        self.declare_parameter("map_frame", "uav1/map")     # used in PoseArray.header.frame_id
        self.declare_parameter("z_fixed", 1.5)
        self.declare_parameter("bases_to_visit", 6)

        self.declare_parameter("cluster_tol_m", 0.7)        # merge detections within this radius
        self.declare_parameter("min_seen_count", 3)         # require stable detection before accepting a base
        self.declare_parameter("candidate_timeout_s", 5.0)  # drop candidates not seen recently

        self.declare_parameter("publish_period_s", 0.25)    # how often to republish target while navigating
        self.declare_parameter("reach_tol_m", 0.6)           # consider reached when within this XY distance
        self.declare_parameter("home_x", 0.0)
        self.declare_parameter("home_y", 0.0)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.front_det_topic = self.get_parameter("front_det_topic").value
        self.down_det_topic = self.get_parameter("down_det_topic").value
        self.waypoints_topic = self.get_parameter("waypoints_topic").value

        self.trajectory_finished_topic = self.get_parameter("trajectory_finished_topic").value
        self.use_trajectory_finished = bool(self.get_parameter("use_trajectory_finished").value)

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

        # State
        self.have_odom = False
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0

        self.trajectory_finished = False
        self._last_finished_msg_t = 0.0

        self.candidates: List[BaseCandidate] = []
        self.active_target: Optional[Tuple[float, float]] = None
        self.visited_count = 0
        self.state = "DISCOVER"   # DISCOVER -> NAVIGATE -> WAIT_DONE -> ... -> RETURN_HOME -> DONE

        # ROS I/O
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 10)
        self.sub_front = self.create_subscription(PointStamped, self.front_det_topic, lambda m: self.cb_det(m, "front"), 10)
        self.sub_down = self.create_subscription(PointStamped, self.down_det_topic, lambda m: self.cb_det(m, "down"), 10)

        if self.use_trajectory_finished:
            self.sub_finished = self.create_subscription(Bool, self.trajectory_finished_topic, self.cb_finished, 10)
        else:
            self.sub_finished = None

        self.pub_waypoints = self.create_publisher(PoseArray, self.waypoints_topic, 10)

        self.timer = self.create_timer(self.publish_period_s, self.tick)

        self.get_logger().info(
            "pad_waypoint_supervisor started.\n"
            f" odom_topic={self.odom_topic}\n"
            f" front_det_topic={self.front_det_topic}\n"
            f" down_det_topic={self.down_det_topic}\n"
            f" waypoints_topic={self.waypoints_topic}\n"
            f" map_frame={self.map_frame} z_fixed={self.z_fixed}\n"
            f" bases_to_visit={self.bases_to_visit}\n"
            f" cluster_tol_m={self.cluster_tol_m} min_seen_count={self.min_seen_count}"
        )

    def cb_finished(self, msg: Bool):
        self.trajectory_finished = bool(msg.data)
        self._last_finished_msg_t = time.time()

    def cb_odom(self, msg: Odometry):
        self.have_odom = True
        self.cur_x = float(msg.pose.pose.position.x)
        self.cur_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.cur_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        # Update map frame from odom if it looks namespaced (optional)
        if msg.header.frame_id:
            self.map_frame = msg.header.frame_id

    def cb_det(self, msg: PointStamped, source: str):
        # Need odom to project to map
        if not self.have_odom:
            return

        # msg.point is in optical frame. Our detector currently publishes:
        #  - x ~ X_optical (right)
        #  - y ~ Y_optical (down)   [not used]
        #  - z ~ Z_optical (forward)
        right = float(msg.point.x)
        front = float(msg.point.z)

        # Project to map (2D)
        yaw = self.cur_yaw
        dx_map = front * math.cos(yaw) + right * math.sin(yaw)
        dy_map = front * math.sin(yaw) - right * math.cos(yaw)

        bx = self.cur_x + dx_map
        by = self.cur_y + dy_map

        self._update_candidates(bx, by)

    def _update_candidates(self, bx: float, by: float):
        now = time.time()

        # Drop stale candidates
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
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = self.map_frame

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

    def tick(self):
        if not self.have_odom:
            return

        # If finished topic is enabled, require it to be "recent" (avoid stale latched true)
        finished_recent = True
        if self.use_trajectory_finished:
            finished_recent = (time.time() - self._last_finished_msg_t) < 2.0

        # State machine
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
                self.get_logger().info(f"Reached target within {self.reach_tol_m}m (d={d:.2f}). Waiting mission completion.")
                self.state = "WAIT_DONE"
                # reset finished flag so we wait for the next completion edge
                self.trajectory_finished = False
                return

        if self.state == "WAIT_DONE":
            # If you have trajectory_finished, wait for it to be true.
            # Otherwise just wait a bit.
            if self.use_trajectory_finished:
                if finished_recent and self.trajectory_finished:
                    # mark visited and move on
                    tx, ty = self.active_target if self.active_target else (None, None)
                    if tx is not None:
                        self._mark_visited(tx, ty)

                    self.visited_count += 1
                    self.get_logger().info(f"Mission done. visited={self.visited_count}/{self.bases_to_visit}")
                    self.active_target = None
                    self.state = "DISCOVER"
            else:
                # simple timeout fallback
                time.sleep(1.0)
                tx, ty = self.active_target if self.active_target else (None, None)
                if tx is not None:
                    self._mark_visited(tx, ty)
                self.visited_count += 1
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
