# yolo_pad_pose

ROS 2 package containing YOLO-based landing pad detection and the pad
waypoint supervisor.

---

## yolo_pad_pose (detector node)

### Range filter parameters

To prevent high-confidence false positives at long range from being published
(and subsequently spamming downstream nodes with rejection warnings), the node
supports configurable body-frame range limits.

| Parameter          | Type  | Default | Description |
|--------------------|-------|---------|-------------|
| `max_base_range_m` | float | `6.0`   | Maximum body-frame range `hypot(right, front)` for **base** detections. Detections beyond this value are not published to `/landing_pad/base_relative_position` (nor the legacy alias `/landing_pad/relative_position`). Set to `0.0` to disable. |
| `max_h_range_m`    | float | `6.0`   | Maximum body-frame range for **H-marker** detections. Detections beyond this value are not published to `/landing_pad/h_relative_position`. Set to `0.0` to disable. |

When a detection is suppressed by the range filter, a throttled warning (at
most 1 Hz per class) is logged showing the measured range, the limit, and the
detection confidence.

Example — suppress base detections beyond 4.0 m (eliminates the observed
~4.75 m false positives):

```bash
ros2 run yolo_pad_pose yolo_pad_pose --ros-args \
  -p max_base_range_m:=4.0
```

These parameters can also be changed at runtime:

```bash
ros2 param set /yolo_pad_pose max_base_range_m 4.0
ros2 param set /yolo_pad_pose max_h_range_m 4.0
```

---

## pad_waypoint_supervisor

### Detection convention and coordinate projection

Detection messages (`/landing_pad/base_relative_position`,
`geometry_msgs/PointStamped`, `frame_id = uav1/base_link`) use the convention:

| Field       | Meaning            |
|-------------|--------------------|
| `point.x`   | **right** (+right of drone, i.e. starboard) |
| `point.y`   | **front** (+forward, nose direction)        |
| `point.z`   | fixed altitude (ignored here)               |

This is **not** the ROS base_link convention (REP-103: `x = forward`, `y = left`),
but the projection formula inside `cb_det` accounts for it:

```
# Map detection frame → ROS base_link:
#   x_bl = front_det,  y_bl = left = -right_det
# ENU rotation by yaw (CCW positive):
dx_world = cos(yaw)*front + sin(yaw)*right   # = cos(yaw)*x_bl - sin(yaw)*y_bl
dy_world = sin(yaw)*front - cos(yaw)*right   # = sin(yaw)*x_bl + cos(yaw)*y_bl
```

where `yaw` is the heading read from `/uav1/mavros/local_position/odom`
(standard ENU convention, CCW positive).

### Standalone mode (no `/mission_cycle_done` handshake)

By default the supervisor enters `WAIT_MISSION_DONE` after reaching each base
and blocks until `supervisor_T` publishes `/mission_cycle_done=true`.  If you
are running the node **standalone** (without `supervisor_T`), set
`use_mission_cycle_done:=false`:

```bash
ros2 run yolo_pad_pose pad_waypoint_supervisor --ros-args \
  -p use_mission_cycle_done:=false
```

In this mode the base is marked visited **immediately** upon reaching the target
(within `reach_tol_m`) and the node transitions straight back to `DISCOVER`.

Alternatively, keep the handshake enabled but add a safety timeout so the node
never blocks indefinitely if the signal is lost:

```bash
ros2 run yolo_pad_pose pad_waypoint_supervisor --ros-args \
  -p mission_cycle_timeout_s:=30.0
```

After `mission_cycle_timeout_s` seconds without the handshake, the node logs a
warning and forces progress (marks visited, continues to next base).

| Parameter                  | Type  | Default | Description |
|----------------------------|-------|---------|-------------|
| `use_mission_cycle_done`   | bool  | `true`  | If `false`, skip the `/mission_cycle_done` handshake entirely (standalone mode). |
| `mission_cycle_timeout_s`  | float | `0.0`   | Seconds to wait for `/mission_cycle_done` before forcing progress. `0.0` disables the timeout. Only used when `use_mission_cycle_done=true`. |

### H-marker final centering

When `enable_h_centering` is `true` (the default), the supervisor uses the YOLO
H-marker topic (`/landing_pad/h_relative_position`) to perform a final centering
pass before marking a base as reached.  The same coordinate projection formula
used for base detections is applied to the H detection to compute a world-frame
target `(hx, hy)`:

```
dx_world = cos(yaw)*front_H + sin(yaw)*right_H
dy_world = sin(yaw)*front_H - cos(yaw)*right_H
```

**Behaviour:**

1. While navigating to a base, once the drone is within `centering_start_dist_m`
   of the candidate center **and** a fresh H detection is available (received
   within the last `h_timeout_s` seconds), the supervisor switches the commanded
   waypoint from the candidate EMA position to the projected H center `(hx, hy)`.
2. The base is only considered **reached** when the drone is within
   `centering_reach_tol_m` of `(hx, hy)` (instead of the normal `reach_tol_m`).
3. If H detections become stale while centering, the supervisor falls back to
   the candidate center and normal `reach_tol_m` criterion (a throttled warning
   is logged).
4. With `enable_h_centering:=false` the behaviour is identical to the original
   (no H subscription influence on reach logic).

| Parameter                | Type  | Default                               | Description |
|--------------------------|-------|---------------------------------------|-------------|
| `h_det_topic`            | string | `/landing_pad/h_relative_position`   | Topic for YOLO H-marker detections (`geometry_msgs/PointStamped`). |
| `enable_h_centering`     | bool  | `true`                                | Enable H-marker final centering. Set to `false` to restore original behaviour. |
| `centering_start_dist_m` | float | `0.6`                                 | World-frame distance to active base below which centering mode activates. |
| `centering_reach_tol_m`  | float | `0.05`                                | Required XY distance to the H-marker world target to consider the base reached (when centering is active). |
| `h_timeout_s`            | float | `0.5`                                 | Maximum age of an H detection for it to be considered fresh. |
| `max_h_range_m`          | float | `6.0`                                 | Maximum body-frame range of an H detection; detections beyond this are ignored. |

Example — enable centering with tighter tolerance:

```bash
ros2 run yolo_pad_pose pad_waypoint_supervisor --ros-args \
  -p enable_h_centering:=true \
  -p centering_start_dist_m:=0.6 \
  -p centering_reach_tol_m:=0.05 \
  -p h_timeout_s:=0.5
```

Example — disable centering (original behaviour):

```bash
ros2 run yolo_pad_pose pad_waypoint_supervisor --ros-args \
  -p enable_h_centering:=false
```

### Outlier rejection and robustness parameters

These parameters prevent phantom / noisy detections from polluting the
candidate list and ensure the node does not revisit already-visited bases.

| Parameter               | Type  | Default | Description |
|-------------------------|-------|---------|-------------|
| `max_detection_range_m` | float | `6.0`   | Reject incoming detections whose body-frame range `hypot(right, front)` exceeds this value. Recommended: set to the maximum reliable sensor range (typically **6 m**). |
| `max_jump_m`            | float | `2.0`   | Reject a detection if its body-frame position jumps more than this distance from the last accepted detection. Filters sudden outlier spikes. |
| `jump_reject_reset_count` | int | `10`   | After this many **consecutive** jump-rejections the filter resets its anchor and accepts the next detection as the new reference point. This prevents the node from getting permanently stuck rejecting a legitimately shifted detection stream (e.g. after the drone flies over a different pad). Set to `0` to disable the auto-reset (original behaviour). |
| `repeat_block_m`        | float | `1.2`   | When choosing the next target, skip any candidate whose world-frame position is within this radius of any **already-visited** odom position. Prevents re-visiting the same physical base. |
| `candidate_timeout_s`   | float | `120.0` | Drop **unvisited** candidates that have not been seen within this many seconds. Visited candidates are never dropped. Set to `0.0` to disable pruning. |

Recommended values for a 6-base indoor arena (bases spaced ≥ 1.5 m apart):

```bash
ros2 run yolo_pad_pose pad_waypoint_supervisor --ros-args \
  -p max_detection_range_m:=6.0 \
  -p max_jump_m:=2.0 \
  -p jump_reject_reset_count:=10 \
  -p repeat_block_m:=1.2 \
  -p candidate_timeout_s:=120.0
```

### Parameters (axis / yaw correction)

All flags default to **False** — the math is correct for the standard
`x=right, y=front` detection convention.  Change these only if your
detector or odometry source uses a different sign convention.

| Parameter            | Type | Default | Description |
|----------------------|------|---------|-------------|
| `invert_right_axis`  | bool | `false` | Flip the sign of the incoming right (x) value before projection. Set to `true` if your detector publishes x=left. |
| `invert_front_axis`  | bool | `false` | Flip the sign of the incoming front (y) value before projection. Set to `true` if your detector publishes y=backward. |
| `invert_yaw`         | bool | `false` | Negate the odom yaw before projection. Set to `true` if odom yaw appears mirrored (e.g. NED vs ENU). |
| `target_offset_right_m` | float | `0.0` | Constant offset added to right after inversion (calibration). |
| `target_offset_front_m` | float | `0.0` | Constant offset added to front after inversion (calibration). |

### world_frame_id auto-detection

The `world_frame_id` parameter (default `"map"`) sets the `frame_id` of the
published `PoseArray` waypoints.  When the parameter equals `"map"` (the
default), the supervisor automatically overrides it with the
`header.frame_id` of the **first odometry message** it receives (typically
`"uav1/map"` in MAVROS setups).  To disable this behaviour, explicitly set
`world_frame_id` to the desired frame name via `--ros-args`.

### Debug logging

Set the ROS 2 logger level to DEBUG to see per-detection traces:

```bash
ros2 run yolo_pad_pose pad_waypoint_supervisor --ros-args \
  --log-level pad_waypoint_supervisor:=debug
```

Each accepted detection logs:

```
[base] det ok: raw=(x,y)  right=R front=F  yaw=Y rad  bx=BX by=BY  ...
```

and a finer `DEBUG`-level line shows the full projection pipeline:

```
[base] projection: right=R front=F yaw=Y rad  dx=DX dy=DY  cur=(CX,CY)  bx=BX by=BY
```

---

## odom_tf_broadcaster

### Why is this needed?

MAVROS publishes odometry on `/uav1/mavros/local_position/odom`
(`nav_msgs/Odometry`) with `header.frame_id = uav1/map` and
`child_frame_id = uav1/base_link`, but it **does not** write the corresponding
transform to `/tf`.  This causes the TF tree to be disconnected:

```
$ ros2 run tf2_ros tf2_echo uav1/map uav1/base_link
# → "frame does not exist" / "two or more unconnected trees"
```

`odom_tf_broadcaster` subscribes to the Odometry topic and re-publishes the
pose as a `geometry_msgs/TransformStamped` on `/tf` so the tree is connected.

### Running the node

```bash
ros2 run yolo_pad_pose odom_tf_broadcaster
```

Or with a custom odometry topic:

```bash
ros2 run yolo_pad_pose odom_tf_broadcaster --ros-args \
  -p odom_topic:=/uav1/mavros/local_position/odom
```

### Parameters

| Parameter                 | Type   | Default                                  | Description                                              |
|---------------------------|--------|------------------------------------------|----------------------------------------------------------|
| `odom_topic`              | string | `/uav1/mavros/local_position/odom`       | Odometry topic to subscribe to                           |
| `tf_parent_frame_override`| string | `""` (use `msg.header.frame_id`)         | Override the TF parent frame id                          |
| `tf_child_frame_override` | string | `""` (use `msg.child_frame_id`)          | Override the TF child frame id (falls back to `base_link`)|
| `use_odom_header_stamp`   | bool   | `true`                                   | Use the Odometry message stamp; if `false`, use `now()`  |

### Verifying the TF is published

After starting the node, run:

```bash
ros2 run tf2_ros tf2_echo uav1/map uav1/base_link
```

You should see transforms printing continuously without any
"frame does not exist" or "two or more unconnected trees" errors.

### Running alongside pad_waypoint_supervisor

A launch file is provided:

```bash
ros2 launch yolo_pad_pose odom_tf_broadcaster.launch.py
```

This starts both `odom_tf_broadcaster` and `pad_waypoint_supervisor` together.
