# yolo_pad_pose

ROS 2 package containing YOLO-based landing pad detection and the pad
waypoint supervisor.

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
