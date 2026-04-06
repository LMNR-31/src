# yolo_pad_pose

ROS 2 package containing YOLO-based landing pad detection and the pad
waypoint supervisor.

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
