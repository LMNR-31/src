# drone_ws

ROS 2 workspace for autonomous drone control using an LPV-MPC (Linear Parameter-Varying Model Predictive Controller) with MAVROS / PX4.

---

## Architecture Overview

Two main control nodes work together:

| Node | Role | Publishes |
|------|------|-----------|
| `hover_supervisor_node` | Persistent hover: arms the drone, takes off, and holds position indefinitely | `/{uav}/mavros/setpoint_position/local` (PoseStamped) |
| `lpv_mpc_drone_node` | Full-flight controller: attitude MPC, trajectory execution | `/{uav}/mavros/setpoint_raw/attitude` (AttitudeTarget) |

Only **one controller should publish setpoints at a time**. The two nodes perform an automatic handoff:

1. **hover_supervisor_node** starts first, arms the drone, takes off, and holds position.
2. **lpv_mpc_drone_node** starts in `WAIT_WP`. As soon as it detects the drone is already armed+OFFBOARD (via `auto_arm_handoff=true`), it latches the current position and enters `HOVER`, publishing attitude setpoints.
3. **hover_supervisor_node** detects attitude setpoints from the MPC (via `auto_disable_when_mpc_active=true`) and **stops publishing** position setpoints automatically.
4. The **MPC is now fully in control**. Send trajectories via `/waypoints`.
5. When the MPC is no longer publishing (e.g. stopped), hover_supervisor **re-enables** itself (`auto_reenable=true`) to safely hold position.

---

## Topics

### Subscribed by `lpv_mpc_drone_node`

| Topic | Type | Description |
|-------|------|-------------|
| `/{uav}/mavros/local_position/odom` | `nav_msgs/Odometry` | ENU position + orientation (frame `uav1/map`) |
| `/{uav}/mavros/local_position/velocity_body` | `geometry_msgs/TwistStamped` | Body-frame velocity (preferred) |
| `/{uav}/mavros/state` | `mavros_msgs/State` | Armed/OFFBOARD state from FCU |
| `/waypoints` | `geometry_msgs/PoseArray` | Trajectory: 1 pose = takeoff/hover, 2+ poses = trajectory |
| `/waypoint_goal` | `geometry_msgs/PoseStamped` | Single goal update (cached; use `/waypoints` to trigger flight) |

### Published by `lpv_mpc_drone_node`

| Topic | Type | Description |
|-------|------|-------------|
| `/{uav}/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | Attitude + thrust commands to FCU |

### Services on `hover_supervisor_node`

| Service | Type | Description |
|---------|------|-------------|
| `~/takeoff` | `std_srvs/Trigger` | Latch XY from odom, climb to `takeoff_altitude`, hold |
| `~/latch_here` | `std_srvs/Trigger` | Latch current XYZ from odom immediately and hold |
| `~/set_enabled` | `std_srvs/SetBool` | `true` = publish setpoints; `false` = stop (yield to MPC) |

---

## Frame Convention

- All position data uses **ENU** (East-North-Up, z positive when airborne).
- MAVROS `local_position/odom` reports `frame_id='uav1/map'` — treated identically to `map` by MAVROS.
- `hover_supervisor_node` publishes setpoints with `frame_id="map"`.
- `lpv_mpc_drone_node` reads position directly from odom (z = ENU altitude).

---

## Running the Stack

### Simulation (tmux session)

```bash
cd mrs_uav_gazebo_simulator/tmux/one_drone
tmuxinator start -p session.yml
```

This starts (in order): Zenoh router → Gazebo → PX4 API → status monitor → RViz → LPV-MPC controller → hover supervisor → debug console.

### Manual Handoff Commands

**Disable supervisor (let MPC control the drone):**
```bash
ros2 service call /hover_supervisor_node/set_enabled std_srvs/srv/SetBool "{data: false}"
```

**Re-enable supervisor (resume hover hold):**
```bash
ros2 service call /hover_supervisor_node/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /hover_supervisor_node/latch_here std_srvs/srv/Trigger "{}"
```

### Sending Waypoints to the MPC

**Single hover target (takeoff/hover at x=5, y=3, z=2.5 m):**
```bash
ros2 topic pub --once /waypoints geometry_msgs/msg/PoseArray "
header:
  frame_id: 'map'
poses:
- position: {x: 5.0, y: 3.0, z: 2.5}
  orientation: {w: 1.0}"
```

**Trajectory (2+ waypoints):**
```bash
ros2 topic pub --once /waypoints geometry_msgs/msg/PoseArray "
header:
  frame_id: 'map'
poses:
- position: {x: 2.0, y: 0.0, z: 2.0}
  orientation: {w: 1.0}
- position: {x: 5.0, y: 3.0, z: 2.5}
  orientation: {w: 1.0}
- position: {x: 0.0, y: 0.0, z: 1.5}
  orientation: {w: 1.0}"
```

> **Note:** `/waypoints` with **1 pose** triggers takeoff/hover; **2+ poses** activate trajectory mode (only when MPC is already in HOVER state). The `z` value is positive altitude in metres (ENU).

---

## `pouso` — Publish a Landing Point Anywhere on the Map

The `pouso` node (Portuguese for *landing*) lets you command the drone to land at
**any arbitrary XY position** on the map.  It publishes a two-waypoint descent
sequence to `/waypoints`, which is consumed directly by `my_drone_controller`:

| Waypoint | Description |
|----------|-------------|
| WP 0 | Approach hover — same altitude as the drone's current Z, above the target XY |
| WP 1 | Ground — target XY at `landing_z` (default 0.05 m) |

### Quick start

**Land at the drone's current XY position (default behaviour):**
```bash
ros2 run drone_control pouso
```

**Land at a specific map coordinate (x=3.0, y=−2.0):**
```bash
ros2 run drone_control pouso --ros-args \
  -p use_current_xy:=false \
  -p x:=3.0 \
  -p y:=-2.0
```

**Full parameter set:**
```bash
ros2 run drone_control pouso --ros-args \
  -p uav_name:=uav1 \
  -p use_current_xy:=false \
  -p x:=5.0 \
  -p y:=2.5 \
  -p landing_z:=0.05 \
  -p frame_id:=map \
  -p check_after_sec:=20.0
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `uav_name` | `uav1` | UAV namespace prefix (used for MAVROS topics) |
| `use_current_xy` | `true` | Use live odometry XY; set `false` to specify `x`/`y` |
| `x` | `0.0` | Target landing X coordinate (ENU, m) — used when `use_current_xy=false` |
| `y` | `0.0` | Target landing Y coordinate (ENU, m) — used when `use_current_xy=false` |
| `landing_z` | `0.05` | Final ground altitude (m) |
| `frame_id` | `map` | Coordinate frame for the published `PoseArray` |
| `check_after_sec` | `15.0` | Timeout (s) before the node shuts down if landing is not confirmed |
| `rate_hz` | `10.0` | Timer rate (Hz) |

### Topics

| Topic | Direction | Type | Description |
|-------|-----------|------|-------------|
| `/waypoints` | Published | `geometry_msgs/PoseArray` | Landing waypoints consumed by `my_drone_controller` |
| `/{uav}/mavros/local_position/odom` | Subscribed | `nav_msgs/Odometry` | Current drone position |
| `/{uav}/mavros/state` | Subscribed | `mavros_msgs/State` | FCU connection status |

---

## `missao_P_T` — Land then Take Off (Pouso → 5 s → Takeoff)

The `missao_P_T` node (Portuguese: *mission P-T*) orchestrates a full
**land-and-relaunch** sequence:

1. **pouso** — commands the drone to land at the current (or specified) XY
   position via `my_drone_controller`.
2. **Wait 5 seconds** — allows time for the drone to disarm / settle.
3. **takeoff** — takes the drone back to its configured altitude.

If `pouso` fails (non-zero exit code), the node logs an error and **does not
proceed** with `takeoff` (fail-safe).

### Quick start

```bash
ros2 run drone_control missao_P_T
```

### Behaviour

| Phase | Action |
|-------|--------|
| FASE 1 | Runs `ros2 run drone_control pouso` and waits for it to complete |
| FASE 2 | Waits exactly **5 seconds** |
| FASE 3 | Runs `ros2 run drone_control takeoff` (only if FASE 1 succeeded) |

### Notes

- `missao_P_T` runs `pouso` and `takeoff` with their **default parameters**. To
  customise landing position, altitude, UAV name, etc., run those nodes
  individually using their `--ros-args` options; use `missao_P_T` for the
  automated default-parameter sequence.
- The node exits automatically after the sequence completes.

---

## Key Parameters

### `lpv_mpc_drone_node`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `uav_name` | `uav1` | UAV namespace prefix |
| `auto_arm_handoff` | `true` | Auto-latch position and enter HOVER when drone is already armed+OFFBOARD (safe handoff from hover_supervisor) |
| `hover_altitude` | `2.0` | Default takeoff altitude (m) |
| `max_ref_speed_xy` | `0.5` | Max XY reference speed (m/s) |
| `landing_mode` | `1` | `0` = standby on ground, `1` = DISARM after landing |

### `hover_supervisor_node`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `uav_name` | `uav1` | UAV namespace prefix |
| `takeoff_altitude` | `2.0` | Target altitude for `~/takeoff` service (m) |
| `auto_latch_airborne_on_start` | `true` | Auto-detect airborne + OFFBOARD+ARM on startup and latch position |
| `auto_disable_when_mpc_active` | `false` | Stop publishing when attitude setpoints detected from MPC |
| `handoff_timeout` | `0.5` | Time (s) without attitude setpoints before MPC is considered inactive |
| `auto_reenable` | `false` | Re-enable supervisor automatically when MPC stops publishing |

---

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select drone_custom_control
source install/setup.bash
```

---

## Logs

Node logs are saved under `/tmp/drone_logs/`. To follow live:
```bash
tail -f /tmp/drone_logs/hover_supervisor_node_*.log
```
# drone_control
# src
# src
