# drone_control

ROS 2 C++ package with drone mission and supervision nodes.

## Nodes

### `supervisor`

Monitors trajectory progress and, once the trajectory finishes, launches
`mission_manager` exactly **once**.  Also handles return-to-home and landing.

```bash
ros2 run drone_control supervisor
```

### `supervisor_T`

Monitors trajectory progress and, **each time** a trajectory finishes, waits
`wait_after_traj_done_s` seconds (default **5 s**) and then launches
`missao_P_T`.  After the trajectory is detected as complete the guards are
automatically reset, so the node fires again for every subsequent trajectory.

```bash
ros2 run drone_control supervisor_T
# custom delay and same-spot guard:
ros2 run drone_control supervisor_T --ros-args \
  -p wait_after_traj_done_s:=5.0 \
  -p min_relaunch_dist_m:=0.5
# stricter origin landing (requires 3 s of stability within 0.15 m):
ros2 run drone_control supervisor_T --ros-args \
  -p base_tol_m:=0.15 \
  -p base_hold_s:=3.0 \
  -p pouso_xy_hold_tol:=0.05 \
  -p pouso_xy_hold_stable_s:=2.0 \
  -p pouso_xy_abort_tol:=0.3
```

**Topics monitored:**

| Topic                   | Type                 | Description                              |
|-------------------------|----------------------|------------------------------------------|
| `/trajectory_progress`  | `std_msgs/Float32`   | Progress value; ≥ 100 → trajectory done; < 99.9 → new trajectory started |
| `/trajectory_finished`  | `std_msgs/Bool`      | `true` → done; `false` → new trajectory |

**Parameters:**

| Parameter                 | Type   | Default | Description                                        |
|---------------------------|--------|---------|----------------------------------------------------|
| `uav_name`                | string | `uav1`  | UAV namespace prefix                               |
| `use_origin_as_base`      | bool   | `true`  | Land at current XY when stably near origin instead of running missao_P_T |
| `wait_after_traj_done_s`  | double | `5.0`   | Seconds to wait after trajectory completion before launching next mission |
| `min_relaunch_dist_m`     | double | `0.5`   | Minimum XY distance (m) the drone must have moved from the last mission launch position before a new mission is started; applies to both `missao_P_T` and `pouso` (local) launches; if closer, the mission is skipped (set to `0.0` to disable) |
| `base_tol_m`              | double | `0.20`  | Radial tolerance (m) used to decide whether the UAV is at the origin; drone must be within `hypot(x,y) ≤ base_tol_m` |
| `base_hold_s`             | double | `2.0`   | The UAV must remain within `base_tol_m` of the origin **continuously** for this many seconds before the local pouso is triggered; set to `0.0` to trigger immediately without a hold period |
| `pouso_xy_hold_tol`       | double | `0.10`  | `xy_hold_tol` forwarded to `pouso` when landing at the origin (m) |
| `pouso_xy_hold_stable_s`  | double | `1.0`   | `xy_hold_stable_s` forwarded to `pouso` when landing at the origin (s) |
| `pouso_xy_abort_tol`      | double | `0.5`   | `xy_abort_tol` forwarded to `pouso` when landing at the origin (m) |
| `pouso_approach_z`        | double | `-1.0`  | `approach_z` forwarded to `pouso` when landing at the origin (m); `-1.0` means use current odom Z |

**Behaviour:**

1. Subscribes to `/trajectory_progress` and `/trajectory_finished`.
2. When either signal indicates the trajectory is complete, the supervisor
   enters the `WAIT_BEFORE_MISSION` state and logs
   `🏁 Trajetória concluída. Aguardando X.X s antes de iniciar nova missão…`.
3. After `wait_after_traj_done_s` seconds, it checks that the drone has moved
   at least `min_relaunch_dist_m` from the last mission launch position; if
   not, the mission is **skipped** (logged as `⛔ Missão ignorada`) and the
   supervisor returns to `WAIT_TRAJ` to prevent landing twice at the same spot.
   This guard applies to both `missao_P_T` and `pouso` (local) launches.
4. When `use_origin_as_base=true` (default), the supervisor checks whether the
   drone is stably near the origin:
   - The drone's odometry is tracked continuously; when
     `hypot(x, y) ≤ base_tol_m` the supervisor starts a hold timer.
   - If the drone leaves the zone, the hold timer resets; while accumulating,
     a throttled log `🏠 UAV na zona base (dist=X.XXX m). Estabilidade: X.X s / X.X s…`
     is emitted each second.
   - Once `base_hold_s` continuous seconds inside the zone have elapsed,
     `pouso` is launched with `use_current_xy:=true` and the extra parameters
     (`pouso_xy_hold_tol`, `pouso_xy_hold_stable_s`, `pouso_xy_abort_tol`,
     `pouso_approach_z`).
   - If the drone is not at the origin (or `use_origin_as_base=false`), `missao_P_T`
     is launched instead.
5. Only **one** instance of `missao_P_T` / `pouso` runs at a time; extra
   triggers are queued and processed in order.
6. When a new trajectory starts (`/trajectory_finished == false` or
   `progress < 100`) the guards reset so the next completion triggers a new
   launch.
7. Child processes are reaped via `waitpid(WNOHANG)` to avoid zombies.

**Log messages to expect:**

| Message | Meaning |
|---------|---------|
| `🏠 UAV entrou na zona base (dist=X.XXX m ≤ tol=X.XX m). Aguardando estabilidade de X.X s…` | Drone entered origin zone; hold timer started |
| `🏠 UAV na zona base (dist=X.XXX m). Estabilidade: X.X s / X.X s…` | Hold timer accumulating (throttled, 1 s) |
| `🏠 UAV na zona base há X.X s (dist=X.XXX m ≤ tol=X.XX m) — pouso local autorizado.` | Hold period complete; local pouso will be triggered |
| `⏱️  X.X s concluídos. UAV no ponto base (x=X.XX y=X.XX) — lançando pouso com posição local (…)` | Launching local pouso with all parameters logged |
| `⛔ Posição atual (X.XX, X.XX) está a X.XXm da última missão…` | Relaunch guard fired; mission skipped |

### `missao_P_T`

Orchestrator that runs the following sequence:

1. `ros2 run drone_control pouso` — land the drone.
2. Wait **5 seconds**.
3. `ros2 run drone_control takeoff` — take off again.

```bash
ros2 run drone_control missao_P_T
```

### `pouso`

Navigates the drone to a target XY position and lands using a two-phase FSM
that avoids oscillation caused by mid-descent waypoint republishing.

**Landing phases:**

1. **CENTER** — publishes a single hover waypoint at the target XY and a
   configurable approach altitude.  The node stays in this phase until
   `dxy ≤ xy_hold_tol` continuously for `xy_hold_stable_s` seconds (prevents
   transient pass-through from triggering an early descent).
2. **DESCEND** — publishes a single descent waypoint to `landing_z`.
   Waypoints are **not** reissued on minor XY drift during descent.  If drift
   exceeds `xy_abort_tol` the node reverts to CENTER so the drone climbs back
   to `approach_z` and recentres before retrying.

```bash
ros2 run drone_control pouso
# custom tolerances
ros2 run drone_control pouso --ros-args \
  -p xy_hold_tol:=0.15 \
  -p xy_abort_tol:=0.4 \
  -p xy_hold_stable_s:=1.5 \
  -p approach_z:=1.5
```

**Parameters:**

| Parameter         | Default | Description |
|-------------------|---------|-------------|
| `uav_name`        | `uav1`  | UAV namespace prefix |
| `x`               | `0.0`   | Target landing X (ENU, m) — ignored when `use_current_xy=true` |
| `y`               | `0.0`   | Target landing Y (ENU, m) — ignored when `use_current_xy=true` |
| `use_current_xy`  | `true`  | Use live odom XY as target instead of `x`/`y` |
| `landing_z`       | `0.05`  | Final ground altitude (m) |
| `frame_id`        | `map`   | Coordinate frame |
| `rate_hz`         | `10.0`  | Timer rate (Hz) |
| `check_after_sec` | `15.0`  | Overall timeout (s) before giving up |
| `xy_hold_tol`     | `0.10`  | Max planar error (m) to be considered centred |
| `xy_hold_stable_s`| `1.0`   | Seconds to remain within `xy_hold_tol` before starting descent |
| `xy_abort_tol`    | `0.5`   | Abort descent and recentre if XY error exceeds this (m) |
| `approach_z`      | `-1.0`  | Hover altitude for CENTER phase (m); `-1` = use current odom Z |
| `use_yolo_h`      | `false` | Use YOLO-detected H marker as landing target |
| `h_topic`         | `/landing_pad/h_relative_position` | YOLO H detection topic |
| `h_collect_time_s`| `1.0`   | Seconds to collect H detections before selecting best |
| `h_timeout_s`     | `0.75`  | Max age (s) for H detection window entries |
| `max_h_range_m`   | `6.0`   | Max range (m) to accept a YOLO H detection |
| `prefer_closest_h`| `true`  | Select H closest to drone; `false` = latest |
| `h_filter_type`   | `"none"`| Filter applied to collected right/front samples: `"none"`, `"mean"`, `"median"` |
| `h_filter_window` | `5`     | Max number of recent samples kept in the filter buffer (≥1) |
| `h_filter_min_samples` | `3` | Minimum samples required to apply the filter; falls back to `best_collected_h_` if fewer |

**Log messages to expect:**

| Message | Meaning |
|---------|---------|
| `🎯 Entrando em CENTER: dxy=X.XX m …` | Entering CENTER phase (also on abort/recentre) |
| `🎯 CENTER: dxy=X.XX m …` | Periodic status during centering |
| `✅ Centrado: dxy=… por Y.Ys. Iniciando descida…` | Stable centering achieved; transitioning to DESCEND |
| `⬇️  DESCEND: Z=…` | Periodic status during descent |
| `⚠️  Deriva excessiva durante descida: dxy=… > xy_abort_tol=…` | Aborting descent; returning to CENTER |
| `✅ Pouso concluído: …` | Landing complete; node exits |
| `[yolo_h] Filtro 'mean'/'median' aplicado (n=N): right X→Y  front X→Y` | Filter was applied; shows raw vs filtered values |
| `[yolo_h] Filtro '…': amostras insuficientes (N < M). Usando best_collected_h_…` | Not enough samples; fell back to best detection (DEBUG level) |

### `takeoff`

Commands the drone to take off.

```bash
ros2 run drone_control takeoff
```

## Typical workflow

```bash
# Terminal 1 – start supervisor_T (waits for trajectory events)
ros2 run drone_control supervisor_T

# Terminal 2 – send a trajectory and observe supervisor_T automatically
#              launch missao_P_T when the trajectory finishes
ros2 topic pub /trajectory_finished std_msgs/msg/Bool "data: true" --once
```

## Validation / minimal tests

Because the package currently has no automated test suite, validate the node
manually:

1. **Single trajectory:**

   ```bash
   # Start supervisor_T in one terminal
   ros2 run drone_control supervisor_T

   # Simulate trajectory start then finish in another terminal
   ros2 topic pub /trajectory_progress std_msgs/msg/Float32 "data: 50.0" --once
   ros2 topic pub /trajectory_finished  std_msgs/msg/Bool   "data: true"  --once
   ```

   Expected: `supervisor_T` logs `🏁 Trajetória concluída` and then
   `🚀 [SUPERVISOR_T] Lançando missao_P_T…`.

2. **Repeated trajectories (reset guard):**

   ```bash
   # After the first missao_P_T finishes, simulate a new trajectory
   ros2 topic pub /trajectory_finished std_msgs/msg/Bool "data: false" --once
   ros2 topic pub /trajectory_finished std_msgs/msg/Bool "data: true"  --once
   ```

   Expected: `supervisor_T` logs `▶️  Nova trajetória detectada` and then
   launches `missao_P_T` a second time.

3. **Origin landing with hold period:**

   ```bash
   # Start supervisor_T with short hold period for quick testing
   ros2 run drone_control supervisor_T --ros-args \
     -p base_tol_m:=0.20 \
     -p base_hold_s:=2.0

   # Simulate drone near origin via odometry
   ros2 topic pub /uav1/mavros/local_position/odom \
     nav_msgs/msg/Odometry \
     "{pose: {pose: {position: {x: 0.05, y: 0.05, z: 1.0}}}}" --once

   # Trigger trajectory completion
   ros2 topic pub /trajectory_finished std_msgs/msg/Bool "data: true" --once
   ```

   Expected:
   - `🏠 UAV entrou na zona base (dist=0.071 m ≤ tol=0.20 m). Aguardando estabilidade de 2.0 s…`
   - After `wait_after_traj_done_s` elapses: `🏠 UAV na zona base (dist=0.071 m). Estabilidade: X.X s / 2.0 s…`
   - After 2 s stability: `🏠 UAV na zona base há 2.X s … — pouso local autorizado.`
   - Then: `⏱️  5.0 s concluídos. UAV no ponto base (x=0.05 y=0.05) — lançando pouso com posição local (xy_hold_tol=0.10, …)…`

4. **Build check:**

   ```bash
   cd <workspace>
   colcon build --packages-select drone_control
   ros2 run drone_control supervisor_T --help   # should print usage / exit 0
   ```
