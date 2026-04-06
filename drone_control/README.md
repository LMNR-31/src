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
```

**Topics monitored:**

| Topic                   | Type                 | Description                              |
|-------------------------|----------------------|------------------------------------------|
| `/trajectory_progress`  | `std_msgs/Float32`   | Progress value; ≥ 100 → trajectory done; < 99.9 → new trajectory started |
| `/trajectory_finished`  | `std_msgs/Bool`      | `true` → done; `false` → new trajectory |

**Parameters:**

| Parameter                | Type   | Default | Description                                         |
|--------------------------|--------|---------|-----------------------------------------------------|
| `uav_name`               | string | `uav1`  | UAV namespace prefix                                |
| `use_origin_as_base`     | bool   | `true`  | Land at current XY when near origin instead of running missao_P_T |
| `wait_after_traj_done_s` | double | `5.0`   | Seconds to wait after trajectory completion before launching next mission |
| `min_relaunch_dist_m`    | double | `0.5`   | Minimum XY distance (m) the drone must have moved from the last mission launch position before a new mission is started; if closer, the mission is skipped to avoid landing twice in a row at the same spot (set to `0.0` to disable) |

**Behaviour:**

1. Subscribes to `/trajectory_progress` and `/trajectory_finished`.
2. When either signal indicates the trajectory is complete, the supervisor
   enters the `WAIT_BEFORE_MISSION` state and logs
   `🏁 Trajetória concluída. Aguardando X.X s antes de iniciar nova missão…`.
3. After `wait_after_traj_done_s` seconds, it checks that the drone has moved
   at least `min_relaunch_dist_m` from the last mission launch position; if
   not, the mission is **skipped** (logged as `⛔ Missão ignorada`) and the
   supervisor returns to `WAIT_TRAJ` to prevent landing twice at the same spot.
4. If the position check passes, it logs
   `⏱️  X.X s concluídos — lançando missao_P_T…` and launches the mission.
5. Only **one** instance of `missao_P_T` runs at a time; extra triggers are
   queued and processed in order.
6. When a new trajectory starts (`/trajectory_finished == false` or
   `progress < 100`) the guards reset so the next completion triggers a new
   launch.
7. Child processes are reaped via `waitpid(WNOHANG)` to avoid zombies.

### `missao_P_T`

Orchestrator that runs the following sequence:

1. `ros2 run drone_control pouso` — land the drone.
2. Wait **5 seconds**.
3. `ros2 run drone_control takeoff` — take off again.

```bash
ros2 run drone_control missao_P_T
```

### `pouso`

Publishes a landing waypoint at the current (or specified) XY position on the
map.

```bash
ros2 run drone_control pouso
```

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

3. **Build check:**

   ```bash
   cd <workspace>
   colcon build --packages-select drone_control
   ros2 run drone_control supervisor_T --help   # should print usage / exit 0
   ```
