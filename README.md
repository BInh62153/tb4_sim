# TurtleBot4 Simulation Stack (tb4_sim_v2)

A full **TurtleBot4** simulation stack on **ROS2 Humble**, running via **Docker Compose**, using **Ignition Gazebo (Fortress)** as the simulator. Includes SLAM, Nav2, multiple runtime-selectable controllers (DWA / TEB / Pure Pursuit), frontier exploration, an event-driven Task Manager lifecycle node, and an interactive CLI for controlling the robot.

---

## Features

* **Ignition Gazebo Fortress** simulation, running headless via **Xvfb + VirtualGL** (NVIDIA GPU acceleration by default, with a hardened CPU/`llvmpipe` fallback), server/GUI split
* SLAM mapping with **slam_toolbox**
* Autonomous navigation with **Nav2**, with **runtime controller selection** via a single latched `/controller_selector` topic (DWA / TEB / Pure Pursuit — Stanley currently disabled, see [Roadmap](#roadmap))
* **Frontier Exploration** (`m-explore-ros2` / `explore_lite`) — automatic map exploration, using the *same* `/controller_selector` mechanism as manual navigation (no C++ patch needed)
* **Task Manager**: lifecycle node, event-driven architecture, cleanly split into managers (mission, navigation, recovery, battery, dock, task executor, explore). **Manual-first control**: stack starts configured but inactive; operator runs `activate` in the CLI before sending any motion command — robot does not move on its own when RViz opens
* Centralized state machine with an explicit transition table (11 states, every transition validated)
* **Two-tier recovery**: Nav2 BT clears costmaps only; `RecoveryManager` (Python) runs the full 6-level pipeline (wait → clear costmaps → spin → backup → replan/abort) with structured logs — avoids duplicate backup commands that trigger Create3's backup limit
* Battery monitoring + automatic docking/charging when low; auto-resumes **patrol** after charging (manual `goto` does not resume a mission loop)
* Interactive CLI: lifecycle control (`activate`/`deactivate`) + mission commands over `/tb4/cmd` + `/tb4/status`, with tab autocomplete
* Multi-profile Docker Compose (`sim` / `nav` / `full` / `rviz`), per-service healthchecks, per-service CPU limits
* Tuned for real-time performance on modest GPUs/CPUs — see [Performance Tuning](#performance-tuning)
* Unit tests for all managers + the state machine (pytest)

---
[![Demo](https://youtube.com)](https://www.youtube.com/watch?v=bq-OAB5vu88)


## System Overview

```text
                              User (CLI: tb4_cli.py)
                                       │
                          publish String → /tb4/cmd
                                       ▼
                     ┌─────────────────────────────────────┐
                     │   TurtleBot4LifecycleNode (task_manager)  │
                     │        (thin orchestrator, holds no        │
                     │         business logic itself)              │
                     └─────────────────────────────────────┘
                                       │
                                       ▼
                              StateMachine (Event dispatch)
                                       │
        ┌───────────────┬─────────────┼──────────────┬───────────────┐
        ▼                ▼             ▼              ▼               ▼
 MissionPlanner   NavigationManager  BatteryManager  DockManager  ExploreManager
        │                │                                │            │
        │                ▼                                │            ▼
        │        Nav2 (NavigateToPose)              (nav + battery)  explore_lite
        │        + /controller_selector (latched)                    (subprocess)
        │                │                                            │
        │                └───────────────┬────────────────────────────┘
        │                                 ▼
        │                    /controller_selector (shared, TRANSIENT_LOCAL)
        │
        └──────► RecoveryManager (6-level recovery)
                         │
                         ▼
                     ROS2 Layer
        ┌────────────────┼────────────────┐
        ▼                ▼                ▼
      Nav2         SLAM Toolbox      CycloneDDS
                         │
                         ▼
              Ignition Gazebo (TurtleBot4 sim)
```

---

## Task Manager Architecture

`task_manager` is a **LifecycleNode** acting as a thin orchestrator: all business logic lives in `managers/`, and every state change goes through `StateMachine` (no shortcut sets state directly).

```text
lifecycle_node.py  (TurtleBot4LifecycleNode)
   │
   ├── state/
   │     ├── states.py           SystemState enum + VALID_TRANSITIONS table
   │     ├── state_machine.py    StateMachine + Event enum, register()/dispatch()
   │     └── structured_logger.py Structured JSON logging (mission_id, goal_id, waypoint)
   │
   └── managers/
         ├── mission_planner.py     loads waypoints.yaml, tracks patrol progress
         ├── navigation_manager.py  sends NavigateToPose, selects controller via /controller_selector
         ├── recovery_manager.py    6-level recovery (wait/clear costmap/spin/backup/replan)
         ├── battery_manager.py     monitors /battery_state, emits BATTERY_LOW/CRITICAL/CHARGE_COMPLETE
         ├── dock_manager.py        orchestrates docking + waiting for charge + resume
         ├── task_executor.py       runs tasks at a waypoint (wait/rotate/scan/log)
         └── explore_manager.py     starts/stops/pauses explore_lite, syncs controller via /controller_selector
```

### Controller (algorithm) selection mechanism

The CLI lets you pick a driving algorithm per `goto` / `patrol` / `explore` command:

| CLI alias | Nav2 controller plugin                            | Status                  |
| --------- | ------------------------------------------------- | ------------------------ |
| `dwa`     | `FollowPathDWA` (`dwb_core::DWBLocalPlanner`)      | Default                  |
| `teb`     | `FollowPathTEB` (`teb_local_planner::TebLocalPlannerROS`) | Built from source (`src/teb_local_planner`) |
| `pp`      | `FollowPathPP` (`nav2_regulated_pure_pursuit_controller`) | Working                  |
| `stanley` | `FollowPathStanley`                                | **Disabled** — `nav2_stanley_controller` not built yet, see Roadmap |

Both `NavigationManager` and `ExploreManager` publish the controller name to the same `/controller_selector` topic before sending a `NavigateToPose` goal — this is the topic the BT `ControllerSelector` node (declared in `navigate_to_pose.xml` / `navigate_through_poses.xml`) already reads for **any** client, including goals sent by `explore_lite` itself. No custom `set_parameters` service call and no patch to `explore.cpp` are needed anymore.

The publisher uses a **latched QoS** (`TRANSIENT_LOCAL`, depth 1): a late-joining subscriber (e.g. the `ControllerSelector` BT node, which is recreated by Nav2 on every goal) still receives the last published value immediately, without the node having to block waiting for `get_subscription_count() > 0`. This replaced an old `time.sleep()` polling loop that stalled the whole single-threaded executor for up to 0.5 s on every `goto`/`patrol` command (the cause of the CLI feeling laggy).

The BT XML's `default_controller` was also fixed from the non-existent `"FollowPath"` to `"FollowPathDWA"`, so navigation still works correctly even before the first `/controller_selector` message arrives.

---

## State Machine

11 states, transitions validated against the explicit table in `state/states.py`:

```text
UNCONFIGURED ─► IDLE ──┬─► NAVIGATING ──┬─► EXECUTING_TASK ──► IDLE (mission loop)
                        │                ├─► RECOVERING ──┬─► IDLE
                        │                │                 └─► ABORTED
                        │                └─► LOW_BATTERY_DOCKING
                        │
                        ├─► EXPLORING ──┬─► PAUSED
                        │                ├─► LOW_BATTERY_DOCKING
                        │                └─► ABORTED
                        │
                        └─► PAUSED ──┬─► IDLE
                                     ├─► EXPLORING
                                     └─► ABORTED

LOW_BATTERY_DOCKING ─► CHARGING ─► RESUME_AFTER_CHARGE ─► IDLE
ABORTED ─► IDLE | UNCONFIGURED
```

Every operator command from the CLI is routed through an `Event`: `CMD_GOTO`, `CMD_PATROL`, `CMD_EXPLORE`, `CMD_PAUSE`, `CMD_RESUME`, `CMD_STOP`. When `pause` is issued while in `EXPLORING`, the node remembers `_paused_from = 'explore'` so `resume` knows to return to exploring instead of the mission patrol.

**Mission modes** (`_mission_mode` in `lifecycle_node.py`):

| Mode | Trigger | Behavior after goal/tasks finish |
| ---- | ------- | -------------------------------- |
| `None` | Stack up / `stop` / `goto` done | Robot stays idle — waits for next command |
| `patrol` | CLI `patrol` | Advances `patrol_sequence` and loops (if `loop_patrol: true`) |
| `goto` | CLI `goto` | Returns to IDLE once — does **not** continue patrol |
| `explore` | CLI `explore` | Runs frontier exploration until `stop`/`pause` |

On container start the task manager is **configured only** (lifecycle state `inactive`). `activate` (via CLI) enables `/tb4/cmd` subscription and battery monitoring but does **not** auto-start patrol.

---

## Repository Structure

```text
.
├── config/
│   ├── behavior_trees/        navigate_to_pose.xml, navigate_through_poses.xml
│   ├── cyclonedds/            cyclonedds.xml
│   ├── nav2/                  nav2_params.yaml (controller/planner/costmap/BT/recovery)
│   ├── rviz/                  tb4_view.rviz (custom RViz layout, OAK-D image + scan + costmaps)
│   ├── slam/                  slam_toolbox_params.yaml
│   └── waypoints.yaml         waypoints + patrol_sequence + emergency_rules
│
├── docker/
│   ├── Dockerfile.sim         osrf/ros:humble-desktop-full + VirtualGL + builds teb/explore/task_manager
│   └── entrypoint.sh          Xvfb + VirtualGL bootstrap, sources the workspace
│
├── maps/                      saved maps (.yaml/.pgm) + debug TF frame graph
│
├── scripts/
│   ├── turtlebot4_headless.launch.py   launches Gazebo via gz_sim.launch.py (-s server-only)
│   ├── vgl_launch.sh                   vglrun wrapper for the simulator, with CPU-fallback hardening
│   ├── set_motion_safety.sh            sets Create3 motion_control safety_override=backup_only at sim start
│   ├── send_waypoints.py
│   └── tb4_cli.py                      interactive CLI (activate/patrol/goto/explore/stop/pause/resume/status)
│
├── src/
│   ├── task_manager/           main package — see details above
│   ├── teb_local_planner/      built from source (no official Humble binary)
│   ├── m-explore-ros2/         explore/ (explore_lite) + map_merge/
│   └── costmap_converter/      required dependency of teb_local_planner
│
├── .logs/                      runtime debug logs (mounted into every container as /ros2_ws/.logs)
├── docker-compose.yml
├── tb4sim.sh                   main launcher (build/run/logs/shell/watchdog/save_map)
└── README.md
```

---

## Docker Compose — services & profiles

| Profile | Services started                                          |
| ------- | ------------------------------------------------------------ |
| `sim`   | `simulator`                                                   |
| `nav`   | `simulator` → `slam` → `navigation`, `cli`                    |
| `full`  | `simulator` → `slam` → `navigation` → `task_manager`, `cli`   |
| `rviz`  | `gazebo_gui` + `rviz` (host X, run alongside another profile) |

Startup order relies on `depends_on: condition: service_healthy` — each service only starts once the previous one is healthy (e.g. `slam` waits for `simulator` to have `/scan`, `/tf` (via `tf2_echo odom base_link`, not a random `/tf` sample), and sim clock ≥ 5s before starting).

The `task_manager` service auto-runs `ros2 lifecycle set … configure` only. **Activate** is left to the operator via CLI (`activate` / `start`) so the robot stays still until explicitly commanded.

The `simulator` service runs `set_motion_safety.sh` in the background to set Create3 `motion_control` `safety_override=backup_only`, allowing Nav2/`RecoveryManager` backup actions without hitting the default backup distance limit.

**Rendering modes:**
* `simulator`: headless inside Xvfb (`DISPLAY=:99`). By default runs **VirtualGL** (`USE_VIRTUALGL=true`, `vglrun`, `egl0` backend) to use the real NVIDIA GPU for the Gazebo server + camera sensors (the OAK-D camera still spawns an OGRE render thread even with `-s`). `vgl_launch.sh` probes `vglrun glxinfo` before launch and automatically falls back to `LIBGL_ALWAYS_SOFTWARE=1` (software rendering) if the EGL/GLX context can't be created. Set `USE_VIRTUALGL=false` to force the CPU path directly — `vgl_launch.sh` then explicitly exports `LIBGL_ALWAYS_SOFTWARE=1` + `GALLIUM_DRIVER=llvmpipe` so Mesa never silently falls back to *indirect* GLX over the X11 socket (indirect GLX serializes every GL call and used to tank real-time factor by 100-1000x).
* `slam` / `navigation` / `task_manager` / `cli`: fully headless, **do not** start Xvfb (`START_XVFB=false`).
* `gazebo_gui` / `rviz`: use the **real host X server** (`DISPLAY` from the host, `xhost +local:docker`), no Xvfb spawned. **Do not run `gazebo_gui` and `rviz` at the same time** on a 4GB GPU — just use `./tb4sim.sh rviz` (Gazebo keeps running headless on the server, no GUI client needed). `rviz` loads the custom `config/rviz/tb4_view.rviz` layout (edit it on the host — the file is bind-mounted, no rebuild needed).

---

## Performance Tuning

Everything below is override-able via `.env` or an exported shell variable before `./tb4sim.sh ...`:

| Variable | Default | Purpose |
| --- | --- | --- |
| `ROS_LOG_LEVEL` | `WARN` | rclcpp/rclpy logging threshold. Every node logging at `INFO` across Gazebo/SLAM/Nav2/bridges adds up to real CPU cost — only raise it when actively debugging. |
| `USE_VIRTUALGL` | `true` | `true` = GPU render through VirtualGL/EGL; `false` = forced CPU/`llvmpipe` (see rendering modes above). |
| `RENDER_ENGINE` | `ogre` | Gazebo render engine. `ogre` (Ogre1) is much lighter under CPU/Xvfb rendering; switch to `ogre2` only once `USE_VIRTUALGL=true` and the GPU/EGL path is confirmed stable. |
| `XVFB_RESOLUTION` | `960x540x24` | Virtual framebuffer size — nobody views it directly, so it's kept small to cut OGRE's per-frame cost. |
| `SIM_CPU_LIMIT` | `12` | CPU core cap for the `simulator` service (heaviest). |
| `SLAM_CPU_LIMIT` | `3` | CPU core cap for `slam`. |
| `NAV_CPU_LIMIT` | `4` | CPU core cap for `navigation`. |
| `MOTION_SAFETY_OVERRIDE` | `backup_only` | Create3 `motion_control.safety_override` set by `set_motion_safety.sh` at sim start. Use `none` (default Create3) or `full` only when debugging safety behavior. |

Adjust the `*_CPU_LIMIT` values to match `nproc` on your machine. `deploy.resources.limits` requires **Docker Compose v2** (`docker compose`, not the legacy `docker-compose` v1 binary) to take effect outside Swarm mode — check with `docker compose version`.

### Chasing a low real-time factor (RTF)

If `simulator` reports a real-time factor well below 1.0 (symptoms: `cmd_vel` feels laggy, SLAM/Nav2 fall behind), the fixes already baked into this repo, roughly in the order they matter:

1. **Indirect GLX fallback** — `vgl_launch.sh` now forces `LIBGL_ALWAYS_SOFTWARE=1` explicitly on the CPU path. Without it, Mesa can silently negotiate *indirect* GLX against Xvfb (no real DRI device inside the virtual X server) and serialize every GL call over the X11 socket — CPU usage looks low precisely because most time is spent waiting on round-trips, not computing. This was the single biggest RTF killer (dropped RTF to ~0.04 before the fix).
2. **Cliff + IR intensity sensors** — the Create3 base ships 4 cliff sensors + 7 IR intensity sensors, all raycasting at their real-hardware rate of 62 Hz. Under `llvmpipe` (CPU rendering), 11 extra raycast sensors competing for the same render context is expensive and isn't needed by any of Nav2/SLAM/task_manager. `docker/Dockerfile.sim` patches both xacro files to drop their `update_rate` to 5 Hz (RPLIDAR is left untouched — Nav2/SLAM need its full rate).
3. **DWB debug output** — `debug_trajectory_details: true` makes the DWA controller serialize every candidate trajectory (vx × vtheta samples) to a debug topic each cycle. Now `false` by default.
4. **DWB trajectory sample count** — `vx_samples`/`vtheta_samples` reduced `20×20 → 10×10` (400 → 100 candidates/cycle), still smooth enough for a low-speed differential robot (`max_vel_x: 0.31`).
5. **SmacPlannerHybrid** — costmap downsampling enabled (`downsample_costmap: true`, factor 2), `angle_quantization_bins` halved (72 → 36), `max_iterations`/`max_on_approach_iterations` lowered to sane ceilings, and `cache_obstacle_heuristic: true` (the heuristic map is now computed once and reused instead of recomputed on every replan).
6. **Path smoothers** (`SmacPlannerHybrid.smoother` and `smoother_server.simple_smoother`) — `tolerance` relaxed from `1e-10` (which almost never converges early) to `1e-6`, and `max_iterations`/`max_its` lowered from 1000 to 200 to match.

If RTF is still low after all of the above, check `ros2 topic hz /scan` and `docker stats` while the stack is running — that will show whether the LiDAR raycast itself (dense warehouse mesh + `llvmpipe`) or another sensor/node is now the bottleneck.

---

## ROS2 Communication

### Topics

| Topic                  | Direction            | Purpose                                                 |
| ----------------------- | --------------------- | ---------------------------------------------------------- |
| `/tb4/cmd`              | CLI → task_manager     | Operator commands (`goto:wp:algo`, `goto_pos:x:y:z:yaw:algo`, `patrol:algo`, `explore:algo`, `stop`, `pause`, `resume`) |
| `/tb4/status`           | task_manager → CLI      | 1Hz heartbeat: state + mission summary + battery           |
| `/controller_selector`  | task_manager → Nav2 BT  | Runtime controller plugin selection (`FollowPathDWA`/`TEB`/`PP`), latched (`TRANSIENT_LOCAL`), published by both `NavigationManager` and `ExploreManager` |
| `/explore/resume`       | task_manager → explore_lite | Toggle frontier search on/off (pause doesn't kill the process) |
| `/scan`                 | —                      | LiDAR                                                       |
| `/odom`, `/tf`          | —                      | Odometry / transforms                                      |
| `/battery_state`        | —                      | Battery status                                              |
| `/map`                  | —                      | Occupancy grid from SLAM                                    |

### Actions

| Action           | Used by                |
| ----------------- | ------------------------ |
| `NavigateToPose`  | NavigationManager         |
| `Spin`            | RecoveryManager           |
| `BackUp`          | RecoveryManager           |

### Services

| Service                                        | Used by             |
| ----------------------------------------------- | --------------------- |
| `/local_costmap/clear_entirely_local_costmap`   | RecoveryManager        |
| `/global_costmap/clear_entirely_global_costmap` | RecoveryManager        |

---

## Quick Start

### 1. Install Docker (Arch Linux)

```bash
sudo pacman -S docker docker-compose
sudo systemctl enable --now docker
sudo usermod -aG docker $USER
newgrp docker
```

### 2. Enable X11 (only needed for `rviz` / `gazebo_gui`)

```bash
xhost +local:docker
```

Hyprland — add this to your config to run it automatically on login:

```text
exec-once = xhost +local:docker
```

### 3. Build the image

```bash
chmod +x tb4sim.sh
./tb4sim.sh build
```

### 4. Run

```bash
./tb4sim.sh sim       # Gazebo only
./tb4sim.sh nav        # Gazebo + SLAM + Nav2
./tb4sim.sh full       # full stack (+ task_manager, configured but inactive)
./tb4sim.sh rviz       # RViz2 on host X (run alongside another profile)
./tb4sim.sh cli        # open the control CLI
```

### 5. Recommended control flow (`full` profile)

Opening RViz does **not** start the robot. After the stack is healthy:

```bash
./tb4sim.sh cli
```

```text
> activate              # lifecycle activate — required before any motion command
> status                # state=IDLE, task_manager active
> goto diem_C dwa       # one-shot navigation; stops when done (no auto-patrol)
> patrol dwa            # start full patrol loop from waypoints.yaml
> stop                  # halt and clear mission mode
> deactivate            # lifecycle deactivate (optional shutdown)
```

Verify lifecycle state from any container:

```bash
ros2 lifecycle get /task_manager_lifecycle_node   # inactive [2] until activate → active [3]
ros2 param get /motion_control safety_override    # backup_only (after sim is up)
```

---

## `tb4sim.sh` — management commands

```text
build            Build the Docker image
sim / nav / full  Start the corresponding profile
rviz             Add RViz2 (host X, don't run together with gazebo_gui)
cli              Open the interactive CLI
stop             Stop all containers
clean            Remove containers + volumes (asks for confirmation)
status           docker compose ps
logs [service]   Tail logs (defaults to all services)
shell <service>  Shell into a container (sim/nav/task/rviz)
save_map [name]  Save the current map from slam_toolbox → maps/
watchdog         Loop that auto-restarts unhealthy containers (60s cooldown)
```

---

## CLI (`tb4_cli.py`)

Communicates over `/tb4/cmd` (publish) and `/tb4/status` (subscribe), with Tab autocomplete for commands and waypoint names. Lifecycle transitions (`activate`/`deactivate`) call `ros2 lifecycle set` directly — they do not go through `/tb4/cmd`.

```text
activate / start       Lifecycle activate task_manager (required before patrol/goto/explore)
deactivate             Lifecycle deactivate task_manager
goto <target> [algo]   Move once to a waypoint from waypoints.yaml
goto x y [z] [yaw] [algo]   Move once to map coordinates (yaw in radians, default 0)
patrol [algo]          Start automatic patrol following patrol_sequence in waypoints.yaml
explore [algo]         Enable Frontier Exploration (automatic map exploration)
stop                   Stop the robot + cancel any running task + clear mission mode
pause                  Pause (remembers whether it was mission or explore)
resume                 Resume whichever flow was paused
status                 Lifecycle state + robot heartbeat (works before/after activate)
help                   Show the command menu
clear                  Clear the screen
exit                   Exit the CLI

supported algo values: dwa | teb | pp | stanley (stanley currently falls back to dwa — controller not built yet)
```

Examples: `activate`, `status`, `goto diem_B teb`, `goto 3.5 3.5 0 1.57 dwa`, `patrol pp`.

---

## Waypoint Configuration (`config/waypoints.yaml`)

```yaml
patrol_sequence: ["diem_A", "diem_B", "diem_C", "diem_D"]
loop_patrol: true
default_wait_duration: 2.0

waypoints:
  diem_A:
    label: "Point A - Entrance"
    pose: { x: 1.0, y: 0.5, yaw: 0.0 }
    tasks:
      - type: wait
        duration: 3.0
      - type: rotate
        angle: 1.5707
      - type: log
        message: "Arrived at Point A"

emergency_rules:
  low_battery_threshold: 20        # %
  low_battery_action: "tram_sac"   # waypoint name used as the charging dock
```

Supported task types: `wait`, `rotate`, `scan` (camera capture/scan on a given topic), `log`.

After editing the file, restart the task manager to reload the config:

```bash
docker restart tb4_task_manager
# then in CLI: activate  (container only auto-configures on start)
```

---

## Recovery Behaviors

Recovery is split across two layers to avoid duplicate spin/backup commands (which caused Create3 `Reached backup limit!` and stuck navigation):

| Layer | File | What it does |
| ----- | ---- | ------------ |
| Nav2 BT | `config/behavior_trees/navigate_to_pose.xml` | On nav failure: clear local + global costmap only |
| Task Manager | `recovery_manager.py` | Full 6-level pipeline after Nav2 goal fails |

`RecoveryManager` runs through 6 levels, stopping at the first one that succeeds, and `abort`s (skipping the current waypoint during **patrol**) if all 6 fail:

1. Wait 2 s
2. Clear local costmap
3. Clear global costmap
4. Spin 90°
5. Backup 0.2 m (requires `safety_override=backup_only` on Create3 `motion_control` — set automatically by `set_motion_safety.sh`)
6. Replan (retry the nav goal) → skip waypoint / stop (patrol vs manual `goto`)

During **manual `goto`**, recovery retries the same waypoint; on final abort the robot returns to IDLE without starting patrol.

---

## Performance / Tech Stack

| Component        | Technology                                      |
| ------------------ | -------------------------------------------------- |
| ROS2               | Humble (base image `osrf/ros:humble-desktop-full`) |
| Simulator          | Ignition Gazebo Fortress (`ros-gz`, gz_version 6)   |
| Headless rendering | Xvfb + VirtualGL (`vglrun`, EGL backend) with hardened `llvmpipe` fallback |
| Navigation         | Nav2 (multi-plugin controller_server: DWA/TEB/PP)    |
| Global planner     | Smac Planner Hybrid (downsampled costmap, cached obstacle heuristic) |
| SLAM               | slam_toolbox (online_async)                          |
| Exploration        | m-explore-ros2 (`explore_lite`)                      |
| DDS                | CycloneDDS (`rmw_cyclonedds_cpp`)                     |
| Language           | Python 3.10 (task_manager) / C++17 (teb, costmap_converter, explore) |

---

## Testing

```bash
pytest src/task_manager/task_manager/tests
```

Modules with test coverage:

* MissionPlanner
* NavigationManager
* RecoveryManager
* DockManager
* BatteryManager
* ExploreManager
* StateMachine
* TaskExecutor
* StructuredLogger

---

## Troubleshooting

**Gazebo GUI doesn't open**

```bash
echo $DISPLAY
xhost +local:docker
```

**Simulator stuck unhealthy / hangs during `start_period`**

The healthcheck requires `/scan` + `/tf` (odom→base_link via `tf2_echo`) + sim clock ≥ 5s + scan with more than 50 range points — usually because the world (warehouse) hasn't finished spawning yet. Check:

```bash
./tb4sim.sh logs simulator
```

**Low real-time factor / robot feels laggy**

See [Performance Tuning](#performance-tuning) above. Quick checks:

```bash
docker stats                                   # is a service pinned at its CPU limit?
ros2 topic hz /scan                            # is LiDAR publishing at the expected rate?
```

Confirm `USE_VIRTUALGL`/`RENDER_ENGINE` currently in effect for the running container:

```bash
docker exec tb4_simulator printenv USE_VIRTUALGL RENDER_ENGINE
```

**Nav2 errors**

```bash
./tb4sim.sh logs navigation | grep ERROR
```

**Robot stuck during navigation / `Reached backup limit!` in simulator logs**

Nav2 BT and `RecoveryManager` both used to command backup; Create3 defaults to `safety_override=none`. Confirm the sim startup script applied the override:

```bash
ros2 param get /motion_control safety_override   # expect: backup_only
./tb4sim.sh logs simulator | grep set_motion_safety
```

**Robot moves as soon as RViz opens (unexpected auto-patrol)**

The task manager should start **inactive**. If it auto-patrols, check whether something else is calling `ros2 lifecycle set … activate` or sending `/tb4/cmd`. Expected flow: `./tb4sim.sh cli` → `activate` → `patrol` or `goto`.

```bash
ros2 lifecycle get /task_manager_lifecycle_node
```

**No LiDAR**

```bash
docker exec -it tb4_navigation bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /scan --once"
```

**Check DDS**

```bash
docker exec -it tb4_navigation printenv RMW_IMPLEMENTATION
# Expected: rmw_cyclonedds_cpp
```

**Container keeps restarting as unhealthy**

```bash
./tb4sim.sh watchdog
```

---

## Roadmap

* ✓ Docker deployment (multi-profile, dependency-ordered healthchecks, per-service CPU limits)
* ✓ Nav2 navigation, multiple runtime controllers (DWA/TEB/PP)
* ✓ SLAM mapping
* ✓ Waypoint patrol + task execution at each waypoint
* ✓ Recovery behaviors (6 levels, BT costmap-only + RecoveryManager spin/backup)
* ✓ Manual-first operator control (configure on start, CLI `activate`, `goto` one-shot vs `patrol` loop)
* ✓ Create3 backup safety override for sim recovery (`set_motion_safety.sh`)
* ✓ Battery monitoring + auto-dock/charge
* ✓ Frontier exploration (explore_lite) + controller sync via shared `/controller_selector` topic
* ✓ Unit testing (managers + state machine)
* ✓ Headless GPU rendering (Xvfb + VirtualGL) with hardened CPU/`llvmpipe` fallback
* ✓ Real-time factor tuning (sensor update rates, DWB/Smac planner cost cuts, latched controller-selector QoS)
* ✘ `nav2_stanley_controller` — build it and re-enable `FollowPathStanley`
* ✘ Multi-robot support / fleet management
* ✘ Camera perception (OAK-D pipeline)
* ✘ Web dashboard (React) wired directly into the stack
* ✘ Multi-robot map merge (`m-explore-ros2/map_merge` already present in `src/`, not yet wired into compose)

---

## License

MIT License