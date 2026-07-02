# TurtleBot4 Simulation Stack (tb4_sim_v2)

A full **TurtleBot4** simulation stack on **ROS2 Humble**, running via **Docker Compose**, using **Ignition Gazebo (Fortress)** as the simulator. Includes SLAM, Nav2, multiple runtime-selectable controllers (DWA / TEB / Pure Pursuit), frontier exploration, an event-driven Task Manager lifecycle node, and an interactive CLI for controlling the robot.

---

## Features

* **Ignition Gazebo Fortress** simulation, running headless via **Xvfb + VirtualGL** (NVIDIA GPU acceleration), with server/GUI split
* SLAM mapping with **slam_toolbox**
* Autonomous navigation with **Nav2**, with **runtime controller selection** via `/controller_selector` (DWA / TEB / Pure Pursuit — Stanley currently disabled, see [Roadmap](#roadmap))
* **Frontier Exploration** (`m-explore-ros2` / `explore_lite`) — automatic map exploration, synced with the same controller algorithm as navigation
* **Task Manager**: lifecycle node, event-driven architecture, cleanly split into managers (mission, navigation, recovery, battery, dock, task executor, explore)
* Centralized state machine with an explicit transition table (11 states, every transition validated)
* Nav2-standard 6-level recovery (wait → clear local costmap → clear global costmap → spin → backup → replan/abort)
* Battery monitoring + automatic docking/charging when low, auto-resumes mission after charging
* Interactive CLI over `/tb4/cmd` + `/tb4/status`, with tab autocomplete
* Multi-profile Docker Compose (`sim` / `nav` / `full` / `rviz`), per-service healthchecks
* Unit tests for all managers + the state machine (pytest)

---

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
        │        + /controller_selector                              (subprocess)
        │                │
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
         └── explore_manager.py     starts/stops/pauses explore_lite, syncs controller_id
```

### Controller (algorithm) selection mechanism

The CLI lets you pick a driving algorithm per `goto` / `patrol` / `explore` command:

| CLI alias | Nav2 controller plugin                            | Status                  |
| --------- | ------------------------------------------------- | ------------------------ |
| `dwa`     | `FollowPathDWA` (`dwb_core::DWBLocalPlanner`)      | Default                  |
| `teb`     | `FollowPathTEB` (`teb_local_planner::TebLocalPlannerROS`) | Built from source (`src/teb_local_planner`) |
| `pp`      | `FollowPathPP` (`nav2_regulated_pure_pursuit_controller`) | Working                  |
| `stanley` | `FollowPathStanley`                                | **Disabled** — `nav2_stanley_controller` not built yet, see Roadmap |

`NavigationManager` publishes the controller name to `/controller_selector` before sending each `NavigateToPose` goal. `ExploreManager` does the same for `explore_node` via the `set_parameters` service (requires a small patch in `explore.cpp` so the node reads the `controller_id` parameter — see the docstring in `explore_manager.py`).

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

---

## Repository Structure

```text
.
├── config/
│   ├── behavior_trees/        navigate_to_pose.xml, navigate_through_poses.xml
│   ├── cyclonedds/            cyclonedds.xml
│   ├── nav2/                  nav2_params.yaml (controller/planner/costmap/BT/recovery)
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
│   ├── vgl_launch.sh                   vglrun wrapper for the simulator
│   ├── send_waypoints.py
│   └── tb4_cli.py                      interactive CLI (goto/patrol/explore/stop/pause/resume/status)
│
├── src/
│   ├── task_manager/           main package — see details above
│   ├── teb_local_planner/      built from source (no official Humble binary)
│   ├── m-explore-ros2/         explore/ (explore_lite) + map_merge/
│   └── costmap_converter/      required dependency of teb_local_planner
│
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

Startup order relies on `depends_on: condition: service_healthy` — each service only starts once the previous one is healthy (e.g. `slam` waits for `simulator` to have `/scan`, `/tf`, and sim clock ≥ 5s before starting).

**Rendering modes:**
* `simulator`: runs headless inside Xvfb (`DISPLAY=:99`) + **VirtualGL** (`vglrun`, `egl0` backend) to use the real NVIDIA GPU for the Gazebo server + camera sensors (the OAK-D camera still spawns an OGRE render thread even with `-s`).
* `slam` / `navigation` / `task_manager` / `cli`: fully headless, **do not** start Xvfb (`START_XVFB=false`).
* `gazebo_gui` / `rviz`: use the **real host X server** (`DISPLAY` from the host, `xhost +local:docker`), no Xvfb spawned. **Do not run `gazebo_gui` and `rviz` at the same time** on a 4GB GPU — just use `./tb4sim.sh rviz` (Gazebo keeps running headless on the server, no GUI client needed).

---

## ROS2 Communication

### Topics

| Topic                  | Direction            | Purpose                                                 |
| ----------------------- | --------------------- | ---------------------------------------------------------- |
| `/tb4/cmd`              | CLI → task_manager     | Operator commands (`goto:wp:algo`, `patrol:algo`, `explore:algo`, `stop`, `pause`, `resume`) |
| `/tb4/status`           | task_manager → CLI      | 1Hz heartbeat: state + mission summary + battery           |
| `/controller_selector`  | task_manager → Nav2     | Runtime controller plugin selection (`FollowPathDWA`/`TEB`/`PP`) |
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

### Services

| Service                                        | Used by             |
| ----------------------------------------------- | --------------------- |
| `/local_costmap/clear_entirely_local_costmap`   | RecoveryManager        |
| `/global_costmap/clear_entirely_global_costmap` | RecoveryManager        |
| `/explore_node/set_parameters`                  | ExploreManager (changes `controller_id` at runtime) |

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
./tb4sim.sh full       # full stack (+ task_manager)
./tb4sim.sh rviz       # RViz2 on host X (run alongside another profile)
./tb4sim.sh cli        # open the control CLI
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

Communicates over `/tb4/cmd` (publish) and `/tb4/status` (subscribe), with Tab autocomplete for commands and waypoint names.

```text
goto <target> [algo]   Move to a waypoint using the given algorithm (default dwa)
patrol [algo]           Start automatic patrol following patrol_sequence in waypoints.yaml
explore [algo]           Enable Frontier Exploration (automatic map exploration)
stop                     Stop the robot + cancel any running task
pause                    Pause (remembers whether it was mission or explore)
resume                   Resume whichever flow was paused
status                   Show current status (state, mission, battery)
help                     Show the command menu
clear                    Clear the screen
exit                     Exit the CLI

supported algo values: dwa | teb | pp | stanley (stanley currently falls back to dwa — controller not built yet)
```

Examples: `goto diem_B teb`, `patrol pp`, `explore dwa`.

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
```

---

## Recovery Behaviors

`RecoveryManager` runs through 6 levels, stopping at the first one that succeeds, and `abort`s (skipping the current waypoint) if all 6 fail:

1. Wait
2. Clear local costmap
3. Clear global costmap
4. Spin 90°
5. Backup 0.2 m
6. Replan (retry the nav goal) → `ABORTED` if it still fails

---

## Performance / Tech Stack

| Component        | Technology                                      |
| ------------------ | -------------------------------------------------- |
| ROS2               | Humble (base image `osrf/ros:humble-desktop-full`) |
| Simulator          | Ignition Gazebo Fortress (`ros-gz`, gz_version 6)   |
| Headless rendering | Xvfb + VirtualGL (`vglrun`, EGL backend)             |
| Navigation         | Nav2 (multi-plugin controller_server: DWA/TEB/PP)    |
| Global planner     | Smac Planner Hybrid                                  |
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

The healthcheck requires `/scan` + `/tf` (odom) + sim clock ≥ 5s + scan with more than 50 range points — usually because the world (warehouse) hasn't finished spawning yet. Check:

```bash
./tb4sim.sh logs simulator
```

**Nav2 errors**

```bash
./tb4sim.sh logs navigation | grep ERROR
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

* ✓ Docker deployment (multi-profile, dependency-ordered healthchecks)
* ✓ Nav2 navigation, multiple runtime controllers (DWA/TEB/PP)
* ✓ SLAM mapping
* ✓ Waypoint patrol + task execution at each waypoint
* ✓ Recovery behaviors (6 levels)
* ✓ Battery monitoring + auto-dock/charge
* ✓ Frontier exploration (explore_lite) + controller sync
* ✓ Unit testing (managers + state machine)
* ✓ Headless GPU rendering (Xvfb + VirtualGL)
* ✘ `nav2_stanley_controller` — build it and re-enable `FollowPathStanley`
* ✘ Multi-robot support / fleet management
* ✘ Camera perception (OAK-D pipeline)
* ✘ Web dashboard (React) wired directly into the stack
* ✘ Multi-robot map merge (`m-explore-ros2/map_merge` already present in `src/`, not yet wired into compose)

---

## License

MIT License