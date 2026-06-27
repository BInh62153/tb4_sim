# TurtleBot4 Simulation Stack

A complete **ROS2 Humble** simulation stack for **TurtleBot4** running on **Docker** with **Ignition Gazebo Fortress**. This project provides mapping, autonomous navigation, waypoint patrol, task management, recovery behaviors, and an interactive command-line interface.

---

## Features

* Autonomous navigation using **Nav2**
* SLAM mapping using **slam_toolbox**
* Waypoint navigation and patrol
* Behavior Tree based navigation
* Modular Task Manager architecture
* Battery monitoring
* Automatic docking support
* Recovery behaviors (retry, replanning, costmap clearing)
* Interactive CLI
* RViz visualization
* Docker-based deployment
* Unit testing for core modules

---

# System Overview

```text
                         User
                           │
                           ▼
                Interactive CLI / Scripts
                           │
                           ▼
                  Mission Planner
                           │
                           ▼
                    Task Executor
                           │
                           ▼
                    State Machine
                           │
        ┌──────────────────┼──────────────────┐
        ▼                  ▼                  ▼
 Navigation Manager   Battery Manager   Dock Manager
        │                                      │
        └───────────────┬──────────────────────┘
                        ▼
                 Recovery Manager
                        │
                        ▼
                     ROS2 Layer
        ┌───────────────┼─────────────────┐
        ▼               ▼                 ▼
      Nav2       SLAM Toolbox      CycloneDDS
                        │
                        ▼
                  TurtleBot4 Robot
```

---

# Architecture

```
Application Layer
────────────────────────────────────
MissionPlanner
TaskExecutor
LifecycleNode

Business Logic Layer
────────────────────────────────────
StateMachine
BatteryManager
DockManager
RecoveryManager

Infrastructure Layer
────────────────────────────────────
NavigationManager
ROS2 Topics
ROS2 Services
ROS2 Actions

External Layer
────────────────────────────────────
Nav2
SLAM Toolbox
Ignition Gazebo
CycloneDDS
TurtleBot4
```

---

# Repository Structure

```text
.
├── config/
│   ├── behavior_trees/
│   ├── cyclonedds/
│   ├── nav2/
│   ├── slam/
│   └── waypoints.yaml
│
├── docker/
│   ├── Dockerfile.sim
│   └── entrypoint.sh
│
├── maps/
│
├── scripts/
│   ├── turtlebot4_headless.launch.py
│   ├── send_waypoints.py
│   └── tb4_cli.py
│
├── src/
│   └── task_manager/
│       ├── managers/
│       ├── state/
│       ├── tests/
│       └── lifecycle_node.py
│
├── docker-compose.yml
├── tb4sim.sh
└── README.md
```

---

# Task Manager Architecture

```
MissionPlanner
      │
      ▼
TaskExecutor
      │
      ▼
StateMachine
      │
      ├────────► NavigationManager
      │               │
      │               ▼
      │            Nav2 Actions
      │
      ├────────► BatteryManager
      │
      ├────────► DockManager
      │
      └────────► RecoveryManager
```

---

# State Machine

```
                +--------+
                |  Idle  |
                +--------+
                     │
                     ▼
              +-------------+
              | Navigate    |
              +-------------+
               │         │
      Success  │         │ Failed
               ▼         ▼
         Execute Task  Recovery
               │         │
               └────┬────┘
                    ▼
             Battery Low?
              │         │
             Yes        No
              ▼         ▼
          Dock Robot   Next Mission
              │
              ▼
          Charging
              │
              ▼
             Idle
```

---

# Workflow

```
Launch Docker

↓

Start Gazebo

↓

Start SLAM

↓

Create Map

↓

Save Map

↓

Localization

↓

Load Waypoints

↓

Mission Planner

↓

Navigate

↓

Execute Task

↓

Recovery (if needed)

↓

Dock

↓

Repeat
```

---

# ROS2 Communication

## Topics

| Topic            | Purpose               |
| ---------------- | --------------------- |
| `/scan`          | LiDAR data            |
| `/odom`          | Robot odometry        |
| `/tf`            | Coordinate transforms |
| `/battery_state` | Battery monitoring    |
| `/map`           | Occupancy grid        |

---

## Actions

| Action          | Purpose            |
| --------------- | ------------------ |
| NavigateToPose  | Navigate to a goal |
| FollowWaypoints | Patrol             |
| Dock            | Automatic docking  |

---

## Services

| Service          | Purpose                   |
| ---------------- | ------------------------- |
| SaveMap          | Save SLAM map             |
| ClearCostmap     | Clear navigation costmaps |
| LifecycleManager | Node lifecycle control    |

---

# Quick Start

## 1. Install Docker (Arch Linux)

```bash
sudo pacman -S docker docker-compose

sudo systemctl enable --now docker

sudo usermod -aG docker $USER

newgrp docker
```

---

## 2. Enable X11

```bash
xhost +local:docker
```

Hyprland:

```text
exec-once = xhost +local:docker
```

---

## 3. Build

```bash
chmod +x tb4sim.sh

./tb4sim.sh build
```

---

## 4. Run

Navigation Stack

```bash
./tb4sim.sh nav
```

Full System

```bash
./tb4sim.sh full
```

RViz

```bash
./tb4sim.sh rviz
```

CLI

```bash
./tb4sim.sh cli
```

---

# CLI Commands

```text
waypoints

goto kitchen

patrol

pause

resume

cancel

save_map office

status

battery

dock

undock
```

---

# Waypoint Configuration

```yaml
waypoints:

  kitchen:

    label: Kitchen

    pose:

      x: 2.5

      y: -1.0

      yaw: 1.57

    tasks:

      - type: wait

        duration: 5

      - type: rotate

        angle: 3.14159

      - type: log

        message: Finished
```

Patrol sequence

```yaml
patrol_sequence:

  - kitchen

  - office

  - charging_station

loop_patrol: true
```

Restart Task Manager

```bash
docker restart tb4_task_manager
```

---

# Docker Profiles

| Profile | Description            |
| ------- | ---------------------- |
| sim     | Gazebo only            |
| nav     | Navigation stack       |
| full    | Full autonomous system |
| rviz    | Visualization          |

---

# Performance

| Component  | Technology               |
| ---------- | ------------------------ |
| ROS2       | Humble                   |
| Simulator  | Ignition Gazebo Fortress |
| Navigation | Nav2                     |
| Planner    | Smac Planner             |
| Controller | MPPI Controller          |
| SLAM       | slam_toolbox             |
| DDS        | CycloneDDS               |
| Language   | Python 3.10              |

---

# Testing

Run unit tests

```bash
pytest src/task_manager/task_manager/tests
```

Covered modules

* MissionPlanner
* NavigationManager
* RecoveryManager
* DockManager
* BatteryManager
* StateMachine
* TaskExecutor
* StructuredLogger

---

# Troubleshooting

## Gazebo GUI does not open

```bash
echo $DISPLAY

xhost +local:docker
```

---

## Nav2 failed

```bash
./tb4sim.sh logs nav

grep ERROR
```

---

## No LiDAR

```bash
ros2 topic echo /scan --once
```

---

## DDS issue

```bash
echo $RMW_IMPLEMENTATION
```

Expected

```text
rmw_cyclonedds_cpp
```

---

# Roadmap

* ✓ Docker deployment
* ✓ Nav2 navigation
* ✓ SLAM mapping
* ✓ Waypoint patrol
* ✓ Recovery behaviors
* ✓ Battery monitoring
* ✓ Automatic docking
* ✓ Unit testing
* ✘ Multi-robot support
* ✘ Fleet management
* ✘ Camera perception
* ✘ Web dashboard
* ✘ Dynamic task allocation

---

# Future Improvements

* Behavior Tree editor
* Mission scheduler
* Web UI
* Cloud monitoring
* Multi-floor navigation
* Semantic mapping
* Voice control

---

# License

MIT License