# NeuPAN ROS 2 Real Robot Integration Guide

<div align="center">

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/index.html)
[![License](https://img.shields.io/badge/License-GPL_v3.0-green.svg)](https://www.gnu.org/licenses/gpl-3.0.html)
[![Python](https://img.shields.io/badge/Python-3.10+-yellow.svg)](https://www.python.org/)

**A comprehensive framework for deploying NeuPAN (Neural Path Planner) on real robots.**
Integrates state-of-the-art **Neural Planning**, **Point-LIO SLAM**, and **Real-time Navigation**.

[Features](#-key-features) •
[Prerequisites](#-prerequisites--environment-setup) •
[Installation](#-installation--build) •
[Deployment](#-real-robot-deployment) •
[Troubleshooting](#-troubleshooting)

</div>

---

## 🌟 Key Features

*   **🧠 Neural Path Planning**: Real-time, dynamic obstacle avoidance using NeuPAN.
*   **📍 Robust SLAM**: Integrated **Point-LIO** with auto-time synchronization for drift-free localization.
*   **🔄 Seamless Integration**: Bridge between 3D Point Clouds and 2D Navigation stacks.
*   **🤖 Robot Ready**: Pre-configured for Livox LiDARs (Mid-360) and omni/diff drive robots.

## 📦 Package Overview

This workspace is a cohesive ecosystem of three packages:

| Package | Path | Description |
| :--- | :--- | :--- |
| **`neupan_ros`** | `neupan_ros2/` | Core wrapper for NeuPAN. Handles planning, collision avoidance, and velocity command generation. |
| **`point_lio`** | `point_lio/` | Robust Lidar-Inertial Odometry. **Patched** for automatic system time synchronization. |
| **`pointcloud_to_laserscan`** | `pointcloud_to_laserscan/` | Efficiently converts 3D SLAM point clouds into 2D laser scans for the planner. |

---

## 🛠️ Prerequisites & Environment Setup

### 1. System Requirements
- **OS**: Ubuntu 22.04 (Jammy Jellyfish)
- **ROS Version**: ROS 2 Humble Hawksbill
- **Python**: 3.10+

### 2. Automatic Environment Setup ⚡
We provide a **one-click script** to set up the isolated Python environment for NeuPAN.

**Run the setup script:**
```bash
./setup_neupan_env.sh
```

**What this script does:**
- 🏗️ Creates a virtual environment in `neupan_env/`
- 📦 Installs **PyTorch**, **NumPy**, **SciPy** and other core ML libraries
- ✅ Verifies installation integrity

> **⚠️ Vital Step**: Always activate the environment before running NeuPAN nodes!
> ```bash
> source neupan_env/bin/activate
> ```

---

## 🚀 Installation & Build

Follow these steps to set up your workspace from scratch.

### 1. Create Workspace
```bash
mkdir -p ~/neupan_ws/src
cd ~/neupan_ws/src
```

### 2. Import Code
Copy the contents of this repository into `~/neupan_ws/src`. Your structure should look like this:
```text
src/
├── neupan_ros2/             # Main NeuPAN package
├── point_lio/               # SLAM package
└── pointcloud_to_laserscan/ # Utility package
```

### 3. Install ROS Dependencies
```bash
cd ~/neupan_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 5. Source Workspace
```bash
source install/setup.bash
```

---

## 🤖 Real Robot Deployment

Designed for **Livox LiDARs** (e.g., Mid-360) and standard mobile robot bases.

### Step 1: Hardware Startup 🔌
Ensure your drivers are running:
*   **LiDAR**: Publishing to `/livox/lidar` or `/scan`
*   **Robot Base**: Listening on `/cmd_vel`

### Step 2: Launch Localization (Point-LIO) 📍
Start the SLAM system.
```bash
ros2 launch point_lio point_lio.launch.py
```
> *Note: Our custom Point-LIO automatically fixes "Extrapolation into past" errors by syncing sensor time with ROS system time.*

### Step 3: Launch NeuPAN Planning 🧠
Start the planner. **Don't forget to activate the environment first!**

```bash
# 1. Activate Python Env
source neupan_env/bin/activate

# 2. Launch Planner
ros2 launch neupan_ros neupan_with_pointlio.launch.py \
    config_file:=install/neupan_ros/share/neupan_ros/config/neupan_real_robot.yaml
```

### Step 4: Command & Control 🎮
1.  Open **RViz2**.
2.  Set **Fixed Frame** to `camera_init`.
3.  Use **"2D Goal Pose"** to set a target.
    *   NeuPAN will plan a global path.
    *   The robot will autonomously navigate to the goal.

---

## ⚙️ Configuration Guide

### Launch Arguments (`neupan_with_pointlio.launch.py`)

| Argument | Default | Description |
| :--- | :--- | :--- |
| `config_file` | *Required* | Path to your robot's YAML config. |
| `map_frame` | `camera_init` | Global map frame from SLAM. |
| `base_frame` | `aft_mapped` | Robot base frame (current pose). |
| `lidar_frame` | `body` | Frame of the input point cloud. |
| `cmd_vel_topic`| `/cmd_vel` | Output velocity command topic. |

### Tuning Parameters (`neupan_real_robot.yaml`)
*   `robot_radius`: 🛡️ Collision safety radius.
*   `max_vel`: 🚀 Max linear speed limit.
*   `dune_checkpoint`: 🧠 Path to neural network weights.

---

## 🐛 Troubleshooting

| Issue | Possible Cause | Solution |
| :--- | :--- | :--- |
| **"Waiting for neupan initial path"** | No goal set. | Set a goal using "2D Goal Pose" in RViz. |
| **"Extrapolation into the past"** | Time sync mismatch. | Use our patched `point_lio` (included here). |
| **ModuleNotFoundError** | Venv not active. | Run `source neupan_env/bin/activate`. |
| **Robot spins in place** | Obstacle too close. | Reduce `safety_margin` or check sensor data. |

---

<div align="center">
    <font size="2">Maintained by the NeuPAN Team</font>
</div>
