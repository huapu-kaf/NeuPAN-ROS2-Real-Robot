# NeuPAN ROS 2 Real Robot Integration Guide

This repository contains the complete ROS 2 integration for NeuPAN (Neural Path Planner) on real robots, specifically adapted for ROS 2 Humble. It includes the NeuPAN planner wrapper, Point-LIO SLAM integration, and necessary utilities.

## 📦 Package Overview

The workspace consists of three main packages:

1.  **`neupan_ros`** (`neupan_ros2/`):
    The core ROS 2 wrapper for the NeuPAN planner. It handles path planning, collision avoidance, and velocity command generation.
2.  **`point_lio`** (`point_lio/`):
    A robust Lidar-Inertial Odometry system providing real-time localization and mapping. Modified to support automatic time synchronization.
3.  **`pointcloud_to_laserscan`** (`pointcloud_to_laserscan/`):
    A utility package to convert 3D point clouds from Point-LIO into 2D laser scans required by NeuPAN.

## 🛠️ Prerequisites

- **OS**: Ubuntu 22.04 (Jammy Jellyfish)
- **ROS Version**: ROS 2 Humble 
- **Python Dependencies**:
  - `neupan` (Must be installed separately, e.g., `pip install neupan`)
  - `numpy`, `scipy`, `torch`

## 🚀 Installation & Build

1.  **Create a Workspace**:
    ```bash
    mkdir -p ~/neupan_ws/src
    cd ~/neupan_ws/src
    ```

2.  **Clone/Copy Repository**:
    Copy the contents of this repository into `~/neupan_ws/src`.
    Ensure the directory structure looks like this:
    ```text
    src/
    ├── neupan_ros2/             # Contains neupan_ros package
    ├── point_lio/               # Modified Point-LIO
    └── pointcloud_to_laserscan/ # Local source package
    ```

3.  **Install Dependencies**:
    ```bash
    cd ~/neupan_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

4.  **Build the Workspace**:
    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

5.  **Source the Environment**:
    ```bash
    source install/setup.bash
    ```

## 🤖 Real Robot Deployment

This setup is designed to work with Livox LiDARs (e.g., Mid-360) and differential/omnidirectional robots.

### 1. Hardware Startup
Ensure your robot's base driver and LiDAR driver are running.
- **LiDAR**: Publish to `/livox/lidar` (CustomMsg) or `/scan` (PointCloud2).
- **Base**: Subscribe to `/cmd_vel` for velocity control.

### 2. Launch Point-LIO (Localization)
Start the SLAM system to provide odometry and point clouds.
```bash
ros2 launch point_lio point_lio.launch.py
```
*Note: We have patched Point-LIO to automatically synchronize timestamps with the system time, fixing common "Extrapolation into past" errors.*

### 3. Launch NeuPAN (Planning)
Start the planner with the integrated Point-LIO bridge. This launch file automatically starts the `pointcloud_to_laserscan` node to convert 3D data for NeuPAN.

```bash
ros2 launch neupan_ros neupan_with_pointlio.launch.py config_file:=<path_to_config>
```

**Example**:
```bash
ros2 launch neupan_ros neupan_with_pointlio.launch.py config_file:=install/neupan_ros/share/neupan_ros/config/neupan_real_robot.yaml
```

### 4. Operation
1.  Open **RViz2**.
2.  Set the Fixed Frame to `camera_init`.
3.  Use the **"2D Goal Pose"** tool in RViz to set a navigation target.
    - NeuPAN will generate a global path and start local planning.
    - The robot should start moving towards the goal while avoiding obstacles.

## ⚙️ Configuration

### `neupan_with_pointlio.launch.py` Arguments

| Argument | Default | Description |
| :--- | :--- | :--- |
| `config_file` | **Required** | Path to the NeuPAN YAML config file. |
| `map_frame` | `camera_init` | Global frame from Point-LIO. |
| `base_frame` | `aft_mapped` | Robot base frame (odom frame). |
| `lidar_frame` | `body` | Frame of the point cloud. |
| `cmd_vel_topic`| `/cmd_vel` | Topic to publish velocity commands. |

### Key Config Parameters (`neupan_real_robot.yaml`)

Edit this file to tune performance:
- `robot_radius`: Size of your robot (for collision checking).
- `max_vel`: Maximum linear velocity.
- `max_rot_vel`: Maximum angular velocity.
- `dune_checkpoint`: Path to the DUNE model weights.

## 🐛 Troubleshooting

**Issue: "Waiting for neupan initial path"**
- **Cause**: No goal has been set yet.
- **Fix**: Use "2D Goal Pose" in RViz to set a target.

**Issue: "Extrapolation into the past" (TF Errors)**
- **Cause**: LiDAR hardware time differs from ROS system time.
- **Fix**: Ensure you are using the modified `point_lio` from this repository. It includes an auto-sync patch.

**Issue: Robot rotates but doesn't move**
- **Cause**: Obstacle detected too close or goal is unreachable.
- **Fix**: Check the `safety_margin` in the config and ensure the `scan` topic data is correct in RViz.
