# neupan_ros - ROS2 Humble Edition

This is the **ROS2 Humble** wrapper for [NeuPAN Planner](https://github.com/hanruihua/neupan), adapted for real-world robot deployment with Point-LIO integration.

## Prerequisites
- **Ubuntu 22.04**
- **ROS2 Humble**
- **Python >= 3.10**
- Installed [NeuPAN Planner](https://github.com/hanruihua/neupan)
- Point-LIO for odometry (optional but recommended)

## Installation

```bash
# Create ROS2 workspace
mkdir -p ~/neupan_ros2_ws/src
cd ~/neupan_ros2_ws/src

# Clone the repository
git clone https://github.com/hanruihua/neupan_ros

# Install dependencies
cd ~/neupan_ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build with colcon
colcon build --packages-select neupan_ros

# Source the setup
source install/setup.bash
```

## Quick Start for Real Robot

```bash
# 1. Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source ~/neupan_ros2_ws/install/setup.bash

# 2. Launch with your config file
ros2 launch neupan_ros neupan_ros2.launch.py config_file:=/path/to/your/config.yaml

# 3. Provide initial path (optional)
ros2 topic pub /initial_path nav_msgs/Path [your path data]

# 4. Set goal
ros2 topic pub /neupan_goal geometry_msgs/PoseStamped [your goal]
```

## Demonstration

### Dynamic collision avoidance

We provide the dynamic collision avoidance examples in Gazebo shown as follows. To run these examples, please see [example/gazebo_limo](https://github.com/hanruihua/neupan_ros/tree/main/example/gazebo_limo) for detail.

https://github.com/user-attachments/assets/1d5eb028-0d22-4741-8899-40a3ea7caab4

## ROS2 Node API 

### Published Topics

| Topic Name             | Message Type                     | Description                                |
| ---------------------- | -------------------------------- | ------------------------------------------ |
| `/neupan_cmd_vel`      | `geometry_msgs/Twist`            | Velocity command to the robot (remappable to `/cmd_vel`) |
| `/neupan_plan`         | `nav_msgs/Path`                  | The NeuPAN planned path                    |
| `/neupan_initial_path` | `nav_msgs/Path`                  | The initial path for NeuPAN visualization |
| `/neupan_ref_state`    | `nav_msgs/Path`                  | The current reference state visualization |
| `/dune_point_markers`  | `visualization_msgs/MarkerArray` | The DUNE points markers visualization     |
| `/nrmp_point_markers`  | `visualization_msgs/MarkerArray` | The NRMP points markers visualization     |
| `/robot_marker`        | `visualization_msgs/Marker`      | The robot marker visualization            |

### Subscribed Topics

| Topic Name      | Message Type                | Description                                                                                        |
| --------------- | --------------------------- | -------------------------------------------------------------------------------------------------- |
| `/scan`         | `sensor_msgs/LaserScan`     | Laser scan data (remappable)                                                                      |
| `/initial_path` | `nav_msgs/Path`             | The initial path for NeuPAN                                                                       |
| `/neupan_goal`  | `geometry_msgs/PoseStamped` | Goal for NeuPAN planner                                                                           |

### ROS2 Parameters

| Parameter Name                    | Type / Default Value | Description                                              |
| --------------------------------- | -------------------- | -------------------------------------------------------- |
| `config_file`                     | `str` / None         | **Required**: Path to NeuPAN config YAML file           |
| `map_frame`                       | `str` / `map`        | Map frame name                                           |
| `base_frame`                      | `str` / `base_link`  | Robot base frame name                                    |
| `lidar_frame`                     | `str` / `laser_link` | Lidar frame name                                         |
| `marker_size`                     | `float` / `0.05`     | Marker size for DUNE and NRMP points visualization      |
| `marker_z`                        | `float` / `1.0`      | Robot marker height                                      |
| `scan_angle_range`                | `str` / `-3.14 3.14` | Valid scan angle range [min, max] in radians            |
| `scan_downsample`                 | `int` / `1`          | Laser scan downsample rate                               |
| `scan_range`                      | `str` / `0.0 5.0`    | Valid scan distance range [min, max] in meters          |
| `dune_checkpoint`                 | `str` / `None`       | Path to DUNE checkpoint file                             |
| `refresh_initial_path`            | `bool` / `False`     | Whether to refresh the initial path                      |
| `flip_angle`                      | `bool` / `False`     | Whether to flip scan angles                              |
| `include_initial_path_direction`  | `bool` / `False`     | Use path orientation vs. gradient direction              |

### Launch Parameters

| Parameter Name | Description                              | Example |
| -------------- | ---------------------------------------- | ------- |
| `config_file`  | **Required**: NeuPAN configuration file | `/path/to/config.yaml` |
| `scan_topic`   | Laser scan topic to subscribe           | `/scan` |
| `cmd_vel_topic`| Velocity command output topic           | `/cmd_vel` |

## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).

## Citation

If you find this code or paper is helpful, please kindly star :star: this repository and cite our paper by the following BibTeX entry:

```bibtex
@ARTICLE{10938329,
  author={Han, Ruihua and Wang, Shuai and Wang, Shuaijun and Zhang, Zeqing and Chen, Jianjun and Lin, Shijie and Li, Chengyang and Xu, Chengzhong and Eldar, Yonina C. and Hao, Qi and Pan, Jia},
  journal={IEEE Transactions on Robotics}, 
  title={NeuPAN: Direct Point Robot Navigation With End-to-End Model-Based Learning}, 
  year={2025},
  volume={41},
  number={},
  pages={2804-2824},
  doi={10.1109/TRO.2025.3554252}}
```

