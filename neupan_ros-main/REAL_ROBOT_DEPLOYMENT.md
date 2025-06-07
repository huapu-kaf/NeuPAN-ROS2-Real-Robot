# NeuPAN ROS2 实车部署指南

本文档提供了将NeuPAN与Point-LIO里程计集成进行实车导航的详细步骤。

## 📋 前置条件

### 硬件要求
- 移动机器人平台（差速驱动或阿克曼转向）
- 激光雷达（支持ROS2，如Livox、Velodyne等）
- 车载计算机（推荐Intel i7以上CPU）
- IMU传感器（Point-LIO需要）

### 软件要求
- Ubuntu 22.04
- ROS2 Humble
- Point-LIO (已配置并运行正常)
- NeuPAN Python包

## 🚀 快速部署步骤

### 1. 环境准备

```bash
# 安装ROS2 Humble（如果还没安装）
sudo apt update && sudo apt install ros-humble-desktop

# 创建工作空间
mkdir -p ~/neupan_robot_ws/src
cd ~/neupan_robot_ws/src

# 克隆项目
git clone https://github.com/hanruihua/neupan_ros.git
```

### 2. 安装依赖

```bash
cd ~/neupan_robot_ws

# 安装ROS2依赖
rosdep install --from-paths src --ignore-src -r -y

# 安装Python依赖
pip3 install neupan torch numpy scipy cvxpy

# 构建包
colcon build --packages-select neupan_ros
source install/setup.bash
```

### 3. 配置参数

编辑配置文件 `config/neupan_real_robot.yaml`：

```yaml
# 根据你的机器人修改这些参数
robot:
  length: 0.8      # 机器人长度(米)
  width: 0.6       # 机器人宽度(米)
  max_v: 1.0       # 最大线速度(m/s)
  max_w: 1.0       # 最大角速度(rad/s)

planner:
  collision_threshold: 0.3  # 安全距离(米)
  goal_tolerance: 0.2       # 目标容忍度(米)
```

### 4. 启动系统

#### 方法1: 分步启动（推荐调试时使用）

```bash
# 终端1: 启动机器人底盘
ros2 launch your_robot_bringup robot.launch.py

# 终端2: 启动激光雷达
ros2 launch your_lidar_driver lidar.launch.py

# 终端3: 启动Point-LIO
ros2 launch point_lio mapping.launch.py

# 终端4: 启动NeuPAN
ros2 launch neupan_ros neupan_with_pointlio.launch.py \
    config_file:=$(ros2 pkg prefix neupan_ros)/share/neupan_ros/config/neupan_real_robot.yaml
```

#### 方法2: 一键启动

```bash
# 使用集成launch文件
ros2 launch neupan_ros neupan_with_pointlio.launch.py \
    config_file:=/path/to/your/config.yaml \
    scan_topic:=/your_lidar_topic \
    cmd_vel_topic:=/your_robot_cmd_vel \
    map_frame:=camera_init \
    base_frame:=base_link
```

## 🎯 设置导航目标

### 通过命令行

```bash
# 设置目标点 (x=5.0, y=2.0, yaw=0.0)
ros2 topic pub /neupan_goal geometry_msgs/PoseStamped "
header:
  frame_id: 'camera_init'
pose:
  position: {x: 5.0, y: 2.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"
```

### 通过RViz2

1. 启动RViz2: `rviz2`
2. 添加显示项：
   - `/neupan_plan` (nav_msgs/Path) - 规划路径
   - `/robot_marker` (visualization_msgs/Marker) - 机器人位置
   - `/dune_point_markers` (MarkerArray) - 障碍点可视化
3. 使用"2D Pose Estimate"工具设置目标

## 📊 监控和调试

### 关键话题监控

```bash
# 查看速度命令
ros2 topic echo /cmd_vel

# 查看规划路径
ros2 topic echo /neupan_plan

# 查看机器人状态
ros2 topic echo /Odometry

# 检查激光数据
ros2 topic echo /scan --no-arr
```

### 性能监控

```bash
# 监控节点性能
ros2 run rqt_top rqt_top

# 查看计算图
rqt_graph

# 监控TF树
ros2 run tf2_tools view_frames.py
```

## ⚙️ 参数调优

### 关键参数说明

| 参数 | 作用 | 建议值 | 调优建议 |
|------|------|--------|----------|
| `collision_threshold` | 避障距离 | 0.3m | 室外可适当增大到0.5m |
| `scan_range` | 激光有效范围 | "0.5 8.0" | 根据环境和雷达性能调整 |
| `max_v` | 最大线速度 | 1.0 m/s | 开始时设置较小值进行测试 |
| `scan_downsample` | 激光数据降采样 | 2 | 性能不足时增大此值 |

### 常见问题调优

**问题1: 机器人运动不平滑**
```yaml
# 增加预测时域
planner:
  horizon: 30  # 从20增加到30
  dt: 0.05     # 减小时间步长
```

**问题2: 避障过于保守**
```yaml
# 减小安全裕度
planner:
  collision_threshold: 0.2  # 从0.3减小到0.2
  safety_margin: 0.1        # 从0.2减小到0.1
```

**问题3: 计算性能不足**
```yaml
# 优化性能设置
scan_downsample: 3     # 增大降采样
scan_range: "0.5 5.0"  # 减小扫描范围
```

## 🔧 故障排除

### 常见错误

**错误1: TF变换失败**
```bash
# 检查TF树
ros2 run tf2_tools view_frames.py
# 确保camera_init -> base_link变换存在
```

**错误2: 没有收到激光数据**
```bash
# 检查话题
ros2 topic list | grep scan
ros2 topic info /your_scan_topic
```

**错误3: 控制命令无效**
```bash
# 检查cmd_vel话题
ros2 topic info /cmd_vel
# 确保机器人底盘正在监听此话题
```

### 安全注意事项

1. **首次测试**: 在开阔区域进行，远离人员和障碍物
2. **紧急停止**: 确保有紧急停止按钮或遥控器
3. **速度限制**: 初始测试时设置较低的最大速度
4. **监控距离**: 始终保持在安全距离内监控机器人

## 📈 性能优化建议

### CPU优化
- 使用多线程处理激光数据
- 适当降低发布频率
- 优化PyTorch推理性能

### 内存优化  
- 限制路径历史长度
- 定期清理可视化数据
- 使用合适的QoS设置

### 网络优化
- 使用合适的QoS策略
- 避免大量不必要的话题发布
- 考虑使用压缩传输

## 📞 支持和反馈

遇到问题时：
1. 查看ROS2日志: `ros2 log info`
2. 检查系统资源使用情况
3. 在GitHub提issue附上详细日志
4. 考虑加入相关技术交流群 