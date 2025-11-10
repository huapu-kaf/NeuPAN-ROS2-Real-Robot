# NeuPAN ROS2 Package

这个包已经从ROS1成功转换为ROS2。

## 转换总结

- ✅ 更新了`package.xml`到ROS2格式（format=3）
- ✅ 创建了`setup.py`文件用于Python包管理
- ✅ 转换了所有Python代码从`rospy`到`rclpy`
- ✅ 更新了TF系统从`tf`到`tf2_ros`  
- ✅ 修正了参数API从ROS1到ROS2
- ✅ 更新了发布者、订阅者和服务的API
- ✅ 成功编译和安装

## 使用方法

### 1. 编译包

```bash
cd /home/robotmaster/ros2_ws
colcon build --packages-select neupan_ros
source install/setup.bash
```

### 2. 运行节点

#### 方法1：直接运行节点
```bash
ros2 run neupan_ros neupan_node --ros-args -p config_file:=<path_to_config_file>
```

#### 方法2：使用launch文件
```bash
ros2 launch neupan_ros neupan_test.launch.py
```

### 3. 节点参数

可以通过以下参数配置节点：

- `config_file`: NeuPAN配置文件路径 (必需)
- `map_frame`: 地图坐标系名称 (默认: "map")
- `base_frame`: 机器人基础坐标系名称 (默认: "base_link") 
- `lidar_frame`: 激光雷达坐标系名称 (默认: "laser_link")
- `marker_size`: 可视化标记大小 (默认: 0.05)
- `marker_z`: 标记Z轴高度 (默认: 1.0)
- `scan_angle_range`: 扫描角度范围 (默认: "-3.14 3.14")
- `scan_downsample`: 扫描点降采样率 (默认: 1)
- `scan_range`: 扫描距离范围 (默认: "0.1 5.0")
- `refresh_initial_path`: 是否刷新初始路径 (默认: false)
- `flip_angle`: 是否翻转角度 (默认: false)
- `include_initial_path_direction`: 是否包含初始路径方向 (默认: false)

### 4. 发布的话题

- `/neupan_cmd_vel` (geometry_msgs/Twist): 速度命令
- `/neupan_plan` (nav_msgs/Path): 规划路径
- `/neupan_ref_state` (nav_msgs/Path): 参考状态
- `/neupan_initial_path` (nav_msgs/Path): 初始路径
- `/dune_point_markers` (visualization_msgs/MarkerArray): DUNE点可视化
- `/nrmp_point_markers` (visualization_msgs/MarkerArray): NRMP点可视化
- `/robot_marker` (visualization_msgs/Marker): 机器人可视化

### 5. 订阅的话题

- `/scan` (sensor_msgs/LaserScan): 激光扫描数据
- `/initial_path` (nav_msgs/Path): 给定的初始路径
- `/neupan_waypoints` (nav_msgs/Path): 路点列表
- `/neupan_goal` (geometry_msgs/PoseStamped): 目标位置

## 主要变化

### API变化
- `rospy` → `rclpy`
- `rospy.init_node()` → `rclpy.init()` + `Node` 类继承
- `rospy.get_param()` → `self.declare_parameter()` + `self.get_parameter()`
- `rospy.Publisher()` → `self.create_publisher()`
- `rospy.Subscriber()` → `self.create_subscription()`
- `rospy.Rate()` + `while not rospy.is_shutdown()` → `self.create_timer()`
- `rospy.Time.now()` → `self.get_clock().now().to_msg()`
- `tf.TransformListener()` → `tf2_ros.TransformListener()`

### 结构变化
- 删除了`CMakeLists.txt`（改用Python包）
- 添加了`setup.py`
- 更新了`package.xml`格式
- 创建了Python包结构：`neupan_ros/neupan_ros/`

### 功能改进
- 更好的错误处理
- 改进的日志系统（节流日志）
- 现代化的ROS2参数系统

## 依赖项

确保安装了以下ROS2包：
- `rclpy`
- `geometry_msgs`
- `nav_msgs` 
- `sensor_msgs`
- `visualization_msgs`
- `tf2_ros`
- `tf2_geometry_msgs`

## 注意事项

1. 需要确保`neupan`Python包已安装
2. 配置文件路径必须正确指定
3. TF树必须正确设置（map → base_link, lidar_frame等）
4. 激光扫描数据应发布在`/scan`话题上