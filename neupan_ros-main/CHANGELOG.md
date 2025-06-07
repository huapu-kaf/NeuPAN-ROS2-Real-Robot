# Changelog

All notable changes to this project will be documented in this file.

## [2.0.0] - 2025-01-XX - ROS2 Humble Migration

### 🚀 Major Changes
- **完全迁移到ROS2 Humble**: 从ROS1 Noetic完全重写为ROS2 Humble
- **Point-LIO集成**: 专门为与Point-LIO里程计集成而优化
- **实车部署准备**: 添加了完整的实车部署指南和配置

### ✨ Added
- ROS2 Humble完全兼容的代码基础
- `neupan_core_ros2.py`: 全新的ROS2节点实现
- `neupan_node_ros2.py`: ROS2入口点
- ROS2 launch文件系统:
  - `neupan_ros2.launch.py`: 基础launch文件
  - `neupan_with_pointlio.launch.py`: Point-LIO集成launch文件
- 配置文件: `neupan_real_robot.yaml` 实车部署模板
- 完整的ROS2包结构: `setup.py`, `resource/`, `neupan_ros/__init__.py`
- 实车部署文档: `REAL_ROBOT_DEPLOYMENT.md`

### 🔄 Changed
- **API架构**: rospy → rclpy完全重写
- **TF系统**: tf → tf2_ros迁移
- **参数系统**: ROS参数 → ROS2参数声明系统
- **消息处理**: 升级到ROS2 QoS系统
- **构建系统**: catkin → ament_cmake + ament_python
- **包格式**: package.xml format="2" → format="3"

### 🛠️ Technical Improvements
- **性能优化**: 使用ROS2定时器替代while循环
- **内存管理**: 更好的节点生命周期管理
- **错误处理**: 改进的异常处理和日志记录
- **类型安全**: 更强的类型检查和验证
- **可配置性**: 增强的参数化和动态配置

### 📚 Documentation
- 完全重写的README.md for ROS2
- 详细的实车部署指南
- ROS2 API文档更新
- 参数调优指南
- 故障排除文档

### 🔧 Infrastructure
- CI/CD适配ROS2 (待添加)
- 依赖管理优化
- 包版本管理

### ⚠️ Breaking Changes
- **不向后兼容ROS1**: 此版本完全移除ROS1支持
- **API变更**: 所有API从rospy迁移到rclpy
- **话题命名**: 某些话题名称可能有变化
- **参数格式**: 参数声明和访问方式完全改变

### 🎯 Real Robot Features
- Point-LIO里程计无缝集成
- 实车安全参数配置
- 激光雷达数据优化处理
- 坐标系自动适配
- 性能监控和调试工具

### 🐛 Bug Fixes
- 修复TF查找超时问题
- 改进激光数据处理稳定性
- 优化内存使用
- 修复节点生命周期问题

---

## [1.x.x] - Previous ROS1 Versions

### Legacy ROS1 Noetic Support
- 原始ROS1 Noetic实现
- Gazebo仿真集成
- 基础导航功能

---

## Migration Guide

### From ROS1 to ROS2

如果你从ROS1版本迁移到ROS2，请注意以下关键变化：

1. **工作空间**: catkin_ws → colcon workspace
2. **构建命令**: `catkin_make` → `colcon build`
3. **源文件**: `source devel/setup.bash` → `source install/setup.bash`
4. **启动**: `roslaunch` → `ros2 launch`
5. **参数**: `~param_name` → `param_name` (声明式)
6. **话题**: `rostopic` → `ros2 topic`

### 详细迁移步骤
请参考 `REAL_ROBOT_DEPLOYMENT.md` 获取完整的迁移和部署指南。 