#!/bin/bash

# NeuPAN控制器可视化测试启动脚本

echo "🚀 启动NeuPAN控制器可视化测试"
echo "==============================================="

# 设置环境
cd /home/robotmaster/ros2_ws
source install/setup.bash

echo "📊 启动以下组件:"
echo "  1. 坐标变换发布器"
echo "  2. 机器人模拟器 (提供里程计和激光雷达数据)"
echo "  3. 路径发布器"
echo "  4. 手动速度发布器 (模拟控制器)"
echo "  5. 目标点发布器"
echo "  6. RViz2 可视化"
echo ""

# 1. 启动静态坐标变换
echo "启动坐标变换..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &
sleep 1

# 2. 启动机器人模拟器
echo "启动机器人模拟器..."
enhanced_robot_simulator &
sleep 2

# 3. 启动路径发布器
echo "启动路径发布器..."
simple_path_publisher &
sleep 1

# 4. 启动手动速度发布器
echo "启动速度控制器模拟..."
manual_velocity_publisher &
sleep 1

# 5. 启动目标点发布器
echo "启动目标点发布器..."
goal_publisher_node &
sleep 1

# 6. 启动RViz
echo "启动RViz可视化..."
rviz2 -d /home/robotmaster/ros2_ws/src/neupan_ros/rviz/simple_visual_test.rviz &

echo ""
echo "✅ 所有组件已启动！"
echo ""
echo "📋 在RViz中你应该能看到："
echo "  - 网格坐标系"
echo "  - 激光雷达扫描数据 (/scan topic)"
echo "  - 规划路径 (/path topic)" 
echo "  - 目标点 (/goal_pose topic)"
echo ""
echo "🔍 可用的调试命令："
echo "  查看速度指令: ros2 topic echo /cmd_vel"
echo "  查看里程计: ros2 topic echo /odom"
echo "  查看激光数据: ros2 topic echo /scan"
echo "  查看路径: ros2 topic echo /path"
echo ""
echo "按 Ctrl+C 停止所有节点"

# 等待用户中断
wait