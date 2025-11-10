#!/bin/bash

# NeuPAN真实导航测试脚本

set -e

echo "🚀 启动真实的NeuPAN导航测试"
echo "=========================================="

# 设置环境
cd /home/robotmaster/ros2_ws
source install/setup.bash
export PYTHONPATH="/home/robotmaster/ros2_ws/src/NeuPAN:$PYTHONPATH"

echo "📊 启动导航组件:"
echo "  1. 坐标变换发布器"
echo "  2. 机器人模拟器 (里程计 + 激光雷达)"
echo "  3. 路径发布器" 
echo "  4. ⭐ 真实NeuPAN控制器 ⭐"
echo "  5. 目标点发布器"
echo "  6. 话题验证"
echo "  7. RViz2 可视化"
echo ""

# 启动日志文件
LOG_DIR="/tmp/neupan_test_logs"
mkdir -p $LOG_DIR

# 清理函数
cleanup() {
    echo ""
    echo "🛑 停止所有节点..."
    pkill -f "static_transform_publisher" || true
    pkill -f "enhanced_robot_simulator" || true 
    pkill -f "simple_path_publisher" || true
    pkill -f "real_neupan_controller" || true
    pkill -f "goal_publisher_node" || true
    pkill -f "rviz2" || true
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 1. 启动静态坐标变换
echo "1. 启动TF变换..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link > $LOG_DIR/tf_odom.log 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame > $LOG_DIR/tf_laser.log 2>&1 &
sleep 2

# 2. 启动机器人模拟器
echo "2. 启动机器人模拟器..."
enhanced_robot_simulator > $LOG_DIR/robot_sim.log 2>&1 &
ROBOT_SIM_PID=$!
sleep 3

# 3. 启动路径发布器
echo "3. 启动路径发布器..."
simple_path_publisher > $LOG_DIR/path_pub.log 2>&1 &
PATH_PUB_PID=$!
sleep 2

# 4. 启动真实的NeuPAN控制器 (关键组件)
echo "4. ⭐ 启动真实NeuPAN控制器..."
real_neupan_controller > $LOG_DIR/neupan_controller.log 2>&1 &
NEUPAN_PID=$!
sleep 3

# 5. 启动目标点发布器  
echo "5. 启动目标点发布器..."
goal_publisher_node > $LOG_DIR/goal_pub.log 2>&1 &
GOAL_PUB_PID=$!
sleep 2

# 6. 验证话题是否正常发布
echo "6. 验证ROS话题..."
echo "等待话题出现..."
for i in {1..5}; do
    echo "检查话题 ($i/5):"
    ros2 topic list | grep -E "(odom|scan|path|cmd_vel|goal_pose)" || true
    sleep 1
done

# 7. 启动RViz可视化
echo "7. 启动RViz2可视化..."
rviz2 -d /home/robotmaster/ros2_ws/src/neupan_ros/rviz/neupan_simple.rviz > $LOG_DIR/rviz.log 2>&1 &
RVIZ_PID=$!
sleep 3

echo ""
echo "✅ 所有组件已启动！"
echo ""
echo "📋 在RViz中你应该能看到："
echo "  - 网格坐标系"
echo "  - 机器人激光雷达扫描数据"
echo "  - 规划路径 (绿色线条)"  
echo "  - 目标点 (红色箭头)"
echo ""
echo "🚗 NeuPAN控制器状态:"
echo "  - 监听路径: /path"
echo "  - 监听里程计: /odom"
echo "  - 监听激光: /scan"
echo "  - 发布速度: /cmd_vel"
echo ""
echo "🔍 可用的调试命令 (在另一个终端中)："
echo "  查看NeuPAN速度输出: ros2 topic echo /cmd_vel"
echo "  查看机器人位置: ros2 topic echo /odom"  
echo "  查看激光数据: ros2 topic echo /scan"
echo "  查看路径: ros2 topic echo /path"
echo "  查看TF树: ros2 run tf2_tools view_frames"
echo ""
echo "📄 日志文件位置: $LOG_DIR/"
echo "  - NeuPAN控制器: $LOG_DIR/neupan_controller.log"
echo "  - 机器人模拟器: $LOG_DIR/robot_sim.log"
echo "  - 其他组件: $LOG_DIR/*.log"
echo ""
echo "⏰ 测试将运行30秒后自动停止，或按 Ctrl+C 手动停止"

# 监控关键进程
monitor_processes() {
    sleep 5
    while true; do
        if ! kill -0 $NEUPAN_PID 2>/dev/null; then
            echo "❌ NeuPAN控制器进程已停止！"
            break
        fi
        
        if ! kill -0 $ROBOT_SIM_PID 2>/dev/null; then
            echo "❌ 机器人模拟器进程已停止！"
            break
        fi
        
        echo "✅ $(date '+%H:%M:%S') - 所有进程正常运行"
        sleep 5
    done
}

# 启动监控
monitor_processes &
MONITOR_PID=$!

# 等待用户中断或超时
timeout 30 bash -c 'while true; do sleep 1; done' || echo "⏰ 30秒测试时间结束"

# 清理
cleanup