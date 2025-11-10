#!/bin/bash

# NeuPAN控制器可视化测试脚本

echo "🎯 NeuPAN控制器可视化测试"
echo "=========================================="

cd /home/robotmaster/ros2_ws
source install/setup.bash
export PYTHONPATH="/home/robotmaster/ros2_ws/src/NeuPAN:$PYTHONPATH"

# 清理函数
cleanup() {
    echo ""
    echo "🛑 停止所有节点..."
    pkill -f "enhanced_robot_simulator" || true
    pkill -f "simple_path_publisher" || true
    pkill -f "real_neupan_controller" || true
    pkill -f "static_transform_publisher" || true
    pkill -f "rviz2" || true
    sleep 2
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "📊 启动可视化测试组件:"
echo "  1. 坐标变换发布器"
echo "  2. 机器人模拟器（简化环境）"
echo "  3. 路径发布器"
echo "  4. NeuPAN控制器"  
echo "  5. RViz2可视化"
echo ""

# 1. 启动TF变换
echo "1. 启动TF坐标变换..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link > /tmp/tf_odom.log 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame > /tmp/tf_laser.log 2>&1 &
sleep 2

# 2. 启动简化的机器人模拟器
echo "2. 启动机器人模拟器（简化环境）..."
enhanced_robot_simulator > /tmp/robot_sim.log 2>&1 &
sleep 3

# 3. 启动路径发布器
echo "3. 启动路径发布器..."
simple_path_publisher > /tmp/path_pub.log 2>&1 &
sleep 2

echo ""
echo "📊 验证话题状态:"
ros2 topic list | grep -E "(odom|scan|path|tf)"
echo ""

# 4. 启动NeuPAN控制器
echo "4. 启动NeuPAN控制器..."
real_neupan_controller > /tmp/neupan_controller.log 2>&1 &
NEUPAN_PID=$!
sleep 5

# 检查控制器是否成功启动
if kill -0 $NEUPAN_PID 2>/dev/null; then
    echo "   ✅ NeuPAN控制器运行中"
else
    echo "   ❌ NeuPAN控制器启动失败"
    cat /tmp/neupan_controller.log
    exit 1
fi

# 5. 启动RViz
echo "5. 启动RViz2可视化..."
echo "   配置文件: neupan_simple.rviz"

# 使用基本RViz配置
echo "   使用配置: basic_neupan.rviz"
rviz2 -d /home/robotmaster/ros2_ws/src/neupan_ros/rviz/basic_neupan.rviz > /tmp/rviz.log 2>&1 &

RVIZ_PID=$!
sleep 3

echo ""
echo "✅ 所有组件已启动!"
echo ""
echo "📋 在RViz中应该能看到："
echo "  📐 网格坐标系"
echo "  🔴 激光雷达扫描点"
echo "  🟢 规划路径（绿色线条）"
echo "  🚗 机器人位置"
echo ""
echo "🎛️  RViz配置步骤："
echo "  1. 设置Fixed Frame为 'odom'"
echo "  2. 添加显示项目："
echo "     - Grid (网格)"
echo "     - LaserScan (话题: /scan)"
echo "     - Path (话题: /path)"
echo "     - RobotModel (可选)"
echo ""
echo "🔍 实时监控数据："
echo ""

# 实时监控循环
for i in {1..30}; do
    echo "=== 第 $i 次检查 ==="
    
    # 检查关键话题
    echo "话题状态："
    for topic in "/odom" "/scan" "/path" "/cmd_vel"; do
        if ros2 topic list | grep -q "$topic"; then
            hz_info=$(timeout 2 ros2 topic hz $topic 2>/dev/null | head -1 || echo "无数据")
            echo "  $topic: $hz_info"
        else
            echo "  $topic: 不存在"
        fi
    done
    
    # 检查速度命令
    echo "当前速度命令:"
    timeout 1 ros2 topic echo /cmd_vel --once 2>/dev/null | grep -A1 -B1 "linear\|angular" | head -4 || echo "  无速度数据"
    
    echo ""
    sleep 3
done

echo ""
echo "📄 日志文件："
echo "  - NeuPAN控制器: /tmp/neupan_controller.log"
echo "  - 机器人模拟器: /tmp/robot_sim.log"
echo "  - 路径发布器: /tmp/path_pub.log"
echo "  - RViz: /tmp/rviz.log"
echo ""
echo "💡 调试命令："
echo "  查看NeuPAN日志: tail -f /tmp/neupan_controller.log"
echo "  查看速度输出: ros2 topic echo /cmd_vel"
echo "  查看路径数据: ros2 topic echo /path"
echo "  查看TF树: ros2 run tf2_tools view_frames"
echo ""
echo "⏰ 测试将继续运行，按 Ctrl+C 停止"

# 保持运行
while true; do
    # 检查关键进程
    if ! kill -0 $NEUPAN_PID 2>/dev/null; then
        echo "❌ NeuPAN控制器进程停止"
        break
    fi
    
    if ! kill -0 $RVIZ_PID 2>/dev/null; then
        echo "❌ RViz进程停止"
        break
    fi
    
    sleep 5
done

cleanup