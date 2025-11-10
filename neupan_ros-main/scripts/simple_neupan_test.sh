#!/bin/bash

# 简化的NeuPAN控制器功能测试脚本

echo "🎯 简化NeuPAN控制器功能测试"
echo "=========================================="

cd /home/robotmaster/ros2_ws
source install/setup.bash
export PYTHONPATH="/home/robotmaster/ros2_ws/src/NeuPAN:$PYTHONPATH"

# 清理函数
cleanup() {
    echo ""
    echo "🛑 清理进程..."
    pkill -f "enhanced_robot_simulator" || true
    pkill -f "simple_path_publisher" || true
    pkill -f "real_neupan_controller" || true
    pkill -f "static_transform_publisher" || true
    sleep 2
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "🚀 启动最小化测试组件:"
echo ""

# 1. 启动TF变换 (后台静默)
echo "1. 启动TF变换..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link >/dev/null 2>&1 &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame >/dev/null 2>&1 &
sleep 2

# 2. 启动机器人模拟器 (前台显示输出)
echo "2. 启动机器人模拟器..."
enhanced_robot_simulator &
SIM_PID=$!
sleep 3

# 3. 启动路径发布器
echo "3. 启动路径发布器..."
simple_path_publisher &
PATH_PID=$!
sleep 2

echo ""
echo "📊 检查话题状态:"
ros2 topic list | grep -E "(odom|scan|path)"
echo ""

# 4. 启动NeuPAN控制器 (前台显示)
echo "4. ⭐ 启动NeuPAN控制器..."
echo "   监控输出10秒..."
echo ""

timeout 10 real_neupan_controller &
NEUPAN_PID=$!

# 5. 实时监控关键话题
echo "📈 实时监控 (10秒):"
echo ""

for i in {1..10}; do
    echo "=== 第 $i 秒 ==="
    
    # 检查速度命令
    echo "Speed cmd:"
    timeout 1 ros2 topic echo /cmd_vel --once 2>/dev/null | grep -A2 -B1 linear || echo "  无速度数据"
    
    # 检查里程计
    echo "Odom:"
    timeout 1 ros2 topic echo /odom --once 2>/dev/null | grep -A5 "position:" || echo "  无里程计数据"
    
    echo ""
    sleep 1
done

wait $NEUPAN_PID

echo ""
echo "📋 测试总结:"
echo "============"

# 最终话题检查
echo "最终话题状态:"
ros2 topic list | grep -E "(cmd_vel|odom|scan|path)" | while read topic; do
    echo "  $topic: $(timeout 1 ros2 topic hz $topic 2>/dev/null | head -1 || echo '无数据')"
done

echo ""
echo "🏁 测试完成!"
echo "如果看到NeuPAN控制器初始化成功并且有速度输出，说明控制器工作正常。"

cleanup