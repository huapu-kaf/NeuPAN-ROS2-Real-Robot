#!/bin/bash

# 检查NeuPAN导航相关话题的脚本

echo "🔍 检查NeuPAN导航话题状态"
echo "===================================="

cd /home/robotmaster/ros2_ws
source install/setup.bash

echo "📋 所有可用话题:"
ros2 topic list
echo ""

echo "🚗 检查关键导航话题:"
echo ""

# 检查里程计
echo "1. 里程计话题 /odom:"
if ros2 topic list | grep -q "/odom"; then
    echo "   ✅ 话题存在"
    echo "   📊 话题信息:"
    timeout 2 ros2 topic echo /odom --once 2>/dev/null && echo "   ✅ 数据正常" || echo "   ❌ 无数据"
else
    echo "   ❌ 话题不存在"
fi
echo ""

# 检查激光雷达
echo "2. 激光雷达话题 /scan:"
if ros2 topic list | grep -q "/scan"; then
    echo "   ✅ 话题存在" 
    echo "   📊 话题信息:"
    timeout 2 ros2 topic echo /scan --once 2>/dev/null && echo "   ✅ 数据正常" || echo "   ❌ 无数据"
else
    echo "   ❌ 话题不存在"
fi
echo ""

# 检查路径
echo "3. 路径话题 /path:"
if ros2 topic list | grep -q "/path"; then
    echo "   ✅ 话题存在"
    echo "   📊 话题信息:"
    timeout 2 ros2 topic echo /path --once 2>/dev/null && echo "   ✅ 数据正常" || echo "   ❌ 无数据"
else
    echo "   ❌ 话题不存在"
fi
echo ""

# 检查速度命令
echo "4. 速度命令话题 /cmd_vel:"
if ros2 topic list | grep -q "/cmd_vel"; then
    echo "   ✅ 话题存在"
    echo "   📊 话题信息:"
    timeout 2 ros2 topic echo /cmd_vel --once 2>/dev/null && echo "   ✅ 数据正常" || echo "   ❌ 无数据"
else
    echo "   ❌ 话题不存在"
fi
echo ""

# 检查目标点
echo "5. 目标点话题 /goal_pose:"
if ros2 topic list | grep -q "/goal_pose"; then
    echo "   ✅ 话题存在"
    echo "   📊 话题信息:"
    timeout 2 ros2 topic echo /goal_pose --once 2>/dev/null && echo "   ✅ 数据正常" || echo "   ❌ 无数据"
else
    echo "   ❌ 话题不存在"
fi
echo ""

# 检查TF变换
echo "6. TF变换:"
if ros2 topic list | grep -q "/tf"; then
    echo "   ✅ /tf话题存在"
else
    echo "   ❌ /tf话题不存在"
fi

if ros2 topic list | grep -q "/tf_static"; then
    echo "   ✅ /tf_static话题存在"
else
    echo "   ❌ /tf_static话题不存在"  
fi
echo ""

echo "🏁 检查完成！"
echo ""
echo "💡 如果有话题缺失数据，建议:"
echo "   - 检查对应节点是否正常运行"
echo "   - 查看节点日志: /tmp/neupan_test_logs/"
echo "   - 重新运行测试脚本"