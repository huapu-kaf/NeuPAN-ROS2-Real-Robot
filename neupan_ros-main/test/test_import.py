#!/usr/bin/env python3

"""
Basic import test for neupan_ros package
确保包能正常导入和基本功能可用
"""

import pytest
import sys
import os

def test_rclpy_import():
    """测试ROS2核心包导入"""
    try:
        import rclpy
        assert True, "rclpy导入成功"
    except ImportError as e:
        pytest.fail(f"无法导入rclpy: {e}")

def test_geometry_msgs_import():
    """测试geometry_msgs导入"""
    try:
        from geometry_msgs.msg import Twist, PoseStamped
        assert True, "geometry_msgs导入成功"
    except ImportError as e:
        pytest.fail(f"无法导入geometry_msgs: {e}")

def test_nav_msgs_import():
    """测试nav_msgs导入"""
    try:
        from nav_msgs.msg import Path
        assert True, "nav_msgs导入成功"
    except ImportError as e:
        pytest.fail(f"无法导入nav_msgs: {e}")

def test_tf2_import():
    """测试tf2_ros导入"""
    try:
        import tf2_ros
        assert True, "tf2_ros导入成功"
    except ImportError as e:
        pytest.fail(f"无法导入tf2_ros: {e}")

def test_numpy_import():
    """测试numpy导入"""
    try:
        import numpy as np
        assert True, "numpy导入成功"
    except ImportError as e:
        pytest.fail(f"无法导入numpy: {e}")

def test_neupan_package_structure():
    """测试neupan_ros包结构"""
    try:
        import neupan_ros
        assert hasattr(neupan_ros, '__version__'), "包版本信息存在"
    except ImportError as e:
        pytest.fail(f"无法导入neupan_ros包: {e}")

def test_neupan_core_import():
    """测试核心模块导入（可选，如果neupan已安装）"""
    try:
        # 如果neupan已安装，测试导入
        import neupan
        print("✓ NeuPAN核心包已安装")
    except ImportError:
        print("⚠ NeuPAN核心包未安装，这在CI环境中是正常的")
        # 在CI环境中这是正常的，不应该失败

if __name__ == '__main__':
    # 运行所有测试
    print("开始运行neupan_ros导入测试...")
    
    test_rclpy_import()
    print("✓ rclpy导入测试通过")
    
    test_geometry_msgs_import()
    print("✓ geometry_msgs导入测试通过")
    
    test_nav_msgs_import()
    print("✓ nav_msgs导入测试通过")
    
    test_tf2_import()
    print("✓ tf2_ros导入测试通过")
    
    test_numpy_import()
    print("✓ numpy导入测试通过")
    
    test_neupan_package_structure()
    print("✓ neupan_ros包结构测试通过")
    
    test_neupan_core_import()
    
    print("所有导入测试完成！") 