# NeuPAN ROS2 Package
"""
NeuPAN ROS2 神经路径规划包

这个包提供了基于神经网络的实时路径规划功能，专为ROS2 Humble设计。
主要用于移动机器人的动态避障和导航。

特性:
- 实时神经网络推理
- Point-LIO里程计集成
- 动态障碍物避障
- 多机器人类型支持（差速驱动、阿克曼转向）
"""

__version__ = "2.0.0"
__author__ = "NeuPAN Team"
__license__ = "MIT"

# 公开API
from .neupan_core_ros2 import NeuPANNode

__all__ = ['NeuPANNode'] 