#!/usr/bin/env python3

"""
NeuPAN ROS2 Node Entry Point
Adapted for ROS2 Humble

Usage:
    ros2 run neupan_ros neupan_node_ros2
"""

import rclpy
from neupan_ros.neupan_core_ros2 import NeuPANCore


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        neupan_node = NeuPANCore()
        
        # 使用ROS2的spin
        rclpy.spin(neupan_node)
        
    except KeyboardInterrupt:
        print('\nNeuPAN node stopped by user')
    except Exception as e:
        print(f'Error in NeuPAN node: {e}')
    finally:
        # 清理资源
        if 'neupan_node' in locals():
            neupan_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()