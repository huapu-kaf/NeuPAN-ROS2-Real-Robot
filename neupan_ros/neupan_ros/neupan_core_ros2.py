#!/usr/bin/env python3

"""
neupan_core_ros2 is the main class for the neupan_ros package adapted for ROS2 Humble. 
It subscribes to laser scan and provides velocity commands for real-time robot navigation.

Adapted for ROS2 Humble by adding:
- rclpy instead of rospy
- tf2_ros instead of tf
- ROS2 parameter system
- ROS2 node lifecycle

Developed by Ruihua Han
Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
"""

from neupan import neupan
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan
from math import sin, cos, atan2
import numpy as np
from neupan.util import get_transform
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import tf2_geometry_msgs
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from builtin_interfaces.msg import Time


class NeuPANCore(Node):
    def __init__(self):
        super().__init__('neupan_node')
        
        # 声明ROS2参数
        self.declare_parameter('config_file', rclpy.Parameter.Type.STRING)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'laser_link')
        self.declare_parameter('marker_size', 0.05)
        self.declare_parameter('marker_z', 1.0)
        self.declare_parameter('scan_angle_range', '-3.14 3.14')
        self.declare_parameter('scan_downsample', 1)
        self.declare_parameter('scan_range', '0.0 5.0')
        self.declare_parameter('dune_checkpoint', rclpy.Parameter.Type.STRING)
        self.declare_parameter('refresh_initial_path', False)
        self.declare_parameter('flip_angle', False)
        self.declare_parameter('include_initial_path_direction', False)

        # 获取参数
        self.planner_config_file = self.get_parameter('config_file').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.marker_size = float(self.get_parameter('marker_size').value)
        self.marker_z = float(self.get_parameter('marker_z').value)
        
        scan_angle_range_str = self.get_parameter('scan_angle_range').value
        self.scan_angle_range = np.fromstring(scan_angle_range_str, dtype=np.float32, sep=' ')
        
        self.scan_downsample = int(self.get_parameter('scan_downsample').value)
        
        scan_range_str = self.get_parameter('scan_range').value
        self.scan_range = np.fromstring(scan_range_str, dtype=np.float32, sep=' ')
        
        self.dune_checkpoint = self.get_parameter('dune_checkpoint').value
        self.refresh_initial_path = self.get_parameter('refresh_initial_path').value
        self.flip_angle = self.get_parameter('flip_angle').value
        self.include_initial_path_direction = self.get_parameter('include_initial_path_direction').value

        if self.planner_config_file is None:
            raise ValueError("No planner config file provided! Please set the parameter config_file")

        # 初始化NeuPAN规划器
        pan = {'dune_checkpoint': self.dune_checkpoint}
        self.neupan_planner = neupan.init_from_yaml(self.planner_config_file, pan=pan)
        
        # 数据存储
        self.obstacle_points = None  # (2, n)  n number of points
        self.robot_state = None  # (3, 1) [x, y, theta]
        self.stop = False
        self.arrive = False

        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 发布器
        self.vel_pub = self.create_publisher(Twist, '/neupan_cmd_vel', qos_profile)
        self.plan_pub = self.create_publisher(Path, '/neupan_plan', qos_profile)
        self.ref_state_pub = self.create_publisher(Path, '/neupan_ref_state', qos_profile)
        self.ref_path_pub = self.create_publisher(Path, '/neupan_initial_path', qos_profile)
        
        # 可视化发布器
        self.point_markers_pub_dune = self.create_publisher(MarkerArray, '/dune_point_markers', qos_profile)
        self.robot_marker_pub = self.create_publisher(Marker, '/robot_marker', qos_profile)
        self.point_markers_pub_nrmp = self.create_publisher(MarkerArray, '/nrmp_point_markers', qos_profile)

        # TF2监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 订阅器
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.path_sub = self.create_subscription(Path, '/initial_path', self.path_callback, qos_profile)
        self.goal_sub = self.create_subscription(PoseStamped, '/neupan_goal', self.goal_callback, qos_profile)

        # 创建定时器以50Hz运行主循环
        self.timer = self.create_timer(0.02, self.run_loop)  # 50Hz = 0.02s
        
        self.get_logger().info('NeuPAN node initialized')

    def run_loop(self):
        """主运行循环"""
        try:
            # 获取机器人位置
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, 
                self.base_frame, 
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            
            # 提取位置和姿态
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            yaw = self.quat_to_yaw_list([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ])
            
            self.robot_state = np.array([x, y, yaw]).reshape(3, 1)

        except Exception as e:
            self.get_logger().info(
                f'Waiting for tf transform from {self.base_frame} to {self.map_frame}: {str(e)}',
                throttle_duration_sec=1.0
            )
            return

        if self.robot_state is None:
            self.get_logger().warn('Waiting for robot state', throttle_duration_sec=1.0)
            return

        # 设置初始路径
        if (len(self.neupan_planner.waypoints) >= 1 and 
            self.neupan_planner.initial_path is None):
            self.neupan_planner.set_initial_path_from_state(self.robot_state)

        if self.neupan_planner.initial_path is None:
            self.get_logger().warn('Waiting for neupan initial path', throttle_duration_sec=1.0)
            return

        # 发布初始路径
        self.ref_path_pub.publish(self.generate_path_msg(self.neupan_planner.initial_path))

        if self.obstacle_points is None:
            self.get_logger().warn(
                'No obstacle points, only path tracking task will be performed',
                throttle_duration_sec=1.0
            )

        # 运行NeuPAN规划器
        action, info = self.neupan_planner(self.robot_state, self.obstacle_points)

        self.stop = info["stop"]
        self.arrive = info["arrive"]

        if info["arrive"]:
            self.get_logger().info('Arrive at the target', throttle_duration_sec=0.1)

        # 发布路径和速度
        self.plan_pub.publish(self.generate_path_msg(info["opt_state_list"]))
        self.ref_state_pub.publish(self.generate_path_msg(info["ref_state_list"]))
        self.vel_pub.publish(self.generate_twist_msg(action))

        # 发布可视化信息
        self.point_markers_pub_dune.publish(self.generate_dune_points_markers_msg())
        self.point_markers_pub_nrmp.publish(self.generate_nrmp_points_markers_msg())
        self.robot_marker_pub.publish(self.generate_robot_marker_msg())

        if info["stop"]:
            self.get_logger().warn(
                f'NeuPAN stop with min distance {self.neupan_planner.min_distance.detach().item():.3f}, '
                f'threshold {self.neupan_planner.collision_threshold}',
                throttle_duration_sec=0.5
            )

    def scan_callback(self, scan_msg):
        """激光雷达数据回调函数"""
        if self.robot_state is None:
            return

        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        points = []

        if self.flip_angle:
            angles = np.flip(angles)

        for i in range(len(ranges)):
            distance = ranges[i]
            angle = angles[i]

            if (i % self.scan_downsample == 0 and
                distance >= self.scan_range[0] and
                distance <= self.scan_range[1] and
                angle > self.scan_angle_range[0] and
                angle < self.scan_angle_range[1]):
                
                point = np.array([[distance * cos(angle)], [distance * sin(angle)]])
                points.append(point)

        if len(points) == 0:
            self.obstacle_points = None
            self.get_logger().info('No valid scan points', once=True)
            return

        point_array = np.hstack(points)

        try:
            # 获取激光雷达到地图的变换
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.lidar_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            
            self.obstacle_points = get_transform(transform, point_array)
            
        except Exception as e:
            self.get_logger().debug(f'TF error in scan callback: {str(e)}')

    def path_callback(self, path):
        """初始路径回调函数"""
        if len(path.poses) == 0:
            return

        waypoints = []
        for pose in path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            if self.include_initial_path_direction:
                yaw = self.quat_to_yaw(pose.pose.orientation)
                waypoints.append([x, y, yaw])
            else:
                waypoints.append([x, y])

        self.neupan_planner.waypoints = waypoints
        self.get_logger().info(f'Received {len(waypoints)} waypoints')

        if self.refresh_initial_path and self.robot_state is not None:
            self.neupan_planner.set_initial_path_from_state(self.robot_state)

    def goal_callback(self, goal):
        """目标点回调函数"""
        x = goal.pose.position.x
        y = goal.pose.position.y
        yaw = self.quat_to_yaw(goal.pose.orientation)
        
        self.neupan_planner.waypoints = [[x, y, yaw]]
        self.get_logger().info(f'Received goal: [{x:.2f}, {y:.2f}, {yaw:.2f}]')

    def quat_to_yaw_list(self, quater):
        """四元数转偏航角（列表形式）"""
        _, _, yaw = self.euler_from_quaternion(quater)
        return yaw

    def generate_path_msg(self, path_list):
        """生成路径消息"""
        if path_list is None:
            return Path()

        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in path_list:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = float(point[0])
            pose_stamped.pose.position.y = float(point[1])
            
            if len(point) > 2:
                q = self.yaw_to_quat(float(point[2]))
                pose_stamped.pose.orientation = q

            path_msg.poses.append(pose_stamped)

        return path_msg

    def generate_twist_msg(self, vel):
        """生成Twist消息"""
        if vel is None:
            return Twist()

        cmd_vel = Twist()
        
        if self.neupan_planner.robot_type == 'diff':
            cmd_vel.linear.x = float(vel[0])
            cmd_vel.angular.z = float(vel[1])
        elif self.neupan_planner.robot_type == 'acker':
            # 对于Ackermann机器人，将转向角转换为角速度
            wheelbase = getattr(self.neupan_planner.robot, 'wheelbase', 1.0)
            if wheelbase > 0:
                cmd_vel.linear.x = float(vel[0])
                cmd_vel.angular.z = float(vel[0] * np.tan(vel[1]) / wheelbase)
            else:
                cmd_vel.linear.x = float(vel[0])
                cmd_vel.angular.z = float(vel[1])

        return cmd_vel

    def generate_dune_points_markers_msg(self):
        """生成DUNE点标记消息"""
        markers = MarkerArray()
        
        if (hasattr(self.neupan_planner, 'dune_points') and 
            self.neupan_planner.dune_points is not None):
            
            points_array = self.neupan_planner.dune_points.detach().cpu().numpy()
            
            for i, point in enumerate(points_array):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "dune_points"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(point[0])
                marker.pose.position.y = float(point[1])
                marker.pose.position.z = 0.0
                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = self.marker_size
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                markers.markers.append(marker)

        return markers

    def generate_nrmp_points_markers_msg(self):
        """生成NRMP点标记消息"""
        markers = MarkerArray()
        
        if (hasattr(self.neupan_planner, 'nrmp_points') and 
            self.neupan_planner.nrmp_points is not None):
            
            points_array = self.neupan_planner.nrmp_points.detach().cpu().numpy()
            
            for i, point in enumerate(points_array):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "nrmp_points"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(point[0])
                marker.pose.position.y = float(point[1])
                marker.pose.position.z = 0.0
                marker.scale.x = self.marker_size * 1.5
                marker.scale.y = self.marker_size * 1.5
                marker.scale.z = self.marker_size * 1.5
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 0.8
                markers.markers.append(marker)

        return markers

    def generate_robot_marker_msg(self):
        """生成机器人标记消息"""
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = self.marker_z
        marker.pose.orientation.w = 1.0
        
        if hasattr(self.neupan_planner, 'robot'):
            robot = self.neupan_planner.robot
            if hasattr(robot, 'length') and hasattr(robot, 'width'):
                marker.scale.x = float(robot.length)
                marker.scale.y = float(robot.width)
            else:
                marker.scale.x = 0.5
                marker.scale.y = 0.3
        else:
            marker.scale.x = 0.5
            marker.scale.y = 0.3
            
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        return marker

    @staticmethod
    def yaw_to_quat(yaw):
        """偏航角转四元数"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = sin(yaw / 2.0)
        q.w = cos(yaw / 2.0)
        return q

    @staticmethod
    def quat_to_yaw(quater):
        """四元数转偏航角"""
        return atan2(2.0 * (quater.w * quater.z + quater.x * quater.y),
                     1.0 - 2.0 * (quater.y * quater.y + quater.z * quater.z))

    @staticmethod
    def euler_from_quaternion(quaternion):
        """四元数转欧拉角"""
        x, y, z, w = quaternion
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = atan2(t0, t1)
        
        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)
        
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(t3, t4)
        
        return roll, pitch, yaw