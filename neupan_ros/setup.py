from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'neupan_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # 安装配置文件
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
        # 安装示例文件
        (os.path.join('share', package_name, 'example'), 
         glob('example/**/*', recursive=True)),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'geometry_msgs', 
        'nav_msgs',
        'sensor_msgs',
        'visualization_msgs',
        'tf2_ros',
        'tf2_geometry_msgs',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Ruihua Han',
    maintainer_email='hanrh@connect.hku.hk',
    description='NeuPAN ROS2 wrapper - Real-time neural path planner for autonomous robots',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'neupan_node_ros2 = neupan_ros.neupan_node_ros2:main',
        ],
    },
)