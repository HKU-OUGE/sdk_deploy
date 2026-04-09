import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动主控节点
        Node(
            package='m20_sdk_deploy',
            executable='rl_deploy',
            name='rl_deploy_node',
            output='screen'
        ),
        # 启动 C++ Lidar Relay 节点
        Node(
            package='m20_sdk_deploy',
            executable='lidar_relay',
            name='lidar_relay_node',
            output='screen'
        )
    ])