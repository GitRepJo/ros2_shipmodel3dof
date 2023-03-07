# Copyright (c) 2022 Jonas Mahler

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    package_share_directory = get_package_share_directory('shipmodel3dof')
    
    return LaunchDescription([
        Node(
            package='shipmodel3dof',
            executable='shipmodel3dof',
            name='shipmodel3dof',
            parameters= [
            {
            'config_file': "/home/jo/dev_ws/src/shipmodel3dof/config/config.yaml"
            }
            ]
        )
    ])
