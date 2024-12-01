#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os   
    
def generate_launch_description():
    
    human_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('human_arm_simulation'),
                    "launch",
                    "human_arm_description.launch.py"
                )
            ]
        ),
    )

    controller_node = Node(
            package = 'human_arm_simulation',
            executable = 'controller_node.py',
            name = 'controller_node')
    
    jointstate_node = Node(
            package = 'human_arm_simulation',
            executable = 'jointstate_node.py',
            name = 'jointstate_node')


    launch_description = LaunchDescription()
    
    launch_description.add_action(human_arm)
    launch_description.add_action(controller_node)
    launch_description.add_action(jointstate_node)
    
    return launch_description