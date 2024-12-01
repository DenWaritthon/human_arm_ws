#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro    
    
def generate_launch_description():
    
    pkg = get_package_share_directory('human_arm_simulation')
    rviz_path = os.path.join(pkg,'config','display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
    
    path_description = os.path.join(pkg,'robot','visual','7DOF_robot.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    
    parameters = [{'robot_description':robot_desc_xml}]
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )

    launch_description = LaunchDescription()
    
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    
    return launch_description