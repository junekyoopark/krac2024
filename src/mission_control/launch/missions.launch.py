#!/usr/bin/env python

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('mission_control')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    return LaunchDescription([
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        Node(
            package='mission_control',
            namespace='mission_control',
            executable='visualizer',
            name='visualizer'
        ),
        Node(
            package='mission_control',
            namespace='mission_control',
            executable='offboard_start',
            name='offboard_start',
            prefix='gnome-terminal --'
        ),
        Node(
            package='mission_control',
            namespace='mission_control',
            executable='mission1',
            name='mission1',
            prefix='gnome-terminal --',
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        )
        # Node(
        #     package='px4_offboard',
        #     namespace='px4_offboard',
        #     executable='fixed_frame_tf2',
        #     name='fixed_frame_tf2'
        # )
    ])