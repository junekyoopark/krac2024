from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    static_tf2_odom_to_base = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '1', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'odom', '--child-frame-id', 'base_link'],
            parameters=[
                {'use_sim_time': False}
                ],
        )
    
    static_tf2_base_to_lidar = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '1', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'],
            parameters=[
                {'use_sim_time': False}
                ],
        )

    tf2_map_to_odom = Node(
        package='dontscrewthepooch',
        executable='tf2_map_to_odom'
    )

    ld.add_action(static_tf2_odom_to_base)
    ld.add_action(static_tf2_base_to_lidar)
    ld.add_action(tf2_map_to_odom)

    return ld