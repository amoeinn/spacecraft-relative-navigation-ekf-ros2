from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='space_sim_py',
            executable='truth_sim_node',
            name='truth_sim_node',
            output='screen'
        ),

        Node(
            package='space_sim_py',
            executable='sensor_sim_node',
            name='sensor_sim_node',
            output='screen'
        ),

        Node(
            package='relative_nav_py',
            executable='ekf_node',
            name='ekf_node',
            output='screen'
        ),
    ])
