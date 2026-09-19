from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('irc_rover_obstacle_filter')
    config = os.path.join(pkg, 'config', 'obstacle_filter.yaml')

    return LaunchDescription([
        Node(
            package='irc_rover_obstacle_filter',
            executable='obstacle_filter',
            name='obstacle_filter',
            output='screen',
            parameters=[config],
        )
    ])
