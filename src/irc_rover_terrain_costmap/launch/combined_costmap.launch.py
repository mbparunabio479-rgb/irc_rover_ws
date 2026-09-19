from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('irc_rover_terrain_costmap')
    config = os.path.join(pkg, 'config', 'combined_costmap.yaml')

    return LaunchDescription([
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='combined_costmap',
            output='screen',
            parameters=[config],
            remappings=[
                ('costmap', '/combined_costmap'),
                ('costmap_raw', '/combined_costmap_raw'),
                ('costmap_updates', '/combined_costmap_updates'),
                ('costmap_raw_updates', '/combined_costmap_raw_updates'),
            ],
        )
    ])
