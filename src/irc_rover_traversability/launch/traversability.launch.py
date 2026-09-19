from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_dir = get_package_share_directory(
        'irc_rover_traversability'
    )

    config_file = os.path.join(
        package_dir,
        'config',
        'traversability.yaml'
    )

    return LaunchDescription([

        Node(
            package='irc_rover_traversability',
            executable='traversability',
            name='traversability',
            output='screen',
            parameters=[config_file],
        )

    ])
