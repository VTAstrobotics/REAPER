import launch
import launch_ros.actions

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('camera_streamer'),
        'config',
        'cameras.yaml'
    )
    return launch.LaunchDescription([
        launch_ros.actions.Node(
        package='camera_streamer',
        executable='usbCamStreamerParam',
        namespace='camera',
        name='camera_streamer'#,
        # parameters=[config]
        )
    ])
    