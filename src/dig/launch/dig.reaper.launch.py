import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dig'),
        'config',
        "reaper.yaml"
    )
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='dig',
            executable='DigActionServer',
            name='dig_action_server',
            parameters=[config]),
    ])