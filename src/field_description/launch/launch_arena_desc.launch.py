from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('field_description')
    param_file = os.path.join(pkg_share, 'config', 'arenas.yaml')

    return LaunchDescription([
        Node(
            package='field_description',
            executable='field_description_publisher',
            name='field_description_publisher',
            output='screen',
            parameters=[ param_file ],
        )
    ])
