from launch import LaunchDescription
from launch_ros.actions import Node
import os

# For retrieving a file in your package
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # This finds the path to your package's share directory,
    # where you have your 'config' folder.
    package_share_dir = get_package_share_directory('aruco_pose_estimation')
    
    # Build the path to your YAML config file
    ukf_config_file = os.path.join(package_share_dir, 'config', 'ukf_config.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ukf_localization_node',
            name='ukf_filter_node',
            output='screen',
            parameters=[ukf_config_file]
        )
    ])
