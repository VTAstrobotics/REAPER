import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual slam node configured for ZEDXM."""
    "The zed camera mode name. zed, zed2, zed2i, zedm, zedx or zedxm"

    camera_model = 'zedxm'

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia: : isaac_ros: : visual_slam: : VisualSlamNode',
        parameters=[{'denoise_input_images': False,
                     'rectified_images': True,
                     'enable_debug_mode': False,
                     'debug_dump_path': '/ tmp/cuvslam',
                     'enable_slam_visualization': True,
                     'enable_landmarks_view': True,
                     'enable_observations_view': True,
                     'map_frame': 'map',
                     'odom_frame': 'odom',
                     'base_frame': camera_model + '_camera_center',
                     'input_imu_frame': camera_model + '_imu_link',
                     'enable_imu_fusion': False,
                     'gyro_noise_density': 0.000244,
                     'gyro_random_walk': 0.000019393,
                     'accel_noise_density': 0.001862,
                     'accel_random_walk': 0.003,
                     'calibration_frequency': 200.0,
                     'img_jitter_threshold_ms': 35.00
                     }],
        remappings=[('stereo_camera/left/image', 'zed/zed_node/left/image_rect_color'),
                    ('stereo_camera/left/camera_info',
                    'zed/zed_node/left/camera_info'),
                    ('stereo_camera/right/image',
                    'zed/zed_node/right/image_rect_color'),
                    ('stereo_camera/right/camera_info',
                    'zed/zed_node/right/camera_info'),
                    ('visual_slam/imu', 'zed/zed_node/imu/data')]
    )

    image_format_converter_node_left = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia: : isaac_ros: : image_proc: : ImageFormatConverterNode',
        name='image_format_node_left',
        parameters=[{'encoding_desired': 'rgb8',
                     }],
        remappings=[
            ('image_raw', 'zed/zed_node/left/image_rect_color'),
            ('image', 'zed/zed_node/left/image_rect_color')]
    )

    image_format_converter_node_right = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia: : isaac_ros: : image_proc: : ImageFormatConverterNode',
        name='image_format_node_right',
        parameters=[{'encoding_desired': 'rgb8',
                     }],
        remappings=[
            ('image_raw', 'zed/zed_node/right/image_rect_color'),
            ('image', 'zed/zed_node/right/image_rect_color')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            image_format_converter_node_left,
            image_format_converter_node_right,
            visual_slam_node
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        TimerAction(
            period=5.0,  # Delay in seconds
            actions=[visual_slam_launch_container]
        ),
    ])