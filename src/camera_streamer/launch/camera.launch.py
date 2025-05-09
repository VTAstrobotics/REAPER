import launch
import launch_ros.actions
from glob import glob
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def get_serial(device_path: str) -> str:
    """
    Given '/dev/v4l/by-id/usb-046d_Brio_101_2441AP7CHQV8-video-index0',
    returns '2441AP7CHQV8'.
    """
    base = os.path.basename(device_path)
    # split on '_' and take the last chunk, e.g.
    # '2441AP7CHQV8-video-index0'
    last = base.split('_')[-1]
    # strip off '-video-index0'
    serial = last.replace('-video-index0', '')
    return serial


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('camera_streamer'),
        'config',
        'cameras.yaml'
    )
    pattern   = os.path.join('/dev/v4l/by-id', 'usb-046d*-video-index0')
    all_cams = glob(pattern)
    available = [c for c in all_cams if os.path.exists(c)]
    nodes = []
    for i, cam_path in enumerate(available):
        # turn '/dev/v4l/by-id/usb-FOOBAR-video-index0'
        # into a valid topic suffix, e.g. 'usb_FOOBAR_video_index0'
        suffix = cam_path.replace('/', '_').lstrip('_')
        topic  = f'usbcam_image_{suffix}'

        nodes.append(
            Node(
                package='camera_streamer',
                executable='usbCamStreamerParam',
                name=f'usbcam_node_{get_serial(cam_path)}',
                parameters=[{'camera_path': cam_path}],
                # you can also set output='screen' if you want logs on stdout
            )
        )

    return LaunchDescription(nodes)
    