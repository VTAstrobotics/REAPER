from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'camera_streamer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob("config/*"))
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cameron',
    maintainer_email='cameronw23@vt.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usbCamStreamer = camera_streamer.usb_camera_streamer:main',
            'usbCamStreamerParam = camera_streamer.usb_camera_streamer_param:main',
            'streamFuser = camera_streamer.stream_fuser:main'
        ],
    },
)
