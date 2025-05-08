from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'field_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/field_description/launch', glob(os.path.join('launch', '*.launch.py'))),
        # (if you still have a config dir)
        ('share/field_description/config', glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='astro-dev',
    maintainer_email='ckiw23@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'field_description_publisher = field_description.field_description_publisher:main'
        ],
    },
)
