from setuptools import setup
import os
from glob import glob

package_name = 'cam_side_view'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aslam',
    maintainer_email='aslam@example.com',
    description='Camera side view package for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_node_left = cam_side_view.cam_node_left:main',
            'cam_node_right = cam_side_view.cam_node_right:main',
            'stitch_node = cam_side_view.stitch_node:main',
            'object_detect = cam_side_view.object_detect:main',
            'cam_node_back = cam_side_view.cam_node_back:main',
            'cam_node_front = cam_side_view.cam_node_front:main',
            'cam_lidar_calib = cam_side_view.cam_lidar_calib:main',
        ],
    },
)

