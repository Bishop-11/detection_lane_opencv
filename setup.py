import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'detection_lane_opencv'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bishop',
    maintainer_email='bishop.prakash01@gmail.com',
    description='Classical color-segmentation lane detection: HSV-thresholds the white road-boundary lines from a camera feed and reprojects them onto the ground plane to estimate in-lane position, heading, and curvature.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_segmentation_node = detection_lane_opencv.color_segmentation_node:main',
            'lane_geometry_node = detection_lane_opencv.lane_geometry_node:main',
        ],
    },
)
