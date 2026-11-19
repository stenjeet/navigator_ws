import os
from glob import glob

from setuptools import setup

package_name = 'navigator_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Anton Cheparin',
    maintainer_email='anton@example.com',
    description='Occupancy grid, landmarks and EKF-SLAM for the Navigator-M test polygon.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'occupancy_grid_node = navigator_slam.occupancy_grid_node:main',
            'landmark_node = navigator_slam.landmark_node:main',
            'ekf_slam_node = navigator_slam.ekf_slam_node:main',
        ],
    },
)
