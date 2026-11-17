import os
from glob import glob

from setuptools import setup

package_name = 'navigator_sim'

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
    zip_safe=True,
    maintainer='Anton Cheparin',
    maintainer_email='anton@example.com',
    description='Simulated ground truth motion, UWB and lidar models for the Navigator-M test polygon.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'polygon_markers = navigator_sim.polygon_markers_node:main',
            'motion_node = navigator_sim.motion_node:main',
            'uwb_model_node = navigator_sim.uwb_model_node:main',
            'lidar_model_node = navigator_sim.lidar_model_node:main',
        ],
    },
)
