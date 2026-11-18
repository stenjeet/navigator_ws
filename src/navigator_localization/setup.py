import os
from glob import glob

from setuptools import setup

package_name = 'navigator_localization'

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
    description='EKF localization of the Navigator-M robot from UWB ranges.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ekf_node = navigator_localization.ekf_node:main',
        ],
    },
)
