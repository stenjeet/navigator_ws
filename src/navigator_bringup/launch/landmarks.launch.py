import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Simulation plus landmark extraction and tracking (stage 7).

    The simulation runs with the stage-5 EKF localization node, so the landmark
    node gets a pose on ``/estimate/odom`` and can build the landmark map next
    to the raw observations. Pass ``use_rviz:=false`` for a headless run.
    """
    bringup_share = get_package_share_directory('navigator_bringup')
    slam_share = get_package_share_directory('navigator_slam')

    simulation = os.path.join(
        bringup_share, 'launch', 'simulation.launch.py'
    )
    landmarks_yaml = os.path.join(slam_share, 'config', 'landmarks.yaml')

    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 (set false for headless runs).'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation),
            launch_arguments={
                'use_rviz': use_rviz,
                'use_localization': 'true',
            }.items(),
        ),
        Node(
            package='navigator_slam',
            executable='landmark_node',
            name='landmark_node',
            parameters=[landmarks_yaml],
            output='screen',
        ),
    ])
