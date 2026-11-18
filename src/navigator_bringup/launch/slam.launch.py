import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Simulation plus landmark extraction and joint EKF-SLAM (stage 8).

    The simulation runs without the stage-5 EKF localizer
    (``use_localization:=false``): the SLAM estimate on ``/slam/odom`` replaces
    it. ``landmark_node`` runs with an empty ``pose_topic`` so it only extracts
    pose-free observations; the map is built by ``ekf_slam_node``.

    Pass ``use_rviz:=false`` for a headless run.
    """
    bringup_share = get_package_share_directory('navigator_bringup')
    slam_share = get_package_share_directory('navigator_slam')

    simulation = os.path.join(bringup_share, 'launch', 'simulation.launch.py')
    landmarks_yaml = os.path.join(slam_share, 'config', 'landmarks.yaml')
    ekf_slam_yaml = os.path.join(slam_share, 'config', 'ekf_slam.yaml')

    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 (set false for headless runs).'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation),
            launch_arguments={
                'use_rviz': use_rviz,
                'use_localization': 'false',
            }.items(),
        ),
        Node(
            package='navigator_slam',
            executable='landmark_node',
            name='landmark_node',
            parameters=[landmarks_yaml, {'pose_topic': ''}],
            output='screen',
        ),
        Node(
            package='navigator_slam',
            executable='ekf_slam_node',
            name='ekf_slam_node',
            parameters=[ekf_slam_yaml],
            output='screen',
        ),
    ])
