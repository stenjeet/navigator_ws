import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Simulation plus the stage-5 EKF localization node.

    Equivalent to ``simulation.launch.py use_localization:=true`` but kept as a
    dedicated entry point for the localization stage.
    """
    bringup_share = get_package_share_directory('navigator_bringup')
    simulation = os.path.join(bringup_share, 'launch', 'simulation.launch.py')

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
    ])
