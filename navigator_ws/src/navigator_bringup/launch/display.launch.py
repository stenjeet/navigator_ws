import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Static display of the Navigator-M model and polygon (report section 2.4).
    Pass use_rviz:=false for a headless run."""
    description_share = get_package_share_directory('navigator_description')

    urdf_path = os.path.join(description_share, 'urdf', 'navigator_m.urdf')
    rviz_path = os.path.join(description_share, 'config', 'navigator.rviz')

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 (set false for headless runs).'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '1.5', '--y', '1.5', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'base_footprint',
            ],
            output='screen'
        ),
        Node(
            package='navigator_sim',
            executable='polygon_markers',
            name='polygon_markers',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_path],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])
