import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Full simulation for the Navigator-M polygon: true motion + UWB + lidar +
    map + RViz (report sections 3-4). Pass use_rviz:=false for a headless run;
    sensors_params / ekf_params point the nodes at alternative parameter files
    (used by the stage-9 scenario runner)."""
    description_share = get_package_share_directory('navigator_description')
    sim_share = get_package_share_directory('navigator_sim')
    loc_share = get_package_share_directory('navigator_localization')

    urdf_path = os.path.join(description_share, 'urdf', 'navigator_m.urdf')
    rviz_path = os.path.join(description_share, 'config', 'navigator.rviz')
    motion_yaml = os.path.join(sim_share, 'config', 'motion.yaml')
    sensors_yaml = os.path.join(sim_share, 'config', 'sensors.yaml')
    ekf_yaml = os.path.join(loc_share, 'config', 'ekf.yaml')

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    use_rviz = LaunchConfiguration('use_rviz')
    use_localization = LaunchConfiguration('use_localization')
    sensors_params = LaunchConfiguration('sensors_params')
    ekf_params = LaunchConfiguration('ekf_params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 (set false for headless runs).'),
        DeclareLaunchArgument(
            'use_localization', default_value='false',
            description='Start the EKF localization node (stage 5).'),
        DeclareLaunchArgument(
            'sensors_params', default_value=sensors_yaml,
            description='sensors.yaml used by the UWB and lidar models '
                        '(default: the navigator_sim config).'),
        DeclareLaunchArgument(
            'ekf_params', default_value=ekf_yaml,
            description='ekf.yaml used by the localization node '
                        '(default: the navigator_localization config).'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        # map -> odom is a fixed identity transform.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            output='screen'
        ),
        Node(
            package='navigator_sim',
            executable='motion_node',
            name='motion_node',
            parameters=[motion_yaml],
            output='screen'
        ),
        Node(
            package='navigator_sim',
            executable='polygon_markers',
            name='polygon_markers',
            output='screen'
        ),
        Node(
            package='navigator_sim',
            executable='uwb_model_node',
            name='uwb_model_node',
            parameters=[sensors_params],
            output='screen'
        ),
        Node(
            package='navigator_sim',
            executable='lidar_model_node',
            name='lidar_model_node',
            parameters=[sensors_params],
            output='screen'
        ),
        Node(
            package='navigator_localization',
            executable='ekf_node',
            name='ekf_node',
            parameters=[ekf_params],
            condition=IfCondition(use_localization),
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
