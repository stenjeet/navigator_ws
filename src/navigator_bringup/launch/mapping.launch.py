import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# pose_source -> (pose topic, map topic, node name)
SOURCES = {
    'ground_truth': ('/ground_truth/odom', '/map/ground_truth', 'occupancy_grid_gt'),
    'estimate': ('/estimate/odom', '/map/estimate', 'occupancy_grid_ekf'),
}


def generate_launch_description():
    """Simulation plus occupancy-grid mapping (stage 6).

    ``pose_source`` selects which poses build the map:

    * ``estimate`` (default) - the EKF estimate from ``/estimate/odom``, so the
      stage-5 localization node is started too;
    * ``ground_truth`` - ``/ground_truth/odom``, the reference map;
    * ``both`` - two mapper instances on the same scans, which is exactly what
      the stage-6 map comparison needs (see ``tools/map_compare.py``).
    """
    bringup_share = get_package_share_directory('navigator_bringup')
    slam_share = get_package_share_directory('navigator_slam')

    simulation = os.path.join(bringup_share, 'launch', 'simulation.launch.py')
    occupancy_yaml = os.path.join(slam_share, 'config', 'occupancy.yaml')

    use_rviz = LaunchConfiguration('use_rviz')
    pose_source = LaunchConfiguration('pose_source')

    def make_nodes(context, *args, **kwargs):
        source = pose_source.perform(context).strip()
        if source not in ('ground_truth', 'estimate', 'both'):
            raise RuntimeError(
                "pose_source must be one of 'ground_truth', 'estimate', 'both', "
                "got '%s'" % source
            )

        selected = ['ground_truth', 'estimate'] if source == 'both' else [source]
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation),
                launch_arguments={
                    'use_rviz': use_rviz,
                    'use_localization': (
                        'true' if 'estimate' in selected else 'false'
                    ),
                }.items(),
            ),
        ]
        for name in selected:
            pose_topic, map_topic, node_name = SOURCES[name]
            actions.append(Node(
                package='navigator_slam',
                executable='occupancy_grid_node',
                name=node_name,
                parameters=[
                    occupancy_yaml,
                    {'pose_topic': pose_topic, 'map_topic': map_topic},
                ],
                output='screen',
            ))
        return actions

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 (set false for headless runs).'),
        DeclareLaunchArgument(
            'pose_source', default_value='estimate',
            description="Pose used to build the map: 'estimate', "
                        "'ground_truth' or 'both'."),
        OpaqueFunction(function=make_nodes),
    ])
